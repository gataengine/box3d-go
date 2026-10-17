// Go-only test: no upstream counterpart. Runs the contact solver (contact_solver.c and
// contact_solver_wide.inl) on a hand assembled world: awake bodies (dynamic and kinematic, plus a
// static body), convex contacts in two graph colors (wide constraints), mesh contacts with one to
// three manifolds in the same colors (wide mesh constraints with lane order and manifold starts),
// and overflow contacts. It runs prepare, then per sub-step warm start, solve, a simple position
// integration and push, then restitution and store impulses, and compares the body velocities after
// every stage and the stored manifold impulses, normal velocities and hit event bits against the
// pinned upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867) compiled with
// -ffp-contract=off. See testdata/contact_solver_golden_gen.c; this file mirrors it line by line.
//
// The C output is identical for the NEON, SSE2 (x86_64), BOX3D_DISABLE_SIMD (B3_SIMD_NONE) and
// BOX3D_VALIDATE builds, so the Go kernels must match it with both gather/scatter variants
// (simdNone true and false).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"os"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const (
	csGoldenBodyCount     = 24
	csGoldenStaticBodyId  = csGoldenBodyCount
	csGoldenMaxContacts   = 64
	csGoldenMaxManifolds  = 256
	csGoldenColorCount    = 2
	csGoldenOverflowCount = 5
)

var (
	csGoldenConvexCounts = [csGoldenColorCount]int{7, 4}
	csGoldenMeshCounts   = [csGoldenColorCount]int{6, 3}
)

type csGoldenGen struct {
	seed uint32
	sb   strings.Builder

	w             *world
	states        []bodyState
	sims          []bodySim
	bodies        []body
	contacts      []contact
	manifolds     []Manifold
	contactCount  int
	manifoldCount int

	convexIds     [csGoldenColorCount][]int32
	meshSpecs     [csGoldenColorCount][]contactSpec
	overflowSpecs []contactSpec

	wideContactCount int
	meshGroupCount   int
}

func (g *csGoldenGen) nextRandom() uint32 {
	// xorshift32
	x := g.seed
	x ^= x << 13
	x ^= x >> 17
	x ^= x << 5
	g.seed = x
	return x
}

func (g *csGoldenGen) randomFloat(lo, hi float32) float32 {
	t := float32(g.nextRandom()>>8) / float32(1<<24)
	return mulAdd(lo, t, hi-lo)
}

func (g *csGoldenGen) randomInt(count int) int {
	return int(g.nextRandom() % uint32(count))
}

func (g *csGoldenGen) randomVec(lo, hi float32) Vec3 {
	var v Vec3
	v.X = g.randomFloat(lo, hi)
	v.Y = g.randomFloat(lo, hi)
	v.Z = g.randomFloat(lo, hi)
	return v
}

func (g *csGoldenGen) printf(format string, args ...any) { fmt.Fprintf(&g.sb, format, args...) }

func csBits(f float32) uint32 { return math.Float32bits(f) }

func (g *csGoldenGen) createContact(manifoldCount int) int32 {
	id := g.contactCount
	g.contactCount++
	c := &g.contacts[id]
	*c = contact{}
	c.contactId = int32(id)

	r := g.randomInt(6)
	a := g.randomInt(csGoldenBodyCount)
	b := g.randomInt(csGoldenBodyCount - 1)
	if b >= a {
		b += 1
	}

	c.edges[0].bodyId = int32(a)
	c.edges[1].bodyId = int32(b)
	c.encodedBodySimA = int32(a)
	c.encodedBodySimB = int32(b)
	if r == 0 {
		c.edges[0].bodyId = csGoldenStaticBodyId
		c.encodedBodySimA = -2
		c.flags |= contactStaticFlag
	} else if r == 1 {
		c.edges[1].bodyId = csGoldenStaticBodyId
		c.encodedBodySimB = -2
		c.flags |= contactStaticFlag
	}

	if g.randomInt(3) == 0 {
		c.flags |= simEnableHitEvent
	}

	c.friction = g.randomFloat(0.0, 1.0)
	c.restitution = 0.0
	if g.randomInt(3) == 0 {
		c.restitution = g.randomFloat(0.1, 0.9)
	}
	c.rollingResistance = 0.0
	if g.randomInt(3) == 0 {
		c.rollingResistance = g.randomFloat(0.01, 0.3)
	}
	c.tangentVelocity = Vec3Zero
	if g.randomInt(4) == 0 {
		c.tangentVelocity = g.randomVec(-1.0, 1.0)
	}

	c.manifolds = g.manifolds[g.manifoldCount : g.manifoldCount+manifoldCount]
	c.manifoldCount = int32(manifoldCount)
	g.manifoldCount += manifoldCount

	for i := range manifoldCount {
		m := &c.manifolds[i]
		*m = Manifold{}
		n := g.randomVec(-1.0, 1.0)
		m.Normal = Normalize(n)
		m.TwistImpulse = g.randomFloat(-0.1, 0.1)
		m.FrictionImpulse = g.randomVec(-0.2, 0.2)
		m.RollingImpulse = g.randomVec(-0.05, 0.05)
		m.PointCount = int32(1 + g.randomInt(4))
		for j := range int(m.PointCount) {
			mp := &m.Points[j]
			mp.AnchorA = g.randomVec(-0.5, 0.5)
			mp.AnchorB = g.randomVec(-0.5, 0.5)
			mp.Separation = g.randomFloat(-0.03, 0.06)
			mp.NormalImpulse = g.randomFloat(0.0, 0.5)
		}
	}

	return int32(id)
}

func (g *csGoldenGen) createScene(seed uint32, warmStarting, propagate bool) *stepContext {
	g.seed = seed
	g.contactCount = 0
	g.manifoldCount = 0

	g.w = &world{}
	g.states = make([]bodyState, csGoldenBodyCount)
	g.sims = make([]bodySim, csGoldenBodyCount)
	g.bodies = make([]body, csGoldenBodyCount+1)
	g.contacts = make([]contact, csGoldenMaxContacts)
	g.manifolds = make([]Manifold, csGoldenMaxManifolds)

	for i := range csGoldenBodyCount {
		state := &g.states[i]
		state.linearVelocity = g.randomVec(-2.0, 2.0)
		state.angularVelocity = g.randomVec(-3.0, 3.0)
		state.deltaPosition = g.randomVec(-0.05, 0.05)
		var q Quat
		q.V = g.randomVec(-0.05, 0.05)
		q.S = 1.0
		state.deltaRotation = NormalizeQuat(q)
		dynamic := (i % 5) != 4
		if dynamic {
			state.flags = dynamicFlag
		}

		sim := &g.sims[i]
		invMass := g.randomFloat(0.2, 2.0)
		d := g.randomVec(0.5, 3.0)
		o := g.randomVec(-0.2, 0.2)
		if dynamic {
			sim.invMass = invMass
			sim.invInertiaWorld.Cx = Vec3{d.X, o.X, o.Y}
			sim.invInertiaWorld.Cy = Vec3{o.X, d.Y, o.Z}
			sim.invInertiaWorld.Cz = Vec3{o.Y, o.Z, d.Z}
		}

		g.bodies[i].setIndex = awakeSet
		g.bodies[i].localIndex = int32(i)
	}

	g.bodies[csGoldenStaticBodyId].setIndex = staticSet
	g.bodies[csGoldenStaticBodyId].localIndex = 0

	for c := range csGoldenColorCount {
		g.convexIds[c] = make([]int32, csGoldenConvexCounts[c])
		for i := range csGoldenConvexCounts[c] {
			g.convexIds[c][i] = g.createContact(1)
		}
	}

	for c := range csGoldenColorCount {
		g.meshSpecs[c] = make([]contactSpec, csGoldenMeshCounts[c])
		for i := range csGoldenMeshCounts[c] {
			manifoldCount := 1 + g.randomInt(3)
			spec := &g.meshSpecs[c][i]
			spec.contactId = g.createContact(manifoldCount)
			spec.manifoldStart = 0
			spec.manifoldCount = uint16(manifoldCount)
		}
	}

	overflowManifoldCount := 0
	g.overflowSpecs = make([]contactSpec, csGoldenOverflowCount)
	for i := range csGoldenOverflowCount {
		manifoldCount := 1 + g.randomInt(3)
		spec := &g.overflowSpecs[i]
		spec.contactId = g.createContact(manifoldCount)
		spec.manifoldStart = int32(overflowManifoldCount)
		spec.manifoldCount = uint16(manifoldCount)
		overflowManifoldCount += manifoldCount
	}

	// World
	w := g.w
	w.contacts = g.contacts[:g.contactCount]
	w.bodies = g.bodies
	w.solverSets = make([]solverSet, 3)
	w.solverSets[awakeSet].bodyStates = g.states
	w.solverSets[awakeSet].bodySims = g.sims
	w.taskContexts = make([]taskContext, 2)
	for i := range 2 {
		tc := &w.taskContexts[i]
		tc.hitEventBitSet = createBitSet(csGoldenMaxContacts)
		setBitCountAndClear(&tc.hitEventBitSet, csGoldenMaxContacts)
		tc.hasHitEvents = false
	}
	w.enableWarmStarting = warmStarting
	w.enableRestitutionPropagation = propagate
	w.contactSpeed = 3.0
	w.restitutionThreshold = 1.0
	w.hitEventThreshold = 1.0
	w.simdWidth = 4

	colors := &w.constraintGraph.colors

	// Convex contacts: wide constraints
	g.wideContactCount = 0
	for c := range csGoldenColorCount {
		g.wideContactCount += ((csGoldenConvexCounts[c] - 1) >> 2) + 1
	}
	wideConstraints := wideConstraintBuffer{
		constraints: make([]contactConstraintWide, g.wideContactCount),
		manifolds:   make([]contactManifoldWide, g.wideContactCount),
	}

	// Mesh contacts: lane order and manifold starts
	meshLaneOrder := make([]int32, 16)
	meshManifoldStarts := make([]int32, 8)
	g.meshGroupCount = 0
	meshSlotCount := 0
	{
		maxManifoldCount := 3
		var bucketStarts [4]int
		orderBase := 0
		for c := range csGoldenColorCount {
			colorContactCount := csGoldenMeshCounts[c]
			specs := g.meshSpecs[c]
			order := meshLaneOrder[orderBase:]

			for k := 0; k <= maxManifoldCount; k++ {
				bucketStarts[k] = 0
			}

			for j := range colorContactCount {
				bucketStarts[specs[j].manifoldCount] += 1
			}

			start := 0
			for k := maxManifoldCount; k >= 0; k-- {
				count := bucketStarts[k]
				bucketStarts[k] = start
				start += count
			}

			for j := range colorContactCount {
				order[bucketStarts[specs[j].manifoldCount]] = int32(j)
				bucketStarts[specs[j].manifoldCount]++
			}

			colorMeshGroupCount := ((colorContactCount - 1) >> 2) + 1
			for gi := range colorMeshGroupCount {
				slotCount := int(specs[order[gi<<2]].manifoldCount)
				meshManifoldStarts[g.meshGroupCount+gi] = int32(meshSlotCount)
				meshSlotCount += slotCount
			}

			orderBase += colorContactCount
			g.meshGroupCount += colorMeshGroupCount
		}
		meshManifoldStarts[g.meshGroupCount] = int32(meshSlotCount)
	}

	wideMeshConstraints := wideConstraintBuffer{
		constraints: make([]contactConstraintWide, g.meshGroupCount),
		manifolds:   make([]contactManifoldWide, meshSlotCount),
	}

	// Distribute to colors and build spans
	widePrepareSpans := make([]widePrepareSpan, csGoldenColorCount+1)
	meshPrepareSpans := make([]meshPrepareSpan, csGoldenColorCount+1)
	{
		wideBase := 0
		contactBase := 0
		meshGroupBase := 0
		for c := range csGoldenColorCount {
			color := &colors[c]
			color.convexContacts = g.convexIds[c]
			color.contacts = g.meshSpecs[c]

			widePrepareSpans[c].start = int32(wideBase)
			widePrepareSpans[c].count = int32(csGoldenConvexCounts[c])
			widePrepareSpans[c].contacts = g.convexIds[c]
			color.wideConstraints = wideConstraintBuffer{
				constraints: wideConstraints.constraints[wideBase:],
				manifolds:   wideConstraints.manifolds[wideBase:],
			}
			colorContactCountW := ((csGoldenConvexCounts[c] - 1) >> 2) + 1
			color.wideConstraintCount = int32(colorContactCountW)
			wideBase += colorContactCountW

			meshPrepareSpans[c].start = int32(meshGroupBase)
			meshPrepareSpans[c].count = int32(csGoldenMeshCounts[c])
			meshPrepareSpans[c].contacts = g.meshSpecs[c]
			meshPrepareSpans[c].order = meshLaneOrder[contactBase:]
			colorMeshGroupCount := ((csGoldenMeshCounts[c] - 1) >> 2) + 1
			color.wideMeshConstraints = wideConstraintBuffer{
				constraints: wideMeshConstraints.constraints[meshGroupBase:],
				manifolds:   wideMeshConstraints.manifolds[meshManifoldStarts[meshGroupBase]:],
			}
			color.wideMeshManifoldStarts = meshManifoldStarts[meshGroupBase:]
			color.wideMeshConstraintCount = int32(colorMeshGroupCount)
			meshGroupBase += colorMeshGroupCount
			contactBase += csGoldenMeshCounts[c]
		}

		widePrepareSpans[csGoldenColorCount].start = int32(g.wideContactCount)
		meshPrepareSpans[csGoldenColorCount].start = int32(g.meshGroupCount)
	}

	overflow := &colors[overflowIndex]
	overflow.contacts = g.overflowSpecs
	overflow.contactConstraints = make([]contactConstraint, 8)
	overflow.manifoldConstraints = make([]manifoldConstraint, 32)
	overflowSpans := make([]contactPrepareSpan, 2)
	overflowSpans[0].start = 0
	overflowSpans[0].count = csGoldenOverflowCount
	overflowSpans[0].contacts = g.overflowSpecs
	overflowSpans[1].start = csGoldenOverflowCount

	context := &stepContext{}
	context.world = w
	context.graph = &w.constraintGraph
	context.states = g.states
	context.sims = g.sims
	context.h = 1.0 / 120.0
	context.inv_h = 120.0
	context.contactSoftness = softness{biasRate: 30.5, massScale: 0.8, impulseScale: 0.2}
	context.staticSoftness = softness{biasRate: 61.0, massScale: 0.9, impulseScale: 0.1}
	context.wideConstraints = wideConstraints
	context.widePrepareSpans = widePrepareSpans
	context.wideContactCount = int32(g.wideContactCount)
	context.overflowSpans = overflowSpans
	context.wideMeshConstraints = wideMeshConstraints
	context.wideMeshManifoldStarts = meshManifoldStarts
	context.meshPrepareSpans = meshPrepareSpans
	context.activeColorCount = csGoldenColorCount
	return context
}

func csBlock(startIndex, count, blockType, colorIndex int) solverBlock {
	return solverBlock{
		startIndex: int32(startIndex),
		count:      uint16(count),
		blockType:  uint8(blockType),
		colorIndex: uint8(colorIndex),
	}
}

func (g *csGoldenGen) printStates(run int, stage string) {
	for i := range csGoldenBodyCount {
		s := &g.states[i]
		g.printf("run %d %s body %d | %08x %08x %08x %08x %08x %08x\n", run, stage, i, csBits(s.linearVelocity.X),
			csBits(s.linearVelocity.Y), csBits(s.linearVelocity.Z), csBits(s.angularVelocity.X),
			csBits(s.angularVelocity.Y), csBits(s.angularVelocity.Z))
	}
}

// colorStage mirrors ColorStage: color 0 is split in two blocks to exercise a non-zero start index.
func csColorStage(context *stepContext, convexFcn, meshFcn func(solverBlock, *stepContext), overflowFcn func(*stepContext)) {
	for c := range csGoldenColorCount {
		color := &context.graph.colors[c]
		count := int(color.wideConstraintCount)
		if count > 1 {
			convexFcn(csBlock(0, 1, graphWideContactBlock, c), context)
			convexFcn(csBlock(1, count-1, graphWideContactBlock, c), context)
		} else {
			convexFcn(csBlock(0, count, graphWideContactBlock, c), context)
		}

		meshFcn(csBlock(0, int(color.wideMeshConstraintCount), graphContactBlock, c), context)
	}

	overflowFcn(context)
}

func (g *csGoldenGen) integratePositions(context *stepContext) {
	h := context.h
	for i := range csGoldenBodyCount {
		s := &g.states[i]
		if (s.flags & dynamicFlag) == 0 {
			continue
		}

		s.deltaPosition = MulAdd(s.deltaPosition, h, s.linearVelocity)
		q := s.deltaRotation
		q.V = MulAdd(q.V, float32(0.5*h), s.angularVelocity)
		s.deltaRotation = NormalizeQuat(q)
	}
}

func (g *csGoldenGen) run(run int, seed uint32, warmStarting, propagate bool) {
	context := g.createScene(seed, warmStarting, propagate)

	g.printStates(run, "initial")

	// Prepare: flat blocks across colors, the first convex block ends inside color 0
	prepareContactsConvex(csBlock(0, 1, wideContactBlock, 0), context)
	prepareContactsConvex(csBlock(1, g.wideContactCount-1, wideContactBlock, 0), context)
	prepareContactsMeshWide(csBlock(0, 2, contactBlock, 0), context)
	prepareContactsMeshWide(csBlock(2, g.meshGroupCount-2, contactBlock, 0), context)
	prepareContactsOverflow(context)
	g.printf("run %d prepare | anyRestitution %d\n", run, atomicLoadInt(&context.anyRestitution))
	g.printStates(run, "prepare")

	for subStep := range 2 {
		csColorStage(context, warmStartContactsConvex, warmStartContactsMeshWide, warmStartContactsOverflow)
		g.printStates(run, fmt.Sprintf("sub%d-warm", subStep))

		csColorStage(context, solveContactsConvex, solveContactsMeshWide, solveContactsOverflow)
		g.printStates(run, fmt.Sprintf("sub%d-solve", subStep))

		g.integratePositions(context)

		csColorStage(context, pushContactsConvex, pushContactsMeshWide, pushContactsOverflow)
		g.printStates(run, fmt.Sprintf("sub%d-push", subStep))
	}

	csColorStage(context, applyRestitutionConvex, applyRestitutionMeshWide, applyRestitutionOverflow)
	g.printStates(run, "restitution")

	storeImpulsesConvex(csBlock(0, g.wideContactCount, wideContactBlock, 0), context, 0)
	storeImpulsesMeshWide(csBlock(0, 1, contactBlock, 0), context, 1)
	storeImpulsesMeshWide(csBlock(1, g.meshGroupCount-1, contactBlock, 0), context, 0)
	storeImpulsesOverflow(context)

	for i := range g.contactCount {
		c := &g.contacts[i]
		for j := range int(c.manifoldCount) {
			m := &c.manifolds[j]
			g.printf("run %d store contact %d manifold %d | %08x %08x %08x %08x %08x %08x %08x |", run, i, j,
				csBits(m.TwistImpulse), csBits(m.FrictionImpulse.X), csBits(m.FrictionImpulse.Y),
				csBits(m.FrictionImpulse.Z), csBits(m.RollingImpulse.X), csBits(m.RollingImpulse.Y),
				csBits(m.RollingImpulse.Z))
			for k := range int(m.PointCount) {
				mp := &m.Points[k]
				g.printf(" %08x %08x %08x", csBits(mp.NormalImpulse), csBits(mp.TotalNormalImpulse), csBits(mp.NormalVelocity))
			}
			g.printf("\n")
		}
	}

	for wi := range 2 {
		tc := &g.w.taskContexts[wi]
		hasHitEvents := 0
		if tc.hasHitEvents {
			hasHitEvents = 1
		}
		g.printf("run %d hits worker %d | %d |", run, wi, hasHitEvents)
		for i := range g.contactCount {
			if getBit(&tc.hitEventBitSet, uint32(i)) {
				g.printf(" %d", i)
			}
		}
		g.printf("\n")
	}
}

func (g *csGoldenGen) all() string {
	g.run(0, 0x2545F491, true, false)
	g.run(1, 0x9E3779B9, false, true)
	g.run(2, 0x1B873593, true, true)
	return g.sb.String()
}

func csGoldenCompare(got string) {
	want, err := os.ReadFile("testdata/contact_solver_golden.txt")
	Expect(err).NotTo(HaveOccurred())

	wantLines := strings.Split(string(want), "\n")
	gotLines := strings.Split(got, "\n")
	Expect(gotLines).To(HaveLen(len(wantLines)))
	for i := range wantLines {
		Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
	}
}

var _ = Describe("ContactSolverGoldenTest", func() {
	It("MatchesC", func() {
		var g csGoldenGen
		csGoldenCompare(g.all())
	})

	It("MatchesCWithLaneGatherScatter", func() {
		// The B3_SIMD_WIDTH 4 lane variants of b3GatherBodies / b3ScatterBodies (transpose, row
		// stores) must give the same bits as the B3_SIMD_NONE variants.
		saved := simdNone
		simdNone = false
		defer func() { simdNone = saved }()

		var g csGoldenGen
		csGoldenCompare(g.all())
	})
})
