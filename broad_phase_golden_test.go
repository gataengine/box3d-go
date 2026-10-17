// Go-only test: no upstream counterpart. Runs updateBroadPhasePairs on a hand assembled world
// (dynamic, kinematic and static proxies, static compounds, sensors, filters, groups, custom
// filtering, joints with and without collideConnected) through rounds of proxy moves, marks,
// destroy/recreate, static moves, pair set removals and joint toggles, and compares the moved
// siblings, the cross seeds, the per-worker pair arrival, the created pairs in sorted order, the
// pair set, the rebuilt trees and overlap probes against the pinned upstream C sources
// (efca0c184d40fd6a82016983f1601c4d23965867) compiled with -ffp-contract=off. See
// testdata/broad_phase_golden_gen.c; this file mirrors it line by line.
//
// createContact (contact.c) is a no-op placeholder until milestone 3, so the test performs the
// one effect the broad-phase depends on after each update: the gathered keys are sorted with
// qsort, duplicates skipped, and each key is recorded and added to the pair set, exactly as the
// C generator's b3CreateContact stub does when b3UpdateBroadPhasePairs calls it.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"os"
	"strings"
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const (
	bpGoldenDynamicBodies   = 120
	bpGoldenKinematicBodies = 12
	bpGoldenStaticBodies    = 50
	bpGoldenCompoundBodies  = 2
	bpGoldenBodyCount       = bpGoldenDynamicBodies + bpGoldenKinematicBodies + bpGoldenStaticBodies + bpGoldenCompoundBodies
	bpGoldenMaxShapes       = 2 * bpGoldenBodyCount
	bpGoldenJointCount      = 10
	bpGoldenRoundCount      = 8
	bpGoldenExtent          = float32(8.0)
)

type bpGoldenGen struct {
	seed    uint32
	sb      strings.Builder
	created []uint64
}

func (g *bpGoldenGen) nextRandom() uint32 {
	// xorshift32
	x := g.seed
	x ^= x << 13
	x ^= x >> 17
	x ^= x << 5
	g.seed = x
	return x
}

func (g *bpGoldenGen) randomFloat(lo, hi float32) float32 {
	t := float32(g.nextRandom()>>8) / float32(1<<24)
	return mulAdd(lo, t, hi-lo)
}

func (g *bpGoldenGen) randomInt(count int) int {
	return int(g.nextRandom() % uint32(count))
}

func (g *bpGoldenGen) randomVec(lo, hi float32) Vec3 {
	var v Vec3
	v.X = g.randomFloat(lo, hi)
	v.Y = g.randomFloat(lo, hi)
	v.Z = g.randomFloat(lo, hi)
	return v
}

func (g *bpGoldenGen) randomBox(center Vec3, minHalf, maxHalf float32) AABB {
	half := g.randomVec(minHalf, maxHalf)
	return AABB{Sub(center, half), Add(center, half)}
}

func (g *bpGoldenGen) printf(format string, args ...any) { fmt.Fprintf(&g.sb, format, args...) }

func (g *bpGoldenGen) printHash(data []byte) { g.printf(" %08x", Hash(HashInit, data)) }

func (g *bpGoldenGen) printTree(label string, tree *DynamicTree) {
	g.printf(" %s %d %d %d", label, tree.NodeEnd, tree.ProxyCount, boolInt(tree.DfsOrdered))
	g.printHash(treeNodeBytes(tree.Nodes, int(tree.NodeEnd)))
}

func bpGoldenBodyType(bodyIndex int) BodyType {
	if bodyIndex < bpGoldenDynamicBodies {
		return DynamicBody
	}
	if bodyIndex < bpGoldenDynamicBodies+bpGoldenKinematicBodies {
		return KinematicBody
	}
	return StaticBody
}

func bpGoldenCustomFilter(idA, idB ShapeID, context any) bool {
	return ((idA.Index1*7 + idB.Index1 + int32(idA.Generation)) % 3) != 0
}

func uint64Bytes(values []uint64) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(values))), len(values)*8)
}

func nodePairBytes(pairs []nodePair) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(pairs))), len(pairs)*int(unsafe.Sizeof(nodePair{})))
}

// emulateCreateContacts does what b3UpdateBroadPhasePairs does after its pair search when
// contact.c exists: gather the worker keys, sort them, skip duplicates and create one contact per
// key. The C generator's b3CreateContact stub records the key and adds it to the pair set.
func (g *bpGoldenGen) emulateCreateContacts(w *world) {
	var pairKeys []uint64
	for i := 0; i < int(w.workerCount); i++ {
		pairKeys = append(pairKeys, w.taskContexts[i].pairKeys...)
	}
	qsort(len(pairKeys), func(i, j int) bool {
		return pairKeys[i] < pairKeys[j]
	}, func(i, j int) {
		pairKeys[i], pairKeys[j] = pairKeys[j], pairKeys[i]
	})
	for i, key := range pairKeys {
		if i > 0 && key == pairKeys[i-1] {
			continue
		}
		g.created = append(g.created, key)
		addKey(&w.broadPhase.pairSet, key)
	}
}

// run builds the scene and runs the rounds. withArrival prints the per-worker arrival lines,
// which only the serial two worker setup of the generator reproduces.
func (g *bpGoldenGen) run(workerCount int, enqueue EnqueueTaskCallback, finish FinishTaskCallback, withArrival bool) string {
	w := &world{}
	bp := &w.broadPhase

	capacity := Capacity{}
	createBroadPhase(bp, &capacity)
	w.stack = createStack(1 << 20)
	w.workerCount = int32(workerCount)
	w.taskContexts = make([]taskContext, workerCount)
	w.enqueueTaskFcn = enqueue
	w.finishTaskFcn = finish
	w.userTaskContext = nil
	w.customFilterFcn = bpGoldenCustomFilter
	w.customFilterContext = nil
	w.worldId = 0

	// Compounds: spheres only, on static bodies with exactly representable rotations.
	var compounds [bpGoldenCompoundBodies]*CompoundData
	bodySims := make([]bodySim, bpGoldenCompoundBodies)
	compoundRotations := [bpGoldenCompoundBodies]Quat{{Vec3{0.0, 0.6, 0.0}, 0.8}, {Vec3{0.8, 0.0, 0.0}, 0.6}}
	for i := range bpGoldenCompoundBodies {
		var spheres [12]CompoundSphereDef
		for j := range 12 {
			spheres[j] = CompoundSphereDef{}
			spheres[j].Sphere.Center = g.randomVec(-6.0, 6.0)
			spheres[j].Sphere.Radius = g.randomFloat(0.5, 1.5)
		}
		def := CompoundDef{}
		def.Spheres = spheres[:]
		def.SphereCount = 12
		compounds[i] = CreateCompound(&def)

		bodySims[i].transform.P = ToPos(g.randomVec(-10.0, 10.0))
		bodySims[i].transform.Q = compoundRotations[i]
	}
	w.solverSets = []solverSet{{bodySims: bodySims}}
	w.compoundShapeCount = bpGoldenCompoundBodies

	// Bodies
	w.bodies = make([]body, bpGoldenBodyCount)
	for i := range bpGoldenBodyCount {
		body := &w.bodies[i]
		body.id = int32(i)
		body.typ = bpGoldenBodyType(i)
		if body.typ == StaticBody {
			body.setIndex = staticSet
		} else {
			body.setIndex = awakeSet
		}
		if i >= bpGoldenBodyCount-bpGoldenCompoundBodies {
			body.localIndex = int32(i - (bpGoldenBodyCount - bpGoldenCompoundBodies))
		} else {
			body.localIndex = NullIndex
		}
		body.headJointKey = NullIndex
		body.jointCount = 0
		body.headShapeId = NullIndex
	}

	// Joints between dynamic bodies, linked as b3CreateJoint links them
	w.joints = make([]joint, bpGoldenJointCount)
	for j := range bpGoldenJointCount {
		bodyIdA := g.randomInt(bpGoldenDynamicBodies)
		bodyIdB := (bodyIdA + 1 + g.randomInt(bpGoldenDynamicBodies-1)) % bpGoldenDynamicBodies
		joint := &w.joints[j]
		joint.jointId = int32(j)
		joint.collideConnected = g.randomInt(2) == 0

		bodyA := &w.bodies[bodyIdA]
		bodyB := &w.bodies[bodyIdB]

		joint.edges[0].bodyId = int32(bodyIdA)
		joint.edges[0].prevKey = NullIndex
		joint.edges[0].nextKey = bodyA.headJointKey
		keyA := int32(j<<1) | 0
		if bodyA.headJointKey != NullIndex {
			head := &w.joints[bodyA.headJointKey>>1]
			head.edges[bodyA.headJointKey&1].prevKey = keyA
		}
		bodyA.headJointKey = keyA
		bodyA.jointCount += 1

		joint.edges[1].bodyId = int32(bodyIdB)
		joint.edges[1].prevKey = NullIndex
		joint.edges[1].nextKey = bodyB.headJointKey
		keyB := int32(j<<1) | 1
		if bodyB.headJointKey != NullIndex {
			head := &w.joints[bodyB.headJointKey>>1]
			head.edges[bodyB.headJointKey&1].prevKey = keyB
		}
		bodyB.headJointKey = keyB
		bodyB.jointCount += 1
	}

	// Shapes and proxies
	w.shapes = make([]shape, 0, bpGoldenMaxShapes)
	for i := range bpGoldenBodyCount {
		typ := bpGoldenBodyType(i)
		isCompound := i >= bpGoldenBodyCount-bpGoldenCompoundBodies
		count := 1
		if typ == DynamicBody {
			count = 1 + g.randomInt(2)
		}
		for range count {
			shapeId := len(w.shapes)
			shape := arrayEmplace(&w.shapes)
			shape.id = int32(shapeId)
			shape.bodyId = int32(i)
			if isCompound {
				shape.typ = CompoundShape
			} else {
				shape.typ = SphereShape
			}
			shape.filter.CategoryBits = 1 << g.randomInt(3)
			if g.randomInt(4) == 0 {
				shape.filter.MaskBits = 0x5
			} else {
				shape.filter.MaskBits = DefaultMaskBits
			}
			if g.randomInt(8) == 0 {
				shape.filter.GroupIndex = -1
			} else {
				shape.filter.GroupIndex = 0
			}
			if g.randomInt(5) == 0 {
				shape.flags = uint8(enableCustomFiltering)
			} else {
				shape.flags = 0
			}
			if isCompound == false && g.randomInt(15) == 0 {
				shape.sensorIndex = 0
			} else {
				shape.sensorIndex = NullIndex
			}
			shape.generation = uint16(1 + g.randomInt(100))

			var aabb AABB
			force := false
			if isCompound {
				c := i - (bpGoldenBodyCount - bpGoldenCompoundBodies)
				shape.compound = compounds[c]
				xf := bodySims[c].transform
				aabb = ComputeCompoundAABB(compounds[c], Transform{ToVec3(xf.P), xf.Q})
			} else if typ == StaticBody {
				aabb = g.randomBox(g.randomVec(-bpGoldenExtent, bpGoldenExtent), 0.5, 4.0)
				force = g.randomInt(3) == 0
			} else {
				aabb = g.randomBox(g.randomVec(-bpGoldenExtent, bpGoldenExtent), 0.2, 2.0)
			}

			shape.proxyKey = int32(bp.createProxy(typ, aabb, shape.filter.CategoryBits, shapeId, force))
		}
	}
	shapeCount := len(w.shapes)

	g.printf("setup | %d", shapeCount)
	g.printTree("static", &bp.trees[StaticBody])
	g.printTree("kinematic", &bp.trees[KinematicBody])
	g.printTree("dynamic", &bp.trees[DynamicBody])
	g.printf("\n")

	siblings := make([]int32, bpGoldenMaxShapes*4)
	var seeds [2 * crossSeedCount]nodePair

	for round := range bpGoldenRoundCount {
		if round > 0 && round != 4 {
			// Remove every third pair created last round, as destroyed contacts would.
			for i := 0; i < len(g.created); i += 3 {
				removeKey(&bp.pairSet, g.created[i])
			}

			moveCount := 10 + g.randomInt(20)
			for range moveCount {
				shapeId := g.randomInt(shapeCount)
				shape := &w.shapes[shapeId]
				shift := g.randomVec(-2.0, 2.0)
				if shape.proxyKey == NullIndex || shape.typ == CompoundShape {
					continue
				}
				key := int(shape.proxyKey)
				old := bp.trees[proxyType(key)].GetAABB(proxyId(key))
				center := MulSV(0.5, Add(old.LowerBound, old.UpperBound))
				aabb := g.randomBox(Add(center, shift), 0.2, 2.0)
				bp.moveProxy(key, aabb)
			}

			markCount := g.randomInt(5)
			for range markCount {
				shapeId := g.randomInt(shapeCount)
				shape := &w.shapes[shapeId]
				if shape.proxyKey != NullIndex && proxyType(int(shape.proxyKey)) != StaticBody {
					bp.markProxyMovedSerial(int(shape.proxyKey))
				}
			}

			toggleCount := g.randomInt(4)
			for range toggleCount {
				shapeId := g.randomInt(shapeCount)
				shape := &w.shapes[shapeId]
				aabb := g.randomBox(g.randomVec(-bpGoldenExtent, bpGoldenExtent), 0.2, 2.0)
				force := g.randomInt(2) == 0
				if shape.typ == CompoundShape {
					continue
				}
				if shape.proxyKey != NullIndex {
					bp.destroyProxy(int(shape.proxyKey))
					shape.proxyKey = NullIndex
				} else {
					typ := w.bodies[shape.bodyId].typ
					shape.proxyKey = int32(bp.createProxy(typ, aabb, shape.filter.CategoryBits, shapeId, force))
				}
			}

			jointIndex := g.randomInt(bpGoldenJointCount)
			w.joints[jointIndex].collideConnected = !w.joints[jointIndex].collideConnected
		}

		// Diagnostics from the pair helpers before the update (pure reads)
		{
			dynamicTree := &bp.trees[DynamicBody]
			siblingCount := gatherMovedSiblings(dynamicTree, siblings)
			staticSeedCount := gatherCrossSeeds(dynamicTree, &bp.trees[StaticBody], seeds[:])
			kinematicSeedCount := gatherCrossSeeds(dynamicTree, &bp.trees[KinematicBody], seeds[staticSeedCount:])
			g.printf("round %d pre | %d", round, siblingCount)
			g.printHash(treeParentBytes(siblings, siblingCount))
			g.printf(" %d %d", staticSeedCount, kinematicSeedCount)
			g.printHash(nodePairBytes(seeds[:staticSeedCount+kinematicSeedCount]))
			g.printf("\n")
		}

		for i := range workerCount {
			arrayClear(&w.taskContexts[i].pairKeys)
		}
		w.taskCount = 0
		w.activeTaskCount = 0
		g.created = g.created[:0]

		updateBroadPhasePairs(w)

		// Finish the tree task, as b3World_Step does before the next broad-phase update.
		if w.userTreeTask != nil {
			w.finishTaskFcn(w.userTreeTask, w.userTaskContext)
			w.userTreeTask = nil
			w.activeTaskCount -= 1
		}

		g.emulateCreateContacts(w)

		if withArrival {
			g.printf("round %d arrival |", round)
			for i := range workerCount {
				keys := w.taskContexts[i].pairKeys
				g.printf(" %d", len(keys))
				g.printHash(uint64Bytes(keys))
			}
			g.printf("\n")
		}

		g.printf("round %d contacts | %d", round, len(g.created))
		g.printHash(uint64Bytes(g.created))
		for i := 0; i < len(g.created) && i < 6; i++ {
			g.printf(" %016x", g.created[i])
		}
		compoundPairs := 0
		childPairs := 0
		for _, key := range g.created {
			shapeIdA := int((key >> (64 - ShapePower)) & shapeMask)
			shapeIdB := int((key >> (64 - 2*ShapePower)) & shapeMask)
			if w.shapes[shapeIdA].typ == CompoundShape || w.shapes[shapeIdB].typ == CompoundShape {
				compoundPairs += 1
			}
			if key&childMask != 0 {
				childPairs += 1
			}
		}
		g.printf(" | compound %d %d | set %d\n", compoundPairs, childPairs, bp.pairSet.count)

		g.printf("round %d trees |", round)
		g.printTree("static", &bp.trees[StaticBody])
		g.printTree("kinematic", &bp.trees[KinematicBody])
		g.printTree("dynamic", &bp.trees[DynamicBody])
		g.printf("\n")

		g.printf("round %d probe |", round)
		for range 6 {
			shapeIdA := g.randomInt(shapeCount)
			shapeIdB := g.randomInt(shapeCount)
			keyA := int(w.shapes[shapeIdA].proxyKey)
			keyB := int(w.shapes[shapeIdB].proxyKey)
			if keyA == NullIndex || keyB == NullIndex {
				g.printf(" -")
				continue
			}
			g.printf(" %d:%d:%d", boolInt(bp.testOverlap(keyA, keyB)), bp.getShapeIndex(keyA), bp.getShapeIndex(keyB))
		}
		g.printf("\n")

		validateBroadPhase(bp)
		validateNoMoved(bp)
	}

	destroyBroadPhase(bp)
	destroyStack(&w.stack)
	return g.sb.String()
}

func bpGoldenCompare(got string, keep func(line string) bool) {
	want, err := os.ReadFile("testdata/broad_phase_golden.txt")
	Expect(err).NotTo(HaveOccurred())

	var wantLines []string
	for _, line := range strings.Split(string(want), "\n") {
		if keep(line) {
			wantLines = append(wantLines, line)
		}
	}
	gotLines := strings.Split(got, "\n")
	Expect(gotLines).To(HaveLen(len(wantLines)))
	for i := range wantLines {
		Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
	}
}

var _ = Describe("BroadPhaseGoldenTest", func() {
	It("MatchesC", func() {
		// The generator's setup: two workers on the serial default task system.
		g := bpGoldenGen{seed: 0x2545F491}
		got := g.run(2, testSerialEnqueue, testSerialFinish, true)
		bpGoldenCompare(got, func(line string) bool { return true })
	})

	It("MatchesCWithGoroutineWorkers", func() {
		// Real concurrency: blocks are claimed by whichever worker gets there first, so the arrival
		// order differs from run to run, but the sorted pairs, the pair set and the trees must not.
		for range 4 {
			g := bpGoldenGen{seed: 0x2545F491}
			got := g.run(4, testGoroutineEnqueue, testGoroutineFinish, false)
			bpGoldenCompare(got, func(line string) bool { return !strings.Contains(line, " arrival |") })
		}
	})
})
