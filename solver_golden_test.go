// Go-only test: no upstream counterpart. Compares the solver.c block partition
// (b3ComputeBlockCount, b3InitBlocks, GetWorkerStartIndex) and the body kernels
// (b3IntegrateVelocitiesTask with damping, gravity, forces and gyroscopic torque;
// b3IntegratePositionsTask with motion locks and speed clamps) against the pinned upstream C
// sources (efca0c184d40fd6a82016983f1601c4d23965867) compiled with -ffp-contract=off, bit for bit.
// See testdata/solver_golden_gen.c; this file mirrors it line by line.
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

const solverGoldenBodyCount = 150

type solverGoldenGen struct {
	seed uint32
	sb   strings.Builder
}

func (g *solverGoldenGen) nextRandom() uint32 {
	g.seed = 1664525*g.seed + 1013904223
	return g.seed
}

func (g *solverGoldenGen) randomFloat(lo, hi float32) float32 {
	t := float32(g.nextRandom()>>8) / float32(1<<24)
	return mulAdd(lo, t, hi-lo)
}

func (g *solverGoldenGen) randomVec(lo, hi float32) Vec3 {
	var v Vec3
	v.X = g.randomFloat(lo, hi)
	v.Y = g.randomFloat(lo, hi)
	v.Z = g.randomFloat(lo, hi)
	return v
}

func (g *solverGoldenGen) randomQuat() Quat {
	var q Quat
	q.V = g.randomVec(-1.0, 1.0)
	q.S = g.randomFloat(0.1, 1.0)
	return NormalizeQuat(q)
}

// Symmetric, diagonally dominant
func (g *solverGoldenGen) randomInertia(lo, hi float32) Matrix3 {
	d0 := g.randomFloat(lo, hi)
	d1 := g.randomFloat(lo, hi)
	d2 := g.randomFloat(lo, hi)
	o01 := g.randomFloat(-0.2*lo, 0.2*lo)
	o02 := g.randomFloat(-0.2*lo, 0.2*lo)
	o12 := g.randomFloat(-0.2*lo, 0.2*lo)
	var m Matrix3
	m.Cx = Vec3{d0, o01, o02}
	m.Cy = Vec3{o01, d1, o12}
	m.Cz = Vec3{o02, o12, d2}
	return m
}

func (g *solverGoldenGen) printf(format string, args ...any) { fmt.Fprintf(&g.sb, format, args...) }
func (g *solverGoldenGen) pf(f float32)                      { g.printf(" %08x", math.Float32bits(f)) }
func (g *solverGoldenGen) pv(v Vec3)                         { g.pf(v.X); g.pf(v.Y); g.pf(v.Z) }
func (g *solverGoldenGen) pq(q Quat)                         { g.pv(q.V); g.pf(q.S) }

func (g *solverGoldenGen) run() string {
	g.seed = 2468

	// Block partition
	{
		itemCounts := []int{0, 1, 3, 4, 5, 31, 32, 33, 100, 127, 128, 129, 1000, 4095, 70000}
		minSizes := []int{1, 4, 32}
		workerCounts := []int{1, 2, 3, 4, 8, 32}
		for a := range itemCounts {
			for b := range minSizes {
				for c := range workerCounts {
					itemCount := itemCounts[a]
					workerCount := workerCounts[c]
					dim := computeBlockCount(itemCount, minSizes[b], 4*workerCount)
					g.printf("dim %d %d %d | %d %d |", itemCount, minSizes[b], workerCount, dim.size, dim.count)

					var blocks [256]syncBlock
					initBlocks(blocks[:], dim, itemCount, graphContactBlock, uint8(a+b))
					for i := 0; i < dim.count; i++ {
						g.printf(" %d:%d:%d:%d", blocks[i].block.startIndex, blocks[i].block.count, blocks[i].block.blockType,
							blocks[i].block.colorIndex)
					}

					g.printf(" |")
					for w := 0; w < workerCount; w++ {
						g.printf(" %d", getWorkerStartIndex(w, dim.count, workerCount))
					}
					g.printf("\n")
				}
			}
		}
	}

	// Body kernels
	sims := make([]bodySim, solverGoldenBodyCount)
	states := make([]bodyState, solverGoldenBodyCount)
	sets := make([]solverSet, 3)
	sets[awakeSet].bodyStates = states

	var world world
	world.solverSets = sets
	world.gravity = Vec3{0.0, -10.0, 0.0}

	for i := 0; i < solverGoldenBodyCount; i++ {
		sim := &sims[i]
		state := &states[i]
		*sim = bodySim{}
		*state = identityBodyState

		r := g.nextRandom()
		sim.transform.Q = g.randomQuat()
		if (r & 7) == 0 {
			sim.invMass = 0.0
		} else {
			sim.invMass = g.randomFloat(0.1, 2.0)
		}
		sim.gravityScale = g.randomFloat(-1.0, 2.0)
		sim.force = g.randomVec(-20.0, 20.0)
		sim.torque = g.randomVec(-5.0, 5.0)
		if r&8 != 0 {
			sim.linearDamping = g.randomFloat(0.0, 2.0)
		}
		if r&16 != 0 {
			sim.angularDamping = g.randomFloat(0.0, 2.0)
		}
		sim.inertia = g.randomInertia(0.5, 4.0)
		sim.invInertiaWorld = g.randomInertia(0.25, 2.0)
		if r&32 != 0 {
			sim.flags = enableGyroscopicTorque
		}

		state.linearVelocity = g.randomVec(-25.0, 25.0)
		state.angularVelocity = g.randomVec(-60.0, 60.0)
		state.deltaRotation = g.randomQuat()
		state.deltaPosition = g.randomVec(-0.1, 0.1)
		state.flags = (r >> 8) & (allLocks | allowFastRotation)
	}

	var context stepContext
	context.world = &world
	context.states = states
	context.sims = sims
	context.dt = 1.0 / 60.0
	context.inv_dt = 60.0
	context.h = context.dt / 4.0
	context.inv_h = 4.0 * context.inv_dt
	context.maxLinearVelocity = 30.0

	dim := computeBlockCount(solverGoldenBodyCount, 32, 4*2)
	var blocks [16]syncBlock
	initBlocks(blocks[:], dim, solverGoldenBodyCount, bodyBlock, math.MaxUint8)

	for subStep := 0; subStep < 4; subStep++ {
		for k := 0; k < dim.count; k++ {
			integrateVelocitiesTask(blocks[k].block, &context)
		}

		for i := 0; i < solverGoldenBodyCount; i++ {
			g.printf("vel %d %d |", subStep, i)
			g.pv(states[i].linearVelocity)
			g.pv(states[i].angularVelocity)
			g.printf("\n")
		}

		for k := dim.count - 1; k >= 0; k-- {
			integratePositionsTask(blocks[k].block, &context)
		}

		for i := 0; i < solverGoldenBodyCount; i++ {
			g.printf("pos %d %d |", subStep, i)
			g.pv(states[i].linearVelocity)
			g.pv(states[i].angularVelocity)
			g.pv(states[i].deltaPosition)
			g.pq(states[i].deltaRotation)
			g.printf(" %08x\n", states[i].flags)
		}
	}

	return g.sb.String()
}

var _ = Describe("SolverGoldenTest", func() {
	It("MatchesC", func() {
		want, err := os.ReadFile("testdata/solver_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		var g solverGoldenGen
		got := g.run()

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		Expect(gotLines).To(HaveLen(len(wantLines)))
		for i := range wantLines {
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
	})
})
