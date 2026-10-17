// Go-only test: no upstream counterpart (upstream has no test_solver.c). Checks the solver stage
// machinery of solver.c with real goroutines: every block of a stage is claimed by exactly one
// worker through the per-block atomic syncIndex (b3ExecuteStage), the block array is reused across
// iterations with a monotonic sync index, and the orchestrator / spinner protocol of b3SolverTask
// (b3ExecuteMainStage publishing sync bits, workers spinning on atomicSyncBits until the
// UINT_MAX sentinel) drives several stages to completion. The kernel is
// b3IntegratePositionsTask, whose result is the same whichever worker runs a block, so the
// result must match a serial run bit for bit.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"math"
	"sync"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const solverTestBodyCount = 1000

// solverTestContext builds a step context over solverTestBodyCount awake body states, the body
// blocks and one integrate positions stage.
func solverTestContext(workerCount int) (*stepContext, []bodyState) {
	states := make([]bodyState, solverTestBodyCount)
	for i := range states {
		states[i] = identityBodyState
		states[i].linearVelocity = Vec3{float32(i), float32(i%7) - 3.0, 0.5}
		states[i].angularVelocity = Vec3{0.25, float32(i%5) * 0.5, -0.125}
	}

	w := &world{}
	w.solverSets = make([]solverSet, 3)
	w.solverSets[awakeSet].bodyStates = states

	context := &stepContext{}
	context.world = w
	context.states = states
	context.dt = 1.0 / 60.0
	context.inv_dt = 60.0
	context.h = context.dt / 4.0
	context.maxLinearVelocity = 4000.0
	context.workerCount = int32(workerCount)

	// Small blocks so the workers have to steal from each other
	dim := blockDim{size: 8, count: (solverTestBodyCount + 7) / 8}
	blocks := make([]syncBlock, dim.count)
	initBlocks(blocks, dim, solverTestBodyCount, bodyBlock, math.MaxUint8)

	context.stages = make([]solverStage, 1)
	context.stageCount = 1
	initStage(context.stages, 0, stageIntegratePositions, blocks, dim.count, math.MaxUint8)
	return context, states
}

// solverTestSerial runs the same iterations on one worker without any atomics.
func solverTestSerial(iterations int) []bodyState {
	context, states := solverTestContext(1)
	stage := &context.stages[0]
	for it := 0; it < iterations; it++ {
		for i := 0; i < int(stage.blockCount); i++ {
			integratePositionsTask(stage.blocks[i].block, context)
		}
	}
	return states
}

var _ = Describe("SolverTest", func() {
	It("ExecuteStageClaimsEachBlockOnce", func() {
		const iterations = 5
		want := solverTestSerial(iterations)

		for _, workerCount := range []int{1, 2, 3, 4, 8} {
			context, states := solverTestContext(workerCount)
			stage := &context.stages[0]

			for it := 1; it <= iterations; it++ {
				var wg sync.WaitGroup
				for workerIndex := 0; workerIndex < workerCount; workerIndex++ {
					wg.Add(1)
					go func(workerIndex int) {
						defer wg.Done()
						executeStage(stage, context, it-1, it, workerIndex)
					}(workerIndex)
				}
				wg.Wait()

				Expect(atomicLoadInt(&stage.completionCount)).To(Equal(stage.blockCount))
				atomicStoreInt(&stage.completionCount, 0)
				for i := range stage.blocks {
					Expect(atomicLoadInt(&stage.blocks[i].syncIndex)).To(Equal(int32(it)))
				}
			}

			Expect(states).To(Equal(want), "workerCount %d", workerCount)
		}
	})

	It("SolverTaskSpinnersFollowMainStages", func() {
		const iterations = 7
		want := solverTestSerial(iterations)

		for _, workerCount := range []int{2, 4, 8} {
			context, states := solverTestContext(workerCount)

			// Workers 1..n-1 take the spinner path of b3SolverTask.
			var wg sync.WaitGroup
			for workerIndex := 1; workerIndex < workerCount; workerIndex++ {
				wg.Add(1)
				go func(workerIndex int) {
					defer wg.Done()
					solverTask(&workerContext{context: context, workerIndex: workerIndex})
				}(workerIndex)
			}

			// The orchestrator publishes each iteration of stage 0 with a growing sync index.
			for syncIndex := uint32(1); syncIndex <= iterations; syncIndex++ {
				syncBits := (syncIndex << 16) | 0
				executeMainStage(&context.stages[0], context, syncBits)
				Expect(atomicLoadInt(&context.stages[0].completionCount)).To(Equal(int32(0)))
			}

			// Signal workers to finish
			atomicStoreU32(&context.atomicSyncBits, math.MaxUint32)
			wg.Wait()

			Expect(states).To(Equal(want), "workerCount %d", workerCount)
		}
	})

	It("MainClaimIsARace", func() {
		context, _ := solverTestContext(1)
		atomicStoreInt(&context.mainClaimed, 1)

		// A worker 0 that loses the orchestrator race returns at once and touches nothing.
		solverTask(&workerContext{context: context, workerIndex: 0})
		Expect(atomicLoadU32(&context.atomicSyncBits)).To(Equal(uint32(0)))
		Expect(atomicLoadInt(&context.stages[0].blocks[0].syncIndex)).To(Equal(int32(0)))
	})
})
