// Go-only test: no upstream counterpart (parallel_for.c has no unit test upstream). Checks the
// block partition, cooperative claiming with real goroutines, the serial default task system and
// the B3_MAX_TASKS fallback.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"sync"
	"sync/atomic"

	. "github.com/onsi/ginkgo/v2"
)

// testSerialEnqueue is b3DefaultAddTaskFcn: run the task inline and return NULL.
func testSerialEnqueue(task TaskCallback, taskContext any, userContext any, taskName string) any {
	task(taskContext)
	return nil
}

// testSerialFinish is b3DefaultFinishTaskFcn.
func testSerialFinish(userTask any, userContext any) {}

// testGoroutineEnqueue runs each task on its own goroutine; the handle is a WaitGroup.
func testGoroutineEnqueue(task TaskCallback, taskContext any, userContext any, taskName string) any {
	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		defer GinkgoRecover()
		task(taskContext)
	}()
	return &wg
}

func testGoroutineFinish(userTask any, userContext any) {
	userTask.(*sync.WaitGroup).Wait()
}

type parallelForProbe struct {
	visits      []atomic.Int32
	blocks      atomic.Int32
	maxWorker   atomic.Int32
	blockSizes  sync.Map
	workerCount int
}

func parallelForProbeCallback(startIndex, endIndex, workerIndex int, context any) {
	probe := context.(*parallelForProbe)
	ensure(0 <= workerIndex && workerIndex < probe.workerCount)
	ensure(startIndex < endIndex)
	for i := startIndex; i < endIndex; i++ {
		probe.visits[i].Add(1)
	}
	probe.blocks.Add(1)
	for {
		m := probe.maxWorker.Load()
		if int32(workerIndex) <= m || probe.maxWorker.CompareAndSwap(m, int32(workerIndex)) {
			break
		}
	}
	probe.blockSizes.Store(endIndex-startIndex, true)
}

func runParallelForProbe(w *world, itemCount, minRange int) *parallelForProbe {
	probe := &parallelForProbe{visits: make([]atomic.Int32, itemCount), workerCount: int(w.workerCount)}
	probe.maxWorker.Store(-1)
	parallelFor(w, parallelForProbeCallback, itemCount, minRange, probe, "probe")
	for i := range itemCount {
		ensure(probe.visits[i].Load() == 1)
	}
	return probe
}

// expectedBlocks mirrors the partition arithmetic of b3ParallelFor.
func expectedBlocks(workerCount, itemCount, minRange int) (blockSize, blockCount int) {
	maxBlockCount := 32 * workerCount
	if itemCount <= minRange*maxBlockCount {
		blockSize = minRange
	} else {
		blockSize = (itemCount + maxBlockCount - 1) / maxBlockCount
	}
	blockCount = (itemCount + blockSize - 1) / blockSize
	return blockSize, blockCount
}

var _ = Describe("ParallelForTest", func() {
	It("Partition", func() {
		for _, workerCount := range []int{1, 2, 3, 4, 8} {
			for _, itemCount := range []int{0, 1, 7, 64, 65, 1000, 4096, 100000} {
				for _, minRange := range []int{1, 16, 64} {
					w := world{workerCount: int32(workerCount), enqueueTaskFcn: testGoroutineEnqueue, finishTaskFcn: testGoroutineFinish}
					probe := runParallelForProbe(&w, itemCount, minRange)
					if itemCount == 0 {
						ensure(probe.blocks.Load() == 0)
						ensure(w.taskCount == 0)
						continue
					}

					blockSize, blockCount := expectedBlocks(workerCount, itemCount, minRange)
					ensure(int(probe.blocks.Load()) == blockCount)
					ensure(int(w.taskCount) == min(workerCount, blockCount))

					// Every block is full except the last.
					probe.blockSizes.Range(func(key, value any) bool {
						size := key.(int)
						ensure(size == blockSize || size == itemCount-(blockCount-1)*blockSize)
						return true
					})
				}
			}
		}
	})

	It("SerialTaskSystem", func() {
		// The default task system runs each task inline, so the first task drains every block.
		w := world{workerCount: 4, enqueueTaskFcn: testSerialEnqueue, finishTaskFcn: testSerialFinish}
		probe := runParallelForProbe(&w, 1000, 8)
		ensure(probe.maxWorker.Load() == 0)
		ensure(w.taskCount == 4)
	})

	It("MaxTasksFallback", func() {
		// Past B3_MAX_TASKS the tasks run inline on the caller.
		w := world{workerCount: 4, enqueueTaskFcn: testGoroutineEnqueue, finishTaskFcn: testGoroutineFinish}
		w.taskCount = MaxTasks
		probe := runParallelForProbe(&w, 1000, 8)
		ensure(probe.maxWorker.Load() == 0)
		ensure(w.taskCount == MaxTasks)
	})
})
