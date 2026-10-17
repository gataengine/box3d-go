// Port of src/parallel_for.h and src/parallel_for.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- parallel_for.h ----

// parallelForCallback is b3ParallelForCallback.
// Callback invoked by b3ParallelFor to process a range of items. May be called
// multiple times per worker: work is divided into blocks that workers claim
// atomically, so a worker that finishes early picks up the next unclaimed
// block instead of sitting idle. workerIndex is the worker identity and is
// stable across all invocations from the same worker, so it is safe to use as
// an index into per-worker state (e.g. world->taskContexts.data + workerIndex).
type parallelForCallback func(startIndex, endIndex, workerIndex int, context any)

// ---- parallel_for.c ----

// parallelForShared is b3ParallelForShared.
// Shared state for one b3ParallelFor invocation. Workers race on nextBlock to
// claim work, so a slow chunk can't strand the other threads.
type parallelForShared struct {
	nextBlock  atomicInt
	blockCount int
	blockSize  int
	itemCount  int
	callback   parallelForCallback
	context    any
}

// parallelForTask is b3ParallelForTask.
type parallelForTask struct {
	shared      *parallelForShared
	workerIndex int
}

// parallelForTrampoline is b3ParallelForTrampoline.
func parallelForTrampoline(taskContext any) {
	task := taskContext.(*parallelForTask)
	shared := task.shared
	workerIndex := task.workerIndex
	context := shared.context
	callback := shared.callback

	blockCount := shared.blockCount
	blockSize := shared.blockSize
	itemCount := shared.itemCount

	for {
		blockIndex := int(atomicFetchAddInt(&shared.nextBlock, 1))
		if blockIndex >= blockCount {
			break
		}

		start := blockIndex * blockSize
		end := start + blockSize
		if end > itemCount {
			end = itemCount
		}

		callback(start, end, workerIndex, context)
	}
}

// parallelFor is b3ParallelFor.
// Divide [0, itemCount) into blocks and process them with cooperative claiming:
// up to world->workerCount tasks are enqueued, and each task loops, atomically
// claiming the next unclaimed block until the range is drained. Blocks the
// caller until all work is complete. minRange is the minimum block size; block
// size grows once itemCount exceeds 4 * workerCount * minRange so block count
// stays bounded.
func parallelFor(world *world, callback parallelForCallback, itemCount int, minRange int, context any, name string) {
	if itemCount <= 0 {
		return
	}

	assert(minRange > 0, "minRange > 0")

	workerCount := int(world.workerCount)
	assert(0 < workerCount && workerCount <= MaxWorkers, "0 < workerCount && workerCount <= B3_MAX_WORKERS")

	// Target multiple blocks per worker to reduce thread stalls.
	// block size grows once items exceed maxBlockCount * minRange
	// so the block count stays bounded and per-block sync overhead stays low.
	// Benchmarking shows 32 is optimal for the convex pile benchmark and others.
	// This is a lot different than Box2D because the 3D narrow phase cost
	// varies a lot.
	blocksPerWorker := 32
	maxBlockCount := blocksPerWorker * workerCount

	var blockSize int
	var blockCount int
	if itemCount <= minRange*maxBlockCount {
		blockSize = minRange
		blockCount = (itemCount + blockSize - 1) / blockSize
	} else {
		blockSize = (itemCount + maxBlockCount - 1) / maxBlockCount
		blockCount = (itemCount + blockSize - 1) / blockSize
	}
	assert(blockCount >= 1, "blockCount >= 1")
	assert(blockSize*blockCount >= itemCount, "blockSize * blockCount >= itemCount")

	// No point enqueueing more tasks than blocks.
	taskCount := blockCount
	if workerCount < blockCount {
		taskCount = workerCount
	}

	// PORT: shared and tasks live on the C stack; the task system receives pointers to them, so
	// as Go locals they would escape to the heap on every call. They live in the world
	// (stepScratch) instead: b3ParallelFor is only called from the stepping thread and is not
	// nested, and it waits for its tasks before returning.
	shared := &world.stepScratch.parallelForShared
	shared.blockCount = blockCount
	shared.blockSize = blockSize
	shared.itemCount = itemCount
	shared.callback = callback
	shared.context = context
	atomicStoreInt(&shared.nextBlock, 0)

	tasks := &world.stepScratch.parallelForTasks
	var handles [MaxWorkers]any
	for i := 0; i < taskCount; i++ {
		tasks[i].shared = shared
		tasks[i].workerIndex = i

		if world.taskCount < MaxTasks {
			handles[i] = world.enqueueTaskFcn(parallelForTrampoline, &tasks[i], world.userTaskContext, name)
			world.taskCount += 1
		} else {
			handles[i] = nil
			parallelForTrampoline(&tasks[i])
		}
	}

	for i := 0; i < taskCount; i++ {
		if handles[i] != nil {
			world.finishTaskFcn(handles[i], world.userTaskContext)
		}
	}
}
