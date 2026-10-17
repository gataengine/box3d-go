// Port of src/scheduler.h and src/scheduler.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "fmt"

// ---- scheduler.h ----

// PORT: b3TaskCallback is TaskCallback in types.go (same signature as the public callback).

// ---- scheduler.c ----

// schedulerTaskStatus is enum b3SchedulerTaskStatus.
const (
	schedulerFree     = 0
	schedulerPending  = 1
	schedulerClaimed  = 2
	schedulerComplete = 3
)

// schedulerTask is b3SchedulerTask.
type schedulerTask struct {
	callback    TaskCallback
	taskContext any
	status      atomicInt
}

// schedulerWorkerContext is b3SchedulerWorkerContext.
type schedulerWorkerContext struct {
	scheduler   *scheduler
	threadIndex int
}

// scheduler is b3Scheduler.
// PORT: threads are goroutines (timer.go createThread); the semaphore is the sync.Cond based
// semaphore of timer.go.
type scheduler struct {
	threads        [MaxWorkers]*thread
	workerContexts [MaxWorkers]schedulerWorkerContext

	// total workers including main thread
	workerCount int

	// threads created = workerCount - 1
	threadCount int

	tasks    [MaxTasks]schedulerTask
	nextSlot atomicInt

	taskSemaphore *semaphore
	shutdown      atomicInt
}

// schedulerExecuteOne is b3SchedulerExecuteOne.
// Try to claim and execute one pending task.
// Returns true if work was performed, false otherwise.
func schedulerExecuteOne(scheduler *scheduler) bool {
	taskCount := int(atomicLoadInt(&scheduler.nextSlot))
	for t := 0; t < taskCount; t++ {
		task := &scheduler.tasks[t]
		if atomicLoadInt(&task.status) != schedulerPending {
			continue
		}

		if atomicCompareExchangeInt(&task.status, schedulerPending, schedulerClaimed) == false {
			continue
		}

		task.callback(task.taskContext)

		atomicStoreInt(&task.status, schedulerComplete)
		return true
	}

	return false
}

// schedulerWorkerMain is b3SchedulerWorkerMain.
// Background worker thread entry point.
func schedulerWorkerMain(context any) {
	workerContext := context.(*schedulerWorkerContext)
	scheduler := workerContext.scheduler

	for {
		waitSemaphore(scheduler.taskSemaphore)

		if atomicLoadInt(&scheduler.shutdown) != 0 {
			break
		}

		// Claim and execute all available work
		for schedulerExecuteOne(scheduler) {
		}
	}
}

// createScheduler is b3CreateScheduler.
func createScheduler(workerCount int) *scheduler {
	assert(0 < workerCount && workerCount <= MaxWorkers, "0 < workerCount && workerCount <= B3_MAX_WORKERS")

	scheduler := &scheduler{}

	scheduler.workerCount = workerCount
	threadCount := workerCount - 1
	scheduler.threadCount = threadCount
	scheduler.taskSemaphore = createSemaphore(0)
	atomicStoreInt(&scheduler.shutdown, 0)
	atomicStoreInt(&scheduler.nextSlot, 0)

	// Background threads use indices 1..workerCount-1.
	// Main thread uses index 0.
	for i := 0; i < threadCount; i++ {
		scheduler.workerContexts[i].scheduler = scheduler
		scheduler.workerContexts[i].threadIndex = i + 1

		name := fmt.Sprintf("box3d_worker_%02d", i+1)
		scheduler.threads[i] = createThread(schedulerWorkerMain, &scheduler.workerContexts[i], name)
	}

	return scheduler
}

// destroyScheduler is b3DestroyScheduler.
func destroyScheduler(scheduler *scheduler) {
	atomicStoreInt(&scheduler.shutdown, 1)

	// Wake all background threads so they see the shutdown flag
	for i := 0; i < scheduler.threadCount; i++ {
		signalSemaphore(scheduler.taskSemaphore)
	}

	for i := 0; i < scheduler.threadCount; i++ {
		joinThread(scheduler.threads[i])
		scheduler.threads[i] = nil
	}

	destroySemaphore(scheduler.taskSemaphore)
	// PORT: b3Free dropped; the GC owns the scheduler.
}

// resetScheduler is b3ResetScheduler.
func resetScheduler(scheduler *scheduler) {
	atomicStoreInt(&scheduler.nextSlot, 0)
}

// schedulerEnqueueTask is b3SchedulerEnqueueTask.
// The returned handle is a *schedulerTask.
func schedulerEnqueueTask(task TaskCallback, taskContext any, userContext any, name string) any {
	scheduler := userContext.(*scheduler)

	// PORT: b3AtomicFetchAddInt returns the previous value, as atomicFetchAddInt does.
	slot := int(atomicFetchAddInt(&scheduler.nextSlot, 1))
	assert(slot < MaxTasks, "slot < B3_MAX_TASKS")

	schedulerTask := &scheduler.tasks[slot]
	schedulerTask.callback = task
	schedulerTask.taskContext = taskContext

	// Memory fence: status must be published after callback and context are written
	atomicStoreInt(&schedulerTask.status, schedulerPending)

	// One wake per enqueue is enough: at most one worker picks up each task.
	signalSemaphore(scheduler.taskSemaphore)

	return schedulerTask
}

// schedulerFinishTask is b3SchedulerFinishTask.
func schedulerFinishTask(userTask any, userContext any) {
	// PORT: a nil interface and a nil *schedulerTask both count as NULL.
	waitTask, _ := userTask.(*schedulerTask)
	if waitTask == nil {
		return
	}

	scheduler := userContext.(*scheduler)

	// Main thread helps execute any available work while waiting for the
	// target task to complete. This keeps the main thread from idling when
	// background threads are busy on other tasks from the same phase.
	for atomicLoadInt(&waitTask.status) != schedulerComplete {
		if schedulerExecuteOne(scheduler) == false {
			Yield()
		}
	}
}
