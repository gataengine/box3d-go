// Go-only test: no upstream counterpart (scheduler.c has no unit test upstream). Checks the
// enqueue/finish contract of the goroutine scheduler, the main thread helping while it waits,
// reset, shutdown, and parallelFor driven by the scheduler at several worker counts.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"sync/atomic"

	. "github.com/onsi/ginkgo/v2"
)

type schedulerProbe struct {
	runs atomic.Int32
	hits [64]atomic.Int32
}

type schedulerProbeTask struct {
	probe *schedulerProbe
	index int
}

func schedulerProbeCallback(taskContext any) {
	task := taskContext.(*schedulerProbeTask)
	task.probe.hits[task.index].Add(1)
	task.probe.runs.Add(1)
}

var _ = Describe("SchedulerTest", func() {
	It("EnqueueFinish", func() {
		for _, workerCount := range []int{1, 2, 4, 8} {
			s := createScheduler(workerCount)
			ensure(s.workerCount == workerCount)
			ensure(s.threadCount == workerCount-1)

			// Several phases, each reset like a world step.
			for phase := 0; phase < 20; phase++ {
				resetScheduler(s)

				var probe schedulerProbe
				var tasks [64]schedulerProbeTask
				var handles [64]any
				for i := range tasks {
					tasks[i] = schedulerProbeTask{probe: &probe, index: i}
					handles[i] = schedulerEnqueueTask(schedulerProbeCallback, &tasks[i], s, "probe")
					ensure(handles[i] != nil)
				}

				for i := range handles {
					schedulerFinishTask(handles[i], s)
					// Finish blocks until that task completed.
					ensure(probe.hits[i].Load() == 1)
					ensure(atomicLoadInt(&handles[i].(*schedulerTask).status) == schedulerComplete)
				}

				ensure(probe.runs.Load() == 64)
				for i := range probe.hits {
					ensure(probe.hits[i].Load() == 1)
				}
				ensure(int(atomicLoadInt(&s.nextSlot)) == 64)
			}

			destroyScheduler(s)
			for i := 0; i < s.threadCount; i++ {
				ensure(s.threads[i] == nil)
			}
		}
	})

	It("MainThreadHelps", func() {
		// With a single worker there are no background threads: the caller of finish runs
		// every pending task itself.
		s := createScheduler(1)
		var probe schedulerProbe
		task := schedulerProbeTask{probe: &probe, index: 0}
		handle := schedulerEnqueueTask(schedulerProbeCallback, &task, s, "probe")
		ensure(probe.runs.Load() == 0)
		schedulerFinishTask(handle, s)
		ensure(probe.runs.Load() == 1)

		// A nil handle is a no-op.
		schedulerFinishTask(nil, s)
		destroyScheduler(s)
	})

	It("ParallelFor", func() {
		for _, workerCount := range []int{1, 2, 4} {
			s := createScheduler(workerCount)
			for _, itemCount := range []int{1, 100, 5000} {
				resetScheduler(s)
				w := world{workerCount: int32(workerCount), enqueueTaskFcn: schedulerEnqueueTask, finishTaskFcn: schedulerFinishTask, userTaskContext: s}
				probe := runParallelForProbe(&w, itemCount, 4)
				ensure(int(probe.maxWorker.Load()) < workerCount)
				ensure(int(atomicLoadInt(&s.nextSlot)) == int(w.taskCount))
			}
			destroyScheduler(s)
		}
	})
})
