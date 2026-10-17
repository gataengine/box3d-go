// Port of src/scheduler.h (and later src/scheduler.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- scheduler.h ----

// PORT: b3TaskCallback is TaskCallback in types.go (same signature as the public callback).

// scheduler is b3Scheduler.
// PORT: forward declaration only (b3World holds a pointer to it). The struct is defined in
// scheduler.c and gets its fields when scheduler.c lands in milestone 3.
type scheduler struct{}

// ---- scheduler.c ----

// PORT: stubs until scheduler.c lands in milestone 3. Other milestone 3 files (physics_world.c)
// call these; the owner of scheduler.c replaces each stub with the real function, keeping the
// signature, in upstream order.

// createScheduler is b3CreateScheduler.
func createScheduler(workerCount int) *scheduler {
	panic("box3d: b3CreateScheduler not ported yet (scheduler.c, milestone 3)")
}

// destroyScheduler is b3DestroyScheduler.
func destroyScheduler(scheduler *scheduler) {
	panic("box3d: b3DestroyScheduler not ported yet (scheduler.c, milestone 3)")
}

// resetScheduler is b3ResetScheduler.
func resetScheduler(scheduler *scheduler) {
	panic("box3d: b3ResetScheduler not ported yet (scheduler.c, milestone 3)")
}

// schedulerEnqueueTask is b3SchedulerEnqueueTask.
func schedulerEnqueueTask(task TaskCallback, taskContext any, userContext any, name string) any {
	panic("box3d: b3SchedulerEnqueueTask not ported yet (scheduler.c, milestone 3)")
}

// schedulerFinishTask is b3SchedulerFinishTask.
func schedulerFinishTask(userTask any, userContext any) {
	panic("box3d: b3SchedulerFinishTask not ported yet (scheduler.c, milestone 3)")
}
