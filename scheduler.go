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
