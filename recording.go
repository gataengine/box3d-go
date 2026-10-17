// Port of src/recording.h (and later src/recording.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- recording.h ----

// recording is b3Recording.
// PORT: forward declaration only (b3World holds a pointer to it). recording.h and recording.c
// land in milestone 5 and give it its fields.
type recording struct{}
