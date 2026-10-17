// Port of src/solver.h (and later src/solver.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- solver.h ----

// PORT: header types only, and only b3Softness so far (the joint and contact constraint structs
// that b3World reaches need it). The rest of solver.h and solver.c land in milestone 3.

// softness is b3Softness: constraint softness
type softness struct {
	biasRate     float32
	massScale    float32
	impulseScale float32
}
