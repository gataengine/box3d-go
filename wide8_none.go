// Go-only: the width 8 types of Box3D (https://github.com/erincatto/box3d) at commit
// efca0c184d40fd6a82016983f1601c4d23965867 in builds without the width 8 backend.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build !(box3d_simd && goexperiment.simd && amd64 && !box3d_no8)

package box3d

// PORT: upstream builds without B3_SIMD_HAS_WIDTH_8 (or with BOX3D_DISABLE_AVX2, here the
// box3d_no8 tag) have no W8 types. The solver, the step context and the graph colors name the
// width 8 buffers next to the width 4 ones, so this build supplies empty stand-ins; wide8 is nil
// and the width 8 branches never run.

type contactConstraintWide8 struct{}

type contactManifoldWide8 struct{}

type wideConstraintBuffer8 = wideBuffer[contactConstraintWide8, contactManifoldWide8]
