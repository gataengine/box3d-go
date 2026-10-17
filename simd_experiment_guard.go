// Go-only: build guard of the box3d_simd build of the Box3D port
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build box3d_simd && !goexperiment.simd

package box3d

// The box3d_simd build needs GOEXPERIMENT=simd (simd/archsimd). This name is intentionally
// undefined, so the compile error names the fix.
var _ = box3dSimdBuildRequiresGOEXPERIMENTsimd
