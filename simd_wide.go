// Port of src/simd_wide.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: upstream maps the width-neutral names (b3FloatW, b3AddW, …) onto the W4 or W8 functions
// with the B3_WIDE macro so the wide kernels compile twice. The Go port has one lane width
// (laneCount in simd_scalar.go) and the lane functions already carry the width-neutral names:
//
//	b3FloatW → floatW         b3Vec3W → vec3W
//	b3ZeroW, b3SplatW, b3SetW, b3LoadW, b3StoreW, b3NegW, b3AbsW, b3AddW, b3SubW, b3MulW,
//	b3DivW, b3SqrtW, b3MulAddW, b3MinW, b3MaxW, b3SymClampW, b3AndW, b3OrW, b3AndNotW,
//	b3SoftMaskW, b3GreaterThanW, b3LessThanW, b3EqualsW, b3AllZeroW, b3AnyTrueW, b3BlendW,
//	b3EmbedIndexW, b3MinIndexW, b3TransposeW → zeroW … transposeW (simd_scalar.go)
//	b3LoadVW, b3StoreVW, b3SplatVW, b3NegVW, b3MulSVW, b3MulSubSVW, b3MulAddSVW, b3SubVW,
//	b3AddVW, b3DotW, b3CrossW → loadVW … crossW (simd.go)

// wideAlignment is B3_WIDE_ALIGNMENT.
const wideAlignment = 4 * laneCount
