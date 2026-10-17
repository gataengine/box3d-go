// Port of src/simd_sse2.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build box3d_simd && goexperiment.simd && amd64

package box3d

import "simd/archsimd"

// PORT: the amd64 ops of the x4 lane backend (simd_x4.go) whose archsimd form differs from
// arm64. archsimd's Float32x4 ops are AVX encoded, so this build needs AVX where upstream's
// width 4 build needs SSE2; every x86-64 game target has AVX, and SSE2-only machines use the
// default (scalar) build.

// avxRequiredMessage is the panic message of the box3d_simd build on an amd64 CPU without AVX.
const avxRequiredMessage = "box3d: the box3d_simd build requires AVX on amd64; use the default build"

func init() {
	if !archsimd.X86.AVX() {
		panic(avxRequiredMessage)
	}
}

// minps is _mm_min_ps( a, b ): a < b ? a : b per lane, so b when either is NaN or both are zero.
// PORT: archsimd's Min is VMINPS, but the compiler marks it commutative and may swap the
// operands, which changes the NaN and signed zero results. The ordered compare (VCMPPS, false
// for NaN) selecting a, else b, gives _mm_min_ps( a, b ) with the operand order fixed.
func minps(a, b archsimd.Float32x4) archsimd.Float32x4 {
	return selectps(a.Less(b).ToInt32x4().ToBits(), a, b)
}

// maxps is _mm_max_ps( a, b ): a > b ? a : b per lane (see minps).
func maxps(a, b archsimd.Float32x4) archsimd.Float32x4 {
	return selectps(a.Greater(b).ToInt32x4().ToBits(), a, b)
}

// selectps is mask ? a : b per lane for an all-ones / all-zeros mask.
func selectps(mask archsimd.Uint32x4, a, b archsimd.Float32x4) archsimd.Float32x4 {
	return a.ToBits().And(mask).Or(b.ToBits().AndNot(mask)).BitsToFloat32()
}

// minW is b3MinW4: _mm_min_ps( a, b ).
func minW(a, b floatW) floatW {
	return floatW{minps(a.v, b.v)}
}

// maxW is b3MaxW4: _mm_max_ps( a, b ).
func maxW(a, b floatW) floatW {
	return floatW{maxps(a.v, b.v)}
}

// symClampW is b3SymClampW4: clamp a to [-b, b]
// PORT: b3MaxW4( b3NegW4( b ), a ) then b3MinW4( c, b ) on the archsimd values. Over the
// inlining budget; no kernel calls it.
func symClampW(a, b floatW) floatW {
	nb := b.v.Neg()
	c := maxps(nb, a.v)
	return floatW{minps(c, b.v)}
}

// movemaskW is _mm_movemask_ps: the sign bit of lane i in bit i.
// PORT: archsimd has VMOVMSKPS only on Mask32x4 (ToBits); the lanes holding a set sign bit
// become that mask through a signed compare with zero (0 > a as int32), so every bit pattern
// gives _mm_movemask_ps's result, not only all-ones / all-zeros masks.
func movemaskW(a floatW) int {
	var zero archsimd.Int32x4
	return int(zero.Greater(bitsW(a).BitsToInt32()).ToBits())
}

// allZeroW is b3AllZeroW4: _mm_movemask_ps( _mm_cmpeq_ps( a, 0 ) ) == 0xF.
func allZeroW(a floatW) bool {
	var zero archsimd.Float32x4
	return a.v.Equal(zero).ToBits() == 0xF
}

// anyTrueW is b3AnyTrueW4: _mm_movemask_ps( mask ) != 0.
func anyTrueW(mask floatW) bool {
	return movemaskW(mask) != 0
}

// moveMaskW is b3MoveMaskW4: _mm_movemask_ps( mask ).
func moveMaskW(mask floatW) int {
	return movemaskW(mask)
}

// minIndexW is b3MinIndexW4.
// Recovers the index embedded by b3EmbedIndexW4 from the lane holding the minimum.
// PORT: _mm_shuffle_ps( a, a, _MM_SHUFFLE( 2, 3, 0, 1 ) ) and _MM_SHUFFLE( 1, 0, 3, 2 ) are
// ConcatPermuteScalars with the lanes (1, 0, 3, 2) and (2, 3, 0, 1) of a, where y's lanes are
// numbered 4..7 (VSHUFPS); _mm_min_ps is minps.
func minIndexW(a floatW, bitCount int) int {
	v := minps(a.v, a.v.ConcatPermuteScalars(1, 0, 7, 6, a.v))
	v = minps(v, v.ConcatPermuteScalars(2, 3, 4, 5, v))
	return int(v.ToBits().GetElem(0) & (1<<bitCount - 1))
}
