// Port of src/simd_neon.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build box3d_simd && goexperiment.simd && arm64

package box3d

import (
	"math"
	"simd/archsimd"
)

// PORT: the arm64 ops of the x4 lane backend (simd_x4.go) whose archsimd form differs from
// amd64.

// x4LaneShifts is the { 0, 1, 2, 3 } shift vector of b3MoveMaskW4.
var x4LaneShifts = [laneCount]int32{0, 1, 2, 3}

// minW is b3MinW4: vminq_f32( a, b ) (FMIN): NaN when either lane is NaN, -0 below +0.
// PORT: the compiler treats archsimd's Min as commutative; FMIN's result does not depend on the
// operand order except for which payload two NaN operands give.
func minW(a, b floatW) floatW {
	return floatW{a.v.Min(b.v)}
}

// maxW is b3MaxW4: vmaxq_f32( a, b ) (FMAX, see minW).
func maxW(a, b floatW) floatW {
	return floatW{a.v.Max(b.v)}
}

// symClampW is b3SymClampW4: clamp a to [-b, b]
func symClampW(a, b floatW) floatW {
	nb := negW(b)
	c := maxW(nb, a)
	return minW(c, b)
}

// allZeroW is b3AllZeroW4: every lane of vceqq_f32( a, 0 ) is set.
// PORT: the non-SVE branch tests the four lanes one by one, the SVE branch takes vminvq_u32;
// both are "the minimum lane is non-zero", which ReduceMin (UMINV) computes.
func allZeroW(a floatW) bool {
	var zero archsimd.Float32x4
	return a.v.Equal(zero).ToInt32x4().ToBits().ReduceMin() != 0
}

// anyTrueW is b3AnyTrueW4: the OR of all lane bits is non-zero.
// PORT: upstream ORs the low and high halves, then the two remaining lanes; a non-zero OR is a
// non-zero maximum lane, which ReduceMax (UMAXV) computes.
func anyTrueW(mask floatW) bool {
	return bitsW(mask).ReduceMax() != 0
}

// moveMaskW is b3MoveMaskW4: the sign bit of lane i in bit i, as
// vshlq_u32( vshrq_n_u32( mask, 31 ), { 0, 1, 2, 3 } ) folded with ORs.
// PORT: the four shifted bits are disjoint, so their sum (ReduceSum, ADDV) is their OR.
func moveMaskW(mask floatW) int {
	bits := bitsW(mask).ShiftAllRight(31).Shift(archsimd.LoadInt32x4Array(&x4LaneShifts))
	return int(bits.ReduceSum())
}

// minIndexW is b3MinIndexW4.
// PORT: vmin_f32( vget_low_f32( a ), vget_high_f32( a ) ) is Min with HiToLo in the low half;
// vpmin_f32( m, m ) lane 0 is FMIN of lanes 0 and 1, which Go's min computes (same NaN and
// signed zero rules).
func minIndexW(a floatW, bitCount int) int {
	m := a.v.Min(a.v.HiToLo())
	bits := math.Float32bits(min(m.GetElem(0), m.GetElem(1)))
	return int(bits & ((uint32(1) << bitCount) - 1))
}
