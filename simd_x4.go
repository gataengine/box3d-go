// Port of src/simd_sse2.h and src/simd_neon.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build box3d_simd && goexperiment.simd && (amd64 || arm64)

package box3d

import "simd/archsimd"

// PORT: the width 4 SIMD lane backend of the box3d_simd build, on simd/archsimd.Float32x4. On
// amd64 it mirrors simd_sse2.h, on arm64 simd_neon.h, op for op; this file holds the ops whose
// archsimd form is the same on both (the instruction comments name both headers), and
// simd_x4_amd64.go / simd_x4_arm64.go hold the rest. archsimd encodes the amd64 ops with VEX
// (AVX: VADDPS, VMINPS, …) instead of SSE2, which computes the same values; the build therefore
// requires AVX (simd_x4_amd64.go). Masks are all-ones / all-zeros bit patterns held in a floatW,
// as upstream (the scalar backend uses 0/1 floats). Bit reinterpretation is ToBits /
// BitsToFloat32 on both architectures (archsimd's AsInt32x4 / AsFloat32x4 on amd64 are
// deprecated aliases of them). No op is an estimate (no Reciprocal / ReciprocalSqrt) and none
// fuses (no MulAdd): every op rounds like the scalar backend.

// simdBackend names the lane backend of this build ("scalar" or "x4"), for tests whose expected
// values depend on it.
const simdBackend = "x4"

// laneCount is B3_SIMD_WIDTH of this backend.
const laneCount = 4

// simdNone is B3_SIMD_NONE (core.h), which upstream leaves undefined in its SSE2 and NEON builds.
// PORT: a variable, as in simd_scalar.go, so tests can select either kernel variant.
var simdNone = false

// floatW is b3FloatW4: __m128 (simd_sse2.h) / float32x4_t (simd_neon.h).
type floatW struct {
	v archsimd.Float32x4
}

// x4LaneIndices is the { 0, 1, 2, 3 } of b3EmbedIndexW4 (_mm_setr_epi32 / vld1q_s32( lanes )).
var x4LaneIndices = [laneCount]int32{0, 1, 2, 3}

// laneW reads lane i of a.
// PORT: Go-only accessor replacing upstream's float pointer views of a b3FloatW4 (see
// simd_scalar.go). A constant i becomes one lane move once laneW is inlined.
func laneW(a floatW, i int) float32 {
	return a.v.GetElem(uint8(i))
}

// setLaneW writes value to lane i of a.
// PORT: Go-only accessor, the store form of laneW.
func setLaneW(a *floatW, i int, value float32) {
	a.v = a.v.SetElem(uint8(i), value)
}

// bitsW is the Go-only form of the casts _mm_castps_si128 / vreinterpretq_u32_f32.
func bitsW(a floatW) archsimd.Uint32x4 {
	return a.v.ToBits()
}

// fromBitsW is the Go-only form of the casts _mm_castsi128_ps / vreinterpretq_f32_u32.
func fromBitsW(bits archsimd.Uint32x4) floatW {
	return floatW{bits.BitsToFloat32()}
}

// maskW stores a comparison result as all-ones / all-zeros lanes, the form _mm_cmp*_ps and
// vc*q_f32 return.
// PORT: archsimd returns comparisons as Mask32x4; ToInt32x4 gives the all-ones lanes.
func maskW(m archsimd.Mask32x4) floatW {
	return fromBitsW(m.ToInt32x4().ToBits())
}

// zeroW is b3ZeroW4: _mm_setzero_ps / vdupq_n_f32( 0.0f ).
func zeroW() floatW {
	return floatW{}
}

// splatW is b3SplatW4: _mm_set1_ps / vdupq_n_f32.
func splatW(scalar float32) floatW {
	return floatW{archsimd.BroadcastFloat32x4(scalar)}
}

// setW is b3SetW4: _mm_setr_ps / vld1q_f32 of the array { a, b, c, d }.
func setW(a, b, c, d float32) floatW {
	array := [laneCount]float32{a, b, c, d}
	return floatW{archsimd.LoadFloat32x4Array(&array)}
}

// loadW is b3LoadW4: _mm_loadu_ps / vld1q_f32.
// PORT: the float pointer becomes a slice of at least laneCount values.
func loadW(data []float32) floatW {
	return floatW{archsimd.LoadFloat32x4Array((*[laneCount]float32)(data))}
}

// storeW is b3StoreW4: _mm_storeu_ps / vst1q_f32.
func storeW(data []float32, a floatW) {
	a.v.StoreArray((*[laneCount]float32)(data))
}

// negW is b3NegW4: _mm_xor_ps( a, _mm_set1_ps( -0.0f ) ) / vnegq_f32. Both flip the sign bit,
// which archsimd's Neg does (an XOR with the sign mask on amd64, FNEG on arm64).
func negW(a floatW) floatW {
	return floatW{a.v.Neg()}
}

// absW is b3AbsW4: _mm_andnot_ps( _mm_set1_ps( -0.0f ), a ) / vabsq_f32. Both clear the sign bit,
// which archsimd's Abs does (an AND NOT with the sign mask on amd64, FABS on arm64).
func absW(a floatW) floatW {
	return floatW{a.v.Abs()}
}

// addW is b3AddW4: _mm_add_ps / vaddq_f32.
func addW(a, b floatW) floatW {
	return floatW{a.v.Add(b.v)}
}

// subW is b3SubW4: _mm_sub_ps / vsubq_f32.
func subW(a, b floatW) floatW {
	return floatW{a.v.Sub(b.v)}
}

// mulW is b3MulW4: _mm_mul_ps / vmulq_f32.
func mulW(a, b floatW) floatW {
	return floatW{a.v.Mul(b.v)}
}

// divW is b3DivW4: _mm_div_ps / vdivq_f32.
func divW(a, b floatW) floatW {
	return floatW{a.v.Div(b.v)}
}

// sqrtW is b3SqrtW4: _mm_sqrt_ps / vsqrtq_f32.
func sqrtW(a floatW) floatW {
	return floatW{a.v.Sqrt()}
}

// mulAddW is b3MulAddW4: a + b * c as _mm_add_ps( a, _mm_mul_ps( b, c ) ) /
// vaddq_f32( a, vmulq_f32( b, c ) ). "Cannot use real FMA because it doesn't match the non-SIMD
// path": never archsimd's MulAdd.
func mulAddW(a, b, c floatW) floatW {
	return floatW{a.v.Add(b.v.Mul(c.v))}
}

// andW is b3AndW4: _mm_and_ps / vandq_u32.
func andW(a, b floatW) floatW {
	return fromBitsW(bitsW(a).And(bitsW(b)))
}

// orW is b3OrW4: _mm_or_ps / vorrq_u32.
func orW(a, b floatW) floatW {
	return fromBitsW(bitsW(a).Or(bitsW(b)))
}

// andNotW is b3AndNotW4: a & ~b as _mm_andnot_ps( b, a ) / vbicq_u32( a, b ).
func andNotW(a, b floatW) floatW {
	return fromBitsW(bitsW(a).AndNot(bitsW(b)))
}

// softMaskW is b3SoftMaskW4: lanes where indexA or indexB is 0, as
// _mm_or_si128( _mm_cmpeq_epi32( indexA, 0 ), _mm_cmpeq_epi32( indexB, 0 ) ) /
// vorrq_u32( vceqq_s32( indexA, 0 ), vceqq_s32( indexB, 0 ) ).
// PORT: the int pointers become slices of at least laneCount values.
func softMaskW(indexA, indexB []int32) floatW {
	var zero archsimd.Int32x4
	a := archsimd.LoadInt32x4Array((*[laneCount]int32)(indexA)).Equal(zero)
	b := archsimd.LoadInt32x4Array((*[laneCount]int32)(indexB)).Equal(zero)
	return maskW(a.Or(b))
}

// greaterThanW is b3GreaterThanW4: _mm_cmpgt_ps / vcgtq_f32.
func greaterThanW(a, b floatW) floatW {
	return maskW(a.v.Greater(b.v))
}

// lessThanW is b3LessThanW4: _mm_cmplt_ps / vcltq_f32.
func lessThanW(a, b floatW) floatW {
	return maskW(a.v.Less(b.v))
}

// equalsW is b3EqualsW4: _mm_cmpeq_ps / vceqq_f32.
func equalsW(a, b floatW) floatW {
	return maskW(a.v.Equal(b.v))
}

// blendW is b3BlendW4: component-wise returns mask ? b : a, as
// _mm_or_ps( _mm_and_ps( mask, b ), _mm_andnot_ps( mask, a ) ) / vbslq_f32( mask, b, a ).
// PORT: archsimd's IfElse takes a Mask32x4, which a floatW mask only becomes through a compare,
// so both architectures use the SSE2 bitwise form; it selects the same bits as BSL.
func blendW(a, b, mask floatW) floatW {
	m := bitsW(mask)
	return fromBitsW(bitsW(b).And(m).Or(bitsW(a).AndNot(m)))
}

// embedIndexW is b3EmbedIndexW4.
// Replace the low bitCount mantissa bits of each lane with baseIndex + lane. The value must be
// positive so the embedded index sorts with the value, and ties fall to the lower index.
func embedIndexW(value floatW, baseIndex, bitCount int) floatW {
	mask := (uint32(1) << bitCount) - 1
	index := archsimd.BroadcastInt32x4(int32(baseIndex)).Add(archsimd.LoadInt32x4Array(&x4LaneIndices))
	clearLow := archsimd.BroadcastUint32x4(^mask)
	return fromBitsW(bitsW(value).And(clearLow).Or(index.ToBits()))
}

// transposeW is b3TransposeW4: _mm_unpacklo_ps / _mm_unpackhi_ps (simd_sse2.h) and
// vzip1q_f32 / vzip2q_f32 (simd_neon.h), the same lane moves.
// PORT: the output pointers become return values. archsimd has the interleaves on the integer
// types (PUNPCKLDQ / PUNPCKHDQ, ZIP1 / ZIP2), so the lanes are moved as bits.
func transposeW(r0, r1, r2, r3 floatW) (c0, c1, c2, c3 floatW) {
	b0, b1, b2, b3 := r0.v.ToBits(), r1.v.ToBits(), r2.v.ToBits(), r3.v.ToBits()
	t0, t1 := b0.InterleaveLo(b2), b1.InterleaveLo(b3)
	t2, t3 := b0.InterleaveHi(b2), b1.InterleaveHi(b3)
	return floatW{t0.InterleaveLo(t1).BitsToFloat32()}, floatW{t0.InterleaveHi(t1).BitsToFloat32()},
		floatW{t2.InterleaveLo(t3).BitsToFloat32()}, floatW{t2.InterleaveHi(t3).BitsToFloat32()}
}
