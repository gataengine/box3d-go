// Port of src/simd_avx2.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build box3d_simd && goexperiment.simd && amd64 && !box3d_no8

package box3d

import "simd/archsimd"

// PORT: the width 8 SIMD lane backend of the box3d_simd build on amd64, on
// simd/archsimd.Float32x8, mirroring simd_avx2.h op for op. Upstream names these b3*W8 and the
// B3_WIDE macro maps the width-neutral names onto them in wide8.c; here they carry the suffix 8
// that tools/widegen appends to the width 4 lane contract (simd_scalar.go, simd_x4.go) in the
// generated width 8 kernels (*8_gen.go). Masks are all-ones / all-zeros bit patterns held in a
// floatW8, as upstream. The 256 bit integer ops (VPCMPEQD, VPADDD, VPUNPCKLDQ, archsimd's
// emulated Neg / Abs) need AVX2, as upstream's B3_SIMD_HAS_WIDTH_8 build (-mavx2): nothing here
// runs unless wide_lanes8_amd64.go registered the width 8 kernels after checking AVX2. No op is
// an estimate and none fuses (no MulAdd).

// laneCount8 is B3_SIMD_WIDTH of the width 8 kernel set (wide8.c).
const laneCount8 = 8

// simdNone8 is B3_SIMD_NONE in the width 8 build, which upstream never defines there.
// PORT: a constant, unlike simdNone: no width 8 configuration has the scalar kernel variants.
const simdNone8 = false

// floatW8 is b3FloatW8: __m256.
type floatW8 struct {
	v archsimd.Float32x8
}

// x8LaneIndices is the _mm256_setr_epi32( 0, 1, 2, 3, 4, 5, 6, 7 ) of b3EmbedIndexW8.
var x8LaneIndices = [laneCount8]int32{0, 1, 2, 3, 4, 5, 6, 7}

// laneW8 reads lane i of a.
// PORT: Go-only accessor replacing upstream's float pointer views of a b3FloatW8 (see laneW).
// archsimd has no lane access on 256 bit vectors, so the lane comes from the 128 bit half.
func laneW8(a floatW8, i int) float32 {
	if i < 4 {
		return a.v.GetLo().GetElem(uint8(i))
	}
	return a.v.GetHi().GetElem(uint8(i - 4))
}

// setLaneW8 writes value to lane i of a.
// PORT: Go-only accessor, the store form of laneW8.
func setLaneW8(a *floatW8, i int, value float32) {
	if i < 4 {
		a.v = a.v.SetLo(a.v.GetLo().SetElem(uint8(i), value))
		return
	}
	a.v = a.v.SetHi(a.v.GetHi().SetElem(uint8(i-4), value))
}

// bitsW8 is the Go-only form of the cast _mm256_castps_si256.
func bitsW8(a floatW8) archsimd.Uint32x8 {
	return a.v.ToBits()
}

// fromBitsW8 is the Go-only form of the cast _mm256_castsi256_ps.
func fromBitsW8(bits archsimd.Uint32x8) floatW8 {
	return floatW8{bits.BitsToFloat32()}
}

// maskW8 stores a comparison result as all-ones / all-zeros lanes, the form _mm256_cmp_ps and
// _mm256_cmpeq_epi32 return.
// PORT: archsimd returns comparisons as Mask32x8; ToInt32x8 gives the all-ones lanes.
func maskW8(m archsimd.Mask32x8) floatW8 {
	return fromBitsW8(m.ToInt32x8().ToBits())
}

// zeroW8 is b3ZeroW8: _mm256_setzero_ps.
func zeroW8() floatW8 {
	return floatW8{}
}

// splatW8 is b3SplatW8: _mm256_set1_ps.
func splatW8(scalar float32) floatW8 {
	return floatW8{archsimd.BroadcastFloat32x8(scalar)}
}

// setW8 is b3SetW8: _mm256_setr_ps( a, b, c, d, e, f, g, h ).
// PORT: built from two 128 bit halves (VINSERTF128): an [8]float32 composite literal made the
// compiler copy the array with SSE moves before the 256 bit load.
func setW8(a, b, c, d, e, f, g, h float32) floatW8 {
	lo := [4]float32{a, b, c, d}
	hi := [4]float32{e, f, g, h}
	var v archsimd.Float32x8
	return floatW8{v.SetLo(archsimd.LoadFloat32x4Array(&lo)).SetHi(archsimd.LoadFloat32x4Array(&hi))}
}

// loadW8 is b3LoadW8: _mm256_loadu_ps.
// PORT: the float pointer becomes a slice of at least laneCount8 values.
func loadW8(data []float32) floatW8 {
	return floatW8{archsimd.LoadFloat32x8Array((*[laneCount8]float32)(data))}
}

// storeW8 is b3StoreW8: _mm256_storeu_ps.
func storeW8(data []float32, a floatW8) {
	a.v.StoreArray((*[laneCount8]float32)(data))
}

// negW8 is b3NegW8: _mm256_xor_ps( a, _mm256_set1_ps( -0.0f ) ), which archsimd's Neg is (an
// XOR with the sign mask).
func negW8(a floatW8) floatW8 {
	return floatW8{a.v.Neg()}
}

// absW8 is b3AbsW8: _mm256_andnot_ps( _mm256_set1_ps( -0.0f ), a ), which archsimd's Abs is (an
// AND NOT with the sign mask).
func absW8(a floatW8) floatW8 {
	return floatW8{a.v.Abs()}
}

// addW8 is b3AddW8: _mm256_add_ps.
func addW8(a, b floatW8) floatW8 {
	return floatW8{a.v.Add(b.v)}
}

// subW8 is b3SubW8: _mm256_sub_ps.
func subW8(a, b floatW8) floatW8 {
	return floatW8{a.v.Sub(b.v)}
}

// mulW8 is b3MulW8: _mm256_mul_ps.
func mulW8(a, b floatW8) floatW8 {
	return floatW8{a.v.Mul(b.v)}
}

// divW8 is b3DivW8: _mm256_div_ps.
func divW8(a, b floatW8) floatW8 {
	return floatW8{a.v.Div(b.v)}
}

// sqrtW8 is b3SqrtW8: _mm256_sqrt_ps.
func sqrtW8(a floatW8) floatW8 {
	return floatW8{a.v.Sqrt()}
}

// mulAddW8 is b3MulAddW8: a + b * c as _mm256_add_ps( a, _mm256_mul_ps( b, c ) ), never
// archsimd's MulAdd (see mulAddW).
func mulAddW8(a, b, c floatW8) floatW8 {
	return floatW8{a.v.Add(b.v.Mul(c.v))}
}

// selectps8 is mask ? a : b per lane for an all-ones / all-zeros mask.
func selectps8(mask archsimd.Uint32x8, a, b archsimd.Float32x8) archsimd.Float32x8 {
	return a.ToBits().And(mask).Or(b.ToBits().AndNot(mask)).BitsToFloat32()
}

// minW8 is b3MinW8: _mm256_min_ps( a, b ), a < b ? a : b per lane.
// PORT: archsimd's Min is VMINPS, which the compiler treats as commutative (see minps), so the
// ordered compare selecting a, else b, fixes the operand order and with it the NaN and signed
// zero results.
func minW8(a, b floatW8) floatW8 {
	return floatW8{selectps8(a.v.Less(b.v).ToInt32x8().ToBits(), a.v, b.v)}
}

// maxW8 is b3MaxW8: _mm256_max_ps( a, b ), a > b ? a : b per lane (see minW8).
func maxW8(a, b floatW8) floatW8 {
	return floatW8{selectps8(a.v.Greater(b.v).ToInt32x8().ToBits(), a.v, b.v)}
}

// symClampW8 is b3SymClampW8: clamp a to [-b, b]
func symClampW8(a, b floatW8) floatW8 {
	nb := negW8(b)
	c := maxW8(nb, a)
	return minW8(c, b)
}

// andW8 is b3AndW8: _mm256_and_ps.
func andW8(a, b floatW8) floatW8 {
	return fromBitsW8(bitsW8(a).And(bitsW8(b)))
}

// orW8 is b3OrW8: _mm256_or_ps.
func orW8(a, b floatW8) floatW8 {
	return fromBitsW8(bitsW8(a).Or(bitsW8(b)))
}

// andNotW8 is b3AndNotW8: a & ~b as _mm256_andnot_ps( b, a ).
func andNotW8(a, b floatW8) floatW8 {
	return fromBitsW8(bitsW8(a).AndNot(bitsW8(b)))
}

// softMaskW8 is b3SoftMaskW8: lanes where indexA or indexB is 0, as
// _mm256_or_si256( _mm256_cmpeq_epi32( indexA, 0 ), _mm256_cmpeq_epi32( indexB, 0 ) ).
// PORT: the int pointers become slices of at least laneCount8 values.
func softMaskW8(indexA, indexB []int32) floatW8 {
	var zero archsimd.Int32x8
	a := archsimd.LoadInt32x8Array((*[laneCount8]int32)(indexA)).Equal(zero)
	b := archsimd.LoadInt32x8Array((*[laneCount8]int32)(indexB)).Equal(zero)
	return maskW8(a.Or(b))
}

// greaterThanW8 is b3GreaterThanW8: _mm256_cmp_ps( a, b, _CMP_GT_OQ ).
func greaterThanW8(a, b floatW8) floatW8 {
	return maskW8(a.v.Greater(b.v))
}

// lessThanW8 is b3LessThanW8: _mm256_cmp_ps( a, b, _CMP_LT_OQ ).
func lessThanW8(a, b floatW8) floatW8 {
	return maskW8(a.v.Less(b.v))
}

// equalsW8 is b3EqualsW8: _mm256_cmp_ps( a, b, _CMP_EQ_OQ ).
func equalsW8(a, b floatW8) floatW8 {
	return maskW8(a.v.Equal(b.v))
}

// movemaskW8 is _mm256_movemask_ps: the sign bit of lane i in bit i.
// PORT: as movemaskW, the lanes holding a set sign bit become a Mask32x8 (VMOVMSKPS) through a
// signed compare with zero, so every bit pattern gives _mm256_movemask_ps's result.
func movemaskW8(a floatW8) int {
	var zero archsimd.Int32x8
	return int(zero.Greater(bitsW8(a).BitsToInt32()).ToBits())
}

// allZeroW8 is b3AllZeroW8: _mm256_movemask_ps( _mm256_cmp_ps( a, 0, _CMP_EQ_OQ ) ) == 0xFF.
func allZeroW8(a floatW8) bool {
	var zero archsimd.Float32x8
	return a.v.Equal(zero).ToBits() == 0xFF
}

// anyTrueW8 is b3AnyTrueW8: _mm256_movemask_ps( mask ) != 0.
func anyTrueW8(mask floatW8) bool {
	return movemaskW8(mask) != 0
}

// moveMaskW8 is _mm256_movemask_ps( mask ), the width 8 sibling of b3MoveMaskW4.
// PORT: simd_avx2.h has no b3MoveMaskW8; tests use it as they use moveMaskW.
func moveMaskW8(mask floatW8) int {
	return movemaskW8(mask)
}

// blendW8 is b3BlendW8: component-wise returns mask ? b : a, as
// _mm256_or_ps( _mm256_and_ps( mask, b ), _mm256_andnot_ps( mask, a ) ).
func blendW8(a, b, mask floatW8) floatW8 {
	m := bitsW8(mask)
	return fromBitsW8(bitsW8(b).And(m).Or(bitsW8(a).AndNot(m)))
}

// embedIndexW8 is b3EmbedIndexW8.
// Replace the low bitCount mantissa bits of each lane with baseIndex + lane. The value must be
// positive so the embedded index sorts with the value, and ties fall to the lower index.
func embedIndexW8(value floatW8, baseIndex, bitCount int) floatW8 {
	mask := (uint32(1) << bitCount) - 1
	index := archsimd.BroadcastInt32x8(int32(baseIndex)).Add(archsimd.LoadInt32x8Array(&x8LaneIndices))
	clearLow := archsimd.BroadcastUint32x8(^mask)
	return fromBitsW8(bitsW8(value).And(clearLow).Or(index.ToBits()))
}

// minIndexW8 is b3MinIndexW8.
// Recovers the index embedded by b3EmbedIndexW8 from the lane holding the minimum.
// PORT: _mm256_castps256_ps128 and _mm256_extractf128_ps( a, 1 ) are GetLo and GetHi;
// _mm_min_ps is minps (simd_x4_amd64.go); _mm_shuffle_ps( m, m, _MM_SHUFFLE( 2, 3, 0, 1 ) ) and
// _MM_SHUFFLE( 1, 0, 3, 2 ) are ConcatPermuteScalars with the lanes (1, 0, 3, 2) and (2, 3, 0, 1)
// of m, where y's lanes are numbered 4..7 (VSHUFPS). Upstream passes the shuffled value as the
// first operand of _mm_min_ps, unlike b3MinIndexW4.
func minIndexW8(a floatW8, bitCount int) int {
	m := minps(a.v.GetLo(), a.v.GetHi())
	m = minps(m.ConcatPermuteScalars(1, 0, 7, 6, m), m)
	m = minps(m.ConcatPermuteScalars(2, 3, 4, 5, m), m)
	return int(m.ToBits().GetElem(0) & (1<<bitCount - 1))
}

// transposeW8 is b3TransposeW8: eight rows in, eight columns out (lane i of cj is lane j of ri).
// PORT: the output pointers become return values. _mm256_unpacklo_ps / _mm256_unpackhi_ps are
// the integer interleaves InterleaveLoGrouped / InterleaveHiGrouped on the bits (VPUNPCKLDQ /
// VPUNPCKHDQ, the same lane moves); _mm256_shuffle_ps( x, y, _MM_SHUFFLE( 1, 0, 1, 0 ) ) and
// _MM_SHUFFLE( 3, 2, 3, 2 ) are ConcatPermuteScalarsGrouped( 0, 1, 4, 5 ) and ( 2, 3, 6, 7 ),
// where y's lanes are numbered 4..7 (VSHUFPS); _mm256_permute2f128_ps( x, y, 0x20 ) and 0x31
// are ConcatPermute128Scalars( 0, 2 ) and ( 1, 3 ) (VPERM2F128).
func transposeW8(r0, r1, r2, r3, r4, r5, r6, r7 floatW8) (c0, c1, c2, c3, c4, c5, c6, c7 floatW8) {
	b0, b1, b2, b3 := bitsW8(r0), bitsW8(r1), bitsW8(r2), bitsW8(r3)
	b4, b5, b6, b7 := bitsW8(r4), bitsW8(r5), bitsW8(r6), bitsW8(r7)
	t0 := b0.InterleaveLoGrouped(b1).BitsToFloat32()
	t1 := b0.InterleaveHiGrouped(b1).BitsToFloat32()
	t2 := b2.InterleaveLoGrouped(b3).BitsToFloat32()
	t3 := b2.InterleaveHiGrouped(b3).BitsToFloat32()
	t4 := b4.InterleaveLoGrouped(b5).BitsToFloat32()
	t5 := b4.InterleaveHiGrouped(b5).BitsToFloat32()
	t6 := b6.InterleaveLoGrouped(b7).BitsToFloat32()
	t7 := b6.InterleaveHiGrouped(b7).BitsToFloat32()
	tt0 := t0.ConcatPermuteScalarsGrouped(0, 1, 4, 5, t2)
	tt1 := t0.ConcatPermuteScalarsGrouped(2, 3, 6, 7, t2)
	tt2 := t1.ConcatPermuteScalarsGrouped(0, 1, 4, 5, t3)
	tt3 := t1.ConcatPermuteScalarsGrouped(2, 3, 6, 7, t3)
	tt4 := t4.ConcatPermuteScalarsGrouped(0, 1, 4, 5, t6)
	tt5 := t4.ConcatPermuteScalarsGrouped(2, 3, 6, 7, t6)
	tt6 := t5.ConcatPermuteScalarsGrouped(0, 1, 4, 5, t7)
	tt7 := t5.ConcatPermuteScalarsGrouped(2, 3, 6, 7, t7)

	c0 = floatW8{tt0.ConcatPermute128Scalars(0, 2, tt4)}
	c1 = floatW8{tt1.ConcatPermute128Scalars(0, 2, tt5)}
	c2 = floatW8{tt2.ConcatPermute128Scalars(0, 2, tt6)}
	c3 = floatW8{tt3.ConcatPermute128Scalars(0, 2, tt7)}
	c4 = floatW8{tt0.ConcatPermute128Scalars(1, 3, tt4)}
	c5 = floatW8{tt1.ConcatPermute128Scalars(1, 3, tt5)}
	c6 = floatW8{tt2.ConcatPermute128Scalars(1, 3, tt6)}
	c7 = floatW8{tt3.ConcatPermute128Scalars(1, 3, tt7)}
	return c0, c1, c2, c3, c4, c5, c6, c7
}
