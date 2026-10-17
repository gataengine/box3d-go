// Go-only test: upstream has no test file for the lane layer (src/simd_scalar.h, src/simd_sse2.h,
// src/simd_neon.h).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"runtime"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

// These specs run against whichever lane backend is built: simd_scalar.go (default build, and the
// box3d_simd build on architectures without a SIMD backend) or simd_x4.go (box3d_simd on amd64,
// mirroring simd_sse2.h, and on arm64, mirroring simd_neon.h). Expected values are plain float32
// expressions or, where the headers differ (signed zeros, NaN), the header's semantics for the
// backend under test.

// laneBackend is the upstream header the built lane backend mirrors.
func laneBackend() string {
	if simdBackend == "scalar" {
		return "scalar"
	}
	if runtime.GOARCH == "amd64" {
		return "sse2"
	}
	return "neon"
}

// lanesOf returns the lanes of a.
func lanesOf(a floatW) [laneCount]float32 {
	var out [laneCount]float32
	storeW(out[:], a)
	return out
}

// wOf builds a floatW from lanes.
func wOf(lanes [laneCount]float32) floatW {
	return loadW(lanes[:])
}

// expectLanes checks got against want bit for bit. A NaN expectation only requires a NaN: the
// NaN payload is whatever the hardware produces and the plain float32 reference may be folded by
// the compiler.
func expectLanes(op string, got floatW, want [laneCount]float32) {
	GinkgoHelper()
	g := lanesOf(got)
	for i := range laneCount {
		if isNaN32(want[i]) {
			Expect(isNaN32(g[i])).To(BeTrue(), "%s lane %d: got %v (%#08x), want NaN", op, i, g[i], math.Float32bits(g[i]))
			continue
		}
		Expect(math.Float32bits(g[i])).To(Equal(math.Float32bits(want[i])),
			"%s lane %d: got %v (%#08x), want %v (%#08x)", op, i, g[i], math.Float32bits(g[i]), want[i], math.Float32bits(want[i]))
	}
}

func isNaN32(x float32) bool {
	return x != x
}

// perLane applies f to each lane.
func perLane(f func(int) float32) [laneCount]float32 {
	var out [laneCount]float32
	for i := range laneCount {
		out[i] = f(i)
	}
	return out
}

// maskBits is the moveMaskW value expected for per-lane conditions.
func maskBits(f func(int) bool) int {
	r := 0
	for i := range laneCount {
		if f(i) {
			r |= 1 << i
		}
	}
	return r
}

// The reference min/max of each header.

// headerMin is b3MinW4 per lane: simd_scalar.h:90 `a.x <= b.x ? a.x : b.x`; simd_sse2.h:87
// _mm_min_ps( a, b ), which returns the second operand when either is NaN or both are zero
// (a < b ? a : b); simd_neon.h:84 vminq_f32 (FMIN), which returns NaN when either is NaN and
// orders -0 below +0, as Go's min.
func headerMin(a, b float32) float32 {
	switch laneBackend() {
	case "scalar":
		if a <= b {
			return a
		}
		return b
	case "sse2":
		if a < b {
			return a
		}
		return b
	default:
		return min(a, b)
	}
}

// headerMax is b3MaxW4 per lane: simd_scalar.h:100 `a.x >= b.x ? a.x : b.x`; simd_sse2.h:92
// _mm_max_ps( a, b ) (a > b ? a : b); simd_neon.h:89 vmaxq_f32 (FMAX, as Go's max).
func headerMax(a, b float32) float32 {
	switch laneBackend() {
	case "scalar":
		if a >= b {
			return a
		}
		return b
	case "sse2":
		if a > b {
			return a
		}
		return b
	default:
		return max(a, b)
	}
}

// headerSymClamp is b3SymClampW4 per lane: simd_scalar.h:111-118 (min with b, then -b if
// r <= -b); simd_sse2.h:98-100 and simd_neon.h:96-98 b3MinW4( b3MaxW4( -b, a ), b ).
func headerSymClamp(a, b float32) float32 {
	if laneBackend() == "scalar" {
		r := b
		if a <= b {
			r = a
		}
		if r <= -b {
			r = -b
		}
		return r
	}
	return headerMin(headerMax(-b, a), b)
}

// headerAbs is b3AbsW4 per lane: simd_scalar.h:54 `a.x < 0.0f ? -a.x : a.x` (keeps -0);
// simd_sse2.h:51 _mm_andnot_ps( -0.0f, a ) and simd_neon.h:49 vabsq_f32 clear the sign bit.
func headerAbs(a float32) float32 {
	if laneBackend() == "scalar" {
		if a < 0.0 {
			return -a
		}
		return a
	}
	return math.Float32frombits(math.Float32bits(a) &^ 0x80000000)
}

// laneArithmeticRows are ordinary 4-lane inputs (no signed zeros, NaN or infinities).
var laneArithmeticRows = [][3][laneCount]float32{
	{{1.0, -2.0, 3.5, -4.25}, {0.5, 0.25, -8.0, 3.0}, {2.0, -3.0, 0.125, 7.5}},
	{{1e-3, 12345.678, -0.1, 3.0e7}, {7.0, -1e-4, 0.3, -2.5e-3}, {-0.7, 9.5, 1.1, 1e5}},
	{{0.1, 0.2, 0.3, 0.4}, {0.3, 0.2, 0.1, 0.4}, {1.0 / 3.0, 2.0 / 3.0, 0.7, 1e-20}},
	{{float32(math.Pi), float32(math.E), 1.5e-30, 2.5e30}, {1e-7, -1e7, 6.0e-31, 3.0e-9}, {1.0, 1.0, 1e10, -1e-10}},
}

// laneEdgeValues are the edge inputs of LaneEdgeValues: +0, -0, the smallest denormal, MaxFloat32,
// +Inf and -Inf.
var laneEdgeValues = []float32{
	0.0,
	float32(math.Copysign(0, -1)),
	math.Float32frombits(1),
	math.MaxFloat32,
	float32(math.Inf(1)),
	float32(math.Inf(-1)),
}

// edgePairs packs every ordered pair of edge values into 4-lane rows.
func edgePairs() (as, bs [][laneCount]float32) {
	var a, b [laneCount]float32
	n := 0
	for _, x := range laneEdgeValues {
		for _, y := range laneEdgeValues {
			a[n], b[n] = x, y
			n++
			if n == laneCount {
				as, bs = append(as, a), append(bs, b)
				n = 0
			}
		}
	}
	if n != 0 {
		as, bs = append(as, a), append(bs, b)
	}
	return as, bs
}

// checkBinaryOps runs the two operand ops on one row.
func checkBinaryOps(a, b [laneCount]float32) {
	GinkgoHelper()
	wa, wb := wOf(a), wOf(b)
	expectLanes("addW", addW(wa, wb), perLane(func(i int) float32 { return a[i] + b[i] }))
	expectLanes("subW", subW(wa, wb), perLane(func(i int) float32 { return a[i] - b[i] }))
	expectLanes("mulW", mulW(wa, wb), perLane(func(i int) float32 { return float32(a[i] * b[i]) }))
	expectLanes("divW", divW(wa, wb), perLane(func(i int) float32 { return a[i] / b[i] }))
	expectLanes("minW", minW(wa, wb), perLane(func(i int) float32 { return headerMin(a[i], b[i]) }))
	expectLanes("maxW", maxW(wa, wb), perLane(func(i int) float32 { return headerMax(a[i], b[i]) }))
	expectLanes("symClampW", symClampW(wa, wb), perLane(func(i int) float32 { return headerSymClamp(a[i], b[i]) }))
}

// checkUnaryOps runs the one operand ops on one row.
func checkUnaryOps(a [laneCount]float32) {
	GinkgoHelper()
	wa := wOf(a)
	expectLanes("sqrtW", sqrtW(wa), perLane(func(i int) float32 { return sqrtf(a[i]) }))
	expectLanes("negW", negW(wa), perLane(func(i int) float32 { return -a[i] }))
	expectLanes("absW", absW(wa), perLane(func(i int) float32 { return headerAbs(a[i]) }))
}

// checkMulAdd runs mulAddW on one row: a + b * c with the product rounded first.
func checkMulAdd(a, b, c [laneCount]float32) {
	GinkgoHelper()
	expectLanes("mulAddW", mulAddW(wOf(a), wOf(b), wOf(c)), perLane(func(i int) float32 {
		product := float32(b[i] * c[i])
		return a[i] + product
	}))
}

// checkCompares runs the compares on one row, through moveMaskW, and blends with each mask.
func checkCompares(a, b [laneCount]float32) {
	GinkgoHelper()
	wa, wb := wOf(a), wOf(b)
	cases := []struct {
		name string
		mask floatW
		want func(int) bool
	}{
		{"greaterThanW", greaterThanW(wa, wb), func(i int) bool { return a[i] > b[i] }},
		{"lessThanW", lessThanW(wa, wb), func(i int) bool { return a[i] < b[i] }},
		{"equalsW", equalsW(wa, wb), func(i int) bool { return a[i] == b[i] }},
	}
	for _, c := range cases {
		bits := maskBits(c.want)
		Expect(moveMaskW(c.mask)).To(Equal(bits), "moveMaskW(%s(%v, %v))", c.name, a, b)
		Expect(anyTrueW(c.mask)).To(Equal(bits != 0), "anyTrueW(%s)", c.name)
		Expect(allZeroW(c.mask)).To(Equal(bits == 0), "allZeroW(%s)", c.name)
		expectLanes("blendW("+c.name+")", blendW(wa, wb, c.mask), perLane(func(i int) float32 {
			if bits&(1<<i) != 0 {
				return b[i]
			}
			return a[i]
		}))
	}
}

var _ = Describe("SIMDLanes", func() {
	It("LaneArithmetic", func() {
		for _, row := range laneArithmeticRows {
			a, b, c := row[0], row[1], row[2]
			checkBinaryOps(a, b)
			checkBinaryOps(b, c)
			checkUnaryOps(a)
			checkUnaryOps(perLane(func(i int) float32 { return headerAbs(b[i]) }))
			checkMulAdd(a, b, c)
			checkMulAdd(c, a, b)
		}

		// A product that is not exact in float32: (1+2^-12)^2 = 1 + 2^-11 + 2^-24 rounds to
		// 1 + 2^-11 (tie to even), so a + b*c with a = -(1 + 2^-11) is 0 when the product is
		// rounded first and 2^-24 when fused.
		e := float32(1.0 / 4096.0)
		checkMulAdd([laneCount]float32{-(1 + 2*e), 1, -1, 0.5},
			[laneCount]float32{1 + e, 1 + e, 1 + e, 1 + e},
			[laneCount]float32{1 + e, 1 + e, 1 + e, 1 + e})
		ensure(laneW(mulAddW(splatW(-(1+2*e)), splatW(1+e), splatW(1+e)), 0) == 0)
	})

	It("LaneMasks", func() {
		for _, row := range laneArithmeticRows {
			checkCompares(row[0], row[1])
			checkCompares(row[1], row[0])
			checkCompares(row[0], row[0])
		}

		a := setW(1.0, -2.0, 3.0, -4.0)
		b := setW(2.0, -3.0, 3.0, 0.0)
		gt := greaterThanW(a, b) // 0b0010
		lt := lessThanW(a, b)    // 0b1001
		eq := equalsW(a, b)      // 0b0100
		ensure(moveMaskW(gt) == 0b0010 && moveMaskW(lt) == 0b1001 && moveMaskW(eq) == 0b0100)

		ensure(moveMaskW(andW(orW(gt, lt), lt)) == 0b1001)
		ensure(moveMaskW(orW(gt, eq)) == 0b0110)
		ensure(moveMaskW(andNotW(orW(gt, lt), lt)) == 0b0010)
		ensure(moveMaskW(andW(gt, lt)) == 0)
		ensure(allZeroW(andW(gt, lt)))
		ensure(!anyTrueW(andW(gt, lt)))
		ensure(anyTrueW(eq) && !allZeroW(eq))
		ensure(allZeroW(zeroW()) && !anyTrueW(zeroW()))
		ensure(moveMaskW(orW(orW(gt, lt), eq)) == 0b1111)
		expectLanes("blendW", blendW(a, b, lt), [laneCount]float32{2.0, -2.0, 3.0, 0.0})
		expectLanes("blendW", blendW(a, b, andNotW(orW(gt, lt), lt)), [laneCount]float32{1.0, -3.0, 3.0, -4.0})

		indexA := []int32{0, 3, 0, 7}
		indexB := []int32{5, 0, 0, 2}
		soft := softMaskW(indexA, indexB)
		ensure(moveMaskW(soft) == 0b0111)
		expectLanes("blendW(softMaskW)", blendW(a, b, soft), [laneCount]float32{2.0, -3.0, 3.0, -4.0})
		ensure(moveMaskW(softMaskW([]int32{1, 2, 3, 4}, []int32{4, 3, 2, 1})) == 0)

		// allZeroW compares lanes with 0.0f (simd_sse2.h:148 _mm_cmpeq_ps, simd_neon.h:141
		// vceqq_f32, simd_scalar.h `a.x == 0.0f`), so -0 counts as zero in every backend.
		ensure(allZeroW(splatW(float32(math.Copysign(0, -1)))))
		ensure(!allZeroW(setW(0, 0, 0, math.Float32frombits(1))))
	})

	It("LaneEdgeValues", func() {
		as, bs := edgePairs()
		for i := range as {
			checkBinaryOps(as[i], bs[i])
			checkCompares(as[i], bs[i])
			checkMulAdd(as[i], bs[i], as[i])
			checkMulAdd(bs[i], as[i], bs[i])
		}
		for _, x := range laneEdgeValues {
			checkUnaryOps([laneCount]float32{x, x, x, x})
		}

		// Signed zeros, cited per header (see headerMin / headerMax / headerAbs).
		pz, nz := float32(0.0), float32(math.Copysign(0, -1))
		minPN, minNP := lanesOf(minW(splatW(pz), splatW(nz)))[0], lanesOf(minW(splatW(nz), splatW(pz)))[0]
		maxPN, maxNP := lanesOf(maxW(splatW(pz), splatW(nz)))[0], lanesOf(maxW(splatW(nz), splatW(pz)))[0]
		absN := lanesOf(absW(splatW(nz)))[0]
		sign := func(x float32) bool { return math.Signbit(float64(x)) }
		switch laneBackend() {
		case "scalar":
			// simd_scalar.h:90/100: equal operands return the first.
			ensure(!sign(minPN) && sign(minNP) && !sign(maxPN) && sign(maxNP))
			// simd_scalar.h:54: -0 < 0 is false, so -0 stays -0.
			ensure(sign(absN))
		case "sse2":
			// simd_sse2.h:87/92 minps/maxps return the second operand for two zeros.
			ensure(sign(minPN) && !sign(minNP) && sign(maxPN) && !sign(maxNP))
			// simd_sse2.h:51 clears the sign bit.
			ensure(!sign(absN))
		default:
			// simd_neon.h:84/89 FMIN/FMAX order -0 below +0.
			ensure(sign(minPN) && sign(minNP) && !sign(maxPN) && !sign(maxNP))
			// simd_neon.h:49 vabsq_f32 clears the sign bit.
			ensure(!sign(absN))
		}

		// NaN operands of minW / maxW (headerMin / headerMax cite the header lines).
		nan := float32(math.NaN())
		one := float32(1.0)
		minNaN1, min1NaN := lanesOf(minW(splatW(nan), splatW(one)))[0], lanesOf(minW(splatW(one), splatW(nan)))[0]
		maxNaN1, max1NaN := lanesOf(maxW(splatW(nan), splatW(one)))[0], lanesOf(maxW(splatW(one), splatW(nan)))[0]
		switch laneBackend() {
		case "scalar", "sse2":
			// Comparisons with NaN are false, so both return the second operand.
			ensure(minNaN1 == one && isNaN32(min1NaN) && maxNaN1 == one && isNaN32(max1NaN))
		default:
			// FMIN / FMAX propagate NaN.
			ensure(isNaN32(minNaN1) && isNaN32(min1NaN) && isNaN32(maxNaN1) && isNaN32(max1NaN))
		}
		// NaN compares false.
		ensure(moveMaskW(equalsW(splatW(nan), splatW(nan))) == 0)
		ensure(moveMaskW(lessThanW(splatW(nan), splatW(one))) == 0)
		ensure(moveMaskW(greaterThanW(splatW(nan), splatW(one))) == 0)
	})

	It("LaneTransposeIndex", func() {
		var in [16]float32
		for i := range in {
			in[i] = float32(i) + 0.5
		}
		r0, r1, r2, r3 := loadW(in[0:]), loadW(in[4:]), loadW(in[8:]), loadW(in[12:])
		c0, c1, c2, c3 := transposeW(r0, r1, r2, r3)
		for col, c := range []floatW{c0, c1, c2, c3} {
			expectLanes(fmt.Sprintf("transposeW c%d", col), c, perLane(func(i int) float32 { return in[4*i+col] }))
		}
		t0, t1, t2, t3 := transposeW(c0, c1, c2, c3)
		for row, r := range []floatW{t0, t1, t2, t3} {
			expectLanes(fmt.Sprintf("transposeW twice r%d", row), r, perLane(func(i int) float32 { return in[4*row+i] }))
		}

		// loadW / storeW round-trip, including bit patterns that are not ordinary numbers.
		pattern := [laneCount]float32{
			math.Float32frombits(0xffffffff), math.Float32frombits(0x7fc00001),
			math.Float32frombits(0x80000000), math.Float32frombits(0x00000001),
		}
		var out [laneCount + 2]float32
		storeW(out[1:], loadW(pattern[:]))
		ensure(out[0] == 0 && out[laneCount+1] == 0)
		for i := range laneCount {
			ensure(math.Float32bits(out[1+i]) == math.Float32bits(pattern[i]))
		}

		// laneW / setLaneW round-trip.
		w := setW(1, 2, 3, 4)
		for i := range laneCount {
			ensure(laneW(w, i) == float32(i+1))
		}
		for i := range laneCount {
			setLaneW(&w, i, float32(10*i)-5)
		}
		expectLanes("setLaneW", w, [laneCount]float32{-5, 5, 15, 25})
		setLaneW(&w, 2, pattern[1])
		ensure(math.Float32bits(laneW(w, 2)) == 0x7fc00001)
		expectLanes("zeroW", zeroW(), [laneCount]float32{})
		expectLanes("splatW", splatW(-1.25), [laneCount]float32{-1.25, -1.25, -1.25, -1.25})

		// embedIndexW replaces the low bitCount mantissa bits with baseIndex + lane; minIndexW
		// recovers the index of the minimum lane.
		values := setW(4.0, 3.0, 2.0, 5.0)
		e := embedIndexW(values, 8, 4)
		for i := range laneCount {
			bits := math.Float32bits(lanesOf(values)[i])
			ensure(math.Float32bits(laneW(e, i)) == (bits&^0xf)|uint32(8+i))
		}
		ensure(minIndexW(e, 4) == 10)
		for minLane := range laneCount {
			v := perLane(func(i int) float32 {
				if i == minLane {
					return 0.75
				}
				return 1.0 + float32(i)
			})
			ensure(minIndexW(embedIndexW(wOf(v), 16, 5), 5) == 16+minLane)
		}
		// Ties fall to the lower index.
		ensure(minIndexW(embedIndexW(splatW(2.0), 4, 3), 3) == 4)
	})
})
