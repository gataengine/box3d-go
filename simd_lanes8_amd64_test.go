// Go-only test: upstream has no test file for the lane layer (src/simd_avx2.h).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build box3d_simd && goexperiment.simd && amd64 && !box3d_no8

package box3d

import (
	"fmt"
	"math"
	"simd/archsimd"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

// The x8 lane backend (simd_x8_amd64.go) mirrors simd_avx2.h, whose ops are the 256 bit forms of
// simd_sse2.h's: each x8 op must equal the x4 op (simd_x4.go, simd_x4_amd64.go) on the low and
// the high four lanes, bit for bit, including signed zeros and mask bits. The one exception is
// the payload of an arithmetic NaN made from two NaN operands: x86 returns the first source's,
// and the compiler may swap the operands of a commutative VADDPS / VMULPS differently in the
// two functions, so there both lanes only have to be NaN.

// lanes8Of returns the lanes of a.
func lanes8Of(a floatW8) [laneCount8]float32 {
	var out [laneCount8]float32
	storeW8(out[:], a)
	return out
}

// halvesOf splits eight lanes into the x4 values of the low and high halves.
func halvesOf(l [laneCount8]float32) (lo, hi floatW) {
	return loadW(l[0:4]), loadW(l[4:8])
}

// set8Of builds a floatW8 with setW8 from lanes.
func set8Of(l [laneCount8]float32) floatW8 {
	return setW8(l[0], l[1], l[2], l[3], l[4], l[5], l[6], l[7])
}

// expectHalves checks got against the x4 results lo and hi bit for bit.
func expectHalves(op string, got floatW8, lo, hi floatW) {
	GinkgoHelper()
	checkHalves(op, got, lo, hi, false)
}

// expectArithHalves is expectHalves for an arithmetic op: a NaN lane only requires a NaN.
func expectArithHalves(op string, got floatW8, lo, hi floatW) {
	GinkgoHelper()
	checkHalves(op, got, lo, hi, true)
}

func checkHalves(op string, got floatW8, lo, hi floatW, anyNaN bool) {
	GinkgoHelper()
	g := lanes8Of(got)
	want := [2][laneCount]float32{lanesOf(lo), lanesOf(hi)}
	for i := range laneCount8 {
		w := want[i/4][i%4]
		if anyNaN && isNaN32(w) {
			Expect(isNaN32(g[i])).To(BeTrue(), "%s lane %d: got %v (%#08x), want NaN", op, i, g[i], math.Float32bits(g[i]))
			continue
		}
		Expect(math.Float32bits(g[i])).To(Equal(math.Float32bits(w)),
			"%s lane %d: got %v (%#08x), want %v (%#08x)", op, i, g[i], math.Float32bits(g[i]), w, math.Float32bits(w))
	}
}

// lane8Rows are 8-lane operand rows: ordinary values, then every ordered pair of edge values
// (signed zeros, a denormal, MaxFloat32, infinities) and NaNs with distinct payloads.
func lane8Rows() [][laneCount8]float32 {
	var rows [][laneCount8]float32
	for _, row := range laneArithmeticRows {
		rows = append(rows, [laneCount8]float32(append(row[0][:], row[1][:]...)))
		rows = append(rows, [laneCount8]float32(append(row[2][:], row[0][:]...)))
	}
	edges := append(append([]float32{}, laneEdgeValues...), math.Float32frombits(0x7fc00001),
		math.Float32frombits(0xffc00002), 1.5, -2.5)
	var flat []float32
	for _, x := range edges {
		for _, y := range edges {
			flat = append(flat, x, y)
		}
	}
	for len(flat)%laneCount8 != 0 {
		flat = append(flat, 1.0)
	}
	for i := 0; i < len(flat); i += laneCount8 {
		rows = append(rows, [laneCount8]float32(flat[i:i+laneCount8]))
	}
	return rows
}

// checkOps8 compares every x8 lane op on (a, b, c) with the x4 op on each half.
func checkOps8(a, b, c [laneCount8]float32) {
	GinkgoHelper()
	wa, wb, wc := set8Of(a), set8Of(b), set8Of(c)
	alo, ahi := halvesOf(a)
	blo, bhi := halvesOf(b)
	clo, chi := halvesOf(c)

	binary := []struct {
		name  string
		w8    func(x, y floatW8) floatW8
		w4    func(x, y floatW) floatW
		arith bool
	}{
		{"addW", addW8, addW, true}, {"subW", subW8, subW, true}, {"mulW", mulW8, mulW, true},
		{"divW", divW8, divW, true}, {"minW", minW8, minW, false}, {"maxW", maxW8, maxW, false},
		{"symClampW", symClampW8, symClampW, false}, {"andW", andW8, andW, false}, {"orW", orW8, orW, false},
		{"andNotW", andNotW8, andNotW, false}, {"greaterThanW", greaterThanW8, greaterThanW, false},
		{"lessThanW", lessThanW8, lessThanW, false}, {"equalsW", equalsW8, equalsW, false},
	}
	for _, op := range binary {
		checkHalves(op.name, op.w8(wa, wb), op.w4(alo, blo), op.w4(ahi, bhi), op.arith)
		checkHalves(op.name+" swapped", op.w8(wb, wa), op.w4(blo, alo), op.w4(bhi, ahi), op.arith)
	}

	unary := []struct {
		name string
		w8   func(x floatW8) floatW8
		w4   func(x floatW) floatW
	}{
		{"negW", negW8, negW}, {"absW", absW8, absW}, {"sqrtW", sqrtW8, sqrtW},
	}
	for _, op := range unary {
		expectHalves(op.name, op.w8(wa), op.w4(alo), op.w4(ahi))
	}

	expectArithHalves("mulAddW", mulAddW8(wa, wb, wc), mulAddW(alo, blo, clo), mulAddW(ahi, bhi, chi))

	for _, mask := range []struct {
		name   string
		w8     floatW8
		lo, hi floatW
	}{
		{"greaterThanW", greaterThanW8(wa, wb), greaterThanW(alo, blo), greaterThanW(ahi, bhi)},
		{"lessThanW", lessThanW8(wa, wc), lessThanW(alo, clo), lessThanW(ahi, chi)},
		{"equalsW", equalsW8(wa, wa), equalsW(alo, alo), equalsW(ahi, ahi)},
		{"raw bits", wa, alo, ahi},
	} {
		lo, hi := moveMaskW(mask.lo), moveMaskW(mask.hi)
		Expect(moveMaskW8(mask.w8)).To(Equal(lo|hi<<4), "moveMaskW(%s)", mask.name)
		Expect(anyTrueW8(mask.w8)).To(Equal(anyTrueW(mask.lo) || anyTrueW(mask.hi)), "anyTrueW(%s)", mask.name)
		Expect(allZeroW8(mask.w8)).To(Equal(allZeroW(mask.lo) && allZeroW(mask.hi)), "allZeroW(%s)", mask.name)
		expectHalves("blendW("+mask.name+")", blendW8(wb, wc, mask.w8), blendW(blo, clo, mask.lo), blendW(bhi, chi, mask.hi))
	}
}

var _ = Describe("SIMDLanes8", func() {
	BeforeEach(func() {
		if !archsimd.X86.AVX2() {
			Skip("the x8 lane backend needs AVX2")
		}
	})

	It("LaneArithmetic8", func() {
		rows := lane8Rows()
		for i := range rows {
			checkOps8(rows[i], rows[(i+1)%len(rows)], rows[(i+2)%len(rows)])
		}

		// The product rounds before the add (no fused multiply-add): see LaneArithmetic.
		e := float32(1.0 / 4096.0)
		ensure(laneW8(mulAddW8(splatW8(-(1+2*e)), splatW8(1+e), splatW8(1+e)), 5) == 0)

		// Signed zeros and NaN operands of minW8 / maxW8 follow _mm256_min_ps / _mm256_max_ps:
		// the second operand when the compare is false.
		pz, nz := float32(0.0), float32(math.Copysign(0, -1))
		nan, one := float32(math.NaN()), float32(1.0)
		sign := func(x float32) bool { return math.Signbit(float64(x)) }
		ensure(sign(laneW8(minW8(splatW8(pz), splatW8(nz)), 7)) && !sign(laneW8(minW8(splatW8(nz), splatW8(pz)), 7)))
		ensure(sign(laneW8(maxW8(splatW8(pz), splatW8(nz)), 7)) && !sign(laneW8(maxW8(splatW8(nz), splatW8(pz)), 7)))
		ensure(laneW8(minW8(splatW8(nan), splatW8(one)), 6) == one && isNaN32(laneW8(minW8(splatW8(one), splatW8(nan)), 6)))
		ensure(laneW8(maxW8(splatW8(nan), splatW8(one)), 6) == one && isNaN32(laneW8(maxW8(splatW8(one), splatW8(nan)), 6)))
	})

	It("LaneMasks8", func() {
		rows := lane8Rows()
		for i := range rows {
			checkOps8(rows[i], rows[i], rows[(i+3)%len(rows)])
		}

		a := setW8(1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 7.0, -8.0)
		b := setW8(2.0, -3.0, 3.0, 0.0, 5.0, -7.0, 8.0, -8.0)
		ensure(moveMaskW8(greaterThanW8(a, b)) == 0b00100010)
		ensure(moveMaskW8(lessThanW8(a, b)) == 0b01001001)
		ensure(moveMaskW8(equalsW8(a, b)) == 0b10010100)
		ensure(allZeroW8(zeroW8()) && !anyTrueW8(zeroW8()))
		ensure(allZeroW8(splatW8(float32(math.Copysign(0, -1)))))
		ensure(!allZeroW8(setW8(0, 0, 0, 0, 0, 0, 0, math.Float32frombits(1))))

		// softMaskW8 is softMaskW on each half.
		indexA := []int32{0, 3, 0, 7, 1, 0, 9, 4}
		indexB := []int32{5, 0, 0, 2, 0, 6, 8, 3}
		expectHalves("softMaskW", softMaskW8(indexA, indexB), softMaskW(indexA[0:4], indexB[0:4]), softMaskW(indexA[4:8], indexB[4:8]))
		ensure(moveMaskW8(softMaskW8(indexA, indexB)) == 0b00110111)
		ensure(moveMaskW8(softMaskW8([]int32{1, 2, 3, 4, 5, 6, 7, 8}, []int32{8, 7, 6, 5, 4, 3, 2, 1})) == 0)
	})

	It("LaneTransposeIndex8", func() {
		var in [64]float32
		for i := range in {
			in[i] = float32(i) + 0.5
		}
		var r [laneCount8]floatW8
		for i := range r {
			r[i] = loadW8(in[8*i:])
		}
		c := [laneCount8]floatW8{}
		c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7] = transposeW8(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7])
		for col := range laneCount8 {
			got := lanes8Of(c[col])
			for row := range laneCount8 {
				ensure(got[row] == in[8*row+col])
			}
		}
		t := [laneCount8]floatW8{}
		t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7] = transposeW8(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7])
		for row := range laneCount8 {
			ensure(lanes8Of(t[row]) == [laneCount8]float32(in[8*row:8*row+8]))
		}

		// loadW8 / storeW8 / setW8 / laneW8 / setLaneW8 move bits unchanged.
		pattern := [laneCount8]float32{
			math.Float32frombits(0xffffffff), math.Float32frombits(0x7fc00001),
			math.Float32frombits(0x80000000), math.Float32frombits(0x00000001),
			math.Float32frombits(0xff800000), math.Float32frombits(0x7f7fffff),
			math.Float32frombits(0x00800000), math.Float32frombits(0xbf800000),
		}
		var out [laneCount8 + 2]float32
		storeW8(out[1:], loadW8(pattern[:]))
		ensure(out[0] == 0 && out[laneCount8+1] == 0)
		w := set8Of(pattern)
		for i := range laneCount8 {
			ensure(math.Float32bits(out[1+i]) == math.Float32bits(pattern[i]))
			ensure(math.Float32bits(laneW8(w, i)) == math.Float32bits(pattern[i]))
		}
		for i := range laneCount8 {
			setLaneW8(&w, i, float32(10*i)-5)
		}
		ensure(lanes8Of(w) == [laneCount8]float32{-5, 5, 15, 25, 35, 45, 55, 65})
		ensure(lanes8Of(zeroW8()) == [laneCount8]float32{})
		ensure(lanes8Of(splatW8(-1.25)) == [laneCount8]float32{-1.25, -1.25, -1.25, -1.25, -1.25, -1.25, -1.25, -1.25})

		// embedIndexW8 is embedIndexW on each half (the high half with baseIndex + 4); minIndexW8
		// recovers the index of the minimum lane, ties to the lower index.
		values := setW8(4.0, 3.0, 2.0, 5.0, 6.0, 2.5, 7.0, 8.0)
		lo, hi := halvesOf(lanes8Of(values))
		expectHalves("embedIndexW", embedIndexW8(values, 8, 4), embedIndexW(lo, 8, 4), embedIndexW(hi, 12, 4))
		ensure(minIndexW8(embedIndexW8(values, 8, 4), 4) == 10)
		for minLane := range laneCount8 {
			var v [laneCount8]float32
			for i := range v {
				v[i] = 1.0 + float32(i)
			}
			v[minLane] = 0.75
			ensure(minIndexW8(embedIndexW8(set8Of(v), 16, 5), 5) == 16+minLane)
			v[(minLane+5)%laneCount8] = 0.75
			ensure(minIndexW8(embedIndexW8(set8Of(v), 16, 5), 5) == 16+min(minLane, (minLane+5)%laneCount8))
		}
		ensure(minIndexW8(embedIndexW8(splatW8(2.0), 4, 3), 3) == 4)
	})

	// The width 8 helpers of wide_lanes8_amd64.go gather and scatter the same values as the width
	// 4 helpers of wide_lanes.go on each half of the lanes.
	It("WideLanes8", func() {
		saved := simdNone
		simdNone = false
		defer func() { simdNone = saved }()

		seed := uint32(31415)
		next := func() float32 {
			seed = seed*1664525 + 1013904223
			return float32(int32(seed>>8)-(1<<23)) / float32(1<<20)
		}
		vec := func() Vec3 { return Vec3{next(), next(), next()} }

		states := make([]bodyState, 10)
		for i := range states {
			states[i] = bodyState{linearVelocity: vec(), padding1: next(), angularVelocity: vec(), padding2: next(),
				deltaPosition: vec(), flags: uint32(i%3) * dynamicFlag, deltaRotation: Quat{vec(), next()}}
		}
		indices := [laneCount8]int32{3, 0, 1, 10, 7, 0, 5, 2}
		lo4, hi4 := [laneCount]int32(indices[0:4]), [laneCount]int32(indices[4:8])

		expectVec := func(op string, got vec3W8, lo, hi vec3W) {
			GinkgoHelper()
			expectHalves(op+".X", got.X, lo.X, hi.X)
			expectHalves(op+".Y", got.Y, lo.Y, hi.Y)
			expectHalves(op+".Z", got.Z, lo.Z, hi.Z)
		}

		s8 := gatherBodies8(states, &indices)
		slo, shi := gatherBodies(states, &lo4), gatherBodies(states, &hi4)
		expectVec("gatherBodies v", s8.v, slo.v, shi.v)
		expectVec("gatherBodies w", s8.w, slo.w, shi.w)
		expectVec("gatherBodies dp", s8.dp, slo.dp, shi.dp)
		expectVec("gatherBodies dq.V", s8.dq.V, slo.dq.V, shi.dq.V)
		expectHalves("gatherBodies dq.S", s8.dq.S, slo.dq.S, shi.dq.S)

		// Scatter new velocities from both widths into copies of the states.
		for _, f := range []*floatW8{&s8.v.X, &s8.v.Y, &s8.v.Z, &s8.w.X, &s8.w.Y, &s8.w.Z} {
			*f = addW8(*f, splatW8(next()))
		}
		lo := func(f floatW8) floatW { l, _ := halvesOf(lanes8Of(f)); return l }
		hi := func(f floatW8) floatW { _, h := halvesOf(lanes8Of(f)); return h }
		slo.v, shi.v = vec3W{lo(s8.v.X), lo(s8.v.Y), lo(s8.v.Z)}, vec3W{hi(s8.v.X), hi(s8.v.Y), hi(s8.v.Z)}
		slo.w, shi.w = vec3W{lo(s8.w.X), lo(s8.w.Y), lo(s8.w.Z)}, vec3W{hi(s8.w.X), hi(s8.w.Y), hi(s8.w.Z)}
		states8 := append([]bodyState{}, states...)
		states4 := append([]bodyState{}, states...)
		scatterBodies8(states8, &indices, &s8)
		scatterBodies(states4, &lo4, &slo)
		scatterBodies(states4, &hi4, &shi)
		for i := range states {
			Expect(scenesBytesOf(&states8[i])).To(Equal(scenesBytesOf(&states4[i])), "scatterBodies state %d", i)
		}

		var sims [laneCount8]bodySim
		var simLanes [laneCount8]*bodySim
		var contacts [laneCount8]contact
		var contactLanes [laneCount8]*contact
		var manifolds [laneCount8]Manifold
		var manifoldLanes [laneCount8]*Manifold
		var counts [laneCount8]int32
		for i := range laneCount8 {
			sims[i].invMass = next()
			sims[i].invInertiaWorld = Matrix3{vec(), vec(), vec()}
			simLanes[i] = &sims[i]
			contacts[i].friction, contacts[i].rollingResistance, contacts[i].restitution = next(), next(), next()
			contacts[i].tangentVelocity = vec()
			contactLanes[i] = &contacts[i]
			m := &manifolds[i]
			m.Normal, m.TwistImpulse, m.FrictionImpulse, m.RollingImpulse = vec(), next(), vec(), vec()
			for p := range m.Points {
				m.Points[p].AnchorA, m.Points[p].AnchorB = vec(), vec()
				m.Points[p].Separation, m.Points[p].NormalImpulse = next(), next()
			}
			manifoldLanes[i] = m
			counts[i] = int32(i*7%5) - 1
		}
		simLo, simHi := [laneCount]*bodySim(simLanes[0:4]), [laneCount]*bodySim(simLanes[4:8])
		conLo, conHi := [laneCount]*contact(contactLanes[0:4]), [laneCount]*contact(contactLanes[4:8])
		manLo, manHi := [laneCount]*Manifold(manifoldLanes[0:4]), [laneCount]*Manifold(manifoldLanes[4:8])
		cntLo, cntHi := [laneCount]int32(counts[0:4]), [laneCount]int32(counts[4:8])

		expectHalves("gatherInvMassW", gatherInvMassW8(&simLanes), gatherInvMassW(&simLo), gatherInvMassW(&simHi))
		expectHalves("gatherFrictionW", gatherFrictionW8(&contactLanes), gatherFrictionW(&conLo), gatherFrictionW(&conHi))
		expectHalves("gatherRollingResistanceW", gatherRollingResistanceW8(&contactLanes),
			gatherRollingResistanceW(&conLo), gatherRollingResistanceW(&conHi))
		expectHalves("gatherRestitutionW", gatherRestitutionW8(&contactLanes), gatherRestitutionW(&conLo), gatherRestitutionW(&conHi))
		expectVec("gatherTangentVelocityW", gatherTangentVelocityW8(&contactLanes),
			gatherTangentVelocityW(&conLo), gatherTangentVelocityW(&conHi))
		expectHalves("intsToFloatW", intsToFloatW8(&counts), intsToFloatW(&cntLo), intsToFloatW(&cntHi))

		i8, ilo, ihi := gatherInvInertiaW8(&simLanes), gatherInvInertiaW(&simLo), gatherInvInertiaW(&simHi)
		expectHalves("gatherInvInertiaW cxx", i8.cxx, ilo.cxx, ihi.cxx)
		expectHalves("gatherInvInertiaW cxy", i8.cxy, ilo.cxy, ihi.cxy)
		expectHalves("gatherInvInertiaW cxz", i8.cxz, ilo.cxz, ihi.cxz)
		expectHalves("gatherInvInertiaW cyy", i8.cyy, ilo.cyy, ihi.cyy)
		expectHalves("gatherInvInertiaW cyz", i8.cyz, ilo.cyz, ihi.cyz)
		expectHalves("gatherInvInertiaW czz", i8.czz, ilo.czz, ihi.czz)

		n8, t8, f8, r8 := gatherManifoldHeadW8(&manifoldLanes)
		nlo, tlo, flo, rlo := gatherManifoldHeadW(&manLo)
		nhi, thi, fhi, rhi := gatherManifoldHeadW(&manHi)
		expectVec("gatherManifoldHeadW normal", n8, nlo, nhi)
		expectHalves("gatherManifoldHeadW twistImpulse", t8, tlo, thi)
		expectVec("gatherManifoldHeadW frictionImpulse", f8, flo, fhi)
		expectVec("gatherManifoldHeadW rollingImpulse", r8, rlo, rhi)

		for pointIndex := range MaxManifoldPoints {
			a8, b8, s8, ni8 := gatherManifoldPointW8(&manifoldLanes, pointIndex)
			alo, blo, sl, nilo := gatherManifoldPointW(&manLo, pointIndex)
			ahi, bhi, sh, nihi := gatherManifoldPointW(&manHi, pointIndex)
			op := fmt.Sprintf("gatherManifoldPointW %d", pointIndex)
			expectVec(op+" rA", a8, alo, ahi)
			expectVec(op+" rB", b8, blo, bhi)
			expectHalves(op+" separation", s8, sl, sh)
			expectHalves(op+" normalImpulse", ni8, nilo, nihi)
		}
	})
})
