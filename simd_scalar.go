// Port of src/simd_scalar.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build !box3d_simd || !goexperiment.simd || !(amd64 || arm64)

package box3d

import "math"

// PORT: upstream picks a lane backend at compile time (simd_neon.h, simd_sse2.h, simd_avx2.h or
// this scalar fallback). The Go port uses this scalar backend in the default build and on
// architectures without a SIMD backend; the box3d_simd build on amd64 and arm64 uses simd_x4.go
// instead, with the same lane API. The box3d_simd build without GOEXPERIMENT=simd also keeps
// this file, so its only compile error is the one of simd_experiment_guard.go. The W4 suffix is
// dropped (b3AddW4 → addW); laneCount is the B3_SIMD_WIDTH of this backend.

// simdBackend names the lane backend of this build ("scalar" or "x4"), for tests whose expected
// values depend on it.
const simdBackend = "scalar"

// laneCount is the lane width of the scalar backend.
const laneCount = 4

// simdNone is B3_SIMD_NONE (core.h): upstream defines it when no SIMD instruction set is used,
// which is the configuration this scalar backend reproduces. Kernels with an
// `#if defined( B3_SIMD_NONE )` variant branch on it.
// PORT: a variable rather than a constant so tests can also run the lane variants of those
// kernels, which upstream requires to match the scalar variants bit for bit.
var simdNone = true

// scalar math

// floatW is b3FloatW4: four float lanes.
type floatW struct {
	v [laneCount]float32
}

// laneW reads lane i of a.
// PORT: Go-only accessor replacing upstream's float pointer views of a b3FloatW4 (for example
// ( (float*)&wide )[lane]). Kernels outside the lane backend use it instead of the representation.
func laneW(a floatW, i int) float32 {
	return a.v[i]
}

// setLaneW writes value to lane i of a.
// PORT: Go-only accessor, the store form of laneW.
func setLaneW(a *floatW, i int, value float32) {
	a.v[i] = value
}

// zeroW is b3ZeroW4.
func zeroW() floatW {
	return floatW{[laneCount]float32{0.0, 0.0, 0.0, 0.0}}
}

// splatW is b3SplatW4.
func splatW(scalar float32) floatW {
	return floatW{[laneCount]float32{scalar, scalar, scalar, scalar}}
}

// setW is b3SetW4.
func setW(a, b, c, d float32) floatW {
	return floatW{[laneCount]float32{a, b, c, d}}
}

// loadW is b3LoadW4.
// PORT: the float pointer becomes a slice of at least laneCount values.
func loadW(data []float32) floatW {
	return floatW{[laneCount]float32{data[0], data[1], data[2], data[3]}}
}

// storeW is b3StoreW4.
func storeW(data []float32, a floatW) {
	data[0] = a.v[0]
	data[1] = a.v[1]
	data[2] = a.v[2]
	data[3] = a.v[3]
}

// negW is b3NegW4.
func negW(a floatW) floatW {
	return floatW{[laneCount]float32{-a.v[0], -a.v[1], -a.v[2], -a.v[3]}}
}

// absW is b3AbsW4.
func absW(a floatW) floatW {
	return floatW{[laneCount]float32{absLane(a.v[0]), absLane(a.v[1]), absLane(a.v[2]), absLane(a.v[3])}}
}

// addW is b3AddW4.
func addW(a, b floatW) floatW {
	return floatW{[laneCount]float32{a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2], a.v[3] + b.v[3]}}
}

// subW is b3SubW4.
func subW(a, b floatW) floatW {
	return floatW{[laneCount]float32{a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2], a.v[3] - b.v[3]}}
}

// mulW is b3MulW4.
func mulW(a, b floatW) floatW {
	return floatW{[laneCount]float32{
		float32(a.v[0] * b.v[0]), float32(a.v[1] * b.v[1]), float32(a.v[2] * b.v[2]), float32(a.v[3] * b.v[3]),
	}}
}

// divW is b3DivW4.
func divW(a, b floatW) floatW {
	return floatW{[laneCount]float32{a.v[0] / b.v[0], a.v[1] / b.v[1], a.v[2] / b.v[2], a.v[3] / b.v[3]}}
}

// sqrtW is b3SqrtW4.
func sqrtW(a floatW) floatW {
	return floatW{[laneCount]float32{sqrtf(a.v[0]), sqrtf(a.v[1]), sqrtf(a.v[2]), sqrtf(a.v[3])}}
}

// mulAddW is b3MulAddW4: a + b * c. The product is rounded before the add, as in every
// upstream backend (none of them fuse).
func mulAddW(a, b, c floatW) floatW {
	return floatW{[laneCount]float32{
		mulAdd(a.v[0], b.v[0], c.v[0]),
		mulAdd(a.v[1], b.v[1], c.v[1]),
		mulAdd(a.v[2], b.v[2], c.v[2]),
		mulAdd(a.v[3], b.v[3], c.v[3]),
	}}
}

// minW is b3MinW4.
func minW(a, b floatW) floatW {
	var r floatW
	for i := range laneCount {
		if a.v[i] <= b.v[i] {
			r.v[i] = a.v[i]
		} else {
			r.v[i] = b.v[i]
		}
	}
	return r
}

// maxW is b3MaxW4.
func maxW(a, b floatW) floatW {
	var r floatW
	for i := range laneCount {
		if a.v[i] >= b.v[i] {
			r.v[i] = a.v[i]
		} else {
			r.v[i] = b.v[i]
		}
	}
	return r
}

// symClampW is b3SymClampW4: clamp a to [-b, b]
func symClampW(a, b floatW) floatW {
	var r floatW
	for i := range laneCount {
		if a.v[i] <= b.v[i] {
			r.v[i] = a.v[i]
		} else {
			r.v[i] = b.v[i]
		}
	}
	for i := range laneCount {
		if r.v[i] <= -b.v[i] {
			r.v[i] = -b.v[i]
		}
	}
	return r
}

// Logical operations on the scalar path are 0/1 float values. Not bit-wise like SIMD.

// boolLane is the ternary c ? 1.0f : 0.0f.
func boolLane(c bool) float32 {
	if c {
		return 1.0
	}
	return 0.0
}

// andW is b3AndW4.
func andW(a, b floatW) floatW {
	var r floatW
	for i := range laneCount {
		r.v[i] = boolLane(a.v[i] != 0.0 && b.v[i] != 0.0)
	}
	return r
}

// orW is b3OrW4.
func orW(a, b floatW) floatW {
	var r floatW
	for i := range laneCount {
		r.v[i] = boolLane(a.v[i] != 0.0 || b.v[i] != 0.0)
	}
	return r
}

// andNotW is b3AndNotW4: a & ~b
func andNotW(a, b floatW) floatW {
	var r floatW
	for i := range laneCount {
		r.v[i] = boolLane(a.v[i] != 0.0 && b.v[i] == 0.0)
	}
	return r
}

// softMaskW is b3SoftMaskW4.
// PORT: the int pointers become slices of at least laneCount values.
func softMaskW(indexA, indexB []int32) floatW {
	var r floatW
	for i := range laneCount {
		r.v[i] = boolLane(indexA[i] == 0 || indexB[i] == 0)
	}
	return r
}

// greaterThanW is b3GreaterThanW4.
func greaterThanW(a, b floatW) floatW {
	var r floatW
	for i := range laneCount {
		r.v[i] = boolLane(a.v[i] > b.v[i])
	}
	return r
}

// lessThanW is b3LessThanW4.
func lessThanW(a, b floatW) floatW {
	var r floatW
	for i := range laneCount {
		r.v[i] = boolLane(a.v[i] < b.v[i])
	}
	return r
}

// equalsW is b3EqualsW4.
func equalsW(a, b floatW) floatW {
	var r floatW
	for i := range laneCount {
		r.v[i] = boolLane(a.v[i] == b.v[i])
	}
	return r
}

// allZeroW is b3AllZeroW4.
func allZeroW(a floatW) bool {
	return a.v[0] == 0.0 && a.v[1] == 0.0 && a.v[2] == 0.0 && a.v[3] == 0.0
}

// anyTrueW is b3AnyTrueW4.
func anyTrueW(mask floatW) bool {
	return mask.v[0] != 0.0 || mask.v[1] != 0.0 || mask.v[2] != 0.0 || mask.v[3] != 0.0
}

// moveMaskW is b3MoveMaskW4.
func moveMaskW(mask floatW) int {
	r := 0
	for i := range laneCount {
		if mask.v[i] != 0.0 {
			r |= 1 << i
		}
	}
	return r
}

// blendW is b3BlendW4: component-wise returns mask ? b : a
func blendW(a, b, mask floatW) floatW {
	var r floatW
	for i := range laneCount {
		if mask.v[i] != 0.0 {
			r.v[i] = b.v[i]
		} else {
			r.v[i] = a.v[i]
		}
	}
	return r
}

// embedIndexW is b3EmbedIndexW4.
func embedIndexW(value floatW, baseIndex, bitCount int) floatW {
	mask := (uint32(1) << bitCount) - 1
	lanes := value.v

	for i := range laneCount {
		bits := math.Float32bits(lanes[i])
		bits = (bits &^ mask) | uint32(baseIndex+i)
		lanes[i] = math.Float32frombits(bits)
	}

	return floatW{lanes}
}

// minIndexW is b3MinIndexW4.
func minIndexW(a floatW, bitCount int) int {
	m := a.v[0]
	if a.v[1] < m {
		m = a.v[1]
	}
	if a.v[2] < m {
		m = a.v[2]
	}
	if a.v[3] < m {
		m = a.v[3]
	}

	bits := math.Float32bits(m)
	return int(bits & ((uint32(1) << bitCount) - 1))
}

// transposeW is b3TransposeW4.
// PORT: the output pointers become return values.
func transposeW(r0, r1, r2, r3 floatW) (c0, c1, c2, c3 floatW) {
	t0 := floatW{[laneCount]float32{r0.v[0], r2.v[0], r0.v[1], r2.v[1]}}
	t1 := floatW{[laneCount]float32{r1.v[0], r3.v[0], r1.v[1], r3.v[1]}}
	t2 := floatW{[laneCount]float32{r0.v[2], r2.v[2], r0.v[3], r2.v[3]}}
	t3 := floatW{[laneCount]float32{r1.v[2], r3.v[2], r1.v[3], r3.v[3]}}
	c0 = floatW{[laneCount]float32{t0.v[0], t1.v[0], t0.v[1], t1.v[1]}}
	c1 = floatW{[laneCount]float32{t0.v[2], t1.v[2], t0.v[3], t1.v[3]}}
	c2 = floatW{[laneCount]float32{t2.v[0], t3.v[0], t2.v[1], t3.v[1]}}
	c3 = floatW{[laneCount]float32{t2.v[2], t3.v[2], t2.v[3], t3.v[3]}}
	return c0, c1, c2, c3
}
