// Go-only test: upstream has no test file for the lane layer (src/simd*.h, src/simd.c).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

var _ = Describe("SIMDTest", func() {
	It("Width", func() {
		ensure(GetSIMDWidth() == laneCount)
		SetSIMDWidth(8)
		ensure(GetSIMDWidth() == 4)
		SetSIMDWidth(0)
		ensure(GetSIMDWidth() == laneCount)
		ensure(IsAVX2Available() == false)
	})

	It("Lanes", func() {
		a := setW(1.0, -2.0, 3.0, -4.0)
		b := splatW(0.5)

		var out [laneCount]float32
		storeW(out[:], mulAddW(a, b, a))
		ensure(out == [laneCount]float32{1.5, -3.0, 4.5, -6.0})

		ensure(moveMaskW(greaterThanW(a, zeroW())) == 0b0101)
		ensure(blendW(a, b, lessThanW(a, zeroW())) == setW(1.0, 0.5, 3.0, 0.5))
		ensure(symClampW(a, splatW(2.5)) == setW(1.0, -2.0, 2.5, -2.5))

		// Index embedding in the low mantissa bits survives a min reduction.
		e := embedIndexW(setW(4.0, 3.0, 2.0, 5.0), 8, 4)
		ensure(minIndexW(e, 4) == 10)

		c0, c1, c2, c3 := transposeW(setW(0, 1, 2, 3), setW(4, 5, 6, 7), setW(8, 9, 10, 11), setW(12, 13, 14, 15))
		ensure(c0 == setW(0, 4, 8, 12))
		ensure(c1 == setW(1, 5, 9, 13))
		ensure(c2 == setW(2, 6, 10, 14))
		ensure(c3 == setW(3, 7, 11, 15))
	})

	It("TriangleTests", func() {
		center := v32{0.0, 0.0, 0.0}
		extent := v32{1.0, 1.0, 1.0}
		ensure(testBoundsTriangleOverlap(center, extent, v32{-2, 0, -2}, v32{2, 0, -2}, v32{0, 0, 2}))
		ensure(testBoundsTriangleOverlap(center, extent, v32{-2, 3, -2}, v32{2, 3, -2}, v32{0, 3, 2}) == false)

		// Ray straight down onto a triangle in the y = 0 plane, counter-clockwise seen from above.
		fraction := intersectRayTriangle(v32{0, 1, 0}, v32{0, -2, 0}, v32{-1, 0, 1}, v32{1, 0, 1}, v32{0, 0, -1})
		ensureSmall(fraction-0.5, fltEpsilon)
	})
})
