// Port of test/test_bitset.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"math"

	. "github.com/onsi/ginkgo/v2"
)

const bitSetTestCount = 169 // COUNT

var _ = Describe("BitTest", func() {
	It("TestBitMath", func() {
		r1 := clz32(9)
		ensure(r1 == 31-3)

		for i := 1; i < 1000; i++ {
			e1 := lowerPowerOf2Exponent(i)
			// PORT: floorf( log2f( (float)i ) ) evaluated in float64.
			e2 := int(math.Floor(math.Log2(float64(float32(i)))))
			ensure(e1 == e2)
		}
	})

	It("TestBitSet", func() {
		bitSet := createBitSet(bitSetTestCount)

		setBitCountAndClear(&bitSet, bitSetTestCount)
		var values [bitSetTestCount]bool

		var i1, i2 int32 = 0, 1
		setBit(&bitSet, uint32(i1))
		values[i1] = true

		for i2 < bitSetTestCount {
			setBit(&bitSet, uint32(i2))
			values[i2] = true
			next := i1 + i2
			i1 = i2
			i2 = next
		}

		for i := int32(0); i < bitSetTestCount; i++ {
			value := getBit(&bitSet, uint32(i))
			ensure(value == values[i])
		}

		destroyBitSet(&bitSet)
	})
})
