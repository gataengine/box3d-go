// Port of src/ctz.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "math/bits"

// PORT: the MSVC and GCC/Clang intrinsic variants collapse into math/bits, which the compiler
// lowers to the same instructions. As upstream, ctz32/ctz64 of zero are not meaningful.

// https://en.wikipedia.org/wiki/Find_first_set

func ctz32(block uint32) uint32 {
	return uint32(bits.TrailingZeros32(block))
}

func clz32(value uint32) uint32 {
	return uint32(bits.LeadingZeros32(value))
}

func ctz64(block uint64) uint32 {
	return uint32(bits.TrailingZeros64(block))
}

func popCount64(block uint64) int {
	return bits.OnesCount64(block)
}

func isPowerOf2(x int) bool {
	return (x & (x - 1)) == 0
}

func boundingPowerOf2(x int) int {
	if x <= 1 {
		return 1
	}

	return 32 - int(clz32(uint32(x)-1))
}

func roundUpPowerOf2(x int) int {
	if x <= 1 {
		return 1
	}

	return 1 << (32 - int(clz32(uint32(x)-1)))
}

func lowerPowerOf2Exponent(x int) int {
	assert(x > 0, "x > 0")
	clz := int(clz32(uint32(x)))

	// Position of most significant bit = floor(log2(M))
	return 31 - clz
}
