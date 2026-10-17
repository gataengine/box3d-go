// Port of src/bitset.h and src/bitset.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- bitset.h ----

// bitSet provides fast operations on large arrays of bits.
// PORT: bits is a slice of length blockCapacity; blockCapacity and blockCount are kept.
type bitSet struct {
	bits          []uint64
	blockCapacity uint32
	blockCount    uint32
}

// countSetBits is implemented in table.go, as upstream.

func setBit(bitSet *bitSet, bitIndex uint32) {
	blockIndex := bitIndex / 64
	assert(blockIndex < bitSet.blockCount, "blockIndex < bitSet->blockCount")
	bitSet.bits[blockIndex] |= uint64(1) << (bitIndex % 64)
}

func setBitGrow(bitSet *bitSet, bitIndex uint32) {
	blockIndex := bitIndex / 64
	if blockIndex >= bitSet.blockCount {
		growBitSet(bitSet, blockIndex+1)
	}
	bitSet.bits[blockIndex] |= uint64(1) << (bitIndex % 64)
}

func clearBit(bitSet *bitSet, bitIndex uint32) {
	blockIndex := bitIndex / 64
	if blockIndex >= bitSet.blockCount {
		return
	}
	bitSet.bits[blockIndex] &^= uint64(1) << (bitIndex % 64)
}

func getBit(bitSet *bitSet, bitIndex uint32) bool {
	blockIndex := bitIndex / 64
	if blockIndex >= bitSet.blockCount {
		return false
	}
	return (bitSet.bits[blockIndex] & (uint64(1) << (bitIndex % 64))) != 0
}

func getBitSetBytes(bitSet *bitSet) int {
	return int(bitSet.blockCapacity) * 8
}

// ---- bitset.c ----

func createBitSet(bitCapacity uint32) bitSet {
	var bitSet bitSet
	bitSet.blockCapacity = (bitCapacity + 8*8 - 1) / (8 * 8)
	bitSet.blockCount = 0
	bitSet.bits = make([]uint64, bitSet.blockCapacity)
	return bitSet
}

func destroyBitSet(bitSet *bitSet) {
	bitSet.blockCapacity = 0
	bitSet.blockCount = 0
	bitSet.bits = nil
}

func setBitCountAndClear(bitSet *bitSet, bitCount uint32) {
	blockCount := (bitCount + 8*8 - 1) / (8 * 8)
	if bitSet.blockCapacity < blockCount {
		destroyBitSet(bitSet)
		newBitCapacity := bitCount + (bitCount >> 1)
		*bitSet = createBitSet(newBitCapacity)
	}

	bitSet.blockCount = blockCount
	clear(bitSet.bits[:bitSet.blockCount])
}

func growBitSet(bitSet *bitSet, blockCount uint32) {
	assert(blockCount > bitSet.blockCount, "blockCount > bitSet->blockCount")
	if blockCount > bitSet.blockCapacity {
		oldCapacity := bitSet.blockCapacity
		bitSet.blockCapacity = blockCount + blockCount/2
		newBits := make([]uint64, bitSet.blockCapacity)
		assert(bitSet.bits != nil, "bitSet->bits != NULL")
		copy(newBits, bitSet.bits[:oldCapacity])
		bitSet.bits = newBits
	}

	bitSet.blockCount = blockCount
}

func inPlaceUnion(setA *bitSet, setB *bitSet) {
	assert(setA.blockCount == setB.blockCount, "setA->blockCount == setB->blockCount")
	blockCount := setA.blockCount
	for i := uint32(0); i < blockCount; i++ {
		setA.bits[i] |= setB.bits[i]
	}
}
