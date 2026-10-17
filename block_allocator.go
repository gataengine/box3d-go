// Port of src/block_allocator.h and src/block_allocator.c from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: the block allocator hands out fixed-length slices of T (elementCount values per element)
// instead of elementSize bytes. Blocks are Go slices; the intrusive free list (a pointer stored
// in each freed element) becomes a LIFO stack of freed slices, which hands elements back in the
// same order. Element addresses stay stable until freed, as upstream.

// ---- block_allocator.h ----

const (
	blockExponent  = 8                  // B3_BLOCK_EXPONENT
	blockSize      = 1 << blockExponent // B3_BLOCK_SIZE
	blockAlignment = 16                 // B3_BLOCK_ALIGNMENT
)

// block is b3Block.
type block[T any] struct {
	memory []T
}

// blockAllocator is b3BlockAllocator.
type blockAllocator[T any] struct {
	blocks          []block[T]
	freeList        [][]T
	elementSize     int // PORT: element length in T values, not bytes
	nextIndex       int
	allocationCount int
}

// ---- block_allocator.c ----

// createBlockAllocator returns an allocator of elements that are elementSize values of T.
func createBlockAllocator[T any](elementSize int, initialCount int) blockAllocator[T] {
	assert(elementSize >= 1, "elementSize >= (int)sizeof( void* )")

	// PORT: no rounding of the stride to B3_BLOCK_ALIGNMENT; the free list does not live in the
	// elements, so Go's natural alignment of T is enough.

	var allocator blockAllocator[T]

	arrayCreate(&allocator.blocks)
	allocator.elementSize = elementSize
	allocator.freeList = nil
	allocator.nextIndex = 0
	allocator.allocationCount = 0

	if initialCount > 0 {
		count := (initialCount + blockSize - 1) >> blockExponent
		arrayResize(&allocator.blocks, count)

		for i := 0; i < len(allocator.blocks); i++ {
			allocator.blocks[i].memory = make([]T, blockSize*elementSize)
		}
	}

	return allocator
}

func destroyBlockAllocator[T any](allocator *blockAllocator[T]) {
	arrayDestroy(&allocator.blocks)
	allocator.freeList = nil
}

// allocateElement returns one element of elementSize contiguous values. Address is stable until freed.
func allocateElement[T any](allocator *blockAllocator[T]) []T {
	assert(allocator != nil, "allocator != NULL")

	allocator.allocationCount += 1

	// Pop from free list first
	if len(allocator.freeList) > 0 {
		element := arrayPop(&allocator.freeList)
		return element
	}

	index := allocator.nextIndex
	allocator.nextIndex++

	requiredBlockCount := (index >> blockExponent) + 1

	if requiredBlockCount > len(allocator.blocks) {
		oldCount := len(allocator.blocks)
		arrayResize(&allocator.blocks, requiredBlockCount)

		for i := oldCount; i < requiredBlockCount; i++ {
			allocator.blocks[i].memory = make([]T, blockSize*allocator.elementSize)
		}
	}

	blockIndex := index >> blockExponent
	blockOffset := index & (blockSize - 1)

	start := blockOffset * allocator.elementSize
	end := start + allocator.elementSize
	return allocator.blocks[blockIndex].memory[start:end:end]
}

func freeElement[T any](allocator *blockAllocator[T], element []T) {
	assert(allocator != nil, "allocator != NULL")
	assert(element != nil, "element != NULL")
	assert(allocator.allocationCount > 0, "allocator->allocationCount > 0")

	allocator.allocationCount -= 1

	// Push onto free list
	arrayPush(&allocator.freeList, element)
}
