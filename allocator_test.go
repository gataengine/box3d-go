// Port of test/test_allocator.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
)

type allocatorFoo struct {
	value1 int32
	value2 float32
}

// testBlockAlignment is the body of the TestBlockAlignment loop for one element type.
// PORT: the C test allocates raw byte sizes (12, sizeof( b3Manifold ), 3 * sizeof( b3Manifold ));
// the Go allocator is typed, so the element is elementSize values of T. The alignment ENSURE
// checks the natural alignment of T (the issue #78 free-list link does not exist in Go).
func testBlockAlignment[T any](elementSize int) {
	allocator := createBlockAllocator[T](elementSize, 0)

	// Hold the elements so the indices advance and a block boundary is crossed
	var elements [blockSize + 8][]T
	count := len(elements)

	var zero T
	align := uintptr(unsafe.Alignof(zero))

	for j := 0; j < count; j++ {
		elements[j] = allocateElement(&allocator)
		ensure(elements[j] != nil)
		ensure((uintptr(unsafe.Pointer(&elements[j][0])) & (align - 1)) == 0)
	}

	for j := 0; j < count; j++ {
		freeElement(&allocator, elements[j])
	}

	// Now drain the free list
	for j := 0; j < count; j++ {
		elements[j] = allocateElement(&allocator)
		ensure(elements[j] != nil)
		ensure((uintptr(unsafe.Pointer(&elements[j][0])) & (align - 1)) == 0)
	}

	for j := 0; j < count; j++ {
		freeElement(&allocator, elements[j])
	}

	destroyBlockAllocator(&allocator)
}

var _ = Describe("AllocatorTest", func() {
	It("TestBlockAllocate", func() {
		allocator := createBlockAllocator[allocatorFoo](1, 2)

		item1 := allocateElement(&allocator)
		ensure(item1 != nil)

		item2 := allocateElement(&allocator)
		ensure(item2 != nil)

		destroyBlockAllocator(&allocator)
	})

	It("TestBlockClear", func() {
		allocator := createBlockAllocator[allocatorFoo](1, 0)

		item1 := allocateElement(&allocator)
		item2 := allocateElement(&allocator)

		freeElement(&allocator, item1)
		freeElement(&allocator, item2)

		destroyBlockAllocator(&allocator)
	})

	// The free list link is a pointer stored in the element, so a stride that is not a multiple
	// of the alignment leaves odd elements on a bad boundary. See issue #78.
	It("TestBlockAlignment", func() {
		// The manifold sizes are the reported case
		testBlockAlignment[byte](12)
		testBlockAlignment[Manifold](1)
		testBlockAlignment[Manifold](3)
	})
})
