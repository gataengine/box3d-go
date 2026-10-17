// Port of src/arena_allocator.h and src/arena_allocator.c from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "unsafe"

// PORT: the stack and arena allocators are thin bookkeeping shells (spec: allocators dropped).
// Allocations are typed Go slices from make, so the memory is zeroed and owned by the GC. The
// index, capacity, watermark and overflow accounting is kept exactly so that the counters
// upstream reports (stack used, arena capacity) and the grow decisions match. stackAlloc and
// bump take an element count and a type instead of a byte size.

// ---- arena_allocator.h ----

const maxStackEntries = 32 // B3_MAX_STACK_ENTRIES

// stackEntry is b3StackEntry.
type stackEntry struct {
	data       unsafe.Pointer
	name       string
	size       int
	usedMalloc bool
}

// stack is b3Stack. This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will assert
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
type stack struct {
	capacity int
	index    int

	allocation    int
	maxAllocation int

	entries    [maxStackEntries]stackEntry
	entryCount int
}

// overflowBlock is b3OverflowBlock: heap-allocated fallback block tracked when an arena bump overflows.
type overflowBlock struct {
	size int
}

// arenaSharedState is b3ArenaSharedState: shared, heap-allocated state co-owned by every copy of an arena.
// arena is passed by value so its bump pointer auto-restores on
// function return, but overflow tracking and watermarks must persist
// across copies -- hence this pointer-shared block.
type arenaSharedState struct {
	overflows     []overflowBlock
	maxIndex      int // high water mark of the bump pointer this step
	overflowBytes int // total bytes in overflow blocks this step
	peakDemand    int // all-time peak of (maxIndex + overflowBytes), survives sync
}

// arena is b3Arena.
type arena struct {
	capacity int
	index    int
	shared   *arenaSharedState
}

// arenaAlignment is B3_ARENA_ALIGNMENT: 16-byte alignment for SSE2 + typical struct alignment.
const arenaAlignment = 16

// bump is b3Bump: count elements of T from the arena.
func bump[T any](arena *arena, count int) []T {
	var zero T
	size := count * int(unsafe.Sizeof(zero))
	if size == 0 {
		return nil
	}

	aligned := (arena.index + (arenaAlignment - 1)) &^ (arenaAlignment - 1)

	if aligned+size > arena.capacity {
		return arenaOverflowAlloc[T](arena, count)
	}

	arena.index = aligned + size
	if arena.index > arena.shared.maxIndex {
		arena.shared.maxIndex = arena.index
	}
	return make([]T, count)
}

// ---- arena_allocator.c ----

func createStack(capacity int) stack {
	assert(capacity >= 0, "capacity >= 0")
	var stack stack
	stack.capacity = capacity
	return stack
}

func destroyStack(stack *stack) {
	stack.capacity = 0
}

// stackAlloc is b3StackAlloc: count elements of T.
func stackAlloc[T any](stack *stack, count int, name string) []T {
	if stack.entryCount == maxStackEntries {
		assert(false, "false")
		return nil
	}

	var zero T
	size := count * int(unsafe.Sizeof(zero))
	alignedSize := ((size - 1) | (alignment - 1)) + 1

	data := make([]T, count)

	var entry stackEntry
	entry.size = alignedSize
	entry.name = name
	entry.data = unsafe.Pointer(unsafe.SliceData(data))
	if stack.index+alignedSize > stack.capacity {
		// fall back to the heap (undesirable)
		entry.usedMalloc = true
	} else {
		entry.usedMalloc = false
		stack.index += alignedSize
	}

	stack.allocation += alignedSize
	if stack.allocation > stack.maxAllocation {
		stack.maxAllocation = stack.allocation
	}

	stack.entries[stack.entryCount] = entry
	stack.entryCount += 1
	return data
}

// stackFree is b3StackFree.
func stackFree[T any](stack *stack, mem []T) {
	entryCount := stack.entryCount
	assert(entryCount > 0, "entryCount > 0")
	entry := &stack.entries[entryCount-1]
	assert(unsafe.Pointer(unsafe.SliceData(mem)) == entry.data, "mem == entry->data")
	if entry.usedMalloc == false {
		stack.index -= entry.size
	}
	stack.allocation -= entry.size
	stack.entryCount -= 1
}

// growStack grows the stack based on usage
func growStack(stack *stack) {
	// Stack must not be in use
	assert(stack.allocation == 0, "stack->allocation == 0")

	if stack.maxAllocation > stack.capacity {
		stack.capacity = stack.maxAllocation + stack.maxAllocation/2
	}
}

func getStackCapacity(stack *stack) int {
	return stack.capacity
}

func getStackAllocation(stack *stack) int {
	return stack.allocation
}

func getMaxStackAllocation(stack *stack) int {
	return stack.maxAllocation
}

func createArena(capacity int) arena {
	c := capacity
	if c <= 8 {
		c = 8
	}
	shared := &arenaSharedState{}

	return arena{
		capacity: c,
		index:    0,
		shared:   shared,
	}
}

func destroyArena(arena_ *arena) {
	*arena_ = arena{}
}

// arenaOverflowAlloc is b3ArenaOverflowAlloc: heap-allocate an overflow block, register it in the
// shared state, return it.
func arenaOverflowAlloc[T any](arena *arena, count int) []T {
	var zero T
	size := count * int(unsafe.Sizeof(zero))
	shared := arena.shared
	data := make([]T, count)
	block := overflowBlock{size}
	arrayPush(&shared.overflows, block)
	shared.overflowBytes += size
	return data
}

// arenaSync is b3ArenaSync. Call between simulation steps. Frees this step's overflow blocks and grows
// the backing capacity if last step's demand (maxIndex + overflowBytes) exceeded it.
func arenaSync(arena *arena) {
	shared := arena.shared

	arrayClear(&shared.overflows)

	demand := shared.maxIndex + shared.overflowBytes
	if demand > shared.peakDemand {
		shared.peakDemand = demand
	}
	if demand > arena.capacity {
		newCapacity := demand + demand/2
		arena.capacity = newCapacity
	}

	arena.index = 0
	shared.maxIndex = 0
	shared.overflowBytes = 0
}
