// Port of src/arena_allocator.h and src/arena_allocator.c from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"reflect"
	"sync"
	"unsafe"
)

// typedPool is the backing slice of one element type.
type typedPool struct {
	typ  reflect.Type
	data any // []T
}

// typedPools holds one backing slice per element type.
type typedPools struct {
	pools []typedPool
}

// poolAlloc returns count zeroed elements of T at byte offset offset of the upstream buffer.
func poolAlloc[T any](ps *typedPools, offset, count int) []T {
	typ := reflect.TypeFor[T]()
	index := -1
	var buf []T
	for i := range ps.pools {
		if ps.pools[i].typ == typ {
			index = i
			buf = ps.pools[i].data.([]T)
			break
		}
	}

	elementSize := int(typ.Size())
	start := offset
	if elementSize > 1 {
		start = (offset + elementSize - 1) / elementSize
	}
	end := start + count

	if end > len(buf) || buf == nil {
		n := MaxInt(end+end/2, 1)
		buf = make([]T, n)
		if index < 0 {
			ps.pools = append(ps.pools, typedPool{typ, buf})
		} else {
			ps.pools[index].data = buf
		}
	}

	data := buf[start:end:end]
	clear(data)
	return data
}

// scratchPool is a pool of *T for C stack locals that Go would move to the heap because a query
// receives them as `any` (callback contexts), in functions that have no per-worker storage at
// hand. PORT: not in upstream. get returns a pooled value (contents unspecified, callers reset
// it) or a new one; put returns it. Once warm, get does not allocate.
type scratchPool[T any] struct {
	pool sync.Pool
}

func (p *scratchPool[T]) get() *T {
	if v, ok := p.pool.Get().(*T); ok {
		return v
	}
	return new(T)
}

func (p *scratchPool[T]) put(v *T) {
	p.pool.Put(v)
}

// clearPools drops every backing slice.
func clearPools(ps *typedPools) {
	ps.pools = nil
}

// PORT: allocations are typed Go slices (stackAlloc and bump take an element count and a type
// instead of a byte size), so pointer-holding element types stay visible to the GC. The index,
// capacity, watermark and overflow accounting is kept exactly so that the counters upstream
// reports (stack used, arena capacity) and the grow decisions match. Upstream's single byte
// buffer becomes one backing slice per element type (typedPools), owned by the stack or by the
// arena's shared state and reused across steps: an allocation at byte offset off of the upstream
// buffer is the element range starting at ceil(off / sizeof(T)) of T's backing slice. Live
// allocations have disjoint byte ranges, so live allocations of one type have disjoint element
// ranges, and memory is reused exactly when upstream reuses it. A backing slice grows (a new
// make; live allocations keep the old one) only when a step reaches a new high water mark, so a
// warm Step does not allocate. Allocations that overflow the capacity use make, as upstream
// falls back to malloc, and the capacity grows between steps as upstream. Returned memory is
// zeroed (upstream does not zero; zeroing keeps the earlier make semantics).

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

	// PORT: the backing memory, one slice per element type.
	pools typedPools
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

	// PORT: the backing memory, one slice per element type.
	pools typedPools
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
	return poolAlloc[T](&arena.shared.pools, aligned, count)
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
	clearPools(&stack.pools)
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

	var data []T

	var entry stackEntry
	entry.size = alignedSize
	entry.name = name
	if stack.index+alignedSize > stack.capacity {
		// fall back to the heap (undesirable)
		data = make([]T, count)
		entry.usedMalloc = true
	} else {
		data = poolAlloc[T](&stack.pools, stack.index, count)
		entry.usedMalloc = false
		stack.index += alignedSize
	}
	entry.data = unsafe.Pointer(unsafe.SliceData(data))

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
