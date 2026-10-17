// Port of src/container.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "unsafe"

// PORT: b3Array( T ) (data, count, capacity) is a Go slice: count is len, capacity is cap.
// b3DeclareArray / b3DeclareArrayNative are not needed. The b3Array_* macros become the generic
// functions below so call sites stay line for line, and they keep upstream's growth policies
// so capacities (and byte counts) match. A nil slice is the zero initialized array.

// arrayCreate is b3Array_Create: alternative to zero initialization.
func arrayCreate[T any](a *[]T) {
	*a = nil
}

// arrayCreateN is b3Array_CreateN.
func arrayCreateN[T any](a *[]T, n int) {
	if n > 0 {
		*a = make([]T, 0, n)
	} else {
		*a = nil
	}
}

// arrayDestroy is b3Array_Destroy.
func arrayDestroy[T any](a *[]T) {
	*a = nil
}

// arrayGrowTo reallocates to exactly newCapacity, keeping the elements. This is b3GrowAlloc.
func arrayGrowTo[T any](a *[]T, newCapacity int) {
	data := make([]T, len(*a), newCapacity)
	copy(data, *a)
	*a = data
}

// arrayReserve is b3Array_Reserve.
func arrayReserve[T any](a *[]T, n int) {
	if cap(*a) < n {
		arrayGrowTo(a, n)
	}
}

// arrayResize is b3Array_Resize.
// PORT: elements exposed by growing keep their previous values (zero for fresh memory) where
// upstream leaves them uninitialized.
func arrayResize[T any](a *[]T, n int) {
	arrayReserve(a, n)
	*a = (*a)[:n]
}

// arrayPush is b3Array_Push: push a new element by value.
func arrayPush[T any](a *[]T, value T) {
	if len(*a) >= cap(*a) {
		newCapacity := 8
		if cap(*a) != 0 {
			newCapacity = 2 * cap(*a)
		}
		arrayGrowTo(a, newCapacity)
	}
	*a = append(*a, value)
}

// arrayGet is b3Array_Get: get a pointer to an element.
func arrayGet[T any](a []T, index int) *T {
	assert(0 <= index && index < len(a), "0 <= ( index ) && ( index ) < ( a ).count")
	return &a[index]
}

// arrayEmplace is b3Array_Emplace: create a new element and return a pointer to it.
// PORT: the new element is zeroed; upstream leaves it uninitialized.
func arrayEmplace[T any](a *[]T) *T {
	return emplaceHelper(a)
}

// arrayPop is b3Array_Pop: remove the last element and return it by value.
func arrayPop[T any](a *[]T) T {
	assert(0 < len(*a), "0 < ( a ).count")
	n := len(*a) - 1
	value := (*a)[n]
	*a = (*a)[:n]
	return value
}

// arrayAddIndex is b3Array_AddIndex: add an element and return its index.
func arrayAddIndex[T any](a *[]T) int {
	emplaceHelper(a)
	return len(*a) - 1
}

// arrayAppend is b3Array_Append: append a contiguous run of values.
func arrayAppend[T any](a *[]T, src []T) {
	n := len(src)
	if len(*a)+n > cap(*a) {
		req := len(*a) + n
		newCapacity := 8
		if req > 2 {
			newCapacity = req + (req >> 1)
		}
		arrayGrowTo(a, newCapacity)
	}
	*a = append(*a, src...)
}

// arrayMemZero is b3Array_MemZero: zero the entire allocated buffer (capacity, not just count).
func arrayMemZero[T any](a []T) {
	if cap(a) > 0 {
		clear(a[:cap(a)])
	}
}

// arrayRemoveSwap is b3Array_RemoveSwap: remove an element by swapping with the last element. If the index is the
// last element it returns NullIndex, otherwise it returns the index of the last element (which is now out of bounds).
func arrayRemoveSwap[T any](a *[]T, index int) int {
	return removeHelper(a, index)
}

// emplaceHelper is b3EmplaceHelper.
func emplaceHelper[T any](a *[]T) *T {
	if len(*a) >= cap(*a) {
		oldCapacity := cap(*a)
		newCapacity := 16
		if oldCapacity != 0 {
			newCapacity = 2 * oldCapacity
		}
		arrayGrowTo(a, newCapacity)
	}
	var zero T
	*a = append(*a, zero)
	return &(*a)[len(*a)-1]
}

// removeHelper is b3RemoveHelper.
func removeHelper[T any](a *[]T, index int) int {
	count := len(*a)
	assert(0 <= index && index < count, "0 <= index && index < *count && \"Array index out of bounds\"")

	count--
	if index != count {
		(*a)[index] = (*a)[count]
		*a = (*a)[:count]
		return count
	}

	*a = (*a)[:count]
	return NullIndex
}

// arrayClear is b3Array_Clear.
func arrayClear[T any](a *[]T) {
	*a = (*a)[:0]
}

// arrayByteCount is b3Array_ByteCount.
func arrayByteCount[T any](a []T) int {
	var zero T
	return cap(a) * int(unsafe.Sizeof(zero))
}
