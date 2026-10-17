// Port of src/table.h and src/table.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- table.h ----

// keyHash: I need a good hash because the keys are built from pairs of increasing integers.
// A simple hash like hash = (integer1 XOR integer2) has many collisions.
// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
// https://preshing.com/20130107/this-hash-set-is-faster-than-a-judy-array/
// todo try: https://www.jandrewrogers.com/2019/02/12/fast-perfect-hashing/
// todo try:
// https://probablydance.com/2018/06/16/fibonacci-hashing-the-optimization-that-the-world-forgot-or-a-better-alternative-to-integer-modulo/
func keyHash(key uint64) uint32 {
	h := key
	h ^= h >> 33
	h *= 0xff51afd7ed558ccd
	h ^= h >> 33
	h *= 0xc4ceb9fe1a85ec53
	h ^= h >> 33

	return uint32(h)
}

// setItem is b3SetItem.
type setItem struct {
	key  uint64
	hash uint32
}

// hashSet is b3HashSet: an open addressing hash set with linear probing. Iteration order (slot
// order) is deterministic and matches upstream.
type hashSet struct {
	items    []setItem
	capacity uint32
	count    uint32
}

const (
	shapeMask = MaxShapes - 1      // B3_SHAPE_MASK
	childMask = MaxChildShapes - 1 // B3_CHILD_MASK
)

// shapePairKey is b3ShapePairKey.
func shapePairKey(s1, s2, c int) uint64 {
	if s1 < s2 {
		return (uint64(shapeMask&s1) << (64 - ShapePower)) |
			(uint64(shapeMask&s2) << (64 - 2*ShapePower)) | uint64(childMask&c)
	}

	return (uint64(shapeMask&s2) << (64 - ShapePower)) |
		(uint64(shapeMask&s1) << (64 - 2*ShapePower)) |
		uint64(childMask&c)
}

// ---- table.c ----

// probeCount is b3_probeCount, only counted in debug builds.
var probeCount atomicInt

// PORT: the _Static_asserts become compile-time array lengths.
var (
	_ = [2*ShapePower + ChildPower - 64]struct{}{} // compound power
	_ = [ChildPower - 9]struct{}{}                 // compound child power: B3_CHILD_POWER > 8
)

func createSet(capacity int32) hashSet {
	var set hashSet

	// Capacity must be a power of 2
	if capacity > 16 {
		set.capacity = uint32(roundUpPowerOf2(int(capacity)))
	} else {
		set.capacity = 16
	}

	set.count = 0
	set.items = make([]setItem, set.capacity)

	return set
}

func destroySet(set *hashSet) {
	set.items = nil
	set.count = 0
	set.capacity = 0
}

func clearSet(set *hashSet) {
	set.count = 0
	clear(set.items[:set.capacity])
}

func findSlot(set *hashSet, key uint64, hash uint32) int32 {
	capacity := set.capacity
	index := int32(hash & (capacity - 1))
	items := set.items
	for items[index].hash != 0 && items[index].key != key {
		if debugBuild {
			atomicFetchAddInt(&probeCount, 1)
		}
		index = (index + 1) & int32(capacity-1)
	}

	return index
}

func addKeyHaveCapacity(set *hashSet, key uint64, hash uint32) {
	index := findSlot(set, key, hash)
	items := set.items
	assert(items[index].hash == 0, "items[index].hash == 0")

	items[index].key = key
	items[index].hash = hash
	set.count += 1
}

func growTable(set *hashSet) {
	oldCount := set.count

	oldCapacity := set.capacity
	oldItems := set.items

	set.count = 0
	// Capacity must be a power of 2
	set.capacity = 2 * oldCapacity
	set.items = make([]setItem, set.capacity)

	// Transfer items into new array
	for i := uint32(0); i < oldCapacity; i++ {
		item := &oldItems[i]
		if item.hash == 0 {
			// this item was empty
			continue
		}

		addKeyHaveCapacity(set, item.key, item.hash)
	}

	assert(set.count == oldCount, "set->count == oldCount")
}

func containsKey(set *hashSet, key uint64) bool {
	// key of zero is a sentinel
	assert(key != 0, "key != 0")
	hash := keyHash(key)
	index := findSlot(set, key, hash)
	return set.items[index].key == key
}

// containsHashedKey: the caller already has the hash, which lets a batch prefetch the slots before probing them.
func containsHashedKey(set *hashSet, key uint64, hash uint32) bool {
	assert(key != 0, "key != 0")
	assert(hash == keyHash(key), "hash == b3KeyHash( key )")
	index := findSlot(set, key, hash)
	return set.items[index].key == key
}

func getHashSetBytes(set *hashSet) int {
	return int(set.capacity) * 16 // sizeof( b3SetItem )
}

// addKey returns true if key was already in set
func addKey(set *hashSet, key uint64) bool {
	// key of zero is a sentinel
	assert(key != 0, "key != 0")

	hash := keyHash(key)
	assert(hash != 0, "hash != 0")

	index := findSlot(set, key, hash)
	if set.items[index].hash != 0 {
		// Already in set
		assert(set.items[index].hash == hash && set.items[index].key == key, "set->items[index].hash == hash && set->items[index].key == key")
		return true
	}

	if 2*set.count >= set.capacity {
		growTable(set)
	}

	addKeyHaveCapacity(set, key, hash)
	return false
}

// removeKey returns true if the key was found
// See https://en.wikipedia.org/wiki/Open_addressing
func removeKey(set *hashSet, key uint64) bool {
	hash := keyHash(key)
	i := findSlot(set, key, hash)
	items := set.items
	if items[i].hash == 0 {
		// Not in set
		return false
	}

	// Mark item i as unoccupied
	items[i].key = 0
	items[i].hash = 0

	assert(set.count > 0, "set->count > 0")
	set.count -= 1

	// Attempt to fill item i
	j := i
	capacity := set.capacity
	for {
		j = (j + 1) & int32(capacity-1)
		if items[j].hash == 0 {
			break
		}

		// k is the first item for the hash of j
		k := int32(items[j].hash & (capacity - 1))

		// determine if k lies cyclically in (i,j]
		// i <= j: | i..k..j |
		// i > j: |.k..j  i....| or |....j     i..k.|
		if i <= j {
			if i < k && k <= j {
				continue
			}
		} else {
			if i < k || k <= j {
				continue
			}
		}

		// Move j into i
		items[i] = items[j]

		// Mark item j as unoccupied
		items[j].key = 0
		items[j].hash = 0

		i = j
	}

	return true
}

// countSetBits is b3CountSetBits.
// This function is here because ctz.h is included by
// this file but not in bitset.c
func countSetBits(bitSet *bitSet) int {
	popCount := 0
	blockCount := bitSet.blockCount
	for i := uint32(0); i < blockCount; i++ {
		popCount += popCount64(bitSet.bits[i])
	}

	return popCount
}
