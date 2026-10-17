// Port of the API of src/verstable.h (Verstable v2.2.1, bundled with Box3D,
// https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// Verstable: Copyright (c) 2023-2025 Jackson L. Allan. SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "unsafe"

// PORT: verstable.h is a third party generic hash table instantiated with NAME/KEY_TY/VAL_TY
// macros. The Go port is a generic open-addressing table with linear probing that exposes the
// subset of the API upstream uses (init, reserve, insert, get_or_insert, get, erase_itr, size,
// first/next, is_end, bucket_count, total_alloc_size, cleanup). It is not a transliteration:
// bucket placement and iteration order differ from Verstable. Upstream never depends on
// either (lookups, inserts, and an order independent sum), so behaviour is unchanged.
// Load factor and minimum bucket count follow Verstable (MAX_LOAD 0.9, 8 buckets).

const (
	vtMaxLoad                = 0.9 // MAX_LOAD
	vtMinNonzeroBucketCount  = 8   // VT_MIN_NONZERO_BUCKET_COUNT
	vtMetadataBytesPerBucket = 2   // Verstable keeps one uint16_t of metadata per bucket
)

// vtEntry is the key and value of an occupied bucket (NAME_itr.data).
type vtEntry[K any, V any] struct {
	key K
	val V
}

type vtBucket[K any, V any] struct {
	entry vtEntry[K, V]
	used  bool
}

// verstable is a NAME table with KEY_TY K and VAL_TY V. HASH_FN and CMPR_FN are given to init.
type verstable[K any, V any] struct {
	hashFn  func(K) uint64
	cmprFn  func(K, K) bool
	buckets []vtBucket[K, V]
	count   int
}

// vtItr is NAME_itr. data is nil for the end iterator.
type vtItr[K any, V any] struct {
	data  *vtEntry[K, V]
	index int
}

// init is NAME_init.
func (t *verstable[K, V]) init(hashFn func(K) uint64, cmprFn func(K, K) bool) {
	t.hashFn = hashFn
	t.cmprFn = cmprFn
	t.buckets = nil
	t.count = 0
}

// size is NAME_size.
func (t *verstable[K, V]) size() int {
	return t.count
}

// bucketCount is NAME_bucket_count.
func (t *verstable[K, V]) bucketCount() int {
	return len(t.buckets)
}

// totalAllocSize is NAME_total_alloc_size.
// PORT: buckets plus Verstable's 2 bytes of metadata per bucket, using the Go entry size.
func (t *verstable[K, V]) totalAllocSize() int {
	var e vtEntry[K, V]
	return len(t.buckets) * (int(unsafe.Sizeof(e)) + vtMetadataBytesPerBucket)
}

// isEnd is NAME_is_end.
func (t *verstable[K, V]) isEnd(itr vtItr[K, V]) bool {
	return itr.data == nil
}

func (t *verstable[K, V]) find(key K) int {
	if len(t.buckets) == 0 {
		return -1
	}
	mask := len(t.buckets) - 1
	i := int(t.hashFn(key)) & mask
	for t.buckets[i].used {
		if t.cmprFn(t.buckets[i].entry.key, key) {
			return i
		}
		i = (i + 1) & mask
	}
	return -1
}

func (t *verstable[K, V]) rehash(bucketCount int) {
	old := t.buckets
	t.buckets = make([]vtBucket[K, V], bucketCount)
	mask := bucketCount - 1
	for i := range old {
		if old[i].used {
			j := int(t.hashFn(old[i].entry.key)) & mask
			for t.buckets[j].used {
				j = (j + 1) & mask
			}
			t.buckets[j] = old[i]
		}
	}
}

// reserve is NAME_reserve: make room for size keys without rehashing.
func (t *verstable[K, V]) reserve(size int) bool {
	bucketCount := len(t.buckets)
	if bucketCount == 0 {
		bucketCount = vtMinNonzeroBucketCount
	}
	for float64(size) > float64(bucketCount)*vtMaxLoad {
		bucketCount *= 2
	}
	if bucketCount > len(t.buckets) {
		t.rehash(bucketCount)
	}
	return true
}

func (t *verstable[K, V]) insertRaw(key K, val V, replace bool) vtItr[K, V] {
	if i := t.find(key); i >= 0 {
		if replace {
			t.buckets[i].entry = vtEntry[K, V]{key, val}
		}
		return vtItr[K, V]{&t.buckets[i].entry, i}
	}

	if len(t.buckets) == 0 || float64(t.count+1) > float64(len(t.buckets))*vtMaxLoad {
		newCount := vtMinNonzeroBucketCount
		if len(t.buckets) != 0 {
			newCount = 2 * len(t.buckets)
		}
		t.rehash(newCount)
	}

	mask := len(t.buckets) - 1
	i := int(t.hashFn(key)) & mask
	for t.buckets[i].used {
		i = (i + 1) & mask
	}
	t.buckets[i] = vtBucket[K, V]{vtEntry[K, V]{key, val}, true}
	t.count += 1
	return vtItr[K, V]{&t.buckets[i].entry, i}
}

// insert is NAME_insert: insert or replace.
func (t *verstable[K, V]) insert(key K, val V) vtItr[K, V] {
	return t.insertRaw(key, val, true)
}

// getOrInsert is NAME_get_or_insert: return the existing entry or insert a new one.
func (t *verstable[K, V]) getOrInsert(key K, val V) vtItr[K, V] {
	return t.insertRaw(key, val, false)
}

// get is NAME_get.
func (t *verstable[K, V]) get(key K) vtItr[K, V] {
	i := t.find(key)
	if i < 0 {
		return vtItr[K, V]{}
	}
	return vtItr[K, V]{&t.buckets[i].entry, i}
}

// eraseItr is NAME_erase_itr.
// PORT: does not return the next iterator (upstream ignores it); uses backward shift deletion.
func (t *verstable[K, V]) eraseItr(itr vtItr[K, V]) {
	assert(itr.data != nil, "!NAME_is_end( itr )")
	mask := len(t.buckets) - 1
	i := itr.index
	t.buckets[i] = vtBucket[K, V]{}
	t.count -= 1

	j := i
	for {
		j = (j + 1) & mask
		if !t.buckets[j].used {
			break
		}
		home := int(t.hashFn(t.buckets[j].entry.key)) & mask
		// Move j back to i if its home is not cyclically within (i, j].
		if (j > i && (home <= i || home > j)) || (j < i && (home <= i && home > j)) {
			t.buckets[i] = t.buckets[j]
			t.buckets[j] = vtBucket[K, V]{}
			i = j
		}
	}
}

// first is NAME_first.
func (t *verstable[K, V]) first() vtItr[K, V] {
	return t.scan(0)
}

// next is NAME_next.
func (t *verstable[K, V]) next(itr vtItr[K, V]) vtItr[K, V] {
	return t.scan(itr.index + 1)
}

func (t *verstable[K, V]) scan(start int) vtItr[K, V] {
	for i := start; i < len(t.buckets); i++ {
		if t.buckets[i].used {
			return vtItr[K, V]{&t.buckets[i].entry, i}
		}
	}
	return vtItr[K, V]{}
}

// clear is NAME_clear.
func (t *verstable[K, V]) clear() {
	clear(t.buckets)
	t.count = 0
}

// cleanup is NAME_cleanup.
func (t *verstable[K, V]) cleanup() {
	t.buckets = nil
	t.count = 0
}
