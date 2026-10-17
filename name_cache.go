// Port of src/name_cache.h and src/name_cache.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- name_cache.h ----

// PORT: b3World embeds the cache. b3DeclareArray( b3NameEntry ) is a []nameEntry.

// nullName is B3_NULL_NAME.
const nullName uint32 = 0

// nameEntry is b3NameEntry.
type nameEntry struct {
	// Names are used for debugging and I'm favoring simplicity and
	// minimal storage. Collisions will be logged.
	hash   uint32
	length int32
	name   string // PORT: char* becomes string
}

// nameCache is b3NameCache.
type nameCache struct {
	entries []nameEntry
	// PORT: void* map; field renamed (map is a Go keyword) and typed as the b3NameMap of
	// name_cache.c.
	nameMap *nameMap
}

// findNameWithDefault is b3FindNameWithDefault.
// PORT: const char* is a string; NULL is the empty string.
func findNameWithDefault(cache *nameCache, id uint32, def string) string {
	name := findName(cache, id)
	if name == "" {
		return def
	}
	return name
}

// ---- name_cache.c ----

// nameMap is b3NameMap.
// PORT: verstable instance with KEY_TY uint32_t and VAL_TY int; HASH_FN vt_hash_integer widens
// the key to uint64 as Verstable does.
type nameMap = verstable[uint32, int]

func nameMapHash(key uint32) uint64 {
	return vtHashInteger(uint64(key))
}

func nameMapCmpr(key1, key2 uint32) bool {
	return key1 == key2
}

const (
	fnv32OffsetBasis uint32 = 0x811c9dc5 // FNV32_OFFSET_BASIS
	fnv32Prime       uint32 = 0x01000193 // FNV32_PRIME
)

// hash32 is b3Hash32.
// This is designed to hash small strings and while avoiding collisions
// for a 32-bit hash.
// Note: changing this will break recordings
func hash32(data []byte) uint32 {
	// Cast to unsigned to avoid sign extension
	p := data

	// FNV-1a
	h := fnv32OffsetBasis
	for i := 0; i < len(p); i++ {
		h ^= uint32(p[i])
		h *= fnv32Prime
	}

	// Murmur3 finalizer to help with short strings
	h ^= h >> 16
	h *= 0x85ebca6b
	h ^= h >> 13
	h *= 0xc2b2ae35
	h ^= h >> 16
	return h
}

// createNameCache is b3CreateNameCache.
func createNameCache() nameCache {
	cache := nameCache{}
	arrayCreate(&cache.entries)

	m := &nameMap{}
	m.init(nameMapHash, nameMapCmpr)
	cache.nameMap = m
	return cache
}

// destroyNameCache is b3DestroyNameCache.
// PORT: the interned copies are garbage collected; b3Free of each name is dropped.
func destroyNameCache(cache *nameCache) {
	arrayDestroy(&cache.entries)
	cache.nameMap.cleanup()
	cache.nameMap = nil
}

// addName is b3AddName.
// PORT: const char* name is a string; NULL and "" are both the empty string.
func addName(cache *nameCache, name string) uint32 {
	if name == "" {
		return nullName
	}

	length := len(name)
	id := hash32([]byte(name))

	if id == 0 {
		id = 1
	}

	m := cache.nameMap
	itr := m.get(id)
	if m.isEnd(itr) == false {
		if cache.entries[itr.data.val].name != name {
			// Different name, same hash
			log("Hash collision on %s", name)
		}
		return id
	}

	// PORT: Go strings are immutable, so the string is the clone.
	clone := name
	index := len(cache.entries)
	entry := nameEntry{
		hash:   id,
		name:   clone,
		length: int32(length),
	}

	arrayPush(&cache.entries, entry)

	m.insert(id, index)
	return id
}

// findName is b3FindName.
// PORT: returns "" for NULL.
func findName(cache *nameCache, id uint32) string {
	m := cache.nameMap
	itr := m.get(id)
	if m.isEnd(itr) == false {
		index := itr.data.val
		entry := arrayGet(cache.entries, index)
		return entry.name
	}

	return ""
}

// loadName is b3LoadName.
// Load a name from a recording. Name ownership is transferred.
// PORT: char* name is a string; NULL and "" are both the empty string, and b3Free of a dropped
// name is dropped.
func loadName(cache *nameCache, id uint32, name string, length int) {
	if name == "" {
		return
	}

	// PORT: name[0] == 0 is covered by the NULL check above.
	if id == nullName {
		return
	}

	m := cache.nameMap
	itr := m.get(id)
	if m.isEnd(itr) == false {
		return
	}

	index := len(cache.entries)
	entry := nameEntry{
		hash:   id,
		name:   name,
		length: int32(length),
	}

	arrayPush(&cache.entries, entry)
	m.insert(id, index)
}
