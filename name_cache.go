// Port of src/name_cache.h (and later src/name_cache.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- name_cache.h ----

// PORT: header types and the inline b3FindNameWithDefault; name_cache.c lands in milestone 5
// (or earlier, see PROGRESS.md). b3World embeds the cache. The name_cache.c functions that
// milestone 3 files call (body.c, shape.c, physics_world.c) are placeholders in the name_cache.c
// section at the end.

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
	// PORT: void* map; field renamed (map is a Go keyword). Typed when name_cache.c lands.
	nameMap any
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

// PORT: placeholders until milestone 5 (name_cache.c). They keep no names: addName returns
// B3_NULL_NAME and findName returns NULL (""), so names read back empty until name_cache.c lands.
// The signatures are final; the owner of name_cache.c replaces the bodies in upstream order.

// createNameCache is b3CreateNameCache.
func createNameCache() nameCache {
	// PORT: placeholder until milestone 5 (name_cache.c)
	return nameCache{}
}

// destroyNameCache is b3DestroyNameCache.
func destroyNameCache(cache *nameCache) {
	// PORT: placeholder until milestone 5 (name_cache.c)
}

// addName is b3AddName.
// PORT: const char* name is a string (empty = NULL).
func addName(cache *nameCache, name string) uint32 {
	// PORT: placeholder until milestone 5 (name_cache.c)
	return nullName
}

// findName is b3FindName.
// PORT: returns "" for NULL.
func findName(cache *nameCache, id uint32) string {
	// PORT: placeholder until milestone 5 (name_cache.c)
	return ""
}
