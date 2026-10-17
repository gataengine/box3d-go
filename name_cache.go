// Port of src/name_cache.h (and later src/name_cache.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- name_cache.h ----

// PORT: header types only; name_cache.c (and the inline b3FindNameWithDefault) land in
// milestone 5. b3World embeds the cache.

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
