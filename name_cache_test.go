// Port of test/test_name_cache.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// PORT: b3AddName( &cache, NULL ) and b3AddName( &cache, "" ) are the same call in Go (NULL is
// the empty string); both assertions are kept. strcmp( a, b ) == 0 is a == b.

var _ = Describe("NameCacheTest", func() {
	// Direct unit test of the cache: dedup, reverse lookup, empty/null, arbitrary length, no leak.
	It("CacheUnit", func() {
		base := GetByteCount()

		cache := createNameCache()

		// Empty and NULL map to the null id, which resolves to no string.
		ensure(addName(&cache, "") == nullName)
		ensure(addName(&cache, "") == nullName)
		ensure(findName(&cache, nullName) == "")

		// Distinct strings get distinct ids and round-trip.
		crate := addName(&cache, "crate")
		barrel := addName(&cache, "barrel")
		ensure(crate != nullName && barrel != nullName && crate != barrel)
		ensure(findName(&cache, crate) == "crate")
		ensure(findName(&cache, barrel) == "barrel")

		// Re-interning the same bytes returns the same id and stores no second entry.
		ensure(addName(&cache, "crate") == crate)
		ensure(len(cache.entries) == 2)

		// A name far longer than any inline buffer round-trips exactly.
		var longNameBuf [4096]byte
		for i := 0; i < len(longNameBuf)-1; i++ {
			longNameBuf[i] = byte('a' + i%26)
		}
		longNameBuf[len(longNameBuf)-1] = 0
		// PORT: the C string ends at the terminator.
		longName := string(longNameBuf[:len(longNameBuf)-1])
		longId := addName(&cache, longName)
		ensure(findName(&cache, longId) == longName)
		ensure(len(cache.entries) == 3)

		destroyNameCache(&cache)

		// The cache owns every interned copy, so all of it comes back on destroy.
		ensure(GetByteCount() == base)
	})

	// PORT: the round trip and rollback subtests record a world and replay it (recording.c,
	// replay.c, world_snapshot.c from milestone 5, plus the milestone 3 world files).
	PIt("NameRoundTrip (needs recording.c, replay.c, world_snapshot.c and the milestone 3 world files)")
	PIt("RollbackNames (needs recording.c, replay.c, world_snapshot.c and the milestone 3 world files)")
})

var _ = Describe("NameCacheHash32", func() {
	// PORT: Go-only check that b3Hash32 matches the C function bit for bit (values printed by a
	// C build of name_cache.c's b3Hash32); recordings store these ids.
	It("matches the C hash", func() {
		cases := []struct {
			s string
			h uint32
		}{
			{"", 0xab3e7c0b},
			{"crate", 0xe47034ab},
			{"barrel", 0xad4dd570},
			{"box_hull", 0xb32cf85f},
			{"a_very_long_body_name_that_exceeds_any_inline_name_buffer", 0xb25b73e6},
		}
		for _, c := range cases {
			ensure(hash32([]byte(c.s)) == c.h)
		}
	})
})
