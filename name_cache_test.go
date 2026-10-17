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
// the empty string); both assertions are kept. strcmp( a, b ) == 0 is a == b. A NULL name check
// (name != NULL) is name != "".

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

	// Names live on the world, so the world snapshot must carry them. Record a snapshot-seeded session
	// and confirm every body and shape name resolves on the reconstructed replay world.
	It("NameRoundTrip", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		nameCacheBuildNamedScene(worldId)

		// Record from a snapshot of the populated world, so names ride in the frame-0 image.
		worldId.StartRecording(rec)
		for i := 0; i < 20; i++ {
			worldId.Step(1.0/60.0, 4)
		}
		worldId.StopRecording()
		worldId.Destroy()

		ensure(ValidateReplay(rec.GetData()[:rec.GetSize()], 1))

		player := CreatePlayer(rec.GetData()[:rec.GetSize()], 1)
		ensure(player != nil)

		player.SeekFrame(20)
		ensure(!player.HasDiverged())
		ensure(nameCacheCheckReplayNames(player) == 0)

		player.Destroy()
		rec.Destroy()
	})

	// Rollback preserves names: a backward seek restores from a keyframe, which rebuilds world->names
	// on each restore. Scrub across keyframe boundaries, confirm names resolve every time, and require
	// the tracked byte count to return to baseline so a re-load that drops duplicate copies leaks nothing.
	It("RollbackNames", func() {
		base := GetByteCount()

		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		nameCacheBuildNamedScene(worldId)

		// Settle, then record a snapshot-seeded session long enough to span several keyframes.
		for i := 0; i < 10; i++ {
			worldId.Step(1.0/60.0, 4)
		}
		worldId.StartRecording(rec)
		const totalFrames = 80
		for i := 0; i < totalFrames; i++ {
			worldId.Step(1.0/60.0, 4)
		}
		worldId.StopRecording()
		worldId.Destroy()

		player := CreatePlayer(rec.GetData()[:rec.GetSize()], 1)
		ensure(player != nil)

		// Play to the end so the keyframe ring is populated.
		player.SeekFrame(totalFrames)
		ensure(!player.HasDiverged())

		// Scrub backward and forward across keyframe boundaries. Each restore rebuilds the name table.
		targets := []int{10, 60, 5, 70, 1, 40}
		for k := 0; k < arrayCount(targets); k++ {
			player.SeekFrame(targets[k])
			ensure(player.GetFrame() == targets[k])
			ensure(!player.HasDiverged())
			ensure(nameCacheCheckReplayNames(player) == 0)
		}

		player.Destroy()
		rec.Destroy()

		// Every restore reloaded the name table; dropped duplicate copies must be freed, not leaked.
		ensure(GetByteCount() == base)
	})
})

// Build a ground plus named dynamic boxes. Two share a name to exercise dedup and one is longer
// than the old inline buffer to exercise variable length. Ground is ordinal 0, boxes 1..N.
var nameCacheBodyNames = [...]string{ // s_bodyNames
	"crate",
	"barrel",
	"crate",
	"a_very_long_body_name_that_exceeds_any_inline_name_buffer",
}

// Each box also carries a named hull shape. Shapes could not hold a name at all before the cache
// (the old inline shape buffer had length 0), so this is a new path. Two shapes share a name for
// dedup, one shape shares a body's name to prove shape ids intern independently, and one is far
// longer than any inline buffer.
var nameCacheShapeNames = [...]string{ // s_shapeNames
	"box_hull",
	"box_hull",
	"crate",
	"a_very_long_shape_name_that_never_fit_the_old_inline_buffer",
}

// PORT: _Static_assert( ARRAY_COUNT( s_shapeNames ) == ARRAY_COUNT( s_bodyNames ) ) as constant
// index expressions on the array lengths.
var _ = [1]struct{}{}[len(nameCacheShapeNames)-len(nameCacheBodyNames)] // name arrays must stay parallel
var _ = [1]struct{}{}[len(nameCacheBodyNames)-len(nameCacheShapeNames)] // name arrays must stay parallel

// nameCacheBuildNamedScene is BuildNamedScene.
func nameCacheBuildNamedScene(worldId WorldID) {
	worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

	groundDef := DefaultBodyDef()
	groundDef.Type = StaticBody
	groundId := worldId.CreateBody(&groundDef)
	groundBox := MakeBoxHull(20.0, 1.0, 20.0)
	groundShape := DefaultShapeDef()
	groundId.CreateHullShape(&groundShape, &groundBox.Base)

	boxShape := DefaultShapeDef()
	boxShape.Density = 1.0
	for i := 0; i < arrayCount(nameCacheBodyNames[:]); i++ {
		box := MakeBoxHull(0.5, 0.5, 0.5)
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, 1.0 + 1.1*float32(i), 0.0}
		bodyDef.Name = nameCacheBodyNames[i]
		bodyId := worldId.CreateBody(&bodyDef)
		boxShape.Name = nameCacheShapeNames[i]
		bodyId.CreateHullShape(&boxShape, &box.Base)
	}
}

// nameCacheCheckReplayNames is CheckReplayNames.
// PORT: ENSURE returns 1 from the helper in C; here ensure fails the spec directly, so the helper
// returns 0 whenever it returns at all.
func nameCacheCheckReplayNames(player *RecPlayer) int {
	for i := 0; i < arrayCount(nameCacheBodyNames[:]); i++ {
		id := player.GetBodyId(1 + i) // ordinal 0 is the ground
		ensure(id.IsValid())
		name := id.GetName()
		ensure(name != "" && name == nameCacheBodyNames[i])

		// Shape names ride the same world cache, so the snapshot must restore them too.
		var shapeId [1]ShapeID
		ensure(id.GetShapes(shapeId[:]) == 1)
		shapeName := shapeId[0].GetName()
		ensure(shapeName != "" && shapeName == nameCacheShapeNames[i])
	}
	return 0
}

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
