// Port of test/test_hash.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"bytes"
	"math"
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
)

// PORT: the probe takes the hashed bytes as a slice (nil = NULL); B3_ALLOC/B3_FREE become make and
// GC, so hashProbeEnd only gathers the result. Blob bytes are the byte images from hullBytes,
// meshBytes and heightFieldBytes, and float arrays are viewed as bytes with unsafe.Slice, as the C
// casts them. The static helpers of test_hash.c are prefixed with hash so they cannot clash in the
// one test package.

// hashEntry is HashEntry.
// Separates the two things a repeated hash can mean. Identical bytes repeating is normal, since
// distinct generator parameters can still bake to the same geometry. Distinct bytes sharing a hash
// is the failure this file exists to catch, so the invariant under test is that different content
// gets different digests, never that different parameters do.
//
// Zero doubles as "no collision", since the hash reserves that value. Results are read back only
// after teardown so a failing test does not also dump a leak and bury which condition broke.
type hashEntry struct {
	hash      uint64
	byteCount int
	bytes     []byte
}

// hashProbe is HashProbe.
type hashProbe struct {
	set        hashSet
	entries    []hashEntry
	count      int
	capacity   int
	collision  uint64
	duplicates int
	sawZero    bool
	overflow   bool
}

// hashResult is HashResult.
type hashResult struct {
	collision  uint64
	count      int
	duplicates int
	sawZero    bool
	overflow   bool
}

// hashProbeBegin is ProbeBegin.
func hashProbeBegin(capacity int) hashProbe {
	probe := hashProbe{}
	probe.set = createSet(int32(capacity))
	probe.entries = make([]hashEntry, capacity)
	probe.capacity = capacity
	return probe
}

// hashProbeAdd is ProbeAdd.
// Pass NULL bytes when the caller builds provably distinct inputs, so any repeat is a collision by
// construction and there is nothing to compare against.
func hashProbeAdd(probe *hashProbe, hash uint64, blob []byte) {
	byteCount := len(blob)

	if hash == 0 {
		probe.sawZero = true
	}

	if probe.count == probe.capacity {
		probe.overflow = true
		return
	}

	if addKey(&probe.set, hash) {
		sameContent := false
		if blob != nil {
			for i := 0; i < probe.count; i++ {
				if probe.entries[i].hash != hash {
					continue
				}

				sameContent = probe.entries[i].byteCount == byteCount &&
					bytes.Equal(probe.entries[i].bytes, blob)
				break
			}
		}

		if sameContent {
			probe.duplicates += 1
		} else if probe.collision == 0 {
			probe.collision = hash
		}
	}

	entry := &probe.entries[probe.count]
	entry.hash = hash
	entry.byteCount = 0
	entry.bytes = nil

	if blob != nil && byteCount > 0 {
		entry.byteCount = byteCount
		entry.bytes = bytes.Clone(blob)
	}

	probe.count += 1
}

// hashProbeEnd is ProbeEnd.
func hashProbeEnd(probe *hashProbe) hashResult {
	result := hashResult{probe.collision, probe.count, probe.duplicates, probe.sawZero, probe.overflow}

	for i := 0; i < probe.count; i++ {
		probe.entries[i].bytes = nil
	}

	probe.entries = nil
	destroySet(&probe.set)
	return result
}

// hashFloatBytes views a float array as bytes, the (const uint8_t*) cast of the C.
func hashFloatBytes(values []float32) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(values))), len(values)*4)
}

var _ = Describe("HashTest", func() {
	// Pins that the upper bytes of each 8 byte word reach the low bits of the hash. Multiply carries
	// bits upward only, so a hash that keeps just the low half of its product leaves the low bits blind
	// to those bytes, and blobs differing only up there collapse onto a tiny range no finalizer can
	// spread back out. Varying the top byte of two words is the sharp case. A pairwise avalanche check
	// cannot catch this, since a finalizer scatters any single difference across all bits no matter how
	// weak the mixing behind it.
	It("HashWordFamily", func() {
		const (
			blobSize   = 16
			familySize = 256 * 256
		)

		probe := hashProbeBegin(familySize)

		var blob [blobSize]uint8
		for i := 0; i < blobSize; i++ {
			blob[i] = uint8(0x5A + i)
		}

		for a := 0; a < 256; a++ {
			for b := 0; b < 256; b++ {
				blob[7] = uint8(a)
				blob[15] = uint8(b)
				hashProbeAdd(&probe, hash64NonZero(blob[:blobSize]), nil)
			}
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count == familySize)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Every input bit must move the digest, at every offset. Flipping one byte in one position is a
	// weak check: a mixer can pass it while still being blind to whole regions of a longer blob.
	It("HashBitSweep", func() {
		const (
			blobSize = 32
			bitCount = 8 * blobSize
		)

		probe := hashProbeBegin(bitCount + 1)

		var blob [blobSize]uint8
		for i := 0; i < blobSize; i++ {
			blob[i] = uint8(0xA5 ^ (i * 17))
		}

		hashProbeAdd(&probe, hash64NonZero(blob[:blobSize]), nil)

		for bit := 0; bit < bitCount; bit++ {
			mask := uint8(1 << (bit & 7))
			blob[bit>>3] ^= mask
			hashProbeAdd(&probe, hash64NonZero(blob[:blobSize]), nil)
			blob[bit>>3] ^= mask
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count == bitCount+1)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Baked blobs carry explicit padding and unused array slots, so long runs of zeros are common and
	// content alone cannot separate them. Length has to reach the digest.
	It("HashZeroLengths", func() {
		const (
			maxLength = 4096
		)

		zeros := make([]uint8, maxLength)

		probe := hashProbeBegin(maxLength)
		for n := 0; n < maxLength; n++ {
			hashProbeAdd(&probe, hash64NonZero(zeros[:n]), nil)
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count == maxLength)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Mirrored geometry differs from its original only in sign bits, which sit at the top of every
	// float. A mixer that cannot carry high bits downward maps the whole family onto a few digests.
	It("HashFloatSigns", func() {
		const (
			floatCount = 12
			comboCount = 1 << floatCount
		)

		probe := hashProbeBegin(comboCount)

		var values [floatCount]float32
		for i := 0; i < floatCount; i++ {
			values[i] = 1.0 + float32(0.25*float32(i))
		}

		for mask := 0; mask < comboCount; mask++ {
			var flipped [floatCount]float32
			for i := 0; i < floatCount; i++ {
				if (mask & (1 << i)) != 0 {
					flipped[i] = -values[i]
				} else {
					flipped[i] = values[i]
				}
			}

			hashProbeAdd(&probe, hash64NonZero(hashFloatBytes(flipped[:])), nil)
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count == comboCount)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Procedurally placed vertices land one ulp apart. Those blobs differ in a single low mantissa bit
	// buried in a long run of identical bytes.
	It("HashFloatUlp", func() {
		const (
			floatCount = 64
		)

		probe := hashProbeBegin(floatCount + 1)

		var values [floatCount]float32
		for i := 0; i < floatCount; i++ {
			values[i] = 100.0 + float32(i)
		}

		hashProbeAdd(&probe, hash64NonZero(hashFloatBytes(values[:])), nil)

		for i := 0; i < floatCount; i++ {
			saved := values[i]
			bits := math.Float32bits(saved)
			bits += 1
			values[i] = math.Float32frombits(bits)

			hashProbeAdd(&probe, hash64NonZero(hashFloatBytes(values[:])), nil)
			values[i] = saved
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count == floatCount+1)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Real baked hulls. Every box shares its entire topology section and differs in a handful of floats,
	// which is the closest thing the engine produces to a worst case for a content hash.
	It("HashBoxHulls", func() {
		const (
			steps = 16
		)

		probe := hashProbeBegin(steps * steps * steps)

		for i := 0; i < steps; i++ {
			for j := 0; j < steps; j++ {
				for k := 0; k < steps; k++ {
					box := MakeBoxHull(0.5+float32(0.25*float32(i)), 0.5+float32(0.25*float32(j)), 0.5+float32(0.25*float32(k)))
					hashProbeAdd(&probe, box.Base.Hash, hullBytes(&box.Base))
				}
			}
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count == steps*steps*steps)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Same box, moved and turned. The extent bytes are identical across the family so the hash has to
	// separate these on transform alone.
	It("HashTransformedBoxHulls", func() {
		const (
			steps = 12
		)

		probe := hashProbeBegin(steps * steps)

		for i := 0; i < steps; i++ {
			for j := 0; j < steps; j++ {
				axis := Normalize(Vec3{1.0, 0.5 + float32(0.1*float32(j)), 0.25})
				transform := Transform{
					P: Vec3{0.125 * float32(i), -0.25 * float32(j), 0.5 * float32(i+j)},
					Q: MakeQuatFromAxisAngle(axis, 0.05*float32(i*steps+j)),
				}

				box := MakeTransformedBoxHull(1.0, 2.0, 3.0, transform)
				hashProbeAdd(&probe, box.Base.Hash, hullBytes(&box.Base))
			}
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count == steps*steps)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Tessellated hulls across a parameter sweep. Neighboring parameters produce blobs that agree
	// almost everywhere, including identical vertex counts and topology.
	It("HashProceduralHulls", func() {
		probe := hashProbeBegin(1024)

		for sides := 3; sides <= 18; sides++ {
			for r := 1; r <= 5; r++ {
				for h := 1; h <= 4; h++ {
					cylinder := CreateCylinder(0.5*float32(h), 0.25*float32(r), 0.0, sides)
					if cylinder != nil {
						hashProbeAdd(&probe, cylinder.Hash, hullBytes(cylinder))
						DestroyHull(cylinder)
					}

					// Cones need at least four slices
					if sides >= 4 {
						cone := CreateCone(0.5*float32(h), 0.25*float32(r), 0.1*float32(r), sides)
						if cone != nil {
							hashProbeAdd(&probe, cone.Hash, hullBytes(cone))
							DestroyHull(cone)
						}
					}
				}
			}
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count > 0)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Height fields are mostly a flat array of heights, so grids of nearby dimensions differ in very
	// little beyond their counts. Small wave grids flatten to the plain grid, which is why the probe
	// has to tell a duplicate blob apart from a collision.
	It("HashHeightFields", func() {
		probe := hashProbeBegin(512)

		for rows := 2; rows <= 14; rows++ {
			for cols := 2; cols <= 14; cols++ {
				scale := Vec3{1.0, 1.0, 1.0}

				grid := CreateGrid(rows, cols, scale, false)
				hashProbeAdd(&probe, grid.Hash, heightFieldBytes(grid))
				DestroyHeightField(grid)

				wave := CreateWave(rows, cols, scale, 0.25*float32(rows), 0.125*float32(cols), false)
				hashProbeAdd(&probe, wave.Hash, heightFieldBytes(wave))
				DestroyHeightField(wave)
			}
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count > 0)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Meshes carry a baked BVH, so most of the blob is derived data that moves in lockstep with small
	// parameter changes.
	It("HashMeshes", func() {
		probe := hashProbeBegin(256)

		for x := 2; x <= 10; x++ {
			for z := 2; z <= 10; z++ {
				grid := CreateGridMesh(x, z, 1.0, 1, false)
				hashProbeAdd(&probe, grid.Hash, meshBytes(grid))
				DestroyMesh(grid)
			}
		}

		for radial := 3; radial <= 12; radial++ {
			for tubular := 3; tubular <= 12; tubular++ {
				torus := CreateTorusMesh(radial, tubular, 1.0, 0.25)
				hashProbeAdd(&probe, torus.Hash, meshBytes(torus))
				DestroyMesh(torus)
			}
		}

		result := hashProbeEnd(&probe)
		ensure(result.overflow == false)
		ensure(result.count > 0)
		ensure(result.sawZero == false)
		ensure(result.collision == 0)
	})

	// Voxel colliders sit on a regular grid, so their coordinate floats vary only in high bits. The
	// hull database takes its home bucket from the low bits of the hash, so a mixer that cannot carry
	// high bits downward funnels every hull into one bucket. The digests stay distinct throughout,
	// which is exactly why the collision tests above cannot see it. From issue 120.
	It("HashVoxelDispersion", func() {
		const (
			hullCount = 3000
			lowBits   = 13
			lowCount  = 1 << lowBits
		)

		seen := make([]bool, lowCount)

		distinctLow := 0
		for i := 0; i < hullCount; i++ {
			const cell float32 = 0.25
			offset := Vec3{
				float32(i%15) * cell,
				float32((i/15)%20) * cell,
				float32(i/300) * cell,
			}

			hull := MakeOffsetBoxHull(0.5*cell, 0.5*cell, 0.5*cell, offset)

			low := int(hull.Base.Hash & (lowCount - 1))
			if seen[low] == false {
				seen[low] = true
				distinctLow += 1
			}
		}

		// Filling 8192 slots with 3000 draws tops out near 2500. The 32 bit hash this replaced reached 1.
		ensure(distinctLow > hullCount/2)
	})

	// The consequence of the above, measured where it hurt. These hulls drove the world hull database
	// to two million buckets and a hundred and fifty milliseconds, since every insert walked one chain.
	It("HashVoxelHullDatabase", func() {
		const (
			hullCount = 3000
		)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyId := worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()

		world := getWorldFromId(worldId)
		m := world.hullDatabase

		created := true
		for i := 0; i < hullCount; i++ {
			const cell float32 = 0.25
			offset := Vec3{
				float32(i%15) * cell,
				float32((i/15)%20) * cell,
				float32(i/300) * cell,
			}

			hull := MakeOffsetBoxHull(0.5*cell, 0.5*cell, 0.5*cell, offset)
			shapeId := bodyId.CreateHullShape(&shapeDef, &hull.Base)
			if shapeId.Index1 == 0 {
				created = false
				break
			}
		}

		entryCount := m.size()
		bucketCount := m.bucketCount()

		worldId.Destroy()

		ensure(created)
		ensure(entryCount == hullCount)

		// Healthy growth settles at 4096 for this many distinct hulls
		ensure(bucketCount <= 4*hullCount)
	})

	// Identical input must bake to an identical hash, or dedup silently stops working.
	It("HashStability", func() {
		box1 := MakeBoxHull(1.0, 2.0, 3.0)
		box2 := MakeBoxHull(1.0, 2.0, 3.0)
		ensure(box1.Base.Hash == box2.Base.Hash)

		cylinder1 := CreateCylinder(1.0, 0.5, 0.0, 12)
		cylinder2 := CreateCylinder(1.0, 0.5, 0.0, 12)
		sameHash := cylinder1.Hash == cylinder2.Hash
		sameSize := cylinder1.ByteCount == cylinder2.ByteCount
		sameBytes := sameSize && bytes.Equal(hullBytes(cylinder1), hullBytes(cylinder2))
		DestroyHull(cylinder1)
		DestroyHull(cylinder2)

		ensure(sameHash)
		ensure(sameBytes)

		// An empty blob has no content to mix, so it takes the reserved value
		ensure(hash64NonZero(nil) == 1)
	})
})
