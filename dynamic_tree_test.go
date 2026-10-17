// Port of test/test_dynamic_tree.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"bytes"
	"path/filepath"
	"slices"
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
)

const treeTestProxyCount = 300

var treeTestSeed uint32 = 12345

func treeTestNextRandom() uint32 {
	treeTestSeed = 1664525*treeTestSeed + 1013904223
	return treeTestSeed
}

func treeTestRandomFloat(lo, hi float32) float32 {
	t := float32(treeTestNextRandom()>>8) / float32(1<<24)
	return mulAdd(lo, t, hi-lo)
}

// treeTestRandomBox is RandomBox.
// PORT: C leaves the evaluation order of the initializer list unspecified; clang and gcc
// evaluate left to right, which the sequenced calls here reproduce.
func treeTestRandomBox(extent, size float32) AABB {
	var center, half Vec3
	center.X = treeTestRandomFloat(-extent, extent)
	center.Y = treeTestRandomFloat(-extent, extent)
	center.Z = treeTestRandomFloat(-extent, extent)
	half.X = treeTestRandomFloat(0.1, size)
	half.Y = treeTestRandomFloat(0.1, size)
	half.Z = treeTestRandomFloat(0.1, size)
	return AABB{Sub(center, half), Add(center, half)}
}

type treeHitSet struct {
	ids   [treeTestProxyCount]int
	count int
	bad   bool
}

// Every test seeds user data with the index of the box it created the proxy from, so the sets
// are keyed on user data. Proxy ids recycle, box indices do not.
// A callback cannot fail the test directly, so bad news rides back on the context.
func treeCollectCallback(proxyId int, userData uint64, context any) bool {
	set := context.(*treeHitSet)
	if set.count >= treeTestProxyCount || int(int32(userData)) < 0 || int(int32(userData)) >= treeTestProxyCount {
		set.bad = true
		return false
	}

	_ = proxyId
	set.ids[set.count] = int(int32(userData))
	set.count += 1
	return true
}

func treeSameSet(a, b *treeHitSet) int {
	ensure(a.bad == false)
	ensure(b.bad == false)
	ensure(a.count == b.count)
	slices.Sort(a.ids[:a.count])
	slices.Sort(b.ids[:b.count])
	for i := 0; i < a.count; i++ {
		ensure(a.ids[i] == b.ids[i])
	}

	return 0
}

// LeafRootHit
type treeLeafRootHit struct {
	count    int
	proxyId  int
	userData uint64
}

func treeLeafRootCallback(proxyId int, userData uint64, context any) bool {
	hit := context.(*treeLeafRootHit)
	hit.count += 1
	hit.proxyId = proxyId
	hit.userData = userData
	return true
}

// Returning -1 skips the proxy without shrinking the fraction, so the traversal keeps going
// and reports every proxy whose node box the segment crosses.
func treeRayCastCollectCallback(input *RayCastInput, proxyId int, userData uint64, context any) float32 {
	_ = input
	_ = proxyId
	set := context.(*treeHitSet)
	if set.count >= treeTestProxyCount || int(int32(userData)) < 0 || int(int32(userData)) >= treeTestProxyCount {
		set.bad = true
		return 0.0
	}

	set.ids[set.count] = int(int32(userData))
	set.count += 1
	return -1.0
}

func treeBoxCastCollectCallback(input *BoxCastInput, proxyId int, userData uint64, context any) float32 {
	_ = input
	_ = proxyId
	set := context.(*treeHitSet)
	if set.count >= treeTestProxyCount || int(int32(userData)) < 0 || int(int32(userData)) >= treeTestProxyCount {
		set.bad = true
		return 0.0
	}

	set.ids[set.count] = int(int32(userData))
	set.count += 1
	return -1.0
}

type treeClosestResult struct {
	boxes []AABB
	point Vec3
	bad   bool
}

func treeDistanceToBoxSqr(point Vec3, box AABB) float32 {
	r := Sub(point, Clamp(point, box.LowerBound, box.UpperBound))
	return Dot(r, r)
}

func treeClosestCollectCallback(distanceSqrMin float32, proxyId int, userData uint64, context any) float32 {
	result := context.(*treeClosestResult)
	if int(int32(userData)) != proxyId {
		result.bad = true
		return distanceSqrMin
	}

	d := treeDistanceToBoxSqr(result.point, result.boxes[proxyId])
	if d < distanceSqrMin {
		return d
	}
	return distanceSqrMin
}

// treeNodeBytes views nodes as bytes for the memcmp checks.
func treeNodeBytes(nodes []TreeNode, count int) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(nodes))), count*int(unsafe.Sizeof(TreeNode{})))
}

func treeParentBytes(parents []int32, count int) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(parents))), count*4)
}

var _ = Describe("DynamicTreeTest", func() {
	// A one proxy tree is a leaf at the root with an empty node beside it, which every traversal
	// has to survive.
	It("TreeLeafRoot", func() {
		tree := CreateDynamicTree(16)

		box := AABB{Vec3{-1.0, -1.0, -1.0}, Vec3{1.0, 1.0, 1.0}}
		proxyId := tree.CreateProxy(box, DefaultCategoryBits, 7)

		tree.Validate()
		ensure(tree.GetProxyCount() == 1)
		ensure(tree.GetHeight() == 0)
		ensure(tree.GetAreaRatio() == 0.0)
		ensure(tree.GetUserData(proxyId) == 7)

		var hit treeLeafRootHit
		hitBox := AABB{Vec3{0.5, 0.5, 0.5}, Vec3{2.0, 2.0, 2.0}}
		tree.Query(hitBox, DefaultMaskBits, false, treeLeafRootCallback, &hit)
		ensure(hit.count == 1)
		ensure(hit.proxyId == proxyId)
		ensure(hit.userData == 7)

		hit.count = 0
		missBox := AABB{Vec3{4.0, 4.0, 4.0}, Vec3{5.0, 5.0, 5.0}}
		tree.Query(missBox, DefaultMaskBits, false, treeLeafRootCallback, &hit)
		ensure(hit.count == 0)

		// The rebuild has one build leaf and the refit has nothing above it
		tree.Rebuild(true)
		tree.Validate()
		tree.refit()
		tree.Validate()
		ensure(tree.GetProxyCount() == 1)

		grown := box.Inflate(1.0)
		tree.EnlargeProxy(proxyId, grown)
		ensure(tree.GetAABB(proxyId).Contains(grown))

		tree.Destroy()
	})

	// The tree must agree with a linear scan over the stored boxes. This is the gate on the
	// SIMD box helpers, which have three separate lane layouts.
	It("TreeQueryBruteForce", func() {
		treeTestSeed = 12345

		tree := CreateDynamicTree(16)

		var boxes [treeTestProxyCount]AABB
		var proxies [treeTestProxyCount]int

		for i := 0; i < treeTestProxyCount; i++ {
			boxes[i] = treeTestRandomBox(6.0, 1.5)
			proxies[i] = tree.CreateProxy(boxes[i], DefaultCategoryBits, uint64(i))
			ensure(proxies[i] == i)
		}

		tree.Validate()
		ensure(tree.GetProxyCount() == treeTestProxyCount)

		totalHits := 0
		for round := 0; round < 200; round++ {
			queryBox := treeTestRandomBox(7.0, 3.0)

			var treeHits treeHitSet
			tree.Query(queryBox, DefaultMaskBits, false, treeCollectCallback, &treeHits)

			var bruteHits treeHitSet
			for i := 0; i < treeTestProxyCount; i++ {
				if boxes[i].Overlaps(queryBox) {
					bruteHits.ids[bruteHits.count] = i
					bruteHits.count += 1
				}
			}

			ensure(treeSameSet(&treeHits, &bruteHits) == 0)
			totalHits += treeHits.count
		}

		// Guard against the comparison above agreeing on nothing
		ensure(totalHits > 1000)

		tree.Destroy()
	})

	// Category bits live on the proxy now, so the filter has to bite at the leaf.
	It("TreeCategoryFilter", func() {
		treeTestSeed = 99

		tree := CreateDynamicTree(16)

		var boxes [treeTestProxyCount]AABB
		var bits [treeTestProxyCount]uint64
		var proxies [treeTestProxyCount]int

		for i := 0; i < treeTestProxyCount; i++ {
			boxes[i] = treeTestRandomBox(8.0, 0.6)
			bits[i] = 1 << (i % 3)
			proxies[i] = tree.CreateProxy(boxes[i], bits[i], uint64(i))
			ensure(tree.GetCategoryBits(proxies[i]) == bits[i])
		}

		queryBox := AABB{Vec3{-8.0, -8.0, -8.0}, Vec3{8.0, 8.0, 8.0}}

		// A single bit mask makes requireAll equivalent to the any-bit test, so the last pass is
		// the one that proves requireAll accepts rather than just rejects
		masks := [3]uint64{0x1, 0x3, 0x2}
		requireAlls := [3]bool{false, false, true}

		for pass := 0; pass < 3; pass++ {
			mask := masks[pass]
			requireAll := requireAlls[pass]

			var treeHits treeHitSet
			tree.Query(queryBox, mask, requireAll, treeCollectCallback, &treeHits)

			var bruteHits treeHitSet
			for i := 0; i < treeTestProxyCount; i++ {
				var accept bool
				if requireAll {
					accept = (bits[i] & mask) == mask
				} else {
					accept = (bits[i] & mask) != 0
				}
				if accept && boxes[i].Overlaps(queryBox) {
					bruteHits.ids[bruteHits.count] = i
					bruteHits.count += 1
				}
			}

			ensure(treeSameSet(&treeHits, &bruteHits) == 0)
			ensure(treeHits.count > treeTestProxyCount/4)
		}

		// requireAll with two bits can never match a single bit proxy
		var noHits treeHitSet
		tree.Query(queryBox, 0x5, true, treeCollectCallback, &noHits)
		ensure(noHits.count == 0)

		// Changing the bits has to change the filter without touching the structure
		tree.SetCategoryBits(proxies[0], 0x8)
		ensure(tree.GetCategoryBits(proxies[0]) == 0x8)
		tree.Validate()

		var oneHit treeHitSet
		tree.Query(queryBox, 0x8, false, treeCollectCallback, &oneHit)
		ensure(oneHit.count == 1)
		ensure(oneHit.ids[0] == 0)

		tree.Destroy()
	})

	// Moving and enlarging every proxy exercises remove, insert, the ordered flag, the sweep refit
	// and the partial rebuild, then the query has to still agree with the scan.
	It("TreeMoveRefitRebuild", func() {
		treeTestSeed = 777

		tree := CreateDynamicTree(16)

		var boxes [treeTestProxyCount]AABB
		var proxies [treeTestProxyCount]int

		for i := 0; i < treeTestProxyCount; i++ {
			boxes[i] = treeTestRandomBox(6.0, 1.0)
			proxies[i] = tree.CreateProxy(boxes[i], DefaultCategoryBits, uint64(i))
		}

		tree.Rebuild(true)
		tree.Validate()
		ensure(tree.DfsOrdered)

		totalHits := 0
		for round := 0; round < 8; round++ {
			// Half the proxies move, half grow in place
			for i := 0; i < treeTestProxyCount; i++ {
				if (i & 1) == 0 {
					boxes[i] = treeTestRandomBox(6.0, 1.0)
					tree.MoveProxy(proxies[i], boxes[i])
				} else {
					grown := boxes[i].Inflate(0.35)
					boxes[i] = grown
					tree.EnlargeProxy(proxies[i], grown)
				}
			}

			tree.refit()
			tree.Validate()

			tree.Rebuild(false)
			tree.Validate()
			tree.ValidateNoMoved()
			ensure(tree.DfsOrdered)

			// Every stored box must still be inside its leaf and inside the root
			rootBounds := tree.GetRootBounds()
			for i := 0; i < treeTestProxyCount; i++ {
				ensure(tree.GetAABB(proxies[i]).Contains(boxes[i]))
				ensure(rootBounds.Contains(boxes[i]))
				ensure(int(int32(tree.GetUserData(proxies[i]))) == i)
			}

			queryBox := treeTestRandomBox(7.0, 3.0)

			var treeHits treeHitSet
			tree.Query(queryBox, DefaultMaskBits, false, treeCollectCallback, &treeHits)

			var bruteHits treeHitSet
			for i := 0; i < treeTestProxyCount; i++ {
				if boxes[i].Overlaps(queryBox) {
					bruteHits.ids[bruteHits.count] = i
					bruteHits.count += 1
				}
			}

			ensure(treeSameSet(&treeHits, &bruteHits) == 0)
			totalHits += treeHits.count
		}

		// Guard against the comparison above agreeing on nothing
		ensure(totalHits > 50)

		tree.Destroy()
	})

	// Destroying proxies in a scrambled order recycles sibling pairs through the free list and
	// leaves the array unordered, which is the path the fallback refit has to handle.
	It("TreeDestroyScrambled", func() {
		treeTestSeed = 4242

		tree := CreateDynamicTree(16)

		var boxes [treeTestProxyCount]AABB
		var proxies [treeTestProxyCount]int
		var alive [treeTestProxyCount]bool

		for i := 0; i < treeTestProxyCount; i++ {
			boxes[i] = treeTestRandomBox(10.0, 0.8)
			proxies[i] = tree.CreateProxy(boxes[i], DefaultCategoryBits, uint64(i))
			alive[i] = true
		}

		for i := 0; i < treeTestProxyCount; i += 3 {
			tree.DestroyProxy(proxies[i])
			alive[i] = false
		}

		tree.Validate()

		// Re-create into the holes
		for i := 0; i < treeTestProxyCount; i += 3 {
			boxes[i] = treeTestRandomBox(10.0, 0.8)
			proxies[i] = tree.CreateProxy(boxes[i], DefaultCategoryBits, uint64(i))
			alive[i] = true
		}

		tree.Validate()
		ensure(tree.GetProxyCount() == treeTestProxyCount)

		// The user data has to have followed each proxy through the churn
		for i := 0; i < treeTestProxyCount; i++ {
			ensure(alive[i])
			ensure(int(int32(tree.GetUserData(proxies[i]))) == i)
			ensure(tree.GetAABB(proxies[i]).Contains(boxes[i]))
		}

		queryBox := AABB{Vec3{-10.0, -10.0, -10.0}, Vec3{10.0, 10.0, 10.0}}
		var treeHits treeHitSet
		tree.Query(queryBox, DefaultMaskBits, false, treeCollectCallback, &treeHits)

		var bruteHits treeHitSet
		for i := 0; i < treeTestProxyCount; i++ {
			if boxes[i].Overlaps(queryBox) {
				bruteHits.ids[bruteHits.count] = i
				bruteHits.count += 1
			}
		}

		ensure(treeSameSet(&treeHits, &bruteHits) == 0)
		ensure(treeHits.count == treeTestProxyCount)

		// Empty the tree
		for i := 0; i < treeTestProxyCount; i++ {
			tree.DestroyProxy(proxies[i])
		}

		tree.Validate()
		ensure(tree.GetProxyCount() == 0)
		ensure(tree.GetHeight() == 0)

		// Queries on an empty tree must be silent
		var emptyHits treeHitSet
		tree.Query(queryBox, DefaultMaskBits, false, treeCollectCallback, &emptyHits)
		ensure(emptyHits.count == 0)

		tree.Destroy()
	})

	// The casts pop a pair, test both nodes and order the survivors. A ray that skips every hit
	// must still reach every proxy its segment crosses.
	It("TreeCastBruteForce", func() {
		treeTestSeed = 31337

		tree := CreateDynamicTree(16)

		var boxes [treeTestProxyCount]AABB
		var proxies [treeTestProxyCount]int

		for i := 0; i < treeTestProxyCount; i++ {
			boxes[i] = treeTestRandomBox(6.0, 1.0)
			proxies[i] = tree.CreateProxy(boxes[i], DefaultCategoryBits, uint64(i))
		}

		tree.Rebuild(true)
		tree.Validate()

		totalCrossings := 0
		for round := 0; round < 100; round++ {
			var origin, translation Vec3
			origin.X = treeTestRandomFloat(-8.0, 8.0)
			origin.Y = treeTestRandomFloat(-8.0, 8.0)
			origin.Z = treeTestRandomFloat(-8.0, 8.0)
			translation.X = treeTestRandomFloat(-16.0, 16.0)
			translation.Y = treeTestRandomFloat(-16.0, 16.0)
			translation.Z = treeTestRandomFloat(-16.0, 16.0)

			rayInput := RayCastInput{Origin: origin, Translation: translation, MaxFraction: 1.0}

			var treeHits treeHitSet
			tree.RayCast(&rayInput, DefaultMaskBits, false, treeRayCastCollectCallback, &treeHits)
			ensure(treeHits.bad == false)

			// The tree may report a superset because it tests the node box, not the shape. Every
			// proxy the segment actually crosses has to be in there.
			p2 := MulAdd(origin, 1.0, translation)
			for i := 0; i < treeTestProxyCount; i++ {
				if hit, _, _ := rayCastAABB(boxes[i], origin, p2); hit == false {
					continue
				}

				found := false
				for j := 0; j < treeHits.count; j++ {
					if treeHits.ids[j] == i {
						found = true
						break
					}
				}

				ensure(found)
				totalCrossings += 1
			}

			// Every reported proxy has to at least overlap the segment box
			segmentBox := AABB{Min(origin, p2), Max(origin, p2)}
			for j := 0; j < treeHits.count; j++ {
				ensure(boxes[treeHits.ids[j]].Overlaps(segmentBox))
			}

			// A zero extent box cast has to reach the same proxies as the ray
			castBox := AABB{origin, origin}
			boxInput := BoxCastInput{Box: castBox, Translation: translation, MaxFraction: 1.0}

			var boxHits treeHitSet
			tree.BoxCast(&boxInput, DefaultMaskBits, false, treeBoxCastCollectCallback, &boxHits)
			ensure(boxHits.bad == false)

			for j := 0; j < treeHits.count; j++ {
				found := false
				for k := 0; k < boxHits.count; k++ {
					if boxHits.ids[k] == treeHits.ids[j] {
						found = true
						break
					}
				}

				ensure(found)
			}
		}

		// Guard against the comparisons above agreeing on nothing
		ensure(totalCrossings > 100)

		tree.Destroy()
	})

	// The closest query orders by distance and prunes on the running minimum, so seeding it from
	// the root pair has to be right or it prunes away a whole half of the tree.
	It("TreeQueryClosest", func() {
		treeTestSeed = 5150

		tree := CreateDynamicTree(16)

		var boxes [treeTestProxyCount]AABB
		for i := 0; i < treeTestProxyCount; i++ {
			boxes[i] = treeTestRandomBox(6.0, 0.9)
			proxyId := tree.CreateProxy(boxes[i], DefaultCategoryBits, uint64(i))
			ensure(proxyId == i)
		}

		for round := 0; round < 100; round++ {
			var point Vec3
			point.X = treeTestRandomFloat(-8.0, 8.0)
			point.Y = treeTestRandomFloat(-8.0, 8.0)
			point.Z = treeTestRandomFloat(-8.0, 8.0)

			bruteBest := fltMax
			for i := 0; i < treeTestProxyCount; i++ {
				d := treeDistanceToBoxSqr(point, boxes[i])
				if d < bruteBest {
					bruteBest = d
				}
			}

			// The callback reports the exact box distance, so the pruned search has to land on the
			// same minimum as the scan. Same arithmetic both ways, so this is an exact compare.
			result := treeClosestResult{boxes: boxes[:], point: point}
			minDistanceSqr := fltMax
			tree.QueryClosest(point, DefaultMaskBits, false, treeClosestCollectCallback, &result, &minDistanceSqr)

			ensure(result.bad == false)
			ensure(minDistanceSqr == bruteBest)
		}

		tree.Destroy()
	})

	// A saved tree carries only its live node range.
	It("TreeSaveLoadRoundtrip", func() {
		treeTestSeed = 4242

		tree := CreateDynamicTree(16)

		var boxes [treeTestProxyCount]AABB
		var proxies [treeTestProxyCount]int

		for i := 0; i < treeTestProxyCount; i++ {
			boxes[i] = treeTestRandomBox(6.0, 1.0)
			proxies[i] = tree.CreateProxy(boxes[i], DefaultCategoryBits, uint64(i))
		}

		// Destroy a few so the pair free list is populated and the node tail is stale
		for i := 0; i < treeTestProxyCount; i += 37 {
			tree.DestroyProxy(proxies[i])
		}

		tree.Rebuild(true)
		tree.Validate()

		// PORT: the file goes in a temporary directory instead of the working directory.
		treePath := filepath.Join(GinkgoT().TempDir(), "test_dynamic_tree_roundtrip.dat")
		tree.Save(treePath)

		loaded := LoadDynamicTree(treePath, 1.0)

		ensure(loaded.Nodes != nil)
		ensure(loaded.NodeEnd == tree.NodeEnd)
		ensure(loaded.NodeCapacity == tree.NodeEnd)
		ensure(loaded.GetProxyCount() == tree.GetProxyCount())
		ensure(bytes.Equal(treeNodeBytes(loaded.Nodes, int(tree.NodeEnd)), treeNodeBytes(tree.Nodes, int(tree.NodeEnd))))
		ensure(bytes.Equal(treeParentBytes(loaded.Parents, int(tree.NodeEnd)), treeParentBytes(tree.Parents, int(tree.NodeEnd))))
		loaded.Validate()

		loaded.Destroy()
		tree.Destroy()
	})
})
