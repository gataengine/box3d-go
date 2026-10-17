// Port of the b3AABBV part of src/simd_scalar.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: upstream has a b3AABBV in every lane backend header (simd_scalar.h, simd_sse2.h,
// simd_neon.h, simd_avx2.h); the SSE2 and NEON versions load the AABB into two 128-bit registers
// and compute the same values as the scalar one. The Go port keeps the scalar version in every
// build (the spec's non-goal: port the SIMD ones only if a profile shows them), so these helpers
// are width independent and live outside the lane backend files.

// aabbV is b3AABBV.
type aabbV = AABB

// loadAABBV is b3LoadAABBV.
func loadAABBV(aabb *AABB) aabbV {
	return *aabb
}

// overlapAABBV is b3OverlapAABBV.
func overlapAABBV(a, b aabbV) bool {
	return a.LowerBound.X <= b.UpperBound.X && a.LowerBound.Y <= b.UpperBound.Y && a.LowerBound.Z <= b.UpperBound.Z &&
		b.LowerBound.X <= a.UpperBound.X && b.LowerBound.Y <= a.UpperBound.Y && b.LowerBound.Z <= a.UpperBound.Z
}

// overlapNode is b3OverlapNode.
func overlapNode(av aabbV, node *TreeNode) bool {
	return overlapAABBV(av, node.AABB)
}

// overlapV is b3OverlapV.
func overlapV(a, b *AABB) bool {
	return overlapAABBV(*a, *b)
}

// unionAABBV is b3UnionAABBV.
func unionAABBV(a, b aabbV) aabbV {
	return a.Union(b)
}

// unionPairV is b3UnionPairV.
// PORT: the node pointer becomes a slice holding the pair.
func unionPairV(pair []TreeNode) aabbV {
	return pair[0].AABB.Union(pair[1].AABB)
}

// storeAABBV is b3StoreAABBV.
func storeAABBV(aabb *AABB, value aabbV, condition bool) {
	if condition {
		*aabb = value
	}
}
