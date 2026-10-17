// Port of test/test_collision.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// PORT: the #if defined( BOX3D_DOUBLE_PRECISION ) halves of LargeWorldManifoldTest and
// LargeWorldAABBTest are not ported (double precision is a non-goal), as upstream's float build
// compiles them out.

var _ = Describe("CollisionTest", func() {
	It("AABBTest", func() {
		var a AABB
		a.LowerBound = Vec3{-1.0, -1.0, -1.0}
		a.UpperBound = Vec3{-2.0, -2.0, -2.0}

		ensure(IsValidAABB(a) == false)

		a.UpperBound = Vec3{1.0, 1.0, 0.0}
		ensure(IsValidAABB(a) == true)

		b := AABB{Vec3{2.0, 2.0, 0.0}, Vec3{4.0, 4.0, 0.0}}
		ensure(a.Overlaps(b) == false)
		ensure(a.Contains(b) == false)
	})

	It("TestRayAABBIntersection", func() {
		// Test 1: Ray passing through center of AABB
		{
			a := AABB{Vec3{-1.0, -1.0, -1.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{-2.0, 0.0, 0.0}
			p2 := Vec3{2.0, 0.0, 0.0}

			hit, minFraction, maxFraction := rayCastAABB(a, p1, p2)

			ensure(hit == true)
			ensure(AbsFloat(minFraction-0.25) < 0.001) // Enters at 25% of ray
			ensure(AbsFloat(maxFraction-0.75) < 0.001) // Exits at 75% of ray
		}

		// Test 2: Ray starting inside AABB
		{
			a := AABB{Vec3{-1.0, -1.0, -1.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{0.0, 0.0, 0.0}
			p2 := Vec3{2.0, 0.0, 0.0}

			hit, minFraction, maxFraction := rayCastAABB(a, p1, p2)

			ensure(hit == true)
			ensure(minFraction == 0.0)                // Starts inside
			ensure(AbsFloat(maxFraction-0.5) < 0.001) // Exits at 50% of ray
		}

		// Test 3: Ray ending inside AABB
		{
			a := AABB{Vec3{-1.0, -1.0, -1.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{-2.0, 0.0, 0.0}
			p2 := Vec3{0.0, 0.0, 0.0}

			hit, minFraction, maxFraction := rayCastAABB(a, p1, p2)

			ensure(hit == true)
			ensure(AbsFloat(minFraction-0.5) < 0.001) // Enters at 50% of ray
			ensure(maxFraction == 1.0)                // Ends inside
		}

		// Test 4: Ray completely inside AABB
		{
			a := AABB{Vec3{-2.0, -2.0, -2.0}, Vec3{2.0, 2.0, 2.0}}
			p1 := Vec3{-1.0, 0.0, 0.0}
			p2 := Vec3{1.0, 0.0, 0.0}

			hit, minFraction, maxFraction := rayCastAABB(a, p1, p2)

			ensure(hit == true)
			ensure(minFraction == 0.0)
			ensure(maxFraction == 1.0)
		}

		// Test 5: Ray missing AABB
		{
			a := AABB{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{-1.0, 2.0, 0.5}
			p2 := Vec3{2.0, 2.0, 0.5}

			hit, _, _ := rayCastAABB(a, p1, p2)

			ensure(hit == false)
		}

		// Test 6: Ray parallel to AABB face (no intersection)
		{
			a := AABB{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{-1.0, 2.0, 0.5}
			p2 := Vec3{2.0, 2.0, 0.5}

			hit, _, _ := rayCastAABB(a, p1, p2)

			ensure(hit == false)
		}

		// Test 7: Ray parallel to AABB face (within bounds)
		{
			a := AABB{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{-1.0, 0.5, 0.5}
			p2 := Vec3{2.0, 0.5, 0.5}

			hit, minFraction, maxFraction := rayCastAABB(a, p1, p2)

			ensure(hit == true)
			ensure(AbsFloat(minFraction-float32(1.0)/3.0) < 0.001)
			ensure(AbsFloat(maxFraction-float32(2.0)/3.0) < 0.001)
		}

		// Test 8: Degenerate ray (point) inside AABB
		{
			a := AABB{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{0.5, 0.5, 0.5}
			p2 := Vec3{0.5, 0.5, 0.5}

			hit, minFraction, maxFraction := rayCastAABB(a, p1, p2)

			ensure(hit == true)
			ensure(minFraction == 0.0)
			ensure(maxFraction == 0.0)
		}

		// Test 9: Degenerate ray (point) outside AABB
		{
			a := AABB{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{2.0, 2.0, 2.0}
			p2 := Vec3{2.0, 2.0, 2.0}

			hit, _, _ := rayCastAABB(a, p1, p2)

			ensure(hit == false)
		}

		// Test 10: Ray pointing away from AABB
		{
			a := AABB{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{-1.0, 0.5, 0.5}
			p2 := Vec3{-2.0, 0.5, 0.5}

			hit, _, _ := rayCastAABB(a, p1, p2)

			ensure(hit == false)
		}

		// Test 11: Ray hitting corner of AABB
		{
			a := AABB{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{-1.0, -1.0, -1.0}
			p2 := Vec3{2.0, 2.0, 2.0}

			hit, minFraction, maxFraction := rayCastAABB(a, p1, p2)

			ensure(hit == true)
			ensure(AbsFloat(minFraction-float32(1.0)/3.0) < 0.001)
			ensure(AbsFloat(maxFraction-float32(2.0)/3.0) < 0.001)
		}

		// Test 12: Ray grazing edge of AABB
		{
			a := AABB{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 1.0, 1.0}}
			p1 := Vec3{-1.0, 0.0, 0.5}
			p2 := Vec3{2.0, 0.0, 0.5}

			hit, minFraction, maxFraction := rayCastAABB(a, p1, p2)

			ensure(hit == true)
			ensure(AbsFloat(minFraction-float32(1.0)/3.0) < 0.001)
			ensure(AbsFloat(maxFraction-float32(2.0)/3.0) < 0.001)
		}
	})

	// The narrow phase differences the two world positions in double then works in frame A, so a
	// manifold far from the origin must match the same manifold at the origin. Float loses this past
	// ~1e7 m where the ULP grows larger than the overlap, which is the whole point of large world mode.
	It("LargeWorldManifoldTest", func() {
		boxA := MakeBoxHull(0.5, 0.5, 0.5)
		boxB := MakeBoxHull(0.5, 0.5, 0.5)

		// Centers 0.9 apart so the cubes overlap by 0.1 along x
		sep := Vec3{0.9, 0.0, 0.0}

		var pointsOrigin [8]LocalManifoldPoint
		mOrigin := LocalManifold{}
		mOrigin.Points = pointsOrigin[:]

		xfAo := WorldTransformIdentity
		xfBo := WorldTransform{OffsetPos(PosZero, sep), QuatIdentity}
		cacheOrigin := SATCache{}
		CollideHulls(&mOrigin, 8, &boxA.Base, &boxB.Base, InvMulWorldTransforms(xfAo, xfBo), &cacheOrigin)

		// Two cube faces overlap, so the clipped manifold has four points
		ensure(mOrigin.PointCount == 4)
		for i := 0; i < int(mOrigin.PointCount); i++ {
			ensureSmall(mOrigin.Points[i].Separation+0.1, 0.01)
		}
	})

	// Broad-phase AABBs are built in double and narrowed to float with directed outward rounding, so a
	// shape and its speculative margin stay inside their box far from the origin. A float build would
	// round the extent away into the ULP (~1 m at 1e7) and clip the shape out of its own box.
	It("LargeWorldAABBTest", func() {
		// Unit cube, so the tight extent is 0.5 each way
		box := MakeBoxHull(0.5, 0.5, 0.5)
		shape := shape{}
		shape.typ = HullShape
		shape.hull = &box.Base

		aabbOrigin := computeFatShapeAABB(&shape, WorldTransformIdentity, 0.0)
		ensureSmall(aabbOrigin.LowerBound.X+0.5, fltEpsilon)
		ensureSmall(aabbOrigin.LowerBound.Y+0.5, fltEpsilon)
		ensureSmall(aabbOrigin.LowerBound.Z+0.5, fltEpsilon)
		ensureSmall(aabbOrigin.UpperBound.X-0.5, fltEpsilon)
		ensureSmall(aabbOrigin.UpperBound.Y-0.5, fltEpsilon)
		ensureSmall(aabbOrigin.UpperBound.Z-0.5, fltEpsilon)
	})
})
