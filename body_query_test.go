// Port of test/test_body_query.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// The per-body query functions take an explicit world origin and a world body transform.
// Everything is re-centered on the origin so the float collision math stays accurate far from
// the world origin. These tests pin that framing: results come back in world space, the supplied
// transform drives the geometry (not the body's stored pose), and a large origin offset must not
// change a hit fraction or normal.
//
// CastRay and CastShape never touch the body's stored transform, so a static body at the origin
// holding local-frame shapes is enough. No step is needed for any query.

// bodyQueryCreateQueryWorld is CreateQueryWorld.
func bodyQueryCreateQueryWorld(bodyId *BodyID) WorldID {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)
	bodyDef := DefaultBodyDef()
	*bodyId = worldId.CreateBody(&bodyDef)
	return worldId
}

// bodyQueryIdentityAt is IdentityAt.
func bodyQueryIdentityAt(x, y, z float32) WorldTransform {
	return WorldTransform{P: Pos{x, y, z}, Q: QuatIdentity}
}

// bodyQueryCountOverlapCallback is CountOverlapCallback.
func bodyQueryCountOverlapCallback(shapeId ShapeID, context any) bool {
	_ = shapeId
	*context.(*int) += 1
	return true
}

// bodyQueryMakeStandingMover is MakeStandingMover.
// The mover capsule is expressed in the query frame, so a core segment starting at the origin
// stands the character on the query point. Targets sit on the sweep line at y = 0.
func bodyQueryMakeStandingMover() Capsule {
	return Capsule{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, 0.25}
}

// bodyQueryRandomFloat is RandomFloat.
func bodyQueryRandomFloat(seed *uint32, lower, upper float32) float32 {
	x := *seed
	x ^= x << 13
	x ^= x >> 17
	x ^= x << 5
	*seed = x
	unit := float32(x>>8) / 16777216.0
	return mulAdd(lower, upper-lower, unit)
}

// bodyQueryRandomVector is RandomVector.
func bodyQueryRandomVector(seed *uint32, lower, upper float32) Vec3 {
	var v Vec3
	v.X = bodyQueryRandomFloat(seed, lower, upper)
	v.Y = bodyQueryRandomFloat(seed, lower, upper)
	v.Z = bodyQueryRandomFloat(seed, lower, upper)
	return v
}

// bodyQueryRandomDirection is RandomDirection.
func bodyQueryRandomDirection(seed *uint32) Vec3 {
	for {
		v := bodyQueryRandomVector(seed, -1.0, 1.0)
		length := Length(v)
		if 0.1 < length && length < 1.0 {
			return MulSV(1.0/length, v)
		}
	}
}

var _ = Describe("BodyQueryTest", func() {
	// CastRay ----------------------------------------------------------------------------------

	It("CastRayHitsSphere", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}
		shapeDef := DefaultShapeDef()
		bodyId.CreateSphereShape(&shapeDef, &sphere)

		// Body sphere at world (5,0,0), ray straight at it along +X.
		bodyTransform := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		result := bodyId.CastRay(Pos{0.0, 0.0, 0.0}, Vec3{10.0, 0.0, 0.0}, DefaultQueryFilter(), 1.0, bodyTransform)

		ensure(result.Hit)
		ensure(result.ShapeId.IsValid())
		ensureSmall(result.Fraction-0.4, 1e-5)
		ensureSmall(result.Normal.X+1.0, 1e-5)
		ensureSmall(result.Normal.Y, 1e-5)
		ensureSmall(result.Normal.Z, 1e-5)

		point := ToVec3(result.Point)
		ensureSmall(point.X-4.0, 1e-4)
		ensureSmall(point.Y, 1e-4)
		ensureSmall(point.Z, 1e-4)

		worldId.Destroy()
	})

	It("CastRayMiss", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}
		shapeDef := DefaultShapeDef()
		bodyId.CreateSphereShape(&shapeDef, &sphere)

		// Ray runs parallel to the body, never reaching it.
		bodyTransform := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		result := bodyId.CastRay(Pos{0.0, 0.0, 0.0}, Vec3{0.0, 10.0, 0.0}, DefaultQueryFilter(), 1.0, bodyTransform)

		ensure(result.Hit == false)

		worldId.Destroy()
	})

	It("CastRayClosestShape", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		shapeDef := DefaultShapeDef()
		nearSphere := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}
		farSphere := Sphere{Vec3{4.0, 0.0, 0.0}, 1.0}
		nearId := bodyId.CreateSphereShape(&shapeDef, &nearSphere)
		bodyId.CreateSphereShape(&shapeDef, &farSphere)

		// Ray crosses both spheres; the loop must shrink maxFraction to the nearer hit.
		bodyTransform := bodyQueryIdentityAt(0.0, 0.0, 0.0)
		result := bodyId.CastRay(Pos{-5.0, 0.0, 0.0}, Vec3{10.0, 0.0, 0.0}, DefaultQueryFilter(), 1.0, bodyTransform)

		ensure(result.Hit)
		ensure(result.ShapeId.Index1 == nearId.Index1)
		ensure(result.ShapeId.Generation == nearId.Generation)
		ensureSmall(result.Fraction-0.4, 1e-5)

		worldId.Destroy()
	})

	It("CastRayRotatedBody", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		// Local center (0,2,0) rotated +90 deg about Z lands at world (-2,0,0).
		sphere := Sphere{Vec3{0.0, 2.0, 0.0}, 0.5}
		shapeDef := DefaultShapeDef()
		bodyId.CreateSphereShape(&shapeDef, &sphere)

		bodyTransform := WorldTransform{
			P: Pos{0.0, 0.0, 0.0},
			Q: MakeQuatFromAxisAngle(Vec3{0.0, 0.0, 1.0}, 0.5*Pi),
		}
		result := bodyId.CastRay(Pos{0.0, 0.0, 0.0}, Vec3{-4.0, 0.0, 0.0}, DefaultQueryFilter(), 1.0, bodyTransform)

		ensure(result.Hit)
		ensureSmall(result.Fraction-0.375, 1e-5)
		ensureSmall(result.Normal.X-1.0, 1e-5)

		point := ToVec3(result.Point)
		ensureSmall(point.X+1.5, 1e-4)

		worldId.Destroy()
	})

	It("CastRayFarFromOrigin", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}
		shapeDef := DefaultShapeDef()
		bodyId.CreateSphereShape(&shapeDef, &sphere)

		// Same geometry as CastRayHitsSphere shifted far from the world origin. The relative framing
		// keeps the subtraction exact, so fraction and normal must be unchanged.
		origin := Pos{1.0e6, -2.0e6, 5.0e5}
		bodyTransform := WorldTransform{P: OffsetPos(origin, Vec3{5.0, 0.0, 0.0}), Q: QuatIdentity}
		result := bodyId.CastRay(origin, Vec3{10.0, 0.0, 0.0}, DefaultQueryFilter(), 1.0, bodyTransform)

		ensure(result.Hit)
		ensureSmall(result.Fraction-0.4, 1e-5)
		ensureSmall(result.Normal.X+1.0, 1e-5)
		ensureSmall(result.Normal.Y, 1e-5)
		ensureSmall(result.Normal.Z, 1e-5)

		worldId.Destroy()
	})

	// CastShape --------------------------------------------------------------------------------

	It("CastShapeHitsBox", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(1.0, 1.0, 1.0)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		// Sphere proxy of radius 0.5 cast along +X into a box whose front face is at world x = 4.
		point := Vec3{0.0, 0.0, 0.0}
		proxy := ShapeProxy{[]Vec3{point}, 1, 0.5}
		bodyTransform := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		result := bodyId.CastShape(Pos{0.0, 0.0, 0.0}, &proxy, Vec3{10.0, 0.0, 0.0},
			DefaultQueryFilter(), 1.0, false, bodyTransform)

		// Front face at world x = 4. The fraction carries a small shape-cast skin, the contact point
		// and normal do not.
		ensure(result.Hit)
		ensure(result.ShapeId.IsValid())
		ensureSmall(result.Fraction-0.35, 1e-2)
		ensureSmall(result.Normal.X+1.0, 1e-4)

		hit := ToVec3(result.Point)
		ensureSmall(hit.X-4.0, 1e-3)

		worldId.Destroy()
	})

	It("CastShapeMiss", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(1.0, 1.0, 1.0)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		point := Vec3{0.0, 0.0, 0.0}
		proxy := ShapeProxy{[]Vec3{point}, 1, 0.5}
		bodyTransform := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		result := bodyId.CastShape(Pos{0.0, 0.0, 0.0}, &proxy, Vec3{0.0, 10.0, 0.0},
			DefaultQueryFilter(), 1.0, false, bodyTransform)

		ensure(result.Hit == false)

		worldId.Destroy()
	})

	It("CastShapeRotatedBody", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		// Body sphere local center (0,2,0) rotated +90 deg about Z lands at world (-2,0,0).
		sphere := Sphere{Vec3{0.0, 2.0, 0.0}, 1.0}
		shapeDef := DefaultShapeDef()
		bodyId.CreateSphereShape(&shapeDef, &sphere)

		point := Vec3{0.0, 0.0, 0.0}
		proxy := ShapeProxy{[]Vec3{point}, 1, 0.5}
		bodyTransform := WorldTransform{
			P: Pos{0.0, 0.0, 0.0},
			Q: MakeQuatFromAxisAngle(Vec3{0.0, 0.0, 1.0}, 0.5*Pi),
		}
		result := bodyId.CastShape(Pos{0.0, 0.0, 0.0}, &proxy, Vec3{-4.0, 0.0, 0.0},
			DefaultQueryFilter(), 1.0, false, bodyTransform)

		ensure(result.Hit)
		ensureSmall(result.Fraction-0.125, 1e-2)
		ensureSmall(result.Normal.X-1.0, 1e-4)

		hit := ToVec3(result.Point)
		ensureSmall(hit.X+1.0, 1e-3)

		worldId.Destroy()
	})

	It("CastShapeFarFromOrigin", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(1.0, 1.0, 1.0)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		point := Vec3{0.0, 0.0, 0.0}
		proxy := ShapeProxy{[]Vec3{point}, 1, 0.5}
		origin := Pos{1.0e6, -2.0e6, 5.0e5}
		bodyTransform := WorldTransform{P: OffsetPos(origin, Vec3{5.0, 0.0, 0.0}), Q: QuatIdentity}
		result := bodyId.CastShape(origin, &proxy, Vec3{10.0, 0.0, 0.0}, DefaultQueryFilter(),
			1.0, false, bodyTransform)

		ensure(result.Hit)
		ensureSmall(result.Fraction-0.35, 1e-2)
		ensureSmall(result.Normal.X+1.0, 1e-4)

		worldId.Destroy()
	})

	// OverlapShape -----------------------------------------------------------------------------

	It("OverlapTrue", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(1.0, 1.0, 1.0)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		// Proxy sits at the box center.
		point := Vec3{0.0, 0.0, 0.0}
		proxy := ShapeProxy{[]Vec3{point}, 1, 0.5}
		bodyTransform := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		overlaps := bodyId.OverlapShape(Pos{5.0, 0.0, 0.0}, &proxy, DefaultQueryFilter(), bodyTransform)

		ensure(overlaps)

		worldId.Destroy()
	})

	It("OverlapFalse", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(1.0, 1.0, 1.0)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		point := Vec3{0.0, 0.0, 0.0}
		proxy := ShapeProxy{[]Vec3{point}, 1, 0.5}
		bodyTransform := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		overlaps := bodyId.OverlapShape(Pos{20.0, 0.0, 0.0}, &proxy, DefaultQueryFilter(), bodyTransform)

		ensure(overlaps == false)

		worldId.Destroy()
	})

	It("OverlapRespectsBodyTransform", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(1.0, 1.0, 1.0)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		// Fixed proxy and origin: only the supplied transform decides the overlap.
		point := Vec3{0.0, 0.0, 0.0}
		proxy := ShapeProxy{[]Vec3{point}, 1, 0.5}
		origin := Pos{0.0, 0.0, 0.0}

		ensure(bodyId.OverlapShape(origin, &proxy, DefaultQueryFilter(), bodyQueryIdentityAt(0.0, 0.0, 0.0)))
		ensure(bodyId.OverlapShape(origin, &proxy, DefaultQueryFilter(), bodyQueryIdentityAt(20.0, 0.0, 0.0)) == false)

		worldId.Destroy()
	})

	It("OverlapFilter", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(1.0, 1.0, 1.0)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		point := Vec3{0.0, 0.0, 0.0}
		proxy := ShapeProxy{[]Vec3{point}, 1, 0.5}
		bodyTransform := bodyQueryIdentityAt(0.0, 0.0, 0.0)

		// Geometry overlaps, but a zero mask rejects every category.
		filter := DefaultQueryFilter()
		filter.MaskBits = 0
		overlaps := bodyId.OverlapShape(Pos{0.0, 0.0, 0.0}, &proxy, filter, bodyTransform)

		ensure(overlaps == false)

		worldId.Destroy()
	})

	// A box hull proxy built around a world target with a zero origin must hit the same shapes as the
	// same box built at the local origin and queried with the target as origin. This is the origin
	// relative equivalence the world query promises, and the pattern users reach for when they bake a
	// query box with b3MakeTransformedBoxHull.
	It("OverlapHullProxyEquivalence", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = Pos{10.0, 0.0, 0.0}
		bodyId := worldId.CreateBody(&bodyDef)
		box := MakeBoxHull(1.0, 1.0, 1.0)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		worldId.Step(1.0/60.0, 1)

		filter := DefaultQueryFilter()

		// Overlapping target: a 10 wide query box centered on the body.
		{
			offset := Vec3{10.0, 0.0, 0.0}

			baked := MakeTransformedBoxHull(5.0, 5.0, 5.0, Transform{offset, QuatIdentity})
			bakedProxy := ShapeProxy{baked.BoxPoints[:], baked.Base.VertexCount, 0.0}
			bakedHits := 0
			worldId.OverlapShape(PosZero, &bakedProxy, filter, bodyQueryCountOverlapCallback, &bakedHits)

			origin := OffsetPos(PosZero, offset)
			local := MakeBoxHull(5.0, 5.0, 5.0)
			localProxy := ShapeProxy{local.BoxPoints[:], local.Base.VertexCount, 0.0}
			localHits := 0
			worldId.OverlapShape(origin, &localProxy, filter, bodyQueryCountOverlapCallback, &localHits)

			ensure(bakedHits == 1)
			ensure(localHits == bakedHits)
		}

		// Clearing target: same box far from the body, both formulations agree on the miss.
		{
			offset := Vec3{100.0, 0.0, 0.0}

			baked := MakeTransformedBoxHull(5.0, 5.0, 5.0, Transform{offset, QuatIdentity})
			bakedProxy := ShapeProxy{baked.BoxPoints[:], baked.Base.VertexCount, 0.0}
			bakedHits := 0
			worldId.OverlapShape(PosZero, &bakedProxy, filter, bodyQueryCountOverlapCallback, &bakedHits)

			origin := OffsetPos(PosZero, offset)
			local := MakeBoxHull(5.0, 5.0, 5.0)
			localProxy := ShapeProxy{local.BoxPoints[:], local.Base.VertexCount, 0.0}
			localHits := 0
			worldId.OverlapShape(origin, &localProxy, filter, bodyQueryCountOverlapCallback, &localHits)

			ensure(bakedHits == 0)
			ensure(localHits == bakedHits)
		}

		worldId.Destroy()
	})

	// A quarter turn baked into the query hull must reach the overlap test. A long thin bar hits a body
	// off the origin when aligned along X and clears it once rotated to lie along Z.
	It("OverlapHullProxyRotation", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = Pos{3.0, 0.0, 0.0}
		bodyId := worldId.CreateBody(&bodyDef)
		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		worldId.Step(1.0/60.0, 1)

		filter := DefaultQueryFilter()

		// Bar long in local X, centered at the origin, reaches the body at x = 3.
		aligned := MakeTransformedBoxHull(4.0, 0.3, 0.3, TransformIdentity)
		alignedProxy := ShapeProxy{aligned.BoxPoints[:], aligned.Base.VertexCount, 0.0}
		alignedHits := 0
		worldId.OverlapShape(PosZero, &alignedProxy, filter, bodyQueryCountOverlapCallback, &alignedHits)
		ensure(alignedHits == 1)

		// Rotated a quarter turn about Y the long axis points along Z, so the bar no longer reaches x = 3.
		q := MakeQuatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, 0.5*Pi)
		turned := MakeTransformedBoxHull(4.0, 0.3, 0.3, Transform{Vec3Zero, q})
		turnedProxy := ShapeProxy{turned.BoxPoints[:], turned.Base.VertexCount, 0.0}
		turnedHits := 0
		worldId.OverlapShape(PosZero, &turnedProxy, filter, bodyQueryCountOverlapCallback, &turnedHits)
		ensure(turnedHits == 0)

		worldId.Destroy()
	})

	// A baked compound keeps its tree and its child placements in the compound frame while the query
	// proxy is in world space, so a compound on a turned body only answers correctly if the proxy is
	// brought back into that frame first.
	It("OverlapCompoundRotatedBody", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		mat := DefaultSurfaceMaterial()
		spheres := [2]CompoundSphereDef{
			{Sphere: Sphere{Vec3{-3.0, 0.0, 0.0}, 0.5}, Material: mat},
			{Sphere: Sphere{Vec3{3.0, 0.0, 0.0}, 0.5}, Material: mat},
		}
		compoundDef := CompoundDef{Spheres: spheres[:], SphereCount: 2}
		compound := CreateCompound(&compoundDef)
		ensure(compound != nil)

		pose := Transform{P: Vec3{10.0, 20.0, 30.0}, Q: MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)}

		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = Pos{pose.P.X, pose.P.Y, pose.P.Z}
		bodyDef.Rotation = pose.Q
		bodyId := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		bodyId.CreateBakedCompoundShape(&shapeDef, compound)

		filter := DefaultQueryFilter()

		// Sitting on the second child, which the body pose has swung onto the +Y side
		onChild := TransformPoint(pose, Vec3{3.0, 0.0, 0.0})
		hitProxy := ShapeProxy{[]Vec3{onChild}, 1, 0.1}

		hits := 0
		worldId.OverlapShape(PosZero, &hitProxy, filter, bodyQueryCountOverlapCallback, &hits)
		ensure(hits == 1)

		bodyTransform := WorldTransform{P: bodyDef.Position, Q: bodyDef.Rotation}
		ensure(bodyId.OverlapShape(PosZero, &hitProxy, filter, bodyTransform))

		// The gap between the two children stays empty
		inGap := TransformPoint(pose, Vec3{0.0, 0.0, 0.0})
		gapProxy := ShapeProxy{[]Vec3{inGap}, 1, 0.25}

		hits = 0
		worldId.OverlapShape(PosZero, &gapProxy, filter, bodyQueryCountOverlapCallback, &hits)
		ensure(hits == 0)
		ensure(bodyId.OverlapShape(PosZero, &gapProxy, filter, bodyTransform) == false)

		worldId.Destroy()
		DestroyCompound(compound)
	})

	// CollideMover -----------------------------------------------------------------------------

	It("MoverTouchesBox", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		// Mover core runs above the +Y face; its 0.2 radius reaches 0.1 into it.
		mover := Capsule{Vec3{-0.3, 0.6, 0.0}, Vec3{0.3, 0.6, 0.0}, 0.2}
		var planes [4]BodyPlaneResult
		bodyTransform := bodyQueryIdentityAt(0.0, 0.0, 0.0)
		count := bodyId.CollideMover(planes[:4], Pos{0.0, 0.0, 0.0}, &mover, DefaultQueryFilter(), bodyTransform)

		ensure(count == 1)
		ensure(planes[0].ShapeId.IsValid())
		ensure(IsNormalized(planes[0].Result.Plane.Normal))
		ensure(planes[0].Result.Plane.Normal.Y > 0.99)
		ensureSmall(planes[0].Result.Plane.Offset-0.1, 1e-4)

		worldId.Destroy()
	})

	It("MoverSeparated", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		mover := Capsule{Vec3{-0.3, 5.0, 0.0}, Vec3{0.3, 5.0, 0.0}, 0.2}
		var planes [4]BodyPlaneResult
		bodyTransform := bodyQueryIdentityAt(0.0, 0.0, 0.0)
		count := bodyId.CollideMover(planes[:4], Pos{0.0, 0.0, 0.0}, &mover, DefaultQueryFilter(), bodyTransform)

		ensure(count == 0)

		worldId.Destroy()
	})

	It("MoverRotatedBody", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		// Rotating +90 deg about X turns the local +Y face toward world +Z. The mover sits above the
		// world +Z face, so the returned normal must come back rotated into world space.
		mover := Capsule{Vec3{-0.3, 0.0, 0.6}, Vec3{0.3, 0.0, 0.6}, 0.2}
		var planes [4]BodyPlaneResult
		bodyTransform := WorldTransform{
			P: Pos{0.0, 0.0, 0.0},
			Q: MakeQuatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, 0.5*Pi),
		}
		count := bodyId.CollideMover(planes[:4], Pos{0.0, 0.0, 0.0}, &mover, DefaultQueryFilter(), bodyTransform)

		ensure(count == 1)
		ensure(IsNormalized(planes[0].Result.Plane.Normal))
		ensure(planes[0].Result.Plane.Normal.Z > 0.99)
		ensureSmall(planes[0].Result.Plane.Offset-0.1, 1e-4)

		worldId.Destroy()
	})

	It("MoverCapacity", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		// Two spheres each touch a mover that runs between them along X at y = 0.
		shapeDef := DefaultShapeDef()
		left := Sphere{Vec3{-0.4, 0.6, 0.0}, 0.5}
		right := Sphere{Vec3{0.4, 0.6, 0.0}, 0.5}
		bodyId.CreateSphereShape(&shapeDef, &left)
		bodyId.CreateSphereShape(&shapeDef, &right)

		mover := Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 0.2}
		var planes [4]BodyPlaneResult
		bodyTransform := bodyQueryIdentityAt(0.0, 0.0, 0.0)

		// Capacity caps the result and prevents writing past the buffer.
		capped := bodyId.CollideMover(planes[:1], Pos{0.0, 0.0, 0.0}, &mover, DefaultQueryFilter(), bodyTransform)
		ensure(capped == 1)

		full := bodyId.CollideMover(planes[:4], Pos{0.0, 0.0, 0.0}, &mover, DefaultQueryFilter(), bodyTransform)
		ensure(full == 2)

		worldId.Destroy()
	})

	// TimeOfImpactMover ------------------------------------------------------------------------

	It("MoverTOIHitsBox", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		shapeId := bodyId.CreateHullShape(&shapeDef, &box.Base)

		// Face at x = 4.5, mover radius 0.25, so 4.25 of the 10 unit sweep is free.
		mover := bodyQueryMakeStandingMover()
		bodyTransform := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		result := bodyId.TimeOfImpactMover(PosZero, &mover, Vec3{10.0, 0.0, 0.0},
			DefaultQueryFilter(), bodyTransform, bodyTransform)

		ensureSmall(result.Fraction-0.425, 1e-2)
		ensure(IsNormalized(result.Normal))
		ensureSmall(result.Normal.X+1.0, 1e-3)

		// The result carries a shape id, so the hit shape must come back identified.
		ensure(result.ShapeId.IsValid())
		ensure(result.ShapeId.Index1 == shapeId.Index1)
		ensure(result.ShapeId.Generation == shapeId.Generation)

		worldId.Destroy()
	})

	It("MoverTOISeparated", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		// Sweeping along +Y holds the X gap at 4.25 for the whole interval.
		mover := bodyQueryMakeStandingMover()
		bodyTransform := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		result := bodyId.TimeOfImpactMover(PosZero, &mover, Vec3{0.0, 10.0, 0.0},
			DefaultQueryFilter(), bodyTransform, bodyTransform)

		ensureSmall(result.Fraction-1.0, 1e-6)
		ensure(result.ShapeId.IsValid() == false)

		worldId.Destroy()
	})

	It("MoverTOIOverlapped", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		// Mover starts buried in the box, so there is no free interval to search.
		mover := bodyQueryMakeStandingMover()
		bodyTransform := bodyQueryIdentityAt(0.0, 0.0, 0.0)
		result := bodyId.TimeOfImpactMover(PosZero, &mover, Vec3{10.0, 0.0, 0.0},
			DefaultQueryFilter(), bodyTransform, bodyTransform)

		// Overlap should be ignored.
		ensureSmall(result.Fraction-1.0, 1e-6)
		ensure(result.ShapeId.IsNull())

		worldId.Destroy()
	})

	// Two shapes on the sweep line must resolve to the nearer one whatever order the shape list
	// hands them to the loop. Shapes are pushed on the head of the list, so the two bodies below
	// walk their shapes in opposite orders.
	It("MoverTOIClosestShape", func() {
		var nearFirstId BodyID
		worldId := bodyQueryCreateQueryWorld(&nearFirstId)

		bodyDef := DefaultBodyDef()
		nearLastId := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		nearSphere := Sphere{Vec3{5.0, 0.0, 0.0}, 0.5}
		farSphere := Sphere{Vec3{9.0, 0.0, 0.0}, 0.5}

		nearFirstHit := nearFirstId.CreateSphereShape(&shapeDef, &nearSphere)
		nearFirstId.CreateSphereShape(&shapeDef, &farSphere)

		nearLastId.CreateSphereShape(&shapeDef, &farSphere)
		nearLastHit := nearLastId.CreateSphereShape(&shapeDef, &nearSphere)

		mover := bodyQueryMakeStandingMover()
		bodyTransform := bodyQueryIdentityAt(0.0, 0.0, 0.0)
		translation := Vec3{10.0, 0.0, 0.0}

		nearFirst := nearFirstId.TimeOfImpactMover(PosZero, &mover, translation, DefaultQueryFilter(),
			bodyTransform, bodyTransform)
		nearLast := nearLastId.TimeOfImpactMover(PosZero, &mover, translation, DefaultQueryFilter(),
			bodyTransform, bodyTransform)

		ensureSmall(nearFirst.Fraction-0.425, 1e-2)
		ensure(nearFirst.ShapeId.Index1 == nearFirstHit.Index1)

		ensureSmall(nearLast.Fraction-nearFirst.Fraction, 1e-4)
		ensure(nearLast.ShapeId.Index1 == nearLastHit.Index1)

		worldId.Destroy()
	})

	// A shape the sweep clears must not erase a hit found earlier in the list. The off path sphere
	// is created first so the list hands it over last.
	It("MoverTOIKeepsHitAfterMiss", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		shapeDef := DefaultShapeDef()
		offPath := Sphere{Vec3{5.0, 0.0, 20.0}, 0.5}
		onPath := Sphere{Vec3{5.0, 0.0, 0.0}, 0.5}
		bodyId.CreateSphereShape(&shapeDef, &offPath)
		onPathId := bodyId.CreateSphereShape(&shapeDef, &onPath)

		mover := bodyQueryMakeStandingMover()
		bodyTransform := bodyQueryIdentityAt(0.0, 0.0, 0.0)
		result := bodyId.TimeOfImpactMover(PosZero, &mover, Vec3{10.0, 0.0, 0.0},
			DefaultQueryFilter(), bodyTransform, bodyTransform)

		ensureSmall(result.Fraction-0.425, 1e-2)
		ensure(result.ShapeId.Index1 == onPathId.Index1)

		worldId.Destroy()
	})

	// The mover capsule points live in the query frame, so sliding both the mover and the body by
	// the same offset must not move the impact.
	It("MoverTOIMoverOffset", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		translation := Vec3{10.0, 0.0, 0.0}
		filter := DefaultQueryFilter()

		atOrigin := bodyQueryMakeStandingMover()
		transformA := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		resultA := bodyId.TimeOfImpactMover(PosZero, &atOrigin, translation, filter, transformA, transformA)

		offset := Capsule{Vec3{2.0, 0.0, 0.0}, Vec3{2.0, 1.0, 0.0}, 0.25}
		transformB := bodyQueryIdentityAt(7.0, 0.0, 0.0)
		resultB := bodyId.TimeOfImpactMover(PosZero, &offset, translation, filter, transformB, transformB)

		ensureSmall(resultB.Fraction-resultA.Fraction, 1e-4)
		ensureSmall(resultB.Point.X-resultA.Point.X-2.0, 1e-3)

		worldId.Destroy()
	})

	// The body sweeps between the two supplied transforms. A bar spinning a quarter turn about Y
	// reaches a mover that its start pose clears.
	It("MoverTOIRotatingBody", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		bar := MakeBoxHull(2.0, 0.25, 0.25)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &bar.Base)

		// Query origin puts the mover at world (0,0,2.1), just inside the swept end of the bar.
		origin := Pos{0.0, 0.0, 2.1}
		mover := bodyQueryMakeStandingMover()
		transform1 := bodyQueryIdentityAt(0.0, 0.0, 0.0)
		transform2 := WorldTransform{
			P: Pos{0.0, 0.0, 0.0},
			Q: MakeQuatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, 0.5*Pi),
		}

		spinning := bodyId.TimeOfImpactMover(origin, &mover, Vec3Zero, DefaultQueryFilter(), transform1, transform2)

		ensure(0.0 < spinning.Fraction && spinning.Fraction < 1.0)

		// Holding the start pose leaves the bar along X and well clear.
		still := bodyId.TimeOfImpactMover(origin, &mover, Vec3Zero, DefaultQueryFilter(), transform1, transform1)

		ensureSmall(still.Fraction-1.0, 1e-6)
		ensure(still.ShapeId.IsNull())

		worldId.Destroy()
	})

	// Everything is re-centered on the origin, so a huge origin must not shift the fraction and the
	// hit point comes back in the origin frame.
	It("MoverTOIFarFromOrigin", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		origin := Pos{1.0e6, -2.0e6, 5.0e5}
		mover := bodyQueryMakeStandingMover()
		bodyTransform := WorldTransform{P: OffsetPos(origin, Vec3{5.0, 0.0, 0.0}), Q: QuatIdentity}
		result := bodyId.TimeOfImpactMover(origin, &mover, Vec3{10.0, 0.0, 0.0},
			DefaultQueryFilter(), bodyTransform, bodyTransform)

		ensureSmall(result.Fraction-0.425, 1e-2)
		ensureSmall(result.Normal.X+1.0, 1e-3)
		ensureSmall((result.Point.X-origin.X)-4.4, 0.5)

		worldId.Destroy()
	})

	It("MoverTOIFilter", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		box := MakeBoxHull(0.5, 0.5, 0.5)
		shapeDef := DefaultShapeDef()
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		// Geometry is on the sweep line, but a zero mask rejects every category.
		mover := bodyQueryMakeStandingMover()
		filter := DefaultQueryFilter()
		filter.MaskBits = 0
		bodyTransform := bodyQueryIdentityAt(5.0, 0.0, 0.0)
		result := bodyId.TimeOfImpactMover(PosZero, &mover, Vec3{10.0, 0.0, 0.0}, filter,
			bodyTransform, bodyTransform)

		ensureSmall(result.Fraction-1.0, 1e-6)

		worldId.Destroy()
	})

	// A shape the mover starts inside is skipped, but the sweep still runs against the other shapes
	// on the body.
	It("MoverTOIOverlapSkipsShape", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		shapeDef := DefaultShapeDef()

		buried := MakeBoxHull(0.5, 0.5, 0.5)
		bodyId.CreateHullShape(&shapeDef, &buried.Base)

		// Face at x = 4.5 like MoverTOIHitsBox
		onPath := MakeOffsetBoxHull(0.5, 0.5, 0.5, Vec3{5.0, 0.0, 0.0})
		onPathId := bodyId.CreateHullShape(&shapeDef, &onPath.Base)

		mover := bodyQueryMakeStandingMover()
		bodyTransform := bodyQueryIdentityAt(0.0, 0.0, 0.0)
		result := bodyId.TimeOfImpactMover(PosZero, &mover, Vec3{10.0, 0.0, 0.0},
			DefaultQueryFilter(), bodyTransform, bodyTransform)

		ensureSmall(result.Fraction-0.425, 1e-2)
		ensure(result.ShapeId.Index1 == onPathId.Index1)
		ensureSmall(result.Normal.X+1.0, 1e-3)

		worldId.Destroy()
	})

	// The mover sweep follows the solver's continuous rule: any time of impact strictly inside the
	// sweep counts, whatever the root finder reports. A capsule pivoting around a hull vertex can
	// exhaust the iteration cap, and that must not read as a clean miss. Compare against the raw
	// sweep built from the same inputs.
	It("MoverTOIMatchesSweep", func() {
		var bodyId BodyID
		worldId := bodyQueryCreateQueryWorld(&bodyId)

		bar := MakeBoxHull(2.0, 0.25, 0.25)
		shapeDef := DefaultShapeDef()
		shapeId := bodyId.CreateHullShape(&shapeDef, &bar.Base)

		// PORT: several calls below take two seed-advancing arguments, whose evaluation order C leaves
		// unspecified. Go evaluates left to right, as clang does, so the scenarios match a clang build.
		seed := uint32(0x9E3779B9)
		hitCount := 0
		for i := 0; i < 2000; i++ {
			var mover Capsule
			mover.Center1 = bodyQueryRandomVector(&seed, -0.2, 0.2)
			mover.Center2 = MulAdd(mover.Center1, bodyQueryRandomFloat(&seed, 0.05, 1.5), bodyQueryRandomDirection(&seed))
			mover.Radius = bodyQueryRandomFloat(&seed, 0.05, 0.5)
			capsulePoints := [2]Vec3{mover.Center1, mover.Center2}

			o := bodyQueryRandomVector(&seed, -3.5, 3.5)
			origin := Pos{o.X, o.Y, o.Z}
			translation := MulSV(bodyQueryRandomFloat(&seed, 0.0, 3.0), bodyQueryRandomDirection(&seed))

			transform1 := WorldTransform{
				P: PosZero,
				Q: MakeQuatFromAxisAngle(bodyQueryRandomDirection(&seed), bodyQueryRandomFloat(&seed, 0.0, Pi)),
			}
			spin := MakeQuatFromAxisAngle(bodyQueryRandomDirection(&seed), bodyQueryRandomFloat(&seed, 0.001, Pi))
			shift := bodyQueryRandomVector(&seed, -1.0, 1.0)
			transform2 := WorldTransform{
				P: Pos{shift.X, shift.Y, shift.Z},
				Q: MulQuat(spin, transform1.Q),
			}

			result := bodyId.TimeOfImpactMover(origin, &mover, translation, DefaultQueryFilter(),
				transform1, transform2)

			// Same sweep the body query builds. The static body keeps its center at the body origin.
			xf1 := ToRelativeTransform(transform1, origin)
			xf2 := ToRelativeTransform(transform2, origin)

			input := TOIInput{}
			input.ProxyA = ShapeProxy{GetHullPoints(&bar.Base), bar.Base.VertexCount, 0.0}
			input.ProxyB = ShapeProxy{capsulePoints[:], 2, mover.Radius}
			input.SweepA.C1 = xf1.P
			input.SweepA.C2 = xf2.P
			input.SweepA.Q1 = transform1.Q
			input.SweepA.Q2 = transform2.Q
			input.SweepA.LocalCenter = Vec3Zero
			input.SweepB.C1 = Vec3Zero
			input.SweepB.C2 = translation
			input.SweepB.Q1 = QuatIdentity
			input.SweepB.Q2 = QuatIdentity
			input.SweepB.LocalCenter = Vec3Zero
			input.MaxFraction = 1.0

			raw := TimeOfImpact(&input)

			if 0.0 < raw.Fraction && raw.Fraction < 1.0 {
				ensureSmall(result.Fraction-raw.Fraction, 1e-6)
				ensure(result.ShapeId.Index1 == shapeId.Index1)
				ensure(IsNormalized(result.Normal))
				ensureSmall(Distance(SubPos(result.Point, origin), raw.Point), 1e-4)
				hitCount += 1
			} else {
				ensureSmall(result.Fraction-1.0, 1e-6)
				ensure(result.ShapeId.IsNull())
			}
		}

		ensure(hitCount > 0)

		worldId.Destroy()
	})
})
