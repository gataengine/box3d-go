// Port of test/test_large_world.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// PORT: the #if defined( BOX3D_DOUBLE_PRECISION ) halves of LargeWorldStackTest,
// LargeWorldBulletTest and LargeWorldQueryTest are not ported (double precision is a non-goal), as
// upstream's float build compiles them out. The static helpers of test_large_world.c are prefixed
// with largeWorld so they cannot clash in the one test package.

const largeWorldStackCount = 6 // STACK_COUNT
const largeWorldMaxSteps = 400 // MAX_STEPS

// largeWorldStackResult is StackResult.
type largeWorldStackResult struct {
	// Final body positions relative to the base, so origin and far runs are directly comparable
	relativePositions [largeWorldStackCount]Vec3

	// First step on which the top body fell asleep, or -1 if it never settled
	sleepStep int
}

// largeWorldRunStack is RunStack.
// Drop a short stack of boxes onto a ground box centered at baseX. Records each body's final
// position relative to the base and the step on which the stack settles.
func largeWorldRunStack(baseX float32) largeWorldStackResult {
	base := Pos{baseX, 0.0, 0.0}

	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)

	groundDef := DefaultBodyDef()
	groundDef.Position = base
	groundId := worldId.CreateBody(&groundDef)
	groundBox := MakeBoxHull(10.0, 1.0, 10.0)
	groundShapeDef := DefaultShapeDef()
	groundId.CreateHullShape(&groundShapeDef, &groundBox.Base)

	var bodies [largeWorldStackCount]BodyID
	for i := 0; i < largeWorldStackCount; i++ {
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = OffsetPos(base, Vec3{0.0, 2.0 + float32(1.05*float32(i)), 0.0})
		bodies[i] = worldId.CreateBody(&bodyDef)

		box := MakeCubeHull(0.5)
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		bodies[i].CreateHullShape(&shapeDef, &box.Base)
	}

	result := largeWorldStackResult{}
	result.sleepStep = -1

	for step := 0; step < largeWorldMaxSteps; step++ {
		worldId.Step(1.0/60.0, 4)
		if result.sleepStep < 0 && bodies[largeWorldStackCount-1].IsAwake() == false {
			result.sleepStep = step
		}
	}

	for i := 0; i < largeWorldStackCount; i++ {
		p := bodies[i].GetPosition()
		result.relativePositions[i] = SubPos(p, base)
	}

	worldId.Destroy()
	return result
}

// largeWorldRunBullet is RunBullet.
// Fire a fast bullet at a thin wall. Returns the bullet's final x relative to the base. If
// continuous collision works the bullet stops at the wall instead of tunneling past it.
func largeWorldRunBullet(baseX float32) float32 {
	base := Pos{baseX, 0.0, 0.0}

	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)

	// Thin static wall at x = base + 5, spanning y and z
	wallDef := DefaultBodyDef()
	wallDef.Type = StaticBody
	wallDef.Position = OffsetPos(base, Vec3{5.0, 0.0, 0.0})
	wallId := worldId.CreateBody(&wallDef)
	wallBox := MakeBoxHull(0.05, 5.0, 5.0)
	wallShapeDef := DefaultShapeDef()
	wallId.CreateHullShape(&wallShapeDef, &wallBox.Base)

	// Small fast bullet aimed at the wall, no gravity
	bulletDef := DefaultBodyDef()
	bulletDef.Type = DynamicBody
	bulletDef.IsBullet = true
	bulletDef.GravityScale = 0.0
	bulletDef.Position = base
	bulletDef.LinearVelocity = Vec3{200.0, 0.0, 0.0}
	bulletId := worldId.CreateBody(&bulletDef)
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.1}
	bulletShapeDef := DefaultShapeDef()
	bulletShapeDef.Density = 1.0
	bulletId.CreateSphereShape(&bulletShapeDef, &sphere)

	for step := 0; step < 30; step++ {
		worldId.Step(1.0/60.0, 4)
	}

	relative := SubPos(bulletId.GetPosition(), base)
	worldId.Destroy()
	return relative.X
}

// largeWorldQueryResult is QueryResult.
type largeWorldQueryResult struct {
	castHit       bool
	castRelX      float32 // shape cast hit point x relative to the base
	overlapHit    bool
	moverFraction float32
	planeCount    int
	rayHit        bool
	rayRelX       float32 // world ray cast hit point x relative to the base
	shapeRayHit   bool
	shapeRayRelX  float32 // direct shape ray cast hit point x relative to the base
}

// largeWorldCastContext is CastContext.
type largeWorldCastContext struct {
	base Pos
	hit  bool
	relX float32
}

// largeWorldQueryCastCallback is QueryCastCallback.
func largeWorldQueryCastCallback(shapeId ShapeID, point Pos, normal Vec3, fraction float32, materialId uint64,
	triangleIndex int, childIndex int, context any) float32 {
	ctx := context.(*largeWorldCastContext)
	ctx.hit = true
	ctx.relX = SubPos(point, ctx.base).X
	return fraction
}

// largeWorldQueryOverlapCallback is QueryOverlapCallback.
func largeWorldQueryOverlapCallback(shapeId ShapeID, context any) bool {
	*context.(*bool) = true
	return true
}

// largeWorldQueryPlaneCallback is QueryPlaneCallback.
func largeWorldQueryPlaneCallback(shapeId ShapeID, planes []PlaneResult, context any) bool {
	*context.(*int) += len(planes)
	return true
}

// largeWorldRunQueries is RunQueries.
// Run the four origin relative spatial queries against a static box centered at the base. The query
// inputs are all relative to the base, so passing base as the origin keeps them precise far out.
func largeWorldRunQueries(baseX float32) largeWorldQueryResult {
	base := Pos{baseX, 0.0, 0.0}

	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = StaticBody
	bodyDef.Position = base
	bodyId := worldId.CreateBody(&bodyDef)
	box := MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := DefaultShapeDef()
	shapeId := bodyId.CreateHullShape(&shapeDef, &box.Base)

	worldId.Step(1.0/60.0, 1)

	result := largeWorldQueryResult{}

	// Sphere proxy swept from the left into the box, hitting the left face at relative x = -1
	castPoint := Vec3{-5.0, 0.0, 0.0}
	castProxy := ShapeProxy{[]Vec3{castPoint}, 1, 0.25}
	castCtx := largeWorldCastContext{base, false, 0.0}
	worldId.CastShape(base, &castProxy, Vec3{10.0, 0.0, 0.0}, DefaultQueryFilter(), largeWorldQueryCastCallback,
		&castCtx)
	result.castHit = castCtx.hit
	result.castRelX = castCtx.relX

	// Sphere proxy sitting at the box center
	overlapPoint := Vec3{0.0, 0.0, 0.0}
	overlapProxy := ShapeProxy{[]Vec3{overlapPoint}, 1, 0.5}
	overlapHit := false
	worldId.OverlapShape(base, &overlapProxy, DefaultQueryFilter(), largeWorldQueryOverlapCallback, &overlapHit)
	result.overlapHit = overlapHit

	// Capsule swept from the left into the box
	moverCast := Capsule{Vec3{-5.0, -0.3, 0.0}, Vec3{-5.0, 0.3, 0.0}, 0.25}
	result.moverFraction =
		worldId.CastMover(base, &moverCast, Vec3{10.0, 0.0, 0.0}, DefaultQueryFilter(), nil, nil)

	// Capsule overlapping the left face, should report a contact plane
	moverCollide := Capsule{Vec3{-1.1, -0.3, 0.0}, Vec3{-1.1, 0.3, 0.0}, 0.3}
	planeCount := 0
	worldId.CollideMover(base, &moverCollide, DefaultQueryFilter(), largeWorldQueryPlaneCallback, &planeCount)
	result.planeCount = planeCount

	// World ray cast from the left into the box, hitting the left face at relative x = -1
	rayOrigin := OffsetPos(base, Vec3{-5.0, 0.0, 0.0})
	ray := worldId.CastRayClosest(rayOrigin, Vec3{10.0, 0.0, 0.0}, DefaultQueryFilter())
	result.rayHit = ray.Hit
	if ray.Hit {
		result.rayRelX = SubPos(ray.Point, base).X
	} else {
		result.rayRelX = 0.0
	}

	// Direct shape ray cast against the same box
	shapeRay := shapeId.RayCast(rayOrigin, Vec3{10.0, 0.0, 0.0})
	result.shapeRayHit = shapeRay.Hit
	if shapeRay.Hit {
		result.shapeRayRelX = SubPos(shapeRay.Point, base).X
	} else {
		result.shapeRayRelX = 0.0
	}

	worldId.Destroy()
	return result
}

var _ = Describe("LargeWorldTest", func() {
	// A stack at the origin and a stack far from the origin should behave identically in double
	// precision: same settle frame, same relative geometry.
	It("LargeWorldStackTest", func() {
		origin := largeWorldRunStack(0.0)
		ensure(origin.sleepStep >= 0)
	})

	// The bullet must be caught by the wall, not tunnel through it. The blocking check is that this
	// still holds far from the origin where the swept query box rounds back to float with large ULP.
	It("LargeWorldBulletTest", func() {
		// Wall front face is at x = 5 - 0.05; the bullet radius is 0.1, so a caught bullet stays well
		// short of the wall center at x = 5.
		originX := largeWorldRunBullet(0.0)
		ensure(originX < 5.0)
	})

	// The origin relative queries hit at the origin and stay precise far from it: same hit geometry and
	// sweep fraction whether the box is at the origin or at 1e7.
	It("LargeWorldQueryTest", func() {
		origin := largeWorldRunQueries(0.0)
		ensure(origin.castHit)
		ensure(origin.overlapHit)
		ensure(origin.moverFraction < 1.0)
		ensure(origin.planeCount > 0)
		ensureSmall(origin.castRelX+1.0, 0.05)
		ensure(origin.rayHit)
		ensureSmall(origin.rayRelX+1.0, 0.05)
		ensure(origin.shapeRayHit)
		ensureSmall(origin.shapeRayRelX+1.0, 0.05)
	})
})
