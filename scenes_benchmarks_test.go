// Port of shared/benchmarks.c (the junkyard scene and the ground shape id) and
// shared/overflow_color.h / shared/overflow_color.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "math"

// PORT: test scene helpers that test_world.c needs (CreateJunkyard / StepJunkyard from
// benchmarks.c, CreateOverflowColorPile from overflow_color.c), in a _test.go file of package
// box3d. Behind the box3d_m3_integration build tag until the milestone 3 world files are merged.
// The rest of benchmarks.c lands with the benchmarks (milestone 6). BENCHMARK_DEBUG follows
// debugBuild (NDEBUG is box3d_noassert).

// ---- benchmarks.c ----

// benchmarkDebug is BENCHMARK_DEBUG.
const benchmarkDebug = debugBuild

// benchmarkGroundShapeId is g_groundShapeId.
var benchmarkGroundShapeId = ShapeID{}

// getGroundShapeId is GetGroundShapeId.
func getGroundShapeId() ShapeID {
	return benchmarkGroundShapeId
}

// resetGroundShapeId is ResetGroundShapeId.
func resetGroundShapeId() {
	benchmarkGroundShapeId = NullShapeID
}

// junkyardData is struct JunkyardData g_junkyardData.
var junkyardData struct {
	pusherId BodyID
	degrees  float32
	radius   float32
}

// createJunkyard is CreateJunkyard.
func createJunkyard(worldId WorldID) {
	var groundId BodyID
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position.Y = -1.0
		groundId = worldId.CreateBody(&bodyDef)
	}

	{
		shapeDef := DefaultShapeDef()
		{
			box := MakeBoxHull(120.0, 1.0, 120.0)
			benchmarkGroundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
		}
		{
			offset := Vec3{-50.0, 8.0, 0.0}
			box := MakeOffsetBoxHull(1.0, 8.0, 50.0, offset)
			groundId.CreateHullShape(&shapeDef, &box.Base)
		}
		{
			offset := Vec3{50.0, 8.0, 0.0}
			box := MakeOffsetBoxHull(1.0, 8.0, 50.0, offset)
			groundId.CreateHullShape(&shapeDef, &box.Base)
		}
		{
			offset := Vec3{0.0, 8.0, -50.0}
			box := MakeOffsetBoxHull(50.0, 8.0, 1.0, offset)
			groundId.CreateHullShape(&shapeDef, &box.Base)
		}
		{
			offset := Vec3{0.0, 8.0, 50.0}
			box := MakeOffsetBoxHull(50.0, 8.0, 1.0, offset)
			groundId.CreateHullShape(&shapeDef, &box.Base)
		}
	}
	{
		rockHull := CreateRock(1.5)

		count := 24
		if benchmarkDebug {
			count = 2
		}
		height := float32(24.0)
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		shapeDef := DefaultShapeDef()
		for Y := range count {
			for X := 0; X <= 20; X++ {
				for Z := 0; Z <= 20; Z++ {
					bodyDef.Position.X = -40.0 + float32(4.0*float32(X))
					bodyDef.Position.Y = float32(4.0*float32(Y)) + height + 1.0
					bodyDef.Position.Z = -40.0 + float32(4.0*float32(Z))
					bodyId := worldId.CreateBody(&bodyDef)
					bodyId.CreateHullShape(&shapeDef, rockHull)
				}
			}
		}

		DestroyHull(rockHull)
	}

	junkyardData.radius = 35.0
	mHeight := float32(24.0)

	hull := CreateCylinder(mHeight, 4.0, 0.0, 16)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = KinematicBody
	bodyDef.Position = Pos{junkyardData.radius, 0.0, 0.0}
	junkyardData.pusherId = worldId.CreateBody(&bodyDef)
	junkyardData.degrees = 0.0
	shapeDef := DefaultShapeDef()
	junkyardData.pusherId.CreateHullShape(&shapeDef, hull)
	DestroyHull(hull)
}

// getJunkyardCapacity is GetJunkyardCapacity.
func getJunkyardCapacity(capacity *Capacity) {
	if !debugBuild {
		capacity.StaticShapeCount = 16
		capacity.DynamicShapeCount = 20*20*24 + 1
		capacity.StaticBodyCount = 16
		capacity.DynamicBodyCount = 20*20*24 + 1
		capacity.ContactCount = 250 * 1024
	} else {
		capacity.StaticShapeCount = 16
		capacity.DynamicShapeCount = 20*20*2 + 1
		capacity.StaticBodyCount = 16
		capacity.DynamicBodyCount = 20*20*2 + 1
	}
}

// stepJunkyard is StepJunkyard.
func stepJunkyard(worldId WorldID, stepCount int) {
	timeStep := float32(1.0 / 60.0)
	const omega float32 = -6.0
	junkyardData.degrees += float32(omega * timeStep)
	cs := ComputeCosSin(junkyardData.degrees * Pi / 180.0)
	r := junkyardData.radius
	targetPos := Pos{r * cs.Cosine, 0.0, r * cs.Sine}
	target := WorldTransform{P: targetPos, Q: QuatIdentity}
	junkyardData.pusherId.SetTargetTransform(target, timeStep, false)
}

// ---- overflow_color.h ----

// overflowColorPileData is OverflowColorPileData.
// One heavy dynamic hub surrounded by enough dynamic neighbors that the hub's
// degree in the dyn-dyn contact graph exceeds B3_DYNAMIC_COLOR_COUNT (= 20).
// The excess contacts land in the overflow color (B3_GRAPH_COLOR_COUNT - 1),
// exercising the b3*_Overflow solver path that no other scene reaches.
type overflowColorPileData struct {
	groundShapeId ShapeID
	hubId         BodyID
	neighborCount int
}

// ---- overflow_color.c ----

const (
	overflowPileRingCount = 5 // OVERFLOW_PILE_RING_COUNT
	overflowPilePerRing   = 5 // OVERFLOW_PILE_PER_RING
)

// createOverflowColorPile is CreateOverflowColorPile.
// PORT: cosf / sinf are float64 math.Cos / math.Sin rounded to float32.
func createOverflowColorPile(worldId WorldID) overflowColorPileData {
	data := overflowColorPileData{}
	data.neighborCount = overflowPileRingCount * overflowPilePerRing

	// Static ground (top surface at y = 0)
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, -1.0, 0.0}
		groundId := worldId.CreateBody(&bodyDef)

		box := MakeBoxHull(20.0, 1.0, 20.0)
		shapeDef := DefaultShapeDef()
		data.groundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	// Tall, heavy hub. Tall so neighbors can ring around it in multiple
	// vertical layers; heavy so it stays roughly still under uneven inward
	// pressure from the ring.
	hubHalfX := float32(0.5)
	hubHalfY := float32(2.5)
	hubHalfZ := float32(0.5)
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, hubHalfY, 0.0}
		data.hubId = worldId.CreateBody(&bodyDef)

		box := MakeBoxHull(hubHalfX, hubHalfY, hubHalfZ)
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 50.0
		data.hubId.CreateHullShape(&shapeDef, &box.Base)
	}

	// Neighbors: vertical rings around the hub, each box slightly overlapping
	// the hub so a contact exists on the very first step.
	neighborHalf := float32(0.2)
	ringRadius := hubHalfX + neighborHalf - 0.03

	neighborBox := MakeBoxHull(neighborHalf, neighborHalf, neighborHalf)
	neighborShape := DefaultShapeDef()

	ringSpacing := float32(0.5)
	baseY := neighborHalf + 0.05

	for ring := range overflowPileRingCount {
		y := baseY + float32(ringSpacing*float32(ring))

		// Offset alternate rings by half a slot so neighbors don't sit
		// directly above each other.
		thetaOffset := float32(0.0)
		if ring&1 != 0 {
			pi := Pi
			thetaOffset = pi / overflowPilePerRing
		}

		for slot := range overflowPilePerRing {
			twoPi := float32(2.0 * Pi)
			theta := thetaOffset + float32(twoPi*float32(slot))/overflowPilePerRing

			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			cosTheta := float32(math.Cos(float64(theta)))
			sinTheta := float32(math.Sin(float64(theta)))
			bodyDef.Position = Pos{float32(ringRadius * cosTheta), y, float32(ringRadius * sinTheta)}
			bodyId := worldId.CreateBody(&bodyDef)

			bodyId.CreateHullShape(&neighborShape, &neighborBox.Base)
		}
	}

	return data
}
