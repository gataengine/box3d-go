// Port of test/test_recording.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"os"
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
)

// PORT: the static helpers of test_recording.c are prefixed with recTest so they cannot clash in the
// one test package. b3Alloc/b3Free scratch arrays are Go slices; memcpy/memcmp of raw bytes are
// copy/bytes.Equal (the host is little endian, as upstream assumes). const char* NULL is the empty
// string. b3RecPlayer* NULL is a nil *RecPlayer. The It blocks follow the RecordingTest() order.

const recTestRecPath = "recording_allops_test.b3rec" // s_recPath

// recTestValidateReplayAtWidth is ValidateReplayAtWidth.
func recTestValidateReplayAtWidth(rec *Recording, width int) int {
	SetSIMDWidth(width)
	valid := ValidateReplay(rec.GetData()[:rec.GetSize()], 1)
	SetSIMDWidth(0)
	if valid {
		return 0
	}
	return 1
}

// recTestDebugShapeCounters is DebugShapeCounters.
// Host debug-shape callbacks just count create/destroy so the test can prove the player
// wires them into every world it builds. The returned token is opaque to the engine.
type recTestDebugShapeCounters struct {
	created   int
	destroyed int
}

// recTestCreateDebugShape is RecTestCreateDebugShape.
func recTestCreateDebugShape(debugShape *DebugShape, userContext any) any {
	counters := userContext.(*recTestDebugShapeCounters)
	counters.created += 1
	return userContext // any non-NULL token; engine stores and hands it back to destroy
}

// recTestDestroyDebugShape is RecTestDestroyDebugShape.
func recTestDestroyDebugShape(userShape any, userContext any) {
	counters := userContext.(*recTestDebugShapeCounters)
	counters.destroyed += 1
}

// recTestDrawShape is RecTestDrawShape.
func recTestDrawShape(userShape any, transform WorldTransform, color HexColor, context any) {
}

// recTestDrawWorld is RecTestDrawWorld.
// b3World_Draw lazily fires createDebugShape for shapes entering the draw set, the same way the
// sample renderer does. Drive a draw so the player's wired callbacks actually run.
func recTestDrawWorld(worldId WorldID) {
	draw := DefaultDebugDraw()
	draw.DrawShapeFcn = recTestDrawShape
	draw.DrawShapes = true
	big := float32(1.0e6)
	draw.DrawingBounds = AABB{Vec3{-big, -big, -big}, Vec3{big, big, big}}
	worldId.Draw(&draw, DefaultMaskBits)
}

// recTestQueryReplayOverlapFcn is QueryReplayOverlapFcn.
func recTestQueryReplayOverlapFcn(shapeId ShapeID, context any) bool {
	return true
}

// recTestQueryReplayCastFcn is QueryReplayCastFcn.
func recTestQueryReplayCastFcn(shapeId ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64,
	triangleIndex int, childIndex int, context any) float32 {
	// Return the fraction to keep the closest hit, exercising the recorded user-return path.
	return fraction
}

// recTestCompoundHitCastFcn is CompoundHitCastFcn.
func recTestCompoundHitCastFcn(shapeId ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64,
	triangleIndex int, childIndex int, context any) float32 {
	if shapeId.GetType() == CompoundShape {
		*context.(*bool) = true
	}
	return 1.0
}

// recTestQueryReplayPlaneFcn is QueryReplayPlaneFcn.
func recTestQueryReplayPlaneFcn(shapeId ShapeID, planes []PlaneResult, context any) bool {
	return true
}

// recTestQueryReplayMoverFilterFcn is QueryReplayMoverFilterFcn.
func recTestQueryReplayMoverFilterFcn(shapeId ShapeID, context any) bool {
	return true
}

// recTestReplaySafetyFactor is ReplaySafetyFactor.
// Safety factor of the named body in a replay world, or -1 if the body is not in the list
func recTestReplaySafetyFactor(player *RecPlayer, name string) float32 {
	count := player.GetBodyCount()
	for i := 0; i < count; i++ {
		bodyId := player.GetBodyId(i)
		if bodyId.IsValid() && bodyId.GetName() == name {
			return bodyId.GetSafetyFactor()
		}
	}

	return -1.0
}

// recTestRunGeometryMutatorScene is RunGeometryMutatorScene.
// A box sliding across a mesh floor, with the option to swap both geometries and retune a
// per-triangle material part way through. Returns the final state hash so the caller can prove the
// mutations move the simulation. Recording is optional so the same scene serves as the control.
func recTestRunGeometryMutatorScene(rec *Recording, mutate bool, meshA *MeshData, meshB *MeshData,
	swapHull *HullData, swapFriction float32) uint64 {
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = 1
	worldId := CreateWorld(&worldDef)

	if rec != nil {
		worldId.StartRecording(rec)
	}

	// The mesh body is created first so its ordinal is stable for the read back after replay.
	meshMaterials := [2]SurfaceMaterial{DefaultSurfaceMaterial(), DefaultSurfaceMaterial()}
	meshMaterials[1].Friction = 0.05

	meshBodyDef := DefaultBodyDef()
	meshBodyDef.Type = StaticBody
	meshBodyId := worldId.CreateBody(&meshBodyDef)

	meshShapeDef := DefaultShapeDef()
	meshShapeDef.Materials = meshMaterials[:]
	meshShapeDef.MaterialCount = 2
	meshShapeId := meshBodyId.CreateMeshShape(&meshShapeDef, meshA, Vec3{1.0, 1.0, 1.0})

	boxBodyDef := DefaultBodyDef()
	boxBodyDef.Type = DynamicBody
	boxBodyDef.Position = Pos{-2.0, 2.0, 0.0}
	boxBodyDef.LinearVelocity = Vec3{4.0, 0.0, 0.0}
	boxBodyId := worldId.CreateBody(&boxBodyDef)

	box := MakeBoxHull(0.5, 0.5, 0.5)
	boxShapeDef := DefaultShapeDef()
	boxShapeDef.Density = 1.0
	boxShapeId := boxBodyId.CreateHullShape(&boxShapeDef, &box.Base)

	timeStep := float32(1.0 / 60.0)
	for i := 0; i < 10; i++ {
		worldId.Step(timeStep, 4)
	}

	if mutate {
		boxShapeId.SetHull(swapHull)
		meshShapeId.SetMesh(meshB, Vec3{1.0, 1.0, 1.0})

		grippy := DefaultSurfaceMaterial()
		grippy.Friction = swapFriction
		meshShapeId.SetMeshMaterial(grippy, 1)
	}

	for i := 0; i < 30; i++ {
		worldId.Step(timeStep, 4)
	}

	hash := hashWorldState(getWorldFromId(worldId))

	if rec != nil {
		worldId.StopRecording()
	}
	worldId.Destroy()

	return hash
}

var _ = Describe("RecordingTest", func() {
	// Geometry that shares a content hash but differs in bytes must dedup exactly. Mirrors the keyframe
	// flow that crashed: the seed appends every slot 1:1 (even byte-identical duplicates a hash collision
	// left in an already-recorded file), then capture must resolve a live blob back to an existing slot
	// without growing the registry. Forces the collision by handing the same hash to distinct blobs.
	It("GeometryHashCollision", func() {
		const n = 16
		const sharedHash uint64 = 0xABCD1234

		reg := geometryRegistry{}

		var blobA [n]byte
		var blobB [n]byte
		recTestMemset(blobA[:], 0xAA)
		recTestMemset(blobB[:], 0xBB)

		// Distinct blobs colliding on the hash must become two entries, not a false dedup.
		idA := internGeometry(&reg, geometryHull, sharedHash, blobA[:])
		idB := internGeometry(&reg, geometryHull, sharedHash, blobB[:])
		ensure(idA != idB)
		ensure(len(reg.entries) == 2)

		// Re-interning either blob must find it through the hash chain and never grow the registry,
		// including the one shadowed behind the bucket head. The old single-entry lookup missed the
		// shadowed blob and appended a duplicate, which is exactly what tripped the keyframe assert.
		var blobA2 [n]byte
		recTestMemset(blobA2[:], 0xAA)
		ensure(internGeometry(&reg, geometryHull, sharedHash, blobA2[:]) == idA)
		ensure(len(reg.entries) == 2)

		var blobB2 [n]byte
		recTestMemset(blobB2[:], 0xBB)
		ensure(internGeometry(&reg, geometryHull, sharedHash, blobB2[:]) == idB)
		ensure(len(reg.entries) == 2)

		freeRegistry(&reg)

		// Seed-then-capture: appending byte-identical duplicate slots keeps id == slot index, and a later
		// exact intern still resolves to one of them without appending a new entry.
		seeded := geometryRegistry{}
		var slot0 [n]byte
		var slot1 [n]byte
		var slot2 [n]byte
		recTestMemset(slot0[:], 0xAA)
		recTestMemset(slot1[:], 0xBB)
		recTestMemset(slot2[:], 0xAA) // duplicate of slot0
		ensure(appendGeometry(&seeded, geometryHull, sharedHash, slot0[:]) == 0)
		ensure(appendGeometry(&seeded, geometryHull, sharedHash, slot1[:]) == 1)
		ensure(appendGeometry(&seeded, geometryHull, sharedHash, slot2[:]) == 2)

		var live [n]byte
		recTestMemset(live[:], 0xAA)
		resolved := internGeometry(&seeded, geometryHull, sharedHash, live[:])
		ensure(len(seeded.entries) == 3)       // no growth
		ensure(resolved == 0 || resolved == 2) // a valid slot index for that content
		ensure(seeded.entries[resolved].byteCount == n)
		ensure(bytes.Equal(seeded.entries[resolved].bytes[:n], slot0[:]))

		freeRegistry(&seeded)
	})

	// Shape names are debug only and do not feed the determinism hash, so b3ValidateReplay cannot catch a
	// broken name round-trip. Replay through the player and read the names back to prove the def field and
	// the ShapeSetName op survive serialization.
	It("ShapeNameReplay", func() {
		names := [3]string{
			"def",
			"set",
			"abcdefghijklmnopqrstuvwxyz",
		}

		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		worldId.StartRecording(rec)

		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}

		// One shape per body so read-back never depends on per-body shape ordering.
		for i := 0; i < 3; i++ {
			bd := DefaultBodyDef()
			bd.Type = DynamicBody
			bd.Position = Pos{3.0 * float32(i), 5.0, 0.0}
			body := worldId.CreateBody(&bd)

			sd := DefaultShapeDef()
			sd.Density = 1.0
			if i != 1 {
				sd.Name = names[i]
			}
			shape := body.CreateSphereShape(&sd, &sphere)
			if i == 1 {
				shape.SetName(names[i])
			}
		}

		for i := 0; i < 4; i++ {
			worldId.Step(1.0/60.0, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()

		data := rec.GetData()
		sz := rec.GetSize()

		player := CreatePlayer(data[:sz], 1)
		ensure(player != nil)
		for player.StepFrame() {
		}
		ensure(player.HasDiverged() == false)

		// Body ordinals follow creation order in the replayed world.
		for i := 0; i < 3; i++ {
			body := player.GetBodyId(i)
			ensure(body.IsValid())

			var shape [1]ShapeID
			ensure(body.GetShapes(shape[:]) == 1)
			got := shape[0].GetName()
			// PORT: ENSURE( got != NULL ): a Go string is never NULL; the length check below covers it.

			srcLen := len(names[i])
			ensure(len(got) == srcLen)
			if srcLen > 0 {
				ensure(got[:srcLen] == names[i][:srcLen])
			}
		}

		player.Destroy()
		rec.Destroy()
	})

	// Sphere round-trip: record/step/stop, then replay and validate.
	It("SphereRoundTrip", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.StartRecording(rec)

		// Set a non-default gravity so the setter op appears in the stream.
		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		// Static ground
		groundDef := DefaultBodyDef()
		groundDef.Type = StaticBody
		groundId := worldId.CreateBody(&groundDef)

		groundBox := MakeBoxHull(50.0, 1.0, 50.0)
		groundShapeDef := DefaultShapeDef()
		groundId.CreateHullShape(&groundShapeDef, &groundBox.Base)

		// Dynamic body with a sphere shape
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, 5.0, 0.0}
		bodyId := worldId.CreateBody(&bodyDef)

		var sphere Sphere
		sphere.Center = Vec3{0.0, 0.0, 0.0}
		sphere.Radius = 0.5
		sphereDef := DefaultShapeDef()
		sphereDef.Density = 1.0
		bodyId.CreateSphereShape(&sphereDef, &sphere)

		timeStep := float32(1.0 / 60.0)
		subStepCount := 4
		for i := 0; i < 30; i++ {
			worldId.Step(timeStep, subStepCount)
		}

		worldId.StopRecording()
		worldId.Destroy()

		ensure(ValidateReplay(rec.GetData()[:rec.GetSize()], 1))

		rec.Destroy()
	})

	// Empty world: recording starts with no bodies and none are ever created. The empty world is still
	// seed-serialized like any other, so replay validates and Restart restores in place with a stable
	// world id rather than tearing down and rebuilding the world.
	It("EmptyWorldRoundTrip", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.StartRecording(rec)
		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		for i := 0; i < 10; i++ {
			worldId.Step(1.0/60.0, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()

		data := rec.GetData()
		size := rec.GetSize()

		// The seed snapshot is written even with no bodies.
		var hdr recHeader
		copy(unsafe.Slice((*byte)(unsafe.Pointer(&hdr)), unsafe.Sizeof(hdr)), data)
		ensure(hdr.snapshotSize > 0)

		ensure(ValidateReplay(data[:size], 1))

		// Restart restores in place, so the replay world id survives a rewind.
		player := CreatePlayer(data[:size], 1)
		ensure(player != nil)

		worldKey := StoreWorldID(player.GetWorldId())
		for !player.IsAtEnd() {
			player.StepFrame()
		}
		player.Restart()
		ensure(StoreWorldID(player.GetWorldId()) == worldKey)
		ensure(player.GetFrame() == 0)
		ensure(!player.HasDiverged())

		player.Destroy()
		rec.Destroy()
	})

	// Hull dedup: three bodies sharing the same hull should produce one registry entry.
	It("HullDedup", func() {
		// Build a small convex hull
		pts := [8]Vec3{
			{-1.0, -1.0, -1.0}, {1.0, -1.0, -1.0}, {1.0, 1.0, -1.0}, {-1.0, 1.0, -1.0},
			{-1.0, -1.0, 1.0}, {1.0, -1.0, 1.0}, {1.0, 1.0, 1.0}, {-1.0, 1.0, 1.0},
		}
		hull := CreateHull(pts[:], 8, 8)
		ensure(hull != nil)

		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.StartRecording(rec)

		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0

		for i := 0; i < 3; i++ {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{float32(i * 3), 5.0, 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&shapeDef, hull)
		}

		timeStep := float32(1.0 / 60.0)
		for i := 0; i < 5; i++ {
			worldId.Step(timeStep, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()
		DestroyHull(hull)

		// Validate the replay
		ensure(ValidateReplay(rec.GetData()[:rec.GetSize()], 1))

		// Confirm the registry was deduped to 1 hull entry.
		// Parse registryOffset from the header and count entries.
		bytes := rec.GetData()
		sz := rec.GetSize()
		ensure(sz >= 48)

		regOff := binary.LittleEndian.Uint64(bytes[32:]) // registryOffset at offset 32 in b3RecHeader
		ensure(regOff != 0 && int(regOff)+4 <= sz)

		// entryCount is a little-endian u32 at the start of the registry block
		rp := bytes[int(regOff):]
		entryCount := uint32(rp[0]) | (uint32(rp[1]) << 8) | (uint32(rp[2]) << 16) | (uint32(rp[3]) << 24)
		ensure(entryCount == 1)

		rec.Destroy()
	})

	// Mid-stream snapshot with only dynamic bodies floating in air (no contacts).
	It("MidStreamNoContacts", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		var sphere Sphere
		sphere.Center = Vec3{0.0, 0.0, 0.0}
		sphere.Radius = 0.5
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0

		// A few dynamic bodies well apart from each other so no contacts form
		for i := 0; i < 4; i++ {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{float32(i * 10), 50.0, 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateSphereShape(&shapeDef, &sphere)
		}

		timeStep := float32(1.0 / 60.0)
		subStepCount := 4
		for i := 0; i < 10; i++ {
			worldId.Step(timeStep, subStepCount)
		}

		// Start recording mid-stream, with a snapshot of the current world
		rec := CreateRecording(0)
		ensure(rec != nil)
		worldId.StartRecording(rec)

		for i := 0; i < 30; i++ {
			worldId.Step(timeStep, subStepCount)
		}

		worldId.StopRecording()
		worldId.Destroy()

		ensure(ValidateReplay(rec.GetData()[:rec.GetSize()], 1))

		rec.Destroy()
	})

	// Mid-stream snapshot with contacts: bodies touching ground with warm-start manifolds.
	It("MidStreamContacts", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		// Static ground using a box hull
		{
			groundDef := DefaultBodyDef()
			groundDef.Type = StaticBody
			groundId := worldId.CreateBody(&groundDef)

			groundBox := MakeBoxHull(50.0, 1.0, 50.0)
			groundShape := DefaultShapeDef()
			groundId.CreateHullShape(&groundShape, &groundBox.Base)
		}

		dynamicShape := DefaultShapeDef()
		dynamicShape.Density = 1.0

		// A few dynamic boxes dropped onto the ground
		for i := 0; i < 3; i++ {
			box := MakeBoxHull(0.5, 0.5, 0.5)

			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{float32(i*2) - 2.0, 5.0, 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&dynamicShape, &box.Base)
		}

		timeStep := float32(1.0 / 60.0)
		subStepCount := 4

		// Let the scene settle: bodies hit ground, build manifolds, islands, graph colors
		for i := 0; i < 60; i++ {
			worldId.Step(timeStep, subStepCount)
		}

		// Start recording with snapshot of the settled world (contacts, islands, warm starts)
		rec := CreateRecording(0)
		ensure(rec != nil)
		worldId.StartRecording(rec)

		for i := 0; i < 30; i++ {
			worldId.Step(timeStep, subStepCount)
		}

		worldId.StopRecording()
		worldId.Destroy()

		ensure(ValidateReplay(rec.GetData()[:rec.GetSize()], 1))

		rec.Destroy()
	})

	// A recording made at the native SIMD width must replay at width 4 and vice versa.
	// PORT: GetSIMDWidth is laneCount (4) and width 8 is not ported, so this always skips.
	It("CrossWidthReplay", func() {
		SetSIMDWidth(0)
		nativeWidth := GetSIMDWidth()
		if nativeWidth == 4 {
			fmt.Printf("  subtest skipped: CrossWidthReplay, native SIMD width is 4\n")
			return
		}

		const hullCount = 6

		var hulls [hullCount]*HullData
		hulls[0] = CreateRock(0.5)
		hulls[1] = CreateComplexHull(0.5)
		hulls[2] = CreateCylinder(0.8, 0.4, 0.0, 8)
		hulls[3] = CreateCone(0.8, 0.5, 0.2, 7)

		seed := uint32(12345)
		for hullIndex := 4; hullIndex < hullCount; hullIndex++ {
			var points [20]Vec3
			for i := 0; i < 20; i++ {
				var v [3]float32
				for k := 0; k < 3; k++ {
					seed = seed*1664525 + 1013904223
					scale := float32(1.0)
					if k == 1 {
						scale = 0.6
					}
					v[k] = (float32(seed>>8)/float32(1<<24) - 0.5) * scale
				}
				points[i] = Vec3{v[0], v[1], v[2]}
			}
			hulls[hullIndex] = CreateHull(points[:], 20, 20)
		}

		rec := CreateRecording(0)

		SetSIMDWidth(nativeWidth)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.StartRecording(rec)

		groundDef := DefaultBodyDef()
		groundDef.Type = StaticBody
		groundId := worldId.CreateBody(&groundDef)
		groundBox := MakeBoxHull(20.0, 1.0, 20.0)
		groundShape := DefaultShapeDef()
		groundId.CreateHullShape(&groundShape, &groundBox.Base)

		dynamicShape := DefaultShapeDef()
		dynamicShape.Density = 1.0

		for i := 0; i < 18; i++ {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{0.4*float32(i%3) - 0.4, 1.5 + 0.9*float32(i), 0.3 * float32(i%2)}
			bodyDef.Rotation = MakeQuatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, 0.37*float32(i))
			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&dynamicShape, hulls[i%hullCount])
		}

		timeStep := float32(1.0 / 60.0)
		for i := 0; i < 120; i++ {
			worldId.Step(timeStep, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()
		SetSIMDWidth(0)

		narrowStatus := recTestValidateReplayAtWidth(rec, 4)
		nativeStatus := recTestValidateReplayAtWidth(rec, nativeWidth)

		rec.Destroy()
		for i := 0; i < hullCount; i++ {
			DestroyHull(hulls[i])
		}

		ensure(narrowStatus == 0)
		ensure(nativeStatus == 0)
	})

	// Staged stepping must reveal a mid-stream body at its creation transform. A body created and given an
	// impulse in one recorded step is first placed by CreateBody, then displaced by the following Step. Atomic
	// replay fuses the two, so the body is only ever seen already moved. Staged replay parks between them so
	// the pre-integration pose is drawable. Verify the parked pose is the creation transform, that atomic
	// replay does not show it, and that the extra park does not perturb the end state.
	It("StagedStepCreationPose", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		worldId.StartRecording(rec)

		timeStep := float32(1.0 / 60.0)
		subStepCount := 4

		// A few empty steps so the creation lands inside the stream, not at frame 0.
		const leadFrames = 3
		for i := 0; i < leadFrames; i++ {
			worldId.Step(timeStep, subStepCount)
		}

		// The creation transform under test. No ground, so the body is the only create and stays ballistic.
		// Components are exact in float so the round-trip pose compares tight.
		spawn := Pos{1.0, 20.0, -2.0}
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = spawn
		bodyId := worldId.CreateBody(&bodyDef)

		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		bodyId.CreateSphereShape(&shapeDef, &sphere)

		// Impulse along +x so the first integrated step moves the body a visible distance off the spawn.
		bodyId.ApplyLinearImpulseToCenter(Vec3{5.0, 0.0, 0.0}, true)

		const creationFrame = leadFrames + 1
		worldId.Step(timeStep, subStepCount) // advances to creationFrame

		const totalFrames = creationFrame + 3
		for i := creationFrame; i < totalFrames; i++ {
			worldId.Step(timeStep, subStepCount)
		}

		worldId.StopRecording()
		worldId.Destroy()

		data := rec.GetData()
		sz := rec.GetSize()

		// Atomic replay fuses the create and its step, so at the creation frame the body is already
		// integrated and displaced along +x. Capture that pose and the final state hash.
		atomic := CreatePlayer(data[:sz], 1)
		ensure(atomic != nil)
		atomicPose := Pos{0.0, 0.0, 0.0}
		for !atomic.IsAtEnd() {
			atomic.StepFrame()
			if atomic.GetFrame() == creationFrame {
				id := atomic.GetBodyId(0)
				ensure(id.IsValid())
				atomicPose = id.GetPosition()
			}
		}
		ensure(atomic.GetFrame() == totalFrames)
		ensure(!atomic.HasDiverged())
		ensure(atomicPose.X-spawn.X > 0.01) // the impulse moved it before it was ever seen
		atomicHash := hashWorldState(getWorldFromId(atomic.GetWorldId()))
		atomic.Destroy()

		// Staged replay: step forward until the first pre-step park. It must sit at the creation frame's
		// pre-integration state with the new body at exactly its spawn transform.
		staged := CreatePlayer(data[:sz], 1)
		ensure(staged != nil)
		for !staged.IsAtEnd() {
			staged.SubStepFrame()
			if staged.IsAtPreStep() {
				break
			}
		}
		ensure(staged.IsAtPreStep())
		ensure(staged.GetFrame() == creationFrame-1) // parked before the step, frame not advanced

		stagedId := staged.GetBodyId(0)
		ensure(stagedId.IsValid())
		parkPose := stagedId.GetPosition()
		ensureSmall(parkPose.X-spawn.X, 1.0e-4)
		ensureSmall(parkPose.Y-spawn.Y, 1.0e-4)
		ensureSmall(parkPose.Z-spawn.Z, 1.0e-4)

		// Finishing the frame integrates the body, so it leaves the spawn pose on the next advance.
		staged.SubStepFrame()
		ensure(staged.GetFrame() == creationFrame)
		ensure(!staged.IsAtPreStep())

		// Run staged to the end: the same op stream, so it must land on the atomic end state bit for bit.
		for !staged.IsAtEnd() {
			staged.SubStepFrame()
		}
		ensure(staged.GetFrame() == totalFrames)
		ensure(!staged.HasDiverged())
		stagedHash := hashWorldState(getWorldFromId(staged.GetWorldId()))
		ensure(stagedHash == atomicHash)
		staged.Destroy()

		rec.Destroy()
	})

	// Record a scene with hull boxes settling on a ground plane, create a player, step to
	// the end recording per-frame world hashes, then seek backward to several frames and
	// verify each reproduces the recorded hash exactly.
	It("ScrubBackward", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		worldId.StartRecording(rec)

		// Static ground
		{
			groundDef := DefaultBodyDef()
			groundDef.Type = StaticBody
			groundId := worldId.CreateBody(&groundDef)
			groundBox := MakeBoxHull(20.0, 1.0, 20.0)
			groundShape := DefaultShapeDef()
			groundId.CreateHullShape(&groundShape, &groundBox.Base)
		}

		// A small stack of dynamic hull boxes
		boxShape := DefaultShapeDef()
		boxShape.Density = 1.0
		for i := 0; i < 4; i++ {
			box := MakeBoxHull(0.5, 0.5, 0.5)
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{0.0, 2.0 + float32(i)*1.5, 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&boxShape, &box.Base)
		}

		timeStep := float32(1.0 / 60.0)
		subStepCount := 4
		totalFrames := 80
		for i := 0; i < totalFrames; i++ {
			worldId.Step(timeStep, subStepCount)
		}

		worldId.StopRecording()
		worldId.Destroy()

		data := rec.GetData()
		sz := rec.GetSize()

		// Create the player
		player := CreatePlayer(data[:sz], 1)
		ensure(player != nil)
		ensure(player.GetFrameCount() == totalFrames)

		// Forward pass: record per-frame hashes
		hashes := make([]uint64, totalFrames+1)
		hashes[0] = 0 // frame 0 before any step

		for !player.IsAtEnd() {
			player.StepFrame()
			f := player.GetFrame()
			if f <= totalFrames {
				wid := player.GetWorldId()
				w := getWorldFromId(wid)
				hashes[f] = hashWorldState(w)
			}
		}
		ensure(player.GetFrame() == totalFrames)
		ensure(!player.HasDiverged())

		// Backward seek to several interesting frames and verify hash matches
		seekTargets := []int{totalFrames, totalFrames / 2, 5, totalFrames - 1, 0, 1}
		seekCount := len(seekTargets)
		for k := 0; k < seekCount; k++ {
			target := seekTargets[k]
			player.SeekFrame(target)
			ensure(player.GetFrame() == target)
			ensure(!player.HasDiverged())

			if target > 0 {
				wid := player.GetWorldId()
				w := getWorldFromId(wid)
				got := hashWorldState(w)
				ensure(got == hashes[target])
			}
		}

		player.Destroy()
		rec.Destroy()
	})

	// Record a scene that includes a mesh shape, create a player, seek backward, verify
	// it works and no divergence is reported. Also checks the keyframe-by-geometry-id
	// invariant: keyframeRec registry count should not grow beyond initial slot count.
	It("SeekWithHull", func() {
		pts := [8]Vec3{
			{-1.0, -1.0, -1.0}, {1.0, -1.0, -1.0}, {1.0, 1.0, -1.0}, {-1.0, 1.0, -1.0},
			{-1.0, -1.0, 1.0}, {1.0, -1.0, 1.0}, {1.0, 1.0, 1.0}, {-1.0, 1.0, 1.0},
		}
		hull := CreateHull(pts[:], 8, 8)
		ensure(hull != nil)

		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})
		worldId.StartRecording(rec)

		// Static ground
		{
			groundDef := DefaultBodyDef()
			groundDef.Type = StaticBody
			groundId := worldId.CreateBody(&groundDef)
			groundBox := MakeBoxHull(20.0, 1.0, 20.0)
			gs := DefaultShapeDef()
			groundId.CreateHullShape(&gs, &groundBox.Base)
		}

		// Dynamic bodies using the custom hull
		sd := DefaultShapeDef()
		sd.Density = 1.0
		for i := 0; i < 3; i++ {
			bd := DefaultBodyDef()
			bd.Type = DynamicBody
			bd.Position = Pos{float32(i*4) - 4.0, 5.0, 0.0}
			bodyId := worldId.CreateBody(&bd)
			bodyId.CreateHullShape(&sd, hull)
		}

		timeStep := float32(1.0 / 60.0)
		totalFrames := 40
		for i := 0; i < totalFrames; i++ {
			worldId.Step(timeStep, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()
		DestroyHull(hull)

		data := rec.GetData()
		sz := rec.GetSize()

		player := CreatePlayer(data[:sz], 1)
		ensure(player != nil)

		// Step to end
		for !player.IsAtEnd() {
			player.StepFrame()
		}
		ensure(!player.HasDiverged())

		midFrame := totalFrames / 2
		player.SeekFrame(midFrame)
		ensure(player.GetFrame() == midFrame)
		ensure(!player.HasDiverged())

		player.SeekFrame(0)
		ensure(player.GetFrame() == 0)

		player.Destroy()
		rec.Destroy()
	})

	// The 3D sample renderer builds per-shape GPU meshes through createDebugShape, so the replay
	// world must carry the host callbacks. Verify b3RecPlayer_SetDebugShapeCallbacks rewinds, fires
	// the callbacks for every replayed shape, keeps them across a backward-seek world rebuild, and
	// balances create/destroy at teardown.
	It("DebugShapeCallbacks", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.StartRecording(rec)
		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		groundDef := DefaultBodyDef()
		groundDef.Type = StaticBody
		groundId := worldId.CreateBody(&groundDef)
		groundBox := MakeBoxHull(20.0, 1.0, 20.0)
		groundShape := DefaultShapeDef()
		groundId.CreateHullShape(&groundShape, &groundBox.Base)

		// Four dynamic boxes sharing one hull: ground + 4 boxes = 5 shapes total.
		box := MakeBoxHull(0.5, 0.5, 0.5)
		boxShape := DefaultShapeDef()
		boxShape.Density = 1.0
		for i := 0; i < 4; i++ {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{0.0, 1.0 + 1.1*float32(i), 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&boxShape, &box.Base)
		}

		totalFrames := 30
		for i := 0; i < totalFrames; i++ {
			worldId.Step(1.0/60.0, 4)
		}
		worldId.StopRecording()
		worldId.Destroy()

		// Round-trip through a file, mirroring the replay sample's Generate/Load path exactly.
		path := "replay_test.b3rec"
		ensure(SaveRecordingToFile(rec, path))
		loaded := LoadRecordingFromFile(path)
		ensure(loaded != nil)

		player := CreatePlayer(loaded.GetData()[:loaded.GetSize()], 1)
		ensure(player != nil)

		// Wiring the callbacks rebuilds the world and rewinds to frame 0.
		counters := &recTestDebugShapeCounters{0, 0}
		player.SetDebugShapeCallbacks(recTestCreateDebugShape, recTestDestroyDebugShape, counters)
		ensure(player.GetFrame() == 0)

		// Replay to the end, then draw: createDebugShape fires once per shape (ground + 4 boxes = 5).
		for !player.IsAtEnd() {
			player.StepFrame()
		}
		recTestDrawWorld(player.GetWorldId())
		ensure(counters.created >= 5)
		ensure(!player.HasDiverged())

		// A backward seek restores the empty seed in place, releasing the live debug shapes, then forward
		// stepping recreates them through the callbacks, so more creates fire.
		createdBefore := counters.created
		player.SeekFrame(0)
		player.SeekFrame(totalFrames)
		recTestDrawWorld(player.GetWorldId())
		ensure(counters.created > createdBefore)

		// Teardown destroys the final world; every live shape is released, so the counts balance.
		player.Destroy()
		ensure(counters.created == counters.destroyed)

		loaded.Destroy()
		rec.Destroy()
		os.Remove(path)
	})

	// Exercise the viewer-facing player accessors: recording info, creation-ordinal body tracking
	// (seeded from a snapshot), divergence frame, and keyframe policy. Recording starts after the
	// bodies exist, so the snapshot seeds the outliner list and ordinals are stable from frame 0.
	It("PlayerAccessors", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		// Static ground (creation ordinal 0)
		groundDef := DefaultBodyDef()
		groundDef.Type = StaticBody
		groundId := worldId.CreateBody(&groundDef)
		groundBox := MakeBoxHull(20.0, 1.0, 20.0)
		groundShape := DefaultShapeDef()
		groundId.CreateHullShape(&groundShape, &groundBox.Base)

		// Four dynamic boxes (ordinals 1..4)
		const dynamicCount = 4
		boxShape := DefaultShapeDef()
		boxShape.Density = 1.0
		for i := 0; i < dynamicCount; i++ {
			box := MakeBoxHull(0.5, 0.5, 0.5)
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{0.0, 2.0 + float32(i)*1.5, 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&boxShape, &box.Base)
		}

		timeStep := float32(1.0 / 60.0)
		subStepCount := 4

		// Settle, then record with a snapshot of the populated world.
		for i := 0; i < 10; i++ {
			worldId.Step(timeStep, subStepCount)
		}

		rec := CreateRecording(0)
		ensure(rec != nil)
		worldId.StartRecording(rec)

		totalFrames := 80
		for i := 0; i < totalFrames; i++ {
			worldId.Step(timeStep, subStepCount)
		}
		worldId.StopRecording()
		worldId.Destroy()

		player := CreatePlayer(rec.GetData()[:rec.GetSize()], 1)
		ensure(player != nil)

		// Info reflects the recorded tuning and a non-degenerate bounds.
		info := player.GetInfo()
		ensure(int(info.FrameCount) == totalFrames)
		ensure(int(info.SubStepCount) == subStepCount)
		ensure(info.TimeStep > 0.0)
		extent := Sub(info.Bounds.UpperBound, info.Bounds.LowerBound)
		ensure(extent.X > 0.0 && extent.Y > 0.0 && extent.Z > 0.0)

		// Body ordinals: ground + 4 dynamic, seeded from the snapshot and present at frame 0.
		ensure(player.GetBodyCount() == 1+dynamicCount)
		ground := player.GetBodyId(0)
		ensure(ground.IsValid())
		ensure(ground.GetType() == StaticBody)
		for i := 1; i <= dynamicCount; i++ {
			id := player.GetBodyId(i)
			ensure(id.IsValid())
			ensure(id.GetType() == DynamicBody)
		}
		ensure(player.GetBodyId(1 + dynamicCount).IsNull())

		// No divergence on a clean serial replay.
		player.SeekFrame(totalFrames)
		ensure(!player.HasDiverged())
		ensure(player.GetDivergeFrame() == -1)

		// Ordinals survive a backward seek that restores from a keyframe.
		before := player.GetBodyId(2)
		player.SeekFrame(totalFrames / 2)
		player.SeekFrame(totalFrames)
		after := player.GetBodyId(2)
		ensure(before == after)

		// Keyframe policy: defaults present, setter takes effect and clears the ring.
		ensure(player.GetKeyframeMinInterval() == 16)
		player.SetKeyframePolicy(uint64(256)*1024*1024, 8)
		ensure(player.GetKeyframeMinInterval() == 8)
		ensure(player.GetKeyframeInterval() == 8)
		ensure(player.GetKeyframeBudget() == uint64(256)*1024*1024)
		ensure(player.GetKeyframeBytes() == 0)

		player.Destroy()
		rec.Destroy()
	})

	// A keyframe restore is a deterministic replay state, so shapes that persist must keep their renderer
	// handle rather than being torn down and rebuilt every seek. Record a snapshot-seeded session (shapes
	// exist at frame 0), replay with handle callbacks, scrub backward across keyframes repeatedly, and
	// verify no new handles are built per restore and the create/destroy counts still balance at teardown.
	It("KeyframeHandleReuse", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		groundDef := DefaultBodyDef()
		groundDef.Type = StaticBody
		groundId := worldId.CreateBody(&groundDef)
		groundBox := MakeBoxHull(20.0, 1.0, 20.0)
		groundShape := DefaultShapeDef()
		groundId.CreateHullShape(&groundShape, &groundBox.Base)

		const dynamicCount = 5
		box := MakeBoxHull(0.5, 0.5, 0.5)
		boxShape := DefaultShapeDef()
		boxShape.Density = 1.0
		for i := 0; i < dynamicCount; i++ {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{0.0, 1.0 + 1.1*float32(i), 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&boxShape, &box.Base)
		}
		shapeCount := 1 + dynamicCount

		// Settle, then record with a snapshot of the populated world so shapes exist at frame 0.
		for i := 0; i < 10; i++ {
			worldId.Step(1.0/60.0, 4)
		}

		rec := CreateRecording(0)
		ensure(rec != nil)
		worldId.StartRecording(rec)

		totalFrames := 80
		for i := 0; i < totalFrames; i++ {
			worldId.Step(1.0/60.0, 4)
		}
		worldId.StopRecording()
		worldId.Destroy()

		player := CreatePlayer(rec.GetData()[:rec.GetSize()], 1)
		ensure(player != nil)

		counters := &recTestDebugShapeCounters{0, 0}
		player.SetDebugShapeCallbacks(recTestCreateDebugShape, recTestDestroyDebugShape, counters)

		// Replay to the end and draw: one handle per shape.
		player.SeekFrame(totalFrames)
		recTestDrawWorld(player.GetWorldId())
		ensure(!player.HasDiverged())
		ensure(counters.created == shapeCount)

		// Scrub backward and forward across keyframes. Each restore keeps the persistent shapes' handles,
		// so drawing builds no new ones.
		createdAfterFirstDraw := counters.created
		seekTargets := []int{40, 70, 20, 60, 8, 75}
		for k := 0; k < len(seekTargets); k++ {
			player.SeekFrame(seekTargets[k])
			recTestDrawWorld(player.GetWorldId())
		}
		ensure(counters.created == createdAfterFirstDraw)

		// Teardown releases exactly the live handles, so the leak-free invariant holds.
		player.Destroy()
		ensure(counters.created == counters.destroyed)

		rec.Destroy()
	})

	// Issue all seven world queries each frame, then replay. Every query is re-issued against the replay
	// world and compared to what was recorded, so a clean (non-diverged) replay proves the queries
	// reproduce. Also opens a player and confirms the per-frame query store surfaces all seven.
	It("QueryReplay", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		groundDef := DefaultBodyDef()
		groundDef.Type = StaticBody
		groundId := worldId.CreateBody(&groundDef)
		groundBox := MakeBoxHull(20.0, 1.0, 20.0)
		groundShape := DefaultShapeDef()
		groundId.CreateHullShape(&groundShape, &groundBox.Base)

		// A few dynamic spheres for the queries to find.
		for i := 0; i < 4; i++ {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{float32(i) - 1.5, 3.0, 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
			sphereDef := DefaultShapeDef()
			sphereDef.Density = 1.0
			bodyId.CreateSphereShape(&sphereDef, &sphere)
		}

		worldId.StartRecording(rec)

		filter := DefaultQueryFilter()

		const totalFrames = 30
		for i := 0; i < totalFrames; i++ {
			origin := Pos{0.0, 6.0, 0.0}
			translation := Vec3{0.0, -8.0, 0.0}
			aabb := AABB{Vec3{-5.0, -1.0, -5.0}, Vec3{5.0, 6.0, 5.0}}

			proxyPts := Vec3{0.0, 0.0, 0.0}
			proxy := ShapeProxy{[]Vec3{proxyPts}, 1, 0.5}
			mover := Capsule{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, 0.3}

			worldId.OverlapAABB(aabb, filter, recTestQueryReplayOverlapFcn, nil)
			worldId.OverlapShape(origin, &proxy, filter, recTestQueryReplayOverlapFcn, nil)
			worldId.CastRay(origin, translation, filter, recTestQueryReplayCastFcn, nil)
			worldId.CastRayClosest(origin, translation, filter)
			worldId.CastShape(origin, &proxy, translation, filter, recTestQueryReplayCastFcn, nil)
			worldId.CastMover(origin, &mover, translation, filter, recTestQueryReplayMoverFilterFcn, nil)
			worldId.CollideMover(origin, &mover, filter, recTestQueryReplayPlaneFcn, nil)

			worldId.Step(1.0/60.0, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()

		// Headless validation: re-issues every recorded query and compares the results.
		ensure(ValidateReplay(rec.GetData()[:rec.GetSize()], 1))

		// Player path: seek to a mid frame and confirm the per-frame store holds all seven queries.
		player := CreatePlayer(rec.GetData()[:rec.GetSize()], 1)
		ensure(player != nil)

		player.SeekFrame(15)
		ensure(!player.HasDiverged())
		ensure(player.GetFrameQueryCount() == 7)

		first := player.GetFrameQuery(0)
		ensure(first.Type == RecQueryOverlapAABB)

		// The ray cast should find at least the ground, so its recorded hit list is non-empty.
		sawCastRay := false
		for qi := 0; qi < player.GetFrameQueryCount(); qi++ {
			info := player.GetFrameQuery(qi)
			if info.Type == RecQueryCastRay {
				sawCastRay = true
				ensure(info.HitCount > 0)
			}
		}
		ensure(sawCastRay)

		player.Destroy()
		rec.Destroy()
	})

	// Tagged queries: the caller (id, label) is hashed into a key that rides the QueryTag op, with the id
	// and label interned in the trailing tag table. The same label under two entity ids is two distinct
	// keys. All survive a file round-trip and resolve back through b3RecQueryInfo; untagged queries report
	// key 0 / id 0 / name NULL.
	It("TaggedQuery", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		groundDef := DefaultBodyDef()
		groundDef.Type = StaticBody
		groundId := worldId.CreateBody(&groundDef)
		groundBox := MakeBoxHull(20.0, 1.0, 20.0)
		groundShape := DefaultShapeDef()
		groundId.CreateHullShape(&groundShape, &groundBox.Base)

		worldId.StartRecording(rec)

		// Same label, different entity ids: distinct logical queries with distinct keys.
		bullet53 := DefaultQueryFilter()
		bullet53.Id = 53
		bullet53.Name = "bullet"

		bullet54 := DefaultQueryFilter()
		bullet54.Id = 54
		bullet54.Name = "bullet"

		untagged := DefaultQueryFilter()

		key53 := hashQueryTag(53, "bullet")
		key54 := hashQueryTag(54, "bullet")
		ensure(key53 != 0 && key54 != 0 && key53 != key54)

		const totalFrames = 10
		for i := 0; i < totalFrames; i++ {
			origin := Pos{0.0, 6.0, 0.0}
			translation := Vec3{0.0, -8.0, 0.0}
			aabb := AABB{Vec3{-5.0, -1.0, -5.0}, Vec3{5.0, 6.0, 5.0}}

			// Two tagged rays sharing the label "bullet" plus one untagged overlap, every frame.
			worldId.CastRay(origin, translation, bullet53, recTestQueryReplayCastFcn, nil)
			worldId.CastRay(origin, translation, bullet54, recTestQueryReplayCastFcn, nil)
			worldId.OverlapAABB(aabb, untagged, recTestQueryReplayOverlapFcn, nil)

			worldId.Step(1.0/60.0, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()

		ensure(ValidateReplay(rec.GetData()[:rec.GetSize()], 1))

		// Round-trip through a file so the interned tag table is exercised on the persisted bytes.
		path := "tagged_query_test.b3rec"
		ensure(SaveRecordingToFile(rec, path))
		loaded := LoadRecordingFromFile(path)
		ensure(loaded != nil)

		player := CreatePlayer(loaded.GetData()[:loaded.GetSize()], 1)
		ensure(player != nil)

		player.SeekFrame(5)
		ensure(!player.HasDiverged())
		ensure(player.GetFrameQueryCount() == 3)

		saw53, saw54, sawUntagged := false, false, false
		for qi := 0; qi < player.GetFrameQueryCount(); qi++ {
			info := player.GetFrameQuery(qi)
			if info.Key == key53 {
				saw53 = true
				ensure(info.Id == 53 && info.Name != "" && info.Name == "bullet")
			} else if info.Key == key54 {
				saw54 = true
				ensure(info.Id == 54 && info.Name != "" && info.Name == "bullet")
			} else {
				sawUntagged = true
				ensure(info.Key == 0 && info.Id == 0 && info.Name == "")
			}
		}
		ensure(saw53 && saw54 && sawUntagged)

		player.Destroy()
		loaded.Destroy()
		rec.Destroy()
		// PORT: upstream leaves tagged_query_test.b3rec behind; the Go test removes it so the package
		// directory stays clean.
		os.Remove(path)
	})

	// A transformed hull bakes its transform and non-uniform scale into fresh hull data at create time.
	// It must be recorded like any other shape create, else its shape id allocation is invisible to the
	// player and every later id drifts. A plain hull created after it would then mismatch on replay.
	It("TransformedHullRoundTrip", func() {
		pts := [8]Vec3{
			{-1.0, -1.0, -1.0}, {1.0, -1.0, -1.0}, {1.0, 1.0, -1.0}, {-1.0, 1.0, -1.0},
			{-1.0, -1.0, 1.0}, {1.0, -1.0, 1.0}, {1.0, 1.0, 1.0}, {-1.0, 1.0, 1.0},
		}
		hull := CreateHull(pts[:], 8, 8)
		ensure(hull != nil)

		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		worldId.StartRecording(rec)

		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0

		// Baked transform with a rotation and non-uniform scale, the path Unreal uses for instanced hulls.
		xf := Transform{Vec3{0.25, 0.0, -0.5}, MakeQuatFromAxisAngle(Vec3{0.0, 0.0, 1.0}, 0.3)}
		scl := Vec3{1.5, 0.5, 2.0}
		for i := 0; i < 3; i++ {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{float32(i * 3), 5.0, 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			sid := bodyId.CreateTransformedHullShape(&shapeDef, hull, xf, scl)
			ensure(sid.IsValid())
		}

		// A plain hull after the transformed ones: if the transformed creates desynced the id pool, this
		// shape's recorded id would not match what replay allocates and b3ValidateReplay would fail.
		{
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position = Pos{0.0, 10.0, 0.0}
			bodyId := worldId.CreateBody(&bodyDef)
			sid := bodyId.CreateHullShape(&shapeDef, hull)
			ensure(sid.IsValid())
		}

		// Step past the keyframe interval so replay captures a keyframe. That path re-serializes the live
		// world and interns the baked hull, which must already be in the pre-seeded registry.
		for i := 0; i < 20; i++ {
			worldId.Step(1.0/60.0, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()
		DestroyHull(hull)

		ensure(ValidateReplay(rec.GetData()[:rec.GetSize()], 1))
		ensure(ValidateReplay(rec.GetData()[:rec.GetSize()], 4))

		rec.Destroy()
	})

	// Swapping a shape's hull or mesh, or retuning one of its per-triangle materials, is a world
	// mutation like any other and has to ride the stream. The geometry pair interns into the registry
	// at the record site so replay rebuilds the same shape instead of running on the geometry it was
	// created with. The control run proves the mutations move the simulation, so the state hash gate
	// has teeth, and the read back covers each op on its own where dynamics alone would not.
	It("GeometryMutatorReplay", func() {
		// Two flat floors with different triangulations, both carrying two material slots so the
		// material index stays live across the swap.
		meshA := CreateGridMesh(8, 8, 2.0, 2, false)
		ensure(meshA != nil)
		meshB := CreateGridMesh(12, 12, 1.5, 2, false)
		ensure(meshB != nil)
		ensure(meshA.TriangleCount != meshB.TriangleCount)

		swapHull := MakeBoxHull(0.25, 1.5, 0.25)
		const swapFriction float32 = 0.95

		controlHash := recTestRunGeometryMutatorScene(nil, false, meshA, meshB, &swapHull.Base, swapFriction)

		rec := CreateRecording(0)
		ensure(rec != nil)
		mutatedHash := recTestRunGeometryMutatorScene(rec, true, meshA, meshB, &swapHull.Base, swapFriction)

		// Without this the replay gate below could pass on a recording that never carried the ops.
		ensure(mutatedHash != controlHash)

		data := rec.GetData()
		size := rec.GetSize()
		ensure(size > 0)

		ensure(ValidateReplay(data[:size], 1))
		ensure(ValidateReplay(data[:size], 4))

		player := CreatePlayer(data[:size], 1)
		ensure(player != nil)
		for player.StepFrame() {
		}
		ensure(player.HasDiverged() == false)

		// Body ordinals follow creation order in the replayed world.
		replayMeshBody := player.GetBodyId(0)
		replayBoxBody := player.GetBodyId(1)
		ensure(replayMeshBody.IsValid() && replayBoxBody.IsValid())

		var replayMeshShape [1]ShapeID
		ensure(replayMeshBody.GetShapes(replayMeshShape[:]) == 1)
		var replayBoxShape [1]ShapeID
		ensure(replayBoxBody.GetShapes(replayBoxShape[:]) == 1)

		replayHull := replayBoxShape[0].GetHull()
		ensure(replayHull != nil)
		ensure(replayHull.Hash == swapHull.Base.Hash)

		replayMesh := replayMeshShape[0].GetMesh()
		ensure(replayMesh.Data != nil)
		ensure(replayMesh.Data.Hash == meshB.Hash)
		ensure(replayMesh.Data.TriangleCount == meshB.TriangleCount)

		replayMaterial := replayMeshShape[0].GetMeshSurfaceMaterial(1)
		ensure(replayMaterial.Friction == swapFriction)

		player.Destroy()
		rec.Destroy()
		DestroyMesh(meshA)
		DestroyMesh(meshB)
	})

	// Exercise every recorded op in a single session, then validate replay at two worker
	// counts, round-trip through a file, and drive the incremental player.
	It("AllOps", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldDef.WorkerCount = 1
		worldId := CreateWorld(&worldDef)
		ensure(worldId.IsValid())

		worldId.StartRecording(rec)

		// Static ground with a box-hull shape
		groundDef := DefaultBodyDef()
		groundDef.Type = StaticBody
		groundId := worldId.CreateBody(&groundDef)
		ensure(groundId.IsValid())
		groundBox := MakeBoxHull(50.0, 1.0, 50.0)
		groundShapeDef := DefaultShapeDef()
		groundShapeId := groundId.CreateHullShape(&groundShapeDef, &groundBox.Base)
		ensure(groundShapeId.IsValid())

		// Dynamic body with a sphere shape.
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, 5.0, 0.0}
		bodyDef.Name = "testBodyWithVeryLongNameThatIsAVeryLongNameLength"
		bodyId := worldId.CreateBody(&bodyDef)
		ensure(bodyId.IsValid())

		sphereShapeDef := DefaultShapeDef()
		sphereShapeDef.Density = 1.0
		// Over-length shape name so replay exercises the clamp in the shape def reader, like the body above.
		sphereShapeDef.Name = "sphereNameThatExceedsTheLimit"
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
		sphereShapeId := bodyId.CreateSphereShape(&sphereShapeDef, &sphere)
		ensure(sphereShapeId.IsValid())

		// Capsule shape on a second dynamic body
		capsuleBodyDef := DefaultBodyDef()
		capsuleBodyDef.Type = DynamicBody
		capsuleBodyDef.Position = Pos{3.0, 5.0, 0.0}
		capsuleBodyId := worldId.CreateBody(&capsuleBodyDef)
		ensure(capsuleBodyId.IsValid())

		capsuleShapeDef := DefaultShapeDef()
		capsuleShapeDef.Density = 1.0
		capsule := Capsule{Vec3{0.0, -0.4, 0.0}, Vec3{0.0, 0.4, 0.0}, 0.25}
		capsuleShapeId := capsuleBodyId.CreateCapsuleShape(&capsuleShapeDef, &capsule)
		ensure(capsuleShapeId.IsValid())

		// Custom hull shape on a third dynamic body
		hullPts := [8]Vec3{
			{-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
			{-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5},
		}
		customHull := CreateHull(hullPts[:], 8, 8)
		ensure(customHull != nil)

		hullBodyDef := DefaultBodyDef()
		hullBodyDef.Type = DynamicBody
		hullBodyDef.Position = Pos{-3.0, 5.0, 0.0}
		hullBodyId := worldId.CreateBody(&hullBodyDef)
		ensure(hullBodyId.IsValid())

		hullShapeDef := DefaultShapeDef()
		hullShapeDef.Density = 1.0
		hullShapeId := hullBodyId.CreateHullShape(&hullShapeDef, customHull)
		ensure(hullShapeId.IsValid())

		// Box hull shape on a fourth dynamic body (b3MakeBoxHull path)
		boxBodyDef := DefaultBodyDef()
		boxBodyDef.Type = DynamicBody
		boxBodyDef.Position = Pos{6.0, 5.0, 0.0}
		boxBodyId := worldId.CreateBody(&boxBodyDef)
		ensure(boxBodyId.IsValid())

		boxShapeDef := DefaultShapeDef()
		boxShapeDef.Density = 2.0
		boxHull := MakeBoxHull(0.5, 0.5, 0.5)
		boxShapeId := boxBodyId.CreateHullShape(&boxShapeDef, &boxHull.Base)
		ensure(boxShapeId.IsValid())

		// Transformed hull shape on a fifth dynamic body (b3CreateTransformedHullShape path)
		xformBodyDef := DefaultBodyDef()
		xformBodyDef.Type = DynamicBody
		xformBodyDef.Position = Pos{12.0, 5.0, 0.0}
		xformBodyId := worldId.CreateBody(&xformBodyDef)
		ensure(xformBodyId.IsValid())
		xformShapeDef := DefaultShapeDef()
		xformShapeDef.Density = 1.0
		xformXf := Transform{Vec3{0.1, 0.2, -0.1}, MakeQuatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, 0.4)}
		xformShapeId := xformBodyId.CreateTransformedHullShape(&xformShapeDef, customHull, xformXf, Vec3{1.25, 0.75, 1.5})
		ensure(xformShapeId.IsValid())

		// Mesh, height field, and compound static shapes (3D-only)
		meshBodyDef := DefaultBodyDef()
		meshBodyDef.Type = StaticBody
		meshBodyDef.Position = Pos{20.0, 0.0, 0.0}
		meshBodyId := worldId.CreateBody(&meshBodyDef)
		meshData := CreateGridMesh(3, 3, 2.0, 0, false)
		ensure(meshData != nil)
		swapMeshData := CreateGridMesh(4, 4, 1.5, 0, false)
		ensure(swapMeshData != nil)
		meshShapeDef := DefaultShapeDef()
		meshShapeId := meshBodyId.CreateMeshShape(&meshShapeDef, meshData, Vec3{1.0, 1.0, 1.0})
		ensure(meshShapeId.IsValid())

		hfBodyDef := DefaultBodyDef()
		hfBodyDef.Type = StaticBody
		hfBodyDef.Position = Pos{-20.0, 0.0, 0.0}
		hfBodyId := worldId.CreateBody(&hfBodyDef)
		hf := CreateGrid(4, 4, Vec3{2.0, 1.0, 2.0}, false)
		ensure(hf != nil)
		hfShapeDef := DefaultShapeDef()
		hfBodyId.CreateHeightFieldShape(&hfShapeDef, hf)

		compoundBodyDef := DefaultBodyDef()
		compoundBodyDef.Type = StaticBody
		compoundBodyDef.Position = Pos{30.0, 0.0, 0.0}
		compoundBodyId := worldId.CreateBody(&compoundBodyDef)
		var compSphere CompoundSphereDef
		compSphere.Sphere = Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}
		compSphere.Material = DefaultSurfaceMaterial()

		// Mesh child with five materials so the compound material map is recorded
		compoundMeshData := CreateGridMesh(1, 5, 1.0, 5, false)
		ensure(compoundMeshData != nil)
		ensure(compoundMeshData.MaterialCount == 5)
		var compoundMeshMaterials [5]SurfaceMaterial
		for i := 0; i < 5; i++ {
			compoundMeshMaterials[i] = DefaultSurfaceMaterial()
		}
		var compMesh CompoundMeshDef
		compMesh.MeshData = compoundMeshData
		compMesh.Transform = TransformIdentity
		compMesh.Scale = Vec3{1.0, 1.0, 1.0}
		compMesh.Materials = compoundMeshMaterials[:]
		compMesh.MaterialCount = 5

		var compoundDef CompoundDef
		compoundDef.Spheres = []CompoundSphereDef{compSphere}
		compoundDef.SphereCount = 1
		compoundDef.Meshes = []CompoundMeshDef{compMesh}
		compoundDef.MeshCount = 1
		compound := CreateCompound(&compoundDef)
		ensure(compound != nil)
		compoundShapeDef := DefaultShapeDef()
		compoundBodyId.CreateBakedCompoundShape(&compoundShapeDef, compound)

		// Throwaway shape to exercise DestroyShape
		tmpSphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.1}
		tmpShapeId := capsuleBodyId.CreateSphereShape(&capsuleShapeDef, &tmpSphere)
		tmpShapeId.Destroy(true)

		// Shape mutators: SetFriction, SetRestitution, SetDensity, SetSurfaceMaterial, SetMeshMaterial,
		// SetFilter, EnableSensorEvents, EnableContactEvents, EnableHitEvents, EnablePreSolveEvents,
		// ApplyWind, SetSphere, SetCapsule, SetHull, SetMesh, SetName
		boxShapeId.SetFriction(0.3)
		capsuleShapeId.SetRestitution(0.5)
		boxShapeId.SetDensity(3.0, true)
		surfMat := DefaultSurfaceMaterial()
		surfMat.Friction = 0.7
		surfMat.Restitution = 0.1
		capsuleShapeId.SetSurfaceMaterial(surfMat)
		shapeFilter := DefaultFilter()
		shapeFilter.CategoryBits = 0x2
		boxShapeId.SetFilter(shapeFilter, false)
		capsuleShapeId.EnableSensorEvents(true)
		capsuleShapeId.EnableContactEvents(true)
		boxShapeId.EnableHitEvents(true)
		boxShapeId.EnablePreSolveEvents(true)
		capsuleShapeId.ApplyWind(Vec3{1.0, 0.0, 0.0}, 0.1, 0.0, 10.0, true)
		newSphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.45}
		sphereShapeId.SetSphere(&newSphere)
		newCapsule := Capsule{Vec3{0.0, -0.3, 0.0}, Vec3{0.0, 0.3, 0.0}, 0.3}
		capsuleShapeId.SetCapsule(&newCapsule)
		boxShapeId.SetName("box")

		// Geometry swaps intern into the registry at the record site. The repeated SetHull takes the
		// shared hull short circuit, which changes nothing and so must leave the stream alone.
		swapHull := MakeBoxHull(0.3, 0.7, 0.4)
		boxShapeId.SetHull(&swapHull.Base)
		boxShapeId.SetHull(&swapHull.Base)
		meshShapeId.SetMesh(swapMeshData, Vec3{1.0, 1.0, 1.0})
		meshShapeId.SetMeshMaterial(surfMat, 0)

		// Body mutators: SetTransform, SetLinearVelocity/AngularVelocity (Vec3), SetName,
		// damping, gravity scale, sleep threshold, SetAwake, EnableSleep, SetBullet, SetMotionLocks,
		// SetMassData, ApplyMassFromShapes, SetType, SetTargetTransform, Disable/Enable, EnableContactRecycling,
		// EnableHitEvents, all force/impulse/torque variants
		bodyId.SetTransform(Pos{1.0, 6.0, 0.0}, QuatIdentity)
		bodyId.SetLinearVelocity(Vec3{0.5, 0.0, 0.0})
		bodyId.SetAngularVelocity(Vec3{0.0, 0.25, 0.0})
		bodyId.SetName("renamedBody")
		bodyId.SetLinearDamping(0.1)
		bodyId.SetAngularDamping(0.05)
		bodyId.SetGravityScale(0.9)
		bodyId.SetSleepThreshold(0.02)
		bodyId.EnableSleep(false)
		bodyId.SetBullet(true)
		bodyId.EnableContactRecycling(false)
		bodyId.EnableHitEvents(true)
		bodyId.SetMotionLocks(MotionLocks{false, false, false, false, false, true})
		var massData MassData
		massData.Mass = 2.0
		massData.Center = Vec3{0.0, 0.0, 0.0}
		massData.Inertia = Mat3Identity
		bodyId.SetMassData(massData)
		bodyId.ApplyMassFromShapes()
		capsuleBodyId.SetType(KinematicBody)
		capsuleBodyId.SetType(DynamicBody)
		bodyId.SetAwake(true)

		// Kinematic body to exercise SetTargetTransform
		kinematicDef := DefaultBodyDef()
		kinematicDef.Type = KinematicBody
		kinematicDef.Position = Pos{-6.0, 5.0, 0.0}
		kinematicId := worldId.CreateBody(&kinematicDef)
		kinBox := MakeBoxHull(0.4, 0.4, 0.4)
		kinShapeDef := DefaultShapeDef()
		kinematicId.CreateHullShape(&kinShapeDef, &kinBox.Base)
		var kinTarget WorldTransform
		kinTarget.P = Pos{-5.0, 5.0, 0.0}
		kinTarget.Q = QuatIdentity
		kinematicId.SetTargetTransform(kinTarget, 1.0/60.0, true)

		// Body to exercise Disable/Enable
		disableDef := DefaultBodyDef()
		disableDef.Type = DynamicBody
		disableDef.Position = Pos{9.0, 5.0, 0.0}
		disableDef.Name = "dropBody"
		disableId := worldId.CreateBody(&disableDef)
		disableSphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.3}
		disableId.CreateSphereShape(&sphereShapeDef, &disableSphere)
		disableId.Disable()
		disableId.Enable()
		disableId.SetSafetyFactor(0.25)

		// A body that falls fast enough to stay on the continuous path for the whole session. The
		// non-default factor is what proves the widened body def payload round-trips, since every
		// other body records the default. Named so the replay check can find it.
		fastDef := DefaultBodyDef()
		fastDef.Type = DynamicBody
		fastDef.Position = Pos{-14.0, 30.0, 0.0}
		fastDef.LinearVelocity = Vec3{0.0, -8.0, 0.0}
		fastDef.SafetyFactor = 0.1
		fastDef.Name = "fastBody"
		fastId := worldId.CreateBody(&fastDef)
		ensure(fastId.GetSafetyFactor() == 0.1)
		fastBox := MakeBoxHull(0.5, 0.5, 0.5)
		fastId.CreateHullShape(&sphereShapeDef, &fastBox.Base)

		// Force/impulse/torque (Vec3 args in 3D)
		bodyId.ApplyForce(Vec3{0.0, 50.0, 0.0}, Pos{1.0, 6.0, 0.0}, true)
		bodyId.ApplyForceToCenter(Vec3{5.0, 0.0, 0.0}, true)
		bodyId.ApplyTorque(Vec3{0.0, 1.0, 0.0}, true)
		bodyId.ApplyLinearImpulse(Vec3{0.1, 0.0, 0.0}, Pos{1.0, 6.0, 0.0}, true)
		bodyId.ApplyLinearImpulseToCenter(Vec3{0.0, 0.1, 0.0}, true)
		bodyId.ApplyAngularImpulse(Vec3{0.0, 0.05, 0.0}, true)

		// Joint bodies: a row of dynamic bodies connected by each joint type
		var jb [9]BodyID
		for i := 0; i < 9; i++ {
			jbd := DefaultBodyDef()
			jbd.Type = DynamicBody
			jbd.Position = Pos{-8.0 + float32(i)*2.0, 10.0, 0.0}
			jb[i] = worldId.CreateBody(&jbd)
			js := Sphere{Vec3{0.0, 0.0, 0.0}, 0.25}
			jsd := DefaultShapeDef()
			jsd.Density = 1.0
			jb[i].CreateSphereShape(&jsd, &js)
		}

		// Revolute joint with full setter coverage and the generic joint mutators
		revDef := DefaultRevoluteJointDef()
		revDef.Base.BodyIdA = jb[0]
		revDef.Base.BodyIdB = jb[1]
		revDef.Base.LocalFrameA.P = Vec3{1.0, 0.0, 0.0}
		revDef.Base.LocalFrameB.P = Vec3{-1.0, 0.0, 0.0}
		revId := worldId.CreateRevoluteJoint(&revDef)
		ensure(revId.IsValid())
		revId.RevoluteJointEnableLimit(true)
		revId.RevoluteJointSetLimits(-1.0, 1.0)
		revId.RevoluteJointEnableMotor(true)
		revId.RevoluteJointSetMotorSpeed(0.5)
		revId.RevoluteJointSetMaxMotorTorque(10.0)
		revId.RevoluteJointEnableSpring(true)
		revId.RevoluteJointSetSpringHertz(2.0)
		revId.RevoluteJointSetSpringDampingRatio(0.5)
		revId.RevoluteJointSetTargetAngle(0.25)
		revId.SetLocalFrameA(Transform{Vec3{1.0, 0.0, 0.0}, QuatIdentity})
		revId.SetLocalFrameB(Transform{Vec3{-1.0, 0.0, 0.0}, QuatIdentity})
		revId.SetConstraintTuning(60.0, 2.0)
		revId.SetForceThreshold(100.0)
		revId.SetTorqueThreshold(50.0)
		revId.SetCollideConnected(false)
		revId.WakeBodies()

		// Distance joint
		distDef := DefaultDistanceJointDef()
		distDef.Base.BodyIdA = jb[1]
		distDef.Base.BodyIdB = jb[2]
		distDef.Length = 2.0
		distId := worldId.CreateDistanceJoint(&distDef)
		distId.DistanceJointSetLength(2.2)
		distId.DistanceJointEnableSpring(true)
		distId.DistanceJointSetSpringHertz(3.0)
		distId.DistanceJointSetSpringDampingRatio(0.4)
		distId.DistanceJointSetSpringForceRange(-50.0, 50.0)
		distId.DistanceJointEnableLimit(true)
		distId.DistanceJointSetLengthRange(1.0, 4.0)
		distId.DistanceJointEnableMotor(true)
		distId.DistanceJointSetMotorSpeed(0.3)
		distId.DistanceJointSetMaxMotorForce(5.0)

		// Filter joint (plus a throwaway to exercise DestroyJoint)
		filterDef := DefaultFilterJointDef()
		filterDef.Base.BodyIdA = jb[2]
		filterDef.Base.BodyIdB = jb[3]
		filterId := worldId.CreateFilterJoint(&filterDef)
		ensure(filterId.IsValid())

		tmpJointDef := DefaultDistanceJointDef()
		tmpJointDef.Base.BodyIdA = jb[0]
		tmpJointDef.Base.BodyIdB = jb[8]
		tmpJointDef.Length = 5.0
		tmpJointId := worldId.CreateDistanceJoint(&tmpJointDef)
		tmpJointId.Destroy(true)

		// Motor joint (Vec3 velocities in 3D)
		motorDef := DefaultMotorJointDef()
		motorDef.Base.BodyIdA = jb[3]
		motorDef.Base.BodyIdB = jb[4]
		motorId := worldId.CreateMotorJoint(&motorDef)
		motorId.MotorJointSetLinearVelocity(Vec3{0.1, 0.0, 0.0})
		motorId.MotorJointSetAngularVelocity(Vec3{0.0, 0.2, 0.0})
		motorId.MotorJointSetMaxVelocityForce(10.0)
		motorId.MotorJointSetMaxVelocityTorque(10.0)
		motorId.MotorJointSetLinearHertz(2.0)
		motorId.MotorJointSetLinearDampingRatio(0.5)
		motorId.MotorJointSetAngularHertz(2.0)
		motorId.MotorJointSetAngularDampingRatio(0.5)
		motorId.MotorJointSetMaxSpringForce(20.0)
		motorId.MotorJointSetMaxSpringTorque(20.0)

		// Prismatic joint
		prisDef := DefaultPrismaticJointDef()
		prisDef.Base.BodyIdA = jb[4]
		prisDef.Base.BodyIdB = jb[5]
		prisId := worldId.CreatePrismaticJoint(&prisDef)
		prisId.PrismaticJointEnableSpring(true)
		prisId.PrismaticJointSetSpringHertz(2.0)
		prisId.PrismaticJointSetSpringDampingRatio(0.5)
		prisId.PrismaticJointSetTargetTranslation(0.1)
		prisId.PrismaticJointEnableLimit(true)
		prisId.PrismaticJointSetLimits(-1.0, 1.0)
		prisId.PrismaticJointEnableMotor(true)
		prisId.PrismaticJointSetMotorSpeed(0.2)
		prisId.PrismaticJointSetMaxMotorForce(8.0)

		// Spherical joint (3D-only)
		sphDef := DefaultSphericalJointDef()
		sphDef.Base.BodyIdA = jb[5]
		sphDef.Base.BodyIdB = jb[6]
		sphId := worldId.CreateSphericalJoint(&sphDef)
		sphId.SphericalJointEnableConeLimit(true)
		sphId.SphericalJointSetConeLimit(0.5)
		sphId.SphericalJointEnableTwistLimit(true)
		sphId.SphericalJointSetTwistLimits(-0.3, 0.3)
		sphId.SphericalJointEnableSpring(true)
		sphId.SphericalJointSetSpringHertz(3.0)
		sphId.SphericalJointSetSpringDampingRatio(0.5)
		sphId.SphericalJointSetTargetRotation(QuatIdentity)
		sphId.SphericalJointEnableMotor(true)
		sphId.SphericalJointSetMotorVelocity(Vec3{0.0, 0.1, 0.0})
		sphId.SphericalJointSetMaxMotorTorque(5.0)

		// Weld joint
		weldDef := DefaultWeldJointDef()
		weldDef.Base.BodyIdA = jb[6]
		weldDef.Base.BodyIdB = jb[7]
		weldId := worldId.CreateWeldJoint(&weldDef)
		weldId.WeldJointSetLinearHertz(5.0)
		weldId.WeldJointSetLinearDampingRatio(0.6)
		weldId.WeldJointSetAngularHertz(5.0)
		weldId.WeldJointSetAngularDampingRatio(0.6)

		// Wheel joint (Box3D uses Suspension/Spin/Steering naming)
		wheelDef := DefaultWheelJointDef()
		wheelDef.Base.BodyIdA = jb[7]
		wheelDef.Base.BodyIdB = jb[8]
		wheelId := worldId.CreateWheelJoint(&wheelDef)
		wheelId.WheelJointEnableSuspension(true)
		wheelId.WheelJointSetSuspensionHertz(4.0)
		wheelId.WheelJointSetSuspensionDampingRatio(0.7)
		wheelId.WheelJointEnableSuspensionLimit(true)
		wheelId.WheelJointSetSuspensionLimits(-0.5, 0.5)
		wheelId.WheelJointEnableSpinMotor(true)
		wheelId.WheelJointSetSpinMotorSpeed(1.0)
		wheelId.WheelJointSetMaxSpinTorque(6.0)
		wheelId.WheelJointEnableSteering(true)
		wheelId.WheelJointSetSteeringHertz(2.0)
		wheelId.WheelJointSetSteeringDampingRatio(0.5)
		wheelId.WheelJointSetMaxSteeringTorque(3.0)
		wheelId.WheelJointEnableSteeringLimit(true)
		wheelId.WheelJointSetSteeringLimits(-0.5, 0.5)
		wheelId.WheelJointSetTargetSteeringAngle(0.1)

		// Parallel joint (3D-only)
		parallelDef := DefaultParallelJointDef()
		parallelDef.Base.BodyIdA = groundId
		parallelDef.Base.BodyIdB = bodyId
		parallelId := worldId.CreateParallelJoint(&parallelDef)
		parallelId.ParallelJointSetSpringHertz(2.0)
		parallelId.ParallelJointSetSpringDampingRatio(0.5)
		parallelId.ParallelJointSetMaxTorque(20.0)

		// World config mutators
		worldId.SetGravity(Vec3{0.0, -9.8, 0.0})
		worldId.EnableSleeping(true)
		worldId.EnableContinuous(false)
		worldId.EnableWarmStarting(true)
		worldId.EnableSpeculative(true)
		worldId.SetRestitutionThreshold(1.5)
		worldId.SetRestitutionIterations(3)
		worldId.EnableRestitutionPropagation(true)
		worldId.SetHitEventThreshold(2.0)
		worldId.SetContactTuning(30.0, 10.0, 3.0)
		worldId.SetContactRecycleDistance(0.05)
		worldId.SetMaximumLinearSpeed(100.0)
		worldId.RebuildStaticTree()

		// The tolerance only reaches bodies whose mass is computed afterwards. A free spinning rod made
		// after the change loses its gyroscopic torque, so replay diverges if the op goes missing.
		worldId.SetIsotropicInertiaTolerance(1.0)
		{
			rodDef := DefaultBodyDef()
			rodDef.Type = DynamicBody
			rodDef.Position = Pos{40.0, 10.0, 0.0}
			rodDef.GravityScale = 0.0
			rodDef.AngularVelocity = Vec3{1.0, 5.0, 0.5}
			rodId := worldId.CreateBody(&rodDef)
			rod := MakeBoxHull(0.1, 1.0, 0.1)
			rodShapeDef := DefaultShapeDef()
			rodId.CreateHullShape(&rodShapeDef, &rod.Base)
		}

		explosion := DefaultExplosionDef()
		explosion.Position = Pos{0.0, 5.0, 0.0}
		explosion.Radius = 3.0
		explosion.Falloff = 1.0
		explosion.ImpulsePerArea = 2.0
		worldId.Explode(&explosion)

		// Pre-step queries
		qfilter := DefaultQueryFilter()
		qaabb := AABB{Vec3{-10.0, -5.0, -10.0}, Vec3{10.0, 15.0, 10.0}}
		worldId.OverlapAABB(qaabb, qfilter, recTestQueryReplayOverlapFcn, nil)
		qorigin := Pos{0.0, 15.0, 0.0}
		proxyPts := Vec3{0.0, 0.0, 0.0}
		proxy := ShapeProxy{[]Vec3{proxyPts}, 1, 0.5}
		worldId.OverlapShape(qorigin, &proxy, qfilter, recTestQueryReplayOverlapFcn, nil)
		qTranslation := Vec3{0.0, -20.0, 0.0}
		worldId.CastRay(qorigin, qTranslation, qfilter, recTestQueryReplayCastFcn, nil)
		worldId.CastRayClosest(qorigin, qTranslation, qfilter)
		worldId.CastShape(qorigin, &proxy, qTranslation, qfilter, recTestQueryReplayCastFcn, nil)
		mover := Capsule{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, 0.3}
		worldId.CastMover(qorigin, &mover, qTranslation, qfilter, recTestQueryReplayMoverFilterFcn, nil)
		worldId.CollideMover(qorigin, &mover, qfilter, recTestQueryReplayPlaneFcn, nil)

		timeStep := float32(1.0 / 60.0)
		subStepCount := 4
		for i := 0; i < 12; i++ {
			// Inject mutators mid-simulation
			if i == 6 {
				capsuleBodyId.ApplyLinearImpulseToCenter(Vec3{2.0, 0.0, 0.0}, true)
				bodyId.SetGravityScale(1.0)
				fastId.SetSafetyFactor(0.4)
			}

			// Issue queries mid-loop to exercise recording across steps
			if i == 3 {
				worldId.OverlapAABB(qaabb, qfilter, recTestQueryReplayOverlapFcn, nil)
				worldId.CastRay(qorigin, qTranslation, qfilter, recTestQueryReplayCastFcn, nil)
			}

			worldId.Step(timeStep, subStepCount)
		}

		worldId.StopRecording()
		worldId.Destroy()

		// Free geometry allocated for this subtest
		DestroyHull(customHull)
		DestroyMesh(meshData)
		DestroyMesh(swapMeshData)
		DestroyHeightField(hf)
		DestroyCompound(compound)
		DestroyMesh(compoundMeshData)

		recData := rec.GetData()
		recSize := rec.GetSize()
		ensure(recSize > 0)

		// Replay headless at worker count 1 and 4, a cross-thread determinism check matching Box2D.
		ensure(ValidateReplay(recData[:recSize], 1))
		ensure(ValidateReplay(recData[:recSize], 4))

		// File round-trip
		ensure(SaveRecordingToFile(rec, recTestRecPath))
		loaded := LoadRecordingFromFile(recTestRecPath)
		ensure(loaded != nil)
		ensure(ValidateReplay(loaded.GetData()[:loaded.GetSize()], 1))
		loaded.Destroy()

		// Drive the incremental player. Exercises per-frame stepping, restart, getters, and the
		// draw path beyond what b3ValidateReplay covers.
		{
			player := CreatePlayer(recData[:recSize], 1)
			ensure(player != nil)

			// Capture keyframes while every geometry kind is live. The default interval is longer
			// than this recording.
			player.SetKeyframePolicy(player.GetKeyframeBudget(), 4)

			info := player.GetInfo()
			recExtents := Sub(info.Bounds.UpperBound, info.Bounds.LowerBound)
			ensure(recExtents.X > 0.0 && recExtents.Y > 0.0)

			// Build a no-op b3DebugDraw to exercise the draw path headlessly
			dd := DefaultDebugDraw()
			dd.DrawShapeFcn = recTestDrawShape
			dd.DrawShapes = true
			dd.DrawingBounds = AABB{Vec3{-1.0e6, -1.0e6, -1.0e6}, Vec3{1.0e6, 1.0e6, 1.0e6}}

			frames := 0
			for player.StepFrame() {
				if frames%2 == 0 {
					player.DrawFrameQueries(&dd, -1, -1)
				}
				frames += 1
			}
			ensure(frames == 12)
			ensure(player.GetFrame() == 12)
			ensure(player.IsAtEnd())
			ensure(player.HasDiverged() == false)
			ensure(player.GetKeyframeBytes() > 0)

			// Seek back onto the frame 4 keyframe, then skip forward onto the frame 8 keyframe. Each
			// restore rebuilds the compound from the registry, so a ray must still reach its tree and
			// the state hashes must hold through the re-stepped frames.
			seekTargets := [2]int{6, 9}
			for k := 0; k < 2; k++ {
				player.SeekFrame(seekTargets[k])
				ensure(player.GetFrame() == seekTargets[k])

				hitCompound := false
				player.GetWorldId().CastRay(Pos{30.0, 10.0, 0.0}, Vec3{0.0, -20.0, 0.0},
					DefaultQueryFilter(), recTestCompoundHitCastFcn, &hitCompound)
				ensure(hitCompound)
			}
			for player.StepFrame() {
			}
			ensure(player.GetFrame() == 12)
			ensure(player.HasDiverged() == false)

			// The trailing DestroyWorld is an end marker; the world stays valid after end
			ensure(player.GetWorldId().IsValid())

			// Restart reproduces the same run without reloading the file
			player.Restart()
			ensure(player.GetFrame() == 0)
			ensure(player.IsAtEnd() == false)

			frames2 := 0
			for player.StepFrame() {
				frames2 += 1
			}
			ensure(frames2 == 12)
			ensure(player.HasDiverged() == false)

			player.Destroy()
		}

		// The state hash only proves the op stream stayed aligned. Read the safety factor back out of
		// the replay world so a value that is written but never restored is caught too.
		{
			player := CreatePlayer(recData[:recSize], 1)
			ensure(player != nil)

			// Frame 0 replays the pre-step creates, so the def value is in place
			ensure(player.StepFrame())
			ensure(recTestReplaySafetyFactor(player, "fastBody") == 0.1)
			ensure(recTestReplaySafetyFactor(player, "dropBody") == 0.25)

			// The setter is injected at frame 6
			for player.StepFrame() {
			}
			ensure(recTestReplaySafetyFactor(player, "fastBody") == 0.4)

			player.Destroy()
		}

		rec.Destroy()
		os.Remove(recTestRecPath)
	})

	// Patch the reserved header bytes to nonzero and confirm b3ValidateReplay ignores them.
	// Guards a future change that starts validating them or shrinks the header.
	It("ReservedHeaderBytes", func() {
		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		worldId.StartRecording(rec)
		worldId.SetGravity(Vec3{0.0, -10.0, 0.0})

		bd := DefaultBodyDef()
		bd.Type = DynamicBody
		bd.Position = Pos{0.0, 5.0, 0.0}
		bodyId := worldId.CreateBody(&bd)
		s := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
		sd := DefaultShapeDef()
		sd.Density = 1.0
		bodyId.CreateSphereShape(&sd, &s)

		for i := 0; i < 10; i++ {
			worldId.Step(1.0/60.0, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()

		recData := rec.GetData()
		recSize := rec.GetSize()
		ensure(recSize >= int(unsafe.Sizeof(recHeader{})))

		// Patch reserved fields at their byte offsets in b3RecHeader:
		//   byte 11 : reserved  (uint8_t at offset 11)
		//   bytes 16-19 : reserved2 (uint32_t at offset 16)
		//   bytes 20-23 : reserved3 (uint32_t at offset 20)
		patched := make([]byte, recSize)
		copy(patched, recData[:recSize])
		patched[11] = 0xAB
		patched[16] = 0xCD
		patched[17] = 0xEF
		patched[18] = 0x12
		patched[19] = 0x34
		patched[20] = 0x56
		patched[21] = 0x78
		patched[22] = 0x9A
		patched[23] = 0xBC
		ensure(ValidateReplay(patched[:recSize], 1))

		rec.Destroy()
	})

	// A corrupt compound in the geometry registry must fail the replay cleanly. A stale version or a
	// wrong kind is rejected when the shape is created and a truncated entry is rejected at load.
	It("CorruptCompound", func() {
		var sphereDef CompoundSphereDef
		sphereDef.Sphere = Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}
		sphereDef.Material = DefaultSurfaceMaterial()
		var compoundDef CompoundDef
		compoundDef.Spheres = []CompoundSphereDef{sphereDef}
		compoundDef.SphereCount = 1
		compound := CreateCompound(&compoundDef)
		ensure(compound != nil)
		compoundByteCount := compound.ByteCount

		rec := CreateRecording(0)
		ensure(rec != nil)

		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		worldId.StartRecording(rec)

		bd := DefaultBodyDef()
		groundId := worldId.CreateBody(&bd)
		sd := DefaultShapeDef()
		groundId.CreateBakedCompoundShape(&sd, compound)

		bd.Type = DynamicBody
		bd.Position = Pos{0.0, 3.0, 0.0}
		bodyId := worldId.CreateBody(&bd)
		s := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
		bodyId.CreateSphereShape(&sd, &s)

		for i := 0; i < 10; i++ {
			worldId.Step(1.0/60.0, 4)
		}

		worldId.StopRecording()
		worldId.Destroy()
		DestroyCompound(compound)

		recData := rec.GetData()
		recSize := rec.GetSize()
		ensure(ValidateReplay(recData[:recSize], 1))

		// The registry entry is a kind byte and a 4 byte length, then the blob, which leads with its version
		version := CompoundVersion
		var versionBytes [8]byte
		binary.LittleEndian.PutUint64(versionBytes[:], version)
		blobOffset := -1
		for i := 5; i+len(versionBytes) <= recSize; i++ {
			if bytes.Equal(recData[i:i+len(versionBytes)], versionBytes[:]) {
				blobOffset = i
				break
			}
		}
		ensure(blobOffset > 0)
		ensure(recData[blobOffset-5] == uint8(geometryCompound))
		storedByteCount := int32(binary.LittleEndian.Uint32(recData[blobOffset-4:]))
		ensure(storedByteCount == compoundByteCount)

		patched := make([]byte, recSize)

		copy(patched, recData[:recSize])
		patched[blobOffset] ^= 0xFF
		ensure(ValidateReplay(patched[:recSize], 1) == false)

		copy(patched, recData[:recSize])
		shortByteCount := int32(8)
		binary.LittleEndian.PutUint32(patched[blobOffset-4:], uint32(shortByteCount))
		ensure(CreatePlayer(patched[:recSize], 1) == nil)

		copy(patched, recData[:recSize])
		zeroByteCount := int32(0)
		binary.LittleEndian.PutUint32(patched[blobOffset-4:], uint32(zeroByteCount))
		ensure(CreatePlayer(patched[:recSize], 1) == nil)

		copy(patched, recData[:recSize])
		patched[blobOffset-5] = uint8(geometryMesh)
		ensure(ValidateReplay(patched[:recSize], 1) == false)

		rec.Destroy()
	})
})

// recTestMemset is memset( p, v, n ) over a byte slice.
func recTestMemset(p []byte, v byte) {
	for i := range p {
		p[i] = v
	}
}
