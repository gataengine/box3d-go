// Port of test/test_world.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// PORT: b3HashWorldState was pulled forward from recording.c, so the 1 vs 4 worker hash
// comparisons of BodySimLocatorTest are real. The static helpers of test_world.c are prefixed with world so they cannot clash in the one
// test package.

const worldBodyCount = 10 // BODY_COUNT

const worldWorldCount = MaxWorlds / 2 // WORLD_COUNT

// worldCustomFilter is CustomFilter.
func worldCustomFilter(shapeIdA, shapeIdB ShapeID, context any) bool {
	ensure(context == nil)
	return true
}

// worldPreSolveStatic is PreSolveStatic.
func worldPreSolveStatic(shapeIdA, shapeIdB ShapeID, point Pos, normal Vec3, context any) bool {
	ensure(context == nil)
	return false
}

const (
	worldChild0MaterialId = 101 // kChild0MaterialId
	worldChild1MaterialId = 202 // kChild1MaterialId
	worldProbeMaterialId  = 999 // kProbeMaterialId
)

// worldMaterialCapture is materialCapture.
// No context pointer on mixing callbacks, so capture through file scope
var worldMaterialCapture struct {
	callCount     int
	sawChild0     bool
	sawChild1     bool
	mixedFriction float32
}

// worldCaptureFrictionMix is CaptureFrictionMix.
func worldCaptureFrictionMix(frictionA float32, userMaterialIdA uint64, frictionB float32, userMaterialIdB uint64) float32 {
	worldMaterialCapture.callCount += 1

	if userMaterialIdA == worldChild0MaterialId || userMaterialIdB == worldChild0MaterialId {
		worldMaterialCapture.sawChild0 = true
	}

	if userMaterialIdA == worldChild1MaterialId || userMaterialIdB == worldChild1MaterialId {
		worldMaterialCapture.sawChild1 = true
		worldMaterialCapture.mixedFriction = sqrtf(frictionA * frictionB)
	}

	return sqrtf(frictionA * frictionB)
}

// worldRunRecycledApproach is RunRecycledApproach.
// A slow sphere closes on a static round shape from just outside contact. The step is too short
// to make the sphere fast, so the pair is eligible for recycling. A routine without a speculative
// margin left the pair empty and recycling then let the sphere sink without a contact. Discussion #175.
func worldRunRecycledApproach(capsuleTarget bool) int {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec3Zero
	worldId := CreateWorld(&worldDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = StaticBody
	targetId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	targetRadius := float32(0.5)
	if capsuleTarget {
		capsule := Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, targetRadius}
		targetId.CreateCapsuleShape(&shapeDef, &capsule)
	} else {
		sphere := Sphere{Vec3Zero, targetRadius}
		targetId.CreateSphereShape(&shapeDef, &sphere)
	}

	radius := float32(0.25)
	totalRadius := targetRadius + radius
	timeStep := float32(1.0 / 60.0)

	// Half the speculative distance per step
	speed := float32(0.5*SpeculativeDistance()) / timeStep

	bodyDef = DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{0.0, totalRadius + float32(0.25*SpeculativeDistance()), 0.0}
	bodyDef.LinearVelocity = Vec3{0.0, -speed, 0.0}
	bodyId := worldId.CreateBody(&bodyDef)

	sphere := Sphere{Vec3Zero, radius}
	bodyId.CreateSphereShape(&shapeDef, &sphere)

	minSeparation := fltMax
	for range 20 {
		worldId.Step(timeStep, 4)

		p := bodyId.GetPosition()
		minSeparation = MinFloat(minSeparation, p.Y-totalRadius)
	}

	worldId.Destroy()

	ensure(minSeparation > -LinearSlop())

	return 0
}

// worldPreSolveReject is PreSolveReject.
func worldPreSolveReject(shapeIdA, shapeIdB ShapeID, point Pos, normal Vec3, context any) bool {
	reject := context.(*bool)
	return *reject == false
}

// worldCountRecycledResting is CountRecycledResting.
func worldCountRecycledResting(enableSpeculative bool) int {
	worldDef := DefaultWorldDef()
	worldDef.EnableSleep = false
	worldId := CreateWorld(&worldDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = StaticBody
	bodyDef.Position = Pos{0.0, -0.5, 0.0}
	groundId := worldId.CreateBody(&bodyDef)
	groundBox := MakeBoxHull(10.0, 0.5, 10.0)
	shapeDef := DefaultShapeDef()
	groundId.CreateHullShape(&shapeDef, &groundBox.Base)

	bodyDef = DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{0.0, 0.5, 0.0}
	bodyId := worldId.CreateBody(&bodyDef)

	shapeDef.EnableSpeculativeContact = enableSpeculative
	box := MakeBoxHull(0.5, 0.5, 0.5)
	bodyId.CreateHullShape(&shapeDef, &box.Base)

	recycledCount := 0
	for range 60 {
		worldId.Step(1.0/60.0, 4)
		recycledCount += int(worldId.GetCounters().RecycledContactCount)
	}

	worldId.Destroy()

	return recycledCount
}

// worldCountMovedNodes is CountMovedNodes.
func worldCountMovedNodes(tree *DynamicTree) int {
	count := 0
	for i := 0; i < int(tree.NodeEnd); i++ {
		node := &tree.Nodes[i]
		if isEmptyNode(node) == false && isNodeMoved(node) {
			count += 1
		}
	}

	return count
}

// worldExplosionResult is ExplosionResult.
type worldExplosionResult struct {
	linearVelocity  Vec3
	angularVelocity Vec3
}

// worldRunExplosion is RunExplosion.
// Explode just off the +x side of a centered sphere and capture the impulse it receives.
// The shape, the blast point, and the witness math all run in the body local frame, so the
// result must not depend on how far the body sits from the world origin.
func worldRunExplosion(base Pos) worldExplosionResult {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec3Zero
	worldId := CreateWorld(&worldDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = base
	bodyId := worldId.CreateBody(&bodyDef)

	sphere := Sphere{Vec3Zero, 1.0}
	shapeDef := DefaultShapeDef()
	bodyId.CreateSphereShape(&shapeDef, &sphere)

	// Blast sits 3 units along +x, so the body is pushed back along -x
	explosionDef := DefaultExplosionDef()
	explosionDef.Position = OffsetPos(base, Vec3{3.0, 0.0, 0.0})
	explosionDef.Radius = 5.0
	explosionDef.Falloff = 0.0
	explosionDef.ImpulsePerArea = 10.0
	worldId.Explode(&explosionDef)

	var result worldExplosionResult
	result.linearVelocity = bodyId.GetLinearVelocity()
	result.angularVelocity = bodyId.GetAngularVelocity()

	worldId.Destroy()
	return result
}

// The contact body sim locators are persistent state. Every contact in every set carries the
// current encoding for both of its bodies: the awake local index, a static tag, or null for a
// body that fell asleep. The scenes below drive each site that can invalidate one. The gate is
// b3ValidateSolverSets, which asserts the invariant on every mutation and every step, so a
// dropped refresh trips there rather than producing a plausible but wrong simulation. Each scene
// also runs at worker counts 1 and 4 with the state hashes compared.

// worldCreateLocatorWorld is CreateLocatorWorld.
func worldCreateLocatorWorld(workerCount int) WorldID {
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	return CreateWorld(&worldDef)
}

// worldCreateLocatorBody is CreateLocatorBody.
func worldCreateLocatorBody(worldId WorldID, typ BodyType, position Pos, hx, hy, hz float32) BodyID {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = typ
	bodyDef.Position = position
	bodyId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	box := MakeBoxHull(hx, hy, hz)
	bodyId.CreateHullShape(&shapeDef, &box.Base)

	return bodyId
}

// worldStepLocatorWorld is StepLocatorWorld.
func worldStepLocatorWorld(worldId WorldID, stepCount int) {
	for range stepCount {
		worldId.Step(1.0/60.0, 4)
	}
}

// worldStepUntilAsleep is StepUntilAsleep.
// Steps until the body sleeps or the budget runs out. Returns the steps taken.
func worldStepUntilAsleep(worldId WorldID, bodyId BodyID, maxSteps int) int {
	for i := range maxSteps {
		worldId.Step(1.0/60.0, 4)
		if bodyId.IsAwake() == false {
			return i + 1
		}
	}

	return maxSteps
}

// worldLocatorSleepWakeScene is LocatorSleepWakeScene.
// Sleep and wake: first touch puts contacts in the graph, sleeping nulls the locators of the
// island bodies, waking restores their awake indices. The plate beside the stack holds a
// non-touching contact across the whole cycle, and that is the contact the wake path has to fix up
// itself. A touching contact would get a fresh encoding from the graph add and hide a dropped
// refresh.
func worldLocatorSleepWakeScene(workerCount int, failed *int) uint64 {
	worldId := worldCreateLocatorWorld(workerCount)
	worldCreateLocatorBody(worldId, StaticBody, Pos{0.0, -1.0, 0.0}, 5.0, 1.0, 5.0)

	var stackId [3]BodyID
	for i := range 3 {
		stackId[i] = worldCreateLocatorBody(worldId, DynamicBody, Pos{0.0, 0.5 + float32(i), 0.0}, 0.5, 0.5, 0.5)
	}

	// Beside the top box, close enough that the fat bounds overlap and far enough that the
	// shapes never touch. Kinematic and awake, so the contact stays in the awake set while the
	// stack sleeps. Off to the side so it does not catch the body dropped later.
	plateId := worldCreateLocatorBody(worldId, KinematicBody, Pos{1.04, 2.5, 0.0}, 0.5, 0.5, 0.5)
	plateId.EnableSleep(false)

	steps := worldStepUntilAsleep(worldId, stackId[0], 300)
	if steps == 300 || stackId[2].IsAwake() == true {
		*failed = 1
	}

	// Wake without contact so the non-touching contact keeps its stale locator if the wake
	// path forgets to refresh
	stackId[0].SetAwake(true)
	worldStepLocatorWorld(worldId, 5)

	if stackId[2].IsAwake() == false {
		*failed = 1
	}

	steps = worldStepUntilAsleep(worldId, stackId[0], 300)
	if steps == 300 {
		*failed = 1
	}

	// Dropping a body on the pile drives the wake path through first touch. The pile can settle
	// and sleep again, so catch the wake as it happens.
	worldCreateLocatorBody(worldId, DynamicBody, Pos{0.0, 8.0, 0.0}, 0.5, 0.5, 0.5)

	woke := false
	for range 180 {
		worldId.Step(1.0/60.0, 4)
		woke = woke || stackId[0].IsAwake()
	}

	if woke == false {
		*failed = 1
	}

	hash := hashWorldState(getWorldFromId(worldId))
	worldId.Destroy()
	return hash
}

// worldLocatorAwakeNeighborScene is LocatorAwakeNeighborScene.
// An island falling asleep and a body leaving the awake set both swap the last awake sim into
// the hole. The pile that never sleeps supplies a moved body that still owns contacts.
func worldLocatorAwakeNeighborScene(workerCount int, failed *int) uint64 {
	worldId := worldCreateLocatorWorld(workerCount)
	worldCreateLocatorBody(worldId, StaticBody, Pos{0.0, -1.0, 0.0}, 30.0, 1.0, 5.0)

	// This island sleeps
	var sleeperId [2]BodyID
	for i := range 2 {
		sleeperId[i] = worldCreateLocatorBody(worldId, DynamicBody, Pos{-10.0, 0.5 + float32(i), 0.0}, 0.5, 0.5, 0.5)
	}

	// Created after the sleepers so its sims sit at the end of the awake set and get swapped
	// into the holes the sleeping island leaves behind
	spareId := worldCreateLocatorBody(worldId, DynamicBody, Pos{20.0, 0.5, 0.0}, 0.5, 0.5, 0.5)
	spareId.EnableSleep(false)

	var awakeId [3]BodyID
	for i := range 3 {
		awakeId[i] = worldCreateLocatorBody(worldId, DynamicBody, Pos{10.0, 0.5 + float32(i), 0.0}, 0.5, 0.5, 0.5)
		awakeId[i].EnableSleep(false)
	}

	steps := worldStepUntilAsleep(worldId, sleeperId[0], 300)
	if steps == 300 || awakeId[0].IsAwake() == false {
		*failed = 1
	}

	worldStepLocatorWorld(worldId, 10)

	// Leaving the awake set swaps a body that owns contacts into the spare's slot
	spareId.Disable()
	worldStepLocatorWorld(worldId, 10)

	spareId.Enable()
	worldStepLocatorWorld(worldId, 10)

	hash := hashWorldState(getWorldFromId(worldId))
	worldId.Destroy()
	return hash
}

// worldLocatorDestroyScene is LocatorDestroyScene.
// Destroying a body swaps the last sim of its set into the hole. The moved body keeps its
// contacts, so its side of each one has to follow it.
func worldLocatorDestroyScene(workerCount int, failed *int) uint64 {
	worldId := worldCreateLocatorWorld(workerCount)
	worldCreateLocatorBody(worldId, StaticBody, Pos{0.0, -1.0, 0.0}, 5.0, 1.0, 5.0)

	var stackId [4]BodyID
	for i := range 4 {
		stackId[i] = worldCreateLocatorBody(worldId, DynamicBody, Pos{0.0, 0.5 + float32(i), 0.0}, 0.5, 0.5, 0.5)
	}

	worldStepLocatorWorld(worldId, 30)

	// The last awake sim moves into this hole while its contacts live
	stackId[0].Destroy()
	worldStepLocatorWorld(worldId, 30)

	// Three static boxes, one resting box each. Destroying the first moves the last static sim.
	var staticId [3]BodyID
	for i := range 3 {
		x := float32(10.0) + float32(4.0*float32(i))
		staticId[i] = worldCreateLocatorBody(worldId, StaticBody, Pos{x, -1.0, 0.0}, 1.0, 1.0, 1.0)
		worldCreateLocatorBody(worldId, DynamicBody, Pos{x, 0.5, 0.0}, 0.5, 0.5, 0.5)
	}

	worldStepLocatorWorld(worldId, 30)

	staticId[0].Destroy()
	worldStepLocatorWorld(worldId, 30)

	_ = failed

	hash := hashWorldState(getWorldFromId(worldId))
	worldId.Destroy()
	return hash
}

// worldLocatorKinematicScene is LocatorKinematicScene.
// A kinematic body parked a hair above a pile keeps a non-touching contact to a body that falls
// asleep, which is the one case the locator cannot name and has to leave to the cold path.
func worldLocatorKinematicScene(workerCount int, failed *int) uint64 {
	worldId := worldCreateLocatorWorld(workerCount)
	worldCreateLocatorBody(worldId, StaticBody, Pos{0.0, -1.0, 0.0}, 5.0, 1.0, 5.0)

	boxId := worldCreateLocatorBody(worldId, DynamicBody, Pos{0.0, 0.5, 0.0}, 0.5, 0.5, 0.5)

	// Close enough that the fat bounds overlap and a non-touching contact exists
	plateId := worldCreateLocatorBody(worldId, KinematicBody, Pos{0.0, 1.52, 0.0}, 1.0, 0.5, 1.0)

	steps := worldStepUntilAsleep(worldId, boxId, 300)
	if steps == 300 {
		*failed = 1
	}

	// Drive the plate down to touch and wake the sleeper
	plateId.SetLinearVelocity(Vec3{0.0, -1.0, 0.0})
	worldStepLocatorWorld(worldId, 60)

	if boxId.IsAwake() == false {
		*failed = 1
	}

	hash := hashWorldState(getWorldFromId(worldId))
	worldId.Destroy()
	return hash
}

// worldLocatorTransferScene is LocatorTransferScene.
// Set type and disable move a body between solver sets, which changes the encoding of every
// contact it has. A joint across two sleeping sets drives the merge path.
func worldLocatorTransferScene(workerCount int, failed *int) uint64 {
	worldId := worldCreateLocatorWorld(workerCount)
	worldCreateLocatorBody(worldId, StaticBody, Pos{0.0, -1.0, 0.0}, 20.0, 1.0, 5.0)

	boxId := worldCreateLocatorBody(worldId, DynamicBody, Pos{0.0, 0.5, 0.0}, 0.5, 0.5, 0.5)
	worldStepLocatorWorld(worldId, 30)

	boxId.SetType(StaticBody)
	worldStepLocatorWorld(worldId, 5)
	boxId.SetType(DynamicBody)
	worldStepLocatorWorld(worldId, 5)

	boxId.Disable()
	worldStepLocatorWorld(worldId, 5)
	boxId.Enable()
	worldStepLocatorWorld(worldId, 30)

	// Two piles far enough apart to sleep in separate sets
	var sleeperId [2]BodyID
	sleeperId[0] = worldCreateLocatorBody(worldId, DynamicBody, Pos{-12.0, 0.5, 0.0}, 0.5, 0.5, 0.5)
	sleeperId[1] = worldCreateLocatorBody(worldId, DynamicBody, Pos{12.0, 0.5, 0.0}, 0.5, 0.5, 0.5)

	steps := worldStepUntilAsleep(worldId, sleeperId[0], 300)
	if steps == 300 || sleeperId[1].IsAwake() == true {
		*failed = 1
	}

	jointDef := DefaultSphericalJointDef()
	jointDef.Base.BodyIdA = sleeperId[0]
	jointDef.Base.BodyIdB = sleeperId[1]
	worldId.CreateSphericalJoint(&jointDef)

	worldStepLocatorWorld(worldId, 30)

	hash := hashWorldState(getWorldFromId(worldId))
	worldId.Destroy()
	return hash
}

// worldLocatorFastFlagTest is LocatorFastFlagTest.
// The fast flag lives on the body sim and gates contact recycling, but a flag sync rebuilds the
// sim flags from the body row, which never carries it. A public setter called on a body in
// flight must not drop it, or the recycle path can take a fast pair.
func worldLocatorFastFlagTest() int {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)

	worldCreateLocatorBody(worldId, StaticBody, Pos{0.0, -1.0, 0.0}, 5.0, 1.0, 5.0)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{-20.0, 2.0, 0.0}
	bodyDef.LinearVelocity = Vec3{400.0, 0.0, 0.0}
	bulletId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.1}
	bulletId.CreateSphereShape(&shapeDef, &sphere)

	worldId.Step(1.0/60.0, 4)

	world := getWorldFromId(worldId)
	body := getBodyFullId(world, bulletId)
	sim := getBodySim(world, body)
	ensure((sim.flags & isFast) != 0)

	// Any public setter that syncs flags. The locks have to actually change or the setter
	// returns early and never syncs.
	locks := MotionLocks{}
	locks.AngularX = true
	bulletId.SetMotionLocks(locks)

	sim = getBodySim(world, getBodyFullId(world, bulletId))
	ensure((sim.flags & isFast) != 0)

	worldId.Step(1.0/60.0, 4)

	worldId.Destroy()
	return 0
}

var _ = Describe("WorldTest", func() {
	// This is a simple example of building and running a simulation
	// using Box3D. Here we create a large ground box and a small dynamic
	// box.
	// There are no graphics for this example. Box3D is meant to be used
	// with your rendering engine in your game engine.
	It("HelloWorld", func() {
		// Construct a world object, which will hold and simulate the rigid bodies.
		worldDef := DefaultWorldDef()
		worldDef.Gravity = Vec3{0.0, -10.0, 0.0}

		worldId := CreateWorld(&worldDef)
		ensure(worldId.IsValid())

		// Define the ground body.
		groundBodyDef := DefaultBodyDef()
		groundBodyDef.Position = Pos{0.0, -10.0, 0.0}

		// Call the body factory which allocates memory for the ground body
		// from a pool and creates the ground box shape (also from a pool).
		// The body is also added to the world.
		groundId := worldId.CreateBody(&groundBodyDef)
		ensure(groundId.IsValid())

		// Define the ground box shape. The extents are the half-widths of the box.
		groundBox := MakeBoxHull(50.0, 10.0, 50.0)

		// Add the box shape to the ground body.
		groundShapeDef := DefaultShapeDef()
		groundId.CreateHullShape(&groundShapeDef, &groundBox.Base)

		// Define the dynamic body. We set its position and call the body factory.
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, 4.0, 0.0}

		bodyId := worldId.CreateBody(&bodyDef)

		// Define another box shape for our dynamic body.
		dynamicBox := MakeCubeHull(1.0)

		// Define the dynamic body shape
		shapeDef := DefaultShapeDef()

		// Set the box density to be non-zero, so it will be dynamic.
		shapeDef.Density = 1.0

		// Override the default friction.
		shapeDef.BaseMaterial.Friction = 0.3

		// Add the shape to the body.
		bodyId.CreateHullShape(&shapeDef, &dynamicBox.Base)

		// Prepare for simulation. Typically we use a time step of 1/60 of a
		// second (60Hz) and 4 sub-steps. This provides a high quality simulation
		// in most game scenarios.
		timeStep := float32(1.0 / 60.0)
		subStepCount := 4

		position := bodyId.GetPosition()
		rotation := bodyId.GetRotation()

		// This is our little game loop.
		for range 90 {
			// Instruct the world to perform a single step of simulation.
			// It is generally best to keep the time step and iterations fixed.
			worldId.Step(timeStep, subStepCount)

			// Now print the position and angle of the body.
			position = bodyId.GetPosition()
			rotation = bodyId.GetRotation()

			// printf("%4.2f %4.2f %4.2f\n", position.x, position.y, b3Rot_GetAngle(rotation));
		}

		// When the world destructor is called, all bodies and joints are freed. This can
		// create orphaned ids, so be careful about your world management.
		worldId.Destroy()

		ensureSmall(position.Y-1.00, 0.01)
		ensureSmall(rotation.V.X, 0.01)
		ensureSmall(rotation.V.Z, 0.01)
	})

	It("EmptyWorld", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		ensure(worldId.IsValid() == true)

		timeStep := float32(1.0 / 60.0)
		subStepCount := 1

		for range 60 {
			worldId.Step(timeStep, subStepCount)
		}

		worldId.Destroy()

		ensure(worldId.IsValid() == false)
	})

	It("DestroyAllBodiesWorld", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		ensure(worldId.IsValid() == true)

		count := 0
		creating := true

		var bodyIds [worldBodyCount]BodyID
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		cube := MakeCubeHull(0.5)

		for range 2*worldBodyCount + 10 {
			if creating {
				if count < worldBodyCount {
					bodyIds[count] = worldId.CreateBody(&bodyDef)

					shapeDef := DefaultShapeDef()
					bodyIds[count].CreateHullShape(&shapeDef, &cube.Base)
					count += 1
				} else {
					creating = false
				}
			} else if count > 0 {
				bodyIds[count-1].Destroy()
				bodyIds[count-1] = NullBodyID
				count -= 1
			}

			worldId.Step(1.0/60.0, 3)
		}

		counters := worldId.GetCounters()
		ensure(counters.BodyCount == 0)

		worldId.Destroy()

		ensure(worldId.IsValid() == false)
	})

	It("TestIsValid", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		ensure(worldId.IsValid())

		bodyDef := DefaultBodyDef()

		bodyId1 := worldId.CreateBody(&bodyDef)
		ensure(bodyId1.IsValid() == true)

		bodyId2 := worldId.CreateBody(&bodyDef)
		ensure(bodyId2.IsValid() == true)

		bodyId1.Destroy()
		ensure(bodyId1.IsValid() == false)

		bodyId2.Destroy()
		ensure(bodyId2.IsValid() == false)

		worldId.Destroy()

		ensure(worldId.IsValid() == false)
		ensure(bodyId2.IsValid() == false)
		ensure(bodyId1.IsValid() == false)
	})

	It("TestWorldRecycle", func() {
		// PORT: _Static_assert( WORLD_COUNT > 0 ) as an overflowing constant.
		const _ uint = worldWorldCount - 1

		count := 100

		var worldIds [worldWorldCount]WorldID

		for range count {
			worldDef := DefaultWorldDef()
			for j := range worldWorldCount {
				worldIds[j] = CreateWorld(&worldDef)
				ensure(worldIds[j].IsValid() == true)

				bodyDef := DefaultBodyDef()
				worldIds[j].CreateBody(&bodyDef)
			}

			for j := range worldWorldCount {
				timeStep := float32(1.0 / 60.0)
				subStepCount := 1

				for range 10 {
					worldIds[j].Step(timeStep, subStepCount)
				}
			}

			for j := worldWorldCount - 1; j >= 0; j-- {
				worldIds[j].Destroy()
				ensure(worldIds[j].IsValid() == false)
				worldIds[j] = NullWorldID
			}
		}
	})

	// This test is here to ensure all API functions link correctly.
	It("TestWorldCoverage", func() {
		worldDef := DefaultWorldDef()

		worldId := CreateWorld(&worldDef)
		ensure(worldId.IsValid())

		worldId.EnableSleeping(true)
		worldId.EnableSleeping(false)
		flag := worldId.IsSleepingEnabled()
		ensure(flag == false)

		worldId.EnableContinuous(false)
		worldId.EnableContinuous(true)
		flag = worldId.IsContinuousEnabled()
		ensure(flag == true)

		worldId.SetRestitutionThreshold(0.0)
		worldId.SetRestitutionThreshold(2.0)
		value := worldId.GetRestitutionThreshold()
		ensure(value == 2.0)

		worldId.SetHitEventThreshold(0.0)
		worldId.SetHitEventThreshold(100.0)
		value = worldId.GetHitEventThreshold()
		ensure(value == 100.0)

		worldId.SetCustomFilterCallback(worldCustomFilter, nil)
		worldId.SetPreSolveCallback(worldPreSolveStatic, nil)

		g := Vec3{1.0, 2.0, 0.0}
		worldId.SetGravity(g)
		v := worldId.GetGravity()
		ensure(v.X == g.X)
		ensure(v.Y == g.Y)

		explosionDef := DefaultExplosionDef()
		worldId.Explode(&explosionDef)

		worldId.SetContactTuning(10.0, 2.0, 4.0)

		worldId.SetMaximumLinearSpeed(10.0)
		value = worldId.GetMaximumLinearSpeed()
		ensure(value == 10.0)

		worldId.SetIsotropicInertiaTolerance(0.5)
		value = worldId.GetIsotropicInertiaTolerance()
		ensure(value == 0.5)

		worldId.SetIsotropicInertiaTolerance(2.0)
		value = worldId.GetIsotropicInertiaTolerance()
		ensure(value == 1.0)

		worldId.EnableWarmStarting(true)
		flag = worldId.IsWarmStartingEnabled()
		ensure(flag == true)

		count := worldId.GetAwakeBodyCount()
		ensure(count == 0)

		worldId.SetUserData(&value)
		userData := worldId.GetUserData()
		ensure(userData == any(&value))

		worldId.Step(1.0, 1)

		worldId.Destroy()
	})

	It("TestExplosion", func() {
		origin := worldRunExplosion(PosZero)

		// Pushed away from the blast along -x. A centered sphere has no transverse or angular component.
		ensure(origin.linearVelocity.X < -1.0e-4)
		ensureSmall(origin.linearVelocity.Y, 1.0e-6)
		ensureSmall(origin.linearVelocity.Z, 1.0e-6)
		ensureSmall(Length(origin.angularVelocity), 1.0e-6)

		// The same blast far from the origin must produce the same impulse. The world position only
		// reaches float in the relative difference, so the result holds where a naive cast would not.
		far := worldRunExplosion(Pos{1.0e7, 1.0e7, 1.0e7})
		ensureSmall(far.linearVelocity.X-origin.linearVelocity.X, 1.0e-5)
		ensureSmall(far.linearVelocity.Y-origin.linearVelocity.Y, 1.0e-5)
		ensureSmall(far.linearVelocity.Z-origin.linearVelocity.Z, 1.0e-5)
	})

	It("TestSensor", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		// Wall from x = 1 to x = 2
		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = Pos{1.5, 11.0, 0.0}
		wallId := worldId.CreateBody(&bodyDef)
		box := MakeBoxHull(0.5, 10.0, 1.0)
		shapeDef := DefaultShapeDef()
		shapeDef.EnableSensorEvents = true
		wallId.CreateHullShape(&shapeDef, &box.Base)

		// Bullet fired towards the wall
		bodyDef = DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.IsBullet = true
		bodyDef.GravityScale = 0.0
		bodyDef.Position = Pos{7.39814, 4.0, 0.0}
		bodyDef.LinearVelocity = Vec3{-20.0, 0.0, 0.0}
		bulletId := worldId.CreateBody(&bodyDef)
		shapeDef = DefaultShapeDef()
		shapeDef.IsSensor = true
		shapeDef.EnableSensorEvents = true
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.1}
		bulletId.CreateSphereShape(&shapeDef, &sphere)

		beginCount := 0
		endCount := 0

		for {
			timeStep := float32(1.0 / 60.0)
			subStepCount := 4
			worldId.Step(timeStep, subStepCount)

			bulletPos := bulletId.GetPosition()
			// printf( "Bullet pos: %g %g\n", bulletPos.x, bulletPos.y );

			events := worldId.GetSensorEvents()

			if events.BeginCount > 0 {
				beginCount += 1
			}

			if events.EndCount > 0 {
				endCount += 1
			}

			if bulletPos.X < -1.0 {
				break
			}
		}

		worldId.Destroy()

		ensure(beginCount == 1)
		ensure(endCount == 1)
	})

	// Ensure correct move events from bodies involved in CCD.
	It("TestContinuousMoveEvent", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)
		worldId.EnableContinuous(true)

		// Thin static wall, near face at x = 0.1
		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = Pos{0.0, 0.0, 0.0}
		wallId := worldId.CreateBody(&bodyDef)
		wallBox := MakeBoxHull(0.1, 5.0, 5.0)
		shapeDef := DefaultShapeDef()
		wallId.CreateHullShape(&shapeDef, &wallBox.Base)

		// Fast dynamic sphere fired at the wall.
		bodyDef = DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.GravityScale = 0.0
		bodyDef.Position = Pos{3.0, 0.0, 0.0}
		bodyDef.LinearVelocity = Vec3{-30.0, 0.0, 0.0}
		ballId := worldId.CreateBody(&bodyDef)
		shapeDef = DefaultShapeDef()
		shapeDef.Density = 1.0
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.25}
		ballId.CreateSphereShape(&shapeDef, &sphere)

		timeStep := float32(1.0 / 60.0)
		subStepCount := 4
		haveMove := false

		for range 30 {
			worldId.Step(timeStep, subStepCount)

			xf := ballId.GetTransform()

			events := worldId.GetBodyEvents()
			for i := 0; i < int(events.MoveCount); i++ {
				event := &events.MoveEvents[i]
				if (event.BodyId == ballId) == false {
					continue
				}

				haveMove = true

				// The move event must carry the same pose the body reports, CCD rewind included
				ensure(event.Transform.P.X == xf.P.X)
				ensure(event.Transform.P.Y == xf.P.Y)
				ensure(event.Transform.P.Z == xf.P.Z)
				ensure(event.Transform.Q.V.X == xf.Q.V.X)
				ensure(event.Transform.Q.V.Y == xf.Q.V.Y)
				ensure(event.Transform.Q.V.Z == xf.Q.V.Z)
				ensure(event.Transform.Q.S == xf.Q.S)
			}
		}

		ensure(haveMove == true)

		// Tunnel check
		finalPos := ballId.GetPosition()
		ensure(0.2 < finalPos.X && finalPos.X < 0.8)

		worldId.Destroy()
	})

	It("TestContactEvents", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		// Static ground
		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = Pos{0.0, -0.5, 0.0}
		groundId := worldId.CreateBody(&bodyDef)
		groundBox := MakeBoxHull(10.0, 0.5, 10.0)
		groundShapeDef := DefaultShapeDef()
		groundShapeId := groundId.CreateHullShape(&groundShapeDef, &groundBox.Base)

		// Dynamic sphere dropped onto the ground; restitution causes it to bounce so we get end events
		bodyDef = DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, 5.0, 0.0}
		sphereBodyId := worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		shapeDef.EnableContactEvents = true
		shapeDef.BaseMaterial.Restitution = 0.6
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
		sphereShapeId := sphereBodyId.CreateSphereShape(&shapeDef, &sphere)

		beginCount := 0
		endCount := 0
		idsChecked := false

		for range 120 {
			worldId.Step(1.0/60.0, 4)

			events := worldId.GetContactEvents()

			if events.BeginCount > 0 && idsChecked == false {
				be := events.BeginEvents[0]
				aIsSphere := be.ShapeIdA == sphereShapeId
				bIsSphere := be.ShapeIdB == sphereShapeId
				aIsGround := be.ShapeIdA == groundShapeId
				bIsGround := be.ShapeIdB == groundShapeId
				ensure((aIsSphere && bIsGround) || (aIsGround && bIsSphere))
				ensure(be.ContactId.IsValid())
				idsChecked = true
			}

			beginCount += int(events.BeginCount)
			endCount += int(events.EndCount)
		}

		worldId.Destroy()

		ensure(idsChecked)
		ensure(beginCount >= 1)
		ensure(endCount >= 1)
	})

	It("TestHitEvents", func() {
		worldDef := DefaultWorldDef()
		worldDef.HitEventThreshold = 1.0
		worldId := CreateWorld(&worldDef)

		// Static ground
		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = Pos{0.0, -0.5, 0.0}
		groundId := worldId.CreateBody(&bodyDef)
		groundBox := MakeBoxHull(10.0, 0.5, 10.0)
		groundShapeDef := DefaultShapeDef()
		groundId.CreateHullShape(&groundShapeDef, &groundBox.Base)

		// Sphere driven into the ground fast enough to clear the hit threshold
		bodyDef = DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.GravityScale = 0.0
		bodyDef.Position = Pos{0.0, 2.0, 0.0}
		bodyDef.LinearVelocity = Vec3{0.0, -30.0, 0.0}
		sphereBodyId := worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		shapeDef.EnableHitEvents = true
		shapeDef.BaseMaterial.UserMaterialId = 7
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
		sphereBodyId.CreateSphereShape(&shapeDef, &sphere)

		hitCount := 0
		capturedSpeed := float32(0.0)
		capturedMaterialA := uint64(0)
		capturedMaterialB := uint64(0)
		capturedNormal := Vec3{0.0, 0.0, 0.0}

		for range 30 {
			worldId.Step(1.0/60.0, 4)

			events := worldId.GetContactEvents()
			if events.HitCount > 0 && hitCount == 0 {
				hit := events.HitEvents[0]
				capturedSpeed = hit.ApproachSpeed
				capturedNormal = hit.Normal
				capturedMaterialA = hit.UserMaterialIdA
				capturedMaterialB = hit.UserMaterialIdB
			}

			hitCount += int(events.HitCount)
		}

		worldId.Destroy()

		ensure(hitCount >= 1)
		ensure(capturedSpeed > 1.0)
		// Head-on vertical impact: normal lies along Y
		ensureSmall(capturedNormal.X, 0.01)
		ensureSmall(capturedNormal.Z, 0.01)
		// One side of the contact carries the sphere's user material
		ensure(capturedMaterialA == 7 || capturedMaterialB == 7)
	})

	// Hit-event material lookup must respect the compound child that participated in the
	// contact. Two children with distinct userMaterialIds at separated positions, dropped
	// sphere strikes one specifically. Without the fix, both children would report
	// materials[0] and the strike on hull 1 would be misattributed.
	It("TestCompoundHitEvents", func() {
		const kHullMaterialA uint64 = 11
		const kHullMaterialB uint64 = 22
		const kSphereMaterial uint64 = 99
		const kHullCenterX float32 = 3.0

		for side := range 2 {
			expectedHullMaterial := kHullMaterialB
			spawnX := kHullCenterX
			if side == 0 {
				expectedHullMaterial = kHullMaterialA
				spawnX = -kHullCenterX
			}

			worldDef := DefaultWorldDef()
			worldDef.HitEventThreshold = 1.0
			worldId := CreateWorld(&worldDef)

			// Build a compound with two hulls at opposite x positions, distinct userMaterialIds
			boxA := MakeBoxHull(1.0, 1.0, 1.0)
			boxB := MakeBoxHull(1.0, 1.0, 1.0)

			matA := DefaultSurfaceMaterial()
			matA.UserMaterialId = kHullMaterialA

			matB := DefaultSurfaceMaterial()
			matB.UserMaterialId = kHullMaterialB

			var hulls [2]CompoundHullDef
			hulls[0].Hull = &boxA.Base
			hulls[0].Transform = Transform{Vec3{-kHullCenterX, 0.0, 0.0}, QuatIdentity}
			hulls[0].Material = matA
			hulls[1].Hull = &boxB.Base
			hulls[1].Transform = Transform{Vec3{kHullCenterX, 0.0, 0.0}, QuatIdentity}
			hulls[1].Material = matB

			compoundDef := CompoundDef{}
			compoundDef.Hulls = hulls[:]
			compoundDef.HullCount = 2
			compound := CreateCompound(&compoundDef)
			ensure(compound != nil)

			// Static body holds the compound
			bodyDef := DefaultBodyDef()
			bodyDef.Type = StaticBody
			compoundBodyId := worldId.CreateBody(&bodyDef)
			compoundShapeDef := DefaultShapeDef()
			compoundBodyId.CreateBakedCompoundShape(&compoundShapeDef, compound)

			// Sphere driven straight down onto the chosen child
			bodyDef = DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.GravityScale = 0.0
			bodyDef.Position = Pos{spawnX, 3.0, 0.0}
			bodyDef.LinearVelocity = Vec3{0.0, -30.0, 0.0}
			sphereBodyId := worldId.CreateBody(&bodyDef)
			sphereShapeDef := DefaultShapeDef()
			sphereShapeDef.Density = 1.0
			sphereShapeDef.EnableHitEvents = true
			sphereShapeDef.BaseMaterial.UserMaterialId = kSphereMaterial
			sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
			sphereBodyId.CreateSphereShape(&sphereShapeDef, &sphere)

			hitCount := 0
			capturedMaterialA := uint64(0)
			capturedMaterialB := uint64(0)

			for range 30 {
				worldId.Step(1.0/60.0, 4)

				events := worldId.GetContactEvents()
				if events.HitCount > 0 && hitCount == 0 {
					hit := events.HitEvents[0]
					capturedMaterialA = hit.UserMaterialIdA
					capturedMaterialB = hit.UserMaterialIdB
				}
				hitCount += int(events.HitCount)
			}

			worldId.Destroy()
			DestroyCompound(compound)

			ensure(hitCount >= 1)
			// Sphere material on one side
			ensure(capturedMaterialA == kSphereMaterial || capturedMaterialB == kSphereMaterial)
			// Struck compound child's material on the other side. The pre-fix code returned
			// materials[0] (kHullMaterialA) for both sides, so a strike on the +x child would fail.
			ensure(capturedMaterialA == expectedHullMaterial || capturedMaterialB == expectedHullMaterial)
		}
	})

	// Contact material selection must use the struck compound child's material, not entry 0
	// of the compound material table. Child 0 gets low friction, child 1 high friction, and a
	// unit friction sphere strikes only child 1, so the mixing callback must see child 1's
	// values. Covers sphere, capsule, and hull children. Issue #69.
	It("TestCompoundContactMaterials", func() {
		mat0 := DefaultSurfaceMaterial()
		mat0.Friction = 0.04
		mat0.UserMaterialId = worldChild0MaterialId

		mat1 := DefaultSurfaceMaterial()
		mat1.Friction = 0.81
		mat1.UserMaterialId = worldChild1MaterialId

		box := MakeBoxHull(0.5, 0.5, 0.5)

		// Children at x = -3 and x = +3, all with their top face or surface at y = 0.5
		spheres := [2]CompoundSphereDef{
			{Sphere: Sphere{Vec3{-3.0, 0.0, 0.0}, 0.5}, Material: mat0},
			{Sphere: Sphere{Vec3{3.0, 0.0, 0.0}, 0.5}, Material: mat1},
		}

		capsules := [2]CompoundCapsuleDef{
			{Capsule: Capsule{Vec3{-3.0, 0.0, -0.5}, Vec3{-3.0, 0.0, 0.5}, 0.5}, Material: mat0},
			{Capsule: Capsule{Vec3{3.0, 0.0, -0.5}, Vec3{3.0, 0.0, 0.5}, 0.5}, Material: mat1},
		}

		hulls := [2]CompoundHullDef{
			{Hull: &box.Base, Transform: Transform{Vec3{-3.0, 0.0, 0.0}, QuatIdentity}, Material: mat0},
			{Hull: &box.Base, Transform: Transform{Vec3{3.0, 0.0, 0.0}, QuatIdentity}, Material: mat1},
		}

		for childType := range 3 {
			compoundDef := CompoundDef{}
			if childType == 0 {
				compoundDef.Spheres = spheres[:]
				compoundDef.SphereCount = 2
			} else if childType == 1 {
				compoundDef.Capsules = capsules[:]
				compoundDef.CapsuleCount = 2
			} else {
				compoundDef.Hulls = hulls[:]
				compoundDef.HullCount = 2
			}

			compound := CreateCompound(&compoundDef)
			ensure(compound != nil)

			// PORT: memset( &materialCapture, 0, sizeof( materialCapture ) )
			worldMaterialCapture.callCount = 0
			worldMaterialCapture.sawChild0 = false
			worldMaterialCapture.sawChild1 = false
			worldMaterialCapture.mixedFriction = 0.0

			worldDef := DefaultWorldDef()
			worldDef.FrictionCallback = worldCaptureFrictionMix
			worldId := CreateWorld(&worldDef)

			bodyDef := DefaultBodyDef()
			bodyDef.Type = StaticBody
			compoundBodyId := worldId.CreateBody(&bodyDef)
			compoundShapeDef := DefaultShapeDef()
			compoundBodyId.CreateBakedCompoundShape(&compoundShapeDef, compound)

			// Sphere driven straight down onto child 1
			bodyDef = DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.GravityScale = 0.0
			bodyDef.Position = Pos{3.0, 3.0, 0.0}
			bodyDef.LinearVelocity = Vec3{0.0, -30.0, 0.0}
			sphereBodyId := worldId.CreateBody(&bodyDef)
			sphereShapeDef := DefaultShapeDef()
			sphereShapeDef.Density = 1.0
			sphereShapeDef.BaseMaterial.Friction = 1.0
			sphereShapeDef.BaseMaterial.UserMaterialId = worldProbeMaterialId
			sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
			sphereBodyId.CreateSphereShape(&sphereShapeDef, &sphere)

			for range 30 {
				worldId.Step(1.0/60.0, 4)
			}

			worldId.Destroy()
			DestroyCompound(compound)

			ensure(worldMaterialCapture.callCount > 0)
			// The pre-fix code fed material table entry 0 to the mixing callback for every child
			ensure(worldMaterialCapture.sawChild0 == false)
			ensure(worldMaterialCapture.sawChild1 == true)
			// sqrt(0.81 * 1.0)
			ensureSmall(worldMaterialCapture.mixedFriction-0.9, 1e-5)
		}
	})

	// Verifies the b3*_Overflow solver path. The scene puts >B3_DYNAMIC_COLOR_COUNT
	// dyn-dyn contacts on a single hub body so several land in the overflow color.
	// The new Prepare/Store contactId-pairing asserts in contact_solver.c fire here
	// if Store reads constraints from the wrong (base, spans) pair, so the test
	// catches the bug even though world state ends up plausible (memory layout is
	// stable within a build, so determinism alone is not a witness).
	It("TestOverflowColorPile", func() {
		worldDef := DefaultWorldDef()
		worldDef.WorkerCount = 1
		worldId := CreateWorld(&worldDef)

		data := createOverflowColorPile(worldId)
		_ = data

		timeStep := float32(1.0 / 60.0)
		subStepCount := 4

		// One step would be enough to trip the asserts, but several steps also
		// exercise the warm-start path (Store -> manifold impulse -> Prepare).
		stepCount := 10
		for range stepCount {
			worldId.Step(timeStep, subStepCount)
		}

		// Confirm the scene actually populated the overflow color. Without this,
		// a future change to graph coloring could silently turn the test into a
		// no-op.
		counters := worldId.GetCounters()
		overflowContacts := counters.ColorCounts[GraphColorCount-1]

		worldId.Destroy()

		ensure(overflowContacts > 0)
	})

	// Exposes that b3Body_SetBullet writes only bodySim->flags without
	// touching body->flags, while b3SyncBodyFlags overwrites bodySim from
	// body->flags. Any subsequent b3Body_SetMotionLocks or b3Body_SetType
	// silently wipes (or re-asserts) the bullet bit.
	//
	// Reproducer A: create with isBullet=false, SetBullet(true), then
	// SetMotionLocks. b3Body_IsBullet returns false today (wiped); should be
	// true after fix.
	//
	// Reproducer B: create with isBullet=true, SetBullet(false), then
	// SetMotionLocks. b3Body_IsBullet returns true today (re-asserted); should
	// be false after fix.
	It("SetBulletDriftTest", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		{
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.IsBullet = false
			bodyId := worldId.CreateBody(&bodyDef)

			ensure(bodyId.IsBullet() == false)

			bodyId.SetBullet(true)
			ensure(bodyId.IsBullet() == true)

			locks := MotionLocks{}
			locks.LinearX = true
			bodyId.SetMotionLocks(locks)

			ensure(bodyId.IsBullet() == true)
		}

		{
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.IsBullet = true
			bodyId := worldId.CreateBody(&bodyDef)

			ensure(bodyId.IsBullet() == true)

			bodyId.SetBullet(false)
			ensure(bodyId.IsBullet() == false)

			locks := MotionLocks{}
			locks.LinearX = true
			bodyId.SetMotionLocks(locks)

			ensure(bodyId.IsBullet() == false)
		}

		worldId.Destroy()
	})

	// Exposes that b3Body_EnableSleep mutates body->flags but never syncs
	// bodySim->flags / bodyState->flags. When a body created with
	// enableSleep=false is later flipped on, body->flags gains b3_enableSleep
	// but bodySim/bodyState do not. The flag-sync assertion in
	// b3ValidateSolverSets then fires on the next world step.
	//
	// Under Debug + BOX3D_VALIDATE this crashes; after b3Body_EnableSleep
	// calls b3SyncBodyFlags the test runs cleanly.
	It("EnableSleepFlagSyncTest", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.EnableSleep = false
		bodyId := worldId.CreateBody(&bodyDef)

		ensure(bodyId.IsSleepEnabled() == false)

		bodyId.EnableSleep(true)
		ensure(bodyId.IsSleepEnabled() == true)

		worldId.Step(1.0/60.0, 4)

		worldId.Destroy()
	})

	// Regression: b3Body_EnableSleep used to set world->locked = true before checking
	// for a no-op change, then early-return without unlocking. The next mutator call
	// would then trip the assert in b3GetUnlockedWorld.
	It("EnableSleepNoopUnlockTest", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.EnableSleep = true
		bodyId := worldId.CreateBody(&bodyDef)

		// No-op: enableSleep is already true. Must not leak the world lock.
		bodyId.EnableSleep(true)

		// Would assert in b3GetUnlockedWorld if the lock had leaked.
		bodyId.EnableSleep(false)
		ensure(bodyId.IsSleepEnabled() == false)

		worldId.Destroy()
	})

	It("EnableContactRecyclingTest", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody

		// Default is enabled
		bodyA := worldId.CreateBody(&bodyDef)
		ensure(bodyA.IsContactRecyclingEnabled() == true)

		bodyA.EnableContactRecycling(false)
		ensure(bodyA.IsContactRecyclingEnabled() == false)

		bodyA.EnableContactRecycling(true)
		ensure(bodyA.IsContactRecyclingEnabled() == true)

		// Per-def opt-out at creation
		bodyDef.EnableContactRecycling = false
		bodyB := worldId.CreateBody(&bodyDef)
		ensure(bodyB.IsContactRecyclingEnabled() == false)

		// Stepping after toggling must not trip the flag-sync validator
		worldId.Step(1.0/60.0, 4)

		worldId.Destroy()
	})

	It("RecycledSphereApproachTest", func() {
		ensure(worldRunRecycledApproach(false) == 0)
		ensure(worldRunRecycledApproach(true) == 0)
	})

	// A rejected pre-solve frees the manifold while the shapes may already touch. The next step must
	// rebuild the contact and ask again instead of recycling the empty pair.
	It("PreSolveRejectRecycleTest", func() {
		worldDef := DefaultWorldDef()
		worldDef.EnableSleep = false
		worldId := CreateWorld(&worldDef)

		reject := true
		worldId.SetPreSolveCallback(worldPreSolveReject, &reject)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = Pos{0.0, -0.5, 0.0}
		groundId := worldId.CreateBody(&bodyDef)
		groundBox := MakeBoxHull(10.0, 0.5, 10.0)
		shapeDef := DefaultShapeDef()
		groundId.CreateHullShape(&shapeDef, &groundBox.Base)

		radius := float32(0.25)
		bodyDef = DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, radius, 0.0}
		bodyId := worldId.CreateBody(&bodyDef)

		shapeDef.EnablePreSolveEvents = true
		sphere := Sphere{Vec3Zero, radius}
		bodyId.CreateSphereShape(&shapeDef, &sphere)

		worldId.Step(1.0/60.0, 4)

		reject = false

		minSeparation := fltMax
		for range 30 {
			worldId.Step(1.0/60.0, 4)

			p := bodyId.GetPosition()
			minSeparation = MinFloat(minSeparation, p.Y-radius)
		}

		worldId.Destroy()

		ensure(minSeparation > -LinearSlop())
	})

	// Without speculative points an empty pair may be closer than the speculative distance, so these
	// contacts must never recycle
	It("SpeculativeOffRecycleTest", func() {
		ensure(worldCountRecycledResting(true) > 0)
		ensure(worldCountRecycledResting(false) == 0)
	})

	It("TestSetWorkerCount", func() {
		worldDef := DefaultWorldDef()
		worldDef.WorkerCount = 1
		worldId := CreateWorld(&worldDef)
		ensure(worldId.IsValid())
		ensure(worldId.GetWorkerCount() == 1)

		createJunkyard(worldId)
		stepJunkyard(worldId, 1)

		worldId.SetWorkerCount(4)
		ensure(worldId.GetWorkerCount() == 4)

		stepJunkyard(worldId, 2)

		worldId.SetWorkerCount(4)
		ensure(worldId.GetWorkerCount() == 4)

		stepJunkyard(worldId, 3)

		worldId.SetWorkerCount(0)
		ensure(worldId.GetWorkerCount() == 1)

		stepJunkyard(worldId, 4)

		worldId.SetWorkerCount(-5)
		ensure(worldId.GetWorkerCount() == 1)

		stepJunkyard(worldId, 5)

		worldId.SetWorkerCount(MaxWorkers + 10)
		ensure(worldId.GetWorkerCount() == MaxWorkers)

		stepJunkyard(worldId, 2)

		worldId.Destroy()
	})

	// Identical hull data is shared through a reference counted world database.
	It("TestHullDatabase", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		box := MakeBoxHull(0.5, 0.5, 0.5)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyA := worldId.CreateBody(&bodyDef)
		bodyB := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()

		// Two shapes built from identical data share one owned copy in the world database.
		shapeA := bodyA.CreateHullShape(&shapeDef, &box.Base)
		shapeB := bodyB.CreateHullShape(&shapeDef, &box.Base)

		gotA := shapeA.GetHull()
		gotB := shapeB.GetHull()

		// Both shapes point at the single shared copy
		ensure(gotA == gotB)

		// The shared copy is owned by the world, not the caller's stack hull
		ensure(gotA != &box.Base)

		// A box built on an independent stack frame must de-duplicate to the same shared copy.
		// This holds only if content hashing sees deterministic padding bytes.
		box2 := MakeBoxHull(0.5, 0.5, 0.5)
		bodyC := worldId.CreateBody(&bodyDef)
		shapeC := bodyC.CreateHullShape(&shapeDef, &box2.Base)
		ensure(shapeC.GetHull() == gotA)
		shapeC.Destroy(true)

		// Setting a shape's hull to its own sole shared copy must not free it mid update.
		box3 := MakeBoxHull(0.3, 0.3, 0.3)
		bodyD := worldId.CreateBody(&bodyDef)
		shapeD := bodyD.CreateHullShape(&shapeDef, &box3.Base)
		gotD := shapeD.GetHull()
		shapeD.SetHull(gotD)
		ensure(shapeD.GetHull() == gotD)
		shapeD.Destroy(true)

		// Releasing one reference keeps the other alive
		shapeA.Destroy(true)
		stillB := shapeB.GetHull()
		ensure(stillB == gotB)

		shapeB.Destroy(true)

		// World destroy asserts the database drained to zero references
		worldId.Destroy()
	})

	// A proxy can be enlarged in one step and destroyed before the next, emptying the move buffer.
	// The trees must still be rebuilt or the enlarged nodes survive into the next step. Issue #149.
	It("TestEnlargedProxyDestroyed", func() {
		worldDef := DefaultWorldDef()
		worldDef.Gravity = Vec3Zero
		worldId := CreateWorld(&worldDef)

		world := getWorldFromId(worldId)
		tree := &world.broadPhase.trees[DynamicBody]

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody

		shapeDef := DefaultShapeDef()
		sphere := Sphere{Vec3Zero, 0.25}

		// Resting bodies keep the tree deep enough that the mover has an ancestor that outlives it
		for i := range 8 {
			bodyDef.Position = Pos{2.0 * float32(i), 0.0, 0.0}
			restingId := worldId.CreateBody(&bodyDef)
			restingId.CreateSphereShape(&shapeDef, &sphere)
		}

		// This body outruns its AABB margin every step
		bodyDef.Position = Pos{7.0, 1.0, 0.0}
		bodyDef.LinearVelocity = Vec3{6.0, 0.0, 0.0}
		moverId := worldId.CreateBody(&bodyDef)
		moverId.CreateSphereShape(&shapeDef, &sphere)

		timeStep := float32(1.0 / 60.0)
		worldId.Step(timeStep, 4)

		ensure(worldCountMovedNodes(tree) > 0)

		moverId.Destroy()

		// Removing the leaf re-derives the marks along its path, so the tree is already consistent
		// here and nothing is left marked to drive a rebuild. What the destroy can still leave
		// stale is the DFS order the sweep refit needs, so the step has to rebuild regardless.
		tree.Validate()
		ensure(worldCountMovedNodes(tree) == 0)

		worldId.Step(timeStep, 4)

		ensure(worldCountMovedNodes(tree) == 0)
		ensure(tree.DfsOrdered)

		worldId.Destroy()
	})

	// b3DestroyBody tears shapes down inline instead of going through the shape destroy path, so it
	// needs its own compound census decrement. A leaked count pins the broad-phase on the slow
	// per-candidate compound path for the life of the world and rides along into snapshots.
	It("TestCompoundShapeCount", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		world := getWorldFromId(worldId)
		ensure(world.compoundShapeCount == 0)

		box := MakeBoxHull(1.0, 1.0, 1.0)

		var hull CompoundHullDef
		hull.Hull = &box.Base
		hull.Transform = Transform{Vec3Zero, QuatIdentity}
		hull.Material = DefaultSurfaceMaterial()

		compoundDef := CompoundDef{}
		compoundDef.Hulls = []CompoundHullDef{hull}
		compoundDef.HullCount = 1
		compound := CreateCompound(&compoundDef)
		ensure(compound != nil)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		shapeDef := DefaultShapeDef()

		// Shape destroyed on its own
		bodyId := worldId.CreateBody(&bodyDef)
		shapeId := bodyId.CreateBakedCompoundShape(&shapeDef, compound)
		ensure(world.compoundShapeCount == 1)
		shapeId.Destroy(true)
		ensure(world.compoundShapeCount == 0)
		bodyId.Destroy()

		// Shape carried away by its body
		bodyId = worldId.CreateBody(&bodyDef)
		bodyId.CreateBakedCompoundShape(&shapeDef, compound)
		ensure(world.compoundShapeCount == 1)
		bodyId.Destroy()
		ensure(world.compoundShapeCount == 0)

		worldId.Destroy()
		DestroyCompound(compound)
	})

	It("BodySimLocatorTest", func() {
		scenes := [5]func(workerCount int, failed *int) uint64{
			worldLocatorSleepWakeScene,
			worldLocatorAwakeNeighborScene,
			worldLocatorDestroyScene,
			worldLocatorKinematicScene,
			worldLocatorTransferScene,
		}

		for i := range 5 {
			failed := 0
			hash1 := scenes[i](1, &failed)
			hash4 := scenes[i](4, &failed)
			ensure(failed == 0)
			ensure(hash1 == hash4)
		}

		ensure(worldLocatorFastFlagTest() == 0)
	})
})
