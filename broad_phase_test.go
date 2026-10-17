// Port of test/test_broad_phase.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// PORT: BroadPhaseTest's b3SetAssertFcn( FlushingAssert ) exists to flush stdout before a crash in the C
// runner and is not needed under Ginkgo. The static helpers are prefixed with broadPhase so they
// cannot clash in the one test package. printf goes to GinkgoWriter; the product in RandomFloat is
// rounded explicitly so Go cannot fuse it into the add; random draws inside C initializer lists
// are hoisted into locals, left to right as clang evaluates them.

// broadPhaseCheckPairSet is CheckPairSet.
// Brute force check of the broad-phase pair set against every live shape pair. Must run right
// after a zero time step so the pair update and the disjoint contact purge have both happened
// against the current fat boxes.
func broadPhaseCheckPairSet(worldId WorldID) int {
	world := getWorldFromId(worldId)
	bp := &world.broadPhase
	shapes := world.shapes
	fatAABBs := world.fatAABBs
	shapeCount := len(world.shapes)

	for i := 0; i < shapeCount; i++ {
		shapeA := &shapes[i]
		if shapeA.id == NullIndex || shapeA.proxyKey == NullIndex || shapeA.sensorIndex != NullIndex {
			continue
		}

		bodyA := &world.bodies[shapeA.bodyId]

		for j := i + 1; j < shapeCount; j++ {
			shapeB := &shapes[j]
			if shapeB.id == NullIndex || shapeB.proxyKey == NullIndex || shapeB.sensorIndex != NullIndex {
				continue
			}

			if shapeA.bodyId == shapeB.bodyId {
				continue
			}

			bodyB := &world.bodies[shapeB.bodyId]

			expected := shouldShapesCollide(shapeA.filter, shapeB.filter) &&
				shouldBodiesCollide(world, bodyA, bodyB) && fatAABBs[i].Overlaps(fatAABBs[j])

			actual := containsKey(&bp.pairSet, shapePairKey(i, j, 0))

			// A sleeping contact is never refreshed, so a stale pair can outlive the overlap when
			// the other body is teleported, and a set woken by the narrow phase carries its stale
			// contacts into the awake set one collide late. With sleeping on only a missing pair
			// is a bug.
			if world.enableSleep && actual && expected == false {
				continue
			}

			if expected != actual {
				GinkgoWriter.Printf("  pair mismatch: shapes %d (body %d type %d set %d) and %d (body %d type %d set %d), expected %d, "+
					"actual %d\n",
					i, shapeA.bodyId, bodyA.typ, bodyA.setIndex, j, shapeB.bodyId, bodyB.typ, bodyB.setIndex,
					broadPhaseBoolInt(expected), broadPhaseBoolInt(actual))
				return 1
			}
		}
	}

	return 0
}

// broadPhaseBoolInt prints a bool as C's %d does. PORT: Go-only helper.
func broadPhaseBoolInt(b bool) int {
	if b {
		return 1
	}
	return 0
}

// broadPhaseRandomState is RandomState.
type broadPhaseRandomState struct {
	seed uint32
}

// broadPhaseNextRandom is NextRandom.
func broadPhaseNextRandom(state *broadPhaseRandomState) uint32 {
	// xorshift32
	x := state.seed
	x ^= x << 13
	x ^= x >> 17
	x ^= x << 5
	state.seed = x
	return x
}

// broadPhaseRandomFloat is RandomFloat.
func broadPhaseRandomFloat(state *broadPhaseRandomState, lower, upper float32) float32 {
	u := float32(broadPhaseNextRandom(state)&0xFFFFFF) / float32(0x1000000)
	return lower + float32((upper-lower)*u)
}

// broadPhaseRandomInt is RandomInt.
func broadPhaseRandomInt(state *broadPhaseRandomState, count int) int {
	return int(broadPhaseNextRandom(state) % uint32(count))
}

const (
	broadPhaseMaxBodies   = 64 // MAX_BODIES
	broadPhaseStaticCount = 60 // STATIC_COUNT
	broadPhaseTileCount   = 9  // TILE_COUNT
)

// broadPhaseBodySlot is BodySlot.
type broadPhaseBodySlot struct {
	bodyId BodyID
	active bool
}

// broadPhaseStressContext is StressContext.
type broadPhaseStressContext struct {
	worldId    WorldID
	random     broadPhaseRandomState
	origin     Pos
	extent     float32
	slots      [broadPhaseMaxBodies]broadPhaseBodySlot
	joints     [4]JointID
	jointCount int
	box        BoxHull
}

// broadPhaseRandomPosition is RandomPosition.
func broadPhaseRandomPosition(context *broadPhaseStressContext) Pos {
	e := context.extent
	x := broadPhaseRandomFloat(&context.random, -e, e)
	y := broadPhaseRandomFloat(&context.random, -e, e)
	z := broadPhaseRandomFloat(&context.random, -e, e)
	offset := Vec3{x, y, z}
	return OffsetPos(context.origin, offset)
}

// broadPhaseRandomVelocity is RandomVelocity.
func broadPhaseRandomVelocity(context *broadPhaseStressContext) Vec3 {
	// Some bodies outrun their margin every step and go through continuous collision
	s := float32(12.0)
	if broadPhaseRandomInt(&context.random, 4) == 0 {
		s = 40.0
	}
	x := broadPhaseRandomFloat(&context.random, -s, s)
	y := broadPhaseRandomFloat(&context.random, -s, s)
	z := broadPhaseRandomFloat(&context.random, -s, s)
	return Vec3{x, y, z}
}

// broadPhaseCreateSlotBody is CreateSlotBody.
func broadPhaseCreateSlotBody(context *broadPhaseStressContext, slot int) {
	random := &context.random

	bodyDef := DefaultBodyDef()
	if broadPhaseRandomInt(random, 10) == 0 {
		bodyDef.Type = KinematicBody
	} else {
		bodyDef.Type = DynamicBody
	}
	bodyDef.Position = broadPhaseRandomPosition(context)
	bodyDef.LinearVelocity = broadPhaseRandomVelocity(context)
	bodyDef.IsBullet = broadPhaseRandomInt(random, 3) == 0
	bodyDef.GravityScale = 0.0
	bodyId := context.worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	shapeDef.Filter.CategoryBits = uint64(1) << broadPhaseRandomInt(random, 3)
	if broadPhaseRandomInt(random, 4) == 0 {
		shapeDef.Filter.MaskBits = 0x5
	} else {
		shapeDef.Filter.MaskBits = DefaultMaskBits
	}

	shapeCount := 1 + broadPhaseRandomInt(random, 2)
	for i := 0; i < shapeCount; i++ {
		if broadPhaseRandomInt(random, 2) == 0 {
			cx := broadPhaseRandomFloat(random, -1.0, 1.0)
			radius := broadPhaseRandomFloat(random, 0.2, 1.5)
			sphere := Sphere{Vec3{cx, 0.0, 0.0}, radius}
			bodyId.CreateSphereShape(&shapeDef, &sphere)
		} else {
			bodyId.CreateHullShape(&shapeDef, &context.box.Base)
		}
	}

	context.slots[slot].bodyId = bodyId
	context.slots[slot].active = true
}

// broadPhaseRunBroadPhaseStress is RunBroadPhaseStress.
// A swarm of bodies churning through every proxy mutation the world API offers, with the pair
// set checked against a brute force scan after every step.
func broadPhaseRunBroadPhaseStress(workerCount int, seed uint32, enableSleep bool) int {
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldDef.EnableSleep = enableSleep
	worldDef.Gravity = Vec3Zero
	worldId := CreateWorld(&worldDef)

	var context broadPhaseStressContext
	context.worldId = worldId
	context.random.seed = seed
	context.origin = Pos{-1900.0, 0.0, 1330.0}
	context.extent = 12.0
	context.box = MakeBoxHull(0.95, 0.6, 2.2)

	// Kilometre tiles far from the origin under the swarm, like a streamed terrain
	tile := CreateGrid(65, 65, Vec3{16.0, 1.0, 16.0}, false)
	for i := 0; i < broadPhaseTileCount; i++ {
		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = Pos{-3072.0 + 1024.0*float32(i%3), -30.0, 1024.0 * float32(i/3)}
		tileId := worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		shapeDef.EnableSpeculativeContact = false
		tileId.CreateHeightFieldShape(&shapeDef, tile)
	}

	// Static clutter inside the swarm. Some pieces ask for contacts on creation.
	var staticIds [broadPhaseStaticCount]BodyID
	grid := CreateGrid(5, 5, Vec3{2.0, 1.0, 2.0}, false)
	staticBox := MakeBoxHull(3.0, 0.5, 3.0)
	for i := 0; i < broadPhaseStaticCount; i++ {
		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyDef.Position = broadPhaseRandomPosition(&context)
		staticIds[i] = worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		shapeDef.InvokeContactCreation = (i % 4) == 0
		shapeDef.EnableSpeculativeContact = (i % 2) == 0

		if i%3 == 0 {
			staticIds[i].CreateHeightFieldShape(&shapeDef, grid)
		} else {
			staticIds[i].CreateHullShape(&shapeDef, &staticBox.Base)
		}
	}

	for i := 0; i < broadPhaseMaxBodies; i++ {
		if broadPhaseRandomInt(&context.random, 4) != 0 {
			broadPhaseCreateSlotBody(&context, i)
		}
	}

	timeStep := float32(1.0 / 60.0)
	stepCount := 400

	for step := 0; step < stepCount; step++ {
		random := &context.random

		mutationCount := broadPhaseRandomInt(random, 4)
		for m := 0; m < mutationCount; m++ {
			slot := broadPhaseRandomInt(random, broadPhaseMaxBodies)
			body := &context.slots[slot]
			action := broadPhaseRandomInt(random, 12)

			if body.active == false {
				if action < 6 {
					broadPhaseCreateSlotBody(&context, slot)
				}
				continue
			}

			switch action {
			case 0:
				body.bodyId.Destroy()
				body.active = false

			case 1, 2:
				body.bodyId.SetTransform(broadPhaseRandomPosition(&context), QuatIdentity)

			case 3:
				body.bodyId.SetLinearVelocity(broadPhaseRandomVelocity(&context))

			case 4:
				body.bodyId.Disable()

			case 5:
				body.bodyId.Enable()

			case 6:
				body.bodyId.SetBullet(broadPhaseRandomInt(random, 2) == 0)

			case 7:
				types := [3]BodyType{StaticBody, KinematicBody, DynamicBody}
				body.bodyId.SetType(types[broadPhaseRandomInt(random, 3)])
				if body.bodyId.GetType() != StaticBody {
					body.bodyId.SetLinearVelocity(broadPhaseRandomVelocity(&context))
				}

			case 8:
				// Add a joint that blocks collision, or toggle one
				if context.jointCount < 4 {
					otherSlot := broadPhaseRandomInt(random, broadPhaseMaxBodies)
					other := &context.slots[otherSlot]
					if otherSlot != slot && other.active && body.bodyId.GetType() == DynamicBody &&
						other.bodyId.GetType() == DynamicBody {
						jointDef := DefaultWeldJointDef()
						jointDef.Base.BodyIdA = body.bodyId
						jointDef.Base.BodyIdB = other.bodyId
						jointDef.Base.CollideConnected = false
						context.joints[context.jointCount] = worldId.CreateWeldJoint(&jointDef)
						context.jointCount++
					}
				} else {
					jointId := context.joints[broadPhaseRandomInt(random, context.jointCount)]
					if jointId.IsValid() {
						jointId.SetCollideConnected(broadPhaseRandomInt(random, 2) == 0)
					}
				}

			case 9:
				staticIndex := broadPhaseRandomInt(random, broadPhaseStaticCount)
				staticIds[staticIndex].SetTransform(broadPhaseRandomPosition(&context), QuatIdentity)

			case 10:
				worldId.RebuildStaticTree()

			default:
			}
		}

		// Joints whose bodies were destroyed are gone too
		for j := 0; j < context.jointCount; {
			if context.joints[j].IsValid() == false {
				context.joints[j] = context.joints[context.jointCount-1]
				context.jointCount -= 1
			} else {
				j += 1
			}
		}

		// Keep the swarm inside the clutter
		for i := 0; i < broadPhaseMaxBodies; i++ {
			body := &context.slots[i]
			if body.active == false || body.bodyId.IsEnabled() == false {
				continue
			}

			p := body.bodyId.GetPosition()
			d := SubPos(p, context.origin)
			if AbsFloat(d.X) > 2.0*context.extent || AbsFloat(d.Y) > 2.0*context.extent ||
				AbsFloat(d.Z) > 2.0*context.extent {
				body.bodyId.SetTransform(broadPhaseRandomPosition(&context), QuatIdentity)
				body.bodyId.SetLinearVelocity(broadPhaseRandomVelocity(&context))
			}
		}

		worldId.Step(timeStep, 4)

		// A zero step finds pairs and purges disjoint contacts without moving anything
		worldId.Step(0.0, 1)

		if broadPhaseCheckPairSet(worldId) != 0 {
			GinkgoWriter.Printf("  workers=%d seed=%d sleep=%d step=%d\n", workerCount, seed, broadPhaseBoolInt(enableSleep), step)
			worldId.Destroy()
			DestroyHeightField(grid)
			DestroyHeightField(tile)
			return 1
		}
	}

	worldId.Destroy()
	DestroyHeightField(grid)
	DestroyHeightField(tile)
	return 0
}

// broadPhaseStressTest is BroadPhaseStressTest.
func broadPhaseStressTest() {
	seeds := [3]uint32{12345, 0xC0FFEE, 777}
	for i := 0; i < 3; i++ {
		ensure(broadPhaseRunBroadPhaseStress(1, seeds[i], false) == 0)
		ensure(broadPhaseRunBroadPhaseStress(4, seeds[i], false) == 0)
		ensure(broadPhaseRunBroadPhaseStress(3, seeds[i], true) == 0)
	}
}

// broadPhaseRunStaticRebuildKeepsMarks is RunStaticRebuildKeepsMarks.
// The broad-phase keeps its pending pair marks inside the static tree, so a full rebuild between
// a static mutation and the next step used to wipe them and the contact never appeared. Covers
// both ways a static proxy gets marked: a teleport and a shape created with invokeContactCreation.
func broadPhaseRunStaticRebuildKeepsMarks(useInvoke bool) int {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec3Zero
	worldId := CreateWorld(&worldDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{0.0, 1.0, 0.0}
	sphereId := worldId.CreateBody(&bodyDef)
	shapeDef := DefaultShapeDef()
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
	sphereId.CreateSphereShape(&shapeDef, &sphere)

	bodyDef.Type = StaticBody
	bodyDef.Position = Pos{100.0, 0.0, 0.0}
	groundId := worldId.CreateBody(&bodyDef)
	box := MakeBoxHull(2.0, 0.5, 2.0)
	if useInvoke == false {
		groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	// Settle so the resting sphere carries no mark of its own
	for i := 0; i < 3; i++ {
		worldId.Step(1.0/60.0, 4)
	}

	groundId.SetTransform(PosZero, QuatIdentity)
	if useInvoke {
		invokeDef := DefaultShapeDef()
		invokeDef.InvokeContactCreation = true
		groundId.CreateHullShape(&invokeDef, &box.Base)
	}

	worldId.RebuildStaticTree()
	worldId.Step(1.0/60.0, 4)

	ensure(sphereId.GetContactCapacity() == 1)

	worldId.Destroy()
	return 0
}

var _ = Describe("BroadPhaseTest", func() {
	It("StaticRebuildKeepsMarks", func() {
		ensure(broadPhaseRunStaticRebuildKeepsMarks(false) == 0)
		ensure(broadPhaseRunStaticRebuildKeepsMarks(true) == 0)
	})

	It("BroadPhaseStressTest", func() {
		broadPhaseStressTest()
	})
})
