// Port of src/body.h (and later src/body.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- body.h ----

// PORT: header types only; body.c lands in milestone 3. The static inline helpers
// (b3EncodeBodySimIndex, b3IsStaticSimIndex, b3DecodeAwakeIndex, b3SleepBodySimIndex,
// b3MakeRelativeSweep) land with body.c too. The four body.c functions the broad-phase calls are
// ported early in the body.c section below.

// b3BodyFlags. PORT: untyped constants so they mix with the uint32 flag fields.
const (
	// This body has fixed translation along the x-axis
	lockLinearX = 0x00000001

	// This body has fixed translation along the y-axis
	lockLinearY = 0x00000002

	// This body has fixed translation along the z-axis
	lockLinearZ = 0x00000004

	// This body has fixed rotation around the x-axis
	lockAngularX = 0x00000008

	// This body has fixed rotation around the y-axis
	lockAngularY = 0x00000010

	// This body has fixed rotation around the z-axis
	lockAngularZ = 0x00000020

	// This flag is used for debug draw and contact recycling. Only lives on b3BodySim.
	isFast = 0x00000040

	// This dynamic body does a final CCD pass against all body types, but not other bullets
	isBullet = 0x00000080

	// This body was speed capped in the current time step
	isSpeedCapped = 0x00000100

	// This body had a time of impact event in the current time step (body sim only)
	hadTimeOfImpact = 0x00000200

	// This body has no limit on angular velocity
	allowFastRotation = 0x00000400

	// This bullet body needs to have its AABB increased. Needed because bullets don't follow
	// the standard broad-phase update.
	enlargeBulletBounds = 0x00000800

	// This body is dynamic so the solver should write to it.
	// This prevents writing to kinematic bodies that causes a multithreaded sharing
	// cache coherence problem even when the values are not changing.
	// Used for b3BodyState flags.
	dynamicFlag = 0x00001000

	enableSleep = 0x00002000

	enableContactRecycling = 0x00004000

	// The user deferred mass computation via the updateBodyMass shape option and mass
	// data still hasn't been set.
	dirtyMass = 0x00008000

	enableGyroscopicTorque = 0x00010000

	// All lock flags
	allLocks = lockLinearX | lockLinearY | lockLinearZ | lockAngularX | lockAngularY | lockAngularZ

	// If all these flags are set then the body has fixed rotation
	fixedRotation = lockAngularX | lockAngularY | lockAngularZ

	// These flags are transient per time step. These may be different across b3Body, b3BodySim, and b3BodyState.
	bodyTransientFlags = isSpeedCapped | hadTimeOfImpact
)

// body is b3Body: body organizational details that are not used in the solver.
type body struct {
	userData any

	// index of solver set stored in b3World
	// may be B3_NULL_INDEX
	setIndex int32

	// body sim and state index within set
	// may be B3_NULL_INDEX
	localIndex int32

	// [31 : contactId | 1 : edgeIndex]
	headContactKey int32
	contactCount   int32

	// todo maybe move this to the body sim
	headShapeId int32
	shapeCount  int32

	headChainId int32

	// [31 : jointId | 1 : edgeIndex]
	headJointKey int32
	jointCount   int32

	// All enabled dynamic and kinematic bodies are in an island.
	islandId int32

	// Index into the island's bodies array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex int32

	sleepThreshold float32
	sleepTime      float32
	sleepVelocity  float32
	safetyFactor   float32
	mass           float32

	// this is used to adjust the fellAsleep flag in the body move array
	bodyMoveIndex int32

	id int32

	// b3BodyFlags
	flags  uint32
	nameId uint32

	typ BodyType // PORT: type is a Go keyword

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b3BodyId
	generation uint16
}

// bodyState is b3BodyState.
// The body state is designed for fast conversion to and from SIMD via scatter-gather.
// Only awake dynamic and kinematic bodies have a body state.
// This is used in the performance critical constraint solver
//
// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
// workers, even if the values don't change.
// This causes some trouble when computing anchors. I rotate joint anchors using the body rotation every sub-step. For static
// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
// should be minimized.
//
// Solution 1:
// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
// identity using a dummy state. Base separation and angles need to be computed. Manifolds will be behind a frame, but that
// is probably best if bodies move fast.
//
// Solution 2:
// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
// space. Potentially confusing and bug prone.
//
// Note:
// I rotate joint anchors each sub-step but not contact anchors. Joint stability improves a lot by rotating joint anchors
// according to substep progress. Contacts have reduced stability when anchors are rotated during substeps, especially for
// round shapes.
//
// 64 bytes
type bodyState struct {
	linearVelocity Vec3 // 12
	padding1       float32

	angularVelocity Vec3 // 12
	padding2        float32

	// Using delta position reduces round-off error far from the origin
	deltaPosition Vec3 // 12

	// b3BodyFlags
	// Important flags: locking, dynamic
	flags uint32 // 4

	// Using delta rotation because I cannot access the full rotation on static bodies in
	// the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
	deltaRotation Quat // 16
}

// identityBodyState is b3_identityBodyState: identity body state, notice the deltaRotation is identity
var identityBodyState = bodyState{
	Vec3{0.0, 0.0, 0.0}, 0.0, Vec3{0.0, 0.0, 0.0}, 0.0, Vec3{0.0, 0.0, 0.0}, 0, Quat{Vec3{0.0, 0.0, 0.0}, 1.0},
}

// bodySim is b3BodySim.
// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
type bodySim struct {
	// transform for body origin, double translation in large world mode
	transform WorldTransform

	// center of mass position in world space
	center Pos

	// b3BodyFlags
	flags uint32

	minExtent float32
	maxExtent Vec3

	// previous rotation and COM for TOI
	rotation0 Quat
	center0   Pos

	// location of center of mass relative to the body origin
	localCenter Vec3

	force  Vec3
	torque Vec3

	invMass float32

	// Local space inertia
	inertia Matrix3

	// Rotational inertia about the center of mass. The world space inverse inertia tensor
	// must be updated whenever the body rotation is modified.
	invInertiaLocal Matrix3
	invInertiaWorld Matrix3

	linearDamping  float32
	angularDamping float32
	gravityScale   float32

	// Index of b3Body
	bodyId int32
}

// ---- body.c ----

// PORT: only b3GetBodyTransformQuick, b3GetBodyTransform, b3GetBodySim and b3ShouldBodiesCollide
// are ported so far (broad_phase.c needs them in milestone 2). The rest of body.c lands in
// milestone 3 around them, in upstream order.

// getBodyTransformQuick is b3GetBodyTransformQuick.
func getBodyTransformQuick(world *world, body *body) WorldTransform {
	set := arrayGet(world.solverSets, int(body.setIndex))
	bodySim := arrayGet(set.bodySims, int(body.localIndex))
	return bodySim.transform
}

// getBodyTransform is b3GetBodyTransform.
func getBodyTransform(world *world, bodyId int) WorldTransform {
	body := arrayGet(world.bodies, bodyId)
	return getBodyTransformQuick(world, body)
}

// getBodySim is b3GetBodySim.
func getBodySim(world *world, body *body) *bodySim {
	set := arrayGet(world.solverSets, int(body.setIndex))
	bodySim := arrayGet(set.bodySims, int(body.localIndex))
	return bodySim
}

// shouldBodiesCollide is b3ShouldBodiesCollide.
func shouldBodiesCollide(world *world, bodyA, bodyB *body) bool {
	if bodyA.typ != DynamicBody && bodyB.typ != DynamicBody {
		return false
	}

	var jointKey int32
	var otherBodyId int32
	if bodyA.jointCount < bodyB.jointCount {
		jointKey = bodyA.headJointKey
		otherBodyId = bodyB.id
	} else {
		jointKey = bodyB.headJointKey
		otherBodyId = bodyA.id
	}

	for jointKey != NullIndex {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		otherEdgeIndex := edgeIndex ^ 1

		joint := arrayGet(world.joints, int(jointId))
		if joint.collideConnected == false && joint.edges[otherEdgeIndex].bodyId == otherBodyId {
			return false
		}

		jointKey = joint.edges[edgeIndex].nextKey
	}

	return true
}
