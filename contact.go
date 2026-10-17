// Port of src/contact.h (and later src/contact.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- contact.h ----

// PORT: header types only; contact.c lands in milestone 3.

// forceGhostCollisions is B3_FORCE_GHOST_COLLISIONS.
const forceGhostCollisions = false

// contactCache is b3ContactCache.
// PORT: union { satCache; simplexCache }. Both members are present; the shape pair decides which
// one is used.
type contactCache struct {
	satCache     SATCache
	simplexCache SimplexCache
}

// triangleCache is b3TriangleCache.
type triangleCache struct {
	triangleIndex int32
	cache         contactCache
}

// b3ContactFlags. PORT: untyped constants so they mix with the uint32 flag fields.
const (
	// Set when the solid shapes are touching.
	contactTouchingFlag = 0x00000001

	// Contact has a hit event
	contactHitEventFlag = 0x00000002

	// This contact wants contact events
	contactEnableContactEvents = 0x00000004

	// This is contact is between a dynamic and static body
	contactStaticFlag = 0x00000008

	contactRecycleFlag = 0x00000010

	// Set when the shapes are touching
	simTouchingFlag = 0x00010000

	// This contact no longer has overlapping AABBs
	simDisjoint = 0x00020000

	// This contact started touching
	simStartedTouching = 0x00040000

	// This contact stopped touching
	simStoppedTouching = 0x00080000

	// This contact has a hit event
	simEnableHitEvent = 0x00100000

	// This contact wants pre-solve events
	simEnablePreSolveEvents = 0x00200000

	// This is a mesh contact
	simMeshContact = 0x00400000

	// Relative transform is cached for contact recycling
	relativeTransformValid = 0x00800000

	// Enable speculative contact points
	enableSpeculativePoints = 0x01000000
)

// contactEdge is b3ContactEdge.
// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
type contactEdge struct {
	bodyId  int32
	prevKey int32
	nextKey int32
}

// meshContact is b3MeshContact.
type meshContact struct {
	triangleCache []triangleCache
	queryBounds   AABB
}

// convexContact is b3ConvexContact.
type convexContact struct {
	cache contactCache
}

// contact is b3Contact: represents the persistent interaction between two shapes
type contact struct {
	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	setIndex int32

	// index into the constraint graph color array
	// B3_NULL_INDEX for non-touching or sleeping contacts
	// B3_NULL_INDEX when slot is free
	colorIndex int32

	// contact index within set or graph color
	// B3_NULL_INDEX when slot is free
	localIndex int32

	edges      [2]contactEdge
	shapeIdA   int32
	shapeIdB   int32
	childIndex int32

	// A contact only belongs to an island if touching, otherwise B3_NULL_INDEX.
	islandId int32

	// Index into the island's contacts array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex int32

	// Back index into b3World::contacts
	contactId int32

	// Encoded body sim indices to avoid a cache miss accessing the body in the narrow phase.
	encodedBodySimA int32
	encodedBodySimB int32

	// b3ContactFlags
	flags uint32

	// todo embed single manifold with ability to allocate overfloat, use union.
	// PORT: b3Manifold* is a slice from the manifold allocator; manifoldCount is kept.
	manifolds     []Manifold
	manifoldCount int32

	// Cache for contact recycling.
	cachedRotationA    Quat
	cachedRotationB    Quat
	cachedRelativePose Transform

	// Mixed friction and restitution
	friction float32

	// Usage determined by b3_simMeshContact in simFlags
	// PORT: union { convexContact; meshContact }. Both members are present.
	convexContact convexContact
	meshContact   meshContact

	restitution       float32
	rollingResistance float32
	tangentVelocity   Vec3

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b3ContactId
	generation uint32
}

// contactSpec is b3ContactSpec.
type contactSpec struct {
	contactId int32

	// Start of the global manifold constraint array
	manifoldStart int32
	manifoldCount uint16
}

// ---- contact.c ----

// createContact is b3CreateContact.
// PORT: placeholder until contact.c lands in milestone 3 (it needs the contact registers and the
// collide functions of contact.c); it does nothing. broad_phase.go calls it as upstream does. No
// world exists in milestone 2, so only broad_phase_golden_test.go reaches it; that test emulates
// the one effect the broad-phase depends on (adding the pair key to the pair set).
func createContact(world *world, shapeA, shapeB *shape, childIndex int) {
}

// PORT: stubs until contact.c lands in milestone 3. Other milestone 3 files (body.c, shape.c and
// physics_world.c) call these; the owner of contact.c replaces each stub with the real function,
// keeping the signature, in upstream order.

// initializeContactRegisters is b3InitializeContactRegisters.
func initializeContactRegisters() {
	panic("box3d: b3InitializeContactRegisters not ported yet (contact.c, milestone 3)")
}

// destroyContact is b3DestroyContact.
func destroyContact(world *world, contact *contact, wakeBodies bool) {
	panic("box3d: b3DestroyContact not ported yet (contact.c, milestone 3)")
}

// updateContact is b3UpdateContact.
func updateContact(world *world, workerIndex int, contact *contact, shapeA *shape, localCenterA Vec3, xfA WorldTransform,
	shapeB *shape, localCenterB Vec3, xfB WorldTransform, isFast bool, arena arena) bool {
	panic("box3d: b3UpdateContact not ported yet (contact.c, milestone 3)")
}
