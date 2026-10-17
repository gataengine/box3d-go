// Port of src/physics_world.h (and later src/physics_world.c) from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- physics_world.h ----

// PORT: header types and the inline b3AllocateManifolds / b3FreeManifolds; physics_world.c lands
// in milestone 3. Until then the physics_world.c section at the end holds stubs for the functions
// other milestone 3 files call.

const (
	debugPointCapacity = 64 // B3_DEBUG_POINT_CAPACITY
	debugLineCapacity  = 64 // B3_DEBUG_LINE_CAPACITY
)

// b3SetType
const (
	staticSet        = 0
	disabledSet      = 1
	awakeSet         = 2
	firstSleepingSet = 3
)

// debugPoint is b3DebugPoint.
type debugPoint struct {
	p     Pos
	label int32
	value float32
	color HexColor
}

// debugLine is b3DebugLine.
type debugLine struct {
	p1, p2 Pos
	label  int32
	color  HexColor
}

// taskContext is b3TaskContext: per thread task storage
type taskContext struct {
	arena arena

	// Collect per thread sensor continuous hit events.
	sensorHits []sensorHit

	// Broad-phase pairs.
	pairKeys []uint64

	// These bits align with the b3ConstraintGraph::contactBlocks and signal a change in contact status
	contactStateBitSet bitSet

	// These bits align with the joint id capacity and signal a change in contact status
	jointStateBitSet bitSet

	// These bits align with the contact id capacity and signal a hit event.
	hitEventBitSet bitSet

	// Fast-path flag: true when this worker set at least one bit in hitEventBitSet this step.
	hasHitEvents bool

	// Used to put islands to sleep
	awakeIslandBitSet bitSet

	// Per worker split island candidate
	splitSleepTime float32
	splitIslandId  int32

	// Profiling
	satCallCount       int32
	satCacheHitCount   int32
	distanceIterations int32
	pushBackIterations int32
	rootIterations     int32

	// Number of contacts recycled this step (collide pass).
	recycledContactCount int32

	points     [debugPointCapacity]debugPoint
	pointCount int32

	lines     [debugLineCapacity]debugLine
	lineCount int32

	manifoldCounts [ContactManifoldCountBuckets]int32
	// Prevent false sharing
	cacheLine [64]byte
}

// world is b3World.
// The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
// The world also contains efficient memory management facilities.
type world struct {
	stack           stack
	broadPhase      broadPhase
	constraintGraph constraintGraph

	// Manifold allocators have one allocator for each manifold count.
	// PORT: allocator i hands out []Manifold of i + 1 manifolds (see blockAllocator).
	manifoldAllocators     []blockAllocator[Manifold]
	manifoldAllocatorMutex *mutex

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b3Body.
	bodyIdPool idPool

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
	bodies []body

	// Provides free list for solver sets.
	solverSetIdPool idPool

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
	solverSets []solverSet

	// Used to create stable ids for joints
	jointIdPool idPool

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
	joints []joint

	// Used to create stable ids for contacts
	contactIdPool idPool

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
	contacts []contact

	// Used to create stable ids for islands
	islandIdPool idPool

	// This is a sparse array that maps island ids to the island data stored in the solver sets.
	islands []island

	shapeIdPool idPool

	// These are sparse arrays that point into the pools above
	shapes []shape

	// This follows the shape array and is split out for the hot path in the narrow phase.
	fatAABBs []AABB

	// Compound shapes are static only, so only the broad-phase static pass can meet one. This
	// lets a world without compounds skip the shape type read in the pair batch.
	compoundShapeCount int32

	// Reference counted store of shared hull data keyed by content. Shapes hold a
	// pointer to the hull stored in the db. Type erased to avoid leaking the verstable map
	// type into this header.
	// PORT: typed as the hull map (Go has no header leak to avoid).
	hullDatabase *hullMap

	// Name cache for shape and body names. This works with recording.
	names nameCache

	// This is a dense array of sensor data.
	sensors []sensor

	// Per thread storage
	taskContexts       []taskContext
	sensorTaskContexts []sensorTaskContext

	bodyMoveEvents     []BodyMoveEvent
	sensorBeginEvents  []SensorBeginTouchEvent
	contactBeginEvents []ContactBeginTouchEvent

	// End events are double buffered so that the user doesn't need to flush events
	sensorEndEvents    [2][]SensorEndTouchEvent
	contactEndEvents   [2][]ContactEndTouchEvent
	endEventArrayIndex int32

	contactHitEvents []ContactHitEvent
	jointEvents      []JointEvent

	// Used to track debug draw
	debugBodySet          bitSet
	debugJointSet         bitSet
	debugContactSet       bitSet
	debugIslandSet        bitSet
	createDebugShape      CreateDebugShapeCallback
	destroyDebugShape     DestroyDebugShapeCallback
	userDebugShapeContext any

	// Id that is incremented every time step
	stepIndex uint64

	// Identify islands for splitting as follows:
	// - I want to split islands so smaller islands can sleep
	// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
	//   if it has removed constraints
	// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
	// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
	// - if no bodies want to sleep then there is no reason to perform island splitting
	splitIslandId int32

	gravity                      Vec3
	hitEventThreshold            float32
	restitutionThreshold         float32
	restitutionIterations        int32
	enableRestitutionPropagation bool
	maxLinearSpeed               float32
	isotropicInertiaTolerance    float32
	contactSpeed                 float32
	contactHertz                 float32
	contactDampingRatio          float32
	contactRecycleDistance       float32

	frictionCallback    FrictionCallback
	restitutionCallback RestitutionCallback

	generation uint16

	profile          Profile
	satCallCount     int32
	satCacheHitCount int32
	manifoldCounts   [ContactManifoldCountBuckets]int32

	maxCapacity Capacity

	preSolveFcn     PreSolveFcn
	preSolveContext any

	customFilterFcn     CustomFilterFcn
	customFilterContext any

	workerCount     int32
	simdWidth       int32
	enqueueTaskFcn  EnqueueTaskCallback
	finishTaskFcn   FinishTaskCallback
	userTaskContext any
	userTreeTask    any

	scheduler *scheduler

	userData any

	// Non-NULL while a recording session is active. Set by b3World_StartRecording,
	// cleared by b3World_StopRecording. Hooks in mutators check this before writing.
	recording *Recording

	// latest inverse sub-step
	inv_h float32

	// latest inverse full-step
	inv_dt float32

	activeTaskCount int32
	taskCount       int32

	worldId uint16

	enableSleep bool

	// This indicates there is a world write operation in progress. This is for debugging and
	// not a real mutex. This should have minimal performance impact.
	locked             bool
	enableWarmStarting bool
	enableContinuous   bool
	enableSpeculative  bool
	inUse              bool
}

// PORT: b3GetUnlockedWorldFromId, b3GetWorldFromId, b3GetUnlockedWorld, b3GetWorld,
// b3World_RebuildStaticTree, b3ValidateConnectivity, b3ValidateSolverSets, b3ValidateContacts,
// b3AddHullToDatabase, b3AddOwnedHullToDatabase and b3RemoveHullFromDatabase are declared here and
// defined in physics_world.c (below).

// allocateManifolds is b3AllocateManifolds.
// PORT: returns a slice of count manifolds (nil for zero) from allocator count - 1.
func allocateManifolds(world *world, count int) []Manifold {
	if count == 0 {
		return nil
	}

	index := count - 1

	// Need lock because this is called from the parallel narrow phase
	lockMutex(world.manifoldAllocatorMutex)
	currentCount := len(world.manifoldAllocators)
	for i := currentCount; i < count; i++ {
		allocator := createBlockAllocator[Manifold](i+1, 2*blockSize)
		arrayPush(&world.manifoldAllocators, allocator)
	}

	allocator := arrayGet(world.manifoldAllocators, index)
	manifolds := allocateElement(allocator)
	unlockMutex(world.manifoldAllocatorMutex)
	clear(manifolds[:count])
	return manifolds
}

// freeManifolds is b3FreeManifolds.
func freeManifolds(world *world, manifolds []Manifold, count int) {
	if count == 0 {
		return
	}

	index := count - 1
	lockMutex(world.manifoldAllocatorMutex)
	allocator := arrayGet(world.manifoldAllocators, index)
	freeElement(allocator, manifolds)
	unlockMutex(world.manifoldAllocatorMutex)
}

// ---- physics_world.c ----

// validateSolverSets is b3ValidateSolverSets.
// PORT: placeholder until physics_world.c lands in milestone 3. Upstream's body is assertions
// only (connectivity of bodies, contacts, joints and islands across the solver sets); a world
// cannot exist in milestone 2, so the placeholder checks nothing.
func validateSolverSets(world *world) {
}

// PORT: stubs until physics_world.c lands in milestone 3. Other milestone 3 files (body.c, shape.c,
// contact.c) call these; the owner of physics_world.c replaces each stub with the real function,
// keeping the signature, in upstream order.

// getUnlockedWorldFromId is b3GetUnlockedWorldFromId.
func getUnlockedWorldFromId(id WorldID) *world {
	panic("box3d: b3GetUnlockedWorldFromId not ported yet (physics_world.c, milestone 3)")
}

// getUnlockedWorld is b3GetUnlockedWorld.
func getUnlockedWorld(index int) *world {
	panic("box3d: b3GetUnlockedWorld not ported yet (physics_world.c, milestone 3)")
}

// getWorld is b3GetWorld.
func getWorld(index int) *world {
	panic("box3d: b3GetWorld not ported yet (physics_world.c, milestone 3)")
}

// IsValid is b3Body_IsValid.
func (id BodyID) IsValid() bool {
	panic("box3d: b3Body_IsValid not ported yet (physics_world.c, milestone 3)")
}

// validateConnectivity is b3ValidateConnectivity.
func validateConnectivity(world *world) {
	panic("box3d: b3ValidateConnectivity not ported yet (physics_world.c, milestone 3)")
}

// addHullToDatabase is b3AddHullToDatabase.
// Register a hull in the world database, returning the owned shared copy. Identical hulls
// share one copy with a reference count. The input may be freed after this call.
func addHullToDatabase(world *world, src *HullData) *HullData {
	panic("box3d: b3AddHullToDatabase not ported yet (physics_world.c, milestone 3)")
}

// addOwnedHullToDatabase is b3AddOwnedHullToDatabase.
// Like b3AddHullToDatabase but takes ownership of a heap hull: inserted directly on a miss,
// freed on a hit. Avoids cloning data the caller already allocated.
func addOwnedHullToDatabase(world *world, owned *HullData) *HullData {
	panic("box3d: b3AddOwnedHullToDatabase not ported yet (physics_world.c, milestone 3)")
}

// removeHullFromDatabase is b3RemoveHullFromDatabase.
// Release a reference to a shared hull. The owned copy is freed when the count reaches zero.
func removeHullFromDatabase(world *world, data *HullData) {
	panic("box3d: b3RemoveHullFromDatabase not ported yet (physics_world.c, milestone 3)")
}
