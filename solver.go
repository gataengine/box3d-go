// Port of src/solver.h (and later src/solver.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- solver.h ----

// Solver work is partitioned into fixed-size blocks that worker threads claim
// in parallel via atomic CAS on a per-block syncIndex. The descriptor (b3SolverBlock)
// and the atomic counter sit in a wrapping b3SyncBlock so the CAS-winner can
// pass the descriptor by value into stage tasks without aliasing the atomic
// memory other threads are CAS-writing. Three properties of this design
// matter for performance:
//
// 1. Distributed contention. Per-block atomic syncIndex avoids the cache line stampede
//    that a single shared fetch_add counter would cause. Once a worker
//    settles into a block range, its CAS targets live in its own L1.
//
// 2. Monotonic syncIndex across iterations. Iterative stages (warm start,
//    solve, relax) reuse the same block array every sub-step iteration.
//    syncIndex grows each iteration; workers CAS (prev, prev+1), so the
//    main thread never touches any per-block state between iterations.
//    Non-iterative stages simply use syncIndex 1.
//
// 3. L2 affinity across iterations. Each worker picks a start offset from
//    its workerIndex, then scans forward and (after wrap) backward:
//
//      blocks:   [0] [1] [2] [3] [4] [5] [6] [7]
//                 ^           ^           ^   ^
//                 W0          W1          W2  W3   <- start offsets
//
//    W0 claims 0,1,2,3 (forward), W1 claims 4,5, etc. Under balanced load
//    each worker re-hits the same block range every iteration, keeping that
//    range's hot data resident in its L2. A failed CAS means a neighbour
//    already claimed the block, so the stealing worker stops -- preserving
//    locality under mild imbalance while still draining the queue.
//
// A graph color stage lays out joint blocks first, then contact blocks:
//
//      stage->blocks ->
//        +------+------+------+------+------+------+------+
//        |  J0  |  J1  |  J2  |  C0  |  C1  |  C2  |  C3  |
//        +------+------+------+------+------+------+------+
//        <-- graphJointBlocks --><---- graphContactBlocks ---->
//
// Each block carries its type so the dispatcher routes J-blocks to the joint
// solver and C-blocks to the SIMD contact solver; both kinds run concurrently
// within the stage -- no barrier between them. The type tag lives on the
// block (not the stage) so that mixed-type stages can keep the concurrency.
//
// The solver threading model is inspired by https://github.com/bepu/bepuphysics2

// solverStageType is b3SolverStageType: solver stages
type solverStageType int32

const (
	stagePrepareJoints solverStageType = iota
	stagePrepareWideContacts
	stagePrepareContacts
	stageIntegrateVelocities
	stageWarmStart
	stageSolve
	stageIntegratePositions
	stageRelax
	stageRestitution
	stageStoreWideImpulses
	stageStoreImpulses
)

// solverBlockType is b3SolverBlockType.
// PORT: untyped constants; upstream stores the value in the uint8_t solverBlock.blockType.
const (
	// Block for iterating across bodies.
	bodyBlock = iota

	// Block for iterating across joints. For prepare.
	jointBlock

	// Block for iterating across wide contacts. For prepare and store.
	wideContactBlock

	// Block for iterating across contacts. For prepare and store.
	contactBlock

	// Block for iterating across joints of a single graph color.
	graphJointBlock

	// Block for iterating across wide contacts of a single graph color.
	graphWideContactBlock

	// Block for iterating across contacts of a single graph color.
	graphContactBlock

	// Block for processing overflow constraints
	// PORT: b3_overflowBlock; renamed because overflowBlock is the arena allocator's b3OverflowBlock.
	overflowBlockType
)

// solverBlock is b3SolverBlock.
// Solver block describes a multithreaded unit of work.
type solverBlock struct {
	startIndex int32
	count      uint16
	// b3SolverBlockType
	blockType  uint8
	colorIndex uint8
}

// syncBlock is b3SyncBlock.
// A unit of multithreaded work along with atomic synchronization. The syncIndex grows
// monotonically allowing the solver block to be re-used across sub-steps.
type syncBlock struct {
	block     solverBlock
	syncIndex atomicInt
}

// solverStage is b3SolverStage.
// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
// PORT: b3SyncBlock* blocks is a slice; blockCount is kept.
type solverStage struct {
	blocks          []syncBlock
	typ             solverStageType
	blockCount      int32
	colorIndex      uint8
	completionCount atomicInt
}

// softness is b3Softness: constraint softness
type softness struct {
	biasRate     float32
	massScale    float32
	impulseScale float32
}

// widePrepareSpan is b3WidePrepareSpan.
// Prepare/store run as a flat parallel-for over the whole wide-constraint
// range. Each span maps a slice of that range back to the owning color's
// contacts so workers can decode flat wide-slot indices without touching
// graph state. The spans array has one entry per active color plus a sentinel
// whose start == wideContactCount.
// PORT: int* contacts is a slice.
type widePrepareSpan struct {
	start    int32
	count    int32
	contacts []int32
}

// contactPrepareSpan is b3ContactPrepareSpan.
// PORT: b3ContactSpec* contacts is a slice.
type contactPrepareSpan struct {
	start    int32
	count    int32
	contacts []contactSpec
}

// meshPrepareSpan is b3MeshPrepareSpan.
// PORT: the pointers are slices.
type meshPrepareSpan struct {
	start    int32
	count    int32
	contacts []contactSpec
	order    []int32
}

// jointPrepareSpan is b3JointPrepareSpan.
// PORT: b3JointSim* joints is a slice.
type jointPrepareSpan struct {
	start  int32
	count  int32
	joints []jointSim
}

// stepContext is b3StepContext.
// Context for a time step. Recreated each time step.
// PORT: pointer + count pairs are slices with the count fields kept; the void* wide constraint
// buffers are wideConstraintBuffer (contact_solver_wide.go); the false-sharing padding is kept.
type stepContext struct {
	// time step
	dt float32

	// inverse time step (0 if dt == 0).
	inv_dt float32

	// sub-step
	h     float32
	inv_h float32

	subStepCount int32

	contactSoftness softness
	staticSoftness  softness

	maxLinearVelocity float32

	world *world
	graph *constraintGraph

	// shortcut to body states from awake set
	states []bodyState

	// shortcut to body sims from awake set
	sims []bodySim

	// array of all shape ids for shapes that have enlarged AABBs
	enlargedShapes     []int32
	enlargedShapeCount int32

	// Array of bullet bodies that need continuous collision handling
	bulletBodies    []int32
	bulletBodyCount atomicInt

	anyRestitution atomicInt

	// Contact ids for simplified parallel-for access. Used in narrow-phase.
	// These contacts may or may not be touching. They are associated with awake bodies.
	awakeContactIndices []int32

	// Flat view of the wide contact constraint array used by prepare and store.
	// prepareSpans has activeColorCount + 1 entries, the last being a sentinel
	// at wideContactCount. wideContactConstraints is the contiguous base
	// pointer. Per-color slices live at colors[i].wideConstraints.
	wideConstraints  wideConstraintBuffer
	widePrepareSpans []widePrepareSpan
	wideContactCount int32

	// Similar for overflow contact constraints.
	overflowSpans          []contactPrepareSpan
	wideMeshConstraints    wideConstraintBuffer
	wideMeshManifoldStarts []int32
	meshPrepareSpans       []meshPrepareSpan
	jointPrepareSpans      []jointPrepareSpan

	activeColorCount int32
	workerCount      int32

	stages             []solverStage
	stageCount         int32
	enableWarmStarting bool

	// padding to prevent false sharing
	padding1 [64]byte

	// This atomic is central to multi-threaded solver task synchronization.
	// It prevents ABA problems by monotonically growing as the solver advances.
	// This means a delayed worker thread will catch up without repeating already completed
	// work (causing a race condition).
	// sync index (16-bits) | stage type (16-bits)
	atomicSyncBits atomicU32

	// padding to prevent false sharing
	padding2 [64]byte

	// Race flag claimed by whichever runner reaches b3SolverTask with workerIndex 0 first.
	// The calling thread of b3World_Step also races for this slot so the orchestrator can
	// always make progress, regardless of how the user's task system schedules tasks (out
	// of order, fewer threads than workers, or synchronously inside enqueueTaskFcn). The
	// loser of the race no-ops as workerIndex 0.
	mainClaimed atomicInt

	// padding to prevent false sharing
	padding3 [64]byte
}

// b3Solve is declared here and defined in solver.c (below, once ported).

// makeSoft is b3MakeSoft.
func makeSoft(hertz, zeta, h float32) softness {
	if hertz == 0.0 {
		return softness{
			biasRate:     0.0,
			massScale:    0.0,
			impulseScale: 0.0,
		}
	}

	omega := float32(2.0*Pi) * hertz
	a1 := float32(2.0*zeta) + float32(h*omega)
	a2 := float32(float32(h*omega) * a1)
	a3 := 1.0 / (1.0 + a2)

	// bias = w / (2 * z + hw)
	// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
	// impulseScale = 1 / (1 + hw * (2 * z + hw))

	// If z == 0
	// bias = 1/h
	// massScale = hw^2 / (1 + hw^2)
	// impulseScale = 1 / (1 + hw^2)

	// w -> inf
	// bias = 1/h
	// massScale = 1
	// impulseScale = 0

	// if w = pi / 4  * inv_h
	// massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
	// impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

	// In all cases:
	// massScale + impulseScale == 1

	return softness{
		biasRate:     omega / a1,
		massScale:    a2 * a3,
		impulseScale: a3,
	}
}

// ---- solver.c ----

// solve is b3Solve.
// PORT: stub until solver.c lands in milestone 3. physics_world.c calls it; the owner of solver.c
// replaces this stub with the real function (same signature) when porting solver.c.
func solve(world *world, stepContext *stepContext) {
	panic("box3d: b3Solve not ported yet (solver.c, milestone 3)")
}
