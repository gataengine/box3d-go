# Box3D Go port — progress

- Upstream: [erincatto/box3d](https://github.com/erincatto/box3d) at `efca0c184d40fd6a82016983f1601c4d23965867`
  (submodule `third_party/box3d`).
- Design: `docs/superpowers/specs/2026-10-17-box3d-go-port-design.md` in the gata monorepo.
- Current milestone: **3 — Dynamics**. Milestone 2 (Collision) is complete: every M2 source file is
  ported and every M2 test file is ported; 8 specs are pending (`PIt`) because they need the world
  (test_broad_phase.c 2/2, test_mover.c 5/22, test_compound.c 1/34). Milestone 3 step 0 (prep) is
  done: every header type and static inline that the milestone 3 `.c` files need is ported
  (solver.h complete with `b3StepContext`, the wide constraint types of contact_solver_wide.inl,
  recording.h with the manifest expansions, body.h / physics_world.h / name_cache.h inlines), and
  every function one milestone 3 `.c` file calls in another one is a stub in the owner's Go file
  (see "M3 work split" below), so each `.c` file can be ported on its own branch and the package
  still builds. Functions from later milestones that milestone 3 calls are stubs (recording.c,
  M5) or placeholders (joint.c, M4; name_cache.c, M5); they are listed in `LOOSE_ENDS.md`.
  `createContact` (contact.go) and `validateSolverSets` (physics_world.go) are the older no-op
  placeholders that the real functions replace.
- Determinism status: no world goldens yet (they arrive with milestones 3 and 5). Foundation math
  matches the C build (`-ffp-contract=off`) bit for bit on 1500 golden lines
  (`math_golden_test.go`, verified on darwin/arm64 and darwin/amd64 under Rosetta). Hull
  construction (byte images via their hashes), hull/sphere/capsule queries, mover planes, polygon clipping, GJK
  distance, shape cast and TOI match the C build on 86 golden lines (`hull_golden_test.go`,
  darwin/arm64 and amd64). Every collide function of convex_manifold.c and triangle_manifold.c
  (fresh, cached and manual SAT modes) and b3ComputeSeparatingAxis match the C build on 361
  golden lines (`manifold_golden_test.go`, darwin/arm64 and amd64, with both the B3_SIMD_NONE and
  the lane SAT edge loop). The C output itself is identical for the NEON, SSE2 (x86_64),
  BOX3D_DISABLE_SIMD and BOX3D_VALIDATE builds. Mesh creation (blob byte count and hash),
  mesh triangles, AABBs, ray casts, shape casts, overlaps, mover planes and queries match the C
  build on the lines of `geometry_golden_test.go`, and so do height field creation (including a
  dump/load round trip) and compound creation (whole blob hashed), validation, children,
  triangles, casts, overlaps, mover planes and queries (darwin/arm64 and amd64; the C output is
  identical for NEON, SSE2, BOX3D_DISABLE_SIMD and BOX3D_VALIDATE). The broad-phase pair update
  (moved siblings, cross seeds, cross/self pair search with filters, joints, custom filtering
  and compound children, sorted pair creation, pair set, tree rebuilds, overlap probes) matches
  the C build over 8 mutation rounds (`broad_phase_golden_test.go`, darwin/arm64 and amd64; C
  output identical for NEON, SSE2, BOX3D_DISABLE_SIMD and BOX3D_VALIDATE), with the serial task
  system and with 4 goroutine workers. QSORT matches C call for call, including the unstable
  order of equal keys (`qsort_test.go`).
- Geometry blobs: the byte image is canonical. A blob is one 8-byte aligned `[]uint64`
  allocation (`blobAlloc` in collision.go) and `*HullData` points at its first word; the
  header structs have C layout, offsets are byte offsets from the header as upstream, and the
  accessors (`GetHullPoints`, `GetHullPlanes`, …) return slices into the image via
  `blobSlice[T]`. `BoxHull` is a value whose offsets reach into itself, as in C. `hullBytes`
  views the image for hashing/cloning/comparing. Mesh, height field and compound blobs follow
  the same pattern. Fused
  multiply-add guard (`fma_guard_test.go`) passes for arm64 and amd64 v3.

### M3 work split

Each upcoming milestone 3 `.c` file is owned by exactly one agent, which owns the Go files on its
row and edits no other Go file. Shared header-type files are frozen except by the owner of the
matching `.c`; files not listed (types.go, math, collision, base.go, container.go, core.go, the
milestone 1–2 files, recording.go, recording_ops.go, joint.go, …) are frozen for the whole
parallel phase.

| upstream `.c` (+ test) | owner's Go files | stubs it replaces (all in its own files) |
|---|---|---|
| body.c (+ test_body.c) | body.go, body_test.go | getBodyFullId, makeBodyId, getBodyState, wakeBody, wakeBodyWithLock, updateBodyMassData, syncBodyFlags, refreshBodyContactIndices |
| shape.c (+ test_shape.c) | shape.go, shape_test.go | createShapeProxy, destroyShapeProxy, destroyShapeAllocations, computeShapeMass, computeShapeExtent, computeShapeMinExtent, getShapeCentroid, getShapeProjectedArea, getShapeUserMaterialId, makeShapeProxy, rayCastShape, shapeCastShape, overlapShape, shapeTimeOfImpact, collideMover |
| contact.c | contact.go | createContact (old placeholder), initializeContactRegisters, destroyContact, updateContact |
| mesh_contact.c | mesh_contact.go | computeMeshManifolds |
| contact_solver.c + contact_solver_wide.inl | contact_solver.go, contact_solver_wide.go | the 21 contact solver entry points (overflow, convex, mesh-wide, byte counts) |
| constraint_graph.c | constraint_graph.go | createGraph, destroyGraph, addContactToGraph, removeContactFromGraph, addJointToGraph, removeJointFromGraph |
| island.c | island.go | createIsland, destroyIsland, linkContact, unlinkContact, linkJoint, unlinkJoint, splitIsland, splitIslandTask, validateIsland |
| solver.c | solver.go | solve |
| solver_set.c | solver_set.go | destroySolverSet, wakeSolverSet, trySleepIsland, transferBody, transferJoint |
| scheduler.c | scheduler.go | createScheduler, destroyScheduler, resetScheduler, schedulerEnqueueTask, schedulerFinishTask (and gives `scheduler` its fields) |
| sensor.c | sensor.go | overlapSensors, destroySensor |
| physics_world.c (+ test_world.c) | physics_world.go (optionally also physics_world_queries.go), world_test.go | validateSolverSets (old placeholder), getUnlockedWorldFromId, getUnlockedWorld, getWorld, BodyID.IsValid, validateConnectivity, addHullToDatabase, addOwnedHullToDatabase, removeHullFromDatabase |
| name_cache.c (+ test_name_cache.c), pulled forward from M5, optional | name_cache.go, name_cache_test.go | createNameCache, destroyNameCache, addName, findName (placeholders that keep no names; test_shape.c's ShapeNameTest needs the real ones) |

Rules for the parallel phase:

- Stubs are a signature contract: the owner replaces each stub in place with the real function
  and keeps the signature, because other owners already call it. A signature that has to change
  goes to the integrator, not into another owner's file. A function an owner needs from a file it
  does not own and that has no stub yet is reported, not added.
- Upstream `static` helpers stay in the owner's file. A new Go-only helper goes in the owner's
  file with a name that cannot clash (prefix it with the file's subject).
- Agents do not edit PROGRESS.md or LOOSE_ENDS.md; each returns its table row(s) and loose-end
  lines in its report and the integrator applies them, so the shared docs never conflict.
- The package builds on every branch (all cross-file calls have stubs). World-level tests only run
  once the files they reach are merged: test_body_query.c, test_large_world.c, test_restitution.c,
  test_hash.c and the M2 specs pending on the world (test_broad_phase.c, test_mover.c,
  test_compound.c) are ported by the integrator after the merge. test_world.c and
  test_restitution.c compare `b3HashWorldState`, a stub until recording.c (M5); the integrator
  either pulls that one function forward or leaves those checks pending. Joint specs (e.g.
  test_world.c's spherical joint test) stay pending until milestone 4.

Status legend: `todo`, `blocked (needs X)`, `ported` (compiles, no upstream test exists or test not
yet ported), `tested` (upstream test file ported and passing), `verified` (goldens or replay pass
through it), `n/a` (intentionally not ported; reason in notes). A header/source pair has one row
each; both point at the same Go file.

| upstream file | go file | status | ported at commit | notes |
|---|---|---|---|---|
| include/box3d/base.h | base.go | ported | efca0c1 | assert/validate via build tags; allocator API not ported |
| include/box3d/box3d.h | body.go, shape.go, joint.go, physics_world.go, … | todo | | b3Type_* functions live in the file of the object they act on |
| include/box3d/collision.h | collision.go | ported | efca0c1 | blob helpers + hull, mesh and height field accessors done; compound functions are in compound.go |
| include/box3d/config.h | — | n/a | | compile options; Go uses build tags (box3d_noassert, box3d_validate). Double precision and SIMD-disable not ported |
| include/box3d/constants.h | constants.go | ported | efca0c1 |  |
| include/box3d/id.h | id.go | tested | efca0c1 |  |
| include/box3d/math_functions.h | math_functions.go | tested | efca0c1 |  |
| include/box3d/types.h | types.go | ported | efca0c1 | blob headers have C layout (hull sizes asserted in hull.go) |
| src/aabb.c | aabb.go | ported | efca0c1 | no upstream test (exercised by height_field in M2) |
| src/aabb.h | aabb.go | ported | efca0c1 |  |
| src/algorithm.h | — | n/a | | B3_SWAP becomes Go tuple assignment at call sites (hull.go) |
| src/arena_allocator.c | arena_allocator.go | ported | efca0c1 | bookkeeping shell; typed stackAlloc[T]/bump[T] use make |
| src/arena_allocator.h | arena_allocator.go | ported | efca0c1 |  |
| src/bitset.c | bitset.go | tested | efca0c1 | countSetBits lives in table.go as upstream |
| src/bitset.h | bitset.go | tested | efca0c1 |  |
| src/block_allocator.c | block_allocator.go | tested | efca0c1 | typed blockAllocator[T]; elements are []T of elementSize values |
| src/block_allocator.h | block_allocator.go | tested | efca0c1 |  |
| src/body.c | body.go | todo | | b3GetBodyTransformQuick, b3GetBodyTransform, b3GetBodySim, b3ShouldBodiesCollide ported early (broad_phase.c needs them); the rest is M3; stubs for the functions other M3 files call (see M3 work split) |
| src/body.h | body.go | ported | efca0c1 | header types (b3BodyFlags, b3Body, b3BodyState, b3_identityBodyState, b3BodySim) and static inlines (encode/decode sim index, b3MakeRelativeSweep); encoded sim indices are `int` |
| src/broad_phase.c | broad_phase.go | verified | efca0c1 | broad_phase_golden_test.go runs the real updateBroadPhasePairs against the C build (testdata/broad_phase_golden_gen.c); b3CreateContact is a no-op placeholder whose pair-set effect the test emulates; b3Prefetch dropped |
| src/broad_phase.h | broad_phase.go | verified | efca0c1 | proxy key macros are functions; b3BroadPhase_* are methods on *broadPhase |
| src/capsule.c | capsule.go | verified | efca0c1 | mass, AABB, ray cast (incl. parallel/degenerate), mover plane, overlap in hull_golden_test.go |
| src/compound.c | compound.go | verified | efca0c1 | creation (material/hull/mesh dedup, blob bytes hashed), ValidateCompound, children, sweep, ray/shape casts, overlap, mover, queries in geometry_golden_test.go; b3CompoundTimeOfImpact is #if 0 upstream (not ported) |
| src/compound.h | compound.go | verified | efca0c1 | getCompoundTreeView, makeCompoundChildSweep, collideMoverAndCompound |
| src/constraint_graph.c | constraint_graph.go | todo | | stubs for the functions other M3 files call |
| src/constraint_graph.h | constraint_graph.go | ported | efca0c1 | header types only (b3GraphColor, b3ConstraintGraph); void* wide buffers are `wideConstraintBuffer` (contact_solver_wide.go) |
| src/contact.c | contact.go | todo | | createContact is a no-op placeholder; stubs for the functions other M3 files call |
| src/contact.h | contact.go | ported | efca0c1 | header types only; unions keep all members. contact.go also holds a no-op createContact placeholder until contact.c (M3) |
| src/contact_solver.c | contact_solver.go | todo | | stubs for the 21 entry points solver.c calls |
| src/contact_solver.h | contact_solver.go | ported | efca0c1 | header types (b3ManifoldConstraintPoint, b3ManifoldConstraint, b3ContactConstraint); declared functions are stubs until contact_solver.c |
| src/contact_solver_wide.inl | contact_solver_wide.go | todo | | wide types (b3Vec2W … b3ConvexConstraintWide) ported in M3 prep; the void* constraint stream is `wideConstraintBuffer` (parallel constraint and manifold slices); kernels todo |
| src/container.h | container.go | tested | efca0c1 | b3Array(T) is []T; generic arrayPush/arrayPop/… keep growth policy |
| src/convex_manifold.c | convex_manifold.go | verified | efca0c1 | sphere/capsule/hull collide functions, SAT cache and manual modes; test_sat, test_manifold, manifold_golden_test.go |
| src/convex_manifold_wide.inl | convex_manifold_wide.go | verified | efca0c1 | computeSeparatingAxisW; both B3_SIMD_NONE and lane edge loops ported, selected by simdNone (simd_scalar.go); both match the C goldens |
| src/core.c | core.go | ported | efca0c1 |  |
| src/core.h | core.go | ported | efca0c1 |  |
| src/ctz.h | ctz.go | tested | efca0c1 | math/bits; tested by test_bitset TestBitMath |
| src/distance.c | distance.go | verified | efca0c1 | GJK, shape cast, TOI; covered by hull_golden_test.go |
| src/distance_joint.c | distance_joint.go | todo | |  |
| src/dynamic_tree.c | dynamic_tree.go | verified | efca0c1 | insert/rotate, churn, moved marks, sweep and fallback refit, partial/full rebuild, save/load bytes, query/closest/ray/box cast order in dynamic_tree_golden_test.go (C scalar and NEON builds agree) |
| src/dynamic_tree.h | dynamic_tree.go | verified | efca0c1 | node flag helpers, treeView |
| src/height_field.c | height_field.go | verified | efca0c1 | grid/wave/custom fields (holes, clockwise, clamping, no materials), dump/load round trip, triangles, materials, AABB, ray/shape casts, overlap, mover, queries in geometry_golden_test.go (blob hashes match C) |
| src/hull.c | hull.go | verified | efca0c1 | quickhull builder, box hulls, 2D hull; hull_golden_test.go matches C bytes/hashes |
| src/hull.h | hull.go | verified | efca0c1 | hullMap = verstable[*HullData, int] |
| src/id_pool.c | id_pool.go | ported | efca0c1 | no upstream test |
| src/id_pool.h | id_pool.go | ported | efca0c1 |  |
| src/island.c | island.go | todo | | stubs for the functions other M3 files call |
| src/island.h | island.go | ported | efca0c1 | header types only |
| src/joint.c | joint.go | todo | | M4. Placeholders for what M3 calls: getJointSim, destroyJointInternal, getJointReaction, prepareJoint, warmStartJoint, solveJoint, drawJoint panic (no joint can exist yet); prepare/warmStart/solveJointsOverflow loop over the overflow joints as upstream |
| src/joint.h | joint.go | ported | efca0c1 | header types only; b3JointSim union keeps all joint members; inline b3SolvePointConstraint lands with joint.c (M4) |
| src/manifold.c | manifold.go | verified | efca0c1 | findIncidentFace, clipPolygon, flipPair checked in hull_golden_test.go |
| src/manifold.h | manifold.go | ported | efca0c1 | separating axis / clip types; b3ComputeSeparatingAxis*, b3CollideHullsAtWidth are in convex_manifold.go (W4 → computeSeparatingAxisW in convex_manifold_wide.go) |
| src/math_functions.c | math_functions.go | tested | efca0c1 |  |
| src/math_internal.h | math_internal.go | ported | efca0c1 | declared functions land with their .c files |
| src/mesh.c | mesh.go | verified | efca0c1 | creators, CreateMesh (weld, stride, SAH/median, clockwise, degenerates, edge flags), queries, ray/shape casts, overlap, mover in geometry_golden_test.go (blob hashes match C) |
| src/mesh_contact.c | mesh_contact.go | todo | | computeMeshManifolds stub |
| src/motor_joint.c | motor_joint.go | todo | |  |
| src/mover.c | mover.go | verified | efca0c1 | SolvePlanes (delta, iterations, pushes) and ClipVector in mover_golden_test.go |
| src/name_cache.c | name_cache.go | todo | | placeholders createNameCache, destroyNameCache, addName, findName keep no names; may be pulled into M3 (see M3 work split) |
| src/name_cache.h | name_cache.go | ported | efca0c1 | header types and inline b3FindNameWithDefault |
| src/parallel_for.c | parallel_for.go | tested | efca0c1 | no upstream test; parallel_for_test.go (Go-only) checks the block partition, claiming with goroutines, the serial default task system and the B3_MAX_TASKS fallback |
| src/parallel_for.h | parallel_for.go | tested | efca0c1 |  |
| src/parallel_joint.c | parallel_joint.go | todo | |  |
| src/physics_world.c | physics_world.go | todo | | validateSolverSets is a no-op placeholder; stubs for the functions other M3 files call |
| src/physics_world.h | physics_world.go | ported | efca0c1 | header types (b3SetType, b3DebugPoint/Line, b3TaskContext, b3World) and inlines b3AllocateManifolds / b3FreeManifolds (slices of count manifolds) |
| src/platform.h | platform.go | ported | efca0c1 |  |
| src/prismatic_joint.c | prismatic_joint.go | todo | |  |
| src/qsort.h | qsort.go | verified | efca0c1 | macros become functions taking LESS/SWAP closures; call counts and the unstable order of equal keys match C in qsort_test.go (testdata/qsort_golden_gen.c) |
| src/rapidhash.h | rapidhash.go | tested | efca0c1 | no upstream test; rapidhash_test.go checks C reference values |
| src/recording.c | recording.go | todo | | M5. Every recording.h function is a stub (no-op, zero result): buffer helpers, b3RecW_*, framing, b3RecWrite_* / b3RecWriteRet_* per op, query writer, trampolines, registry, interning, lifecycle, b3HashWorldState |
| src/recording.h | recording.go | ported | efca0c1 | types, constants, b3FnvMixPosition; b3Recording is the exported opaque `Recording`; arg structs and opcodes in recording_ops.go |
| src/recording_ops.inl | recording_ops.go | ported | efca0c1 | recording.h expansions written out (b3RecArgs_Name structs, b3_recOpName constants); the recording.c / replay.c expansions land in M5 |
| src/replay.c | replay.go | todo | |  |
| src/replay.h | replay.go | todo | |  |
| src/revolute_joint.c | revolute_joint.go | todo | |  |
| src/scheduler.c | scheduler.go | todo | | stubs for the functions physics_world.c calls |
| src/scheduler.h | scheduler.go | ported | efca0c1 | b3TaskCallback is the public TaskCallback; b3Scheduler is an empty forward declaration until scheduler.c (M3) defines it; declared functions are stubs until scheduler.c |
| src/sensor.c | sensor.go | todo | | stubs for the functions other M3 files call |
| src/sensor.h | sensor.go | ported | efca0c1 | header types only |
| src/shape.c | shape.go | todo | | b3ComputeShapeAABB, b3ComputeFatShapeAABB, b3MakeLocalProxy and b3ComputeProxyAABB ported early (needed by mesh/height field/compound and test_collision); the rest is M3; stubs for the functions other M3 files call |
| src/shape.h | shape.go | ported | efca0c1 | b3Shape (union members all present), b3ShapeFlags, filter/convex inlines, b3GetHeightFieldTriangleCount, b3GetMeshTriangle; declared functions land with shape.c (M3) |
| src/simd.c | simd.go | ported | efca0c1 | also holds IsAVX2Available / Get/SetSIMDWidth; Go-only simd_test.go |
| src/simd.h | simd.go | ported | efca0c1 | b3V32 non-SSE2 path as `v32`; b3Vec3W4 as `vec3W` |
| src/simd_avx2.h | — | n/a | | dropped: scalar lanes only (see spec, SIMD lane layer) |
| src/simd_neon.h | — | n/a | | dropped: scalar lanes only |
| src/simd_scalar.h | simd_scalar.go | ported | efca0c1 | `floatW{v [laneCount]float32}`, laneCount = 4; W4 suffix dropped; `simdNone` (B3_SIMD_NONE) lives here |
| src/simd_sse2.h | — | n/a | | dropped: scalar lanes only |
| src/simd_wide.h | simd_wide.go | ported | efca0c1 | name map only + wideAlignment |
| src/solver.c | solver.go | todo | | b3Solve stub |
| src/solver.h | solver.go | ported | efca0c1 | stage/block enums, b3SolverBlock, b3SyncBlock, b3SolverStage, b3Softness, prepare spans, b3StepContext (pointers are slices, void* buffers are `wideConstraintBuffer`), b3MakeSoft |
| src/solver_set.c | solver_set.go | todo | | stubs for the functions other M3 files call |
| src/solver_set.h | solver_set.go | ported | efca0c1 | header types only |
| src/sphere.c | sphere.go | verified | efca0c1 | mass, AABB, ray casts, mover plane, overlap in hull_golden_test.go |
| src/spherical_joint.c | spherical_joint.go | todo | |  |
| src/table.c | table.go | tested | efca0c1 | hashSet keeps upstream slot order |
| src/table.h | table.go | tested | efca0c1 |  |
| src/timer.c | timer.go | tested | efca0c1 | no upstream test; timer_test.go (Go-only) |
| src/triangle_manifold.c | triangle_manifold.go | verified | efca0c1 | triangle vs sphere/capsule/hull; triangle vertices are a slice; manifold_golden_test.go |
| src/types.c | types.go | ported | efca0c1 |  |
| src/verstable.h | verstable.go | ported | efca0c1 | generic `verstable[K, V]` (linear probing) with the API subset upstream uses; Go-only verstable_test.go |
| src/weld_joint.c | weld_joint.go | todo | |  |
| src/wheel_joint.c | wheel_joint.go | todo | |  |
| src/wide4.c | — | n/a | | dropped: includes the .inl kernels, ported once |
| src/wide8.c | — | n/a | | dropped: width 8 not ported |
| src/world_snapshot.c | world_snapshot.go | todo | |  |
| src/world_snapshot.h | world_snapshot.go | todo | |  |
| test/main.c | box3d_suite_test.go | ported | efca0c1 | Ginkgo suite runner |
| test/test_allocator.c | allocator_test.go | tested | efca0c1 | alignment ENSURE checks Go alignment of T |
| test/test_bitset.c | bitset_test.go | tested | efca0c1 |  |
| test/test_body.c | body_test.go | todo | |  |
| test/test_body_query.c | body_query_test.go | todo | |  |
| test/test_broad_phase.c | broad_phase_test.go | blocked (needs the M3 world files, joint.c, weld_joint.c) | efca0c1 | 0 of 2 tests; StaticRebuildKeepsMarks and BroadPhaseStressTest pending (PIt); broad_phase_golden_test.go covers the broad-phase meanwhile |
| test/test_collision.c | collision_test.go | tested | efca0c1 | 4 of 4 tests; the BOX3D_DOUBLE_PRECISION halves are compiled out as in upstream's float build |
| test/test_compound.c | compound_test.go | tested | efca0c1 | 33 of 34 tests; CompoundMeshContactMaterial pending (PIt) on the world (M3); byte-level serialize tests port as written |
| test/test_container.c | container_test.go | tested | efca0c1 |  |
| test/test_determinism.c | determinism_test.go | todo | |  |
| test/test_distance.c | distance_test.go | tested | efca0c1 |  |
| test/test_dynamic_tree.c | dynamic_tree_test.go | tested | efca0c1 |  |
| test/test_hash.c | hash_test.go | todo | |  |
| test/test_height_field.c | height_field_test.go | tested | efca0c1 | round trip file in a temp dir |
| test/test_hull.c | hull_test.go | tested | efca0c1 |  |
| test/test_id.c | id_test.go | tested | efca0c1 |  |
| test/test_joint.c | joint_test.go | todo | |  |
| test/test_large_world.c | large_world_test.go | todo | |  |
| test/test_macros.h | test_macros_test.go | ported | efca0c1 | ENSURE / ENSURE_SMALL helpers |
| test/test_manifold.c | manifold_test.go | tested | efca0c1 | exercises b3Collide* from convex_manifold.c and triangle_manifold.c |
| test/test_math.c | math_test.go | tested | efca0c1 |  |
| test/test_mesh.c | mesh_test.go | tested | efca0c1 | strided input via `fatPositions` (unsafe.Slice over the interleaved array) |
| test/test_mover.c | mover_test.go | tested | efca0c1 | 17 of 22 tests; 5 pending (PIt) on body.c, shape.c, physics_world.c (b3World_CollideMover, b3Body_CollideMover) |
| test/test_name_cache.c | name_cache_test.go | todo | |  |
| test/test_recording.c | recording_test.go | todo | |  |
| test/test_restitution.c | restitution_test.go | todo | |  |
| test/test_sat.c | sat_test.go | tested | efca0c1 | oracle stats print identical to the C build (NEON and BOX3D_DISABLE_SIMD) |
| test/test_shape.c | shape_test.go | todo | |  |
| test/test_table.c | table_test.go | tested | efca0c1 |  |
| test/test_world.c | world_test.go | todo | |  |
| shared/utils.c | utils_test.go | tested | efca0c1 | test random helpers (RandomFloat, …) |
| shared/utils.h | utils_test.go | tested | efca0c1 | test random helpers (RandomFloat, …) |
| shared/determinism.c | internal/scenes/determinism.go | todo | | test/benchmark scene helper. NOTE: an internal test (package box3d) cannot import a package that imports box3d; put scenes in _test.go files or run the determinism test as package box3d_test |
| shared/determinism.h | internal/scenes/determinism.go | todo | | test/benchmark scene helper |
| shared/human.c | internal/scenes/human.go | todo | | test/benchmark scene helper |
| shared/human.h | internal/scenes/human.go | todo | | test/benchmark scene helper |
| shared/benchmarks.c | internal/scenes/benchmarks.go | todo | | test/benchmark scene helper |
| shared/benchmarks.h | internal/scenes/benchmarks.go | todo | | test/benchmark scene helper |
