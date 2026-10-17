# Box3D Go port — progress

- Upstream: [erincatto/box3d](https://github.com/erincatto/box3d) at `efca0c184d40fd6a82016983f1601c4d23965867`
  (submodule `third_party/box3d`).
- Design: `docs/superpowers/specs/2026-10-17-box3d-go-port-design.md` in the gata monorepo.
- Status: **port complete** — milestones 1-6 are done. Milestone 6 (Benchmarks, parity audit,
  docs) is complete: shared/benchmarks.c and benchmark/main.c are ported (bench_test.go, results
  in PERFORMANCE.md), a warm `World.Step` allocates nothing at 1 and 4 workers
  (step_alloc_test.go), `tools/parity_audit.sh` reports every public upstream symbol mapped or
  intentionally omitted (mapped=1079 omitted=6 missing=0, PARITY_REPORT.md) and README.md
  documents the API. Cross-platform: the full suite (415 specs, all goldens) passes on
  darwin/arm64, windows/amd64 and linux/amd64 (also `GOAMD64=v3` and `box3d_validate`) via
  tools/xcheck, 2026-10-17. Milestone 5 (Determinism and replay)
  is complete: recording.c, replay.c, world_snapshot.c and recording_ops.inl are ported (snapshots
  are Go-native as the spec decides; C snapshot images are rejected by the layout hash before the
  world is touched), test_recording.c passes 21/21, test_name_cache.c 3/3 (NameRoundTrip and
  RollbackNames no longer pending), test_determinism.c 6/6 (SIMDWidthTest takes upstream's
  native-width-4 skip). Recording/replay parity: Go op streams and geometry registries are byte
  identical to C (`recording_golden_test.go` joint-free, `replay_joints_golden_test.go` every
  joint type and every recorded joint op); C-made recordings with and without joints replay in Go
  bit for bit (`replay_golden_test.go`, `replay_joints_golden_test.go`: per-step b3HashWorldState
  and final bodies/joints); Go-made recordings (started on an empty world and mid-life with live
  joints) replay and seek/scrub/restart through Go keyframes. All at 1, 2 and 4 workers.
  Milestone 4 (Joints) is complete: joint.c and the eight per-type joint files are ported,
  test_joint.c passes 11/11, joint goldens match C at 1, 2 and 4 workers. Milestone 3 (Dynamics)
  is complete. No spec is pending.
- Determinism status: test_determinism.c passes with upstream's float goldens verbatim on
  darwin/arm64 and darwin/amd64 (Rosetta), and under box3d_noassert, box3d_validate and -race:
  ragdoll (MultithreadingTest, sleep step 291, hash 0xFF4C605B) at 1, 2, 3, 4 and 5 workers and
  CrossPlatformTest at the default (1); wave pile (272, 0x266F83B0) at 1-4 workers; query spawn
  (242, 0xF3AD9BA6, 59 hits, query hash 0xE3271F3D) at 1-4 workers; mesh drop (226, 0xD2FF069D)
  at 1 and 4 workers as upstream (also checked by hand at 2, 3 and 8). The rolling mix and mesh mix
  scenes (upstream runs them only in SIMDWidthTest) agree run to run at 1, 2 and 4 workers.
  A world-level golden (`world_golden_test.go`, testdata/world_golden_gen.c) steps a joint-free
  scene (grid mesh ground, tilted static box, box stack, rolling sphere, capsule, rock and cylinder
  hulls, a bullet, a pile of small boxes and spheres) 240 steps and matches the C build bit for bit
  (every body transform and velocity, awake count, contact/island counters, b3HashWorldState) at 1,
  2 and 4 workers on darwin/arm64 and darwin/amd64; the C output is identical at 1 and 4 workers and
  for the NEON, BOX3D_DISABLE_SIMD and BOX3D_VALIDATE builds. Foundation math
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

Status legend: `todo`, `blocked (needs X)`, `ported` (compiles, no upstream test exists or test not
yet ported), `tested` (upstream test file ported and passing), `verified` (goldens or replay pass
through it), `n/a` (intentionally not ported; reason in notes). A header/source pair has one row
each; both point at the same Go file.

| upstream file | go file | status | ported at commit | notes |
|---|---|---|---|---|
| include/box3d/base.h | base.go | ported | efca0c1 | assert/validate via build tags; allocator API not ported |
| include/box3d/box3d.h | body.go, shape.go, contact.go, joint.go, physics_world.go, physics_world_queries.go | ported | efca0c1 | b3World_*, b3Body_*, b3Shape_*, b3Contact_* done (methods on the ids); b3Joint_* (methods on JointID), b3Create*Joint (methods on WorldID), b3DestroyJoint (JointID.Destroy) and b3Default*JointDef done with joint.c; b3<Kind>Joint_* are JointID methods <Kind>Joint<Method>; recording/replay API (b3CreateRecording, b3World_StartRecording, b3CreatePlayer, b3RecPlayer_*, …) done with M5 |
| include/box3d/collision.h | collision.go | ported | efca0c1 | blob helpers + hull, mesh and height field accessors done; compound functions are in compound.go |
| include/box3d/config.h | — | n/a | | compile options; Go uses build tags (box3d_noassert, box3d_validate). Double precision and SIMD-disable not ported |
| include/box3d/constants.h | constants.go | ported | efca0c1 |  |
| include/box3d/id.h | id.go | tested | efca0c1 |  |
| include/box3d/math_functions.h | math_functions.go | tested | efca0c1 |  |
| include/box3d/types.h | types.go | ported | efca0c1 | blob headers have C layout (hull sizes asserted in hull.go) |
| src/aabb.c | aabb.go | ported | efca0c1 | no upstream test (exercised by height_field in M2) |
| src/aabb.h | aabb.go | ported | efca0c1 |  |
| src/algorithm.h | — | n/a | | B3_SWAP becomes Go tuple assignment at call sites (hull.go) |
| src/arena_allocator.c | arena_allocator.go | ported | efca0c1 | typed stackAlloc[T]/bump[T] backed by per-type slices reused across steps (no allocation in a warm Step, step_alloc_test.go) |
| src/arena_allocator.h | arena_allocator.go | ported | efca0c1 |  |
| src/bitset.c | bitset.go | tested | efca0c1 | countSetBits lives in table.go as upstream |
| src/bitset.h | bitset.go | tested | efca0c1 |  |
| src/block_allocator.c | block_allocator.go | tested | efca0c1 | typed blockAllocator[T]; elements are []T of elementSize values |
| src/block_allocator.h | block_allocator.go | tested | efca0c1 |  |
| src/body.c | body.go | verified | efca0c1 | whole file in upstream order; b3Body_* are methods on BodyID, b3CreateBody is WorldID.CreateBody, b3DestroyBody is BodyID.Destroy; pointer+capacity outputs are slices; body_unit_test.go (Go-only) covers internals on a hand-built world; world_golden_test.go (testdata/world_golden_gen.c: mesh ground, hull stack, sphere, capsule, hulls, bullet; 240 steps bit for bit at 1, 2 and 4 workers, darwin/arm64 and amd64; C identical for NEON, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE and 1/4 workers) |
| src/body.h | body.go | ported | efca0c1 | header types (b3BodyFlags, b3Body, b3BodyState, b3_identityBodyState, b3BodySim) and static inlines (encode/decode sim index, b3MakeRelativeSweep); encoded sim indices are `int` |
| src/broad_phase.c | broad_phase.go | verified | efca0c1 | broad_phase_golden_test.go runs the real updateBroadPhasePairs against the C build (testdata/broad_phase_golden_gen.c); the test runs the real b3CreateContact; b3Prefetch dropped |
| src/broad_phase.h | broad_phase.go | verified | efca0c1 | proxy key macros are functions; b3BroadPhase_* are methods on *broadPhase |
| src/capsule.c | capsule.go | verified | efca0c1 | mass, AABB, ray cast (incl. parallel/degenerate), mover plane, overlap in hull_golden_test.go |
| src/compound.c | compound.go | verified | efca0c1 | creation (material/hull/mesh dedup, blob bytes hashed), ValidateCompound, children, sweep, ray/shape casts, overlap, mover, queries in geometry_golden_test.go; b3CompoundTimeOfImpact is #if 0 upstream (not ported) |
| src/compound.h | compound.go | verified | efca0c1 | getCompoundTreeView, makeCompoundChildSweep, collideMoverAndCompound |
| src/constraint_graph.c | constraint_graph.go | verified | efca0c1 | no upstream test; constraint_graph_test.go (Go-only) checks contact/joint coloring, overflow, mesh contacts and swap fix-ups; b3CreateJointInGraph and b3GetGraphColor added (no stubs existed); world_golden_test.go (testdata/world_golden_gen.c: mesh ground, hull stack, sphere, capsule, hulls, bullet; 240 steps bit for bit at 1, 2 and 4 workers, darwin/arm64 and amd64; C identical for NEON, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE and 1/4 workers) |
| src/constraint_graph.h | constraint_graph.go | ported | efca0c1 | header types (b3GraphColor, b3ConstraintGraph); void* wide buffers are `wideConstraintBuffer` (contact_solver_wide.go) |
| src/contact.c | contact.go | verified | efca0c1 | all functions incl. b3Contact_GetData (ContactID.GetData); register table matches upstream (supported/primary only, upstream has no fcn pointer); contact_test.go (Go-only) covers registers, create/destroy bookkeeping, convex and compound update paths; mesh path via computeMeshManifolds (mesh_contact.c); broad_phase_golden_test.go runs it; world_golden_test.go (testdata/world_golden_gen.c: mesh ground, hull stack, sphere, capsule, hulls, bullet; 240 steps bit for bit at 1, 2 and 4 workers, darwin/arm64 and amd64; C identical for NEON, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE and 1/4 workers) |
| src/contact.h | contact.go | ported | efca0c1 | header types; unions keep all members |
| src/contact_solver.c | contact_solver.go | verified | efca0c1 | overflow (_Mesh) kernels, overflow wrappers, convex/mesh-wide dispatchers (width-8 branches dropped); contact_solver_golden_test.go matches the C build (testdata/contact_solver_golden_gen.c; NEON, SSE2, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE identical); world_golden_test.go |
| src/contact_solver.h | contact_solver.go | verified | efca0c1 | header types; W4 declarations are the W kernels in contact_solver_wide.go, W8 not ported |
| src/contact_solver_wide.inl | contact_solver_wide.go | verified | efca0c1 | all kernels at laneCount (W suffix dropped); b3GatherBodies/b3ScatterBodies both B3_SIMD_NONE and lane variants via simdNone, both match the C goldens; stream walked as (constraint, manifold) indices in wideConstraintBuffer |
| src/container.h | container.go | tested | efca0c1 | b3Array(T) is []T; generic arrayPush/arrayPop/… keep growth policy |
| src/convex_manifold.c | convex_manifold.go | verified | efca0c1 | sphere/capsule/hull collide functions, SAT cache and manual modes; test_sat, test_manifold, manifold_golden_test.go |
| src/convex_manifold_wide.inl | convex_manifold_wide.go | verified | efca0c1 | computeSeparatingAxisW; both B3_SIMD_NONE and lane edge loops ported, selected by simdNone (simd_scalar.go); both match the C goldens |
| src/core.c | core.go | ported | efca0c1 |  |
| src/core.h | core.go | ported | efca0c1 |  |
| src/ctz.h | ctz.go | tested | efca0c1 | math/bits; tested by test_bitset TestBitMath |
| src/distance.c | distance.go | verified | efca0c1 | GJK, shape cast, TOI; covered by hull_golden_test.go |
| src/distance_joint.c | distance_joint.go | verified | efca0c1 | whole file in upstream order; test_joint.c TestDistanceJoint passes; joints_a_golden_test.go (testdata/joints_a_golden_gen.c: rigid and soft distance joints with spring, limit, motor, force range and setters; 240 steps bit for bit at 1, 2 and 4 workers, darwin/arm64 and amd64; C identical for NEON, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE and 1/4 workers) |
| src/dynamic_tree.c | dynamic_tree.go | verified | efca0c1 | insert/rotate, churn, moved marks, sweep and fallback refit, partial/full rebuild, save/load bytes, query/closest/ray/box cast order in dynamic_tree_golden_test.go (C scalar and NEON builds agree) |
| src/dynamic_tree.h | dynamic_tree.go | verified | efca0c1 | node flag helpers, treeView |
| src/height_field.c | height_field.go | verified | efca0c1 | grid/wave/custom fields (holes, clockwise, clamping, no materials), dump/load round trip, triangles, materials, AABB, ray/shape casts, overlap, mover, queries in geometry_golden_test.go (blob hashes match C) |
| src/hull.c | hull.go | verified | efca0c1 | quickhull builder, box hulls, 2D hull; hull_golden_test.go matches C bytes/hashes |
| src/hull.h | hull.go | verified | efca0c1 | hullMap = verstable[*HullData, int] |
| src/id_pool.c | id_pool.go | ported | efca0c1 | no upstream test |
| src/id_pool.h | id_pool.go | ported | efca0c1 |  |
| src/island.c | island.go | verified | efca0c1 | no upstream test; island_test.go (Go-only) checks link/merge, unlink, union-find split and validateIsland under box3d_validate; world_golden_test.go (islands counter and sleeping) |
| src/island.h | island.go | ported | efca0c1 | header types |
| src/joint.c | joint.go | verified | efca0c1 | whole file in upstream order (the #if 0 b3Joint_SetSpringRotationTarget is not ported); dispatches to the per-type files; test_joint.c passes (11/11); joints_a_golden_test.go and joints_b_golden_test.go step every joint type through it bit for bit (1, 2 and 4 workers, darwin/arm64 and amd64) |
| src/joint.h | joint.go | ported | efca0c1 | header types; b3JointSim union keeps all joint members (`sim.revoluteJoint` etc.); inline b3SolvePointConstraint ported (used by the per-type files) |
| src/manifold.c | manifold.go | verified | efca0c1 | findIncidentFace, clipPolygon, flipPair checked in hull_golden_test.go |
| src/manifold.h | manifold.go | ported | efca0c1 | separating axis / clip types; b3ComputeSeparatingAxis*, b3CollideHullsAtWidth are in convex_manifold.go (W4 → computeSeparatingAxisW in convex_manifold_wide.go) |
| src/math_functions.c | math_functions.go | tested | efca0c1 |  |
| src/math_internal.h | math_internal.go | ported | efca0c1 | declared functions land with their .c files |
| src/mesh.c | mesh.go | verified | efca0c1 | creators, CreateMesh (weld, stride, SAH/median, clockwise, degenerates, edge flags), queries, ray/shape casts, overlap, mover in geometry_golden_test.go (blob hashes match C) |
| src/mesh_contact.c | mesh_contact.go | verified | efca0c1 | bit-exact vs C on mesh_contact_golden_test.go (meshes, height fields, sphere/capsule/hull, cache refresh, impulse matching, materials, cullPoints/reduceCluster; darwin/arm64 and amd64; C output identical for NEON, SSE2, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE); world_golden_test.go (mesh ground) |
| src/motor_joint.c | motor_joint.go | verified | efca0c1 | whole file in upstream order; TestMotorJoint passes; joints_a_golden_test.go (velocity targets, linear and angular springs, setters) |
| src/mover.c | mover.go | verified | efca0c1 | SolvePlanes (delta, iterations, pushes) and ClipVector in mover_golden_test.go |
| src/name_cache.c | name_cache.go | tested | efca0c1 | pulled forward from M5; includes b3Hash32 (checked against C) and b3LoadName; names round trip through recording and rollback |
| src/name_cache.h | name_cache.go | ported | efca0c1 | header types and inline b3FindNameWithDefault |
| src/parallel_for.c | parallel_for.go | tested | efca0c1 | no upstream test; parallel_for_test.go (Go-only) checks the block partition, claiming with goroutines, the serial default task system and the B3_MAX_TASKS fallback |
| src/parallel_for.h | parallel_for.go | tested | efca0c1 |  |
| src/parallel_joint.c | parallel_joint.go | verified | efca0c1 | b3ParallelJoint_* are JointID methods ParallelJoint<Method>; joints_b_golden_test.go (testdata/joints_b_golden_gen.c: spherical, weld, wheel and parallel joints with setters changed mid-run, 240 steps bit for bit incl. constraint force/torque, per-type getters, joint events and joint debug draw, at 1, 2 and 4 workers, darwin/arm64 and amd64; C identical for NEON, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE and 1/4 workers) |
| src/physics_world.c | physics_world.go, physics_world_queries.go | verified | efca0c1 | whole file in upstream order, split in two Go files (world queries, casts, overlaps and mover in physics_world_queries.go; both carry the upstream header); b3World_* are methods on WorldID, getWorldFromId added for tests; physics_world_unit_test.go (Go-only) covers the hull database, id validation and the validators; world_golden_test.go (testdata/world_golden_gen.c: mesh ground, hull stack, sphere, capsule, hulls, bullet; 240 steps bit for bit at 1, 2 and 4 workers, darwin/arm64 and amd64; C identical for NEON, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE and 1/4 workers) |
| src/physics_world.h | physics_world.go | ported | efca0c1 | header types (b3SetType, b3DebugPoint/Line, b3TaskContext, b3World) and inlines b3AllocateManifolds / b3FreeManifolds (slices of count manifolds) |
| src/platform.h | platform.go | ported | efca0c1 |  |
| src/prismatic_joint.c | prismatic_joint.go | verified | efca0c1 | whole file in upstream order; TestPrismaticJoint passes; joints_a_golden_test.go (static-dynamic slider on a rotated axis with spring, limit, motor; dynamic pair; setters, GetTranslation/GetSpeed) |
| src/qsort.h | qsort.go | verified | efca0c1 | macros become functions taking LESS/SWAP closures; call counts and the unstable order of equal keys match C in qsort_test.go (testdata/qsort_golden_gen.c) |
| src/rapidhash.h | rapidhash.go | tested | efca0c1 | no upstream test; rapidhash_test.go checks C reference values |
| src/recording.c | recording.go, recording_ops.go | verified | efca0c1 | all functions ported; manifest writer expansions in recording_ops.go; op stream + registry byte-identical to C, joint ops included (recording_golden_test.go, replay_joints_golden_test.go; 1/2/4 workers, darwin arm64/amd64); b3HashWorldState pulled forward in M3 |
| src/recording.h | recording.go | ported | efca0c1 | types, constants, b3FnvMixPosition; b3Recording is the exported opaque `Recording`; dedupMap/tagMap are Go maps |
| src/recording_ops.inl | recording_ops.go | ported | efca0c1 | arg structs, opcode constants, writer expansions (b3RecWriteArgs_/b3RecWrite_/b3RecWriteRet_); reader expansions in replay_ops.go |
| src/replay.c | replay.go, replay_ops.go, replay_player.go | verified | efca0c1 | C-made recordings without joints (testdata/replay_golden_gen.c) and with every joint type and joint op (testdata/replay_joints_golden_gen.c), empty-world seeds, replay with every StateHash and the final body/joint states matching C at 1/2/4 workers, darwin arm64/amd64; player keyframe seek/scrub and restart match; Go-made recordings replay |
| src/replay.h | replay.go | verified | efca0c1 | |
| src/revolute_joint.c | revolute_joint.go | verified | efca0c1 | whole file in upstream order; TestRevoluteJoint passes; joints_a_golden_test.go (hinge with spring, limit, motor; kinematic-driven tilted axis; free dynamic pair; setters, GetAngle) |
| src/scheduler.c | scheduler.go | verified | efca0c1 | no upstream test; scheduler_test.go (Go-only) checks enqueue/finish at 1/2/4/8 workers, main-thread help, reset, shutdown and parallelFor on the scheduler (race-clean); world_golden_test.go at 2 and 4 workers |
| src/scheduler.h | scheduler.go | ported | efca0c1 | b3TaskCallback is the public TaskCallback; b3Scheduler defined in scheduler.c section |
| src/sensor.c | sensor.go | tested | efca0c1 | Go-only sensor_test.go covers the hit path, events, disabled path and destroy at 1 and 4 workers; world_test.go TestSensor covers the query path |
| src/sensor.h | sensor.go | ported | efca0c1 | header types only |
| src/shape.c | shape.go | verified | efca0c1 | all functions in upstream order; b3Shape_* are ShapeID methods, b3Create*Shape are BodyID methods, b3DestroyShape is ShapeID.Destroy; world-free functions verified bit for bit in shape_golden_test.go (testdata/shape_golden_gen.c links the whole library); world_golden_test.go |
| src/shape.h | shape.go | ported | efca0c1 | b3Shape (union members all present), b3ShapeFlags, filter/convex inlines, b3GetHeightFieldTriangleCount, b3GetMeshTriangle; declared functions are in the shape.c section |
| src/simd.c | simd.go | ported | efca0c1 | also holds IsAVX2Available / Get/SetSIMDWidth; Go-only simd_test.go |
| src/simd.h | simd.go | ported | efca0c1 | b3V32 non-SSE2 path as `v32`; b3Vec3W4 as `vec3W` |
| src/simd_avx2.h | — | n/a | | dropped: scalar lanes only (see spec, SIMD lane layer) |
| src/simd_neon.h | — | n/a | | dropped: scalar lanes only |
| src/simd_scalar.h | simd_scalar.go | ported | efca0c1 | `floatW{v [laneCount]float32}`, laneCount = 4; W4 suffix dropped; `simdNone` (B3_SIMD_NONE) lives here |
| src/simd_sse2.h | — | n/a | | dropped: scalar lanes only |
| src/simd_wide.h | simd_wide.go | ported | efca0c1 | name map only + wideAlignment |
| src/solver.c | solver.go | verified | efca0c1 | whole file; block partition, worker start indices and the body kernels (integrate velocities incl. gyroscopic, integrate positions) match C in solver_golden_test.go (testdata/solver_golden_gen.c, darwin/arm64 and amd64; C identical for NEON, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE); solver_test.go checks atomic block claiming and the b3SolverTask spinner protocol with goroutines; b3Solve end to end in world_golden_test.go (testdata/world_golden_gen.c: mesh ground, hull stack, sphere, capsule, hulls, bullet; 240 steps bit for bit at 1, 2 and 4 workers, darwin/arm64 and amd64; C identical for NEON, BOX3D_DISABLE_SIMD, BOX3D_VALIDATE and 1/4 workers) |
| src/solver.h | solver.go | ported | efca0c1 | stage/block enums, b3SolverBlock, b3SyncBlock, b3SolverStage, b3Softness, prepare spans, b3StepContext (pointers are slices, void* buffers are `wideConstraintBuffer`), b3MakeSoft |
| src/solver_set.c | solver_set.go | verified | efca0c1 | no upstream test; solver_set_test.go (Go-only) checks sleep/wake, merge, transferBody/transferJoint (its hand-built world satisfies b3ValidateSolverSets); b3MergeSolverSets added (no stub existed); world_golden_test.go (sleeping) |
| src/solver_set.h | solver_set.go | ported | efca0c1 | header types |
| src/sphere.c | sphere.go | verified | efca0c1 | mass, AABB, ray casts, mover plane, overlap in hull_golden_test.go |
| src/spherical_joint.c | spherical_joint.go | verified | efca0c1 | b3SphericalJoint_* are JointID methods SphericalJoint<Method>; joints_b_golden_test.go (see parallel_joint.c) |
| src/table.c | table.go | tested | efca0c1 | hashSet keeps upstream slot order |
| src/table.h | table.go | tested | efca0c1 |  |
| src/timer.c | timer.go | tested | efca0c1 | no upstream test; timer_test.go (Go-only) |
| src/triangle_manifold.c | triangle_manifold.go | verified | efca0c1 | triangle vs sphere/capsule/hull; triangle vertices are a slice; manifold_golden_test.go |
| src/types.c | types.go | ported | efca0c1 |  |
| src/verstable.h | verstable.go | ported | efca0c1 | generic `verstable[K, V]` (linear probing) with the API subset upstream uses; Go-only verstable_test.go |
| src/weld_joint.c | weld_joint.go | verified | efca0c1 | b3WeldJoint_* are JointID methods WeldJoint<Method>; joints_b_golden_test.go (see parallel_joint.c) |
| src/wheel_joint.c | wheel_joint.go | verified | efca0c1 | b3WheelJoint_* are JointID methods WheelJoint<Method>; #if 0 b3WheelJoint_Dump not ported; joints_b_golden_test.go (see parallel_joint.c) |
| src/wide4.c | — | n/a | | dropped: includes the .inl kernels, ported once |
| src/wide8.c | — | n/a | | dropped: width 8 not ported |
| src/world_snapshot.c | world_snapshot.go | verified | efca0c1 | Go-native image + Go layout hash; round trip and lock-step stepping in world_snapshot_test.go; keyframe seek with every joint type (replay_joints_golden_test.go); C images rejected before the world is touched; player keyframes use it |
| src/world_snapshot.h | world_snapshot.go | ported | efca0c1 | |
| test/main.c | box3d_suite_test.go | ported | efca0c1 | Ginkgo suite runner |
| test/test_allocator.c | allocator_test.go | tested | efca0c1 | alignment ENSURE checks Go alignment of T |
| test/test_bitset.c | bitset_test.go | tested | efca0c1 |  |
| test/test_body.c | body_test.go | tested | efca0c1 | all 11 tests |
| test/test_body_query.c | body_query_test.go | tested | efca0c1 | all 31 tests; the MoverTOIMatchesSweep random loop matches a C build bit for bit (hit count, seed, hashed fractions and points; arm64 and amd64) |
| test/test_broad_phase.c | broad_phase_test.go | tested | efca0c1 | both tests (StaticRebuildKeepsMarks, BroadPhaseStressTest: 3 seeds at 1, 4 workers and 3 workers with sleep, 400 steps each, pair set brute-force checked every step); broad_phase_golden_test.go also covers the broad-phase |
| test/test_collision.c | collision_test.go | tested | efca0c1 | 4 of 4 tests; the BOX3D_DOUBLE_PRECISION halves are compiled out as in upstream's float build |
| test/test_compound.c | compound_test.go | tested | efca0c1 | all 34 tests; byte-level serialize tests port as written |
| test/test_container.c | container_test.go | tested | efca0c1 |  |
| test/test_determinism.c | determinism_test.go | verified | efca0c1 | all 6 subtests with upstream float goldens verbatim (ragdoll 1-5 workers, wave pile 1-4, query spawn 1-4, mesh drop 1 and 4, cross-platform); SIMDWidthTest skips at native width 4 as upstream; Go-only spec runs rolling mix and mesh mix at 1/2/4 workers |
| test/test_distance.c | distance_test.go | tested | efca0c1 |  |
| test/test_dynamic_tree.c | dynamic_tree_test.go | tested | efca0c1 |  |
| test/test_hash.c | hash_test.go | tested | efca0c1 | all 13 tests |
| test/test_height_field.c | height_field_test.go | tested | efca0c1 | round trip file in a temp dir |
| test/test_hull.c | hull_test.go | tested | efca0c1 |  |
| test/test_id.c | id_test.go | tested | efca0c1 |  |
| test/test_joint.c | joint_test.go | tested | efca0c1 | all 11 tests |
| test/test_large_world.c | large_world_test.go | tested | efca0c1 | all 3 tests; double-precision halves not ported (as collision_test.go); the origin runs print the same values as a C build |
| test/test_macros.h | test_macros_test.go | ported | efca0c1 | ENSURE / ENSURE_SMALL helpers |
| test/test_manifold.c | manifold_test.go | tested | efca0c1 | exercises b3Collide* from convex_manifold.c and triangle_manifold.c |
| test/test_math.c | math_test.go | tested | efca0c1 |  |
| test/test_mesh.c | mesh_test.go | tested | efca0c1 | strided input via `fatPositions` (unsafe.Slice over the interleaved array) |
| test/test_mover.c | mover_test.go | tested | efca0c1 | all 22 tests |
| test/test_name_cache.c | name_cache_test.go | tested | efca0c1 | all 3 tests (CacheUnit, NameRoundTrip, RollbackNames) |
| test/test_recording.c | recording_test.go | tested | efca0c1 | all 21 tests; CrossWidthReplay takes upstream's native-width-4 skip |
| test/test_restitution.c | restitution_test.go | tested | efca0c1 | all 14 subtests (one It each); the measurements match a C build line for line, WorkerParityTest hashes included (1, 2, 4 workers) |
| test/test_sat.c | sat_test.go | tested | efca0c1 | oracle stats print identical to the C build (NEON and BOX3D_DISABLE_SIMD) |
| test/test_shape.c | shape_test.go, shape_world_test.go | tested | efca0c1 | all 20 subtests (PointInShapeTest is #if 0 upstream); ShapeNameTest and ShapeFlagsTest are in shape_world_test.go |
| test/test_table.c | table_test.go | tested | efca0c1 |  |
| test/test_world.c | world_test.go | tested | efca0c1 | all 26 tests (BodySimLocatorTest includes LocatorTransferScene's spherical joint across two sleeping sets) |
| shared/utils.c | utils_test.go | tested | efca0c1 | test random helpers (RandomFloat, …) |
| shared/utils.h | utils_test.go | tested | efca0c1 | test random helpers (RandomFloat, …) |
| shared/determinism.c | scenes_determinism_test.go | ported | efca0c1 | test scene helper in a _test.go file of package box3d (internal tests cannot import a package that imports box3d); used by test_determinism.c (M5) |
| shared/determinism.h | scenes_determinism_test.go | ported | efca0c1 | test scene helper |
| shared/human.c | scenes_human_test.go | ported | efca0c1 | test scene helper (revolute, spherical, filter, motor, parallel joints); the #if 0 pose control block is not ported |
| shared/human.h | scenes_human_test.go | ported | efca0c1 | test scene helper |
| shared/benchmarks.c | scenes_benchmarks_test.go | tested | efca0c1 | every scene (joint grid, large/wide/many pyramids, rain, large world, washer, trees, mesh drop, junkyard, convex pile, spinner, sleep, village) and its capacity/step/destroy helpers; BENCHMARK_DEBUG is the variable benchmarkDebug (tests: debug sizes, bench_test.go: release sizes); used by bench_test.go, step_alloc_test.go and test_world.c |
| shared/benchmarks.h | scenes_benchmarks_test.go | tested | efca0c1 | see benchmarks.c; VillageCast / VillageCurtain are villageCast / villageCurtain |
| shared/overflow_color.c | scenes_benchmarks_test.go | ported | efca0c1 | CreateOverflowColorPile (test_world.c TestOverflowColorPile); cosf / sinf are float64 math rounded to float32 |
| shared/overflow_color.h | scenes_benchmarks_test.go | ported | efca0c1 | see overflow_color.c |
| shared/sat_benchmark.c | — | n/a | efca0c1 | GJK versus SAT micro benchmark behind BOX3D_INTERNAL_BENCHMARKS (static linkage only); not ported |
| shared/sat_benchmark.h | — | n/a | efca0c1 | see sat_benchmark.c |
| benchmark/main.c | bench_test.go | tested | efca0c1 | one testing.B per registered benchmark, sub-benchmarks per worker count (BOX3D_BENCH_WORKERS, default 1,2,4,8), same step counts and timing (first step untimed), release scene sizes; the SAT benchmark, -s step recording, Tracy and the commented-out smash/tumbler entries are not ported; results in PERFORMANCE.md |
| shared/stability.c | scenes_stability_test.go | verified | efca0c1 | complete (CreateMeshDrop, UpdateMeshDrop, DestroyMeshDrop are the whole file); test_determinism.c MeshDropTest golden |
| shared/stability.h | scenes_stability_test.go | verified | efca0c1 | see stability.c |
