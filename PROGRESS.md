# Box3D Go port — progress

- Upstream: [erincatto/box3d](https://github.com/erincatto/box3d) at `efca0c184d40fd6a82016983f1601c4d23965867`
  (submodule `third_party/box3d`).
- Design: `docs/superpowers/specs/2026-10-17-box3d-go-port-design.md` in the gata monorepo.
- Current milestone: **1 — Foundation**.
- Determinism status: no goldens yet (they arrive with milestone 3/5). Fused multiply-add guard
  (`fma_guard_test.go`) passes for arm64 and amd64 v3.

Status legend: `todo`, `blocked (needs X)`, `ported` (compiles, no upstream test exists or test not
yet ported), `tested` (upstream test file ported and passing), `verified` (goldens or replay pass
through it), `n/a` (intentionally not ported; reason in notes). A header/source pair has one row
each; both point at the same Go file.

| upstream file | go file | status | ported at commit | notes |
|---|---|---|---|---|
| include/box3d/base.h | base.go | todo | |  |
| include/box3d/box3d.h | body.go, shape.go, joint.go, physics_world.go, … | todo | | b3Type_* functions live in the file of the object they act on |
| include/box3d/collision.h | collision.go | todo | |  |
| include/box3d/config.h | — | n/a | | compile options; Go uses build tags (box3d_noassert, box3d_validate). Double precision and SIMD-disable not ported |
| include/box3d/constants.h | constants.go | todo | |  |
| include/box3d/id.h | id.go | tested | efca0c1 |  |
| include/box3d/math_functions.h | math_functions.go | todo | |  |
| include/box3d/types.h | types.go | todo | |  |
| src/aabb.c | aabb.go | todo | |  |
| src/aabb.h | aabb.go | todo | |  |
| src/algorithm.h | — | n/a | | B3_SWAP becomes Go tuple assignment at call sites |
| src/arena_allocator.c | arena_allocator.go | todo | |  |
| src/arena_allocator.h | arena_allocator.go | todo | |  |
| src/bitset.c | bitset.go | todo | |  |
| src/bitset.h | bitset.go | todo | |  |
| src/block_allocator.c | block_allocator.go | todo | |  |
| src/block_allocator.h | block_allocator.go | todo | |  |
| src/body.c | body.go | todo | |  |
| src/body.h | body.go | todo | |  |
| src/broad_phase.c | broad_phase.go | todo | |  |
| src/broad_phase.h | broad_phase.go | todo | |  |
| src/capsule.c | capsule.go | todo | |  |
| src/compound.c | compound.go | todo | |  |
| src/compound.h | compound.go | todo | |  |
| src/constraint_graph.c | constraint_graph.go | todo | |  |
| src/constraint_graph.h | constraint_graph.go | todo | |  |
| src/contact.c | contact.go | todo | |  |
| src/contact.h | contact.go | todo | |  |
| src/contact_solver.c | contact_solver.go | todo | |  |
| src/contact_solver.h | contact_solver.go | todo | |  |
| src/contact_solver_wide.inl | contact_solver_wide.go | todo | |  |
| src/container.h | container.go | todo | |  |
| src/convex_manifold.c | convex_manifold.go | todo | |  |
| src/convex_manifold_wide.inl | convex_manifold_wide.go | todo | |  |
| src/core.c | core.go | todo | |  |
| src/core.h | core.go | todo | |  |
| src/ctz.h | ctz.go | todo | |  |
| src/distance.c | distance.go | todo | |  |
| src/distance_joint.c | distance_joint.go | todo | |  |
| src/dynamic_tree.c | dynamic_tree.go | todo | |  |
| src/dynamic_tree.h | dynamic_tree.go | todo | |  |
| src/height_field.c | height_field.go | todo | |  |
| src/hull.c | hull.go | todo | |  |
| src/hull.h | hull.go | todo | |  |
| src/id_pool.c | id_pool.go | todo | |  |
| src/id_pool.h | id_pool.go | todo | |  |
| src/island.c | island.go | todo | |  |
| src/island.h | island.go | todo | |  |
| src/joint.c | joint.go | todo | |  |
| src/joint.h | joint.go | todo | |  |
| src/manifold.c | manifold.go | todo | |  |
| src/manifold.h | manifold.go | todo | |  |
| src/math_functions.c | math_functions.go | todo | |  |
| src/math_internal.h | math_internal.go | todo | |  |
| src/mesh.c | mesh.go | todo | |  |
| src/mesh_contact.c | mesh_contact.go | todo | |  |
| src/motor_joint.c | motor_joint.go | todo | |  |
| src/mover.c | mover.go | todo | |  |
| src/name_cache.c | name_cache.go | todo | |  |
| src/name_cache.h | name_cache.go | todo | |  |
| src/parallel_for.c | parallel_for.go | todo | |  |
| src/parallel_for.h | parallel_for.go | todo | |  |
| src/parallel_joint.c | parallel_joint.go | todo | |  |
| src/physics_world.c | physics_world.go | todo | |  |
| src/physics_world.h | physics_world.go | todo | |  |
| src/platform.h | platform.go | todo | |  |
| src/prismatic_joint.c | prismatic_joint.go | todo | |  |
| src/qsort.h | qsort.go | todo | |  |
| src/rapidhash.h | rapidhash.go | todo | |  |
| src/recording.c | recording.go | todo | |  |
| src/recording.h | recording.go | todo | |  |
| src/recording_ops.inl | recording_ops.go | todo | |  |
| src/replay.c | replay.go | todo | |  |
| src/replay.h | replay.go | todo | |  |
| src/revolute_joint.c | revolute_joint.go | todo | |  |
| src/scheduler.c | scheduler.go | todo | |  |
| src/scheduler.h | scheduler.go | todo | |  |
| src/sensor.c | sensor.go | todo | |  |
| src/sensor.h | sensor.go | todo | |  |
| src/shape.c | shape.go | todo | |  |
| src/shape.h | shape.go | todo | |  |
| src/simd.c | simd.go | todo | |  |
| src/simd.h | simd.go | todo | |  |
| src/simd_avx2.h | — | n/a | | dropped: scalar lanes only (see spec, SIMD lane layer) |
| src/simd_neon.h | — | n/a | | dropped: scalar lanes only |
| src/simd_scalar.h | simd_scalar.go | todo | |  |
| src/simd_sse2.h | — | n/a | | dropped: scalar lanes only |
| src/simd_wide.h | simd_wide.go | todo | |  |
| src/solver.c | solver.go | todo | |  |
| src/solver.h | solver.go | todo | |  |
| src/solver_set.c | solver_set.go | todo | |  |
| src/solver_set.h | solver_set.go | todo | |  |
| src/sphere.c | sphere.go | todo | |  |
| src/spherical_joint.c | spherical_joint.go | todo | |  |
| src/table.c | table.go | todo | |  |
| src/table.h | table.go | todo | |  |
| src/timer.c | timer.go | todo | |  |
| src/triangle_manifold.c | triangle_manifold.go | todo | |  |
| src/types.c | types.go | todo | |  |
| src/verstable.h | verstable.go | todo | |  |
| src/weld_joint.c | weld_joint.go | todo | |  |
| src/wheel_joint.c | wheel_joint.go | todo | |  |
| src/wide4.c | — | n/a | | dropped: includes the .inl kernels, ported once |
| src/wide8.c | — | n/a | | dropped: width 8 not ported |
| src/world_snapshot.c | world_snapshot.go | todo | |  |
| src/world_snapshot.h | world_snapshot.go | todo | |  |
| test/main.c | box3d_suite_test.go | ported | efca0c1 | Ginkgo suite runner |
| test/test_allocator.c | allocator_test.go | todo | |  |
| test/test_bitset.c | bitset_test.go | todo | |  |
| test/test_body.c | body_test.go | todo | |  |
| test/test_body_query.c | body_query_test.go | todo | |  |
| test/test_broad_phase.c | broad_phase_test.go | todo | |  |
| test/test_collision.c | collision_test.go | todo | |  |
| test/test_compound.c | compound_test.go | todo | |  |
| test/test_container.c | container_test.go | todo | |  |
| test/test_determinism.c | determinism_test.go | todo | |  |
| test/test_distance.c | distance_test.go | todo | |  |
| test/test_dynamic_tree.c | dynamic_tree_test.go | todo | |  |
| test/test_hash.c | hash_test.go | todo | |  |
| test/test_height_field.c | height_field_test.go | todo | |  |
| test/test_hull.c | hull_test.go | todo | |  |
| test/test_id.c | id_test.go | tested | efca0c1 |  |
| test/test_joint.c | joint_test.go | todo | |  |
| test/test_large_world.c | large_world_test.go | todo | |  |
| test/test_macros.h | test_macros_test.go | ported | efca0c1 | ENSURE / ENSURE_SMALL helpers |
| test/test_manifold.c | manifold_test.go | todo | |  |
| test/test_math.c | math_test.go | todo | |  |
| test/test_mesh.c | mesh_test.go | todo | |  |
| test/test_mover.c | mover_test.go | todo | |  |
| test/test_name_cache.c | name_cache_test.go | todo | |  |
| test/test_recording.c | recording_test.go | todo | |  |
| test/test_restitution.c | restitution_test.go | todo | |  |
| test/test_sat.c | sat_test.go | todo | |  |
| test/test_shape.c | shape_test.go | todo | |  |
| test/test_table.c | table_test.go | todo | |  |
| test/test_world.c | world_test.go | todo | |  |
| shared/utils.c | utils_test.go | todo | | test random helpers (RandomFloat, …) |
| shared/utils.h | utils_test.go | todo | | test random helpers (RandomFloat, …) |
| shared/determinism.c | internal/scenes/determinism.go | todo | | test/benchmark scene helper |
| shared/determinism.h | internal/scenes/determinism.go | todo | | test/benchmark scene helper |
| shared/human.c | internal/scenes/human.go | todo | | test/benchmark scene helper |
| shared/human.h | internal/scenes/human.go | todo | | test/benchmark scene helper |
| shared/benchmarks.c | internal/scenes/benchmarks.go | todo | | test/benchmark scene helper |
| shared/benchmarks.h | internal/scenes/benchmarks.go | todo | | test/benchmark scene helper |
