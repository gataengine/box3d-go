# Performance

Spec scope: the only performance goal of the port is **no allocations in `World.Step`**. Speed
beyond that is a non-goal for now (scalar lanes first, a SIMD backend later). This file records
where the port stands against upstream so later work has a baseline.

## Environment

| | |
|---|---|
| Machine | Apple M1 Max (8 performance + 2 efficiency cores), 64 GB, macOS 15.7.4 |
| Go | go1.27.1 darwin/arm64, `-tags box3d_noassert` (upstream's release build compiles `B3_ASSERT` out) |
| C | upstream `efca0c1`, built out of tree, Apple clang 17.0.0, `-O3 -DNDEBUG -ffp-contract=off`, `BOX3D_DISABLE_SIMD=ON` (scalar lanes, the same backend the port uses), `BOX3D_VALIDATE=OFF` |
| Upstream CSVs | `third_party/box3d/benchmark/amd7950x_scalar/*.csv` (AMD Ryzen 9 7950X, scalar) and `m2air_neon/*.csv` (Apple M2 MacBook Air, NEON, 1-4 threads). There is no upstream ARM scalar CSV. Different hardware and possibly older upstream code: use them for shape, not for ratios |

## Methodology

- C: `cmake -S third_party/box3d -B /private/tmp/claude-501/box3d-bench-scalar -DCMAKE_BUILD_TYPE=Release
  -DBOX3D_BENCHMARKS=ON -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF -DBOX3D_DISABLE_SIMD=ON
  -DBOX3D_VALIDATE=OFF`, then `benchmark -t=8 -w=N -r=2` for N = 1, 4, 8. The figure is the
  minimum of the runs, as upstream writes it into its CSVs.
- Go: `BOX3D_BENCH_WORKERS=1,4,8 go test -tags box3d_noassert -run '^$' -bench . -benchtime 1x
  -count 2 .` in `libs/box3d-go`; the figure is the minimum of the two ns/op values. One b.N
  iteration is exactly one run of `benchmark/main.c`: world created with the benchmark's
  capacity, step function called with 0, one untimed step, then steps 1 .. N-1 timed (scene step
  functions included, as upstream). Scene construction and teardown are outside the timer, and a
  `runtime.GC()` runs before the timer starts.
- Both ran back to back on an otherwise idle machine, plugged in. Times are total milliseconds for
  all timed steps of one run (upstream's `ms` column). `Go / C` compares against the C scalar
  build on this machine (apples to apples).
- Scenes are upstream's release (NDEBUG) sizes; body and contact counts at the end of a run match
  C's counters (e.g. convex_pile 5121 bodies / 51063 contacts, junkyard 10586 / 193353).

## Results

| benchmark | workers | Go ms | C scalar (this Mac) ms | Go / C | upstream amd7950x scalar ms | upstream m2air neon ms | Go allocs/run |
|---|---|---|---|---|---|---|---|
| convex_pile | 1 | 36,656.0 | 5,994.8 | 6.11 | 40,706.0 | 15,032.6 | 5,276 |
| convex_pile | 4 | 11,006.7 | 1,925.0 | 5.72 | 10,323.8 | 4,296.5 | 5,401 |
| convex_pile | 8 | 8,321.3 | 1,536.5 | 5.42 | 5,292.3 | — | 5,553 |
| joint_grid | 1 | 2,987.3 | 903.6 | 3.31 | 1,525.9 | 1,089.8 | 18 |
| joint_grid | 4 | 932.7 | 275.5 | 3.39 | 401.4 | 309.3 | 19 |
| joint_grid | 8 | 685.2 | 232.2 | 2.95 | 213.6 | — | 33 |
| junkyard | 1 | 80,771.4 | 12,338.9 | 6.55 | 31,501.3 | 13,501.4 | 12,807 |
| junkyard | 4 | 22,150.9 | 4,241.8 | 5.22 | 8,563.2 | 4,253.7 | 12,906 |
| junkyard | 8 | 15,505.8 | 3,432.4 | 4.52 | 4,517.7 | — | 13,034 |
| large_pyramid | 1 | 21,405.3 | 1,932.1 | 11.08 | 5,026.7 | 1,810.7 | 22 |
| large_pyramid | 4 | 6,316.7 | 668.4 | 9.45 | 1,368.5 | 563.5 | 22 |
| large_pyramid | 8 | 4,298.2 | 557.0 | 7.72 | 720.5 | — | 24 |
| large_world | 1 | 61.6 | 5.9 | 10.45 | 24.6 | 9.0 | 697 |
| large_world | 4 | 76.6 | 14.7 | 5.21 | 27.9 | 15.0 | 703 |
| large_world | 8 | 84.2 | 17.6 | 4.79 | 28.8 | — | 740 |
| many_pyramids | 1 | 21,317.8 | 1,855.1 | 11.49 | 4,932.9 | 1,917.4 | 20 |
| many_pyramids | 4 | 5,342.2 | 690.3 | 7.74 | 1,272.4 | 621.3 | 25 |
| many_pyramids | 8 | 3,636.9 | 579.7 | 6.27 | 635.9 | — | 23 |
| mesh_drop | 1 | 9,844.9 | 1,582.8 | 6.22 | — | — | 59,662 |
| mesh_drop | 4 | 2,840.3 | 577.2 | 4.92 | — | — | 59,887 |
| mesh_drop | 8 | 1,775.1 | 464.3 | 3.82 | — | — | 60,127 |
| rain | 1 | 4,843.6 | 1,721.3 | 2.81 | 2,540.8 | 1,954.8 | 26,788 |
| rain | 4 | 1,781.6 | 704.7 | 2.53 | 783.7 | 626.2 | 27,023 |
| rain | 8 | 1,503.5 | 716.5 | 2.10 | 467.8 | — | 27,320 |
| sleep | 1 | 12,544.0 | 980.5 | 12.79 | — | — | 393 |
| sleep | 4 | 3,582.8 | 464.4 | 7.71 | — | — | 393 |
| sleep | 8 | 2,702.8 | 422.0 | 6.40 | — | — | 394 |
| spinner | 1 | 20,125.0 | 3,149.3 | 6.39 | — | — | 5,529 |
| spinner | 4 | 8,227.6 | 1,665.0 | 4.94 | — | — | 5,879 |
| spinner | 8 | 6,156.7 | 1,607.6 | 3.83 | — | — | 6,167 |
| trees100 | 1 | 689.1 | 125.2 | 5.51 | 221.6 | 186.1 | 2,867 |
| trees100 | 4 | 451.0 | 88.6 | 5.09 | 100.1 | 91.2 | 3,124 |
| trees100 | 8 | 474.7 | 93.0 | 5.10 | 85.7 | — | 3,380 |
| trees50 | 1 | 1,108.6 | 201.6 | 5.50 | 346.2 | 288.3 | 3,764 |
| trees50 | 4 | 618.4 | 113.3 | 5.46 | 134.7 | 156.9 | 4,068 |
| trees50 | 8 | 534.5 | 115.5 | 4.63 | 113.3 | — | 4,308 |
| trees25 | 1 | 1,987.1 | 468.9 | 4.24 | 815.3 | 609.9 | 4,971 |
| trees25 | 4 | 938.1 | 205.7 | 4.56 | 296.8 | 236.2 | 5,311 |
| trees25 | 8 | 889.9 | 180.0 | 4.94 | 234.0 | — | 5,599 |
| village | 1 | 21,835.4 | 3,143.3 | 6.95 | — | — | 2,195,506 |
| village | 4 | 6,760.0 | 1,220.0 | 5.54 | — | — | 2,195,852 |
| village | 8 | 5,613.7 | 1,071.5 | 5.24 | — | — | 2,196,009 |
| washer | 1 | 130,195.4 | 19,810.8 | 6.57 | 42,026.4 | 21,065.2 | 56,521 |
| washer | 4 | 38,227.4 | 7,563.5 | 5.05 | 11,643.4 | 7,675.2 | 56,592 |
| washer | 8 | 29,613.4 | 5,976.1 | 4.96 | 6,338.7 | — | 56,720 |

Reading the table:

- The Go port is 2-13x slower than the same C code with the same scalar lane backend on the same
  machine (median 6.4x at 1 worker, 5.2x at 4). Joint-heavy scenes (joint_grid, rain) are
  closest; solver-bound stacking (large_pyramid, many_pyramids, sleep) is furthest.
- Multithreaded scaling has the same shape as C (convex_pile 4.4x from 1 to 8 workers in Go,
  3.9x in C), so the gap is single-thread code generation, not the task system.
- The upstream CSVs are much slower than upstream code on this machine for some scenes
  (convex_pile amd7950x scalar 40.7 s vs 6.0 s here), so they predate optimisations in the pinned
  commit or use other settings; compare against the "C scalar (this Mac)" column.
- `Go allocs/run` counts every allocation in the timed steps of one run, including the warm-up
  part of the run (contact, island and mesh-cache containers growing to their high water marks,
  the arenas and stack growing between steps as upstream does) and the scene step functions
  (rain creates and destroys ragdolls, village runs 401 `World.CastShape` queries per step; world
  queries are outside the no-allocation goal and still allocate their callback contexts). Once a
  scene is warm, `World.Step` itself allocates nothing: see below.

## Where the time goes

CPU profile of large_pyramid at 1 worker (`go tool pprof -top`): 45% of samples are in the
scalar lane primitives `mulW` / `addW` / `subW` (simd_scalar.go), and with their callers
(`solveContactsW`, `pushContactsW`, `prepareConstraintW`, `warmStartContactsW`, `crossW`,
`mulMVW`, `dotW`) the wide contact solver is the bulk of the step. The C compiler keeps
`b3FloatW` lanes in registers and inlines the whole kernel; the Go port passes four-float structs
through small functions with float32 conversions (needed against FMA fusion). The planned SIMD
backend (spec: "a `simd`-package backend behind a build tag") or a register-friendlier scalar
`floatW` is where the speed is; both are outside this milestone.

## No allocations in Step

Before this milestone the stack and arena allocators returned a fresh `make` for every request
and several C stack locals escaped to the heap. Now:

- `stackAlloc` / `bump` hand out slices of per-element-type backing buffers owned by the world's
  stack and by each worker's arena, addressed by upstream's byte offsets, so memory is reused
  exactly when upstream reuses its arena; buffers grow only at a new high water mark.
- The step context, the prepare spans, the worker contexts, the parallel-for shared state and
  tasks, the cross seeds and contexts live in `world.stepScratch`; the pair, continuous and sensor
  query contexts in the per-worker task contexts; query contexts deep in functions without worker
  storage (triangle queries, mesh/compound time of impact, `QueryCompound`) come from typed
  `sync.Pool` scratch. Details per site in `LOOSE_ENDS.md`.
- `step_alloc_test.go` asserts `testing.AllocsPerRun(30, step) == 0` for warm stacking
  (large_pyramid, many_pyramids), joints (joint_grid), joints + mesh (spinner), mesh +
  continuous (mesh_drop), kinematic + continuous (washer), compound (village) and a
  sensor/contact/hit event scene, each at 1 and 4 workers (debug-size scenes; skipped under
  `-race`, where `sync.Pool` drops items at random).

Allocations per warm step (debug-size scenes, 20 steps measured after warm-up, workers=1;
village and rain include their scene step functions):

| scene | before allocs/step | before bytes/step | after allocs/step |
|---|---|---|---|
| joint_grid | 15 | 7,584 | 0 |
| large_pyramid | 20 | 282,032 | 0 |
| mesh_drop | 4,809 | 1,982,309 | 0 |
| spinner | 8,055 | 4,269,938 | 0 |
| washer | 539 | 168,984 | 0 |
| sleep (joint create/destroy each step) | 24 | 443,227 | 0 in Step (island/joint containers grow at new peaks) |
| village (with 401 casts per step) | 566 | 174,421 | 0 in Step; ~371 from `World.CastShape` |
| rain (with ragdoll spawning) | 171 | 49,133 | 0 in Step |

Whole-run effect on the first benchmarks measured before the change (b.N = 1, box3d_noassert):

| benchmark | workers | before ms | after ms | before B/run | after B/run | before allocs/run | after allocs/run |
|---|---|---|---|---|---|---|---|
| joint_grid | 1 | 4,051 | 2,987 | 9,404,624 | 590,040 | 4,236 | 18 |
| joint_grid | 4 | 979 | 933 | 10,205,520 | 590,152 | 6,669 | 19 |
| large_pyramid | 1 | 25,981 | 21,405 | 1,309,871,056 | 80,290,056 | 8,171 | 22 |
| large_pyramid | 4 | 11,471 | 6,317 | 1,310,128,384 | 80,290,056 | 8,242 | 22 |
| many_pyramids | 1 | 22,813 | 21,318 | 1,230,294,528 | 152,469,744 | 1,981 | 20 |
| many_pyramids | 4 | 6,395 | 5,342 | 1,230,428,336 | 152,470,304 | 1,988 | 25 |

The remaining bytes per run are the arenas and stack growing to their peak during the first
timed steps (as upstream's `b3ArenaSync` / `b3GrowStack` do) and the bodies' contact containers
growing while the scene settles.
