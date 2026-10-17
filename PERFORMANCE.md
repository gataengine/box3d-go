# Performance

Spec scope: the only performance goal of the port is **no allocations in `World.Step`**. Speed
beyond that was a non-goal for the port itself (scalar lanes first); the opt-in SIMD backend
(`GOEXPERIMENT=simd`, `-tags box3d_simd`, see README.md) followed, measured in
[SIMD backend](#simd-backend) against upstream C with its SIMD on. This file records where the
port stands against upstream so later work has a baseline. The Environment, Methodology and
Results sections below are the default build (scalar lanes) against C with SIMD off.

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

## SIMD backend

Upstream C with its SIMD on against the Go SIMD build, on the Mac (NEON, width 4) and on
gata-win (AVX2, width 8, and forced width 4).

### Environment

| | |
|---|---|
| Mac | as above (M1 Max, macOS 15.7.4, go1.27.1); C built with `-DBOX3D_DISABLE_SIMD=OFF` (NEON). Not idle: background desktop load of about 4 cores (load average 25-95) during the runs, see below |
| gata-win | AMD Ryzen 9 5900X (12 cores / 24 threads, AVX2, no AVX-512), 64 GB, Windows 11 Pro 10.0.22621, go1.27.1 windows/amd64, otherwise idle (benchmarks under xcheck's host lock) |
| C on gata-win | upstream `efca0c1` (a `git archive` of the submodule), TDM-GCC 10.3.0 (tdm64-1), no CMake on the host, so compiled with the flags CMake would pass: `gcc -O3 -DNDEBUG -std=gnu17 -ffp-contract=off -D_WIN32_WINNT=0x0A00`, `src/*.c`, `shared/*.c` with `BOX3D_INTERNAL_BENCHMARKS` (static linkage, as CMake's static build), `benchmark/main.c`. Default SIMD settings: SSE2 and the AVX2 kernels (`wide8.c` via `B3_AVX2_BEGIN`), runtime width 8. `_WIN32_WINNT` makes `CreateSemaphoreExW` visible to TDM's headers (without it timer.c gets an implicit 32-bit declaration) |
| Upstream CSVs | `amd7950x_avx2` (Ryzen 9 7950X, AVX2) and `m2air_neon` (M2 Air, NEON): other hardware, for shape only |

### Methodology

- C on the Mac: `cmake -S third_party/box3d -B <scratch>/box3d-bench-simd -DCMAKE_BUILD_TYPE=Release
  -DBOX3D_BENCHMARKS=ON -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF -DBOX3D_DISABLE_SIMD=OFF
  -DBOX3D_VALIDATE=OFF` (upstream's CMakeLists adds `-ffp-contract=off` for clang and gcc itself),
  then `benchmark -t=8 -w=N -r=2` for N = 1, 4, 8. On gata-win the same runs with `-simd=8`
  (the native width) and `-simd=4` (needs the static build).
- Go: `BOX3D_BENCH_WORKERS=1,4,8 GOEXPERIMENT=simd go test -tags box3d_simd,box3d_noassert
  -run '^$' -bench . -benchtime 1x -count 2 .`, on gata-win through `tools/xcheck` at the
  native width 8 and with `BOX3D_BENCH_WIDTH=4` (bench_test.go: `SetSIMDWidth(4)` for the whole
  benchmark, restored afterwards; the `width` metric of each line confirms the width). The Go
  default build on gata-win is the same command without `GOEXPERIMENT` and `box3d_simd`.
- Figures are the minimum over the runs, total ms for the timed steps of one run, as in
  [Methodology](#methodology). On the Mac the C set was run before and again after the Go set
  (4 runs per cell, minimum taken) because of the background load; "Go default" on the Mac is
  the [Results](#results) table (a spot check today: convex_pile 36,791, large_pyramid 22,249,
  rain 5,196 ms at 1 worker, within 4% of it).
- Bodies and contacts at the end of each run are the same as in the default build (and C).

### Mac (M1 Max), width 4

| benchmark | workers | Go SIMD ms | C SIMD (NEON, this Mac) ms | Go SIMD / C SIMD | Go default ms | Go default / Go SIMD | C scalar ms | upstream m2air neon ms |
|---|---|---|---|---|---|---|---|---|
| convex_pile | 1 | 11,995.3 | 4,997.6 | 2.40 | 36,656.0 | 3.06 | 5,994.8 | 15,032.6 |
| convex_pile | 4 | 3,552.4 | 1,813.1 | 1.96 | 11,006.7 | 3.10 | 1,925.0 | 4,296.5 |
| convex_pile | 8 | 2,842.4 | 1,396.0 | 2.04 | 8,321.3 | 2.93 | 1,536.5 | — |
| joint_grid | 1 | 2,860.0 | 889.9 | 3.21 | 2,987.3 | 1.04 | 903.6 | 1,089.8 |
| joint_grid | 4 | 823.1 | 296.0 | 2.78 | 932.7 | 1.13 | 275.5 | 309.3 |
| joint_grid | 8 | 592.7 | 213.7 | 2.77 | 685.2 | 1.16 | 232.2 | — |
| junkyard | 1 | 28,291.0 | 10,492.0 | 2.70 | 80,771.4 | 2.86 | 12,338.9 | 13,501.4 |
| junkyard | 4 | 8,303.4 | 4,446.1 | 1.87 | 22,150.9 | 2.67 | 4,241.8 | 4,253.7 |
| junkyard | 8 | 6,260.7 | 3,010.4 | 2.08 | 15,505.8 | 2.48 | 3,432.4 | — |
| large_pyramid | 1 | 3,240.1 | 1,232.7 | 2.63 | 21,405.3 | 6.61 | 1,932.1 | 1,810.7 |
| large_pyramid | 4 | 1,095.1 | 494.6 | 2.21 | 6,316.7 | 5.77 | 668.4 | 563.5 |
| large_pyramid | 8 | 872.2 | 366.2 | 2.38 | 4,298.2 | 4.93 | 557.0 | — |
| large_world | 1 | 19.2 | 4.3 | 4.49 | 61.6 | 3.20 | 5.9 | 9.0 |
| large_world | 4 | 20.9 | 9.9 | 2.12 | 76.6 | 3.66 | 14.7 | 15.0 |
| large_world | 8 | 27.3 | 19.3 | 1.42 | 84.2 | 3.08 | 17.6 | — |
| many_pyramids | 1 | 4,069.6 | 1,198.8 | 3.39 | 21,317.8 | 5.24 | 1,855.1 | 1,917.4 |
| many_pyramids | 4 | 988.2 | 425.1 | 2.32 | 5,342.2 | 5.41 | 690.3 | 621.3 |
| many_pyramids | 8 | 679.4 | 396.0 | 1.72 | 3,636.9 | 5.35 | 579.7 | — |
| mesh_drop | 1 | 2,671.9 | 1,349.8 | 1.98 | 9,844.9 | 3.68 | 1,582.8 | — |
| mesh_drop | 4 | 894.2 | 464.9 | 1.92 | 2,840.3 | 3.18 | 577.2 | — |
| mesh_drop | 8 | 664.6 | 343.8 | 1.93 | 1,775.1 | 2.67 | 464.3 | — |
| rain | 1 | 2,863.7 | 1,649.4 | 1.74 | 4,843.6 | 1.69 | 1,721.3 | 1,954.8 |
| rain | 4 | 1,202.9 | 721.7 | 1.67 | 1,781.6 | 1.48 | 704.7 | 626.2 |
| rain | 8 | 1,076.4 | 564.6 | 1.91 | 1,503.5 | 1.40 | 716.5 | — |
| sleep | 1 | 1,637.8 | 589.6 | 2.78 | 12,544.0 | 7.66 | 980.5 | — |
| sleep | 4 | 701.6 | 317.1 | 2.21 | 3,582.8 | 5.11 | 464.4 | — |
| sleep | 8 | 639.7 | 299.1 | 2.14 | 2,702.8 | 4.23 | 422.0 | — |
| spinner | 1 | 6,290.3 | 2,479.1 | 2.54 | 20,125.0 | 3.20 | 3,149.3 | — |
| spinner | 4 | 3,118.9 | 1,427.2 | 2.19 | 8,227.6 | 2.64 | 1,665.0 | — |
| spinner | 8 | 2,863.3 | 1,247.2 | 2.30 | 6,156.7 | 2.15 | 1,607.6 | — |
| trees100 | 1 | 232.9 | 98.3 | 2.37 | 689.1 | 2.96 | 125.2 | 186.1 |
| trees100 | 4 | 138.9 | 75.2 | 1.85 | 451.0 | 3.25 | 88.6 | 91.2 |
| trees100 | 8 | 174.2 | 78.0 | 2.23 | 474.7 | 2.73 | 93.0 | — |
| trees25 | 1 | 740.7 | 347.2 | 2.13 | 1,987.1 | 2.68 | 468.9 | 609.9 |
| trees25 | 4 | 342.2 | 171.3 | 2.00 | 938.1 | 2.74 | 205.7 | 236.2 |
| trees25 | 8 | 299.7 | 154.1 | 1.95 | 889.9 | 2.97 | 180.0 | — |
| trees50 | 1 | 362.6 | 164.4 | 2.20 | 1,108.6 | 3.06 | 201.6 | 288.3 |
| trees50 | 4 | 191.7 | 102.0 | 1.88 | 618.4 | 3.23 | 113.3 | 156.9 |
| trees50 | 8 | 194.8 | 95.9 | 2.03 | 534.5 | 2.74 | 115.5 | — |
| village | 1 | 5,148.6 | 1,930.0 | 2.67 | 21,835.4 | 4.24 | 3,143.3 | — |
| village | 4 | 2,290.5 | 1,028.3 | 2.23 | 6,760.0 | 2.95 | 1,220.0 | — |
| village | 8 | 1,928.3 | 882.7 | 2.18 | 5,613.7 | 2.91 | 1,071.5 | — |
| washer | 1 | 37,474.3 | 16,107.3 | 2.33 | 130,195.4 | 3.47 | 19,810.8 | 21,065.2 |
| washer | 4 | 12,142.2 | 6,907.1 | 1.76 | 38,227.4 | 3.15 | 7,563.5 | 7,675.2 |
| washer | 8 | 9,429.2 | 4,383.1 | 2.15 | 29,613.4 | 3.14 | 5,976.1 | — |

### gata-win (Ryzen 9 5900X), width 8 and width 4

| benchmark | workers | Go SIMD w8 ms | C AVX2 w8 ms | Go / C (w8) | Go SIMD w4 ms | C w4 ms | Go / C (w4) | Go default ms | Go default / Go w8 | upstream amd7950x avx2 ms |
|---|---|---|---|---|---|---|---|---|---|---|
| convex_pile | 1 | 12,197.0 | 4,510.4 | 2.70 | 11,040.2 | 4,689.0 | 2.35 | 42,839.5 | 3.51 | 5,093.0 |
| convex_pile | 4 | 3,493.1 | 1,360.3 | 2.57 | 3,178.3 | 1,416.6 | 2.24 | 11,712.8 | 3.35 | 1,439.3 |
| convex_pile | 8 | 2,598.7 | 1,103.5 | 2.36 | 2,486.7 | 1,171.7 | 2.12 | 6,933.4 | 2.67 | 846.3 |
| joint_grid | 1 | 1,536.1 | 1,032.9 | 1.49 | 1,516.9 | 1,026.7 | 1.48 | 1,526.8 | 0.99 | 1,332.9 |
| joint_grid | 4 | 455.7 | 284.6 | 1.60 | 443.2 | 284.1 | 1.56 | 451.4 | 0.99 | 352.2 |
| joint_grid | 8 | 410.9 | 219.0 | 1.88 | 408.8 | 216.5 | 1.89 | 405.2 | 0.99 | 185.4 |
| junkyard | 1 | 28,260.6 | 10,706.3 | 2.64 | 28,343.5 | 11,423.4 | 2.48 | 86,628.0 | 3.07 | 11,236.9 |
| junkyard | 4 | 8,334.5 | 3,331.2 | 2.50 | 8,543.4 | 3,554.0 | 2.40 | 25,130.4 | 3.02 | 3,147.6 |
| junkyard | 8 | 5,890.6 | 2,606.2 | 2.26 | 6,229.3 | 2,773.2 | 2.25 | 14,437.5 | 2.45 | 1,823.0 |
| large_pyramid | 1 | 3,836.9 | 892.4 | 4.30 | 4,251.7 | 1,134.9 | 3.75 | 25,410.9 | 6.62 | 1,099.9 |
| large_pyramid | 4 | 1,160.7 | 271.5 | 4.28 | 1,281.2 | 331.2 | 3.87 | 7,296.1 | 6.29 | 302.6 |
| large_pyramid | 8 | 983.2 | 311.9 | 3.15 | 1,020.9 | 349.0 | 2.93 | 4,066.1 | 4.14 | 175.8 |
| large_world | 1 | 16.7 | 4.4 | 3.83 | 18.4 | 5.0 | 3.69 | 67.1 | 4.01 | 5.3 |
| large_world | 4 | 23.5 | 9.1 | 2.58 | 25.0 | 10.0 | 2.48 | 76.4 | 3.26 | 8.8 |
| large_world | 8 | 26.7 | 14.6 | 1.83 | 27.7 | 14.7 | 1.88 | 86.4 | 3.24 | 9.4 |
| many_pyramids | 1 | 3,926.4 | 1,045.3 | 3.76 | 4,265.2 | 1,245.9 | 3.42 | 24,237.6 | 6.17 | 1,252.3 |
| many_pyramids | 4 | 1,155.7 | 297.7 | 3.88 | 1,289.0 | 357.1 | 3.61 | 7,080.4 | 6.13 | 311.5 |
| many_pyramids | 8 | 886.5 | 272.4 | 3.25 | 939.6 | 300.7 | 3.13 | 3,874.2 | 4.37 | 162.0 |
| mesh_drop | 1 | 2,753.3 | 1,233.4 | 2.23 | 2,867.9 | 1,320.9 | 2.17 | 11,070.1 | 4.02 | 1,287.0 |
| mesh_drop | 4 | 870.9 | 369.1 | 2.36 | 915.4 | 418.6 | 2.19 | 3,313.9 | 3.81 | 360.3 |
| mesh_drop | 8 | 683.7 | 309.2 | 2.21 | 706.9 | 334.5 | 2.11 | 1,926.6 | 2.82 | 215.0 |
| rain | 1 | 2,732.7 | 1,640.0 | 1.67 | 2,773.7 | 1,670.4 | 1.66 | 5,006.0 | 1.83 | 1,832.4 |
| rain | 4 | 966.6 | 520.5 | 1.86 | 965.9 | 535.5 | 1.80 | 1,706.2 | 1.77 | 547.2 |
| rain | 8 | 950.4 | 462.8 | 2.05 | 954.8 | 465.7 | 2.05 | 1,409.1 | 1.48 | 325.8 |
| sleep | 1 | 2,101.5 | 497.9 | 4.22 | 2,358.2 | 636.7 | 3.70 | 14,620.4 | 6.96 | 625.7 |
| sleep | 4 | 693.0 | 168.8 | 4.11 | 732.4 | 207.3 | 3.53 | 4,120.4 | 5.95 | 187.9 |
| sleep | 8 | 668.4 | 212.7 | 3.14 | 695.0 | 236.3 | 2.94 | 2,514.6 | 3.76 | 125.6 |
| spinner | 1 | 6,921.5 | 2,155.6 | 3.21 | 7,487.7 | 2,380.4 | 3.15 | 21,793.7 | 3.15 | 2,274.2 |
| spinner | 4 | 2,428.9 | 813.9 | 2.98 | 2,635.0 | 913.6 | 2.88 | 6,923.4 | 2.85 | 782.7 |
| spinner | 8 | 2,500.3 | 1,028.0 | 2.43 | 2,744.9 | 1,156.9 | 2.37 | 5,114.3 | 2.05 | 541.7 |
| trees100 | 1 | 239.6 | 103.5 | 2.31 | 242.1 | 106.7 | 2.27 | 753.2 | 3.14 | 103.8 |
| trees100 | 4 | 130.3 | 54.9 | 2.37 | 127.4 | 56.3 | 2.26 | 456.7 | 3.51 | 53.7 |
| trees100 | 8 | 141.5 | 62.1 | 2.28 | 147.0 | 64.0 | 2.30 | 471.8 | 3.33 | 47.8 |
| trees25 | 1 | 809.5 | 349.7 | 2.31 | 804.4 | 356.1 | 2.26 | 2,223.9 | 2.75 | 357.9 |
| trees25 | 4 | 332.3 | 144.2 | 2.30 | 326.3 | 147.2 | 2.22 | 976.1 | 2.94 | 147.6 |
| trees25 | 8 | 293.7 | 129.7 | 2.26 | 295.5 | 134.8 | 2.19 | 868.2 | 2.96 | 119.3 |
| trees50 | 1 | 387.2 | 170.0 | 2.28 | 387.1 | 173.8 | 2.23 | 1,088.4 | 2.81 | 162.5 |
| trees50 | 4 | 181.6 | 78.0 | 2.33 | 177.4 | 80.8 | 2.20 | 561.8 | 3.09 | 74.3 |
| trees50 | 8 | 178.1 | 78.6 | 2.26 | 182.7 | 81.7 | 2.24 | 551.5 | 3.10 | 63.7 |
| village | 1 | 5,273.2 | 1,878.4 | 2.81 | 5,552.3 | 2,096.5 | 2.65 | 22,878.0 | 4.34 | — |
| village | 4 | 2,065.3 | 764.1 | 2.70 | 2,122.2 | 824.1 | 2.58 | 7,105.0 | 3.44 | — |
| village | 8 | 1,957.2 | 778.5 | 2.51 | 2,057.2 | 850.4 | 2.42 | 4,727.3 | 2.42 | — |
| washer | 1 | 39,432.3 | 14,440.9 | 2.73 | 39,260.0 | 15,568.6 | 2.52 | 139,475.5 | 3.54 | 15,659.7 |
| washer | 4 | 11,625.8 | 4,440.8 | 2.62 | 11,840.8 | 4,799.1 | 2.47 | 40,375.8 | 3.47 | 4,424.7 |
| washer | 8 | 9,104.2 | 3,904.6 | 2.33 | 9,612.1 | 4,254.0 | 2.26 | 24,056.0 | 2.64 | 2,534.4 |

### Reading the tables

- Go SIMD against C SIMD on the same machine: Mac median 2.5x slower at 1 worker (1.7-4.5x),
  2.0x at 4, 2.1x at 8; gata-win at width 8 median 2.7x at 1 worker (1.5-4.3x), 2.6x at 4,
  2.3x at 8, and at width 4 against C width 4 2.5x / 2.4x / 2.3x. Before the backend the gap to C
  on the Mac was 6.4x (median, 1 worker, scalar against scalar), and C itself gains 1.27x
  (median) from NEON over its scalar build there.
- Go SIMD against the Go default build: Mac median 3.2x faster at 1 and 4 workers (1.04-7.7x),
  gata-win 3.5x at 1 worker, 3.4x at 4, 2.8x at 8 (0.99-7.0x). Solver-bound stacking gains most
  (large_pyramid 6.6x on both machines, sleep 7.7x / 7.0x, many_pyramids 5.2x / 6.2x).
- No gain: joint_grid (1.0-1.2x) has no wide kernels; rain gains least (1.4-1.8x, joints and
  ragdoll spawning). The largest remaining gaps to C are the same stacking scenes (large_pyramid,
  many_pyramids, sleep 3.1-4.3x on gata-win), where the body gather/scatter dominates
  ([Where the time goes](#where-the-time-goes)).
- Width 8 against width 4 in Go on gata-win: median 2-4% faster. Width 8 is up to 12% faster
  on large_pyramid, many_pyramids and sleep, about equal on joint_grid, rain and the trees
  scenes, and 4-10% slower on convex_pile. C gains 6-8% from width 8 (median), 19-28% on the
  stacking scenes at 1 and 4 workers. Not re-measured here: on linux (the gata-linux container,
  AMD Renoir) the width 8 work measured large_pyramid at 1 worker as 24.2 ms/step at width 8
  and 26.6 at width 4.
- Mac multi-worker figures are noisy (the two C sets differ by up to 70% in single 4- and
  8-worker cells, median 5%); the 1-worker figures and everything on gata-win are steadier.
- gata-win scales worse from 4 to 8 workers than the upstream 7950X CSVs, in C as in Go
  (large_pyramid C 272 ms at 4 workers, 312 at 8). C shows it too, so it is the machine or the
  toolchain, not the port.

## Where the time goes

Default build, before the SIMD backend: 45% of the large_pyramid samples (1 worker) were in the
scalar lane primitives `mulW` / `addW` / `subW` (simd_scalar.go), four-float structs passed
through small functions with float32 conversions (needed against FMA fusion); with their callers
the wide contact solver was the bulk of the step.

SIMD build, large_pyramid at 1 worker (`go test -tags box3d_simd,box3d_noassert -bench
LargePyramid -benchtime 1x -cpuprofile`, `go tool pprof -top`; one run, about 3.5 s of samples):

| | Mac, width 4 (NEON) | gata-win, width 8 (AVX2) | gata-win, width 4 (SSE) |
|---|---|---|---|
| ms/step | 16.4 | 19.1 | 21.3 |
| body state gather (`gatherBodies*`, cum) | 18.7% (`bodyStateLoadW` 8.2%) | 22.7% | 38.2% (`bodyStateLoadW` 31.9% flat) |
| body state scatter (`scatterBodies*`, cum, of which `storeBodyVelocity*`) | 9.8% (5.3%) | under 1% | 7.2% (6.3%) |
| solve / push / warm start / prepare kernels (cum, gathers included) | 23.9% / 23.9% / 11.5% / 6.6% | 19.1% / 17.2% / 13.3% / 23.0% | 26.3% / 25.2% / 17.3% / 8.6% |
| lane arithmetic, flat (`mulW*`, `addW*`, `subW*`, `crossW*`, `dotW`, `mulMVW*`, `blendW*`, …) | about 13% | about 12% | about 19% |
| `setW8` field gathers in prepare / store impulses (flat) | — | 8.1% | — |
| collide (`collideTask`, cum) | 13.1% | 6.8% | 8.2% |

- The lane arithmetic is no longer where the time goes: it went from 45% to 12-19% of the
  samples, mostly inlined. Moving body state between the `bodyState` arrays and the lanes is now
  the largest single cost.
- Width 4 on amd64 loads each body's state field by field (`bodyStateLoadW`), which takes a third
  of the samples on gata-win; the width 8 path does the same work with a direct 256-bit load and
  store (wide_lanes8_amd64.go), and upstream's SSE2 path loads 128 bits at a time. It is the main
  difference between the two widths in Go on x86.
- Width 8 spends 23% in `prepareConstraintW8`: its manifold and matrix gathers are `setW8` per
  field and, like `gatherBodies8`, `transposeW8` and the other gathers, are not inlined.
- Every width 8 kernel runs behind the `avx8` / `avx8Store` VZEROUPPER wrappers
  (wide_lanes8_amd64.go), which is why `avx8.func1` holds 73% cumulative.

A platform with no gain is reported, not tuned (spec). Follow-up candidates, none of them
implemented: the direct 128-bit body-state load/store of the width 8 path for width 4 (both
arm64 and amd64); inlining or flattening `gatherBodies8`, `transposeW8` and the `setW8`
gathers of the width 8 prepare stage; `minIndexW8` / `symClampW8` (and amd64 `minIndexW` /
`symClampW`) do not inline either, but are off the solver's hot path. joint_grid gains nothing
(the joint solver has no wide kernels, upstream neither) and is reported as such.

## No allocations in Step

Before milestone 6 the stack and arena allocators returned a fresh `make` for every request
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
