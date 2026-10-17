// Port of benchmark/main.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"os"
	"runtime"
	"strconv"
	"strings"
	"testing"
)

// PORT: benchmark/main.c is a command line program; here every registered benchmark is a
// testing.B function with one sub-benchmark per worker count (BOX3D_BENCH_WORKERS, a comma
// separated list, default 1,2,4,8 as in upstream's CSVs, clamped to MaxWorkers). One b.N
// iteration is one upstream run: create the world with the benchmark's capacity, call the
// benchmark's step function with 0, take one untimed step, then time steps 1 .. totalStepCount-1
// exactly as main.c times them. ns/op is therefore comparable with the ms column of upstream's
// CSVs, which hold the minimum over 4 runs (use -count and take the minimum). Scenes are always
// built with upstream's release (NDEBUG) sizes. Continuous collision is on (main.c's default).
// The SAT benchmark (BOX3D_INTERNAL_BENCHMARKS), Tracy, -nc, profile recording (-s) and the
// commented-out smash and tumbler entries are not ported. Run the benchmarks with
// -tags box3d_noassert to match upstream's release build, which compiles B3_ASSERT out.
// BOX3D_BENCH_WIDTH (4 or 8, default: the native width) forces the SIMD width with SetSIMDWidth
// for the whole benchmark, as main.c's -simd=<4|8> does (static builds only); a width the CPU or
// build cannot run fails the benchmark rather than measure another one.

// benchmarkScene is Benchmark.
type benchmarkScene struct {
	name           string
	capacityFcn    func(capacity *Capacity)
	createFcn      func(worldId WorldID)
	destroyFcn     func()
	stepFcn        func(worldId WorldID, stepCount int)
	totalStepCount int
}

// benchmarkScenes is the benchmarks[] table of main.c.
var benchmarkScenes = []benchmarkScene{
	{"convex_pile", getConvexPileCapacity, createConvexPile, nil, nil, 500},
	{"joint_grid", nil, createJointGrid, nil, nil, 100},
	{"junkyard", getJunkyardCapacity, createJunkyard, nil, stepJunkyard, 500},
	{"large_pyramid", nil, createLargePyramid, nil, nil, 200},
	{"large_world", getLargeWorldCapacity, createLargeWorld, nil, stepLargeWorld, 500},
	{"many_pyramids", nil, createManyPyramids, nil, nil, 100},
	{"mesh_drop", nil, createMeshDropBenchmark, destroyMeshDropBenchmark, nil, 170},
	{"rain", getRainCapacity, createRain, destroyRain, stepRain, 400},
	{"sleep", getSleepCapacity, createSleep, nil, stepSleep, 300},
	{"spinner", getSpinnerCapacity, createSpinner, destroySpinner, nil, 800},
	{"trees100", nil, createTrees100, destroyTrees, nil, 500},
	{"trees50", nil, createTrees50, destroyTrees, nil, 500},
	{"trees25", nil, createTrees25, destroyTrees, nil, 500},
	{"village", getVillageCapacity, createVillage, destroyVillage, stepVillage, villageStepCount},
	{"washer", getWasherCapacity, createWasher, nil, nil, 1000},
	//{ "smash", CreateSmash, NULL, 300 },
	//{ "tumbler", CreateTumbler, NULL, 750 },
}

// benchmarkWorkerCounts parses BOX3D_BENCH_WORKERS.
func benchmarkWorkerCounts(b *testing.B) []int {
	spec := os.Getenv("BOX3D_BENCH_WORKERS")
	if spec == "" {
		spec = "1,2,4,8"
	}
	var counts []int
	for field := range strings.SplitSeq(spec, ",") {
		n, err := strconv.Atoi(strings.TrimSpace(field))
		if err != nil || n < 1 {
			b.Fatalf("BOX3D_BENCH_WORKERS: bad worker count %q", field)
		}
		counts = append(counts, ClampInt(n, 1, MaxWorkers))
	}
	return counts
}

// setBenchmarkSIMDWidth applies BOX3D_BENCH_WIDTH and restores the previous setting when b ends.
func setBenchmarkSIMDWidth(b *testing.B) {
	spec := strings.TrimSpace(os.Getenv("BOX3D_BENCH_WIDTH"))
	if spec == "" {
		return
	}
	width, err := strconv.Atoi(spec)
	if err != nil || (width != 4 && width != 8) {
		b.Fatalf("BOX3D_BENCH_WIDTH: want 4 or 8, got %q", spec)
	}
	saved := int(atomicLoadInt(&simdWidth))
	b.Cleanup(func() { SetSIMDWidth(saved) })
	SetSIMDWidth(width)
	if got := GetSIMDWidth(); got != width {
		b.Fatalf("BOX3D_BENCH_WIDTH=%d: this CPU and build run width %d", width, got)
	}
}

// findBenchmarkScene is FindBenchmark (exact name, as the table holds lower case names).
func findBenchmarkScene(name string) *benchmarkScene {
	for i := range benchmarkScenes {
		if benchmarkScenes[i].name == name {
			return &benchmarkScenes[i]
		}
	}
	return nil
}

// runBenchmarkScene runs one registered benchmark at every requested worker count. The body of
// the b.N loop is one iteration of main.c's run loop.
func runBenchmarkScene(b *testing.B, name string) {
	benchmark := findBenchmarkScene(name)
	if benchmark == nil {
		b.Fatalf("no benchmark named %q", name)
	}

	saved := benchmarkDebug
	benchmarkDebug = false
	b.Cleanup(func() { benchmarkDebug = saved })

	setBenchmarkSIMDWidth(b)

	stepCount := benchmark.totalStepCount

	for _, threadCount := range benchmarkWorkerCounts(b) {
		b.Run(fmt.Sprintf("workers=%d", threadCount), func(b *testing.B) {
			b.ReportAllocs()
			var counters Counters
			for range b.N {
				b.StopTimer()

				worldDef := DefaultWorldDef()
				worldDef.EnableContinuous = true
				worldDef.WorkerCount = uint32(threadCount)

				if benchmark.capacityFcn != nil {
					benchmark.capacityFcn(&worldDef.Capacity)
				}

				worldId := CreateWorld(&worldDef)

				benchmark.createFcn(worldId)

				timeStep := float32(1.0 / 60.0)
				subStepCount := 4

				// Initial step can be expensive and skew benchmark
				if benchmark.stepFcn != nil {
					benchmark.stepFcn(worldId, 0)
				}

				worldId.Step(timeStep, subStepCount)

				// Keep scene construction garbage out of the timed region.
				runtime.GC()
				b.StartTimer()

				for stepIndex := 1; stepIndex < stepCount; stepIndex++ {
					if benchmark.stepFcn != nil {
						benchmark.stepFcn(worldId, stepIndex)
					}

					worldId.Step(timeStep, subStepCount)
				}

				b.StopTimer()

				counters = worldId.GetCounters()

				if benchmark.destroyFcn != nil {
					benchmark.destroyFcn()
				}

				worldId.Destroy()
				b.StartTimer()
			}

			b.ReportMetric(float64(b.Elapsed().Microseconds())/float64(b.N)/float64(stepCount-1), "us/step")
			b.ReportMetric(float64(counters.BodyCount), "bodies")
			b.ReportMetric(float64(counters.ContactCount), "contacts")
			b.ReportMetric(float64(GetSIMDWidth()), "width")
		})
	}
}

func BenchmarkConvexPile(b *testing.B)   { runBenchmarkScene(b, "convex_pile") }
func BenchmarkJointGrid(b *testing.B)    { runBenchmarkScene(b, "joint_grid") }
func BenchmarkJunkyard(b *testing.B)     { runBenchmarkScene(b, "junkyard") }
func BenchmarkLargePyramid(b *testing.B) { runBenchmarkScene(b, "large_pyramid") }
func BenchmarkLargeWorld(b *testing.B)   { runBenchmarkScene(b, "large_world") }
func BenchmarkManyPyramids(b *testing.B) { runBenchmarkScene(b, "many_pyramids") }
func BenchmarkMeshDrop(b *testing.B)     { runBenchmarkScene(b, "mesh_drop") }
func BenchmarkRain(b *testing.B)         { runBenchmarkScene(b, "rain") }
func BenchmarkSleep(b *testing.B)        { runBenchmarkScene(b, "sleep") }
func BenchmarkSpinner(b *testing.B)      { runBenchmarkScene(b, "spinner") }
func BenchmarkTrees100(b *testing.B)     { runBenchmarkScene(b, "trees100") }
func BenchmarkTrees50(b *testing.B)      { runBenchmarkScene(b, "trees50") }
func BenchmarkTrees25(b *testing.B)      { runBenchmarkScene(b, "trees25") }
func BenchmarkVillage(b *testing.B)      { runBenchmarkScene(b, "village") }
func BenchmarkWasher(b *testing.B)       { runBenchmarkScene(b, "washer") }
