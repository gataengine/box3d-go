// Go-only test: no upstream counterpart. The spec goal "no allocations in Step": once a scene is
// warm (its containers, arenas and the stack have reached their high water marks), World.Step
// must not allocate, at 1 and at 4 workers.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"testing"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

// stepAllocScene is a warm scene for the allocation test.
type stepAllocScene struct {
	name    string
	create  func(worldId WorldID) (step func(stepIndex int), destroy func())
	warmup  int
	workers []int
}

// createSensorEventScene is a bouncing ball pit with contact, hit and sensor events and a kinematic
// paddle sweeping back and forth through a static sensor, so events begin and end every step.
func createSensorEventScene(worldId WorldID) (func(int), func()) {
	worldId.EnableSleeping(false)

	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, -1.0, 0.0}
		groundId := worldId.CreateBody(&bodyDef)
		box := MakeBoxHull(20.0, 1.0, 20.0)
		shapeDef := DefaultShapeDef()
		shapeDef.EnableContactEvents = true
		shapeDef.EnableHitEvents = true
		groundId.CreateHullShape(&shapeDef, &box.Base)

		sensorBox := MakeOffsetBoxHull(3.0, 3.0, 3.0, Vec3{0.0, 4.0, 0.0})
		sensorDef := DefaultShapeDef()
		sensorDef.IsSensor = true
		sensorDef.EnableSensorEvents = true
		groundId.CreateHullShape(&sensorDef, &sensorBox.Base)
	}

	var paddleId BodyID
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Type = KinematicBody
		bodyDef.Position = Pos{-8.0, 2.0, 0.0}
		bodyDef.LinearVelocity = Vec3{6.0, 0.0, 0.0}
		paddleId = worldId.CreateBody(&bodyDef)
		box := MakeBoxHull(0.5, 2.0, 6.0)
		shapeDef := DefaultShapeDef()
		shapeDef.EnableSensorEvents = true
		shapeDef.EnableContactEvents = true
		shapeDef.EnableHitEvents = true
		paddleId.CreateHullShape(&shapeDef, &box.Base)
	}

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	shapeDef := DefaultShapeDef()
	shapeDef.EnableSensorEvents = true
	shapeDef.EnableContactEvents = true
	shapeDef.EnableHitEvents = true
	shapeDef.BaseMaterial.Restitution = 0.9
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.4}
	cube := MakeCubeHull(0.35)
	for i := range 10 {
		for j := range 10 {
			bodyDef.Position = Pos{-6.0 + 1.3*float32(i), 2.0 + 0.3*float32((i+j)%5), -6.0 + 1.3*float32(j)}
			bodyId := worldId.CreateBody(&bodyDef)
			if (i+j)&1 == 0 {
				bodyId.CreateSphereShape(&shapeDef, &sphere)
			} else {
				bodyId.CreateHullShape(&shapeDef, &cube.Base)
			}
		}
	}

	step := func(stepIndex int) {
		// Turn the paddle around every 2.5 seconds
		if stepIndex%150 == 0 {
			v := paddleId.GetLinearVelocity()
			paddleId.SetLinearVelocity(Vec3{-v.X, 0.0, 0.0})
		}

		// Read the events as an application would
		contactEvents := worldId.GetContactEvents()
		sensorEvents := worldId.GetSensorEvents()
		_ = contactEvents.BeginCount + contactEvents.EndCount + contactEvents.HitCount
		_ = sensorEvents.BeginCount + sensorEvents.EndCount
	}
	return step, func() {}
}

// benchmarkAllocScene adapts a registered benchmark (at its debug size) to the allocation test.
// The benchmark's own step function is skipped when it builds bodies or casts (rain, village):
// only World.Step is measured.
func benchmarkAllocScene(name string, useStepFcn bool) func(WorldID) (func(int), func()) {
	return func(worldId WorldID) (func(int), func()) {
		benchmark := findBenchmarkScene(name)
		benchmark.createFcn(worldId)
		step := func(stepIndex int) {
			if useStepFcn && benchmark.stepFcn != nil {
				benchmark.stepFcn(worldId, stepIndex)
			}
		}
		destroy := func() {
			if benchmark.destroyFcn != nil {
				benchmark.destroyFcn()
			}
		}
		return step, destroy
	}
}

var _ = Describe("StepAllocations", func() {
	scenes := []stepAllocScene{
		{"stacking (large_pyramid)", benchmarkAllocScene("large_pyramid", true), 120, []int{1, 4}},
		{"stacking (many_pyramids)", benchmarkAllocScene("many_pyramids", true), 120, []int{1, 4}},
		{"joints (joint_grid)", benchmarkAllocScene("joint_grid", true), 60, []int{1, 4}},
		{"joints and mesh (spinner)", benchmarkAllocScene("spinner", true), 600, []int{1, 4}},
		{"mesh and continuous (mesh_drop)", benchmarkAllocScene("mesh_drop", true), 400, []int{1, 4}},
		{"kinematic and continuous (washer)", benchmarkAllocScene("washer", true), 300, []int{1, 4}},
		{"compound (village)", benchmarkAllocScene("village", false), 300, []int{1, 4}},
		{"sensors and events", createSensorEventScene, 400, []int{1, 4}},
	}

	for _, scene := range scenes {
		for _, workerCount := range scene.workers {
			It(fmt.Sprintf("%s at %d workers", scene.name, workerCount), func() {
				if raceEnabled {
					Skip("the race detector allocates and sync.Pool drops items at random under -race")
				}

				saved := benchmarkDebug
				benchmarkDebug = true
				defer func() { benchmarkDebug = saved }()

				worldDef := DefaultWorldDef()
				worldDef.WorkerCount = uint32(workerCount)
				worldId := CreateWorld(&worldDef)
				defer worldId.Destroy()

				sceneStep, destroy := scene.create(worldId)
				defer destroy()

				timeStep := float32(1.0 / 60.0)
				stepIndex := 0
				step := func() {
					sceneStep(stepIndex)
					worldId.Step(timeStep, 4)
					stepIndex += 1
				}

				for range scene.warmup {
					step()
				}

				allocs := testing.AllocsPerRun(30, step)
				Expect(allocs).To(BeZero(), "allocations per step after %d warm steps", scene.warmup)
			})
		}
	}
})
