// Port of test/test_restitution.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// PORT: RestitutionTest runs every subtest through RUN_MEASUREMENT and fails if any returned
// non-zero. Here each subtest is its own It that prints the same passed/failed line and ensures the
// subtest returned 0. The static helpers of test_restitution.c are prefixed with restitution so they
// cannot clash in the one test package. printf goes to GinkgoWriter.

// TIME_STEP. A float32 variable rather than a constant so the test-side arithmetic rounds per
// operation as in C (Go constant expressions are exact).
var restitutionTimeStep = float32(1.0 / 60.0)

const restitutionSubStepCount = 4 // SUB_STEP_COUNT

// A flat landing converges its Gauss-Seidel coupling in four stage iterations and the default is two,
// so the tests that read the coefficient or the spin of a flat landing pin the count
const restitutionIterations = 4 // RESTITUTION_ITERATIONS

// Impact speed shared by every scenario that measures a coefficient. Well above the default
// restitution threshold so the bounce is always armed.
const restitutionImpactSpeed float32 = 5.0 // IMPACT_SPEED

// restitutionRunMeasurement is RUN_MEASUREMENT.
// Each subtest prints all of its measurements before asserting, and the battery runs every subtest
// so one bad scenario does not hide the rest.
func restitutionRunMeasurement(name string, t func() int) {
	result := t()
	if result != 0 {
		GinkgoWriter.Printf("  subtest failed: %s\n", name)
	} else {
		GinkgoWriter.Printf("  subtest passed: %s\n", name)
	}
	ensure(result == 0)
}

// restitutionMakeWorld is MakeWorld.
func restitutionMakeWorld(gravityY float32) WorldID {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec3{0.0, gravityY, 0.0}
	worldDef.EnableSleep = false
	return CreateWorld(&worldDef)
}

// restitutionMakeWorldIterations is MakeWorldIterations.
func restitutionMakeWorldIterations(gravityY float32, restitutionIterations int) WorldID {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec3{0.0, gravityY, 0.0}
	worldDef.EnableSleep = false
	worldDef.RestitutionIterations = int32(restitutionIterations)
	return CreateWorld(&worldDef)
}

// restitutionMakeGround is MakeGround.
// Ground with its top surface at y = 0
func restitutionMakeGround(worldId WorldID, restitution float32) {
	bodyDef := DefaultBodyDef()
	bodyDef.Position = Pos{0.0, -1.0, 0.0}
	groundId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	shapeDef.BaseMaterial.Friction = 0.0
	shapeDef.BaseMaterial.Restitution = restitution
	box := MakeBoxHull(40.0, 1.0, 40.0)
	groundId.CreateHullShape(&shapeDef, &box.Base)
}

// restitutionMakeBall is MakeBall.
func restitutionMakeBall(worldId WorldID, x, y, velocityY, restitution float32) BodyID {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{x, y, 0.0}
	bodyDef.LinearVelocity = Vec3{0.0, velocityY, 0.0}
	bodyId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	shapeDef.BaseMaterial.Friction = 0.0
	shapeDef.BaseMaterial.Restitution = restitution
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
	bodyId.CreateSphereShape(&shapeDef, &sphere)
	return bodyId
}

// restitutionMeasureHeadOn is MeasureHeadOn.
// Two free spheres closing head on with no gravity. The coefficient is the ratio of relative normal
// speeds at the contact, which is the mass independent definition.
func restitutionMeasureHeadOn(restitution, densityB float32, momentumError *float32) float32 {
	worldId := restitutionMakeWorld(0.0)

	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.BaseMaterial.Friction = 0.0
	shapeDef.BaseMaterial.Restitution = restitution

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{-1.0, 0.0, 0.0}
	bodyDef.LinearVelocity = Vec3{0.5 * restitutionImpactSpeed, 0.0, 0.0}
	idA := worldId.CreateBody(&bodyDef)
	idA.CreateSphereShape(&shapeDef, &sphere)

	bodyDef.Position = Pos{1.0, 0.0, 0.0}
	bodyDef.LinearVelocity = Vec3{-0.5 * restitutionImpactSpeed, 0.0, 0.0}
	idB := worldId.CreateBody(&bodyDef)
	shapeDef.Density = densityB
	idB.CreateSphereShape(&shapeDef, &sphere)

	massA := idA.GetMass()
	massB := idB.GetMass()
	momentum0 := float32(massA*idA.GetLinearVelocity().X) + float32(massB*idB.GetLinearVelocity().X)

	for range 120 {
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)
	}

	vA := idA.GetLinearVelocity().X
	vB := idB.GetLinearVelocity().X
	momentum1 := float32(massA*vA) + float32(massB*vB)

	worldId.Destroy()

	*momentumError = (momentum1 - momentum0) / (massA + massB)
	return (vB - vA) / restitutionImpactSpeed
}

// restitutionMeasureGroundBounce is MeasureGroundBounce.
// Ball driven into static ground with no gravity so the measurement carries no gravity bias. The
// gap shifts where inside the time step the impact lands.
func restitutionMeasureGroundBounce(restitution, gap float32) float32 {
	worldId := restitutionMakeWorld(0.0)
	restitutionMakeGround(worldId, 0.0)

	ballId := restitutionMakeBall(worldId, 0.0, 0.5+gap, -restitutionImpactSpeed, restitution)

	for range 60 {
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)
	}

	speed := ballId.GetLinearVelocity().Y
	worldId.Destroy()
	return speed / restitutionImpactSpeed
}

// restitutionMeasureSupportedBounce is MeasureSupportedBounce.
// Impactor driven onto a column of resting balls pinned against the ground. The support balls are
// dead so only the top contact can bounce. A supported target has infinite effective mass along the
// normal, so the coefficient must not depend on the column height.
func restitutionMeasureSupportedBounce(restitution float32, supportCount int, propagate bool) float32 {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec3Zero
	worldDef.EnableSleep = false
	worldDef.EnableRestitutionPropagation = propagate
	worldDef.RestitutionIterations = restitutionIterations
	worldId := CreateWorld(&worldDef)
	restitutionMakeGround(worldId, 0.0)

	for i := range supportCount {
		restitutionMakeBall(worldId, 0.0, 0.5+float32(1.0*float32(i)), 0.0, 0.0)
	}

	y := 0.5 + float32(1.0*float32(supportCount)) + float32(0.5*restitutionImpactSpeed*restitutionTimeStep)
	ballId := restitutionMakeBall(worldId, 0.0, y, -restitutionImpactSpeed, restitution)

	for range 60 {
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)
	}

	speed := ballId.GetLinearVelocity().Y
	worldId.Destroy()
	return speed / restitutionImpactSpeed
}

// restitutionMeasureFlatBounce is MeasureFlatBounce.
// Flat box landing on all four corners at once with no gravity
func restitutionMeasureFlatBounce(restitution float32, subStepCount int, spin *float32) float32 {
	worldId := restitutionMakeWorldIterations(0.0, restitutionIterations)
	restitutionMakeGround(worldId, 0.0)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{0.0, 0.25 + float32(0.5*restitutionImpactSpeed*restitutionTimeStep), 0.0}
	bodyDef.LinearVelocity = Vec3{0.0, -restitutionImpactSpeed, 0.0}
	boxId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.BaseMaterial.Friction = 0.0
	shapeDef.BaseMaterial.Restitution = restitution
	box := MakeBoxHull(1.0, 0.25, 1.0)
	boxId.CreateHullShape(&shapeDef, &box.Base)

	for range 60 {
		worldId.Step(restitutionTimeStep, subStepCount)
	}

	speed := boxId.GetLinearVelocity().Y
	*spin = Length(boxId.GetAngularVelocity())
	worldId.Destroy()
	return speed / restitutionImpactSpeed
}

// restitutionMeasureDrop is MeasureDrop.
// Perfectly elastic ball dropped under gravity
func restitutionMeasureDrop(dropHeight float32, apexes []float32, capacity int) int {
	worldId := restitutionMakeWorld(-10.0)
	restitutionMakeGround(worldId, 0.0)

	ballId := restitutionMakeBall(worldId, 0.0, dropHeight, 0.0, 1.0)

	apexCount := 0
	previousSpeed := float32(0.0)

	for i := 0; i < 4000 && apexCount < capacity; i++ {
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)

		speed := ballId.GetLinearVelocity().Y
		if previousSpeed > 0.0 && speed <= 0.0 {
			apexes[apexCount] = ballId.GetPosition().Y
			apexCount += 1
		}
		previousSpeed = speed
	}

	worldId.Destroy()
	return apexCount
}

// restitutionHeadOnTest is HeadOnTest.
func restitutionHeadOnTest() int {
	restitutions := []float32{0.0, 0.25, 0.5, 0.75, 1.0}
	densities := []float32{1.0, 10.0, 100.0}
	const tolerance float32 = 0.02

	failed := 0

	for i := 0; i < arrayCount(densities); i++ {
		worstError := float32(0.0)
		worstMomentum := float32(0.0)

		for j := 0; j < arrayCount(restitutions); j++ {
			momentumError := float32(0.0)
			measured := restitutionMeasureHeadOn(restitutions[j], densities[i], &momentumError)

			error := AbsFloat(measured - restitutions[j])
			if error > worstError {
				worstError = error
			}

			if AbsFloat(momentumError) > AbsFloat(worstMomentum) {
				worstMomentum = momentumError
			}

			if error > tolerance {
				GinkgoWriter.Printf("    head on density %.0f e %.2f -> %.4f\n", densities[i], restitutions[j], measured)
				failed = 1
			}
		}

		GinkgoWriter.Printf("    head on density %6.1f worst error %.4f momentum drift %.2e\n", densities[i], worstError, worstMomentum)

		if AbsFloat(worstMomentum) > 1.0e-4 {
			failed = 1
		}
	}

	return failed
}

// restitutionPhaseTest is PhaseTest.
func restitutionPhaseTest() int {
	restitutions := []float32{0.25, 0.5, 0.9}
	const sampleCount = 16
	const tolerance float32 = 0.05

	failed := 0

	for j := 0; j < arrayCount(restitutions); j++ {
		minimum := fltMax
		maximum := -fltMax
		sum := float32(0.0)

		for i := range sampleCount {
			// Sweep the gap across one step of travel so the impact lands at every phase
			gap := (float32(i) / float32(sampleCount)) * restitutionImpactSpeed * restitutionTimeStep
			measured := restitutionMeasureGroundBounce(restitutions[j], gap)
			minimum = MinFloat(minimum, measured)
			maximum = MaxFloat(maximum, measured)
			sum += measured
		}

		mean := sum / sampleCount
		spread := maximum - minimum
		GinkgoWriter.Printf("    phase e %.2f -> mean %.4f spread %.4f [%.4f, %.4f]\n", restitutions[j], mean, spread, minimum, maximum)

		if spread > tolerance || AbsFloat(mean-restitutions[j]) > tolerance {
			failed = 1
		}
	}

	return failed
}

// restitutionSupportedTest is SupportedTest.
// By default the restitution stage sweeps only the contacts that carry restitution, so the ball
// pushes the column down and the ground under it gets no say until the next step: e = 0.9 gives
// about 0.52 on one support. With propagation enabled the stage sweeps every contact, the ground
// contact supplies the reaction within the stage and e = 0.9 gives 0.85 on one support and 0.72 on
// three at the pinned four iterations, decaying with depth because the column needs an iteration per
// level. Propagation costs a normal pass over every contact per iteration, hence the option. Both
// modes are gated: never above the free coefficient, which would be energy from nowhere, and never
// collapsed.
func restitutionSupportedTest() int {
	restitutions := []float32{0.5, 0.9}

	failed := 0

	for propagate := 0; propagate <= 1; propagate++ {
		floor := float32(0.5)
		if propagate != 0 {
			floor = 0.75
		}

		for j := 0; j < arrayCount(restitutions); j++ {
			for n := 0; n <= 3; n++ {
				measured := restitutionMeasureSupportedBounce(restitutions[j], n, propagate == 1)
				GinkgoWriter.Printf("    supported e %.2f supports %d propagation %d -> %.4f\n", restitutions[j], n, propagate, measured)

				if measured > restitutions[j]+0.01 || measured < floor*restitutions[j] {
					failed = 1
				}
			}
		}
	}

	return failed
}

// restitutionFlatLandingTest is FlatLandingTest.
// A symmetric four point landing. The stage solves the points in sequence, so a single sweep leaves
// the box spinning: the first impulse tilts it and the next point sees a larger closing speed. Four
// stage iterations converge the coupling and leave a small residual. The tolerance admits that
// residual and rejects the single sweep behavior.
func restitutionFlatLandingTest() int {
	restitutions := []float32{0.5, 0.9}
	subStepCounts := []int{4, 8}
	const speedTolerance float32 = 0.01
	const spinTolerance float32 = 0.03

	failed := 0

	for j := 0; j < arrayCount(restitutions); j++ {
		for k := 0; k < arrayCount(subStepCounts); k++ {
			spin := float32(0.0)
			measured := restitutionMeasureFlatBounce(restitutions[j], subStepCounts[k], &spin)
			GinkgoWriter.Printf("    flat e %.2f substeps %d -> %.4f spin %.4f\n", restitutions[j], subStepCounts[k], measured, spin)

			// Only the shipping sub step count is a gate. The wider count is reported so a
			// convergence problem can be told apart from a formulation problem.
			if subStepCounts[k] != restitutionSubStepCount {
				continue
			}

			if AbsFloat(measured-restitutions[j]) > speedTolerance || spin > spinTolerance {
				failed = 1
			}
		}
	}

	return failed
}

// restitutionSpinTest is SpinTest.
// A flat box carrying spin about a horizontal axis. All four corners stay in contact and the normal
// constraints are linear in the body velocity, so perfect restitution reverses linear and angular
// velocity. Four stage iterations bring both within a few thousandths.
func restitutionSpinTest() int {
	spins := []float32{0.0, 0.5, 1.0, 2.0}

	failed := 0

	for j := 0; j < arrayCount(spins); j++ {
		worldId := restitutionMakeWorldIterations(0.0, restitutionIterations)
		restitutionMakeGround(worldId, 0.0)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, 0.25 + float32(0.5*restitutionImpactSpeed*restitutionTimeStep), 0.0}
		bodyDef.LinearVelocity = Vec3{0.0, -restitutionImpactSpeed, 0.0}
		bodyDef.AngularVelocity = Vec3{0.0, 0.0, spins[j]}
		boxId := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		shapeDef.BaseMaterial.Friction = 0.0
		shapeDef.BaseMaterial.Restitution = 1.0
		box := MakeBoxHull(1.0, 0.25, 1.0)
		boxId.CreateHullShape(&shapeDef, &box.Base)

		for range 60 {
			worldId.Step(restitutionTimeStep, restitutionSubStepCount)
		}

		speed := boxId.GetLinearVelocity().Y
		spin := boxId.GetAngularVelocity().Z
		worldId.Destroy()

		GinkgoWriter.Printf("    spin in %+.2f -> vy %+.4f (want %+.4f)  wz %+.4f (want %+.4f)\n", spins[j], speed, restitutionImpactSpeed, spin,
			-spins[j])

		if AbsFloat(speed-restitutionImpactSpeed) > 0.05 || AbsFloat(spin+spins[j]) > 0.05 {
			failed = 1
		}
	}

	return failed
}

// restitutionDropTest is DropTest.
// Perfectly elastic ball under gravity. Only heights where continuous collision engages are used.
// Continuous collision lands the ball on the surface, so the bounce is armed from the true impact
// speed and the apex holds. Slower drops resolve the impact inside the overlap and the penetration
// recovery adds height, by design. The 10 m case is the long horizon read.
func restitutionDropTest() int {
	heights := []float32{40.0, 20.0, 10.0}

	failed := 0

	for j := 0; j < arrayCount(heights); j++ {
		apexes := make([]float32, 6)
		apexCount := restitutionMeasureDrop(heights[j], apexes, arrayCount(apexes))

		GinkgoWriter.Printf("    drop %5.1f ->", heights[j])
		for i := range apexCount {
			GinkgoWriter.Printf(" %8.3f", apexes[i])
		}
		GinkgoWriter.Printf("\n")

		if apexCount < arrayCount(apexes) {
			failed = 1
			continue
		}

		highest := apexes[0]
		for i := 1; i < apexCount; i++ {
			highest = MaxFloat(highest, apexes[i])
		}

		// A later apex above the first, or a first apex above the drop, is energy from nowhere
		if highest > 1.001*apexes[0] || apexes[0] > 1.001*heights[j] || apexes[apexCount-1] < 0.8*apexes[0] {
			failed = 1
		}
	}

	return failed
}

// restitutionSingleBoxTest is SingleBoxTest.
// A cube dropped flat with an aggressive continuous safety factor so it lands square on the surface.
// Perfectly elastic and frictionless, so it should come back to the drop height with no rotation. A
// flat landing solves four coplanar points in sequence and one sweep over-delivers: the first impulse
// tilts the cube, the next point sees a larger closing speed and the sum overshoots the rigid body
// answer, leaving spin that tilts the cube for the next landing. Four stage iterations converge the
// coupling. The bounds below reject the single sweep behavior and any apex above the drop.
func restitutionSingleBoxTest() int {
	worldId := restitutionMakeWorldIterations(-10.0, restitutionIterations)
	restitutionMakeGround(worldId, 0.0)

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.BaseMaterial.Friction = 0.0
	shapeDef.BaseMaterial.Restitution = 1.0
	box := MakeCubeHull(0.5)

	const dropHeight float32 = 10.0

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{0.0, dropHeight, 0.0}
	bodyDef.SafetyFactor = 0.01
	boxId := worldId.CreateBody(&bodyDef)
	boxId.CreateHullShape(&shapeDef, &box.Base)

	firstSpin := float32(0.0)
	var apexes [4]float32
	var spins [4]float32
	apexCount := 0
	previousSpeed := float32(0.0)

	// A nearly elastic bounce from 10 m takes about 165 steps, so budget for four of them
	for i := 0; i < 1200 && apexCount < arrayCount(apexes[:]); i++ {
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)

		speed := boxId.GetLinearVelocity().Y

		if apexCount == 0 && previousSpeed <= 0.0 && speed > 0.0 {
			firstSpin = Length(boxId.GetAngularVelocity())
		}

		if previousSpeed > 0.0 && speed <= 0.0 {
			apexes[apexCount] = boxId.GetPosition().Y
			spins[apexCount] = Length(boxId.GetAngularVelocity())
			apexCount += 1
		}
		previousSpeed = speed
	}

	worldId.Destroy()

	GinkgoWriter.Printf("    single box first bounce spin %.4f apexes", firstSpin)
	for i := range apexCount {
		GinkgoWriter.Printf(" %7.3f (spin %5.2f)", apexes[i], spins[i])
	}
	GinkgoWriter.Printf("\n")

	ensure(apexCount == arrayCount(apexes[:]))

	failed := 0

	if firstSpin > 0.01 {
		failed = 1
	}

	for i := range apexCount {
		if apexes[i] < 0.95*dropHeight || apexes[i] > 1.001*dropHeight {
			failed = 1
		}
	}

	return failed
}

// restitutionThresholdTest is ThresholdTest.
func restitutionThresholdTest() int {
	worldDef := DefaultWorldDef()
	threshold := worldDef.RestitutionThreshold

	scales := []float32{0.5, 0.9, 1.5, 4.0}

	failed := 0

	for j := 0; j < arrayCount(scales); j++ {
		speed := scales[j] * threshold

		worldId := restitutionMakeWorld(0.0)
		restitutionMakeGround(worldId, 0.0)
		ballId := restitutionMakeBall(worldId, 0.0, 0.5+float32(0.5*speed*restitutionTimeStep), -speed, 1.0)

		for range 60 {
			worldId.Step(restitutionTimeStep, restitutionSubStepCount)
		}

		ratio := ballId.GetLinearVelocity().Y / speed
		worldId.Destroy()

		GinkgoWriter.Printf("    threshold %.2fx -> %.4f\n", scales[j], ratio)

		if scales[j] < 1.0 && ratio > 0.05 {
			failed = 1
		}

		if scales[j] > 1.0 && ratio < 0.9 {
			failed = 1
		}
	}

	return failed
}

// restitutionRestingTest is RestingTest.
// High restitution must not wake a settled stack back up
func restitutionRestingTest() int {
	worldId := restitutionMakeWorld(-10.0)
	restitutionMakeGround(worldId, 0.9)

	const boxCount = 10
	var boxIds [10]BodyID

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.BaseMaterial.Friction = 0.6
	shapeDef.BaseMaterial.Restitution = 0.9
	box := MakeCubeHull(0.5)

	for i := range boxCount {
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, 0.5 + float32(1.0*float32(i)), 0.0}
		boxIds[i] = worldId.CreateBody(&bodyDef)
		boxIds[i].CreateHullShape(&shapeDef, &box.Base)
	}

	for range 240 {
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)
	}

	settled := boxIds[boxCount-1].GetPosition().Y
	drift := float32(0.0)
	peakSpeed := float32(0.0)

	for range 300 {
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)

		y := boxIds[boxCount-1].GetPosition().Y
		drift = MaxFloat(drift, AbsFloat(y-settled))

		for j := range boxCount {
			peakSpeed = MaxFloat(peakSpeed, Length(boxIds[j].GetLinearVelocity()))
		}
	}

	worldId.Destroy()

	GinkgoWriter.Printf("    resting drift %.5f peak speed %.5f\n", drift, peakSpeed)

	ensure(drift < 0.01)
	ensure(peakSpeed < 0.05)
	return 0
}

// restitutionRunWorkerScene is RunWorkerScene.
func restitutionRunWorkerScene(workerCount int) uint64 {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec3{0.0, -10.0, 0.0}
	worldDef.EnableSleep = false
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)

	restitutionMakeGround(worldId, 0.6)

	for i := range 20 {
		restitutionMakeBall(worldId, -10.0+float32(1.05*float32(i)), 3.0+float32(0.13*float32(i)), 0.0, 0.6)
	}

	for range 200 {
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)
	}

	hash := hashWorldState(getWorldFromId(worldId))
	worldId.Destroy()
	return hash
}

// restitutionMeasureFirstTouchNormalVelocity is MeasureFirstTouchNormalVelocity.
// The manifold point approach speed is published only for contacts that opted into hit events. It
// is not a stale value otherwise, it is zero, so a reader can tell "not measured" from "measured
// zero" by whether the shape enables the events. Restitution no longer reads it at all, so this is
// the only thing keeping the field alive.
func restitutionMeasureFirstTouchNormalVelocity(enableHitEvents bool) float32 {
	worldId := restitutionMakeWorld(0.0)
	restitutionMakeGround(worldId, 0.0)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{0.0, 0.6, 0.0}
	ballId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	shapeDef.BaseMaterial.Friction = 0.0
	shapeDef.BaseMaterial.Restitution = 0.0
	shapeDef.EnableHitEvents = enableHitEvents
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
	ballId.CreateSphereShape(&shapeDef, &sphere)

	normalVelocity := fltMax

	// Hold the closing speed so the sampled value does not depend on which step the pair is created
	for range 20 {
		ballId.SetLinearVelocity(Vec3{0.0, -restitutionImpactSpeed, 0.0})
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)

		var contactData [1]ContactData
		if ballId.GetContactData(contactData[:]) == 1 && contactData[0].ManifoldCount > 0 &&
			contactData[0].Manifolds[0].PointCount > 0 {
			normalVelocity = contactData[0].Manifolds[0].Points[0].NormalVelocity
			break
		}
	}

	worldId.Destroy()
	return normalVelocity
}

// restitutionNormalVelocityTest is NormalVelocityTest.
func restitutionNormalVelocityTest() int {
	quiet := restitutionMeasureFirstTouchNormalVelocity(false)
	published := restitutionMeasureFirstTouchNormalVelocity(true)

	GinkgoWriter.Printf("    normal velocity hit events off %.4f on %.4f (closing at %.4f)\n", quiet, published, -restitutionImpactSpeed)

	failed := 0

	if quiet != 0.0 {
		failed = 1
	}

	if published > -0.8*restitutionImpactSpeed {
		failed = 1
	}

	return failed
}

// restitutionMeasureEnergy is MeasureEnergy.
// Mechanical energy of a body as symplectic Euler conserves it. The integrator updates velocity
// before position, so the quantity that is exactly constant in free flight is not kinetic plus
// potential but that plus half a sub-step of the gravity power, m * dot(v, g) * h / 2. Plain
// kinetic plus potential sampled at step boundaries jumps by about g * v * h at every bounce and
// drifts down by g^2 h^2 / 2 each sub-step, which would hide a real gain or fake one. With this
// measure a contact that never does positive work leaves the energy flat to round-off.
func restitutionMeasureEnergy(bodyId BodyID, worldId WorldID) float32 {
	massData := bodyId.GetMassData()
	v := bodyId.GetLinearVelocity()
	gravity := MulSV(bodyId.GetGravityScale(), worldId.GetGravity())
	h := restitutionTimeStep / restitutionSubStepCount

	// The inertia tensor is in the body frame, so bring the angular velocity back to it
	wLocal := InvRotateVector(bodyId.GetRotation(), bodyId.GetAngularVelocity())

	kinetic := float32(0.5*massData.Mass*Dot(v, v)) + float32(0.5*Dot(wLocal, MulMV(massData.Inertia, wLocal)))
	potential := -massData.Mass * Dot(gravity, ToVec3(bodyId.GetWorldCenter()))
	phase := 0.5 * h * massData.Mass * Dot(v, gravity)

	return kinetic + potential + phase
}

// restitutionOvershootResult is OvershootResult.
type restitutionOvershootResult struct {
	firstApex        float32
	firstEnergyRatio float32
	peakEnergyRatio  float32
	speedRatio       float32
	bounceCount      int
}

// restitutionMeasureOvershoot is MeasureOvershoot.
// Mirrors the Restitution Overshoot sample in samples/sample_issues.cpp: a unit cube dropped flat on
// a floor narrower than itself, perfectly elastic, so the four contact points sit inboard of the box
// corners. A perfectly elastic bounce cannot come back higher than it was dropped from and cannot
// gain energy.
func restitutionMeasureOvershoot(continuous bool) restitutionOvershootResult {
	worldId := restitutionMakeWorldIterations(-10.0, restitutionIterations)

	floorDef := DefaultBodyDef()
	floorDef.Position = Pos{0.0, -0.25, 0.0}
	floorId := worldId.CreateBody(&floorDef)

	floorShape := DefaultShapeDef()
	floor := MakeBoxHull(0.375, 0.25, 0.375)
	floorId.CreateHullShape(&floorShape, &floor.Base)

	const dropHeight float32 = 10.0

	boxDef := DefaultBodyDef()
	boxDef.Type = DynamicBody
	boxDef.Position = Pos{0.0, dropHeight, 0.0}
	if continuous {
		boxDef.SafetyFactor = 0.1
	}
	boxId := worldId.CreateBody(&boxDef)

	boxShape := DefaultShapeDef()
	boxShape.BaseMaterial.Restitution = 1.0
	box := MakeCubeHull(0.5)
	boxId.CreateHullShape(&boxShape, &box.Base)

	var result restitutionOvershootResult

	startEnergy := restitutionMeasureEnergy(boxId, worldId)
	impactSpeed := float32(0.0)
	reboundSpeed := float32(0.0)
	bounceHeight := float32(0.0)
	previousSpeed := float32(0.0)

	for i := 0; i < 600 && result.bounceCount < 2; i++ {
		before := boxId.GetLinearVelocity().Y
		heightBefore := boxId.GetPosition().Y
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)
		speed := boxId.GetLinearVelocity().Y

		if reboundSpeed == 0.0 && before < 0.0 && speed > 0.0 {
			impactSpeed = -before
			reboundSpeed = speed
			bounceHeight = heightBefore
		}

		if reboundSpeed == 0.0 {
			continue
		}

		// Every step, not just at the apexes, because that is what the sample checks and the impact
		// step itself is where any gain appears
		ratio := restitutionMeasureEnergy(boxId, worldId) / startEnergy
		if ratio > result.peakEnergyRatio {
			result.peakEnergyRatio = ratio
		}

		if previousSpeed > 0.0 && speed <= 0.0 {
			if result.bounceCount == 0 {
				result.firstApex = boxId.GetPosition().Y
				result.firstEnergyRatio = ratio
			}

			result.bounceCount += 1
		}

		previousSpeed = speed
	}

	worldId.Destroy()

	result.speedRatio = reboundSpeed / impactSpeed

	continuousFlag := 0
	if continuous {
		continuousFlag = 1
	}
	GinkgoWriter.Printf("    overshoot continuous %d apex %.4f of %.4f at y %.4f, speed ratio %.4f, energy %.4f then %.4f\n",
		continuousFlag, result.firstApex, dropHeight, bounceHeight, result.speedRatio, result.firstEnergyRatio,
		result.peakEnergyRatio)

	return result
}

// restitutionOvershootTest is OvershootTest.
// Two runs, because two unrelated effects were tangled together here.
//
// The impulse must never return more speed than it received. That is the restitution invariant and it
// is checked in both runs; the unconverged four point solve failed it by 7.9 percent.
//
// Everything else depends on whether continuous collision engages. Without it the cube moves 0.23 m
// in the step before contact, just under the 0.25 m trigger, so the first manifold appears with the
// cube already 0.10 m deep. It bounces from down there and climbs back through that depth for free,
// which is worth about a tenth of a metre of apex and one percent of energy, and none of it is
// restitution. With continuous collision the cube lands on the surface and both the apex and the
// energy come in under where they started. The safety factor here matches the one the sample sets,
// which is the supported answer for a body whose elastic accuracy matters.
//
// The discrete run is only gated on its first bounce. Later bounces there are a genuine blow-up, not
// a tolerance question: the free height makes the cube tilt, the next landing is a deep corner impact
// at 14 m/s, and energy reaches 159 percent with the cube spinning at 26 rad/s. Lowering the safety
// factor is the fix and the sample does that; the discrete run is kept to pin what happens when it is
// left alone, and because its speed ratio is what caught the real bug. Details in
// .claude/same_step_restitution.md.
func restitutionOvershootTest() int {
	const dropHeight float32 = 10.0

	discrete := restitutionMeasureOvershoot(false)
	continuous := restitutionMeasureOvershoot(true)

	// Perfectly elastic returns exactly the speed it received, so any excess is round-off. The
	// overflow and colored solvers round differently and either can land an ulp over one.
	const speedTolerance float32 = 1.0e-5

	failed := 0

	if discrete.speedRatio > 1.0+speedTolerance || continuous.speedRatio > 1.0+speedTolerance {
		failed = 1
	}

	if continuous.firstApex > dropHeight || continuous.peakEnergyRatio > 1.001 {
		failed = 1
	}

	if discrete.firstApex > dropHeight+0.15 || discrete.firstEnergyRatio > 1.02 {
		failed = 1
	}

	// The continuous landing leaves the cube sliding sideways a few tenths of a metre per second, from
	// friction during the sub-steps, and a floor narrower than the cube does not catch it on the way
	// back down. That is sub-step friction, not restitution, so only the first bounce is required.
	if discrete.bounceCount < 2 || continuous.bounceCount < 1 {
		failed = 1
	}

	return failed
}

// restitutionImpulseResult is ImpulseResult.
type restitutionImpulseResult struct {
	worstError      float32
	approachSpeed   float32
	firstSeparation float32
	bounceImpulse   float32
	contactSteps    int
	toiSteps        int
	toiImpulseSteps int
}

// restitutionMeasureDropImpulse is MeasureDropImpulse.
// Ball dropped under gravity with the contact impulse read back every step. Nothing else touches the
// ball, so once gravity is taken out the change in momentum over a step is the impulse the contact
// applied, and that is what the total normal impulse must report. A time of impact step is left out
// of the balance: no contact was solved on it, and the sweep hands back the gravity of the time it
// cut short.
func restitutionMeasureDropImpulse(restitution, dropHeight float32) restitutionImpulseResult {
	worldId := restitutionMakeWorld(-10.0)
	restitutionMakeGround(worldId, 0.0)

	ballId := restitutionMakeBall(worldId, 0.0, 0.5+dropHeight, 0.0, restitution)
	world := getWorldFromId(worldId)

	mass := ballId.GetMass()
	gravityY := worldId.GetGravity().Y

	var result restitutionImpulseResult
	touched := false
	bouncing := false

	// Enough for the longest fall and the bounce that follows it
	for range 240 {
		speedBefore := ballId.GetLinearVelocity().Y
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)
		speedAfter := ballId.GetLinearVelocity().Y

		measured := float32(0.0)
		separation := float32(0.0)
		var contactData [4]ContactData
		contactCount := ballId.GetContactData(contactData[:])
		for c := range contactCount {
			for m := 0; m < int(contactData[c].ManifoldCount); m++ {
				manifold := &contactData[c].Manifolds[m]
				for p := 0; p < int(manifold.PointCount); p++ {
					measured += manifold.Points[p].TotalNormalImpulse
					separation = manifold.Points[p].Separation
				}
			}
		}

		// Transient body flags are cleared and rewritten every step
		ball := getBodyFullId(world, ballId)
		ballSim := getBodySim(world, ball)
		if ballSim.flags&hadTimeOfImpact != 0 {
			result.toiSteps += 1
			if measured != 0.0 {
				result.toiImpulseSteps += 1
			}
			continue
		}

		expected := float32(mass*(speedAfter-speedBefore)) - float32(mass*gravityY*restitutionTimeStep)
		result.worstError = MaxFloat(result.worstError, AbsFloat(measured-expected))

		if contactCount > 0 {
			if touched == false {
				touched = true
				bouncing = true
				result.approachSpeed = -speedBefore
				result.firstSeparation = separation
			}

			if bouncing {
				result.bounceImpulse += measured
				result.contactSteps += 1
			}
		} else {
			bouncing = false
		}
	}

	worldId.Destroy()
	return result
}

// restitutionImpulseTest is ImpulseTest.
// Heights on both sides of the continuous collision threshold, so the impulse is checked for a ball
// that lands inside the overlap and for one the sweep sets down on the surface. The threshold is
// derived from the body so the split survives a change to the default safety factor.
//
// The step balance is the real gate. The bounce total is bracketed as well so the restitution sweep
// means something: the contact has to reverse the approach at the coefficient and may carry the
// weight for at most as long as it lasted. The stage runs after the sub-steps, so the reversal
// lands a step late and the ball can come in up to a step of gravity under it.
func restitutionImpulseTest() int {
	restitutions := []float32{0.0, 0.25, 0.5, 0.75, 1.0}
	heights := []float32{1.0, 5.0, 20.0, 45.0}

	var mass float32
	var fastSpeed float32
	{
		worldId := restitutionMakeWorld(-10.0)
		ballId := restitutionMakeBall(worldId, 0.0, 0.5, 0.0, 0.0)
		mass = ballId.GetMass()
		fastSpeed = ballId.GetSafetyFactor() * ballId.GetMinExtent() / restitutionTimeStep
		worldId.Destroy()
	}

	weightImpulse := mass * 10.0 * restitutionTimeStep

	failed := 0

	for j := 0; j < arrayCount(heights); j++ {
		impactSpeed := sqrtf(20.0 * heights[j])
		expectToi := impactSpeed > fastSpeed

		// Float noise in the solver velocities scales with the impact speed, and the ball is heavy
		tolerance := 1e-5 * mass * (10.0 + impactSpeed)

		for k := 0; k < arrayCount(restitutions); k++ {
			result := restitutionMeasureDropImpulse(restitutions[k], heights[j])

			reversal := (1.0 + restitutions[k]) * mass * result.approachSpeed
			lower := reversal - weightImpulse
			upper := reversal + float32(weightImpulse*float32(result.contactSteps))

			GinkgoWriter.Printf("    impulse drop %4.1f e %.2f toi %d at %+.4f -> worst step error %.1e, bounce %.1f in [%.1f, %.1f] over %d "+
				"steps\n",
				heights[j], restitutions[k], result.toiSteps, result.firstSeparation, result.worstError, result.bounceImpulse,
				lower, upper, result.contactSteps)

			if result.worstError > tolerance {
				failed = 1
			}

			if result.contactSteps == 0 || result.toiImpulseSteps > 0 {
				failed = 1
			}

			slack := float32(result.contactSteps) * tolerance
			if result.bounceImpulse < lower-slack || result.bounceImpulse > upper+slack {
				failed = 1
			}

			if (result.toiSteps > 0) != expectToi {
				expectText := "unexpected"
				if expectToi {
					expectText = "expected"
				}
				GinkgoWriter.Printf("    continuous collision %s at %.1f m/s (threshold %.1f m/s)\n", expectText,
					impactSpeed, fastSpeed)
				failed = 1
			}
		}
	}

	return failed
}

// restitutionEnergyScene is EnergyScene.
type restitutionEnergyScene struct {
	name         string
	safetyFactor float32
	friction     float32
	angle        float32
	spin         float32
	height       float32
	velocityY    float32
	halfWidth    float32
	halfHeight   float32
	bullet       bool
	stack        bool
	stepCount    int
}

// restitutionMeasurePeakEnergy is MeasurePeakEnergy.
// Peak energy over the run relative to the start, for every dynamic body in the scene. Perfectly
// elastic and the energy may still only go down: friction and the soft contact position correction
// dissipate, and a bounce may return at most what the contact absorbed.
func restitutionMeasurePeakEnergy(scene *restitutionEnergyScene) float32 {
	worldId := restitutionMakeWorld(-10.0)

	groundDef := DefaultBodyDef()
	groundDef.Position = Pos{0.0, -1.0, 0.0}
	groundId := worldId.CreateBody(&groundDef)
	groundShape := DefaultShapeDef()
	groundShape.BaseMaterial.Friction = scene.friction
	ground := MakeBoxHull(40.0, 1.0, 40.0)
	groundId.CreateHullShape(&groundShape, &ground.Base)

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.BaseMaterial.Friction = scene.friction
	shapeDef.BaseMaterial.Restitution = 1.0
	box := MakeBoxHull(scene.halfWidth, scene.halfHeight, scene.halfWidth)

	var bodyIds [4]BodyID
	bodyCount := 0

	if scene.stack {
		// Resting box under the dropped one, like the TwoBoxRestitution sample
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, scene.halfHeight, 0.0}
		bodyDef.SafetyFactor = scene.safetyFactor
		bodyIds[bodyCount] = worldId.CreateBody(&bodyDef)
		bodyIds[bodyCount].CreateHullShape(&shapeDef, &box.Base)
		bodyCount += 1
	}

	{
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, scene.height, 0.0}
		bodyDef.Rotation = MakeQuatFromAxisAngle(Vec3AxisZ, scene.angle)
		bodyDef.AngularVelocity = Vec3{0.0, 0.0, scene.spin}
		bodyDef.LinearVelocity = Vec3{0.0, scene.velocityY, 0.0}
		bodyDef.SafetyFactor = scene.safetyFactor
		bodyDef.IsBullet = scene.bullet
		bodyIds[bodyCount] = worldId.CreateBody(&bodyDef)
		bodyIds[bodyCount].CreateHullShape(&shapeDef, &box.Base)
		bodyCount += 1
	}

	start := float32(0.0)
	for k := range bodyCount {
		start += restitutionMeasureEnergy(bodyIds[k], worldId)
	}

	peak := start
	peakStep := -1

	for i := range scene.stepCount {
		worldId.Step(restitutionTimeStep, restitutionSubStepCount)

		energy := float32(0.0)
		for k := range bodyCount {
			energy += restitutionMeasureEnergy(bodyIds[k], worldId)
		}

		if energy > peak {
			peak = energy
			peakStep = i
		}
	}

	worldId.Destroy()

	ratio := peak / start
	GinkgoWriter.Printf("    energy %-24s peak %.5f at step %d\n", scene.name, ratio, peakStep)
	return ratio
}

// restitutionEnergyTest is EnergyTest.
// Perfectly elastic material with friction, dropped every way that used to gain energy: a spinning
// edge landing, tumbling, a flat landing with spin, a box falling on a resting box, and a tall box
// landing flat on its narrow end. Energy must never rise above the start. This is the load bearing
// property of restitution. Accuracy of the coefficient and freedom from spin are secondary.
//
// The tall boxes guard the stage iteration count. Their contact points sit close together while the
// inertia is long, so the off diagonal of the multi point normal mass is large and one sweep
// overshoots.
func restitutionEnergyTest() int {
	scenes := []restitutionEnergyScene{
		{"spinning edge", 0.01, 0.6, -1.43, 7.9, 0.7, -1.9, 0.5, 0.5, false, false, 300},
		{"spinning edge fast", 0.01, 0.6, -0.8, 12.0, 3.0, -8.0, 0.5, 0.5, false, false, 300},
		{"tumble", 0.01, 0.6, -0.3, 3.0, 8.0, 0.0, 0.5, 0.5, false, false, 600},
		{"tumble discrete", 0.5, 0.6, -0.3, 3.0, 8.0, 0.0, 0.5, 0.5, false, false, 600},
		{"flat with spin", 0.01, 0.6, 0.0, 2.0, 2.0, -5.0, 0.5, 0.5, false, false, 300},
		{"flat frictionless", 0.01, 0.0, 0.0, 0.0, 10.0, 0.0, 0.5, 0.5, false, false, 1200},
		{"edge", 0.01, 0.6, -0.2, 0.0, 5.0, 0.0, 0.5, 0.5, false, false, 600},
		{"two box", 0.02, 0.6, 0.0, 0.0, 4.0, 0.0, 0.5, 0.5, true, true, 1500},
		{"two box discrete", 0.5, 0.6, 0.0, 0.0, 4.0, 0.0, 0.5, 0.5, false, true, 1500},
		{"tall flat frictionless", 0.01, 0.0, 0.0, 0.0, 10.0, 0.0, 0.5, 1.0, false, false, 1200},
		{"tall flat friction", 0.01, 0.6, 0.0, 0.0, 10.0, 0.0, 0.5, 1.0, false, false, 1200},
		{"narrow flat frictionless", 0.01, 0.0, 0.0, 0.0, 10.0, 0.0, 0.25, 1.0, false, false, 1200},
		{"narrow flat friction", 0.01, 0.6, 0.0, 0.0, 10.0, 0.0, 0.25, 1.0, false, false, 1200},
	}

	// The soft contact position correction lifts a body that lands inside the ground, which is
	// potential energy from nowhere at any restitution. Continuous collision keeps landings on the
	// surface, so the tolerance only has to cover the stage. A four point flat landing does not
	// converge at the default two stage iterations the way the two point landing does in 2D: the cube
	// comes back with 0.2% extra. The in-relax scheme this replaced gained 2.5% to 29% on these scenes,
	// so the tolerance still separates the two.
	const tolerance float32 = 5.0e-3
	const discreteTolerance float32 = 2.0e-2

	failed := 0

	for i := 0; i < arrayCount(scenes); i++ {
		ratio := restitutionMeasurePeakEnergy(&scenes[i])
		limit := discreteTolerance
		if scenes[i].safetyFactor < 0.1 {
			limit = tolerance
		}
		if ratio > 1.0+limit {
			failed = 1
		}
	}

	return failed
}

// restitutionWorkerParityTest is WorkerParityTest.
// The sampled approach speed and the Poisson accumulator are per manifold point state, so they must
// survive the split across workers
func restitutionWorkerParityTest() int {
	hash1 := restitutionRunWorkerScene(1)
	hash2 := restitutionRunWorkerScene(2)
	hash4 := restitutionRunWorkerScene(4)

	GinkgoWriter.Printf("    worker hashes 0x%016x 0x%016x 0x%016x\n", hash1, hash2, hash4)

	ensure(hash1 == hash2)
	ensure(hash1 == hash4)
	return 0
}

var _ = Describe("RestitutionTest", func() {
	It("HeadOnTest", func() { restitutionRunMeasurement("HeadOnTest", restitutionHeadOnTest) })
	It("PhaseTest", func() { restitutionRunMeasurement("PhaseTest", restitutionPhaseTest) })
	It("SupportedTest", func() { restitutionRunMeasurement("SupportedTest", restitutionSupportedTest) })
	It("FlatLandingTest", func() { restitutionRunMeasurement("FlatLandingTest", restitutionFlatLandingTest) })
	It("SpinTest", func() { restitutionRunMeasurement("SpinTest", restitutionSpinTest) })
	It("DropTest", func() { restitutionRunMeasurement("DropTest", restitutionDropTest) })
	It("SingleBoxTest", func() { restitutionRunMeasurement("SingleBoxTest", restitutionSingleBoxTest) })
	It("ThresholdTest", func() { restitutionRunMeasurement("ThresholdTest", restitutionThresholdTest) })
	It("RestingTest", func() { restitutionRunMeasurement("RestingTest", restitutionRestingTest) })
	It("NormalVelocityTest", func() { restitutionRunMeasurement("NormalVelocityTest", restitutionNormalVelocityTest) })
	It("OvershootTest", func() { restitutionRunMeasurement("OvershootTest", restitutionOvershootTest) })
	It("EnergyTest", func() { restitutionRunMeasurement("EnergyTest", restitutionEnergyTest) })
	It("ImpulseTest", func() { restitutionRunMeasurement("ImpulseTest", restitutionImpulseTest) })
	It("WorkerParityTest", func() { restitutionRunMeasurement("WorkerParityTest", restitutionWorkerParityTest) })
})
