//go:build box3d_m4_joints

// Port of test/test_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

// PORT: behind the box3d_m4_joints build tag until the per-type joint files (distance, motor,
// prismatic, revolute, spherical, weld, wheel, parallel_joint.c) land; their functions are stubs
// that panic. Remove the tag then. The sub-test helpers return nothing: ENSURE fails the spec
// through Gomega instead of returning 1.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// One sub-test per joint type. Each creates the joint, exercises the shared
// b3Joint_* API plus every type-specific accessor, then steps to make sure the
// joint solves without tripping a validation assert.

// jointFixture is JointFixture.
type jointFixture struct {
	worldId  WorldID
	groundId BodyID
	bodyId   BodyID
}

// jointTestCreateFixture is CreateJointFixture.
// Static ground plus a dynamic box, anchored so a point-coincident joint starts
// satisfied. Gravity is off so the body stays put across the handful of steps
// each sub-test takes.
func jointTestCreateFixture() jointFixture {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec3Zero

	var f jointFixture
	f.worldId = CreateWorld(&worldDef)

	groundDef := DefaultBodyDef()
	f.groundId = f.worldId.CreateBody(&groundDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{0.0, 4.0, 0.0}
	f.bodyId = f.worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	box := MakeCubeHull(0.5)
	f.bodyId.CreateHullShape(&shapeDef, &box.Base)

	return f
}

// jointTestSetCommonFrames is SetCommonFrames.
// Place the joint anchor at the dynamic body so both local frames map to the
// same world point.
func jointTestSetCommonFrames(base *JointDef, f *jointFixture) {
	base.BodyIdA = f.groundId
	base.BodyIdB = f.bodyId
	base.LocalFrameA.P = Vec3{0.0, 4.0, 0.0}
	base.LocalFrameB.P = Vec3{0.0, 0.0, 0.0}
}

// jointTestFinish is FinishJoint.
// Step a few times, destroy the joint, then the world. Destroying the joint
// explicitly also covers b3DestroyJoint and stale-handle detection.
func jointTestFinish(jointId JointID, worldId WorldID) {
	for i := 0; i < 8; i++ {
		worldId.Step(1.0/60.0, 4)
	}

	jointId.Destroy(true)
	ensure(jointId.IsValid() == false)

	worldId.Destroy()
}

// jointTestExerciseBase is ExerciseJointBase.
// Exercise the API shared by every joint type. Frames are saved and restored so
// the caller's setup survives.
func jointTestExerciseBase(jointId JointID, worldId WorldID, bodyIdA, bodyIdB BodyID, expectedType JointType) {
	ensure(jointId.IsValid())
	ensure(jointId.GetType() == expectedType)
	ensure(jointId.GetBodyA() == bodyIdA)
	ensure(jointId.GetBodyB() == bodyIdB)

	// B3_ID_EQUALS does not work for world ids
	gotWorld := jointId.GetWorld()
	ensure(gotWorld.Index1 == worldId.Index1 && gotWorld.Generation == worldId.Generation)

	originalA := jointId.GetLocalFrameA()
	originalB := jointId.GetLocalFrameB()

	frameA := Transform{Vec3{0.1, 0.2, 0.3}, QuatIdentity}
	jointId.SetLocalFrameA(frameA)
	gotA := jointId.GetLocalFrameA()
	ensure(gotA.P.X == frameA.P.X && gotA.P.Y == frameA.P.Y && gotA.P.Z == frameA.P.Z)

	frameB := Transform{Vec3{-0.4, 0.5, -0.6}, QuatIdentity}
	jointId.SetLocalFrameB(frameB)
	gotB := jointId.GetLocalFrameB()
	ensure(gotB.P.X == frameB.P.X && gotB.P.Y == frameB.P.Y && gotB.P.Z == frameB.P.Z)

	jointId.SetCollideConnected(true)
	ensure(jointId.GetCollideConnected() == true)
	jointId.SetCollideConnected(false)
	ensure(jointId.GetCollideConnected() == false)

	userData := 0
	jointId.SetUserData(&userData)
	ensure(jointId.GetUserData() == &userData)

	jointId.SetConstraintTuning(90.0, 3.0)
	hertz, dampingRatio := jointId.GetConstraintTuning()
	ensure(hertz == 90.0)
	ensure(dampingRatio == 3.0)

	jointId.SetForceThreshold(100.0)
	ensure(jointId.GetForceThreshold() == 100.0)

	jointId.SetTorqueThreshold(200.0)
	ensure(jointId.GetTorqueThreshold() == 200.0)

	jointId.WakeBodies()

	// No stable value to assert before the first step, call for coverage
	_ = jointId.GetConstraintForce()
	_ = jointId.GetConstraintTorque()
	_ = jointId.GetLinearSeparation()

	// Wheel joint angular separation is an unimplemented todo in joint.c that
	// asserts. Every other type computes it.
	if expectedType != WheelJoint {
		_ = jointId.GetAngularSeparation()
	}

	jointId.SetLocalFrameA(originalA)
	jointId.SetLocalFrameB(originalB)
}

var _ = Describe("JointTest", func() {
	It("TestParallelJoint", func() {
		f := jointTestCreateFixture()

		def := DefaultParallelJointDef()
		jointTestSetCommonFrames(&def.Base, &f)
		def.Hertz = 2.0
		def.DampingRatio = 0.5
		def.MaxTorque = 100.0
		jointId := f.worldId.CreateParallelJoint(&def)

		jointTestExerciseBase(jointId, f.worldId, f.groundId, f.bodyId, ParallelJoint)

		jointId.ParallelJointSetSpringHertz(5.0)
		ensure(jointId.ParallelJointGetSpringHertz() == 5.0)

		jointId.ParallelJointSetSpringDampingRatio(0.7)
		ensure(jointId.ParallelJointGetSpringDampingRatio() == 0.7)

		jointId.ParallelJointSetMaxTorque(250.0)
		ensure(jointId.ParallelJointGetMaxTorque() == 250.0)

		jointTestFinish(jointId, f.worldId)
	})

	It("TestDistanceJoint", func() {
		f := jointTestCreateFixture()

		def := DefaultDistanceJointDef()
		jointTestSetCommonFrames(&def.Base, &f)
		def.Length = 2.0
		jointId := f.worldId.CreateDistanceJoint(&def)

		jointTestExerciseBase(jointId, f.worldId, f.groundId, f.bodyId, DistanceJoint)

		jointId.DistanceJointSetLength(3.0)
		ensure(jointId.DistanceJointGetLength() == 3.0)

		jointId.DistanceJointEnableSpring(true)
		ensure(jointId.DistanceJointIsSpringEnabled() == true)

		jointId.DistanceJointSetSpringForceRange(-50.0, 75.0)
		lowerForce, upperForce := jointId.DistanceJointGetSpringForceRange()
		ensure(lowerForce == -50.0 && upperForce == 75.0)

		jointId.DistanceJointSetSpringHertz(4.0)
		ensure(jointId.DistanceJointGetSpringHertz() == 4.0)

		jointId.DistanceJointSetSpringDampingRatio(0.6)
		ensure(jointId.DistanceJointGetSpringDampingRatio() == 0.6)

		jointId.DistanceJointEnableLimit(true)
		ensure(jointId.DistanceJointIsLimitEnabled() == true)

		jointId.DistanceJointSetLengthRange(1.0, 5.0)
		ensure(jointId.DistanceJointGetMinLength() == 1.0)
		ensure(jointId.DistanceJointGetMaxLength() == 5.0)

		_ = jointId.DistanceJointGetCurrentLength()

		jointId.DistanceJointEnableMotor(true)
		ensure(jointId.DistanceJointIsMotorEnabled() == true)

		jointId.DistanceJointSetMotorSpeed(1.5)
		ensure(jointId.DistanceJointGetMotorSpeed() == 1.5)

		jointId.DistanceJointSetMaxMotorForce(25.0)
		ensure(jointId.DistanceJointGetMaxMotorForce() == 25.0)

		_ = jointId.DistanceJointGetMotorForce()

		jointTestFinish(jointId, f.worldId)
	})

	It("TestFilterJoint", func() {
		f := jointTestCreateFixture()

		// The filter joint has no type-specific API. It only disables collision and
		// keeps both bodies in the same island.
		def := DefaultFilterJointDef()
		def.Base.BodyIdA = f.groundId
		def.Base.BodyIdB = f.bodyId
		jointId := f.worldId.CreateFilterJoint(&def)

		jointTestExerciseBase(jointId, f.worldId, f.groundId, f.bodyId, FilterJoint)

		jointTestFinish(jointId, f.worldId)
	})

	It("TestMotorJoint", func() {
		f := jointTestCreateFixture()

		def := DefaultMotorJointDef()
		jointTestSetCommonFrames(&def.Base, &f)
		jointId := f.worldId.CreateMotorJoint(&def)

		jointTestExerciseBase(jointId, f.worldId, f.groundId, f.bodyId, MotorJoint)

		linearVelocity := Vec3{1.0, 2.0, 3.0}
		jointId.MotorJointSetLinearVelocity(linearVelocity)
		gotLinear := jointId.MotorJointGetLinearVelocity()
		ensure(gotLinear.X == 1.0 && gotLinear.Y == 2.0 && gotLinear.Z == 3.0)

		angularVelocity := Vec3{0.1, 0.2, 0.3}
		jointId.MotorJointSetAngularVelocity(angularVelocity)
		gotAngular := jointId.MotorJointGetAngularVelocity()
		ensure(gotAngular.X == 0.1 && gotAngular.Y == 0.2 && gotAngular.Z == 0.3)

		jointId.MotorJointSetMaxVelocityForce(500.0)
		ensure(jointId.MotorJointGetMaxVelocityForce() == 500.0)

		jointId.MotorJointSetMaxVelocityTorque(600.0)
		ensure(jointId.MotorJointGetMaxVelocityTorque() == 600.0)

		jointId.MotorJointSetLinearHertz(3.0)
		ensure(jointId.MotorJointGetLinearHertz() == 3.0)

		jointId.MotorJointSetLinearDampingRatio(0.8)
		ensure(jointId.MotorJointGetLinearDampingRatio() == 0.8)

		jointId.MotorJointSetAngularHertz(4.0)
		ensure(jointId.MotorJointGetAngularHertz() == 4.0)

		jointId.MotorJointSetAngularDampingRatio(0.9)
		ensure(jointId.MotorJointGetAngularDampingRatio() == 0.9)

		jointId.MotorJointSetMaxSpringForce(700.0)
		ensure(jointId.MotorJointGetMaxSpringForce() == 700.0)

		jointId.MotorJointSetMaxSpringTorque(800.0)
		ensure(jointId.MotorJointGetMaxSpringTorque() == 800.0)

		jointTestFinish(jointId, f.worldId)
	})

	It("TestPrismaticJoint", func() {
		f := jointTestCreateFixture()

		def := DefaultPrismaticJointDef()
		jointTestSetCommonFrames(&def.Base, &f)
		jointId := f.worldId.CreatePrismaticJoint(&def)

		jointTestExerciseBase(jointId, f.worldId, f.groundId, f.bodyId, PrismaticJoint)

		jointId.PrismaticJointEnableSpring(true)
		ensure(jointId.PrismaticJointIsSpringEnabled() == true)

		jointId.PrismaticJointSetSpringHertz(5.0)
		ensure(jointId.PrismaticJointGetSpringHertz() == 5.0)

		jointId.PrismaticJointSetSpringDampingRatio(0.5)
		ensure(jointId.PrismaticJointGetSpringDampingRatio() == 0.5)

		jointId.PrismaticJointSetTargetTranslation(1.0)
		ensure(jointId.PrismaticJointGetTargetTranslation() == 1.0)

		jointId.PrismaticJointEnableLimit(true)
		ensure(jointId.PrismaticJointIsLimitEnabled() == true)

		jointId.PrismaticJointSetLimits(-2.0, 2.0)
		ensure(jointId.PrismaticJointGetLowerLimit() == -2.0)
		ensure(jointId.PrismaticJointGetUpperLimit() == 2.0)

		jointId.PrismaticJointEnableMotor(true)
		ensure(jointId.PrismaticJointIsMotorEnabled() == true)

		jointId.PrismaticJointSetMotorSpeed(1.5)
		ensure(jointId.PrismaticJointGetMotorSpeed() == 1.5)

		jointId.PrismaticJointSetMaxMotorForce(30.0)
		ensure(jointId.PrismaticJointGetMaxMotorForce() == 30.0)

		_ = jointId.PrismaticJointGetMotorForce()
		_ = jointId.PrismaticJointGetTranslation()
		_ = jointId.PrismaticJointGetSpeed()

		jointTestFinish(jointId, f.worldId)
	})

	It("TestRevoluteJoint", func() {
		f := jointTestCreateFixture()

		def := DefaultRevoluteJointDef()
		jointTestSetCommonFrames(&def.Base, &f)
		jointId := f.worldId.CreateRevoluteJoint(&def)

		jointTestExerciseBase(jointId, f.worldId, f.groundId, f.bodyId, RevoluteJoint)

		jointId.RevoluteJointEnableSpring(true)
		ensure(jointId.RevoluteJointIsSpringEnabled() == true)

		jointId.RevoluteJointSetSpringHertz(5.0)
		ensure(jointId.RevoluteJointGetSpringHertz() == 5.0)

		jointId.RevoluteJointSetSpringDampingRatio(0.5)
		ensure(jointId.RevoluteJointGetSpringDampingRatio() == 0.5)

		jointId.RevoluteJointSetTargetAngle(0.5)
		ensure(jointId.RevoluteJointGetTargetAngle() == 0.5)

		_ = jointId.RevoluteJointGetAngle()

		jointId.RevoluteJointEnableLimit(true)
		ensure(jointId.RevoluteJointIsLimitEnabled() == true)

		jointId.RevoluteJointSetLimits(-1.0, 1.0)
		ensure(jointId.RevoluteJointGetLowerLimit() == -1.0)
		ensure(jointId.RevoluteJointGetUpperLimit() == 1.0)

		jointId.RevoluteJointEnableMotor(true)
		ensure(jointId.RevoluteJointIsMotorEnabled() == true)

		jointId.RevoluteJointSetMotorSpeed(2.0)
		ensure(jointId.RevoluteJointGetMotorSpeed() == 2.0)

		jointId.RevoluteJointSetMaxMotorTorque(40.0)
		ensure(jointId.RevoluteJointGetMaxMotorTorque() == 40.0)

		_ = jointId.RevoluteJointGetMotorTorque()

		jointTestFinish(jointId, f.worldId)
	})

	It("TestSphericalJoint", func() {
		f := jointTestCreateFixture()

		def := DefaultSphericalJointDef()
		jointTestSetCommonFrames(&def.Base, &f)
		jointId := f.worldId.CreateSphericalJoint(&def)

		jointTestExerciseBase(jointId, f.worldId, f.groundId, f.bodyId, SphericalJoint)

		jointId.SphericalJointEnableConeLimit(true)
		ensure(jointId.SphericalJointIsConeLimitEnabled() == true)

		jointId.SphericalJointSetConeLimit(0.5)
		ensure(jointId.SphericalJointGetConeLimit() == 0.5)

		_ = jointId.SphericalJointGetConeAngle()

		jointId.SphericalJointEnableTwistLimit(true)
		ensure(jointId.SphericalJointIsTwistLimitEnabled() == true)

		jointId.SphericalJointSetTwistLimits(-0.5, 0.5)
		ensure(jointId.SphericalJointGetLowerTwistLimit() == -0.5)
		ensure(jointId.SphericalJointGetUpperTwistLimit() == 0.5)

		_ = jointId.SphericalJointGetTwistAngle()

		jointId.SphericalJointEnableSpring(true)
		ensure(jointId.SphericalJointIsSpringEnabled() == true)

		jointId.SphericalJointSetSpringHertz(5.0)
		ensure(jointId.SphericalJointGetSpringHertz() == 5.0)

		jointId.SphericalJointSetSpringDampingRatio(0.5)
		ensure(jointId.SphericalJointGetSpringDampingRatio() == 0.5)

		// 90 degrees about z, a unit quaternion that round-trips through storage
		targetRotation := Quat{Vec3{0.0, 0.0, 0.7071068}, 0.7071068}
		jointId.SphericalJointSetTargetRotation(targetRotation)
		gotRotation := jointId.SphericalJointGetTargetRotation()
		ensureSmall(gotRotation.V.X-targetRotation.V.X, 1.0e-5)
		ensureSmall(gotRotation.V.Y-targetRotation.V.Y, 1.0e-5)
		ensureSmall(gotRotation.V.Z-targetRotation.V.Z, 1.0e-5)
		ensureSmall(gotRotation.S-targetRotation.S, 1.0e-5)

		jointId.SphericalJointEnableMotor(true)
		ensure(jointId.SphericalJointIsMotorEnabled() == true)

		motorVelocity := Vec3{0.1, 0.2, 0.3}
		jointId.SphericalJointSetMotorVelocity(motorVelocity)
		gotMotorVelocity := jointId.SphericalJointGetMotorVelocity()
		ensure(gotMotorVelocity.X == 0.1 && gotMotorVelocity.Y == 0.2 && gotMotorVelocity.Z == 0.3)

		jointId.SphericalJointSetMaxMotorTorque(50.0)
		ensure(jointId.SphericalJointGetMaxMotorTorque() == 50.0)

		_ = jointId.SphericalJointGetMotorTorque()

		jointTestFinish(jointId, f.worldId)
	})

	It("TestWeldJoint", func() {
		f := jointTestCreateFixture()

		def := DefaultWeldJointDef()
		jointTestSetCommonFrames(&def.Base, &f)
		jointId := f.worldId.CreateWeldJoint(&def)

		jointTestExerciseBase(jointId, f.worldId, f.groundId, f.bodyId, WeldJoint)

		jointId.WeldJointSetLinearHertz(3.0)
		ensure(jointId.WeldJointGetLinearHertz() == 3.0)

		jointId.WeldJointSetLinearDampingRatio(0.5)
		ensure(jointId.WeldJointGetLinearDampingRatio() == 0.5)

		jointId.WeldJointSetAngularHertz(4.0)
		ensure(jointId.WeldJointGetAngularHertz() == 4.0)

		jointId.WeldJointSetAngularDampingRatio(0.7)
		ensure(jointId.WeldJointGetAngularDampingRatio() == 0.7)

		jointTestFinish(jointId, f.worldId)
	})

	It("TestWheelJoint", func() {
		f := jointTestCreateFixture()

		def := DefaultWheelJointDef()
		jointTestSetCommonFrames(&def.Base, &f)
		jointId := f.worldId.CreateWheelJoint(&def)

		jointTestExerciseBase(jointId, f.worldId, f.groundId, f.bodyId, WheelJoint)

		jointId.WheelJointEnableSuspension(true)
		ensure(jointId.WheelJointIsSuspensionEnabled() == true)

		jointId.WheelJointSetSuspensionHertz(5.0)
		ensure(jointId.WheelJointGetSuspensionHertz() == 5.0)

		jointId.WheelJointSetSuspensionDampingRatio(0.5)
		ensure(jointId.WheelJointGetSuspensionDampingRatio() == 0.5)

		jointId.WheelJointEnableSuspensionLimit(true)
		ensure(jointId.WheelJointIsSuspensionLimitEnabled() == true)

		jointId.WheelJointSetSuspensionLimits(-1.0, 1.0)
		ensure(jointId.WheelJointGetLowerSuspensionLimit() == -1.0)
		ensure(jointId.WheelJointGetUpperSuspensionLimit() == 1.0)

		jointId.WheelJointEnableSpinMotor(true)
		ensure(jointId.WheelJointIsSpinMotorEnabled() == true)

		jointId.WheelJointSetSpinMotorSpeed(6.0)
		ensure(jointId.WheelJointGetSpinMotorSpeed() == 6.0)

		jointId.WheelJointSetMaxSpinTorque(35.0)
		ensure(jointId.WheelJointGetMaxSpinTorque() == 35.0)

		_ = jointId.WheelJointGetSpinSpeed()
		_ = jointId.WheelJointGetSpinTorque()

		jointId.WheelJointEnableSteering(true)
		ensure(jointId.WheelJointIsSteeringEnabled() == true)

		jointId.WheelJointSetSteeringHertz(7.0)
		ensure(jointId.WheelJointGetSteeringHertz() == 7.0)

		jointId.WheelJointSetSteeringDampingRatio(0.8)
		ensure(jointId.WheelJointGetSteeringDampingRatio() == 0.8)

		jointId.WheelJointSetMaxSteeringTorque(45.0)
		ensure(jointId.WheelJointGetMaxSteeringTorque() == 45.0)

		jointId.WheelJointEnableSteeringLimit(true)
		ensure(jointId.WheelJointIsSteeringLimitEnabled() == true)

		jointId.WheelJointSetSteeringLimits(-0.6, 0.6)
		ensure(jointId.WheelJointGetLowerSteeringLimit() == -0.6)
		ensure(jointId.WheelJointGetUpperSteeringLimit() == 0.6)

		jointId.WheelJointSetTargetSteeringAngle(0.25)
		ensure(jointId.WheelJointGetTargetSteeringAngle() == 0.25)

		_ = jointId.WheelJointGetSteeringAngle()
		_ = jointId.WheelJointGetSteeringTorque()

		jointTestFinish(jointId, f.worldId)
	})

	// Issue #117: a joint created with collideConnected false must clear contacts
	// that already exist between the two bodies.
	It("TestCreateJointClearsContacts", func() {
		worldDef := DefaultWorldDef()
		worldDef.Gravity = Vec3Zero
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody

		bodyDef.Position = Pos{0.0, 0.0, 0.0}
		bodyIdA := worldId.CreateBody(&bodyDef)

		// Overlapping so a touching contact forms
		bodyDef.Position = Pos{0.5, 0.0, 0.0}
		bodyIdB := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		box := MakeBoxHull(0.5, 0.5, 0.5)
		bodyIdA.CreateHullShape(&shapeDef, &box.Base)
		bodyIdB.CreateHullShape(&shapeDef, &box.Base)

		worldId.Step(1.0/60.0, 4)
		ensure(bodyIdA.GetContactCapacity() > 0)

		jointDef := DefaultRevoluteJointDef()
		jointDef.Base.BodyIdA = bodyIdA
		jointDef.Base.BodyIdB = bodyIdB
		jointDef.Base.CollideConnected = false
		worldId.CreateRevoluteJoint(&jointDef)

		ensure(bodyIdA.GetContactCapacity() == 0)
		ensure(bodyIdB.GetContactCapacity() == 0)

		worldId.Step(1.0/60.0, 4)
		ensure(bodyIdA.GetContactCapacity() == 0)

		worldId.Destroy()
	})

	// Issue #164: a joint with no dynamic body lives in the static set even when one body is an awake
	// kinematic. Disabling and enabling that body has to respect this, whether the joint was created
	// that way or got there through body type changes.
	It("TestNonDynamicJointDisable", func() {
		worldDef := DefaultWorldDef()
		worldDef.Gravity = Vec3Zero
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyIdA := worldId.CreateBody(&bodyDef)
		bodyDef.Position = Pos{2.0, 0.0, 0.0}
		bodyIdB := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		box := MakeCubeHull(0.5)
		bodyIdA.CreateHullShape(&shapeDef, &box.Base)
		bodyIdB.CreateHullShape(&shapeDef, &box.Base)

		jointDef := DefaultWeldJointDef()
		jointDef.Base.BodyIdA = bodyIdA
		jointDef.Base.BodyIdB = bodyIdB
		weldId := worldId.CreateWeldJoint(&jointDef)

		bodyIdA.SetType(StaticBody)
		bodyIdB.SetType(KinematicBody)
		worldId.Step(1.0/60.0, 4)

		bodyIdB.Disable()
		worldId.Step(1.0/60.0, 4)
		bodyIdB.Enable()
		worldId.Step(1.0/60.0, 4)
		ensure(weldId.IsValid())

		bodyIdB.SetType(DynamicBody)
		worldId.Step(1.0/60.0, 4)
		ensure(weldId.IsAwake())

		// Created directly between a static and a kinematic body
		bodyDef.Type = KinematicBody
		bodyDef.Position = Pos{4.0, 0.0, 0.0}
		bodyIdC := worldId.CreateBody(&bodyDef)
		bodyIdC.CreateHullShape(&shapeDef, &box.Base)

		revoluteDef := DefaultRevoluteJointDef()
		revoluteDef.Base.BodyIdA = bodyIdA
		revoluteDef.Base.BodyIdB = bodyIdC
		revoluteId := worldId.CreateRevoluteJoint(&revoluteDef)

		bodyIdC.Disable()
		bodyIdC.Enable()
		worldId.Step(1.0/60.0, 4)
		ensure(revoluteId.IsAwake() == false)

		worldId.Destroy()
	})
})
