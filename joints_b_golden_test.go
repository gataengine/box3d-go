// Go-only test: no upstream counterpart. Steps a world exercising the spherical, weld, wheel and
// parallel joints (two spherical pendulums with cone/twist limits, motor and spring; a welded
// cantilever with rigid and soft welds; a car on wheel joints with suspension, spin motors and
// steering; two parallel joints), changes the joints through their setters at steps 60 and 120,
// and compares every body's transform and velocities, every joint's constraint force and torque,
// separations and per-type getters, the joint events, b3HashWorldState and the joint debug draw
// bit for bit against the pinned upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867)
// compiled with -ffp-contract=off. See testdata/joints_b_golden_gen.c; this file mirrors it line
// by line. The C output is identical at 1 and 4 workers and for the NEON, BOX3D_DISABLE_SIMD and
// BOX3D_VALIDATE builds; the Go run is checked at 1, 2 and 4 workers.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"os"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const (
	jointsBGoldenSteps  = 240
	jointsBGoldenStride = 20
)

func jointsBGoldenVec(sb *strings.Builder, v Vec3) {
	fmt.Fprintf(sb, " %08x %08x %08x", math.Float32bits(v.X), math.Float32bits(v.Y), math.Float32bits(v.Z))
}

func jointsBGoldenFloat(sb *strings.Builder, f float32) {
	fmt.Fprintf(sb, " %08x", math.Float32bits(f))
}

func jointsBGoldenCreateDynamic(worldId WorldID, p Pos, q Quat) BodyID {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = p
	bodyDef.Rotation = q
	return worldId.CreateBody(&bodyDef)
}

func jointsBGoldenRunScene(workerCount int) string {
	var sb strings.Builder

	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)

	var bodies []BodyID
	var joints []JointID

	// Ground: static box, top face at y = 0
	var groundId BodyID
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, -0.5, 0.0}
		groundId = worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		box := MakeBoxHull(20.0, 0.5, 20.0)
		groundId.CreateHullShape(&shapeDef, &box.Base)
		bodies = append(bodies, groundId)
	}

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	cube := MakeCubeHull(0.25)

	// Cone axis (frame z) pointing down
	down := MakeQuatFromAxisAngle(Vec3AxisX, 0.5*Pi)

	// Spherical pendulum with cone limit, twist limit and motor
	var sphericalLimitId JointID
	{
		bodyId := jointsBGoldenCreateDynamic(worldId, Pos{-6.0, 4.0, 0.0}, QuatIdentity)
		bodyId.CreateHullShape(&shapeDef, &cube.Base)
		bodyId.SetLinearVelocity(Vec3{3.0, 0.0, 1.0})
		bodyId.SetAngularVelocity(Vec3{0.0, 4.0, 0.0})
		bodies = append(bodies, bodyId)

		def := DefaultSphericalJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = Transform{Vec3{-6.0, 5.5, 0.0}, down}
		def.Base.LocalFrameB = Transform{Vec3{0.0, 1.0, 0.0}, down}
		def.EnableConeLimit = true
		def.ConeAngle = 0.5
		def.EnableTwistLimit = true
		def.LowerTwistAngle = -0.25
		def.UpperTwistAngle = 0.25
		def.EnableMotor = true
		def.MaxMotorTorque = 0.5
		def.MotorVelocity = Vec3{0.0, 1.0, 0.0}
		sphericalLimitId = worldId.CreateSphericalJoint(&def)
		joints = append(joints, sphericalLimitId)
	}

	// Spherical pendulum with a spring towards a target rotation
	var sphericalSpringId JointID
	{
		bodyId := jointsBGoldenCreateDynamic(worldId, Pos{-8.0, 4.0, 0.0}, QuatIdentity)
		bodyId.CreateHullShape(&shapeDef, &cube.Base)
		bodyId.SetAngularVelocity(Vec3{1.0, 0.0, -2.0})
		bodies = append(bodies, bodyId)

		def := DefaultSphericalJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = Transform{Vec3{-8.0, 5.0, 0.0}, QuatIdentity}
		def.Base.LocalFrameB = Transform{Vec3{0.0, 0.5, 0.0}, QuatIdentity}
		def.EnableSpring = true
		def.Hertz = 2.0
		def.DampingRatio = 0.25
		def.TargetRotation = Quat{Vec3{0.0, 0.6, 0.0}, 0.8}
		sphericalSpringId = worldId.CreateSphericalJoint(&def)
		joints = append(joints, sphericalSpringId)
	}

	// Cantilever: three boxes welded to a static pillar
	var weldIds [3]JointID
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{-3.0, 1.5, 4.0}
		pillarId := worldId.CreateBody(&bodyDef)
		pillar := MakeBoxHull(0.25, 1.5, 0.25)
		pillarId.CreateHullShape(&shapeDef, &pillar.Base)
		bodies = append(bodies, pillarId)

		prevId := pillarId
		prevAnchor := Vec3{0.25, 1.25, 0.0}
		for i := range 3 {
			bodyId := jointsBGoldenCreateDynamic(worldId, Pos{float32(-2.5) + float32(0.5*float32(i)), 2.75, 4.0}, QuatIdentity)
			bodyId.CreateHullShape(&shapeDef, &cube.Base)
			bodies = append(bodies, bodyId)

			def := DefaultWeldJointDef()
			def.Base.BodyIdA = prevId
			def.Base.BodyIdB = bodyId
			def.Base.LocalFrameA = Transform{prevAnchor, QuatIdentity}
			def.Base.LocalFrameB = Transform{Vec3{-0.25, 0.0, 0.0}, QuatIdentity}
			if i == 1 {
				def.LinearHertz = 5.0
				def.LinearDampingRatio = 0.75
				def.AngularHertz = 3.0
				def.AngularDampingRatio = 0.5
			} else if i == 2 {
				def.AngularHertz = 2.0
				def.AngularDampingRatio = 0.25
			}
			weldIds[i] = worldId.CreateWeldJoint(&def)
			joints = append(joints, weldIds[i])

			prevId = bodyId
			prevAnchor = Vec3{0.25, 0.0, 0.0}
		}

		weldIds[0].SetForceThreshold(20.0)
	}

	// Car: chassis and four sphere wheels on wheel joints
	var wheelIds [4]JointID
	{
		chassisId := jointsBGoldenCreateDynamic(worldId, Pos{4.0, 1.0, 0.0}, QuatIdentity)
		chassis := MakeBoxHull(1.0, 0.25, 0.5)
		chassisId.CreateHullShape(&shapeDef, &chassis.Base)
		bodies = append(bodies, chassisId)

		// Suspension axis (frame x) pointing up
		up := MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)

		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.375}
		shapeDef.BaseMaterial.Friction = 0.875

		for i := range 4 {
			var x, z float32 = 0.75, 0.75
			if i&1 != 0 {
				x = -0.75
			}
			if i&2 != 0 {
				z = -0.75
			}
			front := x > 0.0

			wheelId := jointsBGoldenCreateDynamic(worldId, Pos{4.0 + x, 0.5, z}, QuatIdentity)
			wheelId.CreateSphereShape(&shapeDef, &sphere)
			bodies = append(bodies, wheelId)

			def := DefaultWheelJointDef()
			def.Base.BodyIdA = chassisId
			def.Base.BodyIdB = wheelId
			def.Base.LocalFrameA = Transform{Vec3{x, -0.5, z}, up}
			def.Base.LocalFrameB = Transform{Vec3{0.0, 0.0, 0.0}, QuatIdentity}
			def.EnableSuspensionSpring = true
			def.SuspensionHertz = 4.0
			def.SuspensionDampingRatio = 0.625
			def.EnableSuspensionLimit = true
			def.LowerSuspensionLimit = -0.25
			def.UpperSuspensionLimit = 0.125
			if front {
				def.EnableSteering = true
				def.SteeringHertz = 5.0
				def.SteeringDampingRatio = 0.75
				def.MaxSteeringTorque = 50.0
				def.TargetSteeringAngle = 0.25
				def.EnableSteeringLimit = true
				def.LowerSteeringLimit = -0.375
				def.UpperSteeringLimit = 0.375
			} else {
				def.EnableSpinMotor = true
				def.MaxSpinTorque = 20.0
				def.SpinSpeed = -8.0
			}
			wheelIds[i] = worldId.CreateWheelJoint(&def)
			joints = append(joints, wheelIds[i])
		}

		shapeDef.BaseMaterial = DefaultSurfaceMaterial()
		wheelIds[1].SetTorqueThreshold(5.0)
	}

	// Parallel joints
	var parallelIds [2]JointID
	{
		bodyId := jointsBGoldenCreateDynamic(worldId, Pos{8.0, 2.0, -3.0}, QuatIdentity)
		bodyId.CreateHullShape(&shapeDef, &cube.Base)
		bodyId.SetAngularVelocity(Vec3{3.0, 1.0, 2.0})
		bodies = append(bodies, bodyId)

		def := DefaultParallelJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = Transform{Vec3{8.0, 2.5, -3.0}, QuatIdentity}
		def.Base.LocalFrameB = Transform{Vec3{0.0, 0.0, 0.0}, QuatIdentity}
		def.Base.CollideConnected = true
		def.Hertz = 2.0
		def.DampingRatio = 0.5
		def.MaxTorque = 5.0
		parallelIds[0] = worldId.CreateParallelJoint(&def)
		joints = append(joints, parallelIds[0])

		bodyIdA := jointsBGoldenCreateDynamic(worldId, Pos{8.0, 1.0, 3.0}, QuatIdentity)
		bodyIdA.CreateHullShape(&shapeDef, &cube.Base)
		bodyIdA.SetAngularVelocity(Vec3{0.0, 0.0, 2.0})
		bodies = append(bodies, bodyIdA)

		bodyIdB := jointsBGoldenCreateDynamic(worldId, Pos{8.0, 2.5, 3.0}, Quat{Vec3{0.0, 0.0, 0.6}, 0.8})
		bodyIdB.CreateHullShape(&shapeDef, &cube.Base)
		bodyIdB.SetAngularVelocity(Vec3{-2.0, 0.0, 0.0})
		bodies = append(bodies, bodyIdB)

		def.Base.BodyIdA = bodyIdA
		def.Base.BodyIdB = bodyIdB
		def.Base.LocalFrameA = Transform{Vec3{0.0, 0.5, 0.0}, QuatIdentity}
		def.Base.LocalFrameB = Transform{Vec3{0.0, -0.5, 0.0}, QuatIdentity}
		def.Hertz = 10.0
		def.DampingRatio = 1.0
		def.MaxTorque = 1000.0
		parallelIds[1] = worldId.CreateParallelJoint(&def)
		joints = append(joints, parallelIds[1])
	}

	world := getWorldFromId(worldId)

	draw := DefaultDebugDraw()
	draw.DrawSegmentFcn = func(p1, p2 Pos, color HexColor, context any) {
		sb.WriteString("seg |")
		jointsBGoldenVec(&sb, ToVec3(p1))
		jointsBGoldenVec(&sb, ToVec3(p2))
		fmt.Fprintf(&sb, " %06x\n", uint32(color))
	}
	draw.DrawPointFcn = func(p Pos, size float32, color HexColor, context any) {
		sb.WriteString("point |")
		jointsBGoldenVec(&sb, ToVec3(p))
		jointsBGoldenFloat(&sb, size)
		fmt.Fprintf(&sb, " %06x\n", uint32(color))
	}
	draw.DrawBoxFcn = func(extents Vec3, transform WorldTransform, color HexColor, context any) {
		sb.WriteString("box |")
		jointsBGoldenVec(&sb, extents)
		jointsBGoldenVec(&sb, ToVec3(transform.P))
		jointsBGoldenVec(&sb, transform.Q.V)
		jointsBGoldenFloat(&sb, transform.Q.S)
		fmt.Fprintf(&sb, " %06x\n", uint32(color))
	}
	draw.DrawJoints = true
	draw.DrawGraphColors = true

	for step := 1; step <= jointsBGoldenSteps; step++ {
		if step == 60 {
			sphericalLimitId.SphericalJointSetConeLimit(0.75)
			sphericalLimitId.SphericalJointSetTwistLimits(0.5, -0.5)
			sphericalLimitId.SphericalJointEnableMotor(false)
			sphericalLimitId.SphericalJointSetMaxMotorTorque(1.0)
			sphericalLimitId.SphericalJointSetMotorVelocity(Vec3{0.5, 0.0, 0.0})
			sphericalSpringId.SphericalJointSetTargetRotation(QuatIdentity)
			sphericalSpringId.SphericalJointSetSpringHertz(3.0)
			sphericalSpringId.SphericalJointSetSpringDampingRatio(0.5)

			weldIds[1].WeldJointSetAngularHertz(0.0)
			weldIds[2].WeldJointSetLinearHertz(4.0)
			weldIds[2].WeldJointSetLinearDampingRatio(0.5)
			weldIds[2].WeldJointSetAngularDampingRatio(1.0)

			wheelIds[1].WheelJointSetSpinMotorSpeed(4.0)
			wheelIds[3].WheelJointSetMaxSpinTorque(10.0)
			wheelIds[0].WheelJointSetTargetSteeringAngle(-0.25)
			wheelIds[2].WheelJointSetTargetSteeringAngle(-0.25)
			wheelIds[2].WheelJointSetSteeringLimits(-0.125, 0.125)
			wheelIds[0].WheelJointSetSuspensionHertz(6.0)
			wheelIds[0].WheelJointSetSuspensionDampingRatio(0.5)
			wheelIds[3].WheelJointSetSuspensionLimits(-0.125, 0.25)
			wheelIds[0].WheelJointSetSteeringHertz(3.0)
			wheelIds[0].WheelJointSetSteeringDampingRatio(0.5)
			wheelIds[0].WheelJointSetMaxSteeringTorque(30.0)

			parallelIds[0].ParallelJointSetMaxTorque(0.5)
			parallelIds[1].ParallelJointSetSpringHertz(4.0)
			parallelIds[1].ParallelJointSetSpringDampingRatio(0.25)
		}

		if step == 120 {
			sphericalLimitId.SphericalJointEnableConeLimit(false)
			sphericalLimitId.SphericalJointEnableTwistLimit(false)
			sphericalLimitId.SphericalJointEnableMotor(true)
			sphericalSpringId.SphericalJointEnableSpring(false)
			sphericalSpringId.SphericalJointEnableTwistLimit(true)

			weldIds[1].WeldJointSetLinearHertz(0.0)

			wheelIds[2].WheelJointEnableSteering(false)
			wheelIds[0].WheelJointEnableSteeringLimit(false)
			wheelIds[1].WheelJointEnableSuspension(false)
			wheelIds[3].WheelJointEnableSuspensionLimit(false)
			wheelIds[3].WheelJointEnableSpinMotor(false)
			wheelIds[0].WheelJointEnableSpinMotor(true)
			wheelIds[0].WheelJointSetSpinMotorSpeed(2.0)
			wheelIds[0].WheelJointSetMaxSpinTorque(5.0)
		}

		worldId.Step(1.0/60.0, 4)

		events := worldId.GetJointEvents()
		for i := range int(events.Count) {
			fmt.Fprintf(&sb, "event %d | %d\n", step, events.JointEvents[i].JointId.Index1)
		}

		if step%jointsBGoldenStride != 0 && step != jointsBGoldenSteps {
			continue
		}

		for i, bodyId := range bodies {
			xf := bodyId.GetTransform()
			v := bodyId.GetLinearVelocity()
			w := bodyId.GetAngularVelocity()
			fmt.Fprintf(&sb, "body %d %d |", step, i)
			jointsBGoldenVec(&sb, ToVec3(xf.P))
			jointsBGoldenVec(&sb, xf.Q.V)
			jointsBGoldenFloat(&sb, xf.Q.S)
			jointsBGoldenVec(&sb, v)
			jointsBGoldenVec(&sb, w)
			fmt.Fprintf(&sb, " %d\n", boolInt(bodyId.IsAwake()))
		}

		for i, jointId := range joints {
			typ := jointId.GetType()
			fmt.Fprintf(&sb, "joint %d %d %d |", step, i, int(typ))
			jointsBGoldenVec(&sb, jointId.GetConstraintForce())
			jointsBGoldenVec(&sb, jointId.GetConstraintTorque())
			jointsBGoldenFloat(&sb, jointId.GetLinearSeparation())
			if typ != WheelJoint {
				jointsBGoldenFloat(&sb, jointId.GetAngularSeparation())
			}

			switch typ {
			case SphericalJoint:
				jointsBGoldenFloat(&sb, jointId.SphericalJointGetConeAngle())
				jointsBGoldenFloat(&sb, jointId.SphericalJointGetTwistAngle())
				jointsBGoldenVec(&sb, jointId.SphericalJointGetMotorTorque())
				jointsBGoldenFloat(&sb, jointId.SphericalJointGetConeLimit())
				jointsBGoldenFloat(&sb, jointId.SphericalJointGetLowerTwistLimit())
				jointsBGoldenFloat(&sb, jointId.SphericalJointGetUpperTwistLimit())
				fmt.Fprintf(&sb, " %d%d%d%d", boolInt(jointId.SphericalJointIsConeLimitEnabled()),
					boolInt(jointId.SphericalJointIsTwistLimitEnabled()), boolInt(jointId.SphericalJointIsSpringEnabled()),
					boolInt(jointId.SphericalJointIsMotorEnabled()))

			case WeldJoint:
				jointsBGoldenFloat(&sb, jointId.WeldJointGetLinearHertz())
				jointsBGoldenFloat(&sb, jointId.WeldJointGetAngularHertz())

			case WheelJoint:
				jointsBGoldenFloat(&sb, jointId.WheelJointGetSpinSpeed())
				jointsBGoldenFloat(&sb, jointId.WheelJointGetSpinTorque())
				jointsBGoldenFloat(&sb, jointId.WheelJointGetSteeringAngle())
				jointsBGoldenFloat(&sb, jointId.WheelJointGetSteeringTorque())
				fmt.Fprintf(&sb, " %d%d%d%d%d", boolInt(jointId.WheelJointIsSuspensionEnabled()),
					boolInt(jointId.WheelJointIsSuspensionLimitEnabled()), boolInt(jointId.WheelJointIsSpinMotorEnabled()),
					boolInt(jointId.WheelJointIsSteeringEnabled()), boolInt(jointId.WheelJointIsSteeringLimitEnabled()))

			case ParallelJoint:
				jointsBGoldenFloat(&sb, jointId.ParallelJointGetSpringHertz())
				jointsBGoldenFloat(&sb, jointId.ParallelJointGetMaxTorque())

			default:
			}

			sb.WriteString("\n")
		}

		counters := worldId.GetCounters()
		fmt.Fprintf(&sb, "world %d | %d %d %d %016x\n", step, worldId.GetAwakeBodyCount(), counters.ContactCount,
			counters.IslandCount, hashWorldState(world))

		if step == 60 || step == jointsBGoldenSteps {
			fmt.Fprintf(&sb, "draw %d\n", step)
			worldId.Draw(&draw, DefaultMaskBits)
		}
	}

	worldId.Destroy()

	return sb.String()
}

func jointsBGoldenCompare(got string) {
	data, err := os.ReadFile("testdata/joints_b_golden.txt")
	Expect(err).NotTo(HaveOccurred())
	want := strings.Split(strings.TrimRight(string(data), "\n"), "\n")
	lines := strings.Split(strings.TrimRight(got, "\n"), "\n")
	for i := range min(len(want), len(lines)) {
		if want[i] != lines[i] {
			Fail(fmt.Sprintf("first mismatch at line %d:\n got  %s\n want %s", i+1, lines[i], want[i]))
		}
	}
	Expect(len(lines)).To(Equal(len(want)))
}

var _ = Describe("JointsBGoldenTest", func() {
	It("MatchesC", func() {
		jointsBGoldenCompare(jointsBGoldenRunScene(1))
	})

	It("MatchesCWithWorkers", func() {
		jointsBGoldenCompare(jointsBGoldenRunScene(2))
		jointsBGoldenCompare(jointsBGoldenRunScene(4))
	})
})
