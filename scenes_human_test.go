//go:build box3d_m3_integration

// Port of shared/human.h and shared/human.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: test scene helper (spec: shared/ scenes are test helpers). It lives in a _test.go file of
// package box3d because the internal tests cannot import a package that imports box3d. Behind the
// box3d_m3_integration build tag until body.c, shape.c (milestone 3) and the joints (milestone 4)
// are merged. assert() from <assert.h> is ensure(). The #if 0 pose control block is not ported.

// ---- human.h ----

// boneId is BoneId.
const (
	bonePelvis    = iota // bone_pelvis
	boneSpine01          // bone_spine_01
	boneSpine03          // bone_spine_03
	boneNeck             // bone_neck
	boneThighL           // bone_thigh_l
	boneCalfL            // bone_calf_l
	boneThighR           // bone_thigh_r
	boneCalfR            // bone_calf_r
	boneUpperArmL        // bone_upper_arm_l
	boneLowerArmL        // bone_lower_arm_l
	boneUpperArmR        // bone_upper_arm_r
	boneLowerArmR        // bone_lower_arm_r
	boneCount            // bone_count
)

// bone is Bone.
type bone struct {
	bodyId         BodyID
	jointId        JointID
	anchorId       BodyID
	anchorJointId  JointID
	localFrameA    Transform
	localFrameB    Transform
	referenceFrame Transform
	jointType      JointType
	swingLimit     float32
	twistLimit     Vec2
	jointFriction  float32
	parentIndex    int
}

const filterJointCount = 9 // FILTER_JOINT_COUNT

// human is Human.
// This must be zero initialized
type human struct {
	bones            [boneCount]bone
	filterJoints     [filterJointCount]JointID
	filterJointCount int
	frictionTorque   float32
	isSpawned        bool
}

// ---- human.c ----

// createHuman is CreateHuman.
func createHuman(human *human, worldId WorldID, position Pos, frictionTorque, hertz, dampingRatio float32,
	groupIndex int, userData any, colorize bool) {
	ensure(human.isSpawned == false)

	for i := range boneCount {
		human.bones[i].bodyId = NullBodyID
		human.bones[i].anchorId = NullBodyID
		human.bones[i].jointId = NullJointID
		human.bones[i].jointFriction = 1.0
		human.bones[i].parentIndex = -1
	}

	for i := range filterJointCount {
		human.filterJoints[i] = NullJointID
	}

	human.frictionTorque = frictionTorque

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.UserData = userData

	shapeDef := DefaultShapeDef()
	defaultDensity := shapeDef.Density

	shirtColor := ColorMediumTurquoise
	pantColor := ColorDodgerBlue
	skinColors := [4]HexColor{ColorNavajoWhite, ColorLightYellow, ColorPeru, ColorTan}
	skinColor := skinColors[groupIndex%4]

	// colorOrZero is the colorize ? color : 0 expression.
	colorOrZero := func(color HexColor) uint32 {
		if colorize {
			return uint32(color)
		}
		return 0
	}

	{
		bone := &human.bones[bonePelvis]
		bone.parentIndex = -1

		bodyDef.Name = "pelvis"
		bone.referenceFrame = Transform{Vec3{0.000000, 0.996219, -0.023868}, Quat{Vec3{1.000000, 0.000000, 0.000000}, 0.000000}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = 0
		shapeDef.BaseMaterial.CustomColor = colorOrZero(pantColor)

		shapeDef.BaseMaterial.Friction = 0.6
		shapeDef.BaseMaterial.RollingResistance = 0.1
		capsule := Capsule{Vec3{0.040000, 0.000001, 0.000000}, Vec3{-0.040000, -0.000001, 0.000000}, 0.150000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &capsule)
	}

	{
		bone := &human.bones[boneSpine01]
		bone.parentIndex = bonePelvis

		bodyDef.Name = "spine_01"
		bone.referenceFrame = Transform{Vec3{0.000000, 1.017288, -0.024882}, Quat{Vec3{1.000000, 0.000000, 0.000000}, 0.000000}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = int32(-groupIndex)
		shapeDef.BaseMaterial.CustomColor = colorOrZero(shirtColor)

		shapeDef.BaseMaterial.Friction = 0.5
		shapeDef.BaseMaterial.RollingResistance = 0.1
		capsule := Capsule{Vec3{0.029876, -0.146581, 0.006260}, Vec3{-0.029876, -0.146574, 0.006260}, 0.145663}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &capsule)

		bone.jointType = SphericalJoint
		bone.localFrameA = Transform{Vec3{0.000000, -0.021069, 0.001014}, Quat{Vec3{-0.642737, 0.000000, 0.000000}, -0.766087}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{-0.707107, 0.000000, 0.000000}, -0.707107}}
		bone.swingLimit = 35.0 * DegToRad
		bone.twistLimit = Vec2{-17.5 * DegToRad, 17.5 * DegToRad}
	}

	{
		bone := &human.bones[boneSpine03]
		bone.parentIndex = boneSpine01

		bodyDef.Name = "spine_03"
		bone.referenceFrame = Transform{Vec3{0.000000, 1.267766, -0.022320}, Quat{Vec3{1.000000, 0.000000, 0.000000}, 0.000000}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = 0
		shapeDef.BaseMaterial.CustomColor = colorOrZero(shirtColor)

		shapeDef.BaseMaterial.Friction = 0.5
		shapeDef.BaseMaterial.RollingResistance = 0.1
		capsule := Capsule{Vec3{0.063996, -0.117434, -0.040199}, Vec3{-0.063996, -0.117432, -0.040199}, 0.165004}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &capsule)

		bone.jointType = SphericalJoint
		bone.localFrameA = Transform{Vec3{0.000000, -0.250478, -0.002562}, Quat{Vec3{-0.642766, 0.000000, 0.000000}, -0.766063}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{-0.707107, 0.000000, 0.000000}, -0.707107}}
		bone.swingLimit = 35.0 * DegToRad
		bone.twistLimit = Vec2{-17.5 * DegToRad, 17.5 * DegToRad}
	}

	{
		bone := &human.bones[boneNeck]
		bone.parentIndex = boneSpine03

		bodyDef.Name = "neck_01"
		bone.referenceFrame = Transform{Vec3{0.000000, 1.498783, 0.024462}, Quat{Vec3{0.987879, 0.000000, 0.000000}, 0.155228}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = 0
		shapeDef.BaseMaterial.CustomColor = colorOrZero(skinColor)

		shapeDef.BaseMaterial.Friction = 0.2
		shapeDef.BaseMaterial.RollingResistance = 0.05
		head := Capsule{Vec3{0.000000, -0.212340, 0.000000}, Vec3{0.000000, -0.087340, 0.000000}, 0.110000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &head)

		bone.jointType = SphericalJoint
		bone.localFrameA = Transform{Vec3{0.000000, -0.231017, -0.046781}, Quat{Vec3{-0.516157, -0.000002, 0.000003}, -0.856494}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{-0.707108, -0.000002, 0.000002}, -0.707106}}
		bone.swingLimit = 30.0 * DegToRad
		bone.twistLimit = Vec2{-17.5 * DegToRad, 17.5 * DegToRad}
	}

	{
		bone := &human.bones[boneThighL]
		bone.parentIndex = bonePelvis

		bodyDef.Name = "thigh_l"
		bone.referenceFrame = Transform{Vec3{0.092175, 0.971562, -0.011177}, Quat{Vec3{0.471412, 0.529510, -0.499888}, -0.497495}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = int32(-groupIndex)
		shapeDef.BaseMaterial.CustomColor = colorOrZero(pantColor)

		shapeDef.BaseMaterial.Friction = 0.6
		shapeDef.BaseMaterial.RollingResistance = 0.1
		capsule := Capsule{Vec3{-0.047269, 0.000001, 0.000000}, Vec3{-0.379769, 0.000005, 0.000000}, 0.091539}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &capsule)

		bone.jointType = SphericalJoint
		bone.localFrameA = Transform{Vec3{0.092175, 0.024656, -0.012691}, Quat{Vec3{-0.460186, -0.365368, -0.637662}, 0.498118}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{-0.499997, -0.500003, 0.500002}, 0.499998}}
		bone.swingLimit = 30.0 * DegToRad
		bone.twistLimit = Vec2{-10.0 * DegToRad, 10.0 * DegToRad}
	}

	{
		bone := &human.bones[boneCalfL]
		bone.parentIndex = boneThighL

		bodyDef.Name = "calf_l"
		bone.referenceFrame = Transform{Vec3{0.118720, 0.534541, -0.035362}, Quat{Vec3{0.521188, 0.480597, -0.546380}, -0.445935}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = 0
		shapeDef.BaseMaterial.CustomColor = colorOrZero(pantColor)

		shapeDef.BaseMaterial.Friction = 0.1
		shapeDef.BaseMaterial.RollingResistance = 0.0
		shin := Capsule{Vec3{-0.445000, 0.000001, 0.000000}, Vec3{0.005000, -0.000001, 0.000000}, 0.080000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &shin)

		shapeDef.Density = 0.5 * defaultDensity
		foot := Capsule{Vec3{-0.456371, 0.129321, 0.000013}, Vec3{-0.359977, 0.014394, 0.000013}, 0.070000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &foot)
		shapeDef.Density = defaultDensity

		bone.jointType = RevoluteJoint
		bone.localFrameA = Transform{Vec3{-0.438494, -0.000175, 0.000000}, Quat{Vec3{0.344866, -0.938652, 0.000002}, -0.000008}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{-0.008182, -0.999967, 0.000001}, -0.000007}}
		bone.twistLimit = Vec2{-30.0 * DegToRad, 30.0 * DegToRad}
	}

	{
		bone := &human.bones[boneThighR]
		bone.parentIndex = bonePelvis

		bodyDef.Name = "thigh_r"
		bone.referenceFrame = Transform{Vec3{-0.092175, 0.971562, -0.011177}, Quat{Vec3{-0.497495, 0.499888, 0.529510}, -0.471412}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = int32(-groupIndex)
		shapeDef.BaseMaterial.CustomColor = colorOrZero(pantColor)

		shapeDef.BaseMaterial.Friction = 0.6
		shapeDef.BaseMaterial.RollingResistance = 0.1
		capsule := Capsule{Vec3{0.047269, -0.000001, 0.000000}, Vec3{0.379769, -0.000005, 0.000000}, 0.092587}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &capsule)

		bone.jointType = SphericalJoint
		bone.localFrameA = Transform{Vec3{-0.092175, 0.024656, -0.012691}, Quat{Vec3{0.380666, 0.491846, 0.488644}, -0.611889}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{-0.500002, 0.499998, -0.499998}, 0.500002}}
		bone.swingLimit = 30.0 * DegToRad
		bone.twistLimit = Vec2{-10.0 * DegToRad, 10.0 * DegToRad}
	}

	{
		bone := &human.bones[boneCalfR]
		bone.parentIndex = boneThighR

		bodyDef.Name = "calf_r"
		bone.referenceFrame = Transform{Vec3{-0.118720, 0.534541, -0.035362}, Quat{Vec3{-0.445935, 0.546380, 0.480597}, -0.521188}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = 0
		shapeDef.BaseMaterial.CustomColor = colorOrZero(pantColor)

		shapeDef.BaseMaterial.Friction = 0.1
		shapeDef.BaseMaterial.RollingResistance = 0.0
		shin := Capsule{Vec3{0.445000, -0.000002, 0.000003}, Vec3{-0.005000, 0.000000, 0.000003}, 0.080000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &shin)

		shapeDef.Density = 0.5 * defaultDensity
		foot := Capsule{Vec3{0.369113, -0.006830, 0.000017}, Vec3{0.465507, -0.121757, 0.000017}, 0.070000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &foot)
		shapeDef.Density = defaultDensity

		bone.jointType = RevoluteJoint
		bone.localFrameA = Transform{Vec3{0.438494, 0.000175, 0.000000}, Quat{Vec3{0.000007, 0.000005, -0.344860}, -0.938654}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{0.000006, 0.000005, 0.008188}, -0.999966}}
		bone.twistLimit = Vec2{-30.0 * DegToRad, 30.0 * DegToRad}
	}

	{
		bone := &human.bones[boneUpperArmL]
		bone.parentIndex = boneSpine03

		bodyDef.Name = "upperarm_l"
		bone.referenceFrame = Transform{Vec3{0.185817, 1.443630, 0.031306}, Quat{Vec3{0.560220, -0.307179, 0.366233}, -0.676512}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = 0
		shapeDef.BaseMaterial.CustomColor = colorOrZero(shirtColor)

		shapeDef.BaseMaterial.Friction = 0.6
		shapeDef.BaseMaterial.RollingResistance = 0.1
		capsule := Capsule{Vec3{0.296038, 0.004386, -0.000794}, Vec3{-0.016086, 0.004384, -0.016127}, 0.065000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &capsule)

		bone.jointType = SphericalJoint
		bone.localFrameA = Transform{Vec3{0.185817, -0.175865, -0.053625}, Quat{Vec3{-0.559466, -0.213784, -0.670199}, 0.438323}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{-0.500953, -0.502617, -0.509318}, -0.486844}}
		bone.swingLimit = 45.0 * DegToRad
		bone.twistLimit = Vec2{-20.0 * DegToRad, 20.0 * DegToRad}
		bone.jointFriction = 0.8
	}

	{
		bone := &human.bones[boneLowerArmL]
		bone.parentIndex = boneUpperArmL

		bodyDef.Name = "lowerarm_l"
		bone.referenceFrame = Transform{Vec3{0.347683, 1.193455, 0.029378}, Quat{Vec3{0.406736, -0.493012, 0.089135}, -0.763911}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = 0
		shapeDef.BaseMaterial.CustomColor = colorOrZero(skinColor)

		shapeDef.BaseMaterial.Friction = 0.3
		shapeDef.BaseMaterial.RollingResistance = 0.1
		capsule := Capsule{Vec3{0.312329, -0.008256, -0.016454}, Vec3{0.033153, 0.008259, -0.002741}, 0.060000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &capsule)

		bone.jointType = RevoluteJoint
		bone.localFrameA = Transform{Vec3{0.297979, 0.000361, 0.000000}, Quat{Vec3{0.916726, 0.399352, 0.000839}, -0.011456}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{0.999922, -0.003857, 0.004659}, -0.010908}}
		bone.twistLimit = Vec2{-35.0 * DegToRad, 35.0 * DegToRad}
		bone.jointFriction = 0.06
	}

	{
		bone := &human.bones[boneUpperArmR]
		bone.parentIndex = boneSpine03

		bodyDef.Name = "upperarm_r"
		bone.referenceFrame = Transform{Vec3{-0.185817, 1.443630, 0.031306}, Quat{Vec3{0.676512, 0.366233, 0.307179}, 0.560220}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = 0
		shapeDef.BaseMaterial.CustomColor = colorOrZero(shirtColor)

		shapeDef.BaseMaterial.Friction = 0.6
		shapeDef.BaseMaterial.RollingResistance = 0.1
		capsule := Capsule{Vec3{-0.300237, -0.000001, -0.000319}, Vec3{0.011887, 0.000001, 0.015013}, 0.065000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &capsule)

		bone.jointType = SphericalJoint
		bone.localFrameA = Transform{Vec3{-0.185817, -0.175865, -0.053625}, Quat{Vec3{0.213793, 0.559449, 0.438334}, -0.670204}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{-0.509319, 0.486843, 0.500954}, -0.502617}}
		bone.swingLimit = 45.0 * DegToRad
		bone.twistLimit = Vec2{-20.0 * DegToRad, 20.0 * DegToRad}
		bone.jointFriction = 0.8
	}

	{
		bone := &human.bones[boneLowerArmR]
		bone.parentIndex = boneUpperArmR

		bodyDef.Name = "lowerarm_r"
		bone.referenceFrame = Transform{Vec3{-0.347683, 1.193455, 0.029378}, Quat{Vec3{0.763911, 0.089135, 0.493012}, 0.406736}}
		bodyDef.Rotation = bone.referenceFrame.Q
		bodyDef.Position = OffsetPos(position, bone.referenceFrame.P)
		bone.bodyId = worldId.CreateBody(&bodyDef)

		shapeDef.Filter.GroupIndex = 0
		shapeDef.BaseMaterial.CustomColor = colorOrZero(skinColor)

		shapeDef.BaseMaterial.Friction = 0.3
		shapeDef.BaseMaterial.RollingResistance = 0.1
		capsule := Capsule{Vec3{-0.312649, -0.000003, 0.016471}, Vec3{-0.032986, 0.000000, 0.002732}, 0.060000}
		bone.bodyId.CreateCapsuleShape(&shapeDef, &capsule)

		bone.jointType = RevoluteJoint
		bone.localFrameA = Transform{Vec3{-0.297979, -0.000361, 0.000000}, Quat{Vec3{-0.405444, 0.914059, 0.010541}, 0.000852}}
		bone.localFrameB = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{-0.002794, 0.999936, 0.010042}, 0.004364}}
		bone.twistLimit = Vec2{-35.0 * DegToRad, 35.0 * DegToRad}
		bone.jointFriction = 0.06
	}

	for i := 1; i < boneCount; i++ {
		bone := &human.bones[i]
		parent := &human.bones[bone.parentIndex]

		bodyIdA := parent.bodyId
		bodyIdB := bone.bodyId

		bone.localFrameA.Q = NormalizeQuat(bone.localFrameA.Q)
		bone.localFrameB.Q = NormalizeQuat(bone.localFrameB.Q)

		if bone.jointType == RevoluteJoint {
			jointDef := DefaultRevoluteJointDef()
			jointDef.Base.BodyIdA = bodyIdA
			jointDef.Base.BodyIdB = bodyIdB
			jointDef.Base.LocalFrameA = bone.localFrameA
			jointDef.Base.LocalFrameB = bone.localFrameB
			jointDef.EnableLimit = true
			jointDef.LowerAngle = bone.twistLimit.X
			jointDef.UpperAngle = bone.twistLimit.Y
			jointDef.EnableSpring = hertz > 0.0
			jointDef.Hertz = hertz
			jointDef.DampingRatio = dampingRatio
			jointDef.EnableMotor = true
			jointDef.MaxMotorTorque = bone.jointFriction * frictionTorque
			bone.jointId = worldId.CreateRevoluteJoint(&jointDef)
		} else if bone.jointType == SphericalJoint {
			jointDef := DefaultSphericalJointDef()
			jointDef.Base.BodyIdA = bodyIdA
			jointDef.Base.BodyIdB = bodyIdB
			jointDef.Base.LocalFrameA = bone.localFrameA
			jointDef.Base.LocalFrameB = bone.localFrameB
			jointDef.EnableConeLimit = true
			jointDef.ConeAngle = bone.swingLimit
			jointDef.EnableTwistLimit = true
			jointDef.LowerTwistAngle = bone.twistLimit.X
			jointDef.UpperTwistAngle = bone.twistLimit.Y
			jointDef.EnableSpring = hertz > 0.0
			jointDef.Hertz = hertz
			jointDef.DampingRatio = dampingRatio
			jointDef.EnableMotor = true
			jointDef.MaxMotorTorque = bone.jointFriction * frictionTorque
			bone.jointId = worldId.CreateSphericalJoint(&jointDef)
		}
	}

	// Disable some collisions
	human.filterJointCount = 0
	filterDef := DefaultFilterJointDef()
	filterDef.Base.BodyIdA = human.bones[boneThighL].bodyId
	filterDef.Base.BodyIdB = human.bones[boneThighR].bodyId
	human.filterJoints[human.filterJointCount] = worldId.CreateFilterJoint(&filterDef)
	human.filterJointCount++
	filterDef.Base.BodyIdA = human.bones[boneNeck].bodyId
	filterDef.Base.BodyIdB = human.bones[boneUpperArmL].bodyId
	human.filterJoints[human.filterJointCount] = worldId.CreateFilterJoint(&filterDef)
	human.filterJointCount++
	filterDef.Base.BodyIdA = human.bones[boneNeck].bodyId
	filterDef.Base.BodyIdB = human.bones[boneUpperArmR].bodyId
	human.filterJoints[human.filterJointCount] = worldId.CreateFilterJoint(&filterDef)
	human.filterJointCount++

	human.isSpawned = true
}

// destroyHuman is DestroyHuman.
// There are no allocations so this does not need to be called when the world is destroyed.
func destroyHuman(human *human) {
	ensure(human.isSpawned == true)

	for i := 0; i < human.filterJointCount; i++ {
		human.filterJoints[i].Destroy(false)
		human.filterJoints[i] = NullJointID
	}

	for i := range boneCount {
		if human.bones[i].jointId.IsNull() {
			continue
		}

		human.bones[i].jointId.Destroy(false)
		human.bones[i].jointId = NullJointID
	}

	for i := range boneCount {
		if human.bones[i].bodyId.IsNull() {
			continue
		}

		human.bones[i].bodyId.Destroy()
		human.bones[i].bodyId = NullBodyID
	}

	human.isSpawned = false
}

// humanSetVelocity is Human_SetVelocity.
func humanSetVelocity(human *human, velocity Vec3) {
	for i := range boneCount {
		bodyId := human.bones[i].bodyId

		if bodyId.IsNull() {
			continue
		}

		bodyId.SetLinearVelocity(velocity)
	}
}

// humanApplyRandomAngularImpulse is Human_ApplyRandomAngularImpulse.
func humanApplyRandomAngularImpulse(human *human, magnitude float32) {
	ensure(human.isSpawned == true)
	rng := Vec3{magnitude, magnitude, magnitude}
	impulse := randomVec3(Neg(rng), rng)
	human.bones[boneSpine01].bodyId.ApplyAngularImpulse(impulse, true)
}

// humanSetJointFrictionTorque is Human_SetJointFrictionTorque.
func humanSetJointFrictionTorque(human *human, torque float32) {
	ensure(human.isSpawned == true)
	human.frictionTorque = torque

	for i := 1; i < boneCount; i++ {
		bone := &human.bones[i]
		if bone.jointType == RevoluteJoint {
			bone.jointId.RevoluteJointSetMaxMotorTorque(bone.jointFriction * torque)
		} else {
			bone.jointId.SphericalJointSetMaxMotorTorque(bone.jointFriction * torque)
		}
	}
}

// humanSetJointSpringHertz is Human_SetJointSpringHertz.
func humanSetJointSpringHertz(human *human, hertz float32) {
	ensure(human.isSpawned == true)
	for i := 1; i < boneCount; i++ {
		bone := &human.bones[i]
		if bone.jointType == RevoluteJoint {
			bone.jointId.RevoluteJointSetSpringHertz(hertz)
		} else {
			bone.jointId.SphericalJointSetSpringHertz(hertz)
		}
	}
}

// humanSetJointDampingRatio is Human_SetJointDampingRatio.
func humanSetJointDampingRatio(human *human, dampingRatio float32) {
	ensure(human.isSpawned == true)
	for i := 1; i < boneCount; i++ {
		bone := &human.bones[i]
		if bone.jointType == RevoluteJoint {
			bone.jointId.RevoluteJointSetSpringDampingRatio(dampingRatio)
		} else {
			bone.jointId.SphericalJointSetSpringDampingRatio(dampingRatio)
		}
	}
}

// humanAlignSpring is Human_AlignSpring.
func humanAlignSpring(human *human, worldId WorldID, groundId BodyID, hertz, dampingRatio float32) {
	ensure(human.isSpawned == true)

	bone := &human.bones[bonePelvis]
	ensure(bone.jointId.IsNull() == true)
	q := ComputeQuatBetweenUnitVectors(Vec3AxisZ, Vec3AxisY)
	qb := bone.bodyId.GetRotation()

	jointDef := DefaultParallelJointDef()
	jointDef.Base.BodyIdA = groundId
	jointDef.Base.BodyIdB = bone.bodyId
	jointDef.Base.LocalFrameA.Q = q
	jointDef.Base.LocalFrameB.Q = InvMulQuat(qb, q)
	jointDef.Base.DrawScale = 2.0
	jointDef.Base.CollideConnected = true
	jointDef.Hertz = hertz
	jointDef.DampingRatio = dampingRatio

	bone.jointId = worldId.CreateParallelJoint(&jointDef)
}

// humanCreateMotorAnchors is Human_CreateMotorAnchors.
func humanCreateMotorAnchors(human *human, worldId WorldID) {
	anchorDef := DefaultBodyDef()
	anchorDef.Type = KinematicBody

	motorDef := DefaultMotorJointDef()
	motorDef.AngularHertz = 5.0
	motorDef.AngularDampingRatio = 1.0
	motorDef.LinearHertz = 5.0
	motorDef.LinearDampingRatio = 1.0
	motorDef.MaxSpringForce = fltMax
	motorDef.MaxSpringTorque = fltMax

	for i := range boneCount {
		bone := &human.bones[i]

		bodyTransform := bone.bodyId.GetTransform()
		anchorDef.Position = bodyTransform.P
		anchorDef.Rotation = bodyTransform.Q
		bone.anchorId = worldId.CreateBody(&anchorDef)

		motorDef.Base.BodyIdA = bone.anchorId
		motorDef.Base.BodyIdB = bone.bodyId

		bone.anchorJointId = worldId.CreateMotorJoint(&motorDef)
	}
}

// humanCreateParallelAnchors is Human_CreateParallelAnchors.
func humanCreateParallelAnchors(human *human, worldId WorldID) {
	anchorDef := DefaultBodyDef()
	anchorDef.Type = KinematicBody

	qFrameWorld := ComputeQuatBetweenUnitVectors(Vec3AxisZ, Vec3AxisY)
	jointDef := DefaultParallelJointDef()
	jointDef.Hertz = 8.0
	jointDef.DampingRatio = 1.0
	jointDef.MaxTorque = 800.0

	for i := range boneCount {
		bone := &human.bones[i]

		bodyTransform := bone.bodyId.GetTransform()
		anchorDef.Position = bodyTransform.P
		anchorDef.Rotation = bodyTransform.Q
		bone.anchorId = worldId.CreateBody(&anchorDef)

		jointDef.Base.BodyIdA = bone.anchorId
		jointDef.Base.BodyIdB = bone.bodyId

		frameQuat := InvMulQuat(bodyTransform.Q, qFrameWorld)
		jointDef.Base.LocalFrameA.Q = frameQuat
		jointDef.Base.LocalFrameB.Q = frameQuat

		bone.anchorJointId = worldId.CreateParallelJoint(&jointDef)
	}
}

// humanSetBullet is Human_SetBullet.
func humanSetBullet(human *human, flag bool) {
	for i := range boneCount {
		bone := &human.bones[i]
		bone.bodyId.SetBullet(flag)
	}
}
