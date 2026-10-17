// Port of src/recording_ops.inl from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: recording_ops.inl is an X-macro manifest, B3_REC_OP( opcode, Name, RET, ARG( TAG, field )... ),
// that upstream expands several times. Go has no X-macros, so each expansion is written out:
// this file holds the two expansions recording.h makes (pass 1a, the b3RecArgs_Name arg structs, and
// the b3_recOpName opcode constants) and the three writer expansions of recording.c
// (b3RecWriteArgs_Name, b3RecWrite_Name, b3RecWriteRet_Name), in manifest order. The reader
// expansions of replay.c are in replay.go. Keep the manifest comments and order in sync with
// upstream when bumping.
//
// ARG tag to Go type (b3RecCType_TAG): BOOL bool, I32 int32, U8 uint8, U16 uint16, U32 uint32,
// U64 uint64, F32 float32, F64 float64, VEC3 Vec3, QUAT Quat, TRANSFORM Transform, POSITION Pos,
// WORLDXF WorldTransform, MATRIX3 Matrix3, AABB AABB, SPHERE Sphere, CAPSULE Capsule,
// QUERYFILTER QueryFilter, SHAPEPROXY ShapeProxy, GEOMID uint32, FILTER Filter,
// MATERIAL SurfaceMaterial, MASSDATA MassData, LOCKS MotionLocks, STR string (const char*),
// WORLDID WorldID, BODYID BodyID, SHAPEID ShapeID, JOINTID JointID, BODYDEF BodyDef,
// SHAPEDEF ShapeDef, EXPLOSIONDEF ExplosionDef, <KIND>JOINTDEF <Kind>JointDef.
//
// Field names are upstream's; the BodySetType field `type` is `typ` (Go keyword).

// ---- arg structs (recording.h codegen pass 1a) ----

// recArgsDestroyWorld is b3RecArgs_DestroyWorld (opcode 0x01, RET_NONE).
type recArgsDestroyWorld struct {
	world WorldID
}

// recArgsStep is b3RecArgs_Step (opcode 0x80, RET_NONE).
type recArgsStep struct {
	world        WorldID
	dt           float32
	subStepCount int32
}

// recArgsWorldEnableSleeping is b3RecArgs_WorldEnableSleeping (opcode 0x02, RET_NONE).
type recArgsWorldEnableSleeping struct {
	world WorldID
	flag  bool
}

// recArgsWorldEnableContinuous is b3RecArgs_WorldEnableContinuous (opcode 0x03, RET_NONE).
type recArgsWorldEnableContinuous struct {
	world WorldID
	flag  bool
}

// recArgsWorldSetRestitutionThreshold is b3RecArgs_WorldSetRestitutionThreshold (opcode 0x04, RET_NONE).
type recArgsWorldSetRestitutionThreshold struct {
	world WorldID
	value float32
}

// recArgsWorldSetHitEventThreshold is b3RecArgs_WorldSetHitEventThreshold (opcode 0x05, RET_NONE).
type recArgsWorldSetHitEventThreshold struct {
	world WorldID
	value float32
}

// recArgsWorldSetGravity is b3RecArgs_WorldSetGravity (opcode 0x06, RET_NONE).
type recArgsWorldSetGravity struct {
	world   WorldID
	gravity Vec3
}

// recArgsWorldExplode is b3RecArgs_WorldExplode (opcode 0x07, RET_NONE).
type recArgsWorldExplode struct {
	world WorldID
	def   ExplosionDef
}

// recArgsWorldSetContactTuning is b3RecArgs_WorldSetContactTuning (opcode 0x08, RET_NONE).
type recArgsWorldSetContactTuning struct {
	world        WorldID
	hertz        float32
	dampingRatio float32
	contactSpeed float32
}

// recArgsWorldSetContactRecycleDistance is b3RecArgs_WorldSetContactRecycleDistance (opcode 0x09, RET_NONE).
type recArgsWorldSetContactRecycleDistance struct {
	world           WorldID
	recycleDistance float32
}

// recArgsWorldSetMaximumLinearSpeed is b3RecArgs_WorldSetMaximumLinearSpeed (opcode 0x0A, RET_NONE).
type recArgsWorldSetMaximumLinearSpeed struct {
	world              WorldID
	maximumLinearSpeed float32
}

// recArgsWorldEnableWarmStarting is b3RecArgs_WorldEnableWarmStarting (opcode 0x0B, RET_NONE).
type recArgsWorldEnableWarmStarting struct {
	world WorldID
	flag  bool
}

// recArgsWorldRebuildStaticTree is b3RecArgs_WorldRebuildStaticTree (opcode 0x0C, RET_NONE).
type recArgsWorldRebuildStaticTree struct {
	world WorldID
}

// recArgsWorldEnableSpeculative is b3RecArgs_WorldEnableSpeculative (opcode 0x0D, RET_NONE).
type recArgsWorldEnableSpeculative struct {
	world WorldID
	flag  bool
}

// recArgsWorldSetRestitutionIterations is b3RecArgs_WorldSetRestitutionIterations (opcode 0x0E, RET_NONE).
type recArgsWorldSetRestitutionIterations struct {
	world      WorldID
	iterations int32
}

// recArgsWorldEnableRestitutionPropagation is b3RecArgs_WorldEnableRestitutionPropagation (opcode 0x0F, RET_NONE).
type recArgsWorldEnableRestitutionPropagation struct {
	world WorldID
	flag  bool
}

// recArgsWorldSetIsotropicInertiaTolerance is b3RecArgs_WorldSetIsotropicInertiaTolerance (opcode 0x70, RET_NONE).
type recArgsWorldSetIsotropicInertiaTolerance struct {
	world     WorldID
	tolerance float32
}

// recArgsCreateBody is b3RecArgs_CreateBody (opcode 0x10, RET_BODYID).
type recArgsCreateBody struct {
	world WorldID
	def   BodyDef
}

// recArgsDestroyBody is b3RecArgs_DestroyBody (opcode 0x11, RET_NONE).
type recArgsDestroyBody struct {
	body BodyID
}

// recArgsBodySetTransform is b3RecArgs_BodySetTransform (opcode 0x20, RET_NONE).
type recArgsBodySetTransform struct {
	body     BodyID
	position Pos
	rotation Quat
}

// recArgsBodySetLinearVelocity is b3RecArgs_BodySetLinearVelocity (opcode 0x21, RET_NONE).
type recArgsBodySetLinearVelocity struct {
	body BodyID
	v    Vec3
}

// recArgsBodySetType is b3RecArgs_BodySetType (opcode 0x22, RET_NONE).
type recArgsBodySetType struct {
	body BodyID
	typ  int32
}

// recArgsBodySetName is b3RecArgs_BodySetName (opcode 0x23, RET_NONE).
type recArgsBodySetName struct {
	body BodyID
	name string
}

// recArgsBodySetAngularVelocity is b3RecArgs_BodySetAngularVelocity (opcode 0x24, RET_NONE).
type recArgsBodySetAngularVelocity struct {
	body BodyID
	w    Vec3
}

// recArgsBodySetTargetTransform is b3RecArgs_BodySetTargetTransform (opcode 0x25, RET_NONE).
type recArgsBodySetTargetTransform struct {
	body     BodyID
	target   WorldTransform
	timeStep float32
	wake     bool
}

// recArgsBodyApplyForce is b3RecArgs_BodyApplyForce (opcode 0x26, RET_NONE).
type recArgsBodyApplyForce struct {
	body  BodyID
	force Vec3
	point Pos
	wake  bool
}

// recArgsBodyApplyForceToCenter is b3RecArgs_BodyApplyForceToCenter (opcode 0x27, RET_NONE).
type recArgsBodyApplyForceToCenter struct {
	body  BodyID
	force Vec3
	wake  bool
}

// recArgsBodyApplyTorque is b3RecArgs_BodyApplyTorque (opcode 0x28, RET_NONE).
type recArgsBodyApplyTorque struct {
	body   BodyID
	torque Vec3
	wake   bool
}

// recArgsBodyApplyLinearImpulse is b3RecArgs_BodyApplyLinearImpulse (opcode 0x29, RET_NONE).
type recArgsBodyApplyLinearImpulse struct {
	body    BodyID
	impulse Vec3
	point   Pos
	wake    bool
}

// recArgsBodyApplyLinearImpulseToCenter is b3RecArgs_BodyApplyLinearImpulseToCenter (opcode 0x2A, RET_NONE).
type recArgsBodyApplyLinearImpulseToCenter struct {
	body    BodyID
	impulse Vec3
	wake    bool
}

// recArgsBodyApplyAngularImpulse is b3RecArgs_BodyApplyAngularImpulse (opcode 0x2B, RET_NONE).
type recArgsBodyApplyAngularImpulse struct {
	body    BodyID
	impulse Vec3
	wake    bool
}

// recArgsBodySetMassData is b3RecArgs_BodySetMassData (opcode 0x2C, RET_NONE).
type recArgsBodySetMassData struct {
	body     BodyID
	massData MassData
}

// recArgsBodyApplyMassFromShapes is b3RecArgs_BodyApplyMassFromShapes (opcode 0x2D, RET_NONE).
type recArgsBodyApplyMassFromShapes struct {
	body BodyID
}

// recArgsBodySetLinearDamping is b3RecArgs_BodySetLinearDamping (opcode 0x2E, RET_NONE).
type recArgsBodySetLinearDamping struct {
	body    BodyID
	damping float32
}

// recArgsBodySetAngularDamping is b3RecArgs_BodySetAngularDamping (opcode 0x2F, RET_NONE).
type recArgsBodySetAngularDamping struct {
	body    BodyID
	damping float32
}

// recArgsBodySetGravityScale is b3RecArgs_BodySetGravityScale (opcode 0x30, RET_NONE).
type recArgsBodySetGravityScale struct {
	body  BodyID
	scale float32
}

// recArgsBodySetAwake is b3RecArgs_BodySetAwake (opcode 0x31, RET_NONE).
type recArgsBodySetAwake struct {
	body  BodyID
	awake bool
}

// recArgsBodyEnableSleep is b3RecArgs_BodyEnableSleep (opcode 0x32, RET_NONE).
type recArgsBodyEnableSleep struct {
	body BodyID
	flag bool
}

// recArgsBodySetSleepThreshold is b3RecArgs_BodySetSleepThreshold (opcode 0x33, RET_NONE).
type recArgsBodySetSleepThreshold struct {
	body      BodyID
	threshold float32
}

// recArgsBodyDisable is b3RecArgs_BodyDisable (opcode 0x34, RET_NONE).
type recArgsBodyDisable struct {
	body BodyID
}

// recArgsBodyEnable is b3RecArgs_BodyEnable (opcode 0x35, RET_NONE).
type recArgsBodyEnable struct {
	body BodyID
}

// recArgsBodySetMotionLocks is b3RecArgs_BodySetMotionLocks (opcode 0x36, RET_NONE).
type recArgsBodySetMotionLocks struct {
	body  BodyID
	locks MotionLocks
}

// recArgsBodySetBullet is b3RecArgs_BodySetBullet (opcode 0x37, RET_NONE).
type recArgsBodySetBullet struct {
	body BodyID
	flag bool
}

// recArgsBodyEnableContactRecycling is b3RecArgs_BodyEnableContactRecycling (opcode 0x38, RET_NONE).
type recArgsBodyEnableContactRecycling struct {
	body BodyID
	flag bool
}

// recArgsBodyEnableHitEvents is b3RecArgs_BodyEnableHitEvents (opcode 0x39, RET_NONE).
type recArgsBodyEnableHitEvents struct {
	body BodyID
	flag bool
}

// recArgsBodyAllowFastRotation is b3RecArgs_BodyAllowFastRotation (opcode 0x3A, RET_NONE).
type recArgsBodyAllowFastRotation struct {
	body BodyID
	flag bool
}

// recArgsBodySetSafetyFactor is b3RecArgs_BodySetSafetyFactor (opcode 0x3B, RET_NONE).
type recArgsBodySetSafetyFactor struct {
	body  BodyID
	value float32
}

// recArgsCreateSphereShape is b3RecArgs_CreateSphereShape (opcode 0x40, RET_SHAPEID).
type recArgsCreateSphereShape struct {
	body   BodyID
	def    ShapeDef
	sphere Sphere
}

// recArgsCreateCapsuleShape is b3RecArgs_CreateCapsuleShape (opcode 0x41, RET_SHAPEID).
type recArgsCreateCapsuleShape struct {
	body    BodyID
	def     ShapeDef
	capsule Capsule
}

// recArgsCreateHullShape is b3RecArgs_CreateHullShape (opcode 0x42, RET_SHAPEID).
type recArgsCreateHullShape struct {
	body       BodyID
	def        ShapeDef
	geometryId uint32
}

// recArgsCreateMeshShape is b3RecArgs_CreateMeshShape (opcode 0x43, RET_SHAPEID).
type recArgsCreateMeshShape struct {
	body       BodyID
	def        ShapeDef
	geometryId uint32
	scale      Vec3
}

// recArgsCreateHeightFieldShape is b3RecArgs_CreateHeightFieldShape (opcode 0x44, RET_SHAPEID).
type recArgsCreateHeightFieldShape struct {
	body       BodyID
	def        ShapeDef
	geometryId uint32
}

// recArgsCreateCompoundShape is b3RecArgs_CreateCompoundShape (opcode 0x45, RET_SHAPEID).
type recArgsCreateCompoundShape struct {
	body       BodyID
	def        ShapeDef
	geometryId uint32
}

// recArgsDestroyShape is b3RecArgs_DestroyShape (opcode 0x46, RET_NONE).
type recArgsDestroyShape struct {
	shape          ShapeID
	updateBodyMass bool
}

// recArgsShapeSetDensity is b3RecArgs_ShapeSetDensity (opcode 0x50, RET_NONE).
type recArgsShapeSetDensity struct {
	shape          ShapeID
	density        float32
	updateBodyMass bool
}

// recArgsShapeSetFriction is b3RecArgs_ShapeSetFriction (opcode 0x51, RET_NONE).
type recArgsShapeSetFriction struct {
	shape    ShapeID
	friction float32
}

// recArgsShapeSetRestitution is b3RecArgs_ShapeSetRestitution (opcode 0x52, RET_NONE).
type recArgsShapeSetRestitution struct {
	shape       ShapeID
	restitution float32
}

// recArgsShapeSetSurfaceMaterial is b3RecArgs_ShapeSetSurfaceMaterial (opcode 0x53, RET_NONE).
type recArgsShapeSetSurfaceMaterial struct {
	shape    ShapeID
	material SurfaceMaterial
}

// recArgsShapeSetFilter is b3RecArgs_ShapeSetFilter (opcode 0x54, RET_NONE).
type recArgsShapeSetFilter struct {
	shape          ShapeID
	filter         Filter
	invokeContacts bool
}

// recArgsShapeEnableSensorEvents is b3RecArgs_ShapeEnableSensorEvents (opcode 0x55, RET_NONE).
type recArgsShapeEnableSensorEvents struct {
	shape ShapeID
	flag  bool
}

// recArgsShapeEnableContactEvents is b3RecArgs_ShapeEnableContactEvents (opcode 0x56, RET_NONE).
type recArgsShapeEnableContactEvents struct {
	shape ShapeID
	flag  bool
}

// recArgsShapeEnablePreSolveEvents is b3RecArgs_ShapeEnablePreSolveEvents (opcode 0x57, RET_NONE).
type recArgsShapeEnablePreSolveEvents struct {
	shape ShapeID
	flag  bool
}

// recArgsShapeEnableHitEvents is b3RecArgs_ShapeEnableHitEvents (opcode 0x58, RET_NONE).
type recArgsShapeEnableHitEvents struct {
	shape ShapeID
	flag  bool
}

// recArgsShapeSetSphere is b3RecArgs_ShapeSetSphere (opcode 0x59, RET_NONE).
type recArgsShapeSetSphere struct {
	shape  ShapeID
	sphere Sphere
}

// recArgsShapeSetCapsule is b3RecArgs_ShapeSetCapsule (opcode 0x5A, RET_NONE).
type recArgsShapeSetCapsule struct {
	shape   ShapeID
	capsule Capsule
}

// recArgsShapeApplyWind is b3RecArgs_ShapeApplyWind (opcode 0x5B, RET_NONE).
type recArgsShapeApplyWind struct {
	shape    ShapeID
	wind     Vec3
	drag     float32
	lift     float32
	maxSpeed float32
	wake     bool
}

// recArgsShapeSetName is b3RecArgs_ShapeSetName (opcode 0x5C, RET_NONE).
type recArgsShapeSetName struct {
	shape ShapeID
	name  string
}

// recArgsShapeSetMeshMaterial is b3RecArgs_ShapeSetMeshMaterial (opcode 0x5D, RET_NONE).
type recArgsShapeSetMeshMaterial struct {
	shape    ShapeID
	material SurfaceMaterial
	index    int32
}

// recArgsShapeSetHull is b3RecArgs_ShapeSetHull (opcode 0x5E, RET_NONE).
type recArgsShapeSetHull struct {
	shape      ShapeID
	geometryId uint32
}

// recArgsShapeSetMesh is b3RecArgs_ShapeSetMesh (opcode 0x5F, RET_NONE).
type recArgsShapeSetMesh struct {
	shape      ShapeID
	geometryId uint32
	scale      Vec3
}

// recArgsCreateParallelJoint is b3RecArgs_CreateParallelJoint (opcode 0x90, RET_JOINTID).
type recArgsCreateParallelJoint struct {
	world WorldID
	def   ParallelJointDef
}

// recArgsCreateDistanceJoint is b3RecArgs_CreateDistanceJoint (opcode 0x91, RET_JOINTID).
type recArgsCreateDistanceJoint struct {
	world WorldID
	def   DistanceJointDef
}

// recArgsCreateFilterJoint is b3RecArgs_CreateFilterJoint (opcode 0x92, RET_JOINTID).
type recArgsCreateFilterJoint struct {
	world WorldID
	def   FilterJointDef
}

// recArgsCreateMotorJoint is b3RecArgs_CreateMotorJoint (opcode 0x93, RET_JOINTID).
type recArgsCreateMotorJoint struct {
	world WorldID
	def   MotorJointDef
}

// recArgsCreatePrismaticJoint is b3RecArgs_CreatePrismaticJoint (opcode 0x94, RET_JOINTID).
type recArgsCreatePrismaticJoint struct {
	world WorldID
	def   PrismaticJointDef
}

// recArgsCreateRevoluteJoint is b3RecArgs_CreateRevoluteJoint (opcode 0x95, RET_JOINTID).
type recArgsCreateRevoluteJoint struct {
	world WorldID
	def   RevoluteJointDef
}

// recArgsCreateSphericalJoint is b3RecArgs_CreateSphericalJoint (opcode 0x96, RET_JOINTID).
type recArgsCreateSphericalJoint struct {
	world WorldID
	def   SphericalJointDef
}

// recArgsCreateWeldJoint is b3RecArgs_CreateWeldJoint (opcode 0x97, RET_JOINTID).
type recArgsCreateWeldJoint struct {
	world WorldID
	def   WeldJointDef
}

// recArgsCreateWheelJoint is b3RecArgs_CreateWheelJoint (opcode 0x98, RET_JOINTID).
type recArgsCreateWheelJoint struct {
	world WorldID
	def   WheelJointDef
}

// recArgsDestroyJoint is b3RecArgs_DestroyJoint (opcode 0x99, RET_NONE).
type recArgsDestroyJoint struct {
	joint        JointID
	wakeAttached bool
}

// recArgsJointSetLocalFrameA is b3RecArgs_JointSetLocalFrameA (opcode 0x9A, RET_NONE).
type recArgsJointSetLocalFrameA struct {
	joint      JointID
	localFrame Transform
}

// recArgsJointSetLocalFrameB is b3RecArgs_JointSetLocalFrameB (opcode 0x9B, RET_NONE).
type recArgsJointSetLocalFrameB struct {
	joint      JointID
	localFrame Transform
}

// recArgsJointSetCollideConnected is b3RecArgs_JointSetCollideConnected (opcode 0x9C, RET_NONE).
type recArgsJointSetCollideConnected struct {
	joint         JointID
	shouldCollide bool
}

// recArgsJointWakeBodies is b3RecArgs_JointWakeBodies (opcode 0x9D, RET_NONE).
type recArgsJointWakeBodies struct {
	joint JointID
}

// recArgsJointSetConstraintTuning is b3RecArgs_JointSetConstraintTuning (opcode 0x9E, RET_NONE).
type recArgsJointSetConstraintTuning struct {
	joint        JointID
	hertz        float32
	dampingRatio float32
}

// recArgsJointSetForceThreshold is b3RecArgs_JointSetForceThreshold (opcode 0x9F, RET_NONE).
type recArgsJointSetForceThreshold struct {
	joint     JointID
	threshold float32
}

// recArgsJointSetTorqueThreshold is b3RecArgs_JointSetTorqueThreshold (opcode 0xA0, RET_NONE).
type recArgsJointSetTorqueThreshold struct {
	joint     JointID
	threshold float32
}

// recArgsParallelJointSetSpringHertz is b3RecArgs_ParallelJointSetSpringHertz (opcode 0xA1, RET_NONE).
type recArgsParallelJointSetSpringHertz struct {
	joint JointID
	hertz float32
}

// recArgsParallelJointSetSpringDampingRatio is b3RecArgs_ParallelJointSetSpringDampingRatio (opcode 0xA2, RET_NONE).
type recArgsParallelJointSetSpringDampingRatio struct {
	joint        JointID
	dampingRatio float32
}

// recArgsParallelJointSetMaxTorque is b3RecArgs_ParallelJointSetMaxTorque (opcode 0xA3, RET_NONE).
type recArgsParallelJointSetMaxTorque struct {
	joint     JointID
	maxTorque float32
}

// recArgsDistanceJointSetLength is b3RecArgs_DistanceJointSetLength (opcode 0xA4, RET_NONE).
type recArgsDistanceJointSetLength struct {
	joint  JointID
	length float32
}

// recArgsDistanceJointEnableSpring is b3RecArgs_DistanceJointEnableSpring (opcode 0xA5, RET_NONE).
type recArgsDistanceJointEnableSpring struct {
	joint        JointID
	enableSpring bool
}

// recArgsDistanceJointSetSpringForceRange is b3RecArgs_DistanceJointSetSpringForceRange (opcode 0xA6, RET_NONE).
type recArgsDistanceJointSetSpringForceRange struct {
	joint      JointID
	lowerForce float32
	upperForce float32
}

// recArgsDistanceJointSetSpringHertz is b3RecArgs_DistanceJointSetSpringHertz (opcode 0xA7, RET_NONE).
type recArgsDistanceJointSetSpringHertz struct {
	joint JointID
	hertz float32
}

// recArgsDistanceJointSetSpringDampingRatio is b3RecArgs_DistanceJointSetSpringDampingRatio (opcode 0xA8, RET_NONE).
type recArgsDistanceJointSetSpringDampingRatio struct {
	joint        JointID
	dampingRatio float32
}

// recArgsDistanceJointEnableLimit is b3RecArgs_DistanceJointEnableLimit (opcode 0xA9, RET_NONE).
type recArgsDistanceJointEnableLimit struct {
	joint       JointID
	enableLimit bool
}

// recArgsDistanceJointSetLengthRange is b3RecArgs_DistanceJointSetLengthRange (opcode 0xAA, RET_NONE).
type recArgsDistanceJointSetLengthRange struct {
	joint     JointID
	minLength float32
	maxLength float32
}

// recArgsDistanceJointEnableMotor is b3RecArgs_DistanceJointEnableMotor (opcode 0xAB, RET_NONE).
type recArgsDistanceJointEnableMotor struct {
	joint       JointID
	enableMotor bool
}

// recArgsDistanceJointSetMotorSpeed is b3RecArgs_DistanceJointSetMotorSpeed (opcode 0xAC, RET_NONE).
type recArgsDistanceJointSetMotorSpeed struct {
	joint      JointID
	motorSpeed float32
}

// recArgsDistanceJointSetMaxMotorForce is b3RecArgs_DistanceJointSetMaxMotorForce (opcode 0xAD, RET_NONE).
type recArgsDistanceJointSetMaxMotorForce struct {
	joint JointID
	force float32
}

// recArgsMotorJointSetLinearVelocity is b3RecArgs_MotorJointSetLinearVelocity (opcode 0xAE, RET_NONE).
type recArgsMotorJointSetLinearVelocity struct {
	joint    JointID
	velocity Vec3
}

// recArgsMotorJointSetAngularVelocity is b3RecArgs_MotorJointSetAngularVelocity (opcode 0xAF, RET_NONE).
type recArgsMotorJointSetAngularVelocity struct {
	joint    JointID
	velocity Vec3
}

// recArgsMotorJointSetMaxVelocityForce is b3RecArgs_MotorJointSetMaxVelocityForce (opcode 0xB0, RET_NONE).
type recArgsMotorJointSetMaxVelocityForce struct {
	joint    JointID
	maxForce float32
}

// recArgsMotorJointSetMaxVelocityTorque is b3RecArgs_MotorJointSetMaxVelocityTorque (opcode 0xB1, RET_NONE).
type recArgsMotorJointSetMaxVelocityTorque struct {
	joint     JointID
	maxTorque float32
}

// recArgsMotorJointSetLinearHertz is b3RecArgs_MotorJointSetLinearHertz (opcode 0xB2, RET_NONE).
type recArgsMotorJointSetLinearHertz struct {
	joint JointID
	hertz float32
}

// recArgsMotorJointSetLinearDampingRatio is b3RecArgs_MotorJointSetLinearDampingRatio (opcode 0xB3, RET_NONE).
type recArgsMotorJointSetLinearDampingRatio struct {
	joint   JointID
	damping float32
}

// recArgsMotorJointSetAngularHertz is b3RecArgs_MotorJointSetAngularHertz (opcode 0xB4, RET_NONE).
type recArgsMotorJointSetAngularHertz struct {
	joint JointID
	hertz float32
}

// recArgsMotorJointSetAngularDampingRatio is b3RecArgs_MotorJointSetAngularDampingRatio (opcode 0xB5, RET_NONE).
type recArgsMotorJointSetAngularDampingRatio struct {
	joint   JointID
	damping float32
}

// recArgsMotorJointSetMaxSpringForce is b3RecArgs_MotorJointSetMaxSpringForce (opcode 0xB6, RET_NONE).
type recArgsMotorJointSetMaxSpringForce struct {
	joint    JointID
	maxForce float32
}

// recArgsMotorJointSetMaxSpringTorque is b3RecArgs_MotorJointSetMaxSpringTorque (opcode 0xB7, RET_NONE).
type recArgsMotorJointSetMaxSpringTorque struct {
	joint     JointID
	maxTorque float32
}

// recArgsPrismaticJointEnableSpring is b3RecArgs_PrismaticJointEnableSpring (opcode 0xB8, RET_NONE).
type recArgsPrismaticJointEnableSpring struct {
	joint        JointID
	enableSpring bool
}

// recArgsPrismaticJointSetSpringHertz is b3RecArgs_PrismaticJointSetSpringHertz (opcode 0xB9, RET_NONE).
type recArgsPrismaticJointSetSpringHertz struct {
	joint JointID
	hertz float32
}

// recArgsPrismaticJointSetSpringDampingRatio is b3RecArgs_PrismaticJointSetSpringDampingRatio (opcode 0xBA, RET_NONE).
type recArgsPrismaticJointSetSpringDampingRatio struct {
	joint        JointID
	dampingRatio float32
}

// recArgsPrismaticJointSetTargetTranslation is b3RecArgs_PrismaticJointSetTargetTranslation (opcode 0xBB, RET_NONE).
type recArgsPrismaticJointSetTargetTranslation struct {
	joint       JointID
	translation float32
}

// recArgsPrismaticJointEnableLimit is b3RecArgs_PrismaticJointEnableLimit (opcode 0xBC, RET_NONE).
type recArgsPrismaticJointEnableLimit struct {
	joint       JointID
	enableLimit bool
}

// recArgsPrismaticJointSetLimits is b3RecArgs_PrismaticJointSetLimits (opcode 0xBD, RET_NONE).
type recArgsPrismaticJointSetLimits struct {
	joint JointID
	lower float32
	upper float32
}

// recArgsPrismaticJointEnableMotor is b3RecArgs_PrismaticJointEnableMotor (opcode 0xBE, RET_NONE).
type recArgsPrismaticJointEnableMotor struct {
	joint       JointID
	enableMotor bool
}

// recArgsPrismaticJointSetMotorSpeed is b3RecArgs_PrismaticJointSetMotorSpeed (opcode 0xBF, RET_NONE).
type recArgsPrismaticJointSetMotorSpeed struct {
	joint      JointID
	motorSpeed float32
}

// recArgsPrismaticJointSetMaxMotorForce is b3RecArgs_PrismaticJointSetMaxMotorForce (opcode 0xC0, RET_NONE).
type recArgsPrismaticJointSetMaxMotorForce struct {
	joint JointID
	force float32
}

// recArgsRevoluteJointEnableSpring is b3RecArgs_RevoluteJointEnableSpring (opcode 0xC1, RET_NONE).
type recArgsRevoluteJointEnableSpring struct {
	joint        JointID
	enableSpring bool
}

// recArgsRevoluteJointSetSpringHertz is b3RecArgs_RevoluteJointSetSpringHertz (opcode 0xC2, RET_NONE).
type recArgsRevoluteJointSetSpringHertz struct {
	joint JointID
	hertz float32
}

// recArgsRevoluteJointSetSpringDampingRatio is b3RecArgs_RevoluteJointSetSpringDampingRatio (opcode 0xC3, RET_NONE).
type recArgsRevoluteJointSetSpringDampingRatio struct {
	joint        JointID
	dampingRatio float32
}

// recArgsRevoluteJointSetTargetAngle is b3RecArgs_RevoluteJointSetTargetAngle (opcode 0xC4, RET_NONE).
type recArgsRevoluteJointSetTargetAngle struct {
	joint JointID
	angle float32
}

// recArgsRevoluteJointEnableLimit is b3RecArgs_RevoluteJointEnableLimit (opcode 0xC5, RET_NONE).
type recArgsRevoluteJointEnableLimit struct {
	joint       JointID
	enableLimit bool
}

// recArgsRevoluteJointSetLimits is b3RecArgs_RevoluteJointSetLimits (opcode 0xC6, RET_NONE).
type recArgsRevoluteJointSetLimits struct {
	joint JointID
	lower float32
	upper float32
}

// recArgsRevoluteJointEnableMotor is b3RecArgs_RevoluteJointEnableMotor (opcode 0xC7, RET_NONE).
type recArgsRevoluteJointEnableMotor struct {
	joint       JointID
	enableMotor bool
}

// recArgsRevoluteJointSetMotorSpeed is b3RecArgs_RevoluteJointSetMotorSpeed (opcode 0xC8, RET_NONE).
type recArgsRevoluteJointSetMotorSpeed struct {
	joint      JointID
	motorSpeed float32
}

// recArgsRevoluteJointSetMaxMotorTorque is b3RecArgs_RevoluteJointSetMaxMotorTorque (opcode 0xC9, RET_NONE).
type recArgsRevoluteJointSetMaxMotorTorque struct {
	joint  JointID
	torque float32
}

// recArgsSphericalJointEnableConeLimit is b3RecArgs_SphericalJointEnableConeLimit (opcode 0xCA, RET_NONE).
type recArgsSphericalJointEnableConeLimit struct {
	joint       JointID
	enableLimit bool
}

// recArgsSphericalJointSetConeLimit is b3RecArgs_SphericalJointSetConeLimit (opcode 0xCB, RET_NONE).
type recArgsSphericalJointSetConeLimit struct {
	joint        JointID
	angleRadians float32
}

// recArgsSphericalJointEnableTwistLimit is b3RecArgs_SphericalJointEnableTwistLimit (opcode 0xCC, RET_NONE).
type recArgsSphericalJointEnableTwistLimit struct {
	joint       JointID
	enableLimit bool
}

// recArgsSphericalJointSetTwistLimits is b3RecArgs_SphericalJointSetTwistLimits (opcode 0xCD, RET_NONE).
type recArgsSphericalJointSetTwistLimits struct {
	joint JointID
	lower float32
	upper float32
}

// recArgsSphericalJointEnableSpring is b3RecArgs_SphericalJointEnableSpring (opcode 0xCE, RET_NONE).
type recArgsSphericalJointEnableSpring struct {
	joint        JointID
	enableSpring bool
}

// recArgsSphericalJointSetSpringHertz is b3RecArgs_SphericalJointSetSpringHertz (opcode 0xCF, RET_NONE).
type recArgsSphericalJointSetSpringHertz struct {
	joint JointID
	hertz float32
}

// recArgsSphericalJointSetSpringDampingRatio is b3RecArgs_SphericalJointSetSpringDampingRatio (opcode 0xD0, RET_NONE).
type recArgsSphericalJointSetSpringDampingRatio struct {
	joint        JointID
	dampingRatio float32
}

// recArgsSphericalJointSetTargetRotation is b3RecArgs_SphericalJointSetTargetRotation (opcode 0xD1, RET_NONE).
type recArgsSphericalJointSetTargetRotation struct {
	joint          JointID
	targetRotation Quat
}

// recArgsSphericalJointEnableMotor is b3RecArgs_SphericalJointEnableMotor (opcode 0xD2, RET_NONE).
type recArgsSphericalJointEnableMotor struct {
	joint       JointID
	enableMotor bool
}

// recArgsSphericalJointSetMotorVelocity is b3RecArgs_SphericalJointSetMotorVelocity (opcode 0xD3, RET_NONE).
type recArgsSphericalJointSetMotorVelocity struct {
	joint         JointID
	motorVelocity Vec3
}

// recArgsSphericalJointSetMaxMotorTorque is b3RecArgs_SphericalJointSetMaxMotorTorque (opcode 0xD4, RET_NONE).
type recArgsSphericalJointSetMaxMotorTorque struct {
	joint  JointID
	torque float32
}

// recArgsWeldJointSetLinearHertz is b3RecArgs_WeldJointSetLinearHertz (opcode 0xD5, RET_NONE).
type recArgsWeldJointSetLinearHertz struct {
	joint JointID
	hertz float32
}

// recArgsWeldJointSetLinearDampingRatio is b3RecArgs_WeldJointSetLinearDampingRatio (opcode 0xD6, RET_NONE).
type recArgsWeldJointSetLinearDampingRatio struct {
	joint        JointID
	dampingRatio float32
}

// recArgsWeldJointSetAngularHertz is b3RecArgs_WeldJointSetAngularHertz (opcode 0xD7, RET_NONE).
type recArgsWeldJointSetAngularHertz struct {
	joint JointID
	hertz float32
}

// recArgsWeldJointSetAngularDampingRatio is b3RecArgs_WeldJointSetAngularDampingRatio (opcode 0xD8, RET_NONE).
type recArgsWeldJointSetAngularDampingRatio struct {
	joint        JointID
	dampingRatio float32
}

// recArgsWheelJointEnableSuspension is b3RecArgs_WheelJointEnableSuspension (opcode 0xD9, RET_NONE).
type recArgsWheelJointEnableSuspension struct {
	joint JointID
	flag  bool
}

// recArgsWheelJointSetSuspensionHertz is b3RecArgs_WheelJointSetSuspensionHertz (opcode 0xDA, RET_NONE).
type recArgsWheelJointSetSuspensionHertz struct {
	joint JointID
	hertz float32
}

// recArgsWheelJointSetSuspensionDampingRatio is b3RecArgs_WheelJointSetSuspensionDampingRatio (opcode 0xDB, RET_NONE).
type recArgsWheelJointSetSuspensionDampingRatio struct {
	joint        JointID
	dampingRatio float32
}

// recArgsWheelJointEnableSuspensionLimit is b3RecArgs_WheelJointEnableSuspensionLimit (opcode 0xDC, RET_NONE).
type recArgsWheelJointEnableSuspensionLimit struct {
	joint JointID
	flag  bool
}

// recArgsWheelJointSetSuspensionLimits is b3RecArgs_WheelJointSetSuspensionLimits (opcode 0xDD, RET_NONE).
type recArgsWheelJointSetSuspensionLimits struct {
	joint JointID
	lower float32
	upper float32
}

// recArgsWheelJointEnableSpinMotor is b3RecArgs_WheelJointEnableSpinMotor (opcode 0xDE, RET_NONE).
type recArgsWheelJointEnableSpinMotor struct {
	joint JointID
	flag  bool
}

// recArgsWheelJointSetSpinMotorSpeed is b3RecArgs_WheelJointSetSpinMotorSpeed (opcode 0xDF, RET_NONE).
type recArgsWheelJointSetSpinMotorSpeed struct {
	joint JointID
	speed float32
}

// recArgsWheelJointSetMaxSpinTorque is b3RecArgs_WheelJointSetMaxSpinTorque (opcode 0xE0, RET_NONE).
type recArgsWheelJointSetMaxSpinTorque struct {
	joint  JointID
	torque float32
}

// recArgsWheelJointEnableSteering is b3RecArgs_WheelJointEnableSteering (opcode 0xE1, RET_NONE).
type recArgsWheelJointEnableSteering struct {
	joint JointID
	flag  bool
}

// recArgsWheelJointSetSteeringHertz is b3RecArgs_WheelJointSetSteeringHertz (opcode 0xE2, RET_NONE).
type recArgsWheelJointSetSteeringHertz struct {
	joint JointID
	hertz float32
}

// recArgsWheelJointSetSteeringDampingRatio is b3RecArgs_WheelJointSetSteeringDampingRatio (opcode 0xE3, RET_NONE).
type recArgsWheelJointSetSteeringDampingRatio struct {
	joint        JointID
	dampingRatio float32
}

// recArgsWheelJointSetMaxSteeringTorque is b3RecArgs_WheelJointSetMaxSteeringTorque (opcode 0xE4, RET_NONE).
type recArgsWheelJointSetMaxSteeringTorque struct {
	joint  JointID
	torque float32
}

// recArgsWheelJointEnableSteeringLimit is b3RecArgs_WheelJointEnableSteeringLimit (opcode 0xE5, RET_NONE).
type recArgsWheelJointEnableSteeringLimit struct {
	joint JointID
	flag  bool
}

// recArgsWheelJointSetSteeringLimits is b3RecArgs_WheelJointSetSteeringLimits (opcode 0xE6, RET_NONE).
type recArgsWheelJointSetSteeringLimits struct {
	joint JointID
	lower float32
	upper float32
}

// recArgsWheelJointSetTargetSteeringAngle is b3RecArgs_WheelJointSetTargetSteeringAngle (opcode 0xE7, RET_NONE).
type recArgsWheelJointSetTargetSteeringAngle struct {
	joint   JointID
	radians float32
}

// recArgsQueryOverlapAABB is b3RecArgs_QueryOverlapAABB (opcode 0xE8, RET_NONE).
type recArgsQueryOverlapAABB struct {
	world  WorldID
	aabb   AABB
	filter QueryFilter
}

// recArgsQueryOverlapShape is b3RecArgs_QueryOverlapShape (opcode 0xE9, RET_NONE).
type recArgsQueryOverlapShape struct {
	world  WorldID
	origin Pos
	proxy  ShapeProxy
	filter QueryFilter
}

// recArgsQueryCastRay is b3RecArgs_QueryCastRay (opcode 0xEA, RET_NONE).
type recArgsQueryCastRay struct {
	world       WorldID
	origin      Pos
	translation Vec3
	filter      QueryFilter
}

// recArgsQueryCastShape is b3RecArgs_QueryCastShape (opcode 0xEB, RET_NONE).
type recArgsQueryCastShape struct {
	world       WorldID
	origin      Pos
	proxy       ShapeProxy
	translation Vec3
	filter      QueryFilter
}

// recArgsQueryCastRayClosest is b3RecArgs_QueryCastRayClosest (opcode 0xEC, RET_NONE).
type recArgsQueryCastRayClosest struct {
	world       WorldID
	origin      Pos
	translation Vec3
	filter      QueryFilter
}

// recArgsQueryCastMover is b3RecArgs_QueryCastMover (opcode 0xED, RET_NONE).
type recArgsQueryCastMover struct {
	world       WorldID
	origin      Pos
	mover       Capsule
	translation Vec3
	filter      QueryFilter
}

// recArgsQueryCollideMover is b3RecArgs_QueryCollideMover (opcode 0xEE, RET_NONE).
type recArgsQueryCollideMover struct {
	world  WorldID
	origin Pos
	mover  Capsule
	filter QueryFilter
}

// recArgsQueryTag is b3RecArgs_QueryTag (opcode 0xEF, RET_NONE).
type recArgsQueryTag struct {
	key uint64
}

// recArgsStateHash is b3RecArgs_StateHash (opcode 0xF1, RET_NONE).
type recArgsStateHash struct {
	world WorldID
	hash  uint64
}

// recArgsRecordingBounds is b3RecArgs_RecordingBounds (opcode 0xF2, RET_NONE).
type recArgsRecordingBounds struct {
	bounds AABB
}

// ---- opcode constants (recording.h b3_recOpName enum) ----

const (
	recOpDestroyWorld                        = 0x01
	recOpStep                                = 0x80
	recOpWorldEnableSleeping                 = 0x02
	recOpWorldEnableContinuous               = 0x03
	recOpWorldSetRestitutionThreshold        = 0x04
	recOpWorldSetHitEventThreshold           = 0x05
	recOpWorldSetGravity                     = 0x06
	recOpWorldExplode                        = 0x07
	recOpWorldSetContactTuning               = 0x08
	recOpWorldSetContactRecycleDistance      = 0x09
	recOpWorldSetMaximumLinearSpeed          = 0x0A
	recOpWorldEnableWarmStarting             = 0x0B
	recOpWorldRebuildStaticTree              = 0x0C
	recOpWorldEnableSpeculative              = 0x0D
	recOpWorldSetRestitutionIterations       = 0x0E
	recOpWorldEnableRestitutionPropagation   = 0x0F
	recOpWorldSetIsotropicInertiaTolerance   = 0x70
	recOpCreateBody                          = 0x10
	recOpDestroyBody                         = 0x11
	recOpBodySetTransform                    = 0x20
	recOpBodySetLinearVelocity               = 0x21
	recOpBodySetType                         = 0x22
	recOpBodySetName                         = 0x23
	recOpBodySetAngularVelocity              = 0x24
	recOpBodySetTargetTransform              = 0x25
	recOpBodyApplyForce                      = 0x26
	recOpBodyApplyForceToCenter              = 0x27
	recOpBodyApplyTorque                     = 0x28
	recOpBodyApplyLinearImpulse              = 0x29
	recOpBodyApplyLinearImpulseToCenter      = 0x2A
	recOpBodyApplyAngularImpulse             = 0x2B
	recOpBodySetMassData                     = 0x2C
	recOpBodyApplyMassFromShapes             = 0x2D
	recOpBodySetLinearDamping                = 0x2E
	recOpBodySetAngularDamping               = 0x2F
	recOpBodySetGravityScale                 = 0x30
	recOpBodySetAwake                        = 0x31
	recOpBodyEnableSleep                     = 0x32
	recOpBodySetSleepThreshold               = 0x33
	recOpBodyDisable                         = 0x34
	recOpBodyEnable                          = 0x35
	recOpBodySetMotionLocks                  = 0x36
	recOpBodySetBullet                       = 0x37
	recOpBodyEnableContactRecycling          = 0x38
	recOpBodyEnableHitEvents                 = 0x39
	recOpBodyAllowFastRotation               = 0x3A
	recOpBodySetSafetyFactor                 = 0x3B
	recOpCreateSphereShape                   = 0x40
	recOpCreateCapsuleShape                  = 0x41
	recOpCreateHullShape                     = 0x42
	recOpCreateMeshShape                     = 0x43
	recOpCreateHeightFieldShape              = 0x44
	recOpCreateCompoundShape                 = 0x45
	recOpDestroyShape                        = 0x46
	recOpShapeSetDensity                     = 0x50
	recOpShapeSetFriction                    = 0x51
	recOpShapeSetRestitution                 = 0x52
	recOpShapeSetSurfaceMaterial             = 0x53
	recOpShapeSetFilter                      = 0x54
	recOpShapeEnableSensorEvents             = 0x55
	recOpShapeEnableContactEvents            = 0x56
	recOpShapeEnablePreSolveEvents           = 0x57
	recOpShapeEnableHitEvents                = 0x58
	recOpShapeSetSphere                      = 0x59
	recOpShapeSetCapsule                     = 0x5A
	recOpShapeApplyWind                      = 0x5B
	recOpShapeSetName                        = 0x5C
	recOpShapeSetMeshMaterial                = 0x5D
	recOpShapeSetHull                        = 0x5E
	recOpShapeSetMesh                        = 0x5F
	recOpCreateParallelJoint                 = 0x90
	recOpCreateDistanceJoint                 = 0x91
	recOpCreateFilterJoint                   = 0x92
	recOpCreateMotorJoint                    = 0x93
	recOpCreatePrismaticJoint                = 0x94
	recOpCreateRevoluteJoint                 = 0x95
	recOpCreateSphericalJoint                = 0x96
	recOpCreateWeldJoint                     = 0x97
	recOpCreateWheelJoint                    = 0x98
	recOpDestroyJoint                        = 0x99
	recOpJointSetLocalFrameA                 = 0x9A
	recOpJointSetLocalFrameB                 = 0x9B
	recOpJointSetCollideConnected            = 0x9C
	recOpJointWakeBodies                     = 0x9D
	recOpJointSetConstraintTuning            = 0x9E
	recOpJointSetForceThreshold              = 0x9F
	recOpJointSetTorqueThreshold             = 0xA0
	recOpParallelJointSetSpringHertz         = 0xA1
	recOpParallelJointSetSpringDampingRatio  = 0xA2
	recOpParallelJointSetMaxTorque           = 0xA3
	recOpDistanceJointSetLength              = 0xA4
	recOpDistanceJointEnableSpring           = 0xA5
	recOpDistanceJointSetSpringForceRange    = 0xA6
	recOpDistanceJointSetSpringHertz         = 0xA7
	recOpDistanceJointSetSpringDampingRatio  = 0xA8
	recOpDistanceJointEnableLimit            = 0xA9
	recOpDistanceJointSetLengthRange         = 0xAA
	recOpDistanceJointEnableMotor            = 0xAB
	recOpDistanceJointSetMotorSpeed          = 0xAC
	recOpDistanceJointSetMaxMotorForce       = 0xAD
	recOpMotorJointSetLinearVelocity         = 0xAE
	recOpMotorJointSetAngularVelocity        = 0xAF
	recOpMotorJointSetMaxVelocityForce       = 0xB0
	recOpMotorJointSetMaxVelocityTorque      = 0xB1
	recOpMotorJointSetLinearHertz            = 0xB2
	recOpMotorJointSetLinearDampingRatio     = 0xB3
	recOpMotorJointSetAngularHertz           = 0xB4
	recOpMotorJointSetAngularDampingRatio    = 0xB5
	recOpMotorJointSetMaxSpringForce         = 0xB6
	recOpMotorJointSetMaxSpringTorque        = 0xB7
	recOpPrismaticJointEnableSpring          = 0xB8
	recOpPrismaticJointSetSpringHertz        = 0xB9
	recOpPrismaticJointSetSpringDampingRatio = 0xBA
	recOpPrismaticJointSetTargetTranslation  = 0xBB
	recOpPrismaticJointEnableLimit           = 0xBC
	recOpPrismaticJointSetLimits             = 0xBD
	recOpPrismaticJointEnableMotor           = 0xBE
	recOpPrismaticJointSetMotorSpeed         = 0xBF
	recOpPrismaticJointSetMaxMotorForce      = 0xC0
	recOpRevoluteJointEnableSpring           = 0xC1
	recOpRevoluteJointSetSpringHertz         = 0xC2
	recOpRevoluteJointSetSpringDampingRatio  = 0xC3
	recOpRevoluteJointSetTargetAngle         = 0xC4
	recOpRevoluteJointEnableLimit            = 0xC5
	recOpRevoluteJointSetLimits              = 0xC6
	recOpRevoluteJointEnableMotor            = 0xC7
	recOpRevoluteJointSetMotorSpeed          = 0xC8
	recOpRevoluteJointSetMaxMotorTorque      = 0xC9
	recOpSphericalJointEnableConeLimit       = 0xCA
	recOpSphericalJointSetConeLimit          = 0xCB
	recOpSphericalJointEnableTwistLimit      = 0xCC
	recOpSphericalJointSetTwistLimits        = 0xCD
	recOpSphericalJointEnableSpring          = 0xCE
	recOpSphericalJointSetSpringHertz        = 0xCF
	recOpSphericalJointSetSpringDampingRatio = 0xD0
	recOpSphericalJointSetTargetRotation     = 0xD1
	recOpSphericalJointEnableMotor           = 0xD2
	recOpSphericalJointSetMotorVelocity      = 0xD3
	recOpSphericalJointSetMaxMotorTorque     = 0xD4
	recOpWeldJointSetLinearHertz             = 0xD5
	recOpWeldJointSetLinearDampingRatio      = 0xD6
	recOpWeldJointSetAngularHertz            = 0xD7
	recOpWeldJointSetAngularDampingRatio     = 0xD8
	recOpWheelJointEnableSuspension          = 0xD9
	recOpWheelJointSetSuspensionHertz        = 0xDA
	recOpWheelJointSetSuspensionDampingRatio = 0xDB
	recOpWheelJointEnableSuspensionLimit     = 0xDC
	recOpWheelJointSetSuspensionLimits       = 0xDD
	recOpWheelJointEnableSpinMotor           = 0xDE
	recOpWheelJointSetSpinMotorSpeed         = 0xDF
	recOpWheelJointSetMaxSpinTorque          = 0xE0
	recOpWheelJointEnableSteering            = 0xE1
	recOpWheelJointSetSteeringHertz          = 0xE2
	recOpWheelJointSetSteeringDampingRatio   = 0xE3
	recOpWheelJointSetMaxSteeringTorque      = 0xE4
	recOpWheelJointEnableSteeringLimit       = 0xE5
	recOpWheelJointSetSteeringLimits         = 0xE6
	recOpWheelJointSetTargetSteeringAngle    = 0xE7
	recOpQueryOverlapAABB                    = 0xE8
	recOpQueryOverlapShape                   = 0xE9
	recOpQueryCastRay                        = 0xEA
	recOpQueryCastShape                      = 0xEB
	recOpQueryCastRayClosest                 = 0xEC
	recOpQueryCastMover                      = 0xED
	recOpQueryCollideMover                   = 0xEE
	recOpQueryTag                            = 0xEF
	recOpStateHash                           = 0xF1
	recOpRecordingBounds                     = 0xF2
)

// ---- writer expansions (recording.c codegen pass 1b and the full writers) ----

// Codegen pass 1b: arg writers

// recWriteArgsDestroyWorld is b3RecWriteArgs_DestroyWorld.
func recWriteArgsDestroyWorld(rec *Recording, a *recArgsDestroyWorld) {
	recWWORLDID(&rec.buffer, a.world)
}

// recWriteArgsStep is b3RecWriteArgs_Step.
func recWriteArgsStep(rec *Recording, a *recArgsStep) {
	recWWORLDID(&rec.buffer, a.world)
	recWF32(&rec.buffer, a.dt)
	recWI32(&rec.buffer, a.subStepCount)
}

// recWriteArgsWorldEnableSleeping is b3RecWriteArgs_WorldEnableSleeping.
func recWriteArgsWorldEnableSleeping(rec *Recording, a *recArgsWorldEnableSleeping) {
	recWWORLDID(&rec.buffer, a.world)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWorldEnableContinuous is b3RecWriteArgs_WorldEnableContinuous.
func recWriteArgsWorldEnableContinuous(rec *Recording, a *recArgsWorldEnableContinuous) {
	recWWORLDID(&rec.buffer, a.world)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWorldSetRestitutionThreshold is b3RecWriteArgs_WorldSetRestitutionThreshold.
func recWriteArgsWorldSetRestitutionThreshold(rec *Recording, a *recArgsWorldSetRestitutionThreshold) {
	recWWORLDID(&rec.buffer, a.world)
	recWF32(&rec.buffer, a.value)
}

// recWriteArgsWorldSetHitEventThreshold is b3RecWriteArgs_WorldSetHitEventThreshold.
func recWriteArgsWorldSetHitEventThreshold(rec *Recording, a *recArgsWorldSetHitEventThreshold) {
	recWWORLDID(&rec.buffer, a.world)
	recWF32(&rec.buffer, a.value)
}

// recWriteArgsWorldSetGravity is b3RecWriteArgs_WorldSetGravity.
func recWriteArgsWorldSetGravity(rec *Recording, a *recArgsWorldSetGravity) {
	recWWORLDID(&rec.buffer, a.world)
	recWVEC3(&rec.buffer, a.gravity)
}

// recWriteArgsWorldExplode is b3RecWriteArgs_WorldExplode.
func recWriteArgsWorldExplode(rec *Recording, a *recArgsWorldExplode) {
	recWWORLDID(&rec.buffer, a.world)
	recWEXPLOSIONDEF(&rec.buffer, a.def)
}

// recWriteArgsWorldSetContactTuning is b3RecWriteArgs_WorldSetContactTuning.
func recWriteArgsWorldSetContactTuning(rec *Recording, a *recArgsWorldSetContactTuning) {
	recWWORLDID(&rec.buffer, a.world)
	recWF32(&rec.buffer, a.hertz)
	recWF32(&rec.buffer, a.dampingRatio)
	recWF32(&rec.buffer, a.contactSpeed)
}

// recWriteArgsWorldSetContactRecycleDistance is b3RecWriteArgs_WorldSetContactRecycleDistance.
func recWriteArgsWorldSetContactRecycleDistance(rec *Recording, a *recArgsWorldSetContactRecycleDistance) {
	recWWORLDID(&rec.buffer, a.world)
	recWF32(&rec.buffer, a.recycleDistance)
}

// recWriteArgsWorldSetMaximumLinearSpeed is b3RecWriteArgs_WorldSetMaximumLinearSpeed.
func recWriteArgsWorldSetMaximumLinearSpeed(rec *Recording, a *recArgsWorldSetMaximumLinearSpeed) {
	recWWORLDID(&rec.buffer, a.world)
	recWF32(&rec.buffer, a.maximumLinearSpeed)
}

// recWriteArgsWorldEnableWarmStarting is b3RecWriteArgs_WorldEnableWarmStarting.
func recWriteArgsWorldEnableWarmStarting(rec *Recording, a *recArgsWorldEnableWarmStarting) {
	recWWORLDID(&rec.buffer, a.world)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWorldRebuildStaticTree is b3RecWriteArgs_WorldRebuildStaticTree.
func recWriteArgsWorldRebuildStaticTree(rec *Recording, a *recArgsWorldRebuildStaticTree) {
	recWWORLDID(&rec.buffer, a.world)
}

// recWriteArgsWorldEnableSpeculative is b3RecWriteArgs_WorldEnableSpeculative.
func recWriteArgsWorldEnableSpeculative(rec *Recording, a *recArgsWorldEnableSpeculative) {
	recWWORLDID(&rec.buffer, a.world)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWorldSetRestitutionIterations is b3RecWriteArgs_WorldSetRestitutionIterations.
func recWriteArgsWorldSetRestitutionIterations(rec *Recording, a *recArgsWorldSetRestitutionIterations) {
	recWWORLDID(&rec.buffer, a.world)
	recWI32(&rec.buffer, a.iterations)
}

// recWriteArgsWorldEnableRestitutionPropagation is b3RecWriteArgs_WorldEnableRestitutionPropagation.
func recWriteArgsWorldEnableRestitutionPropagation(rec *Recording, a *recArgsWorldEnableRestitutionPropagation) {
	recWWORLDID(&rec.buffer, a.world)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWorldSetIsotropicInertiaTolerance is b3RecWriteArgs_WorldSetIsotropicInertiaTolerance.
func recWriteArgsWorldSetIsotropicInertiaTolerance(rec *Recording, a *recArgsWorldSetIsotropicInertiaTolerance) {
	recWWORLDID(&rec.buffer, a.world)
	recWF32(&rec.buffer, a.tolerance)
}

// recWriteArgsCreateBody is b3RecWriteArgs_CreateBody.
func recWriteArgsCreateBody(rec *Recording, a *recArgsCreateBody) {
	recWWORLDID(&rec.buffer, a.world)
	recWBODYDEF(&rec.buffer, a.def)
}

// recWriteArgsDestroyBody is b3RecWriteArgs_DestroyBody.
func recWriteArgsDestroyBody(rec *Recording, a *recArgsDestroyBody) {
	recWBODYID(&rec.buffer, a.body)
}

// recWriteArgsBodySetTransform is b3RecWriteArgs_BodySetTransform.
func recWriteArgsBodySetTransform(rec *Recording, a *recArgsBodySetTransform) {
	recWBODYID(&rec.buffer, a.body)
	recWPOSITION(&rec.buffer, a.position)
	recWQUAT(&rec.buffer, a.rotation)
}

// recWriteArgsBodySetLinearVelocity is b3RecWriteArgs_BodySetLinearVelocity.
func recWriteArgsBodySetLinearVelocity(rec *Recording, a *recArgsBodySetLinearVelocity) {
	recWBODYID(&rec.buffer, a.body)
	recWVEC3(&rec.buffer, a.v)
}

// recWriteArgsBodySetType is b3RecWriteArgs_BodySetType.
func recWriteArgsBodySetType(rec *Recording, a *recArgsBodySetType) {
	recWBODYID(&rec.buffer, a.body)
	recWI32(&rec.buffer, a.typ)
}

// recWriteArgsBodySetName is b3RecWriteArgs_BodySetName.
func recWriteArgsBodySetName(rec *Recording, a *recArgsBodySetName) {
	recWBODYID(&rec.buffer, a.body)
	recWSTR(&rec.buffer, a.name)
}

// recWriteArgsBodySetAngularVelocity is b3RecWriteArgs_BodySetAngularVelocity.
func recWriteArgsBodySetAngularVelocity(rec *Recording, a *recArgsBodySetAngularVelocity) {
	recWBODYID(&rec.buffer, a.body)
	recWVEC3(&rec.buffer, a.w)
}

// recWriteArgsBodySetTargetTransform is b3RecWriteArgs_BodySetTargetTransform.
func recWriteArgsBodySetTargetTransform(rec *Recording, a *recArgsBodySetTargetTransform) {
	recWBODYID(&rec.buffer, a.body)
	recWWORLDXF(&rec.buffer, a.target)
	recWF32(&rec.buffer, a.timeStep)
	recWBOOL(&rec.buffer, a.wake)
}

// recWriteArgsBodyApplyForce is b3RecWriteArgs_BodyApplyForce.
func recWriteArgsBodyApplyForce(rec *Recording, a *recArgsBodyApplyForce) {
	recWBODYID(&rec.buffer, a.body)
	recWVEC3(&rec.buffer, a.force)
	recWPOSITION(&rec.buffer, a.point)
	recWBOOL(&rec.buffer, a.wake)
}

// recWriteArgsBodyApplyForceToCenter is b3RecWriteArgs_BodyApplyForceToCenter.
func recWriteArgsBodyApplyForceToCenter(rec *Recording, a *recArgsBodyApplyForceToCenter) {
	recWBODYID(&rec.buffer, a.body)
	recWVEC3(&rec.buffer, a.force)
	recWBOOL(&rec.buffer, a.wake)
}

// recWriteArgsBodyApplyTorque is b3RecWriteArgs_BodyApplyTorque.
func recWriteArgsBodyApplyTorque(rec *Recording, a *recArgsBodyApplyTorque) {
	recWBODYID(&rec.buffer, a.body)
	recWVEC3(&rec.buffer, a.torque)
	recWBOOL(&rec.buffer, a.wake)
}

// recWriteArgsBodyApplyLinearImpulse is b3RecWriteArgs_BodyApplyLinearImpulse.
func recWriteArgsBodyApplyLinearImpulse(rec *Recording, a *recArgsBodyApplyLinearImpulse) {
	recWBODYID(&rec.buffer, a.body)
	recWVEC3(&rec.buffer, a.impulse)
	recWPOSITION(&rec.buffer, a.point)
	recWBOOL(&rec.buffer, a.wake)
}

// recWriteArgsBodyApplyLinearImpulseToCenter is b3RecWriteArgs_BodyApplyLinearImpulseToCenter.
func recWriteArgsBodyApplyLinearImpulseToCenter(rec *Recording, a *recArgsBodyApplyLinearImpulseToCenter) {
	recWBODYID(&rec.buffer, a.body)
	recWVEC3(&rec.buffer, a.impulse)
	recWBOOL(&rec.buffer, a.wake)
}

// recWriteArgsBodyApplyAngularImpulse is b3RecWriteArgs_BodyApplyAngularImpulse.
func recWriteArgsBodyApplyAngularImpulse(rec *Recording, a *recArgsBodyApplyAngularImpulse) {
	recWBODYID(&rec.buffer, a.body)
	recWVEC3(&rec.buffer, a.impulse)
	recWBOOL(&rec.buffer, a.wake)
}

// recWriteArgsBodySetMassData is b3RecWriteArgs_BodySetMassData.
func recWriteArgsBodySetMassData(rec *Recording, a *recArgsBodySetMassData) {
	recWBODYID(&rec.buffer, a.body)
	recWMASSDATA(&rec.buffer, a.massData)
}

// recWriteArgsBodyApplyMassFromShapes is b3RecWriteArgs_BodyApplyMassFromShapes.
func recWriteArgsBodyApplyMassFromShapes(rec *Recording, a *recArgsBodyApplyMassFromShapes) {
	recWBODYID(&rec.buffer, a.body)
}

// recWriteArgsBodySetLinearDamping is b3RecWriteArgs_BodySetLinearDamping.
func recWriteArgsBodySetLinearDamping(rec *Recording, a *recArgsBodySetLinearDamping) {
	recWBODYID(&rec.buffer, a.body)
	recWF32(&rec.buffer, a.damping)
}

// recWriteArgsBodySetAngularDamping is b3RecWriteArgs_BodySetAngularDamping.
func recWriteArgsBodySetAngularDamping(rec *Recording, a *recArgsBodySetAngularDamping) {
	recWBODYID(&rec.buffer, a.body)
	recWF32(&rec.buffer, a.damping)
}

// recWriteArgsBodySetGravityScale is b3RecWriteArgs_BodySetGravityScale.
func recWriteArgsBodySetGravityScale(rec *Recording, a *recArgsBodySetGravityScale) {
	recWBODYID(&rec.buffer, a.body)
	recWF32(&rec.buffer, a.scale)
}

// recWriteArgsBodySetAwake is b3RecWriteArgs_BodySetAwake.
func recWriteArgsBodySetAwake(rec *Recording, a *recArgsBodySetAwake) {
	recWBODYID(&rec.buffer, a.body)
	recWBOOL(&rec.buffer, a.awake)
}

// recWriteArgsBodyEnableSleep is b3RecWriteArgs_BodyEnableSleep.
func recWriteArgsBodyEnableSleep(rec *Recording, a *recArgsBodyEnableSleep) {
	recWBODYID(&rec.buffer, a.body)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsBodySetSleepThreshold is b3RecWriteArgs_BodySetSleepThreshold.
func recWriteArgsBodySetSleepThreshold(rec *Recording, a *recArgsBodySetSleepThreshold) {
	recWBODYID(&rec.buffer, a.body)
	recWF32(&rec.buffer, a.threshold)
}

// recWriteArgsBodyDisable is b3RecWriteArgs_BodyDisable.
func recWriteArgsBodyDisable(rec *Recording, a *recArgsBodyDisable) {
	recWBODYID(&rec.buffer, a.body)
}

// recWriteArgsBodyEnable is b3RecWriteArgs_BodyEnable.
func recWriteArgsBodyEnable(rec *Recording, a *recArgsBodyEnable) {
	recWBODYID(&rec.buffer, a.body)
}

// recWriteArgsBodySetMotionLocks is b3RecWriteArgs_BodySetMotionLocks.
func recWriteArgsBodySetMotionLocks(rec *Recording, a *recArgsBodySetMotionLocks) {
	recWBODYID(&rec.buffer, a.body)
	recWLOCKS(&rec.buffer, a.locks)
}

// recWriteArgsBodySetBullet is b3RecWriteArgs_BodySetBullet.
func recWriteArgsBodySetBullet(rec *Recording, a *recArgsBodySetBullet) {
	recWBODYID(&rec.buffer, a.body)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsBodyEnableContactRecycling is b3RecWriteArgs_BodyEnableContactRecycling.
func recWriteArgsBodyEnableContactRecycling(rec *Recording, a *recArgsBodyEnableContactRecycling) {
	recWBODYID(&rec.buffer, a.body)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsBodyEnableHitEvents is b3RecWriteArgs_BodyEnableHitEvents.
func recWriteArgsBodyEnableHitEvents(rec *Recording, a *recArgsBodyEnableHitEvents) {
	recWBODYID(&rec.buffer, a.body)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsBodyAllowFastRotation is b3RecWriteArgs_BodyAllowFastRotation.
func recWriteArgsBodyAllowFastRotation(rec *Recording, a *recArgsBodyAllowFastRotation) {
	recWBODYID(&rec.buffer, a.body)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsBodySetSafetyFactor is b3RecWriteArgs_BodySetSafetyFactor.
func recWriteArgsBodySetSafetyFactor(rec *Recording, a *recArgsBodySetSafetyFactor) {
	recWBODYID(&rec.buffer, a.body)
	recWF32(&rec.buffer, a.value)
}

// recWriteArgsCreateSphereShape is b3RecWriteArgs_CreateSphereShape.
func recWriteArgsCreateSphereShape(rec *Recording, a *recArgsCreateSphereShape) {
	recWBODYID(&rec.buffer, a.body)
	recWSHAPEDEF(&rec.buffer, a.def)
	recWSPHERE(&rec.buffer, a.sphere)
}

// recWriteArgsCreateCapsuleShape is b3RecWriteArgs_CreateCapsuleShape.
func recWriteArgsCreateCapsuleShape(rec *Recording, a *recArgsCreateCapsuleShape) {
	recWBODYID(&rec.buffer, a.body)
	recWSHAPEDEF(&rec.buffer, a.def)
	recWCAPSULE(&rec.buffer, a.capsule)
}

// recWriteArgsCreateHullShape is b3RecWriteArgs_CreateHullShape.
func recWriteArgsCreateHullShape(rec *Recording, a *recArgsCreateHullShape) {
	recWBODYID(&rec.buffer, a.body)
	recWSHAPEDEF(&rec.buffer, a.def)
	recWGEOMID(&rec.buffer, a.geometryId)
}

// recWriteArgsCreateMeshShape is b3RecWriteArgs_CreateMeshShape.
func recWriteArgsCreateMeshShape(rec *Recording, a *recArgsCreateMeshShape) {
	recWBODYID(&rec.buffer, a.body)
	recWSHAPEDEF(&rec.buffer, a.def)
	recWGEOMID(&rec.buffer, a.geometryId)
	recWVEC3(&rec.buffer, a.scale)
}

// recWriteArgsCreateHeightFieldShape is b3RecWriteArgs_CreateHeightFieldShape.
func recWriteArgsCreateHeightFieldShape(rec *Recording, a *recArgsCreateHeightFieldShape) {
	recWBODYID(&rec.buffer, a.body)
	recWSHAPEDEF(&rec.buffer, a.def)
	recWGEOMID(&rec.buffer, a.geometryId)
}

// recWriteArgsCreateCompoundShape is b3RecWriteArgs_CreateCompoundShape.
func recWriteArgsCreateCompoundShape(rec *Recording, a *recArgsCreateCompoundShape) {
	recWBODYID(&rec.buffer, a.body)
	recWSHAPEDEF(&rec.buffer, a.def)
	recWGEOMID(&rec.buffer, a.geometryId)
}

// recWriteArgsDestroyShape is b3RecWriteArgs_DestroyShape.
func recWriteArgsDestroyShape(rec *Recording, a *recArgsDestroyShape) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWBOOL(&rec.buffer, a.updateBodyMass)
}

// recWriteArgsShapeSetDensity is b3RecWriteArgs_ShapeSetDensity.
func recWriteArgsShapeSetDensity(rec *Recording, a *recArgsShapeSetDensity) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWF32(&rec.buffer, a.density)
	recWBOOL(&rec.buffer, a.updateBodyMass)
}

// recWriteArgsShapeSetFriction is b3RecWriteArgs_ShapeSetFriction.
func recWriteArgsShapeSetFriction(rec *Recording, a *recArgsShapeSetFriction) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWF32(&rec.buffer, a.friction)
}

// recWriteArgsShapeSetRestitution is b3RecWriteArgs_ShapeSetRestitution.
func recWriteArgsShapeSetRestitution(rec *Recording, a *recArgsShapeSetRestitution) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWF32(&rec.buffer, a.restitution)
}

// recWriteArgsShapeSetSurfaceMaterial is b3RecWriteArgs_ShapeSetSurfaceMaterial.
func recWriteArgsShapeSetSurfaceMaterial(rec *Recording, a *recArgsShapeSetSurfaceMaterial) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWMATERIAL(&rec.buffer, a.material)
}

// recWriteArgsShapeSetFilter is b3RecWriteArgs_ShapeSetFilter.
func recWriteArgsShapeSetFilter(rec *Recording, a *recArgsShapeSetFilter) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWFILTER(&rec.buffer, a.filter)
	recWBOOL(&rec.buffer, a.invokeContacts)
}

// recWriteArgsShapeEnableSensorEvents is b3RecWriteArgs_ShapeEnableSensorEvents.
func recWriteArgsShapeEnableSensorEvents(rec *Recording, a *recArgsShapeEnableSensorEvents) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsShapeEnableContactEvents is b3RecWriteArgs_ShapeEnableContactEvents.
func recWriteArgsShapeEnableContactEvents(rec *Recording, a *recArgsShapeEnableContactEvents) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsShapeEnablePreSolveEvents is b3RecWriteArgs_ShapeEnablePreSolveEvents.
func recWriteArgsShapeEnablePreSolveEvents(rec *Recording, a *recArgsShapeEnablePreSolveEvents) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsShapeEnableHitEvents is b3RecWriteArgs_ShapeEnableHitEvents.
func recWriteArgsShapeEnableHitEvents(rec *Recording, a *recArgsShapeEnableHitEvents) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsShapeSetSphere is b3RecWriteArgs_ShapeSetSphere.
func recWriteArgsShapeSetSphere(rec *Recording, a *recArgsShapeSetSphere) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWSPHERE(&rec.buffer, a.sphere)
}

// recWriteArgsShapeSetCapsule is b3RecWriteArgs_ShapeSetCapsule.
func recWriteArgsShapeSetCapsule(rec *Recording, a *recArgsShapeSetCapsule) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWCAPSULE(&rec.buffer, a.capsule)
}

// recWriteArgsShapeApplyWind is b3RecWriteArgs_ShapeApplyWind.
func recWriteArgsShapeApplyWind(rec *Recording, a *recArgsShapeApplyWind) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWVEC3(&rec.buffer, a.wind)
	recWF32(&rec.buffer, a.drag)
	recWF32(&rec.buffer, a.lift)
	recWF32(&rec.buffer, a.maxSpeed)
	recWBOOL(&rec.buffer, a.wake)
}

// recWriteArgsShapeSetName is b3RecWriteArgs_ShapeSetName.
func recWriteArgsShapeSetName(rec *Recording, a *recArgsShapeSetName) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWSTR(&rec.buffer, a.name)
}

// recWriteArgsShapeSetMeshMaterial is b3RecWriteArgs_ShapeSetMeshMaterial.
func recWriteArgsShapeSetMeshMaterial(rec *Recording, a *recArgsShapeSetMeshMaterial) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWMATERIAL(&rec.buffer, a.material)
	recWI32(&rec.buffer, a.index)
}

// recWriteArgsShapeSetHull is b3RecWriteArgs_ShapeSetHull.
func recWriteArgsShapeSetHull(rec *Recording, a *recArgsShapeSetHull) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWGEOMID(&rec.buffer, a.geometryId)
}

// recWriteArgsShapeSetMesh is b3RecWriteArgs_ShapeSetMesh.
func recWriteArgsShapeSetMesh(rec *Recording, a *recArgsShapeSetMesh) {
	recWSHAPEID(&rec.buffer, a.shape)
	recWGEOMID(&rec.buffer, a.geometryId)
	recWVEC3(&rec.buffer, a.scale)
}

// recWriteArgsCreateParallelJoint is b3RecWriteArgs_CreateParallelJoint.
func recWriteArgsCreateParallelJoint(rec *Recording, a *recArgsCreateParallelJoint) {
	recWWORLDID(&rec.buffer, a.world)
	recWPARALLELJOINTDEF(&rec.buffer, a.def)
}

// recWriteArgsCreateDistanceJoint is b3RecWriteArgs_CreateDistanceJoint.
func recWriteArgsCreateDistanceJoint(rec *Recording, a *recArgsCreateDistanceJoint) {
	recWWORLDID(&rec.buffer, a.world)
	recWDISTANCEJOINTDEF(&rec.buffer, a.def)
}

// recWriteArgsCreateFilterJoint is b3RecWriteArgs_CreateFilterJoint.
func recWriteArgsCreateFilterJoint(rec *Recording, a *recArgsCreateFilterJoint) {
	recWWORLDID(&rec.buffer, a.world)
	recWFILTERJOINTDEF(&rec.buffer, a.def)
}

// recWriteArgsCreateMotorJoint is b3RecWriteArgs_CreateMotorJoint.
func recWriteArgsCreateMotorJoint(rec *Recording, a *recArgsCreateMotorJoint) {
	recWWORLDID(&rec.buffer, a.world)
	recWMOTORJOINTDEF(&rec.buffer, a.def)
}

// recWriteArgsCreatePrismaticJoint is b3RecWriteArgs_CreatePrismaticJoint.
func recWriteArgsCreatePrismaticJoint(rec *Recording, a *recArgsCreatePrismaticJoint) {
	recWWORLDID(&rec.buffer, a.world)
	recWPRISMATICJOINTDEF(&rec.buffer, a.def)
}

// recWriteArgsCreateRevoluteJoint is b3RecWriteArgs_CreateRevoluteJoint.
func recWriteArgsCreateRevoluteJoint(rec *Recording, a *recArgsCreateRevoluteJoint) {
	recWWORLDID(&rec.buffer, a.world)
	recWREVOLUTEJOINTDEF(&rec.buffer, a.def)
}

// recWriteArgsCreateSphericalJoint is b3RecWriteArgs_CreateSphericalJoint.
func recWriteArgsCreateSphericalJoint(rec *Recording, a *recArgsCreateSphericalJoint) {
	recWWORLDID(&rec.buffer, a.world)
	recWSPHERICALJOINTDEF(&rec.buffer, a.def)
}

// recWriteArgsCreateWeldJoint is b3RecWriteArgs_CreateWeldJoint.
func recWriteArgsCreateWeldJoint(rec *Recording, a *recArgsCreateWeldJoint) {
	recWWORLDID(&rec.buffer, a.world)
	recWWELDJOINTDEF(&rec.buffer, a.def)
}

// recWriteArgsCreateWheelJoint is b3RecWriteArgs_CreateWheelJoint.
func recWriteArgsCreateWheelJoint(rec *Recording, a *recArgsCreateWheelJoint) {
	recWWORLDID(&rec.buffer, a.world)
	recWWHEELJOINTDEF(&rec.buffer, a.def)
}

// recWriteArgsDestroyJoint is b3RecWriteArgs_DestroyJoint.
func recWriteArgsDestroyJoint(rec *Recording, a *recArgsDestroyJoint) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.wakeAttached)
}

// recWriteArgsJointSetLocalFrameA is b3RecWriteArgs_JointSetLocalFrameA.
func recWriteArgsJointSetLocalFrameA(rec *Recording, a *recArgsJointSetLocalFrameA) {
	recWJOINTID(&rec.buffer, a.joint)
	recWTRANSFORM(&rec.buffer, a.localFrame)
}

// recWriteArgsJointSetLocalFrameB is b3RecWriteArgs_JointSetLocalFrameB.
func recWriteArgsJointSetLocalFrameB(rec *Recording, a *recArgsJointSetLocalFrameB) {
	recWJOINTID(&rec.buffer, a.joint)
	recWTRANSFORM(&rec.buffer, a.localFrame)
}

// recWriteArgsJointSetCollideConnected is b3RecWriteArgs_JointSetCollideConnected.
func recWriteArgsJointSetCollideConnected(rec *Recording, a *recArgsJointSetCollideConnected) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.shouldCollide)
}

// recWriteArgsJointWakeBodies is b3RecWriteArgs_JointWakeBodies.
func recWriteArgsJointWakeBodies(rec *Recording, a *recArgsJointWakeBodies) {
	recWJOINTID(&rec.buffer, a.joint)
}

// recWriteArgsJointSetConstraintTuning is b3RecWriteArgs_JointSetConstraintTuning.
func recWriteArgsJointSetConstraintTuning(rec *Recording, a *recArgsJointSetConstraintTuning) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsJointSetForceThreshold is b3RecWriteArgs_JointSetForceThreshold.
func recWriteArgsJointSetForceThreshold(rec *Recording, a *recArgsJointSetForceThreshold) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.threshold)
}

// recWriteArgsJointSetTorqueThreshold is b3RecWriteArgs_JointSetTorqueThreshold.
func recWriteArgsJointSetTorqueThreshold(rec *Recording, a *recArgsJointSetTorqueThreshold) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.threshold)
}

// recWriteArgsParallelJointSetSpringHertz is b3RecWriteArgs_ParallelJointSetSpringHertz.
func recWriteArgsParallelJointSetSpringHertz(rec *Recording, a *recArgsParallelJointSetSpringHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsParallelJointSetSpringDampingRatio is b3RecWriteArgs_ParallelJointSetSpringDampingRatio.
func recWriteArgsParallelJointSetSpringDampingRatio(rec *Recording, a *recArgsParallelJointSetSpringDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsParallelJointSetMaxTorque is b3RecWriteArgs_ParallelJointSetMaxTorque.
func recWriteArgsParallelJointSetMaxTorque(rec *Recording, a *recArgsParallelJointSetMaxTorque) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.maxTorque)
}

// recWriteArgsDistanceJointSetLength is b3RecWriteArgs_DistanceJointSetLength.
func recWriteArgsDistanceJointSetLength(rec *Recording, a *recArgsDistanceJointSetLength) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.length)
}

// recWriteArgsDistanceJointEnableSpring is b3RecWriteArgs_DistanceJointEnableSpring.
func recWriteArgsDistanceJointEnableSpring(rec *Recording, a *recArgsDistanceJointEnableSpring) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableSpring)
}

// recWriteArgsDistanceJointSetSpringForceRange is b3RecWriteArgs_DistanceJointSetSpringForceRange.
func recWriteArgsDistanceJointSetSpringForceRange(rec *Recording, a *recArgsDistanceJointSetSpringForceRange) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.lowerForce)
	recWF32(&rec.buffer, a.upperForce)
}

// recWriteArgsDistanceJointSetSpringHertz is b3RecWriteArgs_DistanceJointSetSpringHertz.
func recWriteArgsDistanceJointSetSpringHertz(rec *Recording, a *recArgsDistanceJointSetSpringHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsDistanceJointSetSpringDampingRatio is b3RecWriteArgs_DistanceJointSetSpringDampingRatio.
func recWriteArgsDistanceJointSetSpringDampingRatio(rec *Recording, a *recArgsDistanceJointSetSpringDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsDistanceJointEnableLimit is b3RecWriteArgs_DistanceJointEnableLimit.
func recWriteArgsDistanceJointEnableLimit(rec *Recording, a *recArgsDistanceJointEnableLimit) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableLimit)
}

// recWriteArgsDistanceJointSetLengthRange is b3RecWriteArgs_DistanceJointSetLengthRange.
func recWriteArgsDistanceJointSetLengthRange(rec *Recording, a *recArgsDistanceJointSetLengthRange) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.minLength)
	recWF32(&rec.buffer, a.maxLength)
}

// recWriteArgsDistanceJointEnableMotor is b3RecWriteArgs_DistanceJointEnableMotor.
func recWriteArgsDistanceJointEnableMotor(rec *Recording, a *recArgsDistanceJointEnableMotor) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableMotor)
}

// recWriteArgsDistanceJointSetMotorSpeed is b3RecWriteArgs_DistanceJointSetMotorSpeed.
func recWriteArgsDistanceJointSetMotorSpeed(rec *Recording, a *recArgsDistanceJointSetMotorSpeed) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.motorSpeed)
}

// recWriteArgsDistanceJointSetMaxMotorForce is b3RecWriteArgs_DistanceJointSetMaxMotorForce.
func recWriteArgsDistanceJointSetMaxMotorForce(rec *Recording, a *recArgsDistanceJointSetMaxMotorForce) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.force)
}

// recWriteArgsMotorJointSetLinearVelocity is b3RecWriteArgs_MotorJointSetLinearVelocity.
func recWriteArgsMotorJointSetLinearVelocity(rec *Recording, a *recArgsMotorJointSetLinearVelocity) {
	recWJOINTID(&rec.buffer, a.joint)
	recWVEC3(&rec.buffer, a.velocity)
}

// recWriteArgsMotorJointSetAngularVelocity is b3RecWriteArgs_MotorJointSetAngularVelocity.
func recWriteArgsMotorJointSetAngularVelocity(rec *Recording, a *recArgsMotorJointSetAngularVelocity) {
	recWJOINTID(&rec.buffer, a.joint)
	recWVEC3(&rec.buffer, a.velocity)
}

// recWriteArgsMotorJointSetMaxVelocityForce is b3RecWriteArgs_MotorJointSetMaxVelocityForce.
func recWriteArgsMotorJointSetMaxVelocityForce(rec *Recording, a *recArgsMotorJointSetMaxVelocityForce) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.maxForce)
}

// recWriteArgsMotorJointSetMaxVelocityTorque is b3RecWriteArgs_MotorJointSetMaxVelocityTorque.
func recWriteArgsMotorJointSetMaxVelocityTorque(rec *Recording, a *recArgsMotorJointSetMaxVelocityTorque) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.maxTorque)
}

// recWriteArgsMotorJointSetLinearHertz is b3RecWriteArgs_MotorJointSetLinearHertz.
func recWriteArgsMotorJointSetLinearHertz(rec *Recording, a *recArgsMotorJointSetLinearHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsMotorJointSetLinearDampingRatio is b3RecWriteArgs_MotorJointSetLinearDampingRatio.
func recWriteArgsMotorJointSetLinearDampingRatio(rec *Recording, a *recArgsMotorJointSetLinearDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.damping)
}

// recWriteArgsMotorJointSetAngularHertz is b3RecWriteArgs_MotorJointSetAngularHertz.
func recWriteArgsMotorJointSetAngularHertz(rec *Recording, a *recArgsMotorJointSetAngularHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsMotorJointSetAngularDampingRatio is b3RecWriteArgs_MotorJointSetAngularDampingRatio.
func recWriteArgsMotorJointSetAngularDampingRatio(rec *Recording, a *recArgsMotorJointSetAngularDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.damping)
}

// recWriteArgsMotorJointSetMaxSpringForce is b3RecWriteArgs_MotorJointSetMaxSpringForce.
func recWriteArgsMotorJointSetMaxSpringForce(rec *Recording, a *recArgsMotorJointSetMaxSpringForce) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.maxForce)
}

// recWriteArgsMotorJointSetMaxSpringTorque is b3RecWriteArgs_MotorJointSetMaxSpringTorque.
func recWriteArgsMotorJointSetMaxSpringTorque(rec *Recording, a *recArgsMotorJointSetMaxSpringTorque) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.maxTorque)
}

// recWriteArgsPrismaticJointEnableSpring is b3RecWriteArgs_PrismaticJointEnableSpring.
func recWriteArgsPrismaticJointEnableSpring(rec *Recording, a *recArgsPrismaticJointEnableSpring) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableSpring)
}

// recWriteArgsPrismaticJointSetSpringHertz is b3RecWriteArgs_PrismaticJointSetSpringHertz.
func recWriteArgsPrismaticJointSetSpringHertz(rec *Recording, a *recArgsPrismaticJointSetSpringHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsPrismaticJointSetSpringDampingRatio is b3RecWriteArgs_PrismaticJointSetSpringDampingRatio.
func recWriteArgsPrismaticJointSetSpringDampingRatio(rec *Recording, a *recArgsPrismaticJointSetSpringDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsPrismaticJointSetTargetTranslation is b3RecWriteArgs_PrismaticJointSetTargetTranslation.
func recWriteArgsPrismaticJointSetTargetTranslation(rec *Recording, a *recArgsPrismaticJointSetTargetTranslation) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.translation)
}

// recWriteArgsPrismaticJointEnableLimit is b3RecWriteArgs_PrismaticJointEnableLimit.
func recWriteArgsPrismaticJointEnableLimit(rec *Recording, a *recArgsPrismaticJointEnableLimit) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableLimit)
}

// recWriteArgsPrismaticJointSetLimits is b3RecWriteArgs_PrismaticJointSetLimits.
func recWriteArgsPrismaticJointSetLimits(rec *Recording, a *recArgsPrismaticJointSetLimits) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.lower)
	recWF32(&rec.buffer, a.upper)
}

// recWriteArgsPrismaticJointEnableMotor is b3RecWriteArgs_PrismaticJointEnableMotor.
func recWriteArgsPrismaticJointEnableMotor(rec *Recording, a *recArgsPrismaticJointEnableMotor) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableMotor)
}

// recWriteArgsPrismaticJointSetMotorSpeed is b3RecWriteArgs_PrismaticJointSetMotorSpeed.
func recWriteArgsPrismaticJointSetMotorSpeed(rec *Recording, a *recArgsPrismaticJointSetMotorSpeed) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.motorSpeed)
}

// recWriteArgsPrismaticJointSetMaxMotorForce is b3RecWriteArgs_PrismaticJointSetMaxMotorForce.
func recWriteArgsPrismaticJointSetMaxMotorForce(rec *Recording, a *recArgsPrismaticJointSetMaxMotorForce) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.force)
}

// recWriteArgsRevoluteJointEnableSpring is b3RecWriteArgs_RevoluteJointEnableSpring.
func recWriteArgsRevoluteJointEnableSpring(rec *Recording, a *recArgsRevoluteJointEnableSpring) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableSpring)
}

// recWriteArgsRevoluteJointSetSpringHertz is b3RecWriteArgs_RevoluteJointSetSpringHertz.
func recWriteArgsRevoluteJointSetSpringHertz(rec *Recording, a *recArgsRevoluteJointSetSpringHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsRevoluteJointSetSpringDampingRatio is b3RecWriteArgs_RevoluteJointSetSpringDampingRatio.
func recWriteArgsRevoluteJointSetSpringDampingRatio(rec *Recording, a *recArgsRevoluteJointSetSpringDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsRevoluteJointSetTargetAngle is b3RecWriteArgs_RevoluteJointSetTargetAngle.
func recWriteArgsRevoluteJointSetTargetAngle(rec *Recording, a *recArgsRevoluteJointSetTargetAngle) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.angle)
}

// recWriteArgsRevoluteJointEnableLimit is b3RecWriteArgs_RevoluteJointEnableLimit.
func recWriteArgsRevoluteJointEnableLimit(rec *Recording, a *recArgsRevoluteJointEnableLimit) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableLimit)
}

// recWriteArgsRevoluteJointSetLimits is b3RecWriteArgs_RevoluteJointSetLimits.
func recWriteArgsRevoluteJointSetLimits(rec *Recording, a *recArgsRevoluteJointSetLimits) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.lower)
	recWF32(&rec.buffer, a.upper)
}

// recWriteArgsRevoluteJointEnableMotor is b3RecWriteArgs_RevoluteJointEnableMotor.
func recWriteArgsRevoluteJointEnableMotor(rec *Recording, a *recArgsRevoluteJointEnableMotor) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableMotor)
}

// recWriteArgsRevoluteJointSetMotorSpeed is b3RecWriteArgs_RevoluteJointSetMotorSpeed.
func recWriteArgsRevoluteJointSetMotorSpeed(rec *Recording, a *recArgsRevoluteJointSetMotorSpeed) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.motorSpeed)
}

// recWriteArgsRevoluteJointSetMaxMotorTorque is b3RecWriteArgs_RevoluteJointSetMaxMotorTorque.
func recWriteArgsRevoluteJointSetMaxMotorTorque(rec *Recording, a *recArgsRevoluteJointSetMaxMotorTorque) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.torque)
}

// recWriteArgsSphericalJointEnableConeLimit is b3RecWriteArgs_SphericalJointEnableConeLimit.
func recWriteArgsSphericalJointEnableConeLimit(rec *Recording, a *recArgsSphericalJointEnableConeLimit) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableLimit)
}

// recWriteArgsSphericalJointSetConeLimit is b3RecWriteArgs_SphericalJointSetConeLimit.
func recWriteArgsSphericalJointSetConeLimit(rec *Recording, a *recArgsSphericalJointSetConeLimit) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.angleRadians)
}

// recWriteArgsSphericalJointEnableTwistLimit is b3RecWriteArgs_SphericalJointEnableTwistLimit.
func recWriteArgsSphericalJointEnableTwistLimit(rec *Recording, a *recArgsSphericalJointEnableTwistLimit) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableLimit)
}

// recWriteArgsSphericalJointSetTwistLimits is b3RecWriteArgs_SphericalJointSetTwistLimits.
func recWriteArgsSphericalJointSetTwistLimits(rec *Recording, a *recArgsSphericalJointSetTwistLimits) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.lower)
	recWF32(&rec.buffer, a.upper)
}

// recWriteArgsSphericalJointEnableSpring is b3RecWriteArgs_SphericalJointEnableSpring.
func recWriteArgsSphericalJointEnableSpring(rec *Recording, a *recArgsSphericalJointEnableSpring) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableSpring)
}

// recWriteArgsSphericalJointSetSpringHertz is b3RecWriteArgs_SphericalJointSetSpringHertz.
func recWriteArgsSphericalJointSetSpringHertz(rec *Recording, a *recArgsSphericalJointSetSpringHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsSphericalJointSetSpringDampingRatio is b3RecWriteArgs_SphericalJointSetSpringDampingRatio.
func recWriteArgsSphericalJointSetSpringDampingRatio(rec *Recording, a *recArgsSphericalJointSetSpringDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsSphericalJointSetTargetRotation is b3RecWriteArgs_SphericalJointSetTargetRotation.
func recWriteArgsSphericalJointSetTargetRotation(rec *Recording, a *recArgsSphericalJointSetTargetRotation) {
	recWJOINTID(&rec.buffer, a.joint)
	recWQUAT(&rec.buffer, a.targetRotation)
}

// recWriteArgsSphericalJointEnableMotor is b3RecWriteArgs_SphericalJointEnableMotor.
func recWriteArgsSphericalJointEnableMotor(rec *Recording, a *recArgsSphericalJointEnableMotor) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.enableMotor)
}

// recWriteArgsSphericalJointSetMotorVelocity is b3RecWriteArgs_SphericalJointSetMotorVelocity.
func recWriteArgsSphericalJointSetMotorVelocity(rec *Recording, a *recArgsSphericalJointSetMotorVelocity) {
	recWJOINTID(&rec.buffer, a.joint)
	recWVEC3(&rec.buffer, a.motorVelocity)
}

// recWriteArgsSphericalJointSetMaxMotorTorque is b3RecWriteArgs_SphericalJointSetMaxMotorTorque.
func recWriteArgsSphericalJointSetMaxMotorTorque(rec *Recording, a *recArgsSphericalJointSetMaxMotorTorque) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.torque)
}

// recWriteArgsWeldJointSetLinearHertz is b3RecWriteArgs_WeldJointSetLinearHertz.
func recWriteArgsWeldJointSetLinearHertz(rec *Recording, a *recArgsWeldJointSetLinearHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsWeldJointSetLinearDampingRatio is b3RecWriteArgs_WeldJointSetLinearDampingRatio.
func recWriteArgsWeldJointSetLinearDampingRatio(rec *Recording, a *recArgsWeldJointSetLinearDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsWeldJointSetAngularHertz is b3RecWriteArgs_WeldJointSetAngularHertz.
func recWriteArgsWeldJointSetAngularHertz(rec *Recording, a *recArgsWeldJointSetAngularHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsWeldJointSetAngularDampingRatio is b3RecWriteArgs_WeldJointSetAngularDampingRatio.
func recWriteArgsWeldJointSetAngularDampingRatio(rec *Recording, a *recArgsWeldJointSetAngularDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsWheelJointEnableSuspension is b3RecWriteArgs_WheelJointEnableSuspension.
func recWriteArgsWheelJointEnableSuspension(rec *Recording, a *recArgsWheelJointEnableSuspension) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWheelJointSetSuspensionHertz is b3RecWriteArgs_WheelJointSetSuspensionHertz.
func recWriteArgsWheelJointSetSuspensionHertz(rec *Recording, a *recArgsWheelJointSetSuspensionHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsWheelJointSetSuspensionDampingRatio is b3RecWriteArgs_WheelJointSetSuspensionDampingRatio.
func recWriteArgsWheelJointSetSuspensionDampingRatio(rec *Recording, a *recArgsWheelJointSetSuspensionDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsWheelJointEnableSuspensionLimit is b3RecWriteArgs_WheelJointEnableSuspensionLimit.
func recWriteArgsWheelJointEnableSuspensionLimit(rec *Recording, a *recArgsWheelJointEnableSuspensionLimit) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWheelJointSetSuspensionLimits is b3RecWriteArgs_WheelJointSetSuspensionLimits.
func recWriteArgsWheelJointSetSuspensionLimits(rec *Recording, a *recArgsWheelJointSetSuspensionLimits) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.lower)
	recWF32(&rec.buffer, a.upper)
}

// recWriteArgsWheelJointEnableSpinMotor is b3RecWriteArgs_WheelJointEnableSpinMotor.
func recWriteArgsWheelJointEnableSpinMotor(rec *Recording, a *recArgsWheelJointEnableSpinMotor) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWheelJointSetSpinMotorSpeed is b3RecWriteArgs_WheelJointSetSpinMotorSpeed.
func recWriteArgsWheelJointSetSpinMotorSpeed(rec *Recording, a *recArgsWheelJointSetSpinMotorSpeed) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.speed)
}

// recWriteArgsWheelJointSetMaxSpinTorque is b3RecWriteArgs_WheelJointSetMaxSpinTorque.
func recWriteArgsWheelJointSetMaxSpinTorque(rec *Recording, a *recArgsWheelJointSetMaxSpinTorque) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.torque)
}

// recWriteArgsWheelJointEnableSteering is b3RecWriteArgs_WheelJointEnableSteering.
func recWriteArgsWheelJointEnableSteering(rec *Recording, a *recArgsWheelJointEnableSteering) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWheelJointSetSteeringHertz is b3RecWriteArgs_WheelJointSetSteeringHertz.
func recWriteArgsWheelJointSetSteeringHertz(rec *Recording, a *recArgsWheelJointSetSteeringHertz) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.hertz)
}

// recWriteArgsWheelJointSetSteeringDampingRatio is b3RecWriteArgs_WheelJointSetSteeringDampingRatio.
func recWriteArgsWheelJointSetSteeringDampingRatio(rec *Recording, a *recArgsWheelJointSetSteeringDampingRatio) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.dampingRatio)
}

// recWriteArgsWheelJointSetMaxSteeringTorque is b3RecWriteArgs_WheelJointSetMaxSteeringTorque.
func recWriteArgsWheelJointSetMaxSteeringTorque(rec *Recording, a *recArgsWheelJointSetMaxSteeringTorque) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.torque)
}

// recWriteArgsWheelJointEnableSteeringLimit is b3RecWriteArgs_WheelJointEnableSteeringLimit.
func recWriteArgsWheelJointEnableSteeringLimit(rec *Recording, a *recArgsWheelJointEnableSteeringLimit) {
	recWJOINTID(&rec.buffer, a.joint)
	recWBOOL(&rec.buffer, a.flag)
}

// recWriteArgsWheelJointSetSteeringLimits is b3RecWriteArgs_WheelJointSetSteeringLimits.
func recWriteArgsWheelJointSetSteeringLimits(rec *Recording, a *recArgsWheelJointSetSteeringLimits) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.lower)
	recWF32(&rec.buffer, a.upper)
}

// recWriteArgsWheelJointSetTargetSteeringAngle is b3RecWriteArgs_WheelJointSetTargetSteeringAngle.
func recWriteArgsWheelJointSetTargetSteeringAngle(rec *Recording, a *recArgsWheelJointSetTargetSteeringAngle) {
	recWJOINTID(&rec.buffer, a.joint)
	recWF32(&rec.buffer, a.radians)
}

// recWriteArgsQueryOverlapAABB is b3RecWriteArgs_QueryOverlapAABB.
func recWriteArgsQueryOverlapAABB(rec *Recording, a *recArgsQueryOverlapAABB) {
	recWWORLDID(&rec.buffer, a.world)
	recWAABB(&rec.buffer, a.aabb)
	recWQUERYFILTER(&rec.buffer, a.filter)
}

// recWriteArgsQueryOverlapShape is b3RecWriteArgs_QueryOverlapShape.
func recWriteArgsQueryOverlapShape(rec *Recording, a *recArgsQueryOverlapShape) {
	recWWORLDID(&rec.buffer, a.world)
	recWPOSITION(&rec.buffer, a.origin)
	recWSHAPEPROXY(&rec.buffer, a.proxy)
	recWQUERYFILTER(&rec.buffer, a.filter)
}

// recWriteArgsQueryCastRay is b3RecWriteArgs_QueryCastRay.
func recWriteArgsQueryCastRay(rec *Recording, a *recArgsQueryCastRay) {
	recWWORLDID(&rec.buffer, a.world)
	recWPOSITION(&rec.buffer, a.origin)
	recWVEC3(&rec.buffer, a.translation)
	recWQUERYFILTER(&rec.buffer, a.filter)
}

// recWriteArgsQueryCastShape is b3RecWriteArgs_QueryCastShape.
func recWriteArgsQueryCastShape(rec *Recording, a *recArgsQueryCastShape) {
	recWWORLDID(&rec.buffer, a.world)
	recWPOSITION(&rec.buffer, a.origin)
	recWSHAPEPROXY(&rec.buffer, a.proxy)
	recWVEC3(&rec.buffer, a.translation)
	recWQUERYFILTER(&rec.buffer, a.filter)
}

// recWriteArgsQueryCastRayClosest is b3RecWriteArgs_QueryCastRayClosest.
func recWriteArgsQueryCastRayClosest(rec *Recording, a *recArgsQueryCastRayClosest) {
	recWWORLDID(&rec.buffer, a.world)
	recWPOSITION(&rec.buffer, a.origin)
	recWVEC3(&rec.buffer, a.translation)
	recWQUERYFILTER(&rec.buffer, a.filter)
}

// recWriteArgsQueryCastMover is b3RecWriteArgs_QueryCastMover.
func recWriteArgsQueryCastMover(rec *Recording, a *recArgsQueryCastMover) {
	recWWORLDID(&rec.buffer, a.world)
	recWPOSITION(&rec.buffer, a.origin)
	recWCAPSULE(&rec.buffer, a.mover)
	recWVEC3(&rec.buffer, a.translation)
	recWQUERYFILTER(&rec.buffer, a.filter)
}

// recWriteArgsQueryCollideMover is b3RecWriteArgs_QueryCollideMover.
func recWriteArgsQueryCollideMover(rec *Recording, a *recArgsQueryCollideMover) {
	recWWORLDID(&rec.buffer, a.world)
	recWPOSITION(&rec.buffer, a.origin)
	recWCAPSULE(&rec.buffer, a.mover)
	recWQUERYFILTER(&rec.buffer, a.filter)
}

// recWriteArgsQueryTag is b3RecWriteArgs_QueryTag.
func recWriteArgsQueryTag(rec *Recording, a *recArgsQueryTag) {
	recWU64(&rec.buffer, a.key)
}

// recWriteArgsStateHash is b3RecWriteArgs_StateHash.
func recWriteArgsStateHash(rec *Recording, a *recArgsStateHash) {
	recWWORLDID(&rec.buffer, a.world)
	recWU64(&rec.buffer, a.hash)
}

// recWriteArgsRecordingBounds is b3RecWriteArgs_RecordingBounds.
func recWriteArgsRecordingBounds(rec *Recording, a *recArgsRecordingBounds) {
	recWAABB(&rec.buffer, a.bounds)
}

// Codegen: full writers. Setters may run on threads that each own a distinct object,
// so hold the lock across the whole record. Without it a concurrent writer splices its bytes between
// our begin and end and the record desyncs replay. Same lock the query commit path takes.

// recWriteDestroyWorld is b3RecWrite_DestroyWorld.
func recWriteDestroyWorld(rec *Recording, a *recArgsDestroyWorld) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x01))
	recWriteArgsDestroyWorld(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteStep is b3RecWrite_Step.
func recWriteStep(rec *Recording, a *recArgsStep) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x80))
	recWriteArgsStep(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldEnableSleeping is b3RecWrite_WorldEnableSleeping.
func recWriteWorldEnableSleeping(rec *Recording, a *recArgsWorldEnableSleeping) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x02))
	recWriteArgsWorldEnableSleeping(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldEnableContinuous is b3RecWrite_WorldEnableContinuous.
func recWriteWorldEnableContinuous(rec *Recording, a *recArgsWorldEnableContinuous) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x03))
	recWriteArgsWorldEnableContinuous(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldSetRestitutionThreshold is b3RecWrite_WorldSetRestitutionThreshold.
func recWriteWorldSetRestitutionThreshold(rec *Recording, a *recArgsWorldSetRestitutionThreshold) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x04))
	recWriteArgsWorldSetRestitutionThreshold(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldSetHitEventThreshold is b3RecWrite_WorldSetHitEventThreshold.
func recWriteWorldSetHitEventThreshold(rec *Recording, a *recArgsWorldSetHitEventThreshold) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x05))
	recWriteArgsWorldSetHitEventThreshold(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldSetGravity is b3RecWrite_WorldSetGravity.
func recWriteWorldSetGravity(rec *Recording, a *recArgsWorldSetGravity) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x06))
	recWriteArgsWorldSetGravity(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldExplode is b3RecWrite_WorldExplode.
func recWriteWorldExplode(rec *Recording, a *recArgsWorldExplode) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x07))
	recWriteArgsWorldExplode(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldSetContactTuning is b3RecWrite_WorldSetContactTuning.
func recWriteWorldSetContactTuning(rec *Recording, a *recArgsWorldSetContactTuning) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x08))
	recWriteArgsWorldSetContactTuning(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldSetContactRecycleDistance is b3RecWrite_WorldSetContactRecycleDistance.
func recWriteWorldSetContactRecycleDistance(rec *Recording, a *recArgsWorldSetContactRecycleDistance) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x09))
	recWriteArgsWorldSetContactRecycleDistance(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldSetMaximumLinearSpeed is b3RecWrite_WorldSetMaximumLinearSpeed.
func recWriteWorldSetMaximumLinearSpeed(rec *Recording, a *recArgsWorldSetMaximumLinearSpeed) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x0A))
	recWriteArgsWorldSetMaximumLinearSpeed(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldEnableWarmStarting is b3RecWrite_WorldEnableWarmStarting.
func recWriteWorldEnableWarmStarting(rec *Recording, a *recArgsWorldEnableWarmStarting) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x0B))
	recWriteArgsWorldEnableWarmStarting(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldRebuildStaticTree is b3RecWrite_WorldRebuildStaticTree.
func recWriteWorldRebuildStaticTree(rec *Recording, a *recArgsWorldRebuildStaticTree) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x0C))
	recWriteArgsWorldRebuildStaticTree(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldEnableSpeculative is b3RecWrite_WorldEnableSpeculative.
func recWriteWorldEnableSpeculative(rec *Recording, a *recArgsWorldEnableSpeculative) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x0D))
	recWriteArgsWorldEnableSpeculative(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldSetRestitutionIterations is b3RecWrite_WorldSetRestitutionIterations.
func recWriteWorldSetRestitutionIterations(rec *Recording, a *recArgsWorldSetRestitutionIterations) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x0E))
	recWriteArgsWorldSetRestitutionIterations(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldEnableRestitutionPropagation is b3RecWrite_WorldEnableRestitutionPropagation.
func recWriteWorldEnableRestitutionPropagation(rec *Recording, a *recArgsWorldEnableRestitutionPropagation) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x0F))
	recWriteArgsWorldEnableRestitutionPropagation(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWorldSetIsotropicInertiaTolerance is b3RecWrite_WorldSetIsotropicInertiaTolerance.
func recWriteWorldSetIsotropicInertiaTolerance(rec *Recording, a *recArgsWorldSetIsotropicInertiaTolerance) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x70))
	recWriteArgsWorldSetIsotropicInertiaTolerance(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateBody is b3RecWrite_CreateBody.
func recWriteCreateBody(rec *Recording, a *recArgsCreateBody) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x10))
	recWriteArgsCreateBody(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDestroyBody is b3RecWrite_DestroyBody.
func recWriteDestroyBody(rec *Recording, a *recArgsDestroyBody) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x11))
	recWriteArgsDestroyBody(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetTransform is b3RecWrite_BodySetTransform.
func recWriteBodySetTransform(rec *Recording, a *recArgsBodySetTransform) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x20))
	recWriteArgsBodySetTransform(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetLinearVelocity is b3RecWrite_BodySetLinearVelocity.
func recWriteBodySetLinearVelocity(rec *Recording, a *recArgsBodySetLinearVelocity) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x21))
	recWriteArgsBodySetLinearVelocity(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetType is b3RecWrite_BodySetType.
func recWriteBodySetType(rec *Recording, a *recArgsBodySetType) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x22))
	recWriteArgsBodySetType(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetName is b3RecWrite_BodySetName.
func recWriteBodySetName(rec *Recording, a *recArgsBodySetName) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x23))
	recWriteArgsBodySetName(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetAngularVelocity is b3RecWrite_BodySetAngularVelocity.
func recWriteBodySetAngularVelocity(rec *Recording, a *recArgsBodySetAngularVelocity) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x24))
	recWriteArgsBodySetAngularVelocity(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetTargetTransform is b3RecWrite_BodySetTargetTransform.
func recWriteBodySetTargetTransform(rec *Recording, a *recArgsBodySetTargetTransform) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x25))
	recWriteArgsBodySetTargetTransform(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyApplyForce is b3RecWrite_BodyApplyForce.
func recWriteBodyApplyForce(rec *Recording, a *recArgsBodyApplyForce) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x26))
	recWriteArgsBodyApplyForce(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyApplyForceToCenter is b3RecWrite_BodyApplyForceToCenter.
func recWriteBodyApplyForceToCenter(rec *Recording, a *recArgsBodyApplyForceToCenter) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x27))
	recWriteArgsBodyApplyForceToCenter(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyApplyTorque is b3RecWrite_BodyApplyTorque.
func recWriteBodyApplyTorque(rec *Recording, a *recArgsBodyApplyTorque) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x28))
	recWriteArgsBodyApplyTorque(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyApplyLinearImpulse is b3RecWrite_BodyApplyLinearImpulse.
func recWriteBodyApplyLinearImpulse(rec *Recording, a *recArgsBodyApplyLinearImpulse) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x29))
	recWriteArgsBodyApplyLinearImpulse(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyApplyLinearImpulseToCenter is b3RecWrite_BodyApplyLinearImpulseToCenter.
func recWriteBodyApplyLinearImpulseToCenter(rec *Recording, a *recArgsBodyApplyLinearImpulseToCenter) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x2A))
	recWriteArgsBodyApplyLinearImpulseToCenter(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyApplyAngularImpulse is b3RecWrite_BodyApplyAngularImpulse.
func recWriteBodyApplyAngularImpulse(rec *Recording, a *recArgsBodyApplyAngularImpulse) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x2B))
	recWriteArgsBodyApplyAngularImpulse(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetMassData is b3RecWrite_BodySetMassData.
func recWriteBodySetMassData(rec *Recording, a *recArgsBodySetMassData) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x2C))
	recWriteArgsBodySetMassData(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyApplyMassFromShapes is b3RecWrite_BodyApplyMassFromShapes.
func recWriteBodyApplyMassFromShapes(rec *Recording, a *recArgsBodyApplyMassFromShapes) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x2D))
	recWriteArgsBodyApplyMassFromShapes(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetLinearDamping is b3RecWrite_BodySetLinearDamping.
func recWriteBodySetLinearDamping(rec *Recording, a *recArgsBodySetLinearDamping) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x2E))
	recWriteArgsBodySetLinearDamping(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetAngularDamping is b3RecWrite_BodySetAngularDamping.
func recWriteBodySetAngularDamping(rec *Recording, a *recArgsBodySetAngularDamping) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x2F))
	recWriteArgsBodySetAngularDamping(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetGravityScale is b3RecWrite_BodySetGravityScale.
func recWriteBodySetGravityScale(rec *Recording, a *recArgsBodySetGravityScale) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x30))
	recWriteArgsBodySetGravityScale(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetAwake is b3RecWrite_BodySetAwake.
func recWriteBodySetAwake(rec *Recording, a *recArgsBodySetAwake) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x31))
	recWriteArgsBodySetAwake(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyEnableSleep is b3RecWrite_BodyEnableSleep.
func recWriteBodyEnableSleep(rec *Recording, a *recArgsBodyEnableSleep) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x32))
	recWriteArgsBodyEnableSleep(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetSleepThreshold is b3RecWrite_BodySetSleepThreshold.
func recWriteBodySetSleepThreshold(rec *Recording, a *recArgsBodySetSleepThreshold) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x33))
	recWriteArgsBodySetSleepThreshold(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyDisable is b3RecWrite_BodyDisable.
func recWriteBodyDisable(rec *Recording, a *recArgsBodyDisable) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x34))
	recWriteArgsBodyDisable(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyEnable is b3RecWrite_BodyEnable.
func recWriteBodyEnable(rec *Recording, a *recArgsBodyEnable) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x35))
	recWriteArgsBodyEnable(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetMotionLocks is b3RecWrite_BodySetMotionLocks.
func recWriteBodySetMotionLocks(rec *Recording, a *recArgsBodySetMotionLocks) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x36))
	recWriteArgsBodySetMotionLocks(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetBullet is b3RecWrite_BodySetBullet.
func recWriteBodySetBullet(rec *Recording, a *recArgsBodySetBullet) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x37))
	recWriteArgsBodySetBullet(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyEnableContactRecycling is b3RecWrite_BodyEnableContactRecycling.
func recWriteBodyEnableContactRecycling(rec *Recording, a *recArgsBodyEnableContactRecycling) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x38))
	recWriteArgsBodyEnableContactRecycling(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyEnableHitEvents is b3RecWrite_BodyEnableHitEvents.
func recWriteBodyEnableHitEvents(rec *Recording, a *recArgsBodyEnableHitEvents) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x39))
	recWriteArgsBodyEnableHitEvents(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodyAllowFastRotation is b3RecWrite_BodyAllowFastRotation.
func recWriteBodyAllowFastRotation(rec *Recording, a *recArgsBodyAllowFastRotation) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x3A))
	recWriteArgsBodyAllowFastRotation(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteBodySetSafetyFactor is b3RecWrite_BodySetSafetyFactor.
func recWriteBodySetSafetyFactor(rec *Recording, a *recArgsBodySetSafetyFactor) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x3B))
	recWriteArgsBodySetSafetyFactor(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateSphereShape is b3RecWrite_CreateSphereShape.
func recWriteCreateSphereShape(rec *Recording, a *recArgsCreateSphereShape) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x40))
	recWriteArgsCreateSphereShape(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateCapsuleShape is b3RecWrite_CreateCapsuleShape.
func recWriteCreateCapsuleShape(rec *Recording, a *recArgsCreateCapsuleShape) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x41))
	recWriteArgsCreateCapsuleShape(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateHullShape is b3RecWrite_CreateHullShape.
func recWriteCreateHullShape(rec *Recording, a *recArgsCreateHullShape) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x42))
	recWriteArgsCreateHullShape(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateMeshShape is b3RecWrite_CreateMeshShape.
func recWriteCreateMeshShape(rec *Recording, a *recArgsCreateMeshShape) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x43))
	recWriteArgsCreateMeshShape(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateHeightFieldShape is b3RecWrite_CreateHeightFieldShape.
func recWriteCreateHeightFieldShape(rec *Recording, a *recArgsCreateHeightFieldShape) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x44))
	recWriteArgsCreateHeightFieldShape(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateCompoundShape is b3RecWrite_CreateCompoundShape.
func recWriteCreateCompoundShape(rec *Recording, a *recArgsCreateCompoundShape) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x45))
	recWriteArgsCreateCompoundShape(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDestroyShape is b3RecWrite_DestroyShape.
func recWriteDestroyShape(rec *Recording, a *recArgsDestroyShape) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x46))
	recWriteArgsDestroyShape(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetDensity is b3RecWrite_ShapeSetDensity.
func recWriteShapeSetDensity(rec *Recording, a *recArgsShapeSetDensity) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x50))
	recWriteArgsShapeSetDensity(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetFriction is b3RecWrite_ShapeSetFriction.
func recWriteShapeSetFriction(rec *Recording, a *recArgsShapeSetFriction) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x51))
	recWriteArgsShapeSetFriction(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetRestitution is b3RecWrite_ShapeSetRestitution.
func recWriteShapeSetRestitution(rec *Recording, a *recArgsShapeSetRestitution) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x52))
	recWriteArgsShapeSetRestitution(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetSurfaceMaterial is b3RecWrite_ShapeSetSurfaceMaterial.
func recWriteShapeSetSurfaceMaterial(rec *Recording, a *recArgsShapeSetSurfaceMaterial) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x53))
	recWriteArgsShapeSetSurfaceMaterial(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetFilter is b3RecWrite_ShapeSetFilter.
func recWriteShapeSetFilter(rec *Recording, a *recArgsShapeSetFilter) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x54))
	recWriteArgsShapeSetFilter(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeEnableSensorEvents is b3RecWrite_ShapeEnableSensorEvents.
func recWriteShapeEnableSensorEvents(rec *Recording, a *recArgsShapeEnableSensorEvents) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x55))
	recWriteArgsShapeEnableSensorEvents(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeEnableContactEvents is b3RecWrite_ShapeEnableContactEvents.
func recWriteShapeEnableContactEvents(rec *Recording, a *recArgsShapeEnableContactEvents) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x56))
	recWriteArgsShapeEnableContactEvents(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeEnablePreSolveEvents is b3RecWrite_ShapeEnablePreSolveEvents.
func recWriteShapeEnablePreSolveEvents(rec *Recording, a *recArgsShapeEnablePreSolveEvents) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x57))
	recWriteArgsShapeEnablePreSolveEvents(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeEnableHitEvents is b3RecWrite_ShapeEnableHitEvents.
func recWriteShapeEnableHitEvents(rec *Recording, a *recArgsShapeEnableHitEvents) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x58))
	recWriteArgsShapeEnableHitEvents(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetSphere is b3RecWrite_ShapeSetSphere.
func recWriteShapeSetSphere(rec *Recording, a *recArgsShapeSetSphere) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x59))
	recWriteArgsShapeSetSphere(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetCapsule is b3RecWrite_ShapeSetCapsule.
func recWriteShapeSetCapsule(rec *Recording, a *recArgsShapeSetCapsule) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x5A))
	recWriteArgsShapeSetCapsule(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeApplyWind is b3RecWrite_ShapeApplyWind.
func recWriteShapeApplyWind(rec *Recording, a *recArgsShapeApplyWind) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x5B))
	recWriteArgsShapeApplyWind(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetName is b3RecWrite_ShapeSetName.
func recWriteShapeSetName(rec *Recording, a *recArgsShapeSetName) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x5C))
	recWriteArgsShapeSetName(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetMeshMaterial is b3RecWrite_ShapeSetMeshMaterial.
func recWriteShapeSetMeshMaterial(rec *Recording, a *recArgsShapeSetMeshMaterial) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x5D))
	recWriteArgsShapeSetMeshMaterial(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetHull is b3RecWrite_ShapeSetHull.
func recWriteShapeSetHull(rec *Recording, a *recArgsShapeSetHull) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x5E))
	recWriteArgsShapeSetHull(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteShapeSetMesh is b3RecWrite_ShapeSetMesh.
func recWriteShapeSetMesh(rec *Recording, a *recArgsShapeSetMesh) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x5F))
	recWriteArgsShapeSetMesh(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateParallelJoint is b3RecWrite_CreateParallelJoint.
func recWriteCreateParallelJoint(rec *Recording, a *recArgsCreateParallelJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x90))
	recWriteArgsCreateParallelJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateDistanceJoint is b3RecWrite_CreateDistanceJoint.
func recWriteCreateDistanceJoint(rec *Recording, a *recArgsCreateDistanceJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x91))
	recWriteArgsCreateDistanceJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateFilterJoint is b3RecWrite_CreateFilterJoint.
func recWriteCreateFilterJoint(rec *Recording, a *recArgsCreateFilterJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x92))
	recWriteArgsCreateFilterJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateMotorJoint is b3RecWrite_CreateMotorJoint.
func recWriteCreateMotorJoint(rec *Recording, a *recArgsCreateMotorJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x93))
	recWriteArgsCreateMotorJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreatePrismaticJoint is b3RecWrite_CreatePrismaticJoint.
func recWriteCreatePrismaticJoint(rec *Recording, a *recArgsCreatePrismaticJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x94))
	recWriteArgsCreatePrismaticJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateRevoluteJoint is b3RecWrite_CreateRevoluteJoint.
func recWriteCreateRevoluteJoint(rec *Recording, a *recArgsCreateRevoluteJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x95))
	recWriteArgsCreateRevoluteJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateSphericalJoint is b3RecWrite_CreateSphericalJoint.
func recWriteCreateSphericalJoint(rec *Recording, a *recArgsCreateSphericalJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x96))
	recWriteArgsCreateSphericalJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateWeldJoint is b3RecWrite_CreateWeldJoint.
func recWriteCreateWeldJoint(rec *Recording, a *recArgsCreateWeldJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x97))
	recWriteArgsCreateWeldJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteCreateWheelJoint is b3RecWrite_CreateWheelJoint.
func recWriteCreateWheelJoint(rec *Recording, a *recArgsCreateWheelJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x98))
	recWriteArgsCreateWheelJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDestroyJoint is b3RecWrite_DestroyJoint.
func recWriteDestroyJoint(rec *Recording, a *recArgsDestroyJoint) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x99))
	recWriteArgsDestroyJoint(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteJointSetLocalFrameA is b3RecWrite_JointSetLocalFrameA.
func recWriteJointSetLocalFrameA(rec *Recording, a *recArgsJointSetLocalFrameA) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x9A))
	recWriteArgsJointSetLocalFrameA(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteJointSetLocalFrameB is b3RecWrite_JointSetLocalFrameB.
func recWriteJointSetLocalFrameB(rec *Recording, a *recArgsJointSetLocalFrameB) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x9B))
	recWriteArgsJointSetLocalFrameB(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteJointSetCollideConnected is b3RecWrite_JointSetCollideConnected.
func recWriteJointSetCollideConnected(rec *Recording, a *recArgsJointSetCollideConnected) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x9C))
	recWriteArgsJointSetCollideConnected(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteJointWakeBodies is b3RecWrite_JointWakeBodies.
func recWriteJointWakeBodies(rec *Recording, a *recArgsJointWakeBodies) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x9D))
	recWriteArgsJointWakeBodies(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteJointSetConstraintTuning is b3RecWrite_JointSetConstraintTuning.
func recWriteJointSetConstraintTuning(rec *Recording, a *recArgsJointSetConstraintTuning) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x9E))
	recWriteArgsJointSetConstraintTuning(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteJointSetForceThreshold is b3RecWrite_JointSetForceThreshold.
func recWriteJointSetForceThreshold(rec *Recording, a *recArgsJointSetForceThreshold) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x9F))
	recWriteArgsJointSetForceThreshold(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteJointSetTorqueThreshold is b3RecWrite_JointSetTorqueThreshold.
func recWriteJointSetTorqueThreshold(rec *Recording, a *recArgsJointSetTorqueThreshold) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA0))
	recWriteArgsJointSetTorqueThreshold(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteParallelJointSetSpringHertz is b3RecWrite_ParallelJointSetSpringHertz.
func recWriteParallelJointSetSpringHertz(rec *Recording, a *recArgsParallelJointSetSpringHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA1))
	recWriteArgsParallelJointSetSpringHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteParallelJointSetSpringDampingRatio is b3RecWrite_ParallelJointSetSpringDampingRatio.
func recWriteParallelJointSetSpringDampingRatio(rec *Recording, a *recArgsParallelJointSetSpringDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA2))
	recWriteArgsParallelJointSetSpringDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteParallelJointSetMaxTorque is b3RecWrite_ParallelJointSetMaxTorque.
func recWriteParallelJointSetMaxTorque(rec *Recording, a *recArgsParallelJointSetMaxTorque) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA3))
	recWriteArgsParallelJointSetMaxTorque(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointSetLength is b3RecWrite_DistanceJointSetLength.
func recWriteDistanceJointSetLength(rec *Recording, a *recArgsDistanceJointSetLength) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA4))
	recWriteArgsDistanceJointSetLength(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointEnableSpring is b3RecWrite_DistanceJointEnableSpring.
func recWriteDistanceJointEnableSpring(rec *Recording, a *recArgsDistanceJointEnableSpring) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA5))
	recWriteArgsDistanceJointEnableSpring(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointSetSpringForceRange is b3RecWrite_DistanceJointSetSpringForceRange.
func recWriteDistanceJointSetSpringForceRange(rec *Recording, a *recArgsDistanceJointSetSpringForceRange) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA6))
	recWriteArgsDistanceJointSetSpringForceRange(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointSetSpringHertz is b3RecWrite_DistanceJointSetSpringHertz.
func recWriteDistanceJointSetSpringHertz(rec *Recording, a *recArgsDistanceJointSetSpringHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA7))
	recWriteArgsDistanceJointSetSpringHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointSetSpringDampingRatio is b3RecWrite_DistanceJointSetSpringDampingRatio.
func recWriteDistanceJointSetSpringDampingRatio(rec *Recording, a *recArgsDistanceJointSetSpringDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA8))
	recWriteArgsDistanceJointSetSpringDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointEnableLimit is b3RecWrite_DistanceJointEnableLimit.
func recWriteDistanceJointEnableLimit(rec *Recording, a *recArgsDistanceJointEnableLimit) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xA9))
	recWriteArgsDistanceJointEnableLimit(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointSetLengthRange is b3RecWrite_DistanceJointSetLengthRange.
func recWriteDistanceJointSetLengthRange(rec *Recording, a *recArgsDistanceJointSetLengthRange) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xAA))
	recWriteArgsDistanceJointSetLengthRange(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointEnableMotor is b3RecWrite_DistanceJointEnableMotor.
func recWriteDistanceJointEnableMotor(rec *Recording, a *recArgsDistanceJointEnableMotor) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xAB))
	recWriteArgsDistanceJointEnableMotor(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointSetMotorSpeed is b3RecWrite_DistanceJointSetMotorSpeed.
func recWriteDistanceJointSetMotorSpeed(rec *Recording, a *recArgsDistanceJointSetMotorSpeed) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xAC))
	recWriteArgsDistanceJointSetMotorSpeed(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteDistanceJointSetMaxMotorForce is b3RecWrite_DistanceJointSetMaxMotorForce.
func recWriteDistanceJointSetMaxMotorForce(rec *Recording, a *recArgsDistanceJointSetMaxMotorForce) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xAD))
	recWriteArgsDistanceJointSetMaxMotorForce(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetLinearVelocity is b3RecWrite_MotorJointSetLinearVelocity.
func recWriteMotorJointSetLinearVelocity(rec *Recording, a *recArgsMotorJointSetLinearVelocity) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xAE))
	recWriteArgsMotorJointSetLinearVelocity(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetAngularVelocity is b3RecWrite_MotorJointSetAngularVelocity.
func recWriteMotorJointSetAngularVelocity(rec *Recording, a *recArgsMotorJointSetAngularVelocity) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xAF))
	recWriteArgsMotorJointSetAngularVelocity(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetMaxVelocityForce is b3RecWrite_MotorJointSetMaxVelocityForce.
func recWriteMotorJointSetMaxVelocityForce(rec *Recording, a *recArgsMotorJointSetMaxVelocityForce) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB0))
	recWriteArgsMotorJointSetMaxVelocityForce(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetMaxVelocityTorque is b3RecWrite_MotorJointSetMaxVelocityTorque.
func recWriteMotorJointSetMaxVelocityTorque(rec *Recording, a *recArgsMotorJointSetMaxVelocityTorque) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB1))
	recWriteArgsMotorJointSetMaxVelocityTorque(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetLinearHertz is b3RecWrite_MotorJointSetLinearHertz.
func recWriteMotorJointSetLinearHertz(rec *Recording, a *recArgsMotorJointSetLinearHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB2))
	recWriteArgsMotorJointSetLinearHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetLinearDampingRatio is b3RecWrite_MotorJointSetLinearDampingRatio.
func recWriteMotorJointSetLinearDampingRatio(rec *Recording, a *recArgsMotorJointSetLinearDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB3))
	recWriteArgsMotorJointSetLinearDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetAngularHertz is b3RecWrite_MotorJointSetAngularHertz.
func recWriteMotorJointSetAngularHertz(rec *Recording, a *recArgsMotorJointSetAngularHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB4))
	recWriteArgsMotorJointSetAngularHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetAngularDampingRatio is b3RecWrite_MotorJointSetAngularDampingRatio.
func recWriteMotorJointSetAngularDampingRatio(rec *Recording, a *recArgsMotorJointSetAngularDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB5))
	recWriteArgsMotorJointSetAngularDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetMaxSpringForce is b3RecWrite_MotorJointSetMaxSpringForce.
func recWriteMotorJointSetMaxSpringForce(rec *Recording, a *recArgsMotorJointSetMaxSpringForce) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB6))
	recWriteArgsMotorJointSetMaxSpringForce(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteMotorJointSetMaxSpringTorque is b3RecWrite_MotorJointSetMaxSpringTorque.
func recWriteMotorJointSetMaxSpringTorque(rec *Recording, a *recArgsMotorJointSetMaxSpringTorque) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB7))
	recWriteArgsMotorJointSetMaxSpringTorque(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWritePrismaticJointEnableSpring is b3RecWrite_PrismaticJointEnableSpring.
func recWritePrismaticJointEnableSpring(rec *Recording, a *recArgsPrismaticJointEnableSpring) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB8))
	recWriteArgsPrismaticJointEnableSpring(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWritePrismaticJointSetSpringHertz is b3RecWrite_PrismaticJointSetSpringHertz.
func recWritePrismaticJointSetSpringHertz(rec *Recording, a *recArgsPrismaticJointSetSpringHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xB9))
	recWriteArgsPrismaticJointSetSpringHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWritePrismaticJointSetSpringDampingRatio is b3RecWrite_PrismaticJointSetSpringDampingRatio.
func recWritePrismaticJointSetSpringDampingRatio(rec *Recording, a *recArgsPrismaticJointSetSpringDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xBA))
	recWriteArgsPrismaticJointSetSpringDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWritePrismaticJointSetTargetTranslation is b3RecWrite_PrismaticJointSetTargetTranslation.
func recWritePrismaticJointSetTargetTranslation(rec *Recording, a *recArgsPrismaticJointSetTargetTranslation) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xBB))
	recWriteArgsPrismaticJointSetTargetTranslation(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWritePrismaticJointEnableLimit is b3RecWrite_PrismaticJointEnableLimit.
func recWritePrismaticJointEnableLimit(rec *Recording, a *recArgsPrismaticJointEnableLimit) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xBC))
	recWriteArgsPrismaticJointEnableLimit(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWritePrismaticJointSetLimits is b3RecWrite_PrismaticJointSetLimits.
func recWritePrismaticJointSetLimits(rec *Recording, a *recArgsPrismaticJointSetLimits) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xBD))
	recWriteArgsPrismaticJointSetLimits(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWritePrismaticJointEnableMotor is b3RecWrite_PrismaticJointEnableMotor.
func recWritePrismaticJointEnableMotor(rec *Recording, a *recArgsPrismaticJointEnableMotor) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xBE))
	recWriteArgsPrismaticJointEnableMotor(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWritePrismaticJointSetMotorSpeed is b3RecWrite_PrismaticJointSetMotorSpeed.
func recWritePrismaticJointSetMotorSpeed(rec *Recording, a *recArgsPrismaticJointSetMotorSpeed) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xBF))
	recWriteArgsPrismaticJointSetMotorSpeed(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWritePrismaticJointSetMaxMotorForce is b3RecWrite_PrismaticJointSetMaxMotorForce.
func recWritePrismaticJointSetMaxMotorForce(rec *Recording, a *recArgsPrismaticJointSetMaxMotorForce) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC0))
	recWriteArgsPrismaticJointSetMaxMotorForce(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRevoluteJointEnableSpring is b3RecWrite_RevoluteJointEnableSpring.
func recWriteRevoluteJointEnableSpring(rec *Recording, a *recArgsRevoluteJointEnableSpring) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC1))
	recWriteArgsRevoluteJointEnableSpring(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRevoluteJointSetSpringHertz is b3RecWrite_RevoluteJointSetSpringHertz.
func recWriteRevoluteJointSetSpringHertz(rec *Recording, a *recArgsRevoluteJointSetSpringHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC2))
	recWriteArgsRevoluteJointSetSpringHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRevoluteJointSetSpringDampingRatio is b3RecWrite_RevoluteJointSetSpringDampingRatio.
func recWriteRevoluteJointSetSpringDampingRatio(rec *Recording, a *recArgsRevoluteJointSetSpringDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC3))
	recWriteArgsRevoluteJointSetSpringDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRevoluteJointSetTargetAngle is b3RecWrite_RevoluteJointSetTargetAngle.
func recWriteRevoluteJointSetTargetAngle(rec *Recording, a *recArgsRevoluteJointSetTargetAngle) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC4))
	recWriteArgsRevoluteJointSetTargetAngle(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRevoluteJointEnableLimit is b3RecWrite_RevoluteJointEnableLimit.
func recWriteRevoluteJointEnableLimit(rec *Recording, a *recArgsRevoluteJointEnableLimit) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC5))
	recWriteArgsRevoluteJointEnableLimit(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRevoluteJointSetLimits is b3RecWrite_RevoluteJointSetLimits.
func recWriteRevoluteJointSetLimits(rec *Recording, a *recArgsRevoluteJointSetLimits) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC6))
	recWriteArgsRevoluteJointSetLimits(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRevoluteJointEnableMotor is b3RecWrite_RevoluteJointEnableMotor.
func recWriteRevoluteJointEnableMotor(rec *Recording, a *recArgsRevoluteJointEnableMotor) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC7))
	recWriteArgsRevoluteJointEnableMotor(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRevoluteJointSetMotorSpeed is b3RecWrite_RevoluteJointSetMotorSpeed.
func recWriteRevoluteJointSetMotorSpeed(rec *Recording, a *recArgsRevoluteJointSetMotorSpeed) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC8))
	recWriteArgsRevoluteJointSetMotorSpeed(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRevoluteJointSetMaxMotorTorque is b3RecWrite_RevoluteJointSetMaxMotorTorque.
func recWriteRevoluteJointSetMaxMotorTorque(rec *Recording, a *recArgsRevoluteJointSetMaxMotorTorque) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xC9))
	recWriteArgsRevoluteJointSetMaxMotorTorque(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointEnableConeLimit is b3RecWrite_SphericalJointEnableConeLimit.
func recWriteSphericalJointEnableConeLimit(rec *Recording, a *recArgsSphericalJointEnableConeLimit) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xCA))
	recWriteArgsSphericalJointEnableConeLimit(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointSetConeLimit is b3RecWrite_SphericalJointSetConeLimit.
func recWriteSphericalJointSetConeLimit(rec *Recording, a *recArgsSphericalJointSetConeLimit) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xCB))
	recWriteArgsSphericalJointSetConeLimit(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointEnableTwistLimit is b3RecWrite_SphericalJointEnableTwistLimit.
func recWriteSphericalJointEnableTwistLimit(rec *Recording, a *recArgsSphericalJointEnableTwistLimit) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xCC))
	recWriteArgsSphericalJointEnableTwistLimit(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointSetTwistLimits is b3RecWrite_SphericalJointSetTwistLimits.
func recWriteSphericalJointSetTwistLimits(rec *Recording, a *recArgsSphericalJointSetTwistLimits) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xCD))
	recWriteArgsSphericalJointSetTwistLimits(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointEnableSpring is b3RecWrite_SphericalJointEnableSpring.
func recWriteSphericalJointEnableSpring(rec *Recording, a *recArgsSphericalJointEnableSpring) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xCE))
	recWriteArgsSphericalJointEnableSpring(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointSetSpringHertz is b3RecWrite_SphericalJointSetSpringHertz.
func recWriteSphericalJointSetSpringHertz(rec *Recording, a *recArgsSphericalJointSetSpringHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xCF))
	recWriteArgsSphericalJointSetSpringHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointSetSpringDampingRatio is b3RecWrite_SphericalJointSetSpringDampingRatio.
func recWriteSphericalJointSetSpringDampingRatio(rec *Recording, a *recArgsSphericalJointSetSpringDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD0))
	recWriteArgsSphericalJointSetSpringDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointSetTargetRotation is b3RecWrite_SphericalJointSetTargetRotation.
func recWriteSphericalJointSetTargetRotation(rec *Recording, a *recArgsSphericalJointSetTargetRotation) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD1))
	recWriteArgsSphericalJointSetTargetRotation(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointEnableMotor is b3RecWrite_SphericalJointEnableMotor.
func recWriteSphericalJointEnableMotor(rec *Recording, a *recArgsSphericalJointEnableMotor) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD2))
	recWriteArgsSphericalJointEnableMotor(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointSetMotorVelocity is b3RecWrite_SphericalJointSetMotorVelocity.
func recWriteSphericalJointSetMotorVelocity(rec *Recording, a *recArgsSphericalJointSetMotorVelocity) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD3))
	recWriteArgsSphericalJointSetMotorVelocity(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteSphericalJointSetMaxMotorTorque is b3RecWrite_SphericalJointSetMaxMotorTorque.
func recWriteSphericalJointSetMaxMotorTorque(rec *Recording, a *recArgsSphericalJointSetMaxMotorTorque) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD4))
	recWriteArgsSphericalJointSetMaxMotorTorque(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWeldJointSetLinearHertz is b3RecWrite_WeldJointSetLinearHertz.
func recWriteWeldJointSetLinearHertz(rec *Recording, a *recArgsWeldJointSetLinearHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD5))
	recWriteArgsWeldJointSetLinearHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWeldJointSetLinearDampingRatio is b3RecWrite_WeldJointSetLinearDampingRatio.
func recWriteWeldJointSetLinearDampingRatio(rec *Recording, a *recArgsWeldJointSetLinearDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD6))
	recWriteArgsWeldJointSetLinearDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWeldJointSetAngularHertz is b3RecWrite_WeldJointSetAngularHertz.
func recWriteWeldJointSetAngularHertz(rec *Recording, a *recArgsWeldJointSetAngularHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD7))
	recWriteArgsWeldJointSetAngularHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWeldJointSetAngularDampingRatio is b3RecWrite_WeldJointSetAngularDampingRatio.
func recWriteWeldJointSetAngularDampingRatio(rec *Recording, a *recArgsWeldJointSetAngularDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD8))
	recWriteArgsWeldJointSetAngularDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointEnableSuspension is b3RecWrite_WheelJointEnableSuspension.
func recWriteWheelJointEnableSuspension(rec *Recording, a *recArgsWheelJointEnableSuspension) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xD9))
	recWriteArgsWheelJointEnableSuspension(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetSuspensionHertz is b3RecWrite_WheelJointSetSuspensionHertz.
func recWriteWheelJointSetSuspensionHertz(rec *Recording, a *recArgsWheelJointSetSuspensionHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xDA))
	recWriteArgsWheelJointSetSuspensionHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetSuspensionDampingRatio is b3RecWrite_WheelJointSetSuspensionDampingRatio.
func recWriteWheelJointSetSuspensionDampingRatio(rec *Recording, a *recArgsWheelJointSetSuspensionDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xDB))
	recWriteArgsWheelJointSetSuspensionDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointEnableSuspensionLimit is b3RecWrite_WheelJointEnableSuspensionLimit.
func recWriteWheelJointEnableSuspensionLimit(rec *Recording, a *recArgsWheelJointEnableSuspensionLimit) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xDC))
	recWriteArgsWheelJointEnableSuspensionLimit(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetSuspensionLimits is b3RecWrite_WheelJointSetSuspensionLimits.
func recWriteWheelJointSetSuspensionLimits(rec *Recording, a *recArgsWheelJointSetSuspensionLimits) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xDD))
	recWriteArgsWheelJointSetSuspensionLimits(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointEnableSpinMotor is b3RecWrite_WheelJointEnableSpinMotor.
func recWriteWheelJointEnableSpinMotor(rec *Recording, a *recArgsWheelJointEnableSpinMotor) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xDE))
	recWriteArgsWheelJointEnableSpinMotor(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetSpinMotorSpeed is b3RecWrite_WheelJointSetSpinMotorSpeed.
func recWriteWheelJointSetSpinMotorSpeed(rec *Recording, a *recArgsWheelJointSetSpinMotorSpeed) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xDF))
	recWriteArgsWheelJointSetSpinMotorSpeed(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetMaxSpinTorque is b3RecWrite_WheelJointSetMaxSpinTorque.
func recWriteWheelJointSetMaxSpinTorque(rec *Recording, a *recArgsWheelJointSetMaxSpinTorque) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE0))
	recWriteArgsWheelJointSetMaxSpinTorque(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointEnableSteering is b3RecWrite_WheelJointEnableSteering.
func recWriteWheelJointEnableSteering(rec *Recording, a *recArgsWheelJointEnableSteering) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE1))
	recWriteArgsWheelJointEnableSteering(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetSteeringHertz is b3RecWrite_WheelJointSetSteeringHertz.
func recWriteWheelJointSetSteeringHertz(rec *Recording, a *recArgsWheelJointSetSteeringHertz) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE2))
	recWriteArgsWheelJointSetSteeringHertz(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetSteeringDampingRatio is b3RecWrite_WheelJointSetSteeringDampingRatio.
func recWriteWheelJointSetSteeringDampingRatio(rec *Recording, a *recArgsWheelJointSetSteeringDampingRatio) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE3))
	recWriteArgsWheelJointSetSteeringDampingRatio(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetMaxSteeringTorque is b3RecWrite_WheelJointSetMaxSteeringTorque.
func recWriteWheelJointSetMaxSteeringTorque(rec *Recording, a *recArgsWheelJointSetMaxSteeringTorque) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE4))
	recWriteArgsWheelJointSetMaxSteeringTorque(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointEnableSteeringLimit is b3RecWrite_WheelJointEnableSteeringLimit.
func recWriteWheelJointEnableSteeringLimit(rec *Recording, a *recArgsWheelJointEnableSteeringLimit) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE5))
	recWriteArgsWheelJointEnableSteeringLimit(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetSteeringLimits is b3RecWrite_WheelJointSetSteeringLimits.
func recWriteWheelJointSetSteeringLimits(rec *Recording, a *recArgsWheelJointSetSteeringLimits) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE6))
	recWriteArgsWheelJointSetSteeringLimits(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteWheelJointSetTargetSteeringAngle is b3RecWrite_WheelJointSetTargetSteeringAngle.
func recWriteWheelJointSetTargetSteeringAngle(rec *Recording, a *recArgsWheelJointSetTargetSteeringAngle) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE7))
	recWriteArgsWheelJointSetTargetSteeringAngle(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteQueryOverlapAABB is b3RecWrite_QueryOverlapAABB.
func recWriteQueryOverlapAABB(rec *Recording, a *recArgsQueryOverlapAABB) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE8))
	recWriteArgsQueryOverlapAABB(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteQueryOverlapShape is b3RecWrite_QueryOverlapShape.
func recWriteQueryOverlapShape(rec *Recording, a *recArgsQueryOverlapShape) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xE9))
	recWriteArgsQueryOverlapShape(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteQueryCastRay is b3RecWrite_QueryCastRay.
func recWriteQueryCastRay(rec *Recording, a *recArgsQueryCastRay) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xEA))
	recWriteArgsQueryCastRay(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteQueryCastShape is b3RecWrite_QueryCastShape.
func recWriteQueryCastShape(rec *Recording, a *recArgsQueryCastShape) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xEB))
	recWriteArgsQueryCastShape(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteQueryCastRayClosest is b3RecWrite_QueryCastRayClosest.
func recWriteQueryCastRayClosest(rec *Recording, a *recArgsQueryCastRayClosest) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xEC))
	recWriteArgsQueryCastRayClosest(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteQueryCastMover is b3RecWrite_QueryCastMover.
func recWriteQueryCastMover(rec *Recording, a *recArgsQueryCastMover) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xED))
	recWriteArgsQueryCastMover(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteQueryCollideMover is b3RecWrite_QueryCollideMover.
func recWriteQueryCollideMover(rec *Recording, a *recArgsQueryCollideMover) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xEE))
	recWriteArgsQueryCollideMover(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteQueryTag is b3RecWrite_QueryTag.
func recWriteQueryTag(rec *Recording, a *recArgsQueryTag) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xEF))
	recWriteArgsQueryTag(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteStateHash is b3RecWrite_StateHash.
func recWriteStateHash(rec *Recording, a *recArgsStateHash) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xF1))
	recWriteArgsStateHash(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRecordingBounds is b3RecWrite_RecordingBounds.
func recWriteRecordingBounds(rec *Recording, a *recArgsRecordingBounds) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0xF2))
	recWriteArgsRecordingBounds(rec, a)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// Codegen: create-op writers that append the returned id inside the record

// recWriteRetCreateBody is b3RecWriteRet_CreateBody.
func recWriteRetCreateBody(rec *Recording, a *recArgsCreateBody, id BodyID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x10))
	recWriteArgsCreateBody(rec, a)
	recWBODYID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateSphereShape is b3RecWriteRet_CreateSphereShape.
func recWriteRetCreateSphereShape(rec *Recording, a *recArgsCreateSphereShape, id ShapeID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x40))
	recWriteArgsCreateSphereShape(rec, a)
	recWSHAPEID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateCapsuleShape is b3RecWriteRet_CreateCapsuleShape.
func recWriteRetCreateCapsuleShape(rec *Recording, a *recArgsCreateCapsuleShape, id ShapeID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x41))
	recWriteArgsCreateCapsuleShape(rec, a)
	recWSHAPEID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateHullShape is b3RecWriteRet_CreateHullShape.
func recWriteRetCreateHullShape(rec *Recording, a *recArgsCreateHullShape, id ShapeID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x42))
	recWriteArgsCreateHullShape(rec, a)
	recWSHAPEID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateMeshShape is b3RecWriteRet_CreateMeshShape.
func recWriteRetCreateMeshShape(rec *Recording, a *recArgsCreateMeshShape, id ShapeID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x43))
	recWriteArgsCreateMeshShape(rec, a)
	recWSHAPEID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateHeightFieldShape is b3RecWriteRet_CreateHeightFieldShape.
func recWriteRetCreateHeightFieldShape(rec *Recording, a *recArgsCreateHeightFieldShape, id ShapeID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x44))
	recWriteArgsCreateHeightFieldShape(rec, a)
	recWSHAPEID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateCompoundShape is b3RecWriteRet_CreateCompoundShape.
func recWriteRetCreateCompoundShape(rec *Recording, a *recArgsCreateCompoundShape, id ShapeID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x45))
	recWriteArgsCreateCompoundShape(rec, a)
	recWSHAPEID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateParallelJoint is b3RecWriteRet_CreateParallelJoint.
func recWriteRetCreateParallelJoint(rec *Recording, a *recArgsCreateParallelJoint, id JointID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x90))
	recWriteArgsCreateParallelJoint(rec, a)
	recWJOINTID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateDistanceJoint is b3RecWriteRet_CreateDistanceJoint.
func recWriteRetCreateDistanceJoint(rec *Recording, a *recArgsCreateDistanceJoint, id JointID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x91))
	recWriteArgsCreateDistanceJoint(rec, a)
	recWJOINTID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateFilterJoint is b3RecWriteRet_CreateFilterJoint.
func recWriteRetCreateFilterJoint(rec *Recording, a *recArgsCreateFilterJoint, id JointID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x92))
	recWriteArgsCreateFilterJoint(rec, a)
	recWJOINTID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateMotorJoint is b3RecWriteRet_CreateMotorJoint.
func recWriteRetCreateMotorJoint(rec *Recording, a *recArgsCreateMotorJoint, id JointID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x93))
	recWriteArgsCreateMotorJoint(rec, a)
	recWJOINTID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreatePrismaticJoint is b3RecWriteRet_CreatePrismaticJoint.
func recWriteRetCreatePrismaticJoint(rec *Recording, a *recArgsCreatePrismaticJoint, id JointID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x94))
	recWriteArgsCreatePrismaticJoint(rec, a)
	recWJOINTID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateRevoluteJoint is b3RecWriteRet_CreateRevoluteJoint.
func recWriteRetCreateRevoluteJoint(rec *Recording, a *recArgsCreateRevoluteJoint, id JointID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x95))
	recWriteArgsCreateRevoluteJoint(rec, a)
	recWJOINTID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateSphericalJoint is b3RecWriteRet_CreateSphericalJoint.
func recWriteRetCreateSphericalJoint(rec *Recording, a *recArgsCreateSphericalJoint, id JointID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x96))
	recWriteArgsCreateSphericalJoint(rec, a)
	recWJOINTID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateWeldJoint is b3RecWriteRet_CreateWeldJoint.
func recWriteRetCreateWeldJoint(rec *Recording, a *recArgsCreateWeldJoint, id JointID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x97))
	recWriteArgsCreateWeldJoint(rec, a)
	recWJOINTID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}

// recWriteRetCreateWheelJoint is b3RecWriteRet_CreateWheelJoint.
func recWriteRetCreateWheelJoint(rec *Recording, a *recArgsCreateWheelJoint, id JointID) {
	lockMutex(rec.lock)
	recBeginRecord(rec, uint8(0x98))
	recWriteArgsCreateWheelJoint(rec, a)
	recWJOINTID(&rec.buffer, id)
	recEndRecord(rec)
	unlockMutex(rec.lock)
}
