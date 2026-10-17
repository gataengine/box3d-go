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
// the b3_recOpName opcode constants), in manifest order. The writer expansions of recording.c
// (b3RecWriteArgs_Name, b3RecWrite_Name, b3RecWriteRet_Name) are stubs in recording.go until
// milestone 5; the reader expansions of replay.c land with replay.go. Keep the manifest comments
// and order in sync with upstream when bumping.
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
