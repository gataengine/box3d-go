// Port of src/replay.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
)

// PORT: second part of replay.c (see replay.go): dispatch functions, spatial query replay and
// the dispatch switch.

// Dispatch functions, one per op

// recDispatchDestroyWorld is b3RecDispatch_DestroyWorld.
func recDispatchDestroyWorld(a *recArgsDestroyWorld, rdr *recReader) {
	_ = a
	_ = rdr
	// End-of-session marker. The replay world is torn down in b3ValidateReplay, not here.
}

// recDispatchStep is b3RecDispatch_Step.
func recDispatchStep(a *recArgsStep, rdr *recReader) {
	rdr.replayWorldId.Step(a.dt, int(a.subStepCount))
}

// recDispatchWorldEnableSleeping is b3RecDispatch_WorldEnableSleeping.
func recDispatchWorldEnableSleeping(a *recArgsWorldEnableSleeping, rdr *recReader) {
	rdr.replayWorldId.EnableSleeping(a.flag)
}

// recDispatchWorldEnableContinuous is b3RecDispatch_WorldEnableContinuous.
func recDispatchWorldEnableContinuous(a *recArgsWorldEnableContinuous, rdr *recReader) {
	rdr.replayWorldId.EnableContinuous(a.flag)
}

// recDispatchWorldSetRestitutionThreshold is b3RecDispatch_WorldSetRestitutionThreshold.
func recDispatchWorldSetRestitutionThreshold(a *recArgsWorldSetRestitutionThreshold, rdr *recReader) {
	rdr.replayWorldId.SetRestitutionThreshold(a.value)
}

// recDispatchWorldSetHitEventThreshold is b3RecDispatch_WorldSetHitEventThreshold.
func recDispatchWorldSetHitEventThreshold(a *recArgsWorldSetHitEventThreshold, rdr *recReader) {
	rdr.replayWorldId.SetHitEventThreshold(a.value)
}

// recDispatchWorldSetGravity is b3RecDispatch_WorldSetGravity.
func recDispatchWorldSetGravity(a *recArgsWorldSetGravity, rdr *recReader) {
	rdr.replayWorldId.SetGravity(a.gravity)
}

// recDispatchWorldExplode is b3RecDispatch_WorldExplode.
func recDispatchWorldExplode(a *recArgsWorldExplode, rdr *recReader) {
	rdr.replayWorldId.Explode(&a.def)
}

// recDispatchWorldSetContactTuning is b3RecDispatch_WorldSetContactTuning.
func recDispatchWorldSetContactTuning(a *recArgsWorldSetContactTuning, rdr *recReader) {
	rdr.replayWorldId.SetContactTuning(a.hertz, a.dampingRatio, a.contactSpeed)
}

// recDispatchWorldSetContactRecycleDistance is b3RecDispatch_WorldSetContactRecycleDistance.
func recDispatchWorldSetContactRecycleDistance(a *recArgsWorldSetContactRecycleDistance, rdr *recReader) {
	rdr.replayWorldId.SetContactRecycleDistance(a.recycleDistance)
}

// recDispatchWorldSetMaximumLinearSpeed is b3RecDispatch_WorldSetMaximumLinearSpeed.
func recDispatchWorldSetMaximumLinearSpeed(a *recArgsWorldSetMaximumLinearSpeed, rdr *recReader) {
	rdr.replayWorldId.SetMaximumLinearSpeed(a.maximumLinearSpeed)
}

// recDispatchWorldEnableWarmStarting is b3RecDispatch_WorldEnableWarmStarting.
func recDispatchWorldEnableWarmStarting(a *recArgsWorldEnableWarmStarting, rdr *recReader) {
	rdr.replayWorldId.EnableWarmStarting(a.flag)
}

// recDispatchWorldRebuildStaticTree is b3RecDispatch_WorldRebuildStaticTree.
func recDispatchWorldRebuildStaticTree(a *recArgsWorldRebuildStaticTree, rdr *recReader) {
	rdr.replayWorldId.RebuildStaticTree()
}

// recDispatchWorldEnableSpeculative is b3RecDispatch_WorldEnableSpeculative.
func recDispatchWorldEnableSpeculative(a *recArgsWorldEnableSpeculative, rdr *recReader) {
	rdr.replayWorldId.EnableSpeculative(a.flag)
}

// recDispatchWorldSetRestitutionIterations is b3RecDispatch_WorldSetRestitutionIterations.
func recDispatchWorldSetRestitutionIterations(a *recArgsWorldSetRestitutionIterations, rdr *recReader) {
	rdr.replayWorldId.SetRestitutionIterations(int(a.iterations))
}

// recDispatchWorldEnableRestitutionPropagation is b3RecDispatch_WorldEnableRestitutionPropagation.
func recDispatchWorldEnableRestitutionPropagation(a *recArgsWorldEnableRestitutionPropagation, rdr *recReader) {
	rdr.replayWorldId.EnableRestitutionPropagation(a.flag)
}

// recDispatchWorldSetIsotropicInertiaTolerance is b3RecDispatch_WorldSetIsotropicInertiaTolerance.
func recDispatchWorldSetIsotropicInertiaTolerance(a *recArgsWorldSetIsotropicInertiaTolerance, rdr *recReader) {
	rdr.replayWorldId.SetIsotropicInertiaTolerance(a.tolerance)
}

// recDispatchCreateBody is b3RecDispatch_CreateBody.
func recDispatchCreateBody(a *recArgsCreateBody, rdr *recReader) {
	recId := recRBODYID(rdr)
	gotId := rdr.replayWorldId.CreateBody(&a.def)
	recCheckBodyId(rdr, gotId, recId)
	if rdr.owner != nil {
		recTrackBodyCreate(rdr.owner, gotId)
	}
}

// recDispatchDestroyBody is b3RecDispatch_DestroyBody.
func recDispatchDestroyBody(a *recArgsDestroyBody, rdr *recReader) {
	id := recMakeBodyId(rdr, a.body)
	if rdr.owner != nil {
		recTrackBodyDestroy(rdr.owner, id)
	}
	id.Destroy()
}

// recDispatchBodySetTransform is b3RecDispatch_BodySetTransform.
func recDispatchBodySetTransform(a *recArgsBodySetTransform, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetTransform(a.position, a.rotation)
}

// recDispatchBodySetLinearVelocity is b3RecDispatch_BodySetLinearVelocity.
func recDispatchBodySetLinearVelocity(a *recArgsBodySetLinearVelocity, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetLinearVelocity(a.v)
}

// recDispatchBodySetType is b3RecDispatch_BodySetType.
func recDispatchBodySetType(a *recArgsBodySetType, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetType(BodyType(a.typ))
}

// recDispatchBodySetName is b3RecDispatch_BodySetName.
func recDispatchBodySetName(a *recArgsBodySetName, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetName(a.name)
}

// recDispatchBodySetAngularVelocity is b3RecDispatch_BodySetAngularVelocity.
func recDispatchBodySetAngularVelocity(a *recArgsBodySetAngularVelocity, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetAngularVelocity(a.w)
}

// recDispatchBodySetTargetTransform is b3RecDispatch_BodySetTargetTransform.
func recDispatchBodySetTargetTransform(a *recArgsBodySetTargetTransform, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetTargetTransform(a.target, a.timeStep, a.wake)
}

// recDispatchBodyApplyForce is b3RecDispatch_BodyApplyForce.
func recDispatchBodyApplyForce(a *recArgsBodyApplyForce, rdr *recReader) {
	recMakeBodyId(rdr, a.body).ApplyForce(a.force, a.point, a.wake)
}

// recDispatchBodyApplyForceToCenter is b3RecDispatch_BodyApplyForceToCenter.
func recDispatchBodyApplyForceToCenter(a *recArgsBodyApplyForceToCenter, rdr *recReader) {
	recMakeBodyId(rdr, a.body).ApplyForceToCenter(a.force, a.wake)
}

// recDispatchBodyApplyTorque is b3RecDispatch_BodyApplyTorque.
func recDispatchBodyApplyTorque(a *recArgsBodyApplyTorque, rdr *recReader) {
	recMakeBodyId(rdr, a.body).ApplyTorque(a.torque, a.wake)
}

// recDispatchBodyApplyLinearImpulse is b3RecDispatch_BodyApplyLinearImpulse.
func recDispatchBodyApplyLinearImpulse(a *recArgsBodyApplyLinearImpulse, rdr *recReader) {
	recMakeBodyId(rdr, a.body).ApplyLinearImpulse(a.impulse, a.point, a.wake)
}

// recDispatchBodyApplyLinearImpulseToCenter is b3RecDispatch_BodyApplyLinearImpulseToCenter.
func recDispatchBodyApplyLinearImpulseToCenter(a *recArgsBodyApplyLinearImpulseToCenter, rdr *recReader) {
	recMakeBodyId(rdr, a.body).ApplyLinearImpulseToCenter(a.impulse, a.wake)
}

// recDispatchBodyApplyAngularImpulse is b3RecDispatch_BodyApplyAngularImpulse.
func recDispatchBodyApplyAngularImpulse(a *recArgsBodyApplyAngularImpulse, rdr *recReader) {
	recMakeBodyId(rdr, a.body).ApplyAngularImpulse(a.impulse, a.wake)
}

// recDispatchBodySetMassData is b3RecDispatch_BodySetMassData.
func recDispatchBodySetMassData(a *recArgsBodySetMassData, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetMassData(a.massData)
}

// recDispatchBodyApplyMassFromShapes is b3RecDispatch_BodyApplyMassFromShapes.
func recDispatchBodyApplyMassFromShapes(a *recArgsBodyApplyMassFromShapes, rdr *recReader) {
	recMakeBodyId(rdr, a.body).ApplyMassFromShapes()
}

// recDispatchBodySetLinearDamping is b3RecDispatch_BodySetLinearDamping.
func recDispatchBodySetLinearDamping(a *recArgsBodySetLinearDamping, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetLinearDamping(a.damping)
}

// recDispatchBodySetAngularDamping is b3RecDispatch_BodySetAngularDamping.
func recDispatchBodySetAngularDamping(a *recArgsBodySetAngularDamping, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetAngularDamping(a.damping)
}

// recDispatchBodySetGravityScale is b3RecDispatch_BodySetGravityScale.
func recDispatchBodySetGravityScale(a *recArgsBodySetGravityScale, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetGravityScale(a.scale)
}

// recDispatchBodySetAwake is b3RecDispatch_BodySetAwake.
func recDispatchBodySetAwake(a *recArgsBodySetAwake, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetAwake(a.awake)
}

// recDispatchBodyEnableSleep is b3RecDispatch_BodyEnableSleep.
func recDispatchBodyEnableSleep(a *recArgsBodyEnableSleep, rdr *recReader) {
	recMakeBodyId(rdr, a.body).EnableSleep(a.flag)
}

// recDispatchBodySetSleepThreshold is b3RecDispatch_BodySetSleepThreshold.
func recDispatchBodySetSleepThreshold(a *recArgsBodySetSleepThreshold, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetSleepThreshold(a.threshold)
}

// recDispatchBodySetSafetyFactor is b3RecDispatch_BodySetSafetyFactor.
func recDispatchBodySetSafetyFactor(a *recArgsBodySetSafetyFactor, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetSafetyFactor(a.value)
}

// recDispatchBodyDisable is b3RecDispatch_BodyDisable.
func recDispatchBodyDisable(a *recArgsBodyDisable, rdr *recReader) {
	recMakeBodyId(rdr, a.body).Disable()
}

// recDispatchBodyEnable is b3RecDispatch_BodyEnable.
func recDispatchBodyEnable(a *recArgsBodyEnable, rdr *recReader) {
	recMakeBodyId(rdr, a.body).Enable()
}

// recDispatchBodySetMotionLocks is b3RecDispatch_BodySetMotionLocks.
func recDispatchBodySetMotionLocks(a *recArgsBodySetMotionLocks, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetMotionLocks(a.locks)
}

// recDispatchBodySetBullet is b3RecDispatch_BodySetBullet.
func recDispatchBodySetBullet(a *recArgsBodySetBullet, rdr *recReader) {
	recMakeBodyId(rdr, a.body).SetBullet(a.flag)
}

// recDispatchBodyAllowFastRotation is b3RecDispatch_BodyAllowFastRotation.
func recDispatchBodyAllowFastRotation(a *recArgsBodyAllowFastRotation, rdr *recReader) {
	recMakeBodyId(rdr, a.body).AllowFastRotation(a.flag)
}

// recDispatchBodyEnableContactRecycling is b3RecDispatch_BodyEnableContactRecycling.
func recDispatchBodyEnableContactRecycling(a *recArgsBodyEnableContactRecycling, rdr *recReader) {
	recMakeBodyId(rdr, a.body).EnableContactRecycling(a.flag)
}

// recDispatchBodyEnableHitEvents is b3RecDispatch_BodyEnableHitEvents.
func recDispatchBodyEnableHitEvents(a *recArgsBodyEnableHitEvents, rdr *recReader) {
	recMakeBodyId(rdr, a.body).EnableHitEvents(a.flag)
}

// recDispatchCreateSphereShape is b3RecDispatch_CreateSphereShape.
func recDispatchCreateSphereShape(a *recArgsCreateSphereShape, rdr *recReader) {
	recId := recRSHAPEID(rdr)
	bodyId := recMakeBodyId(rdr, a.body)
	gotId := bodyId.CreateSphereShape(&a.def, &a.sphere)
	recCheckShapeId(rdr, gotId, recId)
}

// recDispatchCreateCapsuleShape is b3RecDispatch_CreateCapsuleShape.
func recDispatchCreateCapsuleShape(a *recArgsCreateCapsuleShape, rdr *recReader) {
	recId := recRSHAPEID(rdr)
	bodyId := recMakeBodyId(rdr, a.body)
	gotId := bodyId.CreateCapsuleShape(&a.def, &a.capsule)
	recCheckShapeId(rdr, gotId, recId)
}

// recDispatchCreateHullShape is b3RecDispatch_CreateHullShape.
func recDispatchCreateHullShape(a *recArgsCreateHullShape, rdr *recReader) {
	recId := recRSHAPEID(rdr)
	if !rdr.ok {
		return
	}
	id := a.geometryId
	slot := recGetSlot(rdr, id, geometryHull)
	if slot == nil {
		fmt.Printf("b3ReplayFile: hull geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	bodyId := recMakeBodyId(rdr, a.body)
	// Hull is cloned by b3CreateHullShape into the world DB; no caching needed.
	gotId := bodyId.CreateHullShape(&a.def, recSlotHeader[HullData](slot))
	recCheckShapeId(rdr, gotId, recId)
}

// recDispatchCreateMeshShape is b3RecDispatch_CreateMeshShape.
func recDispatchCreateMeshShape(a *recArgsCreateMeshShape, rdr *recReader) {
	recId := recRSHAPEID(rdr)
	if !rdr.ok {
		return
	}
	id := a.geometryId
	slot := recGetSlot(rdr, id, geometryMesh)
	if slot == nil {
		fmt.Printf("b3ReplayFile: mesh geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	mesh := recSlotHeader[MeshData](slot)
	bodyId := recMakeBodyId(rdr, a.body)
	gotId := bodyId.CreateMeshShape(&a.def, mesh, a.scale)
	recCheckShapeId(rdr, gotId, recId)
}

// recDispatchCreateHeightFieldShape is b3RecDispatch_CreateHeightFieldShape.
func recDispatchCreateHeightFieldShape(a *recArgsCreateHeightFieldShape, rdr *recReader) {
	recId := recRSHAPEID(rdr)
	if !rdr.ok {
		return
	}
	id := a.geometryId
	slot := recGetSlot(rdr, id, geometryHeightField)
	if slot == nil {
		fmt.Printf("b3ReplayFile: heightfield geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	hf := recSlotHeader[HeightFieldData](slot)
	bodyId := recMakeBodyId(rdr, a.body)
	gotId := bodyId.CreateHeightFieldShape(&a.def, hf)
	recCheckShapeId(rdr, gotId, recId)
}

// recDispatchCreateCompoundShape is b3RecDispatch_CreateCompoundShape.
func recDispatchCreateCompoundShape(a *recArgsCreateCompoundShape, rdr *recReader) {
	recId := recRSHAPEID(rdr)
	if !rdr.ok {
		return
	}
	id := a.geometryId
	slot := recGetSlot(rdr, id, geometryCompound)
	if slot == nil {
		fmt.Printf("b3ReplayFile: compound geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	compound := recSlotHeader[CompoundData](slot)
	bodyId := recMakeBodyId(rdr, a.body)
	// b3CreateCompoundShape takes a non-const def pointer; cast away const for the scratch def
	shapeDef := a.def
	gotId := bodyId.CreateBakedCompoundShape(&shapeDef, compound)
	recCheckShapeId(rdr, gotId, recId)
}

// recDispatchDestroyShape is b3RecDispatch_DestroyShape.
func recDispatchDestroyShape(a *recArgsDestroyShape, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).Destroy(a.updateBodyMass)
}

// recDispatchShapeSetName is b3RecDispatch_ShapeSetName.
func recDispatchShapeSetName(a *recArgsShapeSetName, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).SetName(a.name)
}

// recDispatchShapeSetDensity is b3RecDispatch_ShapeSetDensity.
func recDispatchShapeSetDensity(a *recArgsShapeSetDensity, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).SetDensity(a.density, a.updateBodyMass)
}

// recDispatchShapeSetFriction is b3RecDispatch_ShapeSetFriction.
func recDispatchShapeSetFriction(a *recArgsShapeSetFriction, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).SetFriction(a.friction)
}

// recDispatchShapeSetRestitution is b3RecDispatch_ShapeSetRestitution.
func recDispatchShapeSetRestitution(a *recArgsShapeSetRestitution, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).SetRestitution(a.restitution)
}

// recDispatchShapeSetSurfaceMaterial is b3RecDispatch_ShapeSetSurfaceMaterial.
func recDispatchShapeSetSurfaceMaterial(a *recArgsShapeSetSurfaceMaterial, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).SetSurfaceMaterial(a.material)
}

// recDispatchShapeSetMeshMaterial is b3RecDispatch_ShapeSetMeshMaterial.
func recDispatchShapeSetMeshMaterial(a *recArgsShapeSetMeshMaterial, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).SetMeshMaterial(a.material, int(a.index))
}

// recDispatchShapeSetFilter is b3RecDispatch_ShapeSetFilter.
func recDispatchShapeSetFilter(a *recArgsShapeSetFilter, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).SetFilter(a.filter, a.invokeContacts)
}

// recDispatchShapeEnableSensorEvents is b3RecDispatch_ShapeEnableSensorEvents.
func recDispatchShapeEnableSensorEvents(a *recArgsShapeEnableSensorEvents, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).EnableSensorEvents(a.flag)
}

// recDispatchShapeEnableContactEvents is b3RecDispatch_ShapeEnableContactEvents.
func recDispatchShapeEnableContactEvents(a *recArgsShapeEnableContactEvents, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).EnableContactEvents(a.flag)
}

// recDispatchShapeEnablePreSolveEvents is b3RecDispatch_ShapeEnablePreSolveEvents.
func recDispatchShapeEnablePreSolveEvents(a *recArgsShapeEnablePreSolveEvents, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).EnablePreSolveEvents(a.flag)
}

// recDispatchShapeEnableHitEvents is b3RecDispatch_ShapeEnableHitEvents.
func recDispatchShapeEnableHitEvents(a *recArgsShapeEnableHitEvents, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).EnableHitEvents(a.flag)
}

// recDispatchShapeSetSphere is b3RecDispatch_ShapeSetSphere.
func recDispatchShapeSetSphere(a *recArgsShapeSetSphere, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).SetSphere(&a.sphere)
}

// recDispatchShapeSetCapsule is b3RecDispatch_ShapeSetCapsule.
func recDispatchShapeSetCapsule(a *recArgsShapeSetCapsule, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).SetCapsule(&a.capsule)
}

// recDispatchShapeSetHull is b3RecDispatch_ShapeSetHull.
func recDispatchShapeSetHull(a *recArgsShapeSetHull, rdr *recReader) {
	id := a.geometryId
	slot := recGetSlot(rdr, id, geometryHull)
	if slot == nil {
		fmt.Printf("b3ReplayFile: hull geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	shapeId := recMakeShapeId(rdr, a.shape)
	shapeId.SetHull(recSlotHeader[HullData](slot))
}

// recDispatchShapeSetMesh is b3RecDispatch_ShapeSetMesh.
func recDispatchShapeSetMesh(a *recArgsShapeSetMesh, rdr *recReader) {
	id := a.geometryId
	slot := recGetSlot(rdr, id, geometryMesh)
	if slot == nil {
		fmt.Printf("b3ReplayFile: mesh geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	shapeId := recMakeShapeId(rdr, a.shape)
	mesh := recSlotHeader[MeshData](slot)
	shapeId.SetMesh(mesh, a.scale)
}

// recDispatchShapeApplyWind is b3RecDispatch_ShapeApplyWind.
func recDispatchShapeApplyWind(a *recArgsShapeApplyWind, rdr *recReader) {
	recMakeShapeId(rdr, a.shape).ApplyWind(a.wind, a.drag, a.lift, a.maxSpeed, a.wake)
}

// recDispatchCreateParallelJoint is b3RecDispatch_CreateParallelJoint.
func recDispatchCreateParallelJoint(a *recArgsCreateParallelJoint, rdr *recReader) {
	recId := recRJOINTID(rdr)
	def := a.def
	def.Base.BodyIdA = recMakeBodyId(rdr, def.Base.BodyIdA)
	def.Base.BodyIdB = recMakeBodyId(rdr, def.Base.BodyIdB)
	recCheckJointId(rdr, rdr.replayWorldId.CreateParallelJoint(&def), recId)
}

// recDispatchCreateDistanceJoint is b3RecDispatch_CreateDistanceJoint.
func recDispatchCreateDistanceJoint(a *recArgsCreateDistanceJoint, rdr *recReader) {
	recId := recRJOINTID(rdr)
	def := a.def
	def.Base.BodyIdA = recMakeBodyId(rdr, def.Base.BodyIdA)
	def.Base.BodyIdB = recMakeBodyId(rdr, def.Base.BodyIdB)
	recCheckJointId(rdr, rdr.replayWorldId.CreateDistanceJoint(&def), recId)
}

// recDispatchCreateFilterJoint is b3RecDispatch_CreateFilterJoint.
func recDispatchCreateFilterJoint(a *recArgsCreateFilterJoint, rdr *recReader) {
	recId := recRJOINTID(rdr)
	def := a.def
	def.Base.BodyIdA = recMakeBodyId(rdr, def.Base.BodyIdA)
	def.Base.BodyIdB = recMakeBodyId(rdr, def.Base.BodyIdB)
	recCheckJointId(rdr, rdr.replayWorldId.CreateFilterJoint(&def), recId)
}

// recDispatchCreateMotorJoint is b3RecDispatch_CreateMotorJoint.
func recDispatchCreateMotorJoint(a *recArgsCreateMotorJoint, rdr *recReader) {
	recId := recRJOINTID(rdr)
	def := a.def
	def.Base.BodyIdA = recMakeBodyId(rdr, def.Base.BodyIdA)
	def.Base.BodyIdB = recMakeBodyId(rdr, def.Base.BodyIdB)
	recCheckJointId(rdr, rdr.replayWorldId.CreateMotorJoint(&def), recId)
}

// recDispatchCreatePrismaticJoint is b3RecDispatch_CreatePrismaticJoint.
func recDispatchCreatePrismaticJoint(a *recArgsCreatePrismaticJoint, rdr *recReader) {
	recId := recRJOINTID(rdr)
	def := a.def
	def.Base.BodyIdA = recMakeBodyId(rdr, def.Base.BodyIdA)
	def.Base.BodyIdB = recMakeBodyId(rdr, def.Base.BodyIdB)
	recCheckJointId(rdr, rdr.replayWorldId.CreatePrismaticJoint(&def), recId)
}

// recDispatchCreateRevoluteJoint is b3RecDispatch_CreateRevoluteJoint.
func recDispatchCreateRevoluteJoint(a *recArgsCreateRevoluteJoint, rdr *recReader) {
	recId := recRJOINTID(rdr)
	def := a.def
	def.Base.BodyIdA = recMakeBodyId(rdr, def.Base.BodyIdA)
	def.Base.BodyIdB = recMakeBodyId(rdr, def.Base.BodyIdB)
	recCheckJointId(rdr, rdr.replayWorldId.CreateRevoluteJoint(&def), recId)
}

// recDispatchCreateSphericalJoint is b3RecDispatch_CreateSphericalJoint.
func recDispatchCreateSphericalJoint(a *recArgsCreateSphericalJoint, rdr *recReader) {
	recId := recRJOINTID(rdr)
	def := a.def
	def.Base.BodyIdA = recMakeBodyId(rdr, def.Base.BodyIdA)
	def.Base.BodyIdB = recMakeBodyId(rdr, def.Base.BodyIdB)
	recCheckJointId(rdr, rdr.replayWorldId.CreateSphericalJoint(&def), recId)
}

// recDispatchCreateWeldJoint is b3RecDispatch_CreateWeldJoint.
func recDispatchCreateWeldJoint(a *recArgsCreateWeldJoint, rdr *recReader) {
	recId := recRJOINTID(rdr)
	def := a.def
	def.Base.BodyIdA = recMakeBodyId(rdr, def.Base.BodyIdA)
	def.Base.BodyIdB = recMakeBodyId(rdr, def.Base.BodyIdB)
	recCheckJointId(rdr, rdr.replayWorldId.CreateWeldJoint(&def), recId)
}

// recDispatchCreateWheelJoint is b3RecDispatch_CreateWheelJoint.
func recDispatchCreateWheelJoint(a *recArgsCreateWheelJoint, rdr *recReader) {
	recId := recRJOINTID(rdr)
	def := a.def
	def.Base.BodyIdA = recMakeBodyId(rdr, def.Base.BodyIdA)
	def.Base.BodyIdB = recMakeBodyId(rdr, def.Base.BodyIdB)
	recCheckJointId(rdr, rdr.replayWorldId.CreateWheelJoint(&def), recId)
}

// recDispatchDestroyJoint is b3RecDispatch_DestroyJoint.
func recDispatchDestroyJoint(a *recArgsDestroyJoint, rdr *recReader) {
	recMakeJointId(rdr, a.joint).Destroy(a.wakeAttached)
}

// recDispatchJointSetLocalFrameA is b3RecDispatch_JointSetLocalFrameA.
func recDispatchJointSetLocalFrameA(a *recArgsJointSetLocalFrameA, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SetLocalFrameA(a.localFrame)
}

// recDispatchJointSetLocalFrameB is b3RecDispatch_JointSetLocalFrameB.
func recDispatchJointSetLocalFrameB(a *recArgsJointSetLocalFrameB, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SetLocalFrameB(a.localFrame)
}

// recDispatchJointSetCollideConnected is b3RecDispatch_JointSetCollideConnected.
func recDispatchJointSetCollideConnected(a *recArgsJointSetCollideConnected, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SetCollideConnected(a.shouldCollide)
}

// recDispatchJointWakeBodies is b3RecDispatch_JointWakeBodies.
func recDispatchJointWakeBodies(a *recArgsJointWakeBodies, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WakeBodies()
}

// recDispatchJointSetConstraintTuning is b3RecDispatch_JointSetConstraintTuning.
func recDispatchJointSetConstraintTuning(a *recArgsJointSetConstraintTuning, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SetConstraintTuning(a.hertz, a.dampingRatio)
}

// recDispatchJointSetForceThreshold is b3RecDispatch_JointSetForceThreshold.
func recDispatchJointSetForceThreshold(a *recArgsJointSetForceThreshold, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SetForceThreshold(a.threshold)
}

// recDispatchJointSetTorqueThreshold is b3RecDispatch_JointSetTorqueThreshold.
func recDispatchJointSetTorqueThreshold(a *recArgsJointSetTorqueThreshold, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SetTorqueThreshold(a.threshold)
}

// recDispatchParallelJointSetSpringHertz is b3RecDispatch_ParallelJointSetSpringHertz.
func recDispatchParallelJointSetSpringHertz(a *recArgsParallelJointSetSpringHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).ParallelJointSetSpringHertz(a.hertz)
}

// recDispatchParallelJointSetSpringDampingRatio is b3RecDispatch_ParallelJointSetSpringDampingRatio.
func recDispatchParallelJointSetSpringDampingRatio(a *recArgsParallelJointSetSpringDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).ParallelJointSetSpringDampingRatio(a.dampingRatio)
}

// recDispatchParallelJointSetMaxTorque is b3RecDispatch_ParallelJointSetMaxTorque.
func recDispatchParallelJointSetMaxTorque(a *recArgsParallelJointSetMaxTorque, rdr *recReader) {
	recMakeJointId(rdr, a.joint).ParallelJointSetMaxTorque(a.maxTorque)
}

// recDispatchDistanceJointSetLength is b3RecDispatch_DistanceJointSetLength.
func recDispatchDistanceJointSetLength(a *recArgsDistanceJointSetLength, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointSetLength(a.length)
}

// recDispatchDistanceJointEnableSpring is b3RecDispatch_DistanceJointEnableSpring.
func recDispatchDistanceJointEnableSpring(a *recArgsDistanceJointEnableSpring, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointEnableSpring(a.enableSpring)
}

// recDispatchDistanceJointSetSpringForceRange is b3RecDispatch_DistanceJointSetSpringForceRange.
func recDispatchDistanceJointSetSpringForceRange(a *recArgsDistanceJointSetSpringForceRange, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointSetSpringForceRange(a.lowerForce, a.upperForce)
}

// recDispatchDistanceJointSetSpringHertz is b3RecDispatch_DistanceJointSetSpringHertz.
func recDispatchDistanceJointSetSpringHertz(a *recArgsDistanceJointSetSpringHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointSetSpringHertz(a.hertz)
}

// recDispatchDistanceJointSetSpringDampingRatio is b3RecDispatch_DistanceJointSetSpringDampingRatio.
func recDispatchDistanceJointSetSpringDampingRatio(a *recArgsDistanceJointSetSpringDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointSetSpringDampingRatio(a.dampingRatio)
}

// recDispatchDistanceJointEnableLimit is b3RecDispatch_DistanceJointEnableLimit.
func recDispatchDistanceJointEnableLimit(a *recArgsDistanceJointEnableLimit, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointEnableLimit(a.enableLimit)
}

// recDispatchDistanceJointSetLengthRange is b3RecDispatch_DistanceJointSetLengthRange.
func recDispatchDistanceJointSetLengthRange(a *recArgsDistanceJointSetLengthRange, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointSetLengthRange(a.minLength, a.maxLength)
}

// recDispatchDistanceJointEnableMotor is b3RecDispatch_DistanceJointEnableMotor.
func recDispatchDistanceJointEnableMotor(a *recArgsDistanceJointEnableMotor, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointEnableMotor(a.enableMotor)
}

// recDispatchDistanceJointSetMotorSpeed is b3RecDispatch_DistanceJointSetMotorSpeed.
func recDispatchDistanceJointSetMotorSpeed(a *recArgsDistanceJointSetMotorSpeed, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointSetMotorSpeed(a.motorSpeed)
}

// recDispatchDistanceJointSetMaxMotorForce is b3RecDispatch_DistanceJointSetMaxMotorForce.
func recDispatchDistanceJointSetMaxMotorForce(a *recArgsDistanceJointSetMaxMotorForce, rdr *recReader) {
	recMakeJointId(rdr, a.joint).DistanceJointSetMaxMotorForce(a.force)
}

// recDispatchMotorJointSetLinearVelocity is b3RecDispatch_MotorJointSetLinearVelocity.
func recDispatchMotorJointSetLinearVelocity(a *recArgsMotorJointSetLinearVelocity, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetLinearVelocity(a.velocity)
}

// recDispatchMotorJointSetAngularVelocity is b3RecDispatch_MotorJointSetAngularVelocity.
func recDispatchMotorJointSetAngularVelocity(a *recArgsMotorJointSetAngularVelocity, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetAngularVelocity(a.velocity)
}

// recDispatchMotorJointSetMaxVelocityForce is b3RecDispatch_MotorJointSetMaxVelocityForce.
func recDispatchMotorJointSetMaxVelocityForce(a *recArgsMotorJointSetMaxVelocityForce, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetMaxVelocityForce(a.maxForce)
}

// recDispatchMotorJointSetMaxVelocityTorque is b3RecDispatch_MotorJointSetMaxVelocityTorque.
func recDispatchMotorJointSetMaxVelocityTorque(a *recArgsMotorJointSetMaxVelocityTorque, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetMaxVelocityTorque(a.maxTorque)
}

// recDispatchMotorJointSetLinearHertz is b3RecDispatch_MotorJointSetLinearHertz.
func recDispatchMotorJointSetLinearHertz(a *recArgsMotorJointSetLinearHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetLinearHertz(a.hertz)
}

// recDispatchMotorJointSetLinearDampingRatio is b3RecDispatch_MotorJointSetLinearDampingRatio.
func recDispatchMotorJointSetLinearDampingRatio(a *recArgsMotorJointSetLinearDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetLinearDampingRatio(a.damping)
}

// recDispatchMotorJointSetAngularHertz is b3RecDispatch_MotorJointSetAngularHertz.
func recDispatchMotorJointSetAngularHertz(a *recArgsMotorJointSetAngularHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetAngularHertz(a.hertz)
}

// recDispatchMotorJointSetAngularDampingRatio is b3RecDispatch_MotorJointSetAngularDampingRatio.
func recDispatchMotorJointSetAngularDampingRatio(a *recArgsMotorJointSetAngularDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetAngularDampingRatio(a.damping)
}

// recDispatchMotorJointSetMaxSpringForce is b3RecDispatch_MotorJointSetMaxSpringForce.
func recDispatchMotorJointSetMaxSpringForce(a *recArgsMotorJointSetMaxSpringForce, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetMaxSpringForce(a.maxForce)
}

// recDispatchMotorJointSetMaxSpringTorque is b3RecDispatch_MotorJointSetMaxSpringTorque.
func recDispatchMotorJointSetMaxSpringTorque(a *recArgsMotorJointSetMaxSpringTorque, rdr *recReader) {
	recMakeJointId(rdr, a.joint).MotorJointSetMaxSpringTorque(a.maxTorque)
}

// recDispatchPrismaticJointEnableSpring is b3RecDispatch_PrismaticJointEnableSpring.
func recDispatchPrismaticJointEnableSpring(a *recArgsPrismaticJointEnableSpring, rdr *recReader) {
	recMakeJointId(rdr, a.joint).PrismaticJointEnableSpring(a.enableSpring)
}

// recDispatchPrismaticJointSetSpringHertz is b3RecDispatch_PrismaticJointSetSpringHertz.
func recDispatchPrismaticJointSetSpringHertz(a *recArgsPrismaticJointSetSpringHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).PrismaticJointSetSpringHertz(a.hertz)
}

// recDispatchPrismaticJointSetSpringDampingRatio is b3RecDispatch_PrismaticJointSetSpringDampingRatio.
func recDispatchPrismaticJointSetSpringDampingRatio(a *recArgsPrismaticJointSetSpringDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).PrismaticJointSetSpringDampingRatio(a.dampingRatio)
}

// recDispatchPrismaticJointSetTargetTranslation is b3RecDispatch_PrismaticJointSetTargetTranslation.
func recDispatchPrismaticJointSetTargetTranslation(a *recArgsPrismaticJointSetTargetTranslation, rdr *recReader) {
	recMakeJointId(rdr, a.joint).PrismaticJointSetTargetTranslation(a.translation)
}

// recDispatchPrismaticJointEnableLimit is b3RecDispatch_PrismaticJointEnableLimit.
func recDispatchPrismaticJointEnableLimit(a *recArgsPrismaticJointEnableLimit, rdr *recReader) {
	recMakeJointId(rdr, a.joint).PrismaticJointEnableLimit(a.enableLimit)
}

// recDispatchPrismaticJointSetLimits is b3RecDispatch_PrismaticJointSetLimits.
func recDispatchPrismaticJointSetLimits(a *recArgsPrismaticJointSetLimits, rdr *recReader) {
	recMakeJointId(rdr, a.joint).PrismaticJointSetLimits(a.lower, a.upper)
}

// recDispatchPrismaticJointEnableMotor is b3RecDispatch_PrismaticJointEnableMotor.
func recDispatchPrismaticJointEnableMotor(a *recArgsPrismaticJointEnableMotor, rdr *recReader) {
	recMakeJointId(rdr, a.joint).PrismaticJointEnableMotor(a.enableMotor)
}

// recDispatchPrismaticJointSetMotorSpeed is b3RecDispatch_PrismaticJointSetMotorSpeed.
func recDispatchPrismaticJointSetMotorSpeed(a *recArgsPrismaticJointSetMotorSpeed, rdr *recReader) {
	recMakeJointId(rdr, a.joint).PrismaticJointSetMotorSpeed(a.motorSpeed)
}

// recDispatchPrismaticJointSetMaxMotorForce is b3RecDispatch_PrismaticJointSetMaxMotorForce.
func recDispatchPrismaticJointSetMaxMotorForce(a *recArgsPrismaticJointSetMaxMotorForce, rdr *recReader) {
	recMakeJointId(rdr, a.joint).PrismaticJointSetMaxMotorForce(a.force)
}

// recDispatchRevoluteJointEnableSpring is b3RecDispatch_RevoluteJointEnableSpring.
func recDispatchRevoluteJointEnableSpring(a *recArgsRevoluteJointEnableSpring, rdr *recReader) {
	recMakeJointId(rdr, a.joint).RevoluteJointEnableSpring(a.enableSpring)
}

// recDispatchRevoluteJointSetSpringHertz is b3RecDispatch_RevoluteJointSetSpringHertz.
func recDispatchRevoluteJointSetSpringHertz(a *recArgsRevoluteJointSetSpringHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).RevoluteJointSetSpringHertz(a.hertz)
}

// recDispatchRevoluteJointSetSpringDampingRatio is b3RecDispatch_RevoluteJointSetSpringDampingRatio.
func recDispatchRevoluteJointSetSpringDampingRatio(a *recArgsRevoluteJointSetSpringDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).RevoluteJointSetSpringDampingRatio(a.dampingRatio)
}

// recDispatchRevoluteJointSetTargetAngle is b3RecDispatch_RevoluteJointSetTargetAngle.
func recDispatchRevoluteJointSetTargetAngle(a *recArgsRevoluteJointSetTargetAngle, rdr *recReader) {
	recMakeJointId(rdr, a.joint).RevoluteJointSetTargetAngle(a.angle)
}

// recDispatchRevoluteJointEnableLimit is b3RecDispatch_RevoluteJointEnableLimit.
func recDispatchRevoluteJointEnableLimit(a *recArgsRevoluteJointEnableLimit, rdr *recReader) {
	recMakeJointId(rdr, a.joint).RevoluteJointEnableLimit(a.enableLimit)
}

// recDispatchRevoluteJointSetLimits is b3RecDispatch_RevoluteJointSetLimits.
func recDispatchRevoluteJointSetLimits(a *recArgsRevoluteJointSetLimits, rdr *recReader) {
	recMakeJointId(rdr, a.joint).RevoluteJointSetLimits(a.lower, a.upper)
}

// recDispatchRevoluteJointEnableMotor is b3RecDispatch_RevoluteJointEnableMotor.
func recDispatchRevoluteJointEnableMotor(a *recArgsRevoluteJointEnableMotor, rdr *recReader) {
	recMakeJointId(rdr, a.joint).RevoluteJointEnableMotor(a.enableMotor)
}

// recDispatchRevoluteJointSetMotorSpeed is b3RecDispatch_RevoluteJointSetMotorSpeed.
func recDispatchRevoluteJointSetMotorSpeed(a *recArgsRevoluteJointSetMotorSpeed, rdr *recReader) {
	recMakeJointId(rdr, a.joint).RevoluteJointSetMotorSpeed(a.motorSpeed)
}

// recDispatchRevoluteJointSetMaxMotorTorque is b3RecDispatch_RevoluteJointSetMaxMotorTorque.
func recDispatchRevoluteJointSetMaxMotorTorque(a *recArgsRevoluteJointSetMaxMotorTorque, rdr *recReader) {
	recMakeJointId(rdr, a.joint).RevoluteJointSetMaxMotorTorque(a.torque)
}

// recDispatchSphericalJointEnableConeLimit is b3RecDispatch_SphericalJointEnableConeLimit.
func recDispatchSphericalJointEnableConeLimit(a *recArgsSphericalJointEnableConeLimit, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointEnableConeLimit(a.enableLimit)
}

// recDispatchSphericalJointSetConeLimit is b3RecDispatch_SphericalJointSetConeLimit.
func recDispatchSphericalJointSetConeLimit(a *recArgsSphericalJointSetConeLimit, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointSetConeLimit(a.angleRadians)
}

// recDispatchSphericalJointEnableTwistLimit is b3RecDispatch_SphericalJointEnableTwistLimit.
func recDispatchSphericalJointEnableTwistLimit(a *recArgsSphericalJointEnableTwistLimit, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointEnableTwistLimit(a.enableLimit)
}

// recDispatchSphericalJointSetTwistLimits is b3RecDispatch_SphericalJointSetTwistLimits.
func recDispatchSphericalJointSetTwistLimits(a *recArgsSphericalJointSetTwistLimits, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointSetTwistLimits(a.lower, a.upper)
}

// recDispatchSphericalJointEnableSpring is b3RecDispatch_SphericalJointEnableSpring.
func recDispatchSphericalJointEnableSpring(a *recArgsSphericalJointEnableSpring, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointEnableSpring(a.enableSpring)
}

// recDispatchSphericalJointSetSpringHertz is b3RecDispatch_SphericalJointSetSpringHertz.
func recDispatchSphericalJointSetSpringHertz(a *recArgsSphericalJointSetSpringHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointSetSpringHertz(a.hertz)
}

// recDispatchSphericalJointSetSpringDampingRatio is b3RecDispatch_SphericalJointSetSpringDampingRatio.
func recDispatchSphericalJointSetSpringDampingRatio(a *recArgsSphericalJointSetSpringDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointSetSpringDampingRatio(a.dampingRatio)
}

// recDispatchSphericalJointSetTargetRotation is b3RecDispatch_SphericalJointSetTargetRotation.
func recDispatchSphericalJointSetTargetRotation(a *recArgsSphericalJointSetTargetRotation, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointSetTargetRotation(a.targetRotation)
}

// recDispatchSphericalJointEnableMotor is b3RecDispatch_SphericalJointEnableMotor.
func recDispatchSphericalJointEnableMotor(a *recArgsSphericalJointEnableMotor, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointEnableMotor(a.enableMotor)
}

// recDispatchSphericalJointSetMotorVelocity is b3RecDispatch_SphericalJointSetMotorVelocity.
func recDispatchSphericalJointSetMotorVelocity(a *recArgsSphericalJointSetMotorVelocity, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointSetMotorVelocity(a.motorVelocity)
}

// recDispatchSphericalJointSetMaxMotorTorque is b3RecDispatch_SphericalJointSetMaxMotorTorque.
func recDispatchSphericalJointSetMaxMotorTorque(a *recArgsSphericalJointSetMaxMotorTorque, rdr *recReader) {
	recMakeJointId(rdr, a.joint).SphericalJointSetMaxMotorTorque(a.torque)
}

// recDispatchWeldJointSetLinearHertz is b3RecDispatch_WeldJointSetLinearHertz.
func recDispatchWeldJointSetLinearHertz(a *recArgsWeldJointSetLinearHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WeldJointSetLinearHertz(a.hertz)
}

// recDispatchWeldJointSetLinearDampingRatio is b3RecDispatch_WeldJointSetLinearDampingRatio.
func recDispatchWeldJointSetLinearDampingRatio(a *recArgsWeldJointSetLinearDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WeldJointSetLinearDampingRatio(a.dampingRatio)
}

// recDispatchWeldJointSetAngularHertz is b3RecDispatch_WeldJointSetAngularHertz.
func recDispatchWeldJointSetAngularHertz(a *recArgsWeldJointSetAngularHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WeldJointSetAngularHertz(a.hertz)
}

// recDispatchWeldJointSetAngularDampingRatio is b3RecDispatch_WeldJointSetAngularDampingRatio.
func recDispatchWeldJointSetAngularDampingRatio(a *recArgsWeldJointSetAngularDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WeldJointSetAngularDampingRatio(a.dampingRatio)
}

// recDispatchWheelJointEnableSuspension is b3RecDispatch_WheelJointEnableSuspension.
func recDispatchWheelJointEnableSuspension(a *recArgsWheelJointEnableSuspension, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointEnableSuspension(a.flag)
}

// recDispatchWheelJointSetSuspensionHertz is b3RecDispatch_WheelJointSetSuspensionHertz.
func recDispatchWheelJointSetSuspensionHertz(a *recArgsWheelJointSetSuspensionHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetSuspensionHertz(a.hertz)
}

// recDispatchWheelJointSetSuspensionDampingRatio is b3RecDispatch_WheelJointSetSuspensionDampingRatio.
func recDispatchWheelJointSetSuspensionDampingRatio(a *recArgsWheelJointSetSuspensionDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetSuspensionDampingRatio(a.dampingRatio)
}

// recDispatchWheelJointEnableSuspensionLimit is b3RecDispatch_WheelJointEnableSuspensionLimit.
func recDispatchWheelJointEnableSuspensionLimit(a *recArgsWheelJointEnableSuspensionLimit, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointEnableSuspensionLimit(a.flag)
}

// recDispatchWheelJointSetSuspensionLimits is b3RecDispatch_WheelJointSetSuspensionLimits.
func recDispatchWheelJointSetSuspensionLimits(a *recArgsWheelJointSetSuspensionLimits, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetSuspensionLimits(a.lower, a.upper)
}

// recDispatchWheelJointEnableSpinMotor is b3RecDispatch_WheelJointEnableSpinMotor.
func recDispatchWheelJointEnableSpinMotor(a *recArgsWheelJointEnableSpinMotor, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointEnableSpinMotor(a.flag)
}

// recDispatchWheelJointSetSpinMotorSpeed is b3RecDispatch_WheelJointSetSpinMotorSpeed.
func recDispatchWheelJointSetSpinMotorSpeed(a *recArgsWheelJointSetSpinMotorSpeed, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetSpinMotorSpeed(a.speed)
}

// recDispatchWheelJointSetMaxSpinTorque is b3RecDispatch_WheelJointSetMaxSpinTorque.
func recDispatchWheelJointSetMaxSpinTorque(a *recArgsWheelJointSetMaxSpinTorque, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetMaxSpinTorque(a.torque)
}

// recDispatchWheelJointEnableSteering is b3RecDispatch_WheelJointEnableSteering.
func recDispatchWheelJointEnableSteering(a *recArgsWheelJointEnableSteering, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointEnableSteering(a.flag)
}

// recDispatchWheelJointSetSteeringHertz is b3RecDispatch_WheelJointSetSteeringHertz.
func recDispatchWheelJointSetSteeringHertz(a *recArgsWheelJointSetSteeringHertz, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetSteeringHertz(a.hertz)
}

// recDispatchWheelJointSetSteeringDampingRatio is b3RecDispatch_WheelJointSetSteeringDampingRatio.
func recDispatchWheelJointSetSteeringDampingRatio(a *recArgsWheelJointSetSteeringDampingRatio, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetSteeringDampingRatio(a.dampingRatio)
}

// recDispatchWheelJointSetMaxSteeringTorque is b3RecDispatch_WheelJointSetMaxSteeringTorque.
func recDispatchWheelJointSetMaxSteeringTorque(a *recArgsWheelJointSetMaxSteeringTorque, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetMaxSteeringTorque(a.torque)
}

// recDispatchWheelJointEnableSteeringLimit is b3RecDispatch_WheelJointEnableSteeringLimit.
func recDispatchWheelJointEnableSteeringLimit(a *recArgsWheelJointEnableSteeringLimit, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointEnableSteeringLimit(a.flag)
}

// recDispatchWheelJointSetSteeringLimits is b3RecDispatch_WheelJointSetSteeringLimits.
func recDispatchWheelJointSetSteeringLimits(a *recArgsWheelJointSetSteeringLimits, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetSteeringLimits(a.lower, a.upper)
}

// recDispatchWheelJointSetTargetSteeringAngle is b3RecDispatch_WheelJointSetTargetSteeringAngle.
func recDispatchWheelJointSetTargetSteeringAngle(a *recArgsWheelJointSetTargetSteeringAngle, rdr *recReader) {
	recMakeJointId(rdr, a.joint).WheelJointSetTargetSteeringAngle(a.radians)
}

// recDispatchStateHash is b3RecDispatch_StateHash.
func recDispatchStateHash(a *recArgsStateHash, rdr *recReader) {
	world := getWorldFromId(rdr.replayWorldId)
	computed := hashWorldState(world)
	if computed != a.hash {
		fmt.Printf("b3ReplayFile: StateHash mismatch (recorded=0x%x, computed=0x%x)\n", a.hash, computed)
		rdr.diverged = true
	}
}

// recDispatchRecordingBounds is b3RecDispatch_RecordingBounds.
func recDispatchRecordingBounds(a *recArgsRecordingBounds, rdr *recReader) {
	// Primary resolve is the open-time scan, this keeps the value right if it ever moves earlier
	if rdr.owner != nil {
		rdr.owner.bounds = a.bounds
	}
}

// Spatial query replay. The recorded inputs come through the manifest; here the variable-length hit
// tail is read back, the query re-issued against the replay world, and each callback hit compared to
// what was recorded. Any mismatch latches rdr->diverged. When a player owns the reader the hits are
// also stashed for the viewer overlay. The stash helpers dereference the player struct, defined later
// in this file, so they are forward declared and implemented in Block B below.
// PORT: recGrow and recStashQueryBegin are in replay_player.go (Block B); Go needs no forward
// declarations.

// recEnsureHits is b3RecEnsureHits.
// Grow the reader's hit scratch to at least n entries, preserving contents. n is bounded by the file
// size since every recorded hit consumes at least one byte, so a corrupt count fails the read.
func recEnsureHits(rdr *recReader, n int) {
	recReserveScratch(rdr, &rdr.hits, n)
}

// recF32Differs is b3RecF32Differs.
// Bitwise float compare so the determinism check is exact, not within a tolerance.
func recF32Differs(a, b float32) bool {
	ua := math.Float32bits(a)
	ub := math.Float32bits(b)
	return ua != ub
}

// recVec3Differs is b3RecVec3Differs.
func recVec3Differs(a, b Vec3) bool {
	return recF32Differs(a.X, b.X) || recF32Differs(a.Y, b.Y) || recF32Differs(a.Z, b.Z)
}

// recReplayQueryCtx is b3RecReplayQueryCtx.
// Shared context for the replay trampolines: walks recorded hits in order, flagging any divergence
// from the re-issued query.
type recReplayQueryCtx struct {
	rdr    *recReader
	hits   []recRecordedHit
	count  int
	cursor int
}

// recReplayOverlapTrampoline is b3RecReplayOverlapTrampoline.
func recReplayOverlapTrampoline(id ShapeID, ctx any) bool {
	rc := ctx.(*recReplayQueryCtx)
	if rc.cursor >= rc.count {
		rc.rdr.diverged = true
		return false
	}
	h := &rc.hits[rc.cursor]
	rc.cursor++
	if id.Index1 != h.id.Index1 || id.Generation != h.id.Generation {
		rc.rdr.diverged = true
	}
	return h.userReturnB
}

// recReplayMoverFilterTrampoline is b3RecReplayMoverFilterTrampoline.
// The mover filter has the same bool(shapeId, ctx) shape as an overlap callback, so it replays the
// same way. A distinct typed wrapper keeps the function-pointer types clean.
func recReplayMoverFilterTrampoline(id ShapeID, ctx any) bool {
	return recReplayOverlapTrampoline(id, ctx)
}

// recReplayCastTrampoline is b3RecReplayCastTrampoline.
func recReplayCastTrampoline(id ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64,
	triangleIndex int, childIndex int, ctx any) float32 {
	rc := ctx.(*recReplayQueryCtx)
	if rc.cursor >= rc.count {
		rc.rdr.diverged = true
		return 0.0
	}
	h := &rc.hits[rc.cursor]
	rc.cursor++
	// Positions compared through a full-width delta, truncating both sides would pass vacuously far
	// from the origin.
	if id.Index1 != h.id.Index1 || id.Generation != h.id.Generation ||
		recVec3Differs(SubPos(point, h.point), Vec3Zero) || recVec3Differs(normal, h.normal) ||
		recF32Differs(fraction, h.fraction) || userMaterialId != h.userMaterialId || int32(triangleIndex) != h.triangleIndex ||
		int32(childIndex) != h.childIndex {
		rc.rdr.diverged = true
	}
	return h.userReturnF
}

// recReplayPlaneTrampoline is b3RecReplayPlaneTrampoline.
// 3D delivers a whole shape's planes in one call. The recorded group starts at the cursor with its
// plane count and user return replicated on each hit, so compare the batch and advance by the
// recorded count to stay aligned with the stream even when the count itself diverged.
// PORT: const b3PlaneResult* planes + planeCount is a slice.
func recReplayPlaneTrampoline(id ShapeID, planes []PlaneResult, ctx any) bool {
	rc := ctx.(*recReplayQueryCtx)
	planeCount := len(planes)
	if rc.cursor >= rc.count {
		rc.rdr.diverged = true
		return true
	}
	head := &rc.hits[rc.cursor]
	recordedCount := int(head.planeCount)
	ret := head.userReturnB
	if id.Index1 != head.id.Index1 || id.Generation != head.id.Generation || recordedCount != planeCount {
		rc.rdr.diverged = true
	}
	n := planeCount
	if recordedCount < planeCount {
		n = recordedCount
	}
	for i := 0; i < n; i++ {
		h := &rc.hits[rc.cursor+i]
		if recVec3Differs(h.plane.Plane.Normal, planes[i].Plane.Normal) ||
			recF32Differs(h.plane.Plane.Offset, planes[i].Plane.Offset) ||
			recVec3Differs(h.plane.Point, planes[i].Point) || h.plane.TriangleIndex != planes[i].TriangleIndex ||
			h.plane.ChildIndex != planes[i].ChildIndex || h.plane.MaterialIndex != planes[i].MaterialIndex {
			rc.rdr.diverged = true
		}
	}
	rc.cursor += recordedCount
	return ret
}

// recStashProxy is b3RecStashProxy.
// Copy a decoded proxy's points into a draw record so the overlay does not depend on reader scratch.
func recStashProxy(q *recDrawQuery, proxy *ShapeProxy) {
	count := int(proxy.Count)
	if count > MaxShapeCastPoints {
		count = MaxShapeCastPoints
	}
	q.proxyCount = int32(count)
	q.proxyRadius = proxy.Radius
	for i := 0; i < count; i++ {
		q.proxyPoints[i] = proxy.Points[i]
	}
}

// recComputeQueryBounds is b3RecComputeQueryBounds.
// Tight world-space bounds of a query's swept geometry, so the viewer can frame any query and not
// just the overlap AABB. Mover and proxy points are origin relative. A cast sweeps the shape from the
// origin to origin plus translation. The overlap AABB is already a world-space box.
func recComputeQueryBounds(q *recDrawQuery) {
	if q.kind == recqOverlapAABB {
		return
	}

	// Shape points relative to the origin, plus the fattening radius. A ray has no shape, so it falls
	// through to a single point at the origin.
	var local [MaxShapeCastPoints]Vec3
	count := 0
	radius := float32(0.0)
	switch q.kind {
	case recqCastMover, recqCollideMover:
		local[0] = q.mover.Center1
		local[1] = q.mover.Center2
		count = 2
		radius = q.mover.Radius

	case recqOverlapShape, recqCastShape:
		count = int(q.proxyCount)
		for i := 0; i < count; i++ {
			local[i] = q.proxyPoints[i]
		}
		radius = q.proxyRadius

	default:
	}
	if count == 0 {
		local[0] = Vec3Zero
		count = 1
	}

	// Sweep each point across the translation. A non-cast query has zero translation, so both ends
	// coincide and the duplicates fold away.
	end := OffsetPos(q.origin, q.translation)
	var world [2 * MaxShapeCastPoints]Vec3
	n := 0
	for i := 0; i < count; i++ {
		world[n] = ToVec3(OffsetPos(q.origin, local[i]))
		n++
		world[n] = ToVec3(OffsetPos(end, local[i]))
		n++
	}
	q.aabb = MakeAABB(world[:n], radius)
}

// recDispatchQueryOverlapAABB is b3RecDispatch_QueryOverlapAABB.
func recDispatchQueryOverlapAABB(a *recArgsQueryOverlapAABB, rdr *recReader) {
	n := recRU32(rdr)
	recEnsureHits(rdr, int(int32(n)))
	if !rdr.ok {
		return
	}
	for i := uint32(0); i < n; i++ {
		rdr.hits[i].id = recMakeShapeId(rdr, recRSHAPEID(rdr))
		rdr.hits[i].userReturnB = recRBOOL(rdr)
	}
	_ = recRTREESTATS(rdr)
	if !rdr.ok {
		return
	}
	rc := recReplayQueryCtx{rdr, rdr.hits, int(n), 0}
	rdr.replayWorldId.OverlapAABB(a.aabb, a.filter, recReplayOverlapTrampoline, &rc)
	if rc.cursor != int(n) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := recStashQueryBegin(rdr.owner, recqOverlapAABB, rdr.hits, int(n))
		q.filter = a.filter
		q.aabb = a.aabb
		recComputeQueryBounds(q)
	}
}

// recDispatchQueryOverlapShape is b3RecDispatch_QueryOverlapShape.
func recDispatchQueryOverlapShape(a *recArgsQueryOverlapShape, rdr *recReader) {
	n := recRU32(rdr)
	recEnsureHits(rdr, int(int32(n)))
	if !rdr.ok {
		return
	}
	for i := uint32(0); i < n; i++ {
		rdr.hits[i].id = recMakeShapeId(rdr, recRSHAPEID(rdr))
		rdr.hits[i].userReturnB = recRBOOL(rdr)
	}
	_ = recRTREESTATS(rdr)
	if !rdr.ok {
		return
	}
	rc := recReplayQueryCtx{rdr, rdr.hits, int(n), 0}
	rdr.replayWorldId.OverlapShape(a.origin, &a.proxy, a.filter, recReplayOverlapTrampoline, &rc)
	if rc.cursor != int(n) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := recStashQueryBegin(rdr.owner, recqOverlapShape, rdr.hits, int(n))
		q.filter = a.filter
		q.origin = a.origin
		recStashProxy(q, &a.proxy)
		recComputeQueryBounds(q)
	}
}

// recDispatchQueryCastRay is b3RecDispatch_QueryCastRay.
func recDispatchQueryCastRay(a *recArgsQueryCastRay, rdr *recReader) {
	n := recRU32(rdr)
	recEnsureHits(rdr, int(int32(n)))
	if !rdr.ok {
		return
	}
	for i := uint32(0); i < n; i++ {
		rdr.hits[i].id = recMakeShapeId(rdr, recRSHAPEID(rdr))
		rdr.hits[i].point = recRPOSITION(rdr)
		rdr.hits[i].normal = recRVEC3(rdr)
		rdr.hits[i].fraction = recRF32(rdr)
		rdr.hits[i].userMaterialId = recRU64(rdr)
		rdr.hits[i].triangleIndex = recRI32(rdr)
		rdr.hits[i].childIndex = recRI32(rdr)
		rdr.hits[i].userReturnF = recRF32(rdr)
	}
	_ = recRTREESTATS(rdr)
	if !rdr.ok {
		return
	}
	rc := recReplayQueryCtx{rdr, rdr.hits, int(n), 0}
	rdr.replayWorldId.CastRay(a.origin, a.translation, a.filter, recReplayCastTrampoline, &rc)
	if rc.cursor != int(n) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := recStashQueryBegin(rdr.owner, recqCastRay, rdr.hits, int(n))
		q.filter = a.filter
		q.origin = a.origin
		q.translation = a.translation
		recComputeQueryBounds(q)
	}
}

// recDispatchQueryCastShape is b3RecDispatch_QueryCastShape.
func recDispatchQueryCastShape(a *recArgsQueryCastShape, rdr *recReader) {
	n := recRU32(rdr)
	recEnsureHits(rdr, int(int32(n)))
	if !rdr.ok {
		return
	}
	for i := uint32(0); i < n; i++ {
		rdr.hits[i].id = recMakeShapeId(rdr, recRSHAPEID(rdr))
		rdr.hits[i].point = recRPOSITION(rdr)
		rdr.hits[i].normal = recRVEC3(rdr)
		rdr.hits[i].fraction = recRF32(rdr)
		rdr.hits[i].userMaterialId = recRU64(rdr)
		rdr.hits[i].triangleIndex = recRI32(rdr)
		rdr.hits[i].childIndex = recRI32(rdr)
		rdr.hits[i].userReturnF = recRF32(rdr)
	}
	_ = recRTREESTATS(rdr)
	if !rdr.ok {
		return
	}
	rc := recReplayQueryCtx{rdr, rdr.hits, int(n), 0}
	rdr.replayWorldId.CastShape(a.origin, &a.proxy, a.translation, a.filter, recReplayCastTrampoline, &rc)
	if rc.cursor != int(n) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := recStashQueryBegin(rdr.owner, recqCastShape, rdr.hits, int(n))
		q.filter = a.filter
		q.origin = a.origin
		q.translation = a.translation
		recStashProxy(q, &a.proxy)
		recComputeQueryBounds(q)
	}
}

// recDispatchQueryCastRayClosest is b3RecDispatch_QueryCastRayClosest.
func recDispatchQueryCastRayClosest(a *recArgsQueryCastRayClosest, rdr *recReader) {
	rec := recRRAYRESULT(rdr)
	if !rdr.ok {
		return
	}
	got := rdr.replayWorldId.CastRayClosest(a.origin, a.translation, a.filter)
	recId := recMakeShapeId(rdr, rec.ShapeId)
	if got.Hit != rec.Hit ||
		(got.Hit &&
			(got.ShapeId.Index1 != recId.Index1 || got.ShapeId.Generation != recId.Generation ||
				recVec3Differs(SubPos(got.Point, rec.Point), Vec3Zero) || recVec3Differs(got.Normal, rec.Normal) ||
				recF32Differs(got.Fraction, rec.Fraction) || got.UserMaterialId != rec.UserMaterialId)) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		// Stash the closest result as a single pooled hit so the shared draw loop renders its point.
		var h recRecordedHit
		h.id = recId
		h.point = rec.Point
		h.normal = rec.Normal
		h.fraction = rec.Fraction
		hitCount := 0
		if rec.Hit {
			hitCount = 1
		}
		q := recStashQueryBegin(rdr.owner, recqCastRayClosest, []recRecordedHit{h}, hitCount)
		q.filter = a.filter
		q.origin = a.origin
		q.translation = a.translation
		q.rayResult = rec
		recComputeQueryBounds(q)
	}
}

// recDispatchQueryCastMover is b3RecDispatch_QueryCastMover.
func recDispatchQueryCastMover(a *recArgsQueryCastMover, rdr *recReader) {
	n := recRU32(rdr)
	recEnsureHits(rdr, int(int32(n)))
	if !rdr.ok {
		return
	}
	for i := uint32(0); i < n; i++ {
		rdr.hits[i].id = recMakeShapeId(rdr, recRSHAPEID(rdr))
		rdr.hits[i].userReturnB = recRBOOL(rdr)
	}
	recFraction := recRF32(rdr)
	if !rdr.ok {
		return
	}
	rc := recReplayQueryCtx{rdr, rdr.hits, int(n), 0}
	got := rdr.replayWorldId.CastMover(a.origin, &a.mover, a.translation, a.filter,
		recReplayMoverFilterTrampoline, &rc)
	if rc.cursor != int(n) || recF32Differs(got, recFraction) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := recStashQueryBegin(rdr.owner, recqCastMover, nil, 0)
		q.filter = a.filter
		q.origin = a.origin
		q.mover = a.mover
		q.translation = a.translation
		q.castFraction = recFraction
		recComputeQueryBounds(q)
	}
}

// recDispatchQueryCollideMover is b3RecDispatch_QueryCollideMover.
func recDispatchQueryCollideMover(a *recArgsQueryCollideMover, rdr *recReader) {
	// Recorded as shapeCount groups, each: shapeId, planeCount, planeCount planes, user return. Flatten
	// into one hit per plane with the group's count and return replicated, so the replay walker can
	// re-group per shape.
	shapeCount := recRU32(rdr)
	total := 0
	for s := uint32(0); s < shapeCount; s++ {
		id := recMakeShapeId(rdr, recRSHAPEID(rdr))
		planeCount := int(recRI32(rdr))
		if planeCount < 0 {
			planeCount = 0
		}
		recEnsureHits(rdr, total+planeCount)
		if !rdr.ok {
			return
		}
		for i := 0; i < planeCount; i++ {
			rdr.hits[total+i].plane = recRPLANERESULT(rdr)
		}
		ret := recRBOOL(rdr)
		for i := 0; i < planeCount; i++ {
			rdr.hits[total+i].id = id
			rdr.hits[total+i].planeCount = int32(planeCount)
			rdr.hits[total+i].userReturnB = ret
		}
		total += planeCount
	}
	if !rdr.ok {
		return
	}
	rc := recReplayQueryCtx{rdr, rdr.hits, total, 0}
	rdr.replayWorldId.CollideMover(a.origin, &a.mover, a.filter, recReplayPlaneTrampoline, &rc)
	if rc.cursor != total {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := recStashQueryBegin(rdr.owner, recqCollideMover, rdr.hits, total)
		q.filter = a.filter
		q.origin = a.origin
		q.mover = a.mover
		recComputeQueryBounds(q)
	}
}

// recDispatchQueryTag is b3RecDispatch_QueryTag.
// Stash the identity key of the query that immediately follows. Consumed by the next stash.
func recDispatchQueryTag(a *recArgsQueryTag, rdr *recReader) {
	rdr.pendingQueryKey = a.key
}

// recDispatchOne is b3RecDispatchOne.
// X-macro dispatch switch: read opcode+u24 payloadSize, dispatch, skip unknown ops.
// Returns the opcode dispatched, or -1 when the stream is exhausted or broken.
// PORT: the B3_REC_OP / ARG expansion over recording_ops.inl is written out, one case per op in
// manifest order: each case zeroes a recArgsName, reads every ARG with recR<TAG> and dispatches when
// the reads stayed in bounds.
func recDispatchOne(rdr *recReader) int {
	if rdr.cursor >= rdr.size || !rdr.ok {
		return -1
	}
	opcode := recRU8(rdr)
	payloadSize := recRU24(rdr)
	if !rdr.ok {
		return -1
	}

	payloadStart := rdr.cursor

	switch opcode {
	case recOpDestroyWorld:
		var a recArgsDestroyWorld
		a.world = recRWORLDID(rdr)
		if rdr.ok {
			recDispatchDestroyWorld(&a, rdr)
		}
	case recOpStep:
		var a recArgsStep
		a.world = recRWORLDID(rdr)
		a.dt = recRF32(rdr)
		a.subStepCount = recRI32(rdr)
		if rdr.ok {
			recDispatchStep(&a, rdr)
		}
	case recOpWorldEnableSleeping:
		var a recArgsWorldEnableSleeping
		a.world = recRWORLDID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWorldEnableSleeping(&a, rdr)
		}
	case recOpWorldEnableContinuous:
		var a recArgsWorldEnableContinuous
		a.world = recRWORLDID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWorldEnableContinuous(&a, rdr)
		}
	case recOpWorldSetRestitutionThreshold:
		var a recArgsWorldSetRestitutionThreshold
		a.world = recRWORLDID(rdr)
		a.value = recRF32(rdr)
		if rdr.ok {
			recDispatchWorldSetRestitutionThreshold(&a, rdr)
		}
	case recOpWorldSetHitEventThreshold:
		var a recArgsWorldSetHitEventThreshold
		a.world = recRWORLDID(rdr)
		a.value = recRF32(rdr)
		if rdr.ok {
			recDispatchWorldSetHitEventThreshold(&a, rdr)
		}
	case recOpWorldSetGravity:
		var a recArgsWorldSetGravity
		a.world = recRWORLDID(rdr)
		a.gravity = recRVEC3(rdr)
		if rdr.ok {
			recDispatchWorldSetGravity(&a, rdr)
		}
	case recOpWorldExplode:
		var a recArgsWorldExplode
		a.world = recRWORLDID(rdr)
		a.def = recREXPLOSIONDEF(rdr)
		if rdr.ok {
			recDispatchWorldExplode(&a, rdr)
		}
	case recOpWorldSetContactTuning:
		var a recArgsWorldSetContactTuning
		a.world = recRWORLDID(rdr)
		a.hertz = recRF32(rdr)
		a.dampingRatio = recRF32(rdr)
		a.contactSpeed = recRF32(rdr)
		if rdr.ok {
			recDispatchWorldSetContactTuning(&a, rdr)
		}
	case recOpWorldSetContactRecycleDistance:
		var a recArgsWorldSetContactRecycleDistance
		a.world = recRWORLDID(rdr)
		a.recycleDistance = recRF32(rdr)
		if rdr.ok {
			recDispatchWorldSetContactRecycleDistance(&a, rdr)
		}
	case recOpWorldSetMaximumLinearSpeed:
		var a recArgsWorldSetMaximumLinearSpeed
		a.world = recRWORLDID(rdr)
		a.maximumLinearSpeed = recRF32(rdr)
		if rdr.ok {
			recDispatchWorldSetMaximumLinearSpeed(&a, rdr)
		}
	case recOpWorldEnableWarmStarting:
		var a recArgsWorldEnableWarmStarting
		a.world = recRWORLDID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWorldEnableWarmStarting(&a, rdr)
		}
	case recOpWorldRebuildStaticTree:
		var a recArgsWorldRebuildStaticTree
		a.world = recRWORLDID(rdr)
		if rdr.ok {
			recDispatchWorldRebuildStaticTree(&a, rdr)
		}
	case recOpWorldEnableSpeculative:
		var a recArgsWorldEnableSpeculative
		a.world = recRWORLDID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWorldEnableSpeculative(&a, rdr)
		}
	case recOpWorldSetRestitutionIterations:
		var a recArgsWorldSetRestitutionIterations
		a.world = recRWORLDID(rdr)
		a.iterations = recRI32(rdr)
		if rdr.ok {
			recDispatchWorldSetRestitutionIterations(&a, rdr)
		}
	case recOpWorldEnableRestitutionPropagation:
		var a recArgsWorldEnableRestitutionPropagation
		a.world = recRWORLDID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWorldEnableRestitutionPropagation(&a, rdr)
		}
	case recOpWorldSetIsotropicInertiaTolerance:
		var a recArgsWorldSetIsotropicInertiaTolerance
		a.world = recRWORLDID(rdr)
		a.tolerance = recRF32(rdr)
		if rdr.ok {
			recDispatchWorldSetIsotropicInertiaTolerance(&a, rdr)
		}
	case recOpCreateBody:
		var a recArgsCreateBody
		a.world = recRWORLDID(rdr)
		a.def = recRBODYDEF(rdr)
		if rdr.ok {
			recDispatchCreateBody(&a, rdr)
		}
	case recOpDestroyBody:
		var a recArgsDestroyBody
		a.body = recRBODYID(rdr)
		if rdr.ok {
			recDispatchDestroyBody(&a, rdr)
		}
	case recOpBodySetTransform:
		var a recArgsBodySetTransform
		a.body = recRBODYID(rdr)
		a.position = recRPOSITION(rdr)
		a.rotation = recRQUAT(rdr)
		if rdr.ok {
			recDispatchBodySetTransform(&a, rdr)
		}
	case recOpBodySetLinearVelocity:
		var a recArgsBodySetLinearVelocity
		a.body = recRBODYID(rdr)
		a.v = recRVEC3(rdr)
		if rdr.ok {
			recDispatchBodySetLinearVelocity(&a, rdr)
		}
	case recOpBodySetType:
		var a recArgsBodySetType
		a.body = recRBODYID(rdr)
		a.typ = recRI32(rdr)
		if rdr.ok {
			recDispatchBodySetType(&a, rdr)
		}
	case recOpBodySetName:
		var a recArgsBodySetName
		a.body = recRBODYID(rdr)
		a.name = recRSTR(rdr)
		if rdr.ok {
			recDispatchBodySetName(&a, rdr)
		}
	case recOpBodySetAngularVelocity:
		var a recArgsBodySetAngularVelocity
		a.body = recRBODYID(rdr)
		a.w = recRVEC3(rdr)
		if rdr.ok {
			recDispatchBodySetAngularVelocity(&a, rdr)
		}
	case recOpBodySetTargetTransform:
		var a recArgsBodySetTargetTransform
		a.body = recRBODYID(rdr)
		a.target = recRWORLDXF(rdr)
		a.timeStep = recRF32(rdr)
		a.wake = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodySetTargetTransform(&a, rdr)
		}
	case recOpBodyApplyForce:
		var a recArgsBodyApplyForce
		a.body = recRBODYID(rdr)
		a.force = recRVEC3(rdr)
		a.point = recRPOSITION(rdr)
		a.wake = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyApplyForce(&a, rdr)
		}
	case recOpBodyApplyForceToCenter:
		var a recArgsBodyApplyForceToCenter
		a.body = recRBODYID(rdr)
		a.force = recRVEC3(rdr)
		a.wake = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyApplyForceToCenter(&a, rdr)
		}
	case recOpBodyApplyTorque:
		var a recArgsBodyApplyTorque
		a.body = recRBODYID(rdr)
		a.torque = recRVEC3(rdr)
		a.wake = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyApplyTorque(&a, rdr)
		}
	case recOpBodyApplyLinearImpulse:
		var a recArgsBodyApplyLinearImpulse
		a.body = recRBODYID(rdr)
		a.impulse = recRVEC3(rdr)
		a.point = recRPOSITION(rdr)
		a.wake = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyApplyLinearImpulse(&a, rdr)
		}
	case recOpBodyApplyLinearImpulseToCenter:
		var a recArgsBodyApplyLinearImpulseToCenter
		a.body = recRBODYID(rdr)
		a.impulse = recRVEC3(rdr)
		a.wake = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyApplyLinearImpulseToCenter(&a, rdr)
		}
	case recOpBodyApplyAngularImpulse:
		var a recArgsBodyApplyAngularImpulse
		a.body = recRBODYID(rdr)
		a.impulse = recRVEC3(rdr)
		a.wake = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyApplyAngularImpulse(&a, rdr)
		}
	case recOpBodySetMassData:
		var a recArgsBodySetMassData
		a.body = recRBODYID(rdr)
		a.massData = recRMASSDATA(rdr)
		if rdr.ok {
			recDispatchBodySetMassData(&a, rdr)
		}
	case recOpBodyApplyMassFromShapes:
		var a recArgsBodyApplyMassFromShapes
		a.body = recRBODYID(rdr)
		if rdr.ok {
			recDispatchBodyApplyMassFromShapes(&a, rdr)
		}
	case recOpBodySetLinearDamping:
		var a recArgsBodySetLinearDamping
		a.body = recRBODYID(rdr)
		a.damping = recRF32(rdr)
		if rdr.ok {
			recDispatchBodySetLinearDamping(&a, rdr)
		}
	case recOpBodySetAngularDamping:
		var a recArgsBodySetAngularDamping
		a.body = recRBODYID(rdr)
		a.damping = recRF32(rdr)
		if rdr.ok {
			recDispatchBodySetAngularDamping(&a, rdr)
		}
	case recOpBodySetGravityScale:
		var a recArgsBodySetGravityScale
		a.body = recRBODYID(rdr)
		a.scale = recRF32(rdr)
		if rdr.ok {
			recDispatchBodySetGravityScale(&a, rdr)
		}
	case recOpBodySetAwake:
		var a recArgsBodySetAwake
		a.body = recRBODYID(rdr)
		a.awake = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodySetAwake(&a, rdr)
		}
	case recOpBodyEnableSleep:
		var a recArgsBodyEnableSleep
		a.body = recRBODYID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyEnableSleep(&a, rdr)
		}
	case recOpBodySetSleepThreshold:
		var a recArgsBodySetSleepThreshold
		a.body = recRBODYID(rdr)
		a.threshold = recRF32(rdr)
		if rdr.ok {
			recDispatchBodySetSleepThreshold(&a, rdr)
		}
	case recOpBodyDisable:
		var a recArgsBodyDisable
		a.body = recRBODYID(rdr)
		if rdr.ok {
			recDispatchBodyDisable(&a, rdr)
		}
	case recOpBodyEnable:
		var a recArgsBodyEnable
		a.body = recRBODYID(rdr)
		if rdr.ok {
			recDispatchBodyEnable(&a, rdr)
		}
	case recOpBodySetMotionLocks:
		var a recArgsBodySetMotionLocks
		a.body = recRBODYID(rdr)
		a.locks = recRLOCKS(rdr)
		if rdr.ok {
			recDispatchBodySetMotionLocks(&a, rdr)
		}
	case recOpBodySetBullet:
		var a recArgsBodySetBullet
		a.body = recRBODYID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodySetBullet(&a, rdr)
		}
	case recOpBodyEnableContactRecycling:
		var a recArgsBodyEnableContactRecycling
		a.body = recRBODYID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyEnableContactRecycling(&a, rdr)
		}
	case recOpBodyEnableHitEvents:
		var a recArgsBodyEnableHitEvents
		a.body = recRBODYID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyEnableHitEvents(&a, rdr)
		}
	case recOpBodyAllowFastRotation:
		var a recArgsBodyAllowFastRotation
		a.body = recRBODYID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchBodyAllowFastRotation(&a, rdr)
		}
	case recOpBodySetSafetyFactor:
		var a recArgsBodySetSafetyFactor
		a.body = recRBODYID(rdr)
		a.value = recRF32(rdr)
		if rdr.ok {
			recDispatchBodySetSafetyFactor(&a, rdr)
		}
	case recOpCreateSphereShape:
		var a recArgsCreateSphereShape
		a.body = recRBODYID(rdr)
		a.def = recRSHAPEDEF(rdr)
		a.sphere = recRSPHERE(rdr)
		if rdr.ok {
			recDispatchCreateSphereShape(&a, rdr)
		}
	case recOpCreateCapsuleShape:
		var a recArgsCreateCapsuleShape
		a.body = recRBODYID(rdr)
		a.def = recRSHAPEDEF(rdr)
		a.capsule = recRCAPSULE(rdr)
		if rdr.ok {
			recDispatchCreateCapsuleShape(&a, rdr)
		}
	case recOpCreateHullShape:
		var a recArgsCreateHullShape
		a.body = recRBODYID(rdr)
		a.def = recRSHAPEDEF(rdr)
		a.geometryId = recRGEOMID(rdr)
		if rdr.ok {
			recDispatchCreateHullShape(&a, rdr)
		}
	case recOpCreateMeshShape:
		var a recArgsCreateMeshShape
		a.body = recRBODYID(rdr)
		a.def = recRSHAPEDEF(rdr)
		a.geometryId = recRGEOMID(rdr)
		a.scale = recRVEC3(rdr)
		if rdr.ok {
			recDispatchCreateMeshShape(&a, rdr)
		}
	case recOpCreateHeightFieldShape:
		var a recArgsCreateHeightFieldShape
		a.body = recRBODYID(rdr)
		a.def = recRSHAPEDEF(rdr)
		a.geometryId = recRGEOMID(rdr)
		if rdr.ok {
			recDispatchCreateHeightFieldShape(&a, rdr)
		}
	case recOpCreateCompoundShape:
		var a recArgsCreateCompoundShape
		a.body = recRBODYID(rdr)
		a.def = recRSHAPEDEF(rdr)
		a.geometryId = recRGEOMID(rdr)
		if rdr.ok {
			recDispatchCreateCompoundShape(&a, rdr)
		}
	case recOpDestroyShape:
		var a recArgsDestroyShape
		a.shape = recRSHAPEID(rdr)
		a.updateBodyMass = recRBOOL(rdr)
		if rdr.ok {
			recDispatchDestroyShape(&a, rdr)
		}
	case recOpShapeSetDensity:
		var a recArgsShapeSetDensity
		a.shape = recRSHAPEID(rdr)
		a.density = recRF32(rdr)
		a.updateBodyMass = recRBOOL(rdr)
		if rdr.ok {
			recDispatchShapeSetDensity(&a, rdr)
		}
	case recOpShapeSetFriction:
		var a recArgsShapeSetFriction
		a.shape = recRSHAPEID(rdr)
		a.friction = recRF32(rdr)
		if rdr.ok {
			recDispatchShapeSetFriction(&a, rdr)
		}
	case recOpShapeSetRestitution:
		var a recArgsShapeSetRestitution
		a.shape = recRSHAPEID(rdr)
		a.restitution = recRF32(rdr)
		if rdr.ok {
			recDispatchShapeSetRestitution(&a, rdr)
		}
	case recOpShapeSetSurfaceMaterial:
		var a recArgsShapeSetSurfaceMaterial
		a.shape = recRSHAPEID(rdr)
		a.material = recRMATERIAL(rdr)
		if rdr.ok {
			recDispatchShapeSetSurfaceMaterial(&a, rdr)
		}
	case recOpShapeSetFilter:
		var a recArgsShapeSetFilter
		a.shape = recRSHAPEID(rdr)
		a.filter = recRFILTER(rdr)
		a.invokeContacts = recRBOOL(rdr)
		if rdr.ok {
			recDispatchShapeSetFilter(&a, rdr)
		}
	case recOpShapeEnableSensorEvents:
		var a recArgsShapeEnableSensorEvents
		a.shape = recRSHAPEID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchShapeEnableSensorEvents(&a, rdr)
		}
	case recOpShapeEnableContactEvents:
		var a recArgsShapeEnableContactEvents
		a.shape = recRSHAPEID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchShapeEnableContactEvents(&a, rdr)
		}
	case recOpShapeEnablePreSolveEvents:
		var a recArgsShapeEnablePreSolveEvents
		a.shape = recRSHAPEID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchShapeEnablePreSolveEvents(&a, rdr)
		}
	case recOpShapeEnableHitEvents:
		var a recArgsShapeEnableHitEvents
		a.shape = recRSHAPEID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchShapeEnableHitEvents(&a, rdr)
		}
	case recOpShapeSetSphere:
		var a recArgsShapeSetSphere
		a.shape = recRSHAPEID(rdr)
		a.sphere = recRSPHERE(rdr)
		if rdr.ok {
			recDispatchShapeSetSphere(&a, rdr)
		}
	case recOpShapeSetCapsule:
		var a recArgsShapeSetCapsule
		a.shape = recRSHAPEID(rdr)
		a.capsule = recRCAPSULE(rdr)
		if rdr.ok {
			recDispatchShapeSetCapsule(&a, rdr)
		}
	case recOpShapeApplyWind:
		var a recArgsShapeApplyWind
		a.shape = recRSHAPEID(rdr)
		a.wind = recRVEC3(rdr)
		a.drag = recRF32(rdr)
		a.lift = recRF32(rdr)
		a.maxSpeed = recRF32(rdr)
		a.wake = recRBOOL(rdr)
		if rdr.ok {
			recDispatchShapeApplyWind(&a, rdr)
		}
	case recOpShapeSetName:
		var a recArgsShapeSetName
		a.shape = recRSHAPEID(rdr)
		a.name = recRSTR(rdr)
		if rdr.ok {
			recDispatchShapeSetName(&a, rdr)
		}
	case recOpShapeSetMeshMaterial:
		var a recArgsShapeSetMeshMaterial
		a.shape = recRSHAPEID(rdr)
		a.material = recRMATERIAL(rdr)
		a.index = recRI32(rdr)
		if rdr.ok {
			recDispatchShapeSetMeshMaterial(&a, rdr)
		}
	case recOpShapeSetHull:
		var a recArgsShapeSetHull
		a.shape = recRSHAPEID(rdr)
		a.geometryId = recRGEOMID(rdr)
		if rdr.ok {
			recDispatchShapeSetHull(&a, rdr)
		}
	case recOpShapeSetMesh:
		var a recArgsShapeSetMesh
		a.shape = recRSHAPEID(rdr)
		a.geometryId = recRGEOMID(rdr)
		a.scale = recRVEC3(rdr)
		if rdr.ok {
			recDispatchShapeSetMesh(&a, rdr)
		}
	case recOpCreateParallelJoint:
		var a recArgsCreateParallelJoint
		a.world = recRWORLDID(rdr)
		a.def = recRPARALLELJOINTDEF(rdr)
		if rdr.ok {
			recDispatchCreateParallelJoint(&a, rdr)
		}
	case recOpCreateDistanceJoint:
		var a recArgsCreateDistanceJoint
		a.world = recRWORLDID(rdr)
		a.def = recRDISTANCEJOINTDEF(rdr)
		if rdr.ok {
			recDispatchCreateDistanceJoint(&a, rdr)
		}
	case recOpCreateFilterJoint:
		var a recArgsCreateFilterJoint
		a.world = recRWORLDID(rdr)
		a.def = recRFILTERJOINTDEF(rdr)
		if rdr.ok {
			recDispatchCreateFilterJoint(&a, rdr)
		}
	case recOpCreateMotorJoint:
		var a recArgsCreateMotorJoint
		a.world = recRWORLDID(rdr)
		a.def = recRMOTORJOINTDEF(rdr)
		if rdr.ok {
			recDispatchCreateMotorJoint(&a, rdr)
		}
	case recOpCreatePrismaticJoint:
		var a recArgsCreatePrismaticJoint
		a.world = recRWORLDID(rdr)
		a.def = recRPRISMATICJOINTDEF(rdr)
		if rdr.ok {
			recDispatchCreatePrismaticJoint(&a, rdr)
		}
	case recOpCreateRevoluteJoint:
		var a recArgsCreateRevoluteJoint
		a.world = recRWORLDID(rdr)
		a.def = recRREVOLUTEJOINTDEF(rdr)
		if rdr.ok {
			recDispatchCreateRevoluteJoint(&a, rdr)
		}
	case recOpCreateSphericalJoint:
		var a recArgsCreateSphericalJoint
		a.world = recRWORLDID(rdr)
		a.def = recRSPHERICALJOINTDEF(rdr)
		if rdr.ok {
			recDispatchCreateSphericalJoint(&a, rdr)
		}
	case recOpCreateWeldJoint:
		var a recArgsCreateWeldJoint
		a.world = recRWORLDID(rdr)
		a.def = recRWELDJOINTDEF(rdr)
		if rdr.ok {
			recDispatchCreateWeldJoint(&a, rdr)
		}
	case recOpCreateWheelJoint:
		var a recArgsCreateWheelJoint
		a.world = recRWORLDID(rdr)
		a.def = recRWHEELJOINTDEF(rdr)
		if rdr.ok {
			recDispatchCreateWheelJoint(&a, rdr)
		}
	case recOpDestroyJoint:
		var a recArgsDestroyJoint
		a.joint = recRJOINTID(rdr)
		a.wakeAttached = recRBOOL(rdr)
		if rdr.ok {
			recDispatchDestroyJoint(&a, rdr)
		}
	case recOpJointSetLocalFrameA:
		var a recArgsJointSetLocalFrameA
		a.joint = recRJOINTID(rdr)
		a.localFrame = recRTRANSFORM(rdr)
		if rdr.ok {
			recDispatchJointSetLocalFrameA(&a, rdr)
		}
	case recOpJointSetLocalFrameB:
		var a recArgsJointSetLocalFrameB
		a.joint = recRJOINTID(rdr)
		a.localFrame = recRTRANSFORM(rdr)
		if rdr.ok {
			recDispatchJointSetLocalFrameB(&a, rdr)
		}
	case recOpJointSetCollideConnected:
		var a recArgsJointSetCollideConnected
		a.joint = recRJOINTID(rdr)
		a.shouldCollide = recRBOOL(rdr)
		if rdr.ok {
			recDispatchJointSetCollideConnected(&a, rdr)
		}
	case recOpJointWakeBodies:
		var a recArgsJointWakeBodies
		a.joint = recRJOINTID(rdr)
		if rdr.ok {
			recDispatchJointWakeBodies(&a, rdr)
		}
	case recOpJointSetConstraintTuning:
		var a recArgsJointSetConstraintTuning
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchJointSetConstraintTuning(&a, rdr)
		}
	case recOpJointSetForceThreshold:
		var a recArgsJointSetForceThreshold
		a.joint = recRJOINTID(rdr)
		a.threshold = recRF32(rdr)
		if rdr.ok {
			recDispatchJointSetForceThreshold(&a, rdr)
		}
	case recOpJointSetTorqueThreshold:
		var a recArgsJointSetTorqueThreshold
		a.joint = recRJOINTID(rdr)
		a.threshold = recRF32(rdr)
		if rdr.ok {
			recDispatchJointSetTorqueThreshold(&a, rdr)
		}
	case recOpParallelJointSetSpringHertz:
		var a recArgsParallelJointSetSpringHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchParallelJointSetSpringHertz(&a, rdr)
		}
	case recOpParallelJointSetSpringDampingRatio:
		var a recArgsParallelJointSetSpringDampingRatio
		a.joint = recRJOINTID(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchParallelJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpParallelJointSetMaxTorque:
		var a recArgsParallelJointSetMaxTorque
		a.joint = recRJOINTID(rdr)
		a.maxTorque = recRF32(rdr)
		if rdr.ok {
			recDispatchParallelJointSetMaxTorque(&a, rdr)
		}
	case recOpDistanceJointSetLength:
		var a recArgsDistanceJointSetLength
		a.joint = recRJOINTID(rdr)
		a.length = recRF32(rdr)
		if rdr.ok {
			recDispatchDistanceJointSetLength(&a, rdr)
		}
	case recOpDistanceJointEnableSpring:
		var a recArgsDistanceJointEnableSpring
		a.joint = recRJOINTID(rdr)
		a.enableSpring = recRBOOL(rdr)
		if rdr.ok {
			recDispatchDistanceJointEnableSpring(&a, rdr)
		}
	case recOpDistanceJointSetSpringForceRange:
		var a recArgsDistanceJointSetSpringForceRange
		a.joint = recRJOINTID(rdr)
		a.lowerForce = recRF32(rdr)
		a.upperForce = recRF32(rdr)
		if rdr.ok {
			recDispatchDistanceJointSetSpringForceRange(&a, rdr)
		}
	case recOpDistanceJointSetSpringHertz:
		var a recArgsDistanceJointSetSpringHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchDistanceJointSetSpringHertz(&a, rdr)
		}
	case recOpDistanceJointSetSpringDampingRatio:
		var a recArgsDistanceJointSetSpringDampingRatio
		a.joint = recRJOINTID(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchDistanceJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpDistanceJointEnableLimit:
		var a recArgsDistanceJointEnableLimit
		a.joint = recRJOINTID(rdr)
		a.enableLimit = recRBOOL(rdr)
		if rdr.ok {
			recDispatchDistanceJointEnableLimit(&a, rdr)
		}
	case recOpDistanceJointSetLengthRange:
		var a recArgsDistanceJointSetLengthRange
		a.joint = recRJOINTID(rdr)
		a.minLength = recRF32(rdr)
		a.maxLength = recRF32(rdr)
		if rdr.ok {
			recDispatchDistanceJointSetLengthRange(&a, rdr)
		}
	case recOpDistanceJointEnableMotor:
		var a recArgsDistanceJointEnableMotor
		a.joint = recRJOINTID(rdr)
		a.enableMotor = recRBOOL(rdr)
		if rdr.ok {
			recDispatchDistanceJointEnableMotor(&a, rdr)
		}
	case recOpDistanceJointSetMotorSpeed:
		var a recArgsDistanceJointSetMotorSpeed
		a.joint = recRJOINTID(rdr)
		a.motorSpeed = recRF32(rdr)
		if rdr.ok {
			recDispatchDistanceJointSetMotorSpeed(&a, rdr)
		}
	case recOpDistanceJointSetMaxMotorForce:
		var a recArgsDistanceJointSetMaxMotorForce
		a.joint = recRJOINTID(rdr)
		a.force = recRF32(rdr)
		if rdr.ok {
			recDispatchDistanceJointSetMaxMotorForce(&a, rdr)
		}
	case recOpMotorJointSetLinearVelocity:
		var a recArgsMotorJointSetLinearVelocity
		a.joint = recRJOINTID(rdr)
		a.velocity = recRVEC3(rdr)
		if rdr.ok {
			recDispatchMotorJointSetLinearVelocity(&a, rdr)
		}
	case recOpMotorJointSetAngularVelocity:
		var a recArgsMotorJointSetAngularVelocity
		a.joint = recRJOINTID(rdr)
		a.velocity = recRVEC3(rdr)
		if rdr.ok {
			recDispatchMotorJointSetAngularVelocity(&a, rdr)
		}
	case recOpMotorJointSetMaxVelocityForce:
		var a recArgsMotorJointSetMaxVelocityForce
		a.joint = recRJOINTID(rdr)
		a.maxForce = recRF32(rdr)
		if rdr.ok {
			recDispatchMotorJointSetMaxVelocityForce(&a, rdr)
		}
	case recOpMotorJointSetMaxVelocityTorque:
		var a recArgsMotorJointSetMaxVelocityTorque
		a.joint = recRJOINTID(rdr)
		a.maxTorque = recRF32(rdr)
		if rdr.ok {
			recDispatchMotorJointSetMaxVelocityTorque(&a, rdr)
		}
	case recOpMotorJointSetLinearHertz:
		var a recArgsMotorJointSetLinearHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchMotorJointSetLinearHertz(&a, rdr)
		}
	case recOpMotorJointSetLinearDampingRatio:
		var a recArgsMotorJointSetLinearDampingRatio
		a.joint = recRJOINTID(rdr)
		a.damping = recRF32(rdr)
		if rdr.ok {
			recDispatchMotorJointSetLinearDampingRatio(&a, rdr)
		}
	case recOpMotorJointSetAngularHertz:
		var a recArgsMotorJointSetAngularHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchMotorJointSetAngularHertz(&a, rdr)
		}
	case recOpMotorJointSetAngularDampingRatio:
		var a recArgsMotorJointSetAngularDampingRatio
		a.joint = recRJOINTID(rdr)
		a.damping = recRF32(rdr)
		if rdr.ok {
			recDispatchMotorJointSetAngularDampingRatio(&a, rdr)
		}
	case recOpMotorJointSetMaxSpringForce:
		var a recArgsMotorJointSetMaxSpringForce
		a.joint = recRJOINTID(rdr)
		a.maxForce = recRF32(rdr)
		if rdr.ok {
			recDispatchMotorJointSetMaxSpringForce(&a, rdr)
		}
	case recOpMotorJointSetMaxSpringTorque:
		var a recArgsMotorJointSetMaxSpringTorque
		a.joint = recRJOINTID(rdr)
		a.maxTorque = recRF32(rdr)
		if rdr.ok {
			recDispatchMotorJointSetMaxSpringTorque(&a, rdr)
		}
	case recOpPrismaticJointEnableSpring:
		var a recArgsPrismaticJointEnableSpring
		a.joint = recRJOINTID(rdr)
		a.enableSpring = recRBOOL(rdr)
		if rdr.ok {
			recDispatchPrismaticJointEnableSpring(&a, rdr)
		}
	case recOpPrismaticJointSetSpringHertz:
		var a recArgsPrismaticJointSetSpringHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchPrismaticJointSetSpringHertz(&a, rdr)
		}
	case recOpPrismaticJointSetSpringDampingRatio:
		var a recArgsPrismaticJointSetSpringDampingRatio
		a.joint = recRJOINTID(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchPrismaticJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpPrismaticJointSetTargetTranslation:
		var a recArgsPrismaticJointSetTargetTranslation
		a.joint = recRJOINTID(rdr)
		a.translation = recRF32(rdr)
		if rdr.ok {
			recDispatchPrismaticJointSetTargetTranslation(&a, rdr)
		}
	case recOpPrismaticJointEnableLimit:
		var a recArgsPrismaticJointEnableLimit
		a.joint = recRJOINTID(rdr)
		a.enableLimit = recRBOOL(rdr)
		if rdr.ok {
			recDispatchPrismaticJointEnableLimit(&a, rdr)
		}
	case recOpPrismaticJointSetLimits:
		var a recArgsPrismaticJointSetLimits
		a.joint = recRJOINTID(rdr)
		a.lower = recRF32(rdr)
		a.upper = recRF32(rdr)
		if rdr.ok {
			recDispatchPrismaticJointSetLimits(&a, rdr)
		}
	case recOpPrismaticJointEnableMotor:
		var a recArgsPrismaticJointEnableMotor
		a.joint = recRJOINTID(rdr)
		a.enableMotor = recRBOOL(rdr)
		if rdr.ok {
			recDispatchPrismaticJointEnableMotor(&a, rdr)
		}
	case recOpPrismaticJointSetMotorSpeed:
		var a recArgsPrismaticJointSetMotorSpeed
		a.joint = recRJOINTID(rdr)
		a.motorSpeed = recRF32(rdr)
		if rdr.ok {
			recDispatchPrismaticJointSetMotorSpeed(&a, rdr)
		}
	case recOpPrismaticJointSetMaxMotorForce:
		var a recArgsPrismaticJointSetMaxMotorForce
		a.joint = recRJOINTID(rdr)
		a.force = recRF32(rdr)
		if rdr.ok {
			recDispatchPrismaticJointSetMaxMotorForce(&a, rdr)
		}
	case recOpRevoluteJointEnableSpring:
		var a recArgsRevoluteJointEnableSpring
		a.joint = recRJOINTID(rdr)
		a.enableSpring = recRBOOL(rdr)
		if rdr.ok {
			recDispatchRevoluteJointEnableSpring(&a, rdr)
		}
	case recOpRevoluteJointSetSpringHertz:
		var a recArgsRevoluteJointSetSpringHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchRevoluteJointSetSpringHertz(&a, rdr)
		}
	case recOpRevoluteJointSetSpringDampingRatio:
		var a recArgsRevoluteJointSetSpringDampingRatio
		a.joint = recRJOINTID(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchRevoluteJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpRevoluteJointSetTargetAngle:
		var a recArgsRevoluteJointSetTargetAngle
		a.joint = recRJOINTID(rdr)
		a.angle = recRF32(rdr)
		if rdr.ok {
			recDispatchRevoluteJointSetTargetAngle(&a, rdr)
		}
	case recOpRevoluteJointEnableLimit:
		var a recArgsRevoluteJointEnableLimit
		a.joint = recRJOINTID(rdr)
		a.enableLimit = recRBOOL(rdr)
		if rdr.ok {
			recDispatchRevoluteJointEnableLimit(&a, rdr)
		}
	case recOpRevoluteJointSetLimits:
		var a recArgsRevoluteJointSetLimits
		a.joint = recRJOINTID(rdr)
		a.lower = recRF32(rdr)
		a.upper = recRF32(rdr)
		if rdr.ok {
			recDispatchRevoluteJointSetLimits(&a, rdr)
		}
	case recOpRevoluteJointEnableMotor:
		var a recArgsRevoluteJointEnableMotor
		a.joint = recRJOINTID(rdr)
		a.enableMotor = recRBOOL(rdr)
		if rdr.ok {
			recDispatchRevoluteJointEnableMotor(&a, rdr)
		}
	case recOpRevoluteJointSetMotorSpeed:
		var a recArgsRevoluteJointSetMotorSpeed
		a.joint = recRJOINTID(rdr)
		a.motorSpeed = recRF32(rdr)
		if rdr.ok {
			recDispatchRevoluteJointSetMotorSpeed(&a, rdr)
		}
	case recOpRevoluteJointSetMaxMotorTorque:
		var a recArgsRevoluteJointSetMaxMotorTorque
		a.joint = recRJOINTID(rdr)
		a.torque = recRF32(rdr)
		if rdr.ok {
			recDispatchRevoluteJointSetMaxMotorTorque(&a, rdr)
		}
	case recOpSphericalJointEnableConeLimit:
		var a recArgsSphericalJointEnableConeLimit
		a.joint = recRJOINTID(rdr)
		a.enableLimit = recRBOOL(rdr)
		if rdr.ok {
			recDispatchSphericalJointEnableConeLimit(&a, rdr)
		}
	case recOpSphericalJointSetConeLimit:
		var a recArgsSphericalJointSetConeLimit
		a.joint = recRJOINTID(rdr)
		a.angleRadians = recRF32(rdr)
		if rdr.ok {
			recDispatchSphericalJointSetConeLimit(&a, rdr)
		}
	case recOpSphericalJointEnableTwistLimit:
		var a recArgsSphericalJointEnableTwistLimit
		a.joint = recRJOINTID(rdr)
		a.enableLimit = recRBOOL(rdr)
		if rdr.ok {
			recDispatchSphericalJointEnableTwistLimit(&a, rdr)
		}
	case recOpSphericalJointSetTwistLimits:
		var a recArgsSphericalJointSetTwistLimits
		a.joint = recRJOINTID(rdr)
		a.lower = recRF32(rdr)
		a.upper = recRF32(rdr)
		if rdr.ok {
			recDispatchSphericalJointSetTwistLimits(&a, rdr)
		}
	case recOpSphericalJointEnableSpring:
		var a recArgsSphericalJointEnableSpring
		a.joint = recRJOINTID(rdr)
		a.enableSpring = recRBOOL(rdr)
		if rdr.ok {
			recDispatchSphericalJointEnableSpring(&a, rdr)
		}
	case recOpSphericalJointSetSpringHertz:
		var a recArgsSphericalJointSetSpringHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchSphericalJointSetSpringHertz(&a, rdr)
		}
	case recOpSphericalJointSetSpringDampingRatio:
		var a recArgsSphericalJointSetSpringDampingRatio
		a.joint = recRJOINTID(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchSphericalJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpSphericalJointSetTargetRotation:
		var a recArgsSphericalJointSetTargetRotation
		a.joint = recRJOINTID(rdr)
		a.targetRotation = recRQUAT(rdr)
		if rdr.ok {
			recDispatchSphericalJointSetTargetRotation(&a, rdr)
		}
	case recOpSphericalJointEnableMotor:
		var a recArgsSphericalJointEnableMotor
		a.joint = recRJOINTID(rdr)
		a.enableMotor = recRBOOL(rdr)
		if rdr.ok {
			recDispatchSphericalJointEnableMotor(&a, rdr)
		}
	case recOpSphericalJointSetMotorVelocity:
		var a recArgsSphericalJointSetMotorVelocity
		a.joint = recRJOINTID(rdr)
		a.motorVelocity = recRVEC3(rdr)
		if rdr.ok {
			recDispatchSphericalJointSetMotorVelocity(&a, rdr)
		}
	case recOpSphericalJointSetMaxMotorTorque:
		var a recArgsSphericalJointSetMaxMotorTorque
		a.joint = recRJOINTID(rdr)
		a.torque = recRF32(rdr)
		if rdr.ok {
			recDispatchSphericalJointSetMaxMotorTorque(&a, rdr)
		}
	case recOpWeldJointSetLinearHertz:
		var a recArgsWeldJointSetLinearHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchWeldJointSetLinearHertz(&a, rdr)
		}
	case recOpWeldJointSetLinearDampingRatio:
		var a recArgsWeldJointSetLinearDampingRatio
		a.joint = recRJOINTID(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchWeldJointSetLinearDampingRatio(&a, rdr)
		}
	case recOpWeldJointSetAngularHertz:
		var a recArgsWeldJointSetAngularHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchWeldJointSetAngularHertz(&a, rdr)
		}
	case recOpWeldJointSetAngularDampingRatio:
		var a recArgsWeldJointSetAngularDampingRatio
		a.joint = recRJOINTID(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchWeldJointSetAngularDampingRatio(&a, rdr)
		}
	case recOpWheelJointEnableSuspension:
		var a recArgsWheelJointEnableSuspension
		a.joint = recRJOINTID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWheelJointEnableSuspension(&a, rdr)
		}
	case recOpWheelJointSetSuspensionHertz:
		var a recArgsWheelJointSetSuspensionHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetSuspensionHertz(&a, rdr)
		}
	case recOpWheelJointSetSuspensionDampingRatio:
		var a recArgsWheelJointSetSuspensionDampingRatio
		a.joint = recRJOINTID(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetSuspensionDampingRatio(&a, rdr)
		}
	case recOpWheelJointEnableSuspensionLimit:
		var a recArgsWheelJointEnableSuspensionLimit
		a.joint = recRJOINTID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWheelJointEnableSuspensionLimit(&a, rdr)
		}
	case recOpWheelJointSetSuspensionLimits:
		var a recArgsWheelJointSetSuspensionLimits
		a.joint = recRJOINTID(rdr)
		a.lower = recRF32(rdr)
		a.upper = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetSuspensionLimits(&a, rdr)
		}
	case recOpWheelJointEnableSpinMotor:
		var a recArgsWheelJointEnableSpinMotor
		a.joint = recRJOINTID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWheelJointEnableSpinMotor(&a, rdr)
		}
	case recOpWheelJointSetSpinMotorSpeed:
		var a recArgsWheelJointSetSpinMotorSpeed
		a.joint = recRJOINTID(rdr)
		a.speed = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetSpinMotorSpeed(&a, rdr)
		}
	case recOpWheelJointSetMaxSpinTorque:
		var a recArgsWheelJointSetMaxSpinTorque
		a.joint = recRJOINTID(rdr)
		a.torque = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetMaxSpinTorque(&a, rdr)
		}
	case recOpWheelJointEnableSteering:
		var a recArgsWheelJointEnableSteering
		a.joint = recRJOINTID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWheelJointEnableSteering(&a, rdr)
		}
	case recOpWheelJointSetSteeringHertz:
		var a recArgsWheelJointSetSteeringHertz
		a.joint = recRJOINTID(rdr)
		a.hertz = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetSteeringHertz(&a, rdr)
		}
	case recOpWheelJointSetSteeringDampingRatio:
		var a recArgsWheelJointSetSteeringDampingRatio
		a.joint = recRJOINTID(rdr)
		a.dampingRatio = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetSteeringDampingRatio(&a, rdr)
		}
	case recOpWheelJointSetMaxSteeringTorque:
		var a recArgsWheelJointSetMaxSteeringTorque
		a.joint = recRJOINTID(rdr)
		a.torque = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetMaxSteeringTorque(&a, rdr)
		}
	case recOpWheelJointEnableSteeringLimit:
		var a recArgsWheelJointEnableSteeringLimit
		a.joint = recRJOINTID(rdr)
		a.flag = recRBOOL(rdr)
		if rdr.ok {
			recDispatchWheelJointEnableSteeringLimit(&a, rdr)
		}
	case recOpWheelJointSetSteeringLimits:
		var a recArgsWheelJointSetSteeringLimits
		a.joint = recRJOINTID(rdr)
		a.lower = recRF32(rdr)
		a.upper = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetSteeringLimits(&a, rdr)
		}
	case recOpWheelJointSetTargetSteeringAngle:
		var a recArgsWheelJointSetTargetSteeringAngle
		a.joint = recRJOINTID(rdr)
		a.radians = recRF32(rdr)
		if rdr.ok {
			recDispatchWheelJointSetTargetSteeringAngle(&a, rdr)
		}
	case recOpQueryOverlapAABB:
		var a recArgsQueryOverlapAABB
		a.world = recRWORLDID(rdr)
		a.aabb = recRAABB(rdr)
		a.filter = recRQUERYFILTER(rdr)
		if rdr.ok {
			recDispatchQueryOverlapAABB(&a, rdr)
		}
	case recOpQueryOverlapShape:
		var a recArgsQueryOverlapShape
		a.world = recRWORLDID(rdr)
		a.origin = recRPOSITION(rdr)
		a.proxy = recRSHAPEPROXY(rdr)
		a.filter = recRQUERYFILTER(rdr)
		if rdr.ok {
			recDispatchQueryOverlapShape(&a, rdr)
		}
	case recOpQueryCastRay:
		var a recArgsQueryCastRay
		a.world = recRWORLDID(rdr)
		a.origin = recRPOSITION(rdr)
		a.translation = recRVEC3(rdr)
		a.filter = recRQUERYFILTER(rdr)
		if rdr.ok {
			recDispatchQueryCastRay(&a, rdr)
		}
	case recOpQueryCastShape:
		var a recArgsQueryCastShape
		a.world = recRWORLDID(rdr)
		a.origin = recRPOSITION(rdr)
		a.proxy = recRSHAPEPROXY(rdr)
		a.translation = recRVEC3(rdr)
		a.filter = recRQUERYFILTER(rdr)
		if rdr.ok {
			recDispatchQueryCastShape(&a, rdr)
		}
	case recOpQueryCastRayClosest:
		var a recArgsQueryCastRayClosest
		a.world = recRWORLDID(rdr)
		a.origin = recRPOSITION(rdr)
		a.translation = recRVEC3(rdr)
		a.filter = recRQUERYFILTER(rdr)
		if rdr.ok {
			recDispatchQueryCastRayClosest(&a, rdr)
		}
	case recOpQueryCastMover:
		var a recArgsQueryCastMover
		a.world = recRWORLDID(rdr)
		a.origin = recRPOSITION(rdr)
		a.mover = recRCAPSULE(rdr)
		a.translation = recRVEC3(rdr)
		a.filter = recRQUERYFILTER(rdr)
		if rdr.ok {
			recDispatchQueryCastMover(&a, rdr)
		}
	case recOpQueryCollideMover:
		var a recArgsQueryCollideMover
		a.world = recRWORLDID(rdr)
		a.origin = recRPOSITION(rdr)
		a.mover = recRCAPSULE(rdr)
		a.filter = recRQUERYFILTER(rdr)
		if rdr.ok {
			recDispatchQueryCollideMover(&a, rdr)
		}
	case recOpQueryTag:
		var a recArgsQueryTag
		a.key = recRU64(rdr)
		if rdr.ok {
			recDispatchQueryTag(&a, rdr)
		}
	case recOpStateHash:
		var a recArgsStateHash
		a.world = recRWORLDID(rdr)
		a.hash = recRU64(rdr)
		if rdr.ok {
			recDispatchStateHash(&a, rdr)
		}
	case recOpRecordingBounds:
		var a recArgsRecordingBounds
		a.bounds = recRAABB(rdr)
		if rdr.ok {
			recDispatchRecordingBounds(&a, rdr)
		}
	default:
		fmt.Printf("b3ReplayFile: unknown opcode 0x%02X, skipping %d bytes\n", opcode, payloadSize)
		// payloadStart is in bounds, so size - payloadStart is the bytes left to skip over
		if payloadSize > uint32(rdr.size-payloadStart) {
			rdr.ok = false
		} else {
			rdr.cursor = payloadStart + int(payloadSize)
		}
	}
	return int(opcode)
}
