// Port of src/spherical_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: stubs until spherical_joint.c lands (milestone 4). Every function joint.c calls and every
// b3SphericalJoint_* function of box3d.h is declared here with its final signature, in upstream order,
// and panics. The b3SphericalJoint_* functions are methods on JointID named SphericalJoint<Method>.

// SphericalJointEnableConeLimit is b3SphericalJoint_EnableConeLimit.
// Enable/disable the spherical joint cone limit
func (jointId JointID) SphericalJointEnableConeLimit(enableLimit bool) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_EnableConeLimit not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointIsConeLimitEnabled is b3SphericalJoint_IsConeLimitEnabled.
// Is the spherical joint cone limit enabled?
func (jointId JointID) SphericalJointIsConeLimitEnabled() bool {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_IsConeLimitEnabled not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetConeLimit is b3SphericalJoint_GetConeLimit.
// Get the spherical joint cone limit in radians
func (jointId JointID) SphericalJointGetConeLimit() float32 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetConeLimit not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointSetConeLimit is b3SphericalJoint_SetConeLimit.
// Set the spherical joint limits in radians
func (jointId JointID) SphericalJointSetConeLimit(angleRadians float32) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_SetConeLimit not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetConeAngle is b3SphericalJoint_GetConeAngle.
// Get the spherical joint current cone angle in radians.
func (jointId JointID) SphericalJointGetConeAngle() float32 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetConeAngle not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointEnableTwistLimit is b3SphericalJoint_EnableTwistLimit.
// Enable/disable the spherical joint limit
func (jointId JointID) SphericalJointEnableTwistLimit(enableLimit bool) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_EnableTwistLimit not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointIsTwistLimitEnabled is b3SphericalJoint_IsTwistLimitEnabled.
// Is the spherical joint limit enabled?
func (jointId JointID) SphericalJointIsTwistLimitEnabled() bool {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_IsTwistLimitEnabled not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetLowerTwistLimit is b3SphericalJoint_GetLowerTwistLimit.
// Get the spherical joint lower limit in radians
func (jointId JointID) SphericalJointGetLowerTwistLimit() float32 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetLowerTwistLimit not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetUpperTwistLimit is b3SphericalJoint_GetUpperTwistLimit.
// Get the spherical joint upper limit in radians
func (jointId JointID) SphericalJointGetUpperTwistLimit() float32 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetUpperTwistLimit not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointSetTwistLimits is b3SphericalJoint_SetTwistLimits.
// Set the spherical joint limits in radians
func (jointId JointID) SphericalJointSetTwistLimits(lowerLimitRadians, upperLimitRadians float32) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_SetTwistLimits not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetTwistAngle is b3SphericalJoint_GetTwistAngle.
// Get the spherical joint current twist angle in radians.
func (jointId JointID) SphericalJointGetTwistAngle() float32 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetTwistAngle not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointEnableSpring is b3SphericalJoint_EnableSpring.
// Enable/disable the spherical joint spring
func (jointId JointID) SphericalJointEnableSpring(enableSpring bool) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_EnableSpring not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointIsSpringEnabled is b3SphericalJoint_IsSpringEnabled.
// Is the spherical angular spring enabled?
func (jointId JointID) SphericalJointIsSpringEnabled() bool {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_IsSpringEnabled not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointSetTargetRotation is b3SphericalJoint_SetTargetRotation.
// Set the spherical joint spring target rotation
func (jointId JointID) SphericalJointSetTargetRotation(targetRotation Quat) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_SetTargetRotation not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetTargetRotation is b3SphericalJoint_GetTargetRotation.
// Get the spherical joint spring target rotation
func (jointId JointID) SphericalJointGetTargetRotation() Quat {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetTargetRotation not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointSetSpringHertz is b3SphericalJoint_SetSpringHertz.
// Set the spherical joint spring stiffness in Hertz
func (jointId JointID) SphericalJointSetSpringHertz(hertz float32) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_SetSpringHertz not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetSpringHertz is b3SphericalJoint_GetSpringHertz.
// Get the spherical joint spring stiffness in Hertz
func (jointId JointID) SphericalJointGetSpringHertz() float32 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetSpringHertz not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointSetSpringDampingRatio is b3SphericalJoint_SetSpringDampingRatio.
// Set the spherical joint spring damping ratio, non-dimensional
func (jointId JointID) SphericalJointSetSpringDampingRatio(dampingRatio float32) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_SetSpringDampingRatio not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetSpringDampingRatio is b3SphericalJoint_GetSpringDampingRatio.
// Get the spherical joint spring damping ratio, non-dimensional
func (jointId JointID) SphericalJointGetSpringDampingRatio() float32 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetSpringDampingRatio not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointEnableMotor is b3SphericalJoint_EnableMotor.
// Enable/disable a spherical joint motor
func (jointId JointID) SphericalJointEnableMotor(enableMotor bool) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_EnableMotor not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointIsMotorEnabled is b3SphericalJoint_IsMotorEnabled.
// Is the spherical joint motor enabled?
func (jointId JointID) SphericalJointIsMotorEnabled() bool {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_IsMotorEnabled not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointSetMotorVelocity is b3SphericalJoint_SetMotorVelocity.
// Set the spherical joint motor velocity in radians per second. This is the relative angular
// velocity between the two bodies in world space.
// motorVelocity = angularVelocityB - angularVelocityA
func (jointId JointID) SphericalJointSetMotorVelocity(motorVelocity Vec3) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_SetMotorVelocity not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetMotorVelocity is b3SphericalJoint_GetMotorVelocity.
// Get the spherical joint motor velocity in radians per second
func (jointId JointID) SphericalJointGetMotorVelocity() Vec3 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetMotorVelocity not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointSetMaxMotorTorque is b3SphericalJoint_SetMaxMotorTorque.
// Set the spherical joint maximum motor torque, usually in newton-meters
func (jointId JointID) SphericalJointSetMaxMotorTorque(torque float32) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_SetMaxMotorTorque not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetMaxMotorTorque is b3SphericalJoint_GetMaxMotorTorque.
// Get the spherical joint maximum motor torque, usually in newton-meters
func (jointId JointID) SphericalJointGetMaxMotorTorque() float32 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetMaxMotorTorque not ported yet (spherical_joint.c, milestone 4)")
}

// SphericalJointGetMotorTorque is b3SphericalJoint_GetMotorTorque.
// Get the spherical joint current motor torque, usually in newton-meters
func (jointId JointID) SphericalJointGetMotorTorque() Vec3 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SphericalJoint_GetMotorTorque not ported yet (spherical_joint.c, milestone 4)")
}

// getSphericalJointForce is b3GetSphericalJointForce.
func getSphericalJointForce(world *world, base *jointSim) Vec3 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3GetSphericalJointForce not ported yet (spherical_joint.c, milestone 4)")
}

// getSphericalJointTorque is b3GetSphericalJointTorque.
func getSphericalJointTorque(world *world, base *jointSim) Vec3 {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3GetSphericalJointTorque not ported yet (spherical_joint.c, milestone 4)")
}

// prepareSphericalJoint is b3PrepareSphericalJoint.
func prepareSphericalJoint(base *jointSim, context *stepContext) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3PrepareSphericalJoint not ported yet (spherical_joint.c, milestone 4)")
}

// warmStartSphericalJoint is b3WarmStartSphericalJoint.
func warmStartSphericalJoint(base *jointSim, context *stepContext) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3WarmStartSphericalJoint not ported yet (spherical_joint.c, milestone 4)")
}

// solveSphericalJoint is b3SolveSphericalJoint.
func solveSphericalJoint(base *jointSim, context *stepContext, useBias bool) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3SolveSphericalJoint not ported yet (spherical_joint.c, milestone 4)")
}

// drawSphericalJoint is b3DrawSphericalJoint.
func drawSphericalJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	// PORT: stub until spherical_joint.c lands
	panic("box3d: b3DrawSphericalJoint not ported yet (spherical_joint.c, milestone 4)")
}
