// Port of src/wheel_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: stubs until wheel_joint.c lands (milestone 4). Every function joint.c calls and every
// b3WheelJoint_* function of box3d.h is declared here with its final signature, in upstream order,
// and panics. The b3WheelJoint_* functions are methods on JointID named WheelJoint<Method>.

// WheelJointEnableSuspension is b3WheelJoint_EnableSuspension.
// Enable/disable the wheel joint spring.
func (jointId JointID) WheelJointEnableSuspension(flag bool) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_EnableSuspension not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointIsSuspensionEnabled is b3WheelJoint_IsSuspensionEnabled.
// Is the wheel joint spring enabled?
func (jointId JointID) WheelJointIsSuspensionEnabled() bool {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_IsSuspensionEnabled not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetSuspensionHertz is b3WheelJoint_SetSuspensionHertz.
// Set the wheel joint stiffness in Hertz.
func (jointId JointID) WheelJointSetSuspensionHertz(hertz float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetSuspensionHertz not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetSuspensionHertz is b3WheelJoint_GetSuspensionHertz.
// Get the wheel joint stiffness in Hertz.
func (jointId JointID) WheelJointGetSuspensionHertz() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetSuspensionHertz not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetSuspensionDampingRatio is b3WheelJoint_SetSuspensionDampingRatio.
// Set the wheel joint damping ratio, non-dimensional.
func (jointId JointID) WheelJointSetSuspensionDampingRatio(dampingRatio float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetSuspensionDampingRatio not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetSuspensionDampingRatio is b3WheelJoint_GetSuspensionDampingRatio.
// Get the wheel joint damping ratio, non-dimensional.
func (jointId JointID) WheelJointGetSuspensionDampingRatio() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetSuspensionDampingRatio not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointEnableSuspensionLimit is b3WheelJoint_EnableSuspensionLimit.
// Enable/disable the wheel joint limit.
func (jointId JointID) WheelJointEnableSuspensionLimit(flag bool) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_EnableSuspensionLimit not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointIsSuspensionLimitEnabled is b3WheelJoint_IsSuspensionLimitEnabled.
// Is the wheel joint limit enabled?
func (jointId JointID) WheelJointIsSuspensionLimitEnabled() bool {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_IsSuspensionLimitEnabled not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetLowerSuspensionLimit is b3WheelJoint_GetLowerSuspensionLimit.
// Get the wheel joint lower limit.
func (jointId JointID) WheelJointGetLowerSuspensionLimit() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetLowerSuspensionLimit not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetUpperSuspensionLimit is b3WheelJoint_GetUpperSuspensionLimit.
// Get the wheel joint upper limit.
func (jointId JointID) WheelJointGetUpperSuspensionLimit() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetUpperSuspensionLimit not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetSuspensionLimits is b3WheelJoint_SetSuspensionLimits.
// Set the wheel joint limits.
func (jointId JointID) WheelJointSetSuspensionLimits(lower, upper float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetSuspensionLimits not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointEnableSpinMotor is b3WheelJoint_EnableSpinMotor.
// Enable/disable the wheel joint motor.
func (jointId JointID) WheelJointEnableSpinMotor(flag bool) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_EnableSpinMotor not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointIsSpinMotorEnabled is b3WheelJoint_IsSpinMotorEnabled.
// Is the wheel joint motor enabled?
func (jointId JointID) WheelJointIsSpinMotorEnabled() bool {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_IsSpinMotorEnabled not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetSpinMotorSpeed is b3WheelJoint_SetSpinMotorSpeed.
// Set the wheel joint motor speed in radians per second.
func (jointId JointID) WheelJointSetSpinMotorSpeed(speed float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetSpinMotorSpeed not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetSpinMotorSpeed is b3WheelJoint_GetSpinMotorSpeed.
// Get the wheel joint motor speed in radians per second.
func (jointId JointID) WheelJointGetSpinMotorSpeed() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetSpinMotorSpeed not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetMaxSpinTorque is b3WheelJoint_SetMaxSpinTorque.
// Set the wheel joint maximum motor torque, usually in newton-meters.
func (jointId JointID) WheelJointSetMaxSpinTorque(torque float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetMaxSpinTorque not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetMaxSpinTorque is b3WheelJoint_GetMaxSpinTorque.
// Get the wheel joint maximum motor torque, usually in newton-meters.
func (jointId JointID) WheelJointGetMaxSpinTorque() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetMaxSpinTorque not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointEnableSteering is b3WheelJoint_EnableSteering.
// Enable/disable wheel steering. Steering allows the wheel to rotate about the suspension axis.
func (jointId JointID) WheelJointEnableSteering(flag bool) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_EnableSteering not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointIsSteeringEnabled is b3WheelJoint_IsSteeringEnabled.
// Can the wheel steer?
func (jointId JointID) WheelJointIsSteeringEnabled() bool {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_IsSteeringEnabled not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetSteeringHertz is b3WheelJoint_SetSteeringHertz.
// Set the wheel joint steering stiffness in Hertz.
func (jointId JointID) WheelJointSetSteeringHertz(hertz float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetSteeringHertz not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetSteeringHertz is b3WheelJoint_GetSteeringHertz.
// Get the wheel joint steering stiffness in Hertz.
func (jointId JointID) WheelJointGetSteeringHertz() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetSteeringHertz not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetSteeringDampingRatio is b3WheelJoint_SetSteeringDampingRatio.
// Set the wheel joint steering damping ratio, non-dimensional.
func (jointId JointID) WheelJointSetSteeringDampingRatio(dampingRatio float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetSteeringDampingRatio not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetSteeringDampingRatio is b3WheelJoint_GetSteeringDampingRatio.
// Get the wheel joint steering damping ratio, non-dimensional.
func (jointId JointID) WheelJointGetSteeringDampingRatio() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetSteeringDampingRatio not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetMaxSteeringTorque is b3WheelJoint_SetMaxSteeringTorque.
// Set the wheel joint maximum steering torque in N*m.
func (jointId JointID) WheelJointSetMaxSteeringTorque(torque float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetMaxSteeringTorque not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetMaxSteeringTorque is b3WheelJoint_GetMaxSteeringTorque.
// Get the wheel joint maximum steering torque in N*m.
func (jointId JointID) WheelJointGetMaxSteeringTorque() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetMaxSteeringTorque not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointEnableSteeringLimit is b3WheelJoint_EnableSteeringLimit.
// Enable/disable the wheel joint steering limit.
func (jointId JointID) WheelJointEnableSteeringLimit(flag bool) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_EnableSteeringLimit not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointIsSteeringLimitEnabled is b3WheelJoint_IsSteeringLimitEnabled.
// Is the wheel joint steering limit enabled?
func (jointId JointID) WheelJointIsSteeringLimitEnabled() bool {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_IsSteeringLimitEnabled not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetLowerSteeringLimit is b3WheelJoint_GetLowerSteeringLimit.
// Get the wheel joint lower steering limit in radians.
func (jointId JointID) WheelJointGetLowerSteeringLimit() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetLowerSteeringLimit not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetUpperSteeringLimit is b3WheelJoint_GetUpperSteeringLimit.
// Get the wheel joint upper steering limit in radians.
func (jointId JointID) WheelJointGetUpperSteeringLimit() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetUpperSteeringLimit not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetSteeringLimits is b3WheelJoint_SetSteeringLimits.
// Set the wheel joint steering limits in radians.
func (jointId JointID) WheelJointSetSteeringLimits(lowerRadians, upperRadians float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetSteeringLimits not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointSetTargetSteeringAngle is b3WheelJoint_SetTargetSteeringAngle.
// Set the wheel joint target steering angle in radians.
func (jointId JointID) WheelJointSetTargetSteeringAngle(radians float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_SetTargetSteeringAngle not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetTargetSteeringAngle is b3WheelJoint_GetTargetSteeringAngle.
// Get the wheel joint target steering angle in radians.
func (jointId JointID) WheelJointGetTargetSteeringAngle() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetTargetSteeringAngle not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetSpinSpeed is b3WheelJoint_GetSpinSpeed.
// Get the current spin speed in radians per second.
func (jointId JointID) WheelJointGetSpinSpeed() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetSpinSpeed not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetSpinTorque is b3WheelJoint_GetSpinTorque.
// Get the wheel joint current motor torque, usually in newton-meters.
func (jointId JointID) WheelJointGetSpinTorque() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetSpinTorque not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetSteeringAngle is b3WheelJoint_GetSteeringAngle.
// Get the current steering angle in radians.
func (jointId JointID) WheelJointGetSteeringAngle() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetSteeringAngle not ported yet (wheel_joint.c, milestone 4)")
}

// WheelJointGetSteeringTorque is b3WheelJoint_GetSteeringTorque.
// Get the current steering torque in N*m.
func (jointId JointID) WheelJointGetSteeringTorque() float32 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WheelJoint_GetSteeringTorque not ported yet (wheel_joint.c, milestone 4)")
}

// getWheelJointForce is b3GetWheelJointForce.
func getWheelJointForce(world *world, base *jointSim) Vec3 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3GetWheelJointForce not ported yet (wheel_joint.c, milestone 4)")
}

// getWheelJointTorque is b3GetWheelJointTorque.
func getWheelJointTorque(world *world, base *jointSim) Vec3 {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3GetWheelJointTorque not ported yet (wheel_joint.c, milestone 4)")
}

// prepareWheelJoint is b3PrepareWheelJoint.
func prepareWheelJoint(base *jointSim, context *stepContext) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3PrepareWheelJoint not ported yet (wheel_joint.c, milestone 4)")
}

// warmStartWheelJoint is b3WarmStartWheelJoint.
func warmStartWheelJoint(base *jointSim, context *stepContext) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3WarmStartWheelJoint not ported yet (wheel_joint.c, milestone 4)")
}

// solveWheelJoint is b3SolveWheelJoint.
func solveWheelJoint(base *jointSim, context *stepContext, useBias bool) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3SolveWheelJoint not ported yet (wheel_joint.c, milestone 4)")
}

// drawWheelJoint is b3DrawWheelJoint.
func drawWheelJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	// PORT: stub until wheel_joint.c lands
	panic("box3d: b3DrawWheelJoint not ported yet (wheel_joint.c, milestone 4)")
}
