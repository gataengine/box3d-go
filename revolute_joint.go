// Port of src/revolute_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: stubs until revolute_joint.c lands (milestone 4). Every function joint.c calls and every
// b3RevoluteJoint_* function of box3d.h is declared here with its final signature, in upstream order,
// and panics. The b3RevoluteJoint_* functions are methods on JointID named RevoluteJoint<Method>.

// RevoluteJointEnableLimit is b3RevoluteJoint_EnableLimit.
// Enable/disable the revolute joint limit
func (jointId JointID) RevoluteJointEnableLimit(enableLimit bool) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_EnableLimit not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointIsLimitEnabled is b3RevoluteJoint_IsLimitEnabled.
// Is the revolute joint limit enabled?
func (jointId JointID) RevoluteJointIsLimitEnabled() bool {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_IsLimitEnabled not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointGetLowerLimit is b3RevoluteJoint_GetLowerLimit.
// Get the revolute joint lower limit in radians
func (jointId JointID) RevoluteJointGetLowerLimit() float32 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_GetLowerLimit not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointGetUpperLimit is b3RevoluteJoint_GetUpperLimit.
// Get the revolute joint upper limit in radians
func (jointId JointID) RevoluteJointGetUpperLimit() float32 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_GetUpperLimit not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointSetLimits is b3RevoluteJoint_SetLimits.
// Set the revolute joint limits in radians
func (jointId JointID) RevoluteJointSetLimits(lowerLimitRadians, upperLimitRadians float32) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_SetLimits not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointGetAngle is b3RevoluteJoint_GetAngle.
// Get the revolute joint current angle in radians relative to the reference angle
func (jointId JointID) RevoluteJointGetAngle() float32 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_GetAngle not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointEnableSpring is b3RevoluteJoint_EnableSpring.
// Enable/disable the revolute joint spring
func (jointId JointID) RevoluteJointEnableSpring(enableSpring bool) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_EnableSpring not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointIsSpringEnabled is b3RevoluteJoint_IsSpringEnabled.
// Is the revolute angular spring enabled?
func (jointId JointID) RevoluteJointIsSpringEnabled() bool {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_IsSpringEnabled not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointSetTargetAngle is b3RevoluteJoint_SetTargetAngle.
// Set the revolute joint target angle in radians
func (jointId JointID) RevoluteJointSetTargetAngle(targetRadians float32) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_SetTargetAngle not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointGetTargetAngle is b3RevoluteJoint_GetTargetAngle.
// Get the revolute joint target angle in radians
func (jointId JointID) RevoluteJointGetTargetAngle() float32 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_GetTargetAngle not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointSetSpringHertz is b3RevoluteJoint_SetSpringHertz.
// Set the revolute joint spring stiffness in Hertz
func (jointId JointID) RevoluteJointSetSpringHertz(hertz float32) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_SetSpringHertz not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointGetSpringHertz is b3RevoluteJoint_GetSpringHertz.
// Get the revolute joint spring stiffness in Hertz
func (jointId JointID) RevoluteJointGetSpringHertz() float32 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_GetSpringHertz not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointSetSpringDampingRatio is b3RevoluteJoint_SetSpringDampingRatio.
// Set the revolute joint spring damping ratio, non-dimensional
func (jointId JointID) RevoluteJointSetSpringDampingRatio(dampingRatio float32) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_SetSpringDampingRatio not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointGetSpringDampingRatio is b3RevoluteJoint_GetSpringDampingRatio.
// Get the revolute joint spring damping ratio, non-dimensional
func (jointId JointID) RevoluteJointGetSpringDampingRatio() float32 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_GetSpringDampingRatio not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointEnableMotor is b3RevoluteJoint_EnableMotor.
// Enable/disable a revolute joint motor
func (jointId JointID) RevoluteJointEnableMotor(enableMotor bool) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_EnableMotor not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointIsMotorEnabled is b3RevoluteJoint_IsMotorEnabled.
// Is the revolute joint motor enabled?
func (jointId JointID) RevoluteJointIsMotorEnabled() bool {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_IsMotorEnabled not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointSetMotorSpeed is b3RevoluteJoint_SetMotorSpeed.
// Set the revolute joint motor speed in radians per second
func (jointId JointID) RevoluteJointSetMotorSpeed(motorSpeed float32) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_SetMotorSpeed not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointGetMotorSpeed is b3RevoluteJoint_GetMotorSpeed.
// Get the revolute joint motor speed in radians per second
func (jointId JointID) RevoluteJointGetMotorSpeed() float32 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_GetMotorSpeed not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointSetMaxMotorTorque is b3RevoluteJoint_SetMaxMotorTorque.
// Set the revolute joint maximum motor torque, usually in newton-meters
func (jointId JointID) RevoluteJointSetMaxMotorTorque(torque float32) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_SetMaxMotorTorque not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointGetMaxMotorTorque is b3RevoluteJoint_GetMaxMotorTorque.
// Get the revolute joint maximum motor torque, usually in newton-meters
func (jointId JointID) RevoluteJointGetMaxMotorTorque() float32 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_GetMaxMotorTorque not ported yet (revolute_joint.c, milestone 4)")
}

// RevoluteJointGetMotorTorque is b3RevoluteJoint_GetMotorTorque.
// Get the revolute joint current motor torque, usually in newton-meters
func (jointId JointID) RevoluteJointGetMotorTorque() float32 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3RevoluteJoint_GetMotorTorque not ported yet (revolute_joint.c, milestone 4)")
}

// getRevoluteJointForce is b3GetRevoluteJointForce.
func getRevoluteJointForce(world *world, base *jointSim) Vec3 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3GetRevoluteJointForce not ported yet (revolute_joint.c, milestone 4)")
}

// getRevoluteJointTorque is b3GetRevoluteJointTorque.
func getRevoluteJointTorque(world *world, base *jointSim) Vec3 {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3GetRevoluteJointTorque not ported yet (revolute_joint.c, milestone 4)")
}

// prepareRevoluteJoint is b3PrepareRevoluteJoint.
func prepareRevoluteJoint(base *jointSim, context *stepContext) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3PrepareRevoluteJoint not ported yet (revolute_joint.c, milestone 4)")
}

// warmStartRevoluteJoint is b3WarmStartRevoluteJoint.
func warmStartRevoluteJoint(base *jointSim, context *stepContext) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3WarmStartRevoluteJoint not ported yet (revolute_joint.c, milestone 4)")
}

// solveRevoluteJoint is b3SolveRevoluteJoint.
func solveRevoluteJoint(base *jointSim, context *stepContext, useBias bool) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3SolveRevoluteJoint not ported yet (revolute_joint.c, milestone 4)")
}

// drawRevoluteJoint is b3DrawRevoluteJoint.
func drawRevoluteJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	// PORT: stub until revolute_joint.c lands
	panic("box3d: b3DrawRevoluteJoint not ported yet (revolute_joint.c, milestone 4)")
}
