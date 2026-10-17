// Port of src/prismatic_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: stubs until prismatic_joint.c lands (milestone 4). Every function joint.c calls and every
// b3PrismaticJoint_* function of box3d.h is declared here with its final signature, in upstream order,
// and panics. The b3PrismaticJoint_* functions are methods on JointID named PrismaticJoint<Method>.

// PrismaticJointEnableLimit is b3PrismaticJoint_EnableLimit.
// Enable/disable a prismatic joint limit
func (jointId JointID) PrismaticJointEnableLimit(enableLimit bool) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_EnableLimit not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointIsLimitEnabled is b3PrismaticJoint_IsLimitEnabled.
// Is the prismatic joint limit enabled?
func (jointId JointID) PrismaticJointIsLimitEnabled() bool {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_IsLimitEnabled not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetLowerLimit is b3PrismaticJoint_GetLowerLimit.
// Get the prismatic joint lower limit
func (jointId JointID) PrismaticJointGetLowerLimit() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetLowerLimit not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetUpperLimit is b3PrismaticJoint_GetUpperLimit.
// Get the prismatic joint upper limit
func (jointId JointID) PrismaticJointGetUpperLimit() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetUpperLimit not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointSetLimits is b3PrismaticJoint_SetLimits.
// Set the prismatic joint limits
func (jointId JointID) PrismaticJointSetLimits(lower, upper float32) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_SetLimits not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetTranslation is b3PrismaticJoint_GetTranslation.
// Get the current joint translation, usually in meters.
func (jointId JointID) PrismaticJointGetTranslation() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetTranslation not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointEnableSpring is b3PrismaticJoint_EnableSpring.
// Enable/disable the joint spring.
func (jointId JointID) PrismaticJointEnableSpring(enableSpring bool) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_EnableSpring not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointIsSpringEnabled is b3PrismaticJoint_IsSpringEnabled.
// Is the prismatic joint spring enabled or not?
func (jointId JointID) PrismaticJointIsSpringEnabled() bool {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_IsSpringEnabled not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointSetTargetTranslation is b3PrismaticJoint_SetTargetTranslation.
// Set the prismatic joint target translation. Usually in meters.
func (jointId JointID) PrismaticJointSetTargetTranslation(targetTranslation float32) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_SetTargetTranslation not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetTargetTranslation is b3PrismaticJoint_GetTargetTranslation.
// Get the prismatic joint target translation. Usually in meters.
func (jointId JointID) PrismaticJointGetTargetTranslation() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetTargetTranslation not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointSetSpringHertz is b3PrismaticJoint_SetSpringHertz.
// Set the prismatic joint stiffness in Hertz.
// This should usually be less than a quarter of the simulation rate. For example, if the simulation
// runs at 60Hz then the joint stiffness should be 15Hz or less.
func (jointId JointID) PrismaticJointSetSpringHertz(hertz float32) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_SetSpringHertz not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetSpringHertz is b3PrismaticJoint_GetSpringHertz.
// Get the prismatic joint stiffness in Hertz
func (jointId JointID) PrismaticJointGetSpringHertz() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetSpringHertz not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointSetSpringDampingRatio is b3PrismaticJoint_SetSpringDampingRatio.
// Set the prismatic joint damping ratio (non-dimensional)
func (jointId JointID) PrismaticJointSetSpringDampingRatio(dampingRatio float32) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_SetSpringDampingRatio not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetSpringDampingRatio is b3PrismaticJoint_GetSpringDampingRatio.
// Get the prismatic spring damping ratio (non-dimensional)
func (jointId JointID) PrismaticJointGetSpringDampingRatio() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetSpringDampingRatio not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointEnableMotor is b3PrismaticJoint_EnableMotor.
// Enable/disable a prismatic joint motor
func (jointId JointID) PrismaticJointEnableMotor(enableMotor bool) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_EnableMotor not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointIsMotorEnabled is b3PrismaticJoint_IsMotorEnabled.
// Is the prismatic joint motor enabled?
func (jointId JointID) PrismaticJointIsMotorEnabled() bool {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_IsMotorEnabled not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointSetMotorSpeed is b3PrismaticJoint_SetMotorSpeed.
// Set the prismatic joint motor speed, usually in meters per second
func (jointId JointID) PrismaticJointSetMotorSpeed(motorSpeed float32) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_SetMotorSpeed not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetMotorSpeed is b3PrismaticJoint_GetMotorSpeed.
// Get the prismatic joint motor speed, usually in meters per second
func (jointId JointID) PrismaticJointGetMotorSpeed() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetMotorSpeed not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointSetMaxMotorForce is b3PrismaticJoint_SetMaxMotorForce.
// Set the prismatic joint maximum motor force, usually in newtons
func (jointId JointID) PrismaticJointSetMaxMotorForce(force float32) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_SetMaxMotorForce not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetMaxMotorForce is b3PrismaticJoint_GetMaxMotorForce.
// Get the prismatic joint maximum motor force, usually in newtons
func (jointId JointID) PrismaticJointGetMaxMotorForce() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetMaxMotorForce not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetMotorForce is b3PrismaticJoint_GetMotorForce.
// Get the prismatic joint current motor force, usually in newtons
func (jointId JointID) PrismaticJointGetMotorForce() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetMotorForce not ported yet (prismatic_joint.c, milestone 4)")
}

// PrismaticJointGetSpeed is b3PrismaticJoint_GetSpeed.
// Get the current joint translation speed, usually in meters per second.
func (jointId JointID) PrismaticJointGetSpeed() float32 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PrismaticJoint_GetSpeed not ported yet (prismatic_joint.c, milestone 4)")
}

// getPrismaticJointForce is b3GetPrismaticJointForce.
func getPrismaticJointForce(world *world, base *jointSim) Vec3 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3GetPrismaticJointForce not ported yet (prismatic_joint.c, milestone 4)")
}

// getPrismaticJointTorque is b3GetPrismaticJointTorque.
func getPrismaticJointTorque(world *world, base *jointSim) Vec3 {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3GetPrismaticJointTorque not ported yet (prismatic_joint.c, milestone 4)")
}

// preparePrismaticJoint is b3PreparePrismaticJoint.
func preparePrismaticJoint(base *jointSim, context *stepContext) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3PreparePrismaticJoint not ported yet (prismatic_joint.c, milestone 4)")
}

// warmStartPrismaticJoint is b3WarmStartPrismaticJoint.
func warmStartPrismaticJoint(base *jointSim, context *stepContext) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3WarmStartPrismaticJoint not ported yet (prismatic_joint.c, milestone 4)")
}

// solvePrismaticJoint is b3SolvePrismaticJoint.
func solvePrismaticJoint(base *jointSim, context *stepContext, useBias bool) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3SolvePrismaticJoint not ported yet (prismatic_joint.c, milestone 4)")
}

// drawPrismaticJoint is b3DrawPrismaticJoint.
func drawPrismaticJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	// PORT: stub until prismatic_joint.c lands
	panic("box3d: b3DrawPrismaticJoint not ported yet (prismatic_joint.c, milestone 4)")
}
