// Port of src/motor_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: stubs until motor_joint.c lands (milestone 4). Every function joint.c calls and every
// b3MotorJoint_* function of box3d.h is declared here with its final signature, in upstream order,
// and panics. The b3MotorJoint_* functions are methods on JointID named MotorJoint<Method>.

// MotorJointSetLinearVelocity is b3MotorJoint_SetLinearVelocity.
// Set the desired relative linear velocity in meters per second
func (jointId JointID) MotorJointSetLinearVelocity(velocity Vec3) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetLinearVelocity not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetLinearVelocity is b3MotorJoint_GetLinearVelocity.
// Get the desired relative linear velocity in meters per second
func (jointId JointID) MotorJointGetLinearVelocity() Vec3 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetLinearVelocity not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointSetAngularVelocity is b3MotorJoint_SetAngularVelocity.
// Set the desired relative angular velocity in radians per second
func (jointId JointID) MotorJointSetAngularVelocity(velocity Vec3) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetAngularVelocity not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetAngularVelocity is b3MotorJoint_GetAngularVelocity.
// Get the desired relative angular velocity in radians per second
func (jointId JointID) MotorJointGetAngularVelocity() Vec3 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetAngularVelocity not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointSetMaxVelocityTorque is b3MotorJoint_SetMaxVelocityTorque.
// Set the motor joint maximum torque, usually in newton-meters
func (jointId JointID) MotorJointSetMaxVelocityTorque(maxTorque float32) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetMaxVelocityTorque not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetMaxVelocityTorque is b3MotorJoint_GetMaxVelocityTorque.
// Get the motor joint maximum torque, usually in newton-meters
func (jointId JointID) MotorJointGetMaxVelocityTorque() float32 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetMaxVelocityTorque not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointSetMaxVelocityForce is b3MotorJoint_SetMaxVelocityForce.
// Set the motor joint maximum force, usually in newtons
func (jointId JointID) MotorJointSetMaxVelocityForce(maxForce float32) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetMaxVelocityForce not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetMaxVelocityForce is b3MotorJoint_GetMaxVelocityForce.
// Get the motor joint maximum force, usually in newtons
func (jointId JointID) MotorJointGetMaxVelocityForce() float32 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetMaxVelocityForce not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointSetLinearHertz is b3MotorJoint_SetLinearHertz.
// Set the spring linear hertz stiffness
func (jointId JointID) MotorJointSetLinearHertz(hertz float32) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetLinearHertz not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetLinearHertz is b3MotorJoint_GetLinearHertz.
// Get the spring linear hertz stiffness
func (jointId JointID) MotorJointGetLinearHertz() float32 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetLinearHertz not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointSetLinearDampingRatio is b3MotorJoint_SetLinearDampingRatio.
// Set the spring linear damping ratio. Use 1.0 for critical damping.
func (jointId JointID) MotorJointSetLinearDampingRatio(damping float32) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetLinearDampingRatio not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetLinearDampingRatio is b3MotorJoint_GetLinearDampingRatio.
// Get the spring linear damping ratio.
func (jointId JointID) MotorJointGetLinearDampingRatio() float32 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetLinearDampingRatio not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointSetAngularHertz is b3MotorJoint_SetAngularHertz.
// Set the spring angular hertz stiffness
func (jointId JointID) MotorJointSetAngularHertz(hertz float32) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetAngularHertz not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetAngularHertz is b3MotorJoint_GetAngularHertz.
// Get the spring angular hertz stiffness
func (jointId JointID) MotorJointGetAngularHertz() float32 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetAngularHertz not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointSetAngularDampingRatio is b3MotorJoint_SetAngularDampingRatio.
// Set the spring angular damping ratio. Use 1.0 for critical damping.
func (jointId JointID) MotorJointSetAngularDampingRatio(damping float32) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetAngularDampingRatio not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetAngularDampingRatio is b3MotorJoint_GetAngularDampingRatio.
// Get the spring angular damping ratio.
func (jointId JointID) MotorJointGetAngularDampingRatio() float32 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetAngularDampingRatio not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointSetMaxSpringForce is b3MotorJoint_SetMaxSpringForce.
// Set the maximum spring force in newtons.
func (jointId JointID) MotorJointSetMaxSpringForce(maxForce float32) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetMaxSpringForce not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetMaxSpringForce is b3MotorJoint_GetMaxSpringForce.
// Get the maximum spring force in newtons.
func (jointId JointID) MotorJointGetMaxSpringForce() float32 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetMaxSpringForce not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointSetMaxSpringTorque is b3MotorJoint_SetMaxSpringTorque.
// Set the maximum spring torque in newtons * meters
func (jointId JointID) MotorJointSetMaxSpringTorque(maxTorque float32) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_SetMaxSpringTorque not ported yet (motor_joint.c, milestone 4)")
}

// MotorJointGetMaxSpringTorque is b3MotorJoint_GetMaxSpringTorque.
// Get the maximum spring torque in newtons * meters
func (jointId JointID) MotorJointGetMaxSpringTorque() float32 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3MotorJoint_GetMaxSpringTorque not ported yet (motor_joint.c, milestone 4)")
}

// getMotorJointForce is b3GetMotorJointForce.
func getMotorJointForce(world *world, base *jointSim) Vec3 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3GetMotorJointForce not ported yet (motor_joint.c, milestone 4)")
}

// getMotorJointTorque is b3GetMotorJointTorque.
func getMotorJointTorque(world *world, base *jointSim) Vec3 {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3GetMotorJointTorque not ported yet (motor_joint.c, milestone 4)")
}

// prepareMotorJoint is b3PrepareMotorJoint.
func prepareMotorJoint(base *jointSim, context *stepContext) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3PrepareMotorJoint not ported yet (motor_joint.c, milestone 4)")
}

// warmStartMotorJoint is b3WarmStartMotorJoint.
func warmStartMotorJoint(base *jointSim, context *stepContext) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3WarmStartMotorJoint not ported yet (motor_joint.c, milestone 4)")
}

// solveMotorJoint is b3SolveMotorJoint.
func solveMotorJoint(base *jointSim, context *stepContext) {
	// PORT: stub until motor_joint.c lands
	panic("box3d: b3SolveMotorJoint not ported yet (motor_joint.c, milestone 4)")
}
