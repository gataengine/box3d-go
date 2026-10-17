// Port of src/distance_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: stubs until distance_joint.c lands (milestone 4). Every function joint.c calls and every
// b3DistanceJoint_* function of box3d.h is declared here with its final signature, in upstream order,
// and panics. The b3DistanceJoint_* functions are methods on JointID named DistanceJoint<Method>.

// DistanceJointSetLength is b3DistanceJoint_SetLength.
// Set the rest length of a distance joint
func (jointId JointID) DistanceJointSetLength(length float32) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_SetLength not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetLength is b3DistanceJoint_GetLength.
// Get the rest length of a distance joint
func (jointId JointID) DistanceJointGetLength() float32 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetLength not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointEnableLimit is b3DistanceJoint_EnableLimit.
// Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint is rigid
// and the limit has no effect.
func (jointId JointID) DistanceJointEnableLimit(enableLimit bool) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_EnableLimit not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointIsLimitEnabled is b3DistanceJoint_IsLimitEnabled.
// Is the distance joint limit enabled?
func (jointId JointID) DistanceJointIsLimitEnabled() bool {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_IsLimitEnabled not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointSetLengthRange is b3DistanceJoint_SetLengthRange.
// Set the minimum and maximum length parameters of a distance joint
func (jointId JointID) DistanceJointSetLengthRange(minLength, maxLength float32) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_SetLengthRange not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetMinLength is b3DistanceJoint_GetMinLength.
// Get the distance joint minimum length
func (jointId JointID) DistanceJointGetMinLength() float32 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetMinLength not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetMaxLength is b3DistanceJoint_GetMaxLength.
// Get the distance joint maximum length
func (jointId JointID) DistanceJointGetMaxLength() float32 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetMaxLength not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetCurrentLength is b3DistanceJoint_GetCurrentLength.
// Get the current length of a distance joint
func (jointId JointID) DistanceJointGetCurrentLength() float32 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetCurrentLength not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointEnableSpring is b3DistanceJoint_EnableSpring.
// Enable/disable the distance joint spring. When disabled the distance joint is rigid.
func (jointId JointID) DistanceJointEnableSpring(enableSpring bool) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_EnableSpring not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointIsSpringEnabled is b3DistanceJoint_IsSpringEnabled.
// Is the distance joint spring enabled?
func (jointId JointID) DistanceJointIsSpringEnabled() bool {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_IsSpringEnabled not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointSetSpringForceRange is b3DistanceJoint_SetSpringForceRange.
// Set the force range for the spring.
func (jointId JointID) DistanceJointSetSpringForceRange(lowerForce, upperForce float32) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_SetSpringForceRange not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetSpringForceRange is b3DistanceJoint_GetSpringForceRange.
// Get the force range for the spring.
// PORT: the lowerForce and upperForce out-parameters are the return values.
func (jointId JointID) DistanceJointGetSpringForceRange() (lowerForce, upperForce float32) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetSpringForceRange not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointSetSpringHertz is b3DistanceJoint_SetSpringHertz.
// Set the spring stiffness in Hertz
func (jointId JointID) DistanceJointSetSpringHertz(hertz float32) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_SetSpringHertz not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointSetSpringDampingRatio is b3DistanceJoint_SetSpringDampingRatio.
// Set the spring damping ratio, non-dimensional
func (jointId JointID) DistanceJointSetSpringDampingRatio(dampingRatio float32) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_SetSpringDampingRatio not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetSpringHertz is b3DistanceJoint_GetSpringHertz.
// Get the spring Hertz
func (jointId JointID) DistanceJointGetSpringHertz() float32 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetSpringHertz not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetSpringDampingRatio is b3DistanceJoint_GetSpringDampingRatio.
// Get the spring damping ratio
func (jointId JointID) DistanceJointGetSpringDampingRatio() float32 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetSpringDampingRatio not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointEnableMotor is b3DistanceJoint_EnableMotor.
// Enable/disable the distance joint motor
func (jointId JointID) DistanceJointEnableMotor(enableMotor bool) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_EnableMotor not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointIsMotorEnabled is b3DistanceJoint_IsMotorEnabled.
// Is the distance joint motor enabled?
func (jointId JointID) DistanceJointIsMotorEnabled() bool {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_IsMotorEnabled not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointSetMotorSpeed is b3DistanceJoint_SetMotorSpeed.
// Set the distance joint motor speed, usually in meters per second
func (jointId JointID) DistanceJointSetMotorSpeed(motorSpeed float32) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_SetMotorSpeed not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetMotorSpeed is b3DistanceJoint_GetMotorSpeed.
// Get the distance joint motor speed, usually in meters per second
func (jointId JointID) DistanceJointGetMotorSpeed() float32 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetMotorSpeed not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetMotorForce is b3DistanceJoint_GetMotorForce.
// Get the distance joint current motor force, usually in newtons
func (jointId JointID) DistanceJointGetMotorForce() float32 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetMotorForce not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointSetMaxMotorForce is b3DistanceJoint_SetMaxMotorForce.
// Set the distance joint maximum motor force, usually in newtons
func (jointId JointID) DistanceJointSetMaxMotorForce(force float32) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_SetMaxMotorForce not ported yet (distance_joint.c, milestone 4)")
}

// DistanceJointGetMaxMotorForce is b3DistanceJoint_GetMaxMotorForce.
// Get the distance joint maximum motor force, usually in newtons
func (jointId JointID) DistanceJointGetMaxMotorForce() float32 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DistanceJoint_GetMaxMotorForce not ported yet (distance_joint.c, milestone 4)")
}

// getDistanceJointForce is b3GetDistanceJointForce.
func getDistanceJointForce(world *world, base *jointSim) Vec3 {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3GetDistanceJointForce not ported yet (distance_joint.c, milestone 4)")
}

// prepareDistanceJoint is b3PrepareDistanceJoint.
func prepareDistanceJoint(base *jointSim, context *stepContext) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3PrepareDistanceJoint not ported yet (distance_joint.c, milestone 4)")
}

// warmStartDistanceJoint is b3WarmStartDistanceJoint.
func warmStartDistanceJoint(base *jointSim, context *stepContext) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3WarmStartDistanceJoint not ported yet (distance_joint.c, milestone 4)")
}

// solveDistanceJoint is b3SolveDistanceJoint.
func solveDistanceJoint(base *jointSim, context *stepContext, useBias bool) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3SolveDistanceJoint not ported yet (distance_joint.c, milestone 4)")
}

// drawDistanceJoint is b3DrawDistanceJoint.
func drawDistanceJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform) {
	// PORT: stub until distance_joint.c lands
	panic("box3d: b3DrawDistanceJoint not ported yet (distance_joint.c, milestone 4)")
}
