// Port of src/weld_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: stubs until weld_joint.c lands (milestone 4). Every function joint.c calls and every
// b3WeldJoint_* function of box3d.h is declared here with its final signature, in upstream order,
// and panics. The b3WeldJoint_* functions are methods on JointID named WeldJoint<Method>.

// WeldJointSetLinearHertz is b3WeldJoint_SetLinearHertz.
// Set the weld joint linear stiffness in Hertz. 0 is rigid.
func (jointId JointID) WeldJointSetLinearHertz(hertz float32) {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3WeldJoint_SetLinearHertz not ported yet (weld_joint.c, milestone 4)")
}

// WeldJointGetLinearHertz is b3WeldJoint_GetLinearHertz.
// Get the weld joint linear stiffness in Hertz
func (jointId JointID) WeldJointGetLinearHertz() float32 {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3WeldJoint_GetLinearHertz not ported yet (weld_joint.c, milestone 4)")
}

// WeldJointSetLinearDampingRatio is b3WeldJoint_SetLinearDampingRatio.
// Set the weld joint linear damping ratio (non-dimensional)
func (jointId JointID) WeldJointSetLinearDampingRatio(dampingRatio float32) {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3WeldJoint_SetLinearDampingRatio not ported yet (weld_joint.c, milestone 4)")
}

// WeldJointGetLinearDampingRatio is b3WeldJoint_GetLinearDampingRatio.
// Get the weld joint linear damping ratio (non-dimensional)
func (jointId JointID) WeldJointGetLinearDampingRatio() float32 {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3WeldJoint_GetLinearDampingRatio not ported yet (weld_joint.c, milestone 4)")
}

// WeldJointSetAngularHertz is b3WeldJoint_SetAngularHertz.
// Set the weld joint angular stiffness in Hertz. 0 is rigid.
func (jointId JointID) WeldJointSetAngularHertz(hertz float32) {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3WeldJoint_SetAngularHertz not ported yet (weld_joint.c, milestone 4)")
}

// WeldJointGetAngularHertz is b3WeldJoint_GetAngularHertz.
// Get the weld joint angular stiffness in Hertz
func (jointId JointID) WeldJointGetAngularHertz() float32 {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3WeldJoint_GetAngularHertz not ported yet (weld_joint.c, milestone 4)")
}

// WeldJointSetAngularDampingRatio is b3WeldJoint_SetAngularDampingRatio.
// Set weld joint angular damping ratio, non-dimensional
func (jointId JointID) WeldJointSetAngularDampingRatio(dampingRatio float32) {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3WeldJoint_SetAngularDampingRatio not ported yet (weld_joint.c, milestone 4)")
}

// WeldJointGetAngularDampingRatio is b3WeldJoint_GetAngularDampingRatio.
// Get the weld joint angular damping ratio, non-dimensional
func (jointId JointID) WeldJointGetAngularDampingRatio() float32 {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3WeldJoint_GetAngularDampingRatio not ported yet (weld_joint.c, milestone 4)")
}

// getWeldJointForce is b3GetWeldJointForce.
func getWeldJointForce(world *world, base *jointSim) Vec3 {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3GetWeldJointForce not ported yet (weld_joint.c, milestone 4)")
}

// getWeldJointTorque is b3GetWeldJointTorque.
func getWeldJointTorque(world *world, base *jointSim) Vec3 {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3GetWeldJointTorque not ported yet (weld_joint.c, milestone 4)")
}

// prepareWeldJoint is b3PrepareWeldJoint.
func prepareWeldJoint(base *jointSim, context *stepContext) {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3PrepareWeldJoint not ported yet (weld_joint.c, milestone 4)")
}

// warmStartWeldJoint is b3WarmStartWeldJoint.
func warmStartWeldJoint(base *jointSim, context *stepContext) {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3WarmStartWeldJoint not ported yet (weld_joint.c, milestone 4)")
}

// solveWeldJoint is b3SolveWeldJoint.
func solveWeldJoint(base *jointSim, context *stepContext, useBias bool) {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3SolveWeldJoint not ported yet (weld_joint.c, milestone 4)")
}

// drawWeldJoint is b3DrawWeldJoint.
func drawWeldJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	// PORT: stub until weld_joint.c lands
	panic("box3d: b3DrawWeldJoint not ported yet (weld_joint.c, milestone 4)")
}
