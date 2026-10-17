// Port of src/parallel_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: stubs until parallel_joint.c lands (milestone 4). Every function joint.c calls and every
// b3ParallelJoint_* function of box3d.h is declared here with its final signature, in upstream order,
// and panics. The b3ParallelJoint_* functions are methods on JointID named ParallelJoint<Method>.

// ParallelJointSetSpringHertz is b3ParallelJoint_SetSpringHertz.
// Set the spring stiffness in Hertz
func (jointId JointID) ParallelJointSetSpringHertz(hertz float32) {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3ParallelJoint_SetSpringHertz not ported yet (parallel_joint.c, milestone 4)")
}

// ParallelJointGetSpringHertz is b3ParallelJoint_GetSpringHertz.
// Get the spring Hertz
func (jointId JointID) ParallelJointGetSpringHertz() float32 {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3ParallelJoint_GetSpringHertz not ported yet (parallel_joint.c, milestone 4)")
}

// ParallelJointSetSpringDampingRatio is b3ParallelJoint_SetSpringDampingRatio.
// Set the spring damping ratio, non-dimensional
func (jointId JointID) ParallelJointSetSpringDampingRatio(dampingRatio float32) {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3ParallelJoint_SetSpringDampingRatio not ported yet (parallel_joint.c, milestone 4)")
}

// ParallelJointGetSpringDampingRatio is b3ParallelJoint_GetSpringDampingRatio.
// Get the spring damping ratio
func (jointId JointID) ParallelJointGetSpringDampingRatio() float32 {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3ParallelJoint_GetSpringDampingRatio not ported yet (parallel_joint.c, milestone 4)")
}

// ParallelJointSetMaxTorque is b3ParallelJoint_SetMaxTorque.
// Set the maximum spring torque, usually in newton-meters
func (jointId JointID) ParallelJointSetMaxTorque(force float32) {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3ParallelJoint_SetMaxTorque not ported yet (parallel_joint.c, milestone 4)")
}

// ParallelJointGetMaxTorque is b3ParallelJoint_GetMaxTorque.
// Get the maximum spring torque, usually in newton-meters
func (jointId JointID) ParallelJointGetMaxTorque() float32 {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3ParallelJoint_GetMaxTorque not ported yet (parallel_joint.c, milestone 4)")
}

// getParallelJointTorque is b3GetParallelJointTorque.
func getParallelJointTorque(world *world, base *jointSim) Vec3 {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3GetParallelJointTorque not ported yet (parallel_joint.c, milestone 4)")
}

// prepareParallelJoint is b3PrepareParallelJoint.
func prepareParallelJoint(base *jointSim, context *stepContext) {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3PrepareParallelJoint not ported yet (parallel_joint.c, milestone 4)")
}

// warmStartParallelJoint is b3WarmStartParallelJoint.
func warmStartParallelJoint(base *jointSim, context *stepContext) {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3WarmStartParallelJoint not ported yet (parallel_joint.c, milestone 4)")
}

// solveParallelJoint is b3SolveParallelJoint.
func solveParallelJoint(base *jointSim, context *stepContext) {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3SolveParallelJoint not ported yet (parallel_joint.c, milestone 4)")
}

// drawParallelJoint is b3DrawParallelJoint.
func drawParallelJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	// PORT: stub until parallel_joint.c lands
	panic("box3d: b3DrawParallelJoint not ported yet (parallel_joint.c, milestone 4)")
}
