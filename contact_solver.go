// Port of src/contact_solver.h (and later src/contact_solver.c) from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- contact_solver.h ----

// PORT: header types only; contact_solver.c lands in milestone 3. b3GraphColor (and so b3World)
// reaches these structs.

// manifoldConstraintPoint is b3ManifoldConstraintPoint.
type manifoldConstraintPoint struct {
	rA, rB             Vec3
	baseSeparation     float32
	relativeVelocity   float32
	normalImpulse      float32
	totalNormalImpulse float32
	restitutionImpulse float32
	normalMass         float32
	leverArm           float32
}

// manifoldConstraint is b3ManifoldConstraint.
type manifoldConstraint struct {
	points     [MaxManifoldPoints]manifoldConstraintPoint
	pointCount int32
	normal     Vec3
	tangent1   Vec3
	tangent2   Vec3
	// Friction centers
	centerA, centerB Vec3
	twistMass        float32
	twistImpulse     float32
	tangentMass      matrix2
	frictionImpulse  Vec2
	rollingImpulse   Vec3
	tangentVelocity1 float32
	tangentVelocity2 float32
}

// contactConstraint is b3ContactConstraint.
// PORT: b3ManifoldConstraint* constraints is a slice into the color's manifold constraints.
type contactConstraint struct {
	constraints        []manifoldConstraint
	contact            *contact
	indexA             int32
	indexB             int32
	invMassA, invMassB float32
	invIA, invIB       Matrix3
	softness           softness
	rollingMass        Matrix3
	friction           float32
	restitution        float32
	rollingResistance  float32
	manifoldCount      int32
}

// ---- contact_solver.c ----

// PORT: stubs until contact_solver.c lands in milestone 3. Other milestone 3 files (solver.c) call
// these; the owner of contact_solver.c replaces each stub with the real function, keeping the
// signature, in upstream order.

// getWideContactConstraintByteCount is b3GetWideContactConstraintByteCount.
func getWideContactConstraintByteCount(simdWidth int) int {
	panic("box3d: b3GetWideContactConstraintByteCount not ported yet (contact_solver.c, milestone 3)")
}

// prepareContactsOverflow is b3PrepareContacts_Overflow.
func prepareContactsOverflow(context *stepContext) {
	panic("box3d: b3PrepareContacts_Overflow not ported yet (contact_solver.c, milestone 3)")
}

// warmStartContactsOverflow is b3WarmStartContacts_Overflow.
func warmStartContactsOverflow(context *stepContext) {
	panic("box3d: b3WarmStartContacts_Overflow not ported yet (contact_solver.c, milestone 3)")
}

// pushContactsOverflow is b3PushContacts_Overflow.
func pushContactsOverflow(context *stepContext) {
	panic("box3d: b3PushContacts_Overflow not ported yet (contact_solver.c, milestone 3)")
}

// solveContactsOverflow is b3SolveContacts_Overflow.
func solveContactsOverflow(context *stepContext) {
	panic("box3d: b3SolveContacts_Overflow not ported yet (contact_solver.c, milestone 3)")
}

// applyRestitutionOverflow is b3ApplyRestitution_Overflow.
func applyRestitutionOverflow(context *stepContext) {
	panic("box3d: b3ApplyRestitution_Overflow not ported yet (contact_solver.c, milestone 3)")
}

// storeImpulsesOverflow is b3StoreImpulses_Overflow.
func storeImpulsesOverflow(context *stepContext) {
	panic("box3d: b3StoreImpulses_Overflow not ported yet (contact_solver.c, milestone 3)")
}

// prepareContactsConvex is b3PrepareContacts_Convex.
func prepareContactsConvex(block solverBlock, context *stepContext) {
	panic("box3d: b3PrepareContacts_Convex not ported yet (contact_solver.c, milestone 3)")
}

// warmStartContactsConvex is b3WarmStartContacts_Convex.
func warmStartContactsConvex(block solverBlock, context *stepContext) {
	panic("box3d: b3WarmStartContacts_Convex not ported yet (contact_solver.c, milestone 3)")
}

// pushContactsConvex is b3PushContacts_Convex.
func pushContactsConvex(block solverBlock, context *stepContext) {
	panic("box3d: b3PushContacts_Convex not ported yet (contact_solver.c, milestone 3)")
}

// solveContactsConvex is b3SolveContacts_Convex.
func solveContactsConvex(block solverBlock, context *stepContext) {
	panic("box3d: b3SolveContacts_Convex not ported yet (contact_solver.c, milestone 3)")
}

// applyRestitutionConvex is b3ApplyRestitution_Convex.
func applyRestitutionConvex(block solverBlock, context *stepContext) {
	panic("box3d: b3ApplyRestitution_Convex not ported yet (contact_solver.c, milestone 3)")
}

// storeImpulsesConvex is b3StoreImpulses_Convex.
func storeImpulsesConvex(block solverBlock, context *stepContext, workerIndex int) {
	panic("box3d: b3StoreImpulses_Convex not ported yet (contact_solver.c, milestone 3)")
}

// prepareContactsMeshWide is b3PrepareContacts_MeshWide.
func prepareContactsMeshWide(block solverBlock, context *stepContext) {
	panic("box3d: b3PrepareContacts_MeshWide not ported yet (contact_solver.c, milestone 3)")
}

// warmStartContactsMeshWide is b3WarmStartContacts_MeshWide.
func warmStartContactsMeshWide(block solverBlock, context *stepContext) {
	panic("box3d: b3WarmStartContacts_MeshWide not ported yet (contact_solver.c, milestone 3)")
}

// pushContactsMeshWide is b3PushContacts_MeshWide.
func pushContactsMeshWide(block solverBlock, context *stepContext) {
	panic("box3d: b3PushContacts_MeshWide not ported yet (contact_solver.c, milestone 3)")
}

// solveContactsMeshWide is b3SolveContacts_MeshWide.
func solveContactsMeshWide(block solverBlock, context *stepContext) {
	panic("box3d: b3SolveContacts_MeshWide not ported yet (contact_solver.c, milestone 3)")
}

// applyRestitutionMeshWide is b3ApplyRestitution_MeshWide.
func applyRestitutionMeshWide(block solverBlock, context *stepContext) {
	panic("box3d: b3ApplyRestitution_MeshWide not ported yet (contact_solver.c, milestone 3)")
}

// storeImpulsesMeshWide is b3StoreImpulses_MeshWide.
func storeImpulsesMeshWide(block solverBlock, context *stepContext, workerIndex int) {
	panic("box3d: b3StoreImpulses_MeshWide not ported yet (contact_solver.c, milestone 3)")
}

// getWideMeshConstraintByteCount is b3GetWideMeshConstraintByteCount.
func getWideMeshConstraintByteCount(simdWidth int) int {
	panic("box3d: b3GetWideMeshConstraintByteCount not ported yet (contact_solver.c, milestone 3)")
}

// getWideMeshManifoldByteCount is b3GetWideMeshManifoldByteCount.
func getWideMeshManifoldByteCount(simdWidth int) int {
	panic("box3d: b3GetWideMeshManifoldByteCount not ported yet (contact_solver.c, milestone 3)")
}
