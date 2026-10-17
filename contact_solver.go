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
