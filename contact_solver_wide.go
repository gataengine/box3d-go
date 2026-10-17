// Port of src/contact_solver_wide.inl from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: so far only the wide types (the struct definitions at the top of the .inl, which
// b3StepContext and b3GraphColor reach through their void* buffers) and the typed form of those
// buffers. The kernels land with contact_solver_wide.inl in milestone 3. Upstream compiles this
// file once per width (W4, W8) through B3_WIDE; the Go port has one lane width (laneCount) and
// drops the width suffix (see simd_wide.go).

// vec2W is b3Vec2W: wide vec2
type vec2W struct {
	x, y floatW
}

// quatW is b3QuatW: wide quaternion
type quatW struct {
	V vec3W
	S floatW
}

// symMatrix2W is b3SymMatrix2W: wide symmetric matrix2
type symMatrix2W struct {
	cxx, cxy, cyy floatW
}

// symMatrix3W is b3SymMatrix3W: wide symmetric matrix3
type symMatrix3W struct {
	cxx, cxy, cxz, cyy, cyz, czz floatW
}

// matrix3W is b3Matrix3W.
type matrix3W struct {
	cx, cy, cz vec3W
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

// contactConstraintPointWide is b3ContactConstraintPointWide.
type contactConstraintPointWide struct {
	anchorAs, anchorBs  vec3W
	baseSeparations     floatW
	normalImpulses      floatW
	totalNormalImpulses floatW
	normalMasses        floatW
	leverArms           floatW
	relativeVelocities  floatW
	restitutionImpulses floatW
}

// contactManifoldWide is b3ContactManifoldWide.
// PORT: _Alignas( B3_WIDE_ALIGNMENT ) dropped; Go aligns floatW to 4 bytes, which the scalar
// lanes need.
type contactManifoldWide struct {
	normal vec3W

	tangent1 vec3W
	tangent2 vec3W

	// Friction centers
	centerA, centerB vec3W
	twistMass        floatW
	twistImpulse     floatW
	tangentMass      symMatrix2W
	frictionImpulse  vec2W
	rollingImpulse   vec3W
	rollingMask      floatW
	tangentVelocity1 floatW
	tangentVelocity2 floatW

	pointCount int32

	points [MaxManifoldPoints]contactConstraintPointWide
}

// contactConstraintWide is b3ContactConstraintWide.
// Solves one contact per lane
// PORT: _Alignas( B3_WIDE_ALIGNMENT ) dropped (see contactManifoldWide).
type contactConstraintWide struct {
	// These are base 1
	indexA [laneCount]int32
	indexB [laneCount]int32

	contacts [laneCount]*contact

	invMassA, invMassB floatW
	invIA, invIB       symMatrix3W
	rollingMass        symMatrix3W
	friction           floatW
	rollingResistance  floatW
	restitution        floatW

	manifoldCount int32
}

// convexConstraintWide is b3ConvexConstraintWide.
// PORT: kept for b3GetWideContactConstraintByteCount (its size); the convex buffer itself is a
// wideConstraintBuffer whose constraint i owns manifold i, which is the same stream.
type convexConstraintWide struct {
	base     contactConstraintWide
	manifold contactManifoldWide
}

// wideConstraintBuffer is the typed form of upstream's `void*` wide constraint buffers
// (b3StepContext.wideConstraints / wideMeshConstraints, b3GraphColor.wideConstraints /
// wideMeshConstraints).
//
// PORT: upstream stores a heterogeneous byte stream: each b3ContactConstraintWide is followed by
// its b3ContactManifoldWide array (one manifold for convex contacts, manifoldCount for mesh
// contacts), and b3GetConstraintW / b3NextConstraintW / b3GetManifoldsW walk it with byte
// offsets. Go keeps the two element types in two parallel slices instead, so the buffers are typed
// and need no unsafe:
//
//   - constraint index i is constraints[i];
//   - its manifolds start at manifolds[i*fixedManifoldCount] for convex constraints
//     (fixedManifoldCount == 1) and at manifolds[manifoldStarts[i]-manifoldStarts[0]] for mesh
//     constraints, exactly the offsets b3GetConstraintW computes;
//   - a byte pointer into the stream (base + constraintIndex*sizeof(constraint) +
//     manifoldIndex*sizeof(manifold), as solver.c slices the buffer per color) is the pair of
//     sub-slices {constraints[constraintIndex:], manifolds[manifoldIndex:]};
//   - NULL is the zero value.
//
// solver.c allocates the two slices with stackAlloc in place of one byte allocation sized with
// b3GetWide*ByteCount. The kernels and their helpers (b3GetConstraintW and friends) carry the
// constraint and manifold positions as indices into these slices.
type wideConstraintBuffer struct {
	constraints []contactConstraintWide
	manifolds   []contactManifoldWide
}
