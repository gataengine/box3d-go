// Port of src/contact_solver_wide.inl from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"math"
	"unsafe"
)

// PORT: upstream compiles this file once per width (W4, W8) through B3_WIDE. The Go port has one
// lane width (laneCount) and drops the width suffix (see simd_wide.go): b3SolveContacts_ConvexW4 →
// solveContactsConvexW. Only the B3_SIMD_WIDTH == 4 code is ported; where upstream has a
// `#if defined( B3_SIMD_NONE )` variant (b3GatherBodies, b3ScatterBodies) both variants are ported
// and selected by simdNone (simd_scalar.go). The void* constraint stream is wideConstraintBuffer
// (at the end of this file); a b3ContactConstraintWide* into it is the pair of positions
// (constraint index, manifold index), see getConstraintW.

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

// addV2W is b3AddV2W.
// a + b
func addV2W(a, b vec2W) vec2W {
	return vec2W{
		addW(a.x, b.x),
		addW(a.y, b.y),
	}
}

// mulMV2W is b3MulMV2W.
// m * a
func mulMV2W(m symMatrix2W, a vec2W) vec2W {
	b := vec2W{
		addW(mulW(m.cxx, a.x), mulW(m.cxy, a.y)),
		addW(mulW(m.cxy, a.x), mulW(m.cyy, a.y)),
	}

	return b
}

// mulMVW is b3MulMVW.
// m * a
func mulMVW(m symMatrix3W, a vec3W) vec3W {
	b := vec3W{
		addW(mulW(m.cxx, a.X), addW(mulW(m.cxy, a.Y), mulW(m.cxz, a.Z))),
		addW(mulW(m.cxy, a.X), addW(mulW(m.cyy, a.Y), mulW(m.cyz, a.Z))),
		addW(mulW(m.cxz, a.X), addW(mulW(m.cyz, a.Y), mulW(m.czz, a.Z))),
	}

	return b
}

// mulSubMVW is b3MulSubMVW.
// a - m * b
func mulSubMVW(a vec3W, m symMatrix3W, b vec3W) vec3W {
	c := vec3W{
		addW(mulW(m.cxx, b.X), addW(mulW(m.cxy, b.Y), mulW(m.cxz, b.Z))),
		addW(mulW(m.cxy, b.X), addW(mulW(m.cyy, b.Y), mulW(m.cyz, b.Z))),
		addW(mulW(m.cxz, b.X), addW(mulW(m.cyz, b.Y), mulW(m.czz, b.Z))),
	}

	return vec3W{subW(a.X, c.X), subW(a.Y, c.Y), subW(a.Z, c.Z)}
}

// mulAddMVW is b3MulAddMVW.
// a + m * b
func mulAddMVW(a vec3W, m symMatrix3W, b vec3W) vec3W {
	c := vec3W{
		addW(mulW(m.cxx, b.X), addW(mulW(m.cxy, b.Y), mulW(m.cxz, b.Z))),
		addW(mulW(m.cxy, b.X), addW(mulW(m.cyy, b.Y), mulW(m.cyz, b.Z))),
		addW(mulW(m.cxz, b.X), addW(mulW(m.cyz, b.Y), mulW(m.czz, b.Z))),
	}

	return vec3W{addW(a.X, c.X), addW(a.Y, c.Y), addW(a.Z, c.Z)}
}

// mulM3VW is b3MulM3VW.
func mulM3VW(m matrix3W, a vec3W) vec3W {
	b := vec3W{
		addW(mulW(m.cx.X, a.X), addW(mulW(m.cy.X, a.Y), mulW(m.cz.X, a.Z))),
		addW(mulW(m.cx.Y, a.X), addW(mulW(m.cy.Y, a.Y), mulW(m.cz.Y, a.Z))),
		addW(mulW(m.cx.Z, a.X), addW(mulW(m.cy.Z, a.Y), mulW(m.cz.Z, a.Z))),
	}

	return b
}

// invRotateVectorW is b3InvRotateVectorW.
func invRotateVectorW(q quatW, a vec3W) vec3W {
	t := crossW(q.V, a)
	t = vec3W{addW(t.X, t.X), addW(t.Y, t.Y), addW(t.Z, t.Z)}
	u := crossW(q.V, t)
	return vec3W{
		addW(subW(a.X, mulW(q.S, t.X)), u.X),
		addW(subW(a.Y, mulW(q.S, t.Y)), u.Y),
		addW(subW(a.Z, mulW(q.S, t.Z)), u.Z),
	}
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

// PORT: the layout _Static_asserts of the wide types are dropped (see contactManifoldWide).

// getWideContactConstraintByteCountW is b3GetWideContactConstraintByteCountW4.
// PORT: the Go size of convexConstraintWide; the Go buffers are typed slices, so this is
// informational.
func getWideContactConstraintByteCountW() int {
	return int(unsafe.Sizeof(convexConstraintWide{}))
}

// getWideMeshConstraintByteCountW is b3GetWideMeshConstraintByteCountW4.
func getWideMeshConstraintByteCountW() int {
	return int(unsafe.Sizeof(contactConstraintWide{}))
}

// getWideMeshManifoldByteCountW is b3GetWideMeshManifoldByteCountW4.
func getWideMeshManifoldByteCountW() int {
	return int(unsafe.Sizeof(contactManifoldWide{}))
}

// bodyStateW is b3BodyStateW: wide version of b3BodyState
type bodyStateW struct {
	v  vec3W
	w  vec3W
	dp vec3W
	dq quatW
}

// gatherBodies is b3GatherBodies.
// PORT: `#if defined( B3_SIMD_NONE )` becomes a branch on simdNone; the else branch is the
// B3_SIMD_WIDTH == 4 lane variant (b3LoadW of each 16 byte row of b3BodyState, then
// b3TransposeW). The int pointer is a pointer to the lane index array.
func gatherBodies(states []bodyState, indices *[laneCount]int32) bodyStateW {
	if simdNone {
		identity := identityBodyState

		s1 := identity
		if indices[0] != 0 {
			s1 = states[indices[0]-1]
		}
		s2 := identity
		if indices[1] != 0 {
			s2 = states[indices[1]-1]
		}
		s3 := identity
		if indices[2] != 0 {
			s3 = states[indices[2]-1]
		}
		s4 := identity
		if indices[3] != 0 {
			s4 = states[indices[3]-1]
		}

		var simdBody bodyStateW
		simdBody.v.X = floatW{[laneCount]float32{s1.linearVelocity.X, s2.linearVelocity.X, s3.linearVelocity.X, s4.linearVelocity.X}}
		simdBody.v.Y = floatW{[laneCount]float32{s1.linearVelocity.Y, s2.linearVelocity.Y, s3.linearVelocity.Y, s4.linearVelocity.Y}}
		simdBody.v.Z = floatW{[laneCount]float32{s1.linearVelocity.Z, s2.linearVelocity.Z, s3.linearVelocity.Z, s4.linearVelocity.Z}}
		simdBody.w.X = floatW{[laneCount]float32{s1.angularVelocity.X, s2.angularVelocity.X, s3.angularVelocity.X, s4.angularVelocity.X}}
		simdBody.w.Y = floatW{[laneCount]float32{s1.angularVelocity.Y, s2.angularVelocity.Y, s3.angularVelocity.Y, s4.angularVelocity.Y}}
		simdBody.w.Z = floatW{[laneCount]float32{s1.angularVelocity.Z, s2.angularVelocity.Z, s3.angularVelocity.Z, s4.angularVelocity.Z}}
		simdBody.dp.X = floatW{[laneCount]float32{s1.deltaPosition.X, s2.deltaPosition.X, s3.deltaPosition.X, s4.deltaPosition.X}}
		simdBody.dp.Y = floatW{[laneCount]float32{s1.deltaPosition.Y, s2.deltaPosition.Y, s3.deltaPosition.Y, s4.deltaPosition.Y}}
		simdBody.dp.Z = floatW{[laneCount]float32{s1.deltaPosition.Z, s2.deltaPosition.Z, s3.deltaPosition.Z, s4.deltaPosition.Z}}
		simdBody.dq.V.X = floatW{[laneCount]float32{s1.deltaRotation.V.X, s2.deltaRotation.V.X, s3.deltaRotation.V.X, s4.deltaRotation.V.X}}
		simdBody.dq.V.Y = floatW{[laneCount]float32{s1.deltaRotation.V.Y, s2.deltaRotation.V.Y, s3.deltaRotation.V.Y, s4.deltaRotation.V.Y}}
		simdBody.dq.V.Z = floatW{[laneCount]float32{s1.deltaRotation.V.Z, s2.deltaRotation.V.Z, s3.deltaRotation.V.Z, s4.deltaRotation.V.Z}}
		simdBody.dq.S = floatW{[laneCount]float32{s1.deltaRotation.S, s2.deltaRotation.S, s3.deltaRotation.S, s4.deltaRotation.S}}

		return simdBody
	}

	identity := &identityBodyState

	// Indices are 0 for null
	p1 := identity
	if indices[0] != 0 {
		p1 = &states[indices[0]-1]
	}
	p2 := identity
	if indices[1] != 0 {
		p2 = &states[indices[1]-1]
	}
	p3 := identity
	if indices[2] != 0 {
		p3 = &states[indices[2]-1]
	}
	p4 := identity
	if indices[3] != 0 {
		p4 = &states[indices[3]-1]
	}

	var s bodyStateW
	s.v.X, s.v.Y, s.v.Z, _ = transposeW(bodyStateLoadW(p1, 0), bodyStateLoadW(p2, 0), bodyStateLoadW(p3, 0), bodyStateLoadW(p4, 0))
	s.w.X, s.w.Y, s.w.Z, _ = transposeW(bodyStateLoadW(p1, 4), bodyStateLoadW(p2, 4), bodyStateLoadW(p3, 4), bodyStateLoadW(p4, 4))
	s.dp.X, s.dp.Y, s.dp.Z, _ = transposeW(bodyStateLoadW(p1, 8), bodyStateLoadW(p2, 8), bodyStateLoadW(p3, 8), bodyStateLoadW(p4, 8))
	s.dq.V.X, s.dq.V.Y, s.dq.V.Z, s.dq.S = transposeW(bodyStateLoadW(p1, 12), bodyStateLoadW(p2, 12), bodyStateLoadW(p3, 12), bodyStateLoadW(p4, 12))
	return s
}

// bodyStateLoadW is b3LoadW( (const float*)state + offset ) for offset 0, 4, 8 or 12: one 16 byte
// row of the 64 byte b3BodyState (velocity + pad, angular velocity + pad, delta position + flags,
// delta rotation).
// PORT: Go-only helper replacing the float pointer view of b3BodyState.
func bodyStateLoadW(state *bodyState, offset int) floatW {
	switch offset {
	case 0:
		return setW(state.linearVelocity.X, state.linearVelocity.Y, state.linearVelocity.Z, state.padding1)
	case 4:
		return setW(state.angularVelocity.X, state.angularVelocity.Y, state.angularVelocity.Z, state.padding2)
	case 8:
		return setW(state.deltaPosition.X, state.deltaPosition.Y, state.deltaPosition.Z, math.Float32frombits(state.flags))
	default:
		return setW(state.deltaRotation.V.X, state.deltaRotation.V.Y, state.deltaRotation.V.Z, state.deltaRotation.S)
	}
}

// storeBodyVelocity is b3StoreBodyVelocity (B3_SIMD_WIDTH == 4, not B3_SIMD_NONE).
func storeBodyVelocity(states []bodyState, index int32, v, w floatW) {
	// Indices are 0 for null
	if index == 0 {
		return
	}

	state := &states[index-1]
	flags := state.flags
	if (flags & dynamicFlag) == 0 {
		return
	}

	// PORT: b3StoreW( (float*)state, v ) and b3StoreW( (float*)state + 4, w ) write the velocity
	// rows including the padding lanes.
	state.linearVelocity = Vec3{v.v[0], v.v[1], v.v[2]}
	state.padding1 = v.v[3]
	state.angularVelocity = Vec3{w.v[0], w.v[1], w.v[2]}
	state.padding2 = w.v[3]
}

// scatterBodies is b3ScatterBodies.
// This writes only the velocities back to the solver bodies
// PORT: `#if defined( B3_SIMD_NONE )` becomes a branch on simdNone, as in gatherBodies.
func scatterBodies(states []bodyState, indices *[laneCount]int32, simdBody *bodyStateW) {
	if simdNone {
		index1 := indices[0] - 1
		if index1 != -1 && (states[index1].flags&dynamicFlag) != 0 {
			state := &states[index1]
			state.linearVelocity.X = simdBody.v.X.v[0]
			state.linearVelocity.Y = simdBody.v.Y.v[0]
			state.linearVelocity.Z = simdBody.v.Z.v[0]
			state.angularVelocity.X = simdBody.w.X.v[0]
			state.angularVelocity.Y = simdBody.w.Y.v[0]
			state.angularVelocity.Z = simdBody.w.Z.v[0]
		}

		index2 := indices[1] - 1
		if index2 != -1 && (states[index2].flags&dynamicFlag) != 0 {
			state := &states[index2]
			state.linearVelocity.X = simdBody.v.X.v[1]
			state.linearVelocity.Y = simdBody.v.Y.v[1]
			state.linearVelocity.Z = simdBody.v.Z.v[1]
			state.angularVelocity.X = simdBody.w.X.v[1]
			state.angularVelocity.Y = simdBody.w.Y.v[1]
			state.angularVelocity.Z = simdBody.w.Z.v[1]
		}

		index3 := indices[2] - 1
		if index3 != -1 && (states[index3].flags&dynamicFlag) != 0 {
			state := &states[index3]
			state.linearVelocity.X = simdBody.v.X.v[2]
			state.linearVelocity.Y = simdBody.v.Y.v[2]
			state.linearVelocity.Z = simdBody.v.Z.v[2]
			state.angularVelocity.X = simdBody.w.X.v[2]
			state.angularVelocity.Y = simdBody.w.Y.v[2]
			state.angularVelocity.Z = simdBody.w.Z.v[2]
		}

		index4 := indices[3] - 1
		if index4 != -1 && (states[index4].flags&dynamicFlag) != 0 {
			state := &states[index4]
			state.linearVelocity.X = simdBody.v.X.v[3]
			state.linearVelocity.Y = simdBody.v.Y.v[3]
			state.linearVelocity.Z = simdBody.v.Z.v[3]
			state.angularVelocity.X = simdBody.w.X.v[3]
			state.angularVelocity.Y = simdBody.w.Y.v[3]
			state.angularVelocity.Z = simdBody.w.Z.v[3]
		}
		return
	}

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	zero := zeroW()
	v1, v2, v3, v4 := transposeW(simdBody.v.X, simdBody.v.Y, simdBody.v.Z, zero)
	w1, w2, w3, w4 := transposeW(simdBody.w.X, simdBody.w.Y, simdBody.w.Z, zero)

	storeBodyVelocity(states, indices[0], v1, w1)
	storeBodyVelocity(states, indices[1], v2, w2)
	storeBodyVelocity(states, indices[2], v3, w3)
	storeBodyVelocity(states, indices[3], v4, w4)
}

// PORT: the manifold point / manifold / matrix layout _Static_asserts are dropped; the loads below
// read the fields those offsets name.

// zeroContact, zeroManifold and zeroBodySim are b3_zeroContact, b3_zeroManifold and b3_zeroBodySim.
var (
	zeroContact  = contact{}
	zeroManifold = Manifold{}
	zeroBodySim  = bodySim{}
)

// PORT: B3_GATHER_LANES( wide, lanes, field ) is written out as setW( lanes[0].field, … ) at each
// site.

// gatherInvInertiaW is b3GatherInvInertiaW.
// PORT: b3LoadW( i + k ) reads invInertiaWorld as the float array cx.x cx.y cx.z cy.x | cy.y cy.z
// cz.x cz.y | cz.z.
func gatherInvInertiaW(simLanes *[laneCount]*bodySim) symMatrix3W {
	i0 := &simLanes[0].invInertiaWorld
	i1 := &simLanes[1].invInertiaWorld
	i2 := &simLanes[2].invInertiaWorld
	i3 := &simLanes[3].invInertiaWorld

	var m symMatrix3W
	m.cxx, m.cxy, m.cxz, _ = transposeW(
		setW(i0.Cx.X, i0.Cx.Y, i0.Cx.Z, i0.Cy.X), setW(i1.Cx.X, i1.Cx.Y, i1.Cx.Z, i1.Cy.X),
		setW(i2.Cx.X, i2.Cx.Y, i2.Cx.Z, i2.Cy.X), setW(i3.Cx.X, i3.Cx.Y, i3.Cx.Z, i3.Cy.X))
	m.cyy, m.cyz, _, _ = transposeW(
		setW(i0.Cy.Y, i0.Cy.Z, i0.Cz.X, i0.Cz.Y), setW(i1.Cy.Y, i1.Cy.Z, i1.Cz.X, i1.Cz.Y),
		setW(i2.Cy.Y, i2.Cy.Z, i2.Cz.X, i2.Cz.Y), setW(i3.Cy.Y, i3.Cy.Z, i3.Cz.X, i3.Cz.Y))
	m.czz = setW(i0.Cz.Z, i1.Cz.Z, i2.Cz.Z, i3.Cz.Z)
	return m
}

// addSymW is b3AddSymW.
func addSymW(a, b symMatrix3W) symMatrix3W {
	return symMatrix3W{
		addW(a.cxx, b.cxx), addW(a.cxy, b.cxy), addW(a.cxz, b.cxz),
		addW(a.cyy, b.cyy), addW(a.cyz, b.cyz), addW(a.czz, b.czz),
	}
}

// invertSymW is b3InvertSymW.
func invertSymW(m symMatrix3W) symMatrix3W {
	cxx := subW(mulW(m.cyy, m.czz), mulW(m.cyz, m.cyz))
	cxy := subW(mulW(m.cxz, m.cyz), mulW(m.cxy, m.czz))
	cxz := subW(mulW(m.cxy, m.cyz), mulW(m.cxz, m.cyy))
	cyy := subW(mulW(m.cxx, m.czz), mulW(m.cxz, m.cxz))
	cyz := subW(mulW(m.cxy, m.cxz), mulW(m.cxx, m.cyz))
	czz := subW(mulW(m.cxx, m.cyy), mulW(m.cxy, m.cxy))

	det := addW(mulW(m.cxx, cxx), addW(mulW(m.cxy, cxy), mulW(m.cxz, cxz)))
	valid := greaterThanW(absW(det), splatW(1000.0*fltMin))
	invDet := blendW(zeroW(), divW(splatW(1.0), det), valid)

	return symMatrix3W{
		mulW(invDet, cxx), mulW(invDet, cxy), mulW(invDet, cxz),
		mulW(invDet, cyy), mulW(invDet, cyz), mulW(invDet, czz),
	}
}

// perpW is b3PerpW.
func perpW(a vec3W) vec3W {
	zero := zeroW()
	half := splatW(0.5)
	mask := orW(lessThanW(a.X, negW(half)), greaterThanW(a.X, half))

	var p vec3W
	p.X = blendW(zero, a.Y, mask)
	p.Y = blendW(a.Z, negW(a.X), mask)
	p.Z = blendW(negW(a.Y), zero, mask)

	lengthSquared := dotW(p, p)
	valid := greaterThanW(lengthSquared, splatW(1000.0*fltMin))
	s := blendW(zero, divW(splatW(1.0), sqrtW(lengthSquared)), valid)
	return mulSVW(s, p)
}

// intsToFloatW is b3IntsToFloatW.
func intsToFloatW(v *[laneCount]int32) floatW {
	return setW(float32(v[0]), float32(v[1]), float32(v[2]), float32(v[3]))
}

// getManifoldsW is b3GetManifoldsW.
// Get the contiguous manifold array that trails the constraint.
// PORT: the constraint is identified by its manifold position m in base (see getConstraintW).
func getManifoldsW(base wideConstraintBuffer, m int) []contactManifoldWide {
	return base.manifolds[m:]
}

// getConstraintW is b3GetConstraintW.
// fixedManifoldCount is 1 to indicate a convex contact with one manifold.
// PORT: returns the constraint's position in base as (constraint index, manifold index): the
// constraint is base.constraints[c] and its manifolds start at base.manifolds[m], the two parts of
// the byte offset upstream computes.
func getConstraintW(base wideConstraintBuffer, manifoldStarts []int32, index int, fixedManifoldCount int) (c, m int) {
	if fixedManifoldCount > 0 {
		return index, index * fixedManifoldCount
	}

	return index, int(manifoldStarts[index] - manifoldStarts[0])
}

// nextConstraintW is b3NextConstraintW.
// PORT: advances the (constraint index, manifold index) position of getConstraintW.
func nextConstraintW(c, m int, manifoldCount int) (int, int) {
	return c + 1, m + manifoldCount
}

// prepareConstraintW is b3PrepareConstraintW.
// Contact constraint data is stored in a heterogious stream. Constraints followed by an array of constraint manifolds.
// [c1 m11 m12 c2 m21 c3 m31 m32 m33 ... ]
// PORT: the constraint pointer is base.constraints[ci] with its manifolds at base.manifolds[mi:].
func prepareConstraintW(base wideConstraintBuffer, ci, mi int, contacts *[laneCount]*contact, manifoldCount int,
	context *stepContext, warmStartScale floatW,
) bool {
	c := &base.constraints[ci]
	sims := context.sims
	states := context.states

	zeroW := zeroW()
	oneW := splatW(1.0)
	twoW := splatW(2.0)
	invTau := splatW(1.0 / SpeculativeDistance())
	minFrictionWeight := splatW(MinFrictionWeight)
	minDet := splatW(1000.0 * fltMin)

	var contactLanes [laneCount]*contact
	var simLanesA [laneCount]*bodySim
	var simLanesB [laneCount]*bodySim
	var manifoldCounts [laneCount]int
	hitEventLanes := 0
	maxManifoldCount := 0

	for lane := range laneCount {
		contact := contacts[lane]
		c.contacts[lane] = contact

		if contact != nil {
			indexA := decodeAwakeIndex(int(contact.encodedBodySimA))
			indexB := decodeAwakeIndex(int(contact.encodedBodySimB))

			// PORT: #if B3_ENABLE_VALIDATION becomes a branch on validateEnabled.
			if validateEnabled {
				bodies := context.world.bodies
				bodyA := &bodies[contact.edges[0].bodyId]
				bodyB := &bodies[contact.edges[1].bodyId]
				assert(int(contact.encodedBodySimA) == encodeBodySimIndex(bodyA), "contact->encodedBodySimA == b3EncodeBodySimIndex( bodyA )")
				assert(int(contact.encodedBodySimB) == encodeBodySimIndex(bodyB), "contact->encodedBodySimB == b3EncodeBodySimIndex( bodyB )")
			}

			c.indexA[lane] = int32(indexA + 1)
			c.indexB[lane] = int32(indexB + 1)
			manifoldCounts[lane] = int(contact.manifoldCount)

			contactLanes[lane] = contact
			if indexA == NullIndex {
				simLanesA[lane] = &zeroBodySim
			} else {
				simLanesA[lane] = &sims[indexA]
			}
			if indexB == NullIndex {
				simLanesB[lane] = &zeroBodySim
			} else {
				simLanesB[lane] = &sims[indexB]
			}
			if (contact.flags & simEnableHitEvent) != 0 {
				hitEventLanes |= 1 << lane
			}
			maxManifoldCount = MaxInt(maxManifoldCount, int(contact.manifoldCount))
		} else {
			c.indexA[lane] = 0
			c.indexB[lane] = 0
			manifoldCounts[lane] = 0

			contactLanes[lane] = &zeroContact
			simLanesA[lane] = &zeroBodySim
			simLanesB[lane] = &zeroBodySim
		}
	}

	validate(maxManifoldCount == manifoldCount, "maxManifoldCount == manifoldCount")
	c.manifoldCount = int32(manifoldCount)

	mA := setW(simLanesA[0].invMass, simLanesA[1].invMass, simLanesA[2].invMass, simLanesA[3].invMass)
	mB := setW(simLanesB[0].invMass, simLanesB[1].invMass, simLanesB[2].invMass, simLanesB[3].invMass)
	iA := gatherInvInertiaW(&simLanesA)
	iB := gatherInvInertiaW(&simLanesB)
	c.invMassA = mA
	c.invMassB = mB
	c.invIA = iA
	c.invIB = iB

	var tangentVelocity vec3W
	c.friction = setW(contactLanes[0].friction, contactLanes[1].friction, contactLanes[2].friction, contactLanes[3].friction)
	c.rollingResistance = setW(contactLanes[0].rollingResistance, contactLanes[1].rollingResistance,
		contactLanes[2].rollingResistance, contactLanes[3].rollingResistance)
	c.restitution = setW(contactLanes[0].restitution, contactLanes[1].restitution, contactLanes[2].restitution,
		contactLanes[3].restitution)
	tangentVelocity.X = setW(contactLanes[0].tangentVelocity.X, contactLanes[1].tangentVelocity.X,
		contactLanes[2].tangentVelocity.X, contactLanes[3].tangentVelocity.X)
	tangentVelocity.Y = setW(contactLanes[0].tangentVelocity.Y, contactLanes[1].tangentVelocity.Y,
		contactLanes[2].tangentVelocity.Y, contactLanes[3].tangentVelocity.Y)
	tangentVelocity.Z = setW(contactLanes[0].tangentVelocity.Z, contactLanes[1].tangentVelocity.Z,
		contactLanes[2].tangentVelocity.Z, contactLanes[3].tangentVelocity.Z)

	invIAB := addSymW(iA, iB)
	if allZeroW(c.rollingResistance) == false {
		c.rollingMass = invertSymW(invIAB)
	} else {
		c.rollingMass = symMatrix3W{zeroW, zeroW, zeroW, zeroW, zeroW, zeroW}
	}

	rollingResistanceMask := greaterThanW(c.rollingResistance, zeroW)

	manifolds := getManifoldsW(base, mi)
	for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
		cm := &manifolds[manifoldIndex]

		var manifoldLanes [laneCount]*Manifold
		var activeLanes [laneCount]int32
		var pointCounts [laneCount]int32
		pointCount := 0
		for lane := range laneCount {
			if manifoldIndex < manifoldCounts[lane] {
				manifold := &contactLanes[lane].manifolds[manifoldIndex]
				manifoldLanes[lane] = manifold
				activeLanes[lane] = 1
				pointCounts[lane] = manifold.PointCount
			} else {
				manifoldLanes[lane] = &zeroManifold
				activeLanes[lane] = 0
				pointCounts[lane] = 0
			}

			pointCount = MaxInt(pointCount, int(pointCounts[lane]))
		}

		validate(0 <= pointCount && pointCount <= MaxManifoldPoints, "0 <= pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS")
		cm.pointCount = int32(pointCount)
		cm.rollingMask = andW(rollingResistanceMask, greaterThanW(intsToFloatW(&activeLanes), zeroW))

		var normal, frictionImpulse, rollingImpulse vec3W
		var twistImpulse floatW
		{
			// PORT: b3LoadW( &manifold->normal.x + k ) reads normal.x normal.y normal.z twistImpulse |
			// frictionImpulse.x .y .z rollingImpulse.x | rollingImpulse.y | rollingImpulse.z.
			m0 := manifoldLanes[0]
			m1 := manifoldLanes[1]
			m2 := manifoldLanes[2]
			m3 := manifoldLanes[3]
			normal.X, normal.Y, normal.Z, twistImpulse = transposeW(
				setW(m0.Normal.X, m0.Normal.Y, m0.Normal.Z, m0.TwistImpulse),
				setW(m1.Normal.X, m1.Normal.Y, m1.Normal.Z, m1.TwistImpulse),
				setW(m2.Normal.X, m2.Normal.Y, m2.Normal.Z, m2.TwistImpulse),
				setW(m3.Normal.X, m3.Normal.Y, m3.Normal.Z, m3.TwistImpulse))
			frictionImpulse.X, frictionImpulse.Y, frictionImpulse.Z, rollingImpulse.X = transposeW(
				setW(m0.FrictionImpulse.X, m0.FrictionImpulse.Y, m0.FrictionImpulse.Z, m0.RollingImpulse.X),
				setW(m1.FrictionImpulse.X, m1.FrictionImpulse.Y, m1.FrictionImpulse.Z, m1.RollingImpulse.X),
				setW(m2.FrictionImpulse.X, m2.FrictionImpulse.Y, m2.FrictionImpulse.Z, m2.RollingImpulse.X),
				setW(m3.FrictionImpulse.X, m3.FrictionImpulse.Y, m3.FrictionImpulse.Z, m3.RollingImpulse.X))
			rollingImpulse.Y = setW(m0.RollingImpulse.Y, m1.RollingImpulse.Y, m2.RollingImpulse.Y, m3.RollingImpulse.Y)
			rollingImpulse.Z = setW(m0.RollingImpulse.Z, m1.RollingImpulse.Z, m2.RollingImpulse.Z, m3.RollingImpulse.Z)
		}

		tangent1 := perpW(normal)
		tangent2 := crossW(tangent1, normal)
		cm.normal = normal
		cm.tangent1 = tangent1
		cm.tangent2 = tangent2
		cm.tangentVelocity1 = dotW(tangentVelocity, tangent1)
		cm.tangentVelocity2 = dotW(tangentVelocity, tangent2)

		cm.twistImpulse = mulW(warmStartScale, twistImpulse)
		cm.rollingImpulse.X = blendW(zeroW, mulW(warmStartScale, rollingImpulse.X), cm.rollingMask)
		cm.rollingImpulse.Y = blendW(zeroW, mulW(warmStartScale, rollingImpulse.Y), cm.rollingMask)
		cm.rollingImpulse.Z = blendW(zeroW, mulW(warmStartScale, rollingImpulse.Z), cm.rollingMask)
		cm.frictionImpulse.x = mulW(warmStartScale, dotW(frictionImpulse, tangent1))
		cm.frictionImpulse.y = mulW(warmStartScale, dotW(frictionImpulse, tangent2))

		pointCountW := intsToFloatW(&pointCounts)

		centerA := vec3W{zeroW, zeroW, zeroW}
		centerB := vec3W{zeroW, zeroW, zeroW}
		totalFrictionWeight := zeroW

		for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
			cp := &cm.points[pointIndex]
			pointMask := greaterThanW(pointCountW, splatW(float32(pointIndex)))

			// PORT: b3LoadW( (const float*)( manifold->points + pointIndex ) + k ) reads anchorA.x .y .z
			// anchorB.x | anchorB.y .z separation normalImpulse.
			p0 := &manifoldLanes[0].Points[pointIndex]
			p1 := &manifoldLanes[1].Points[pointIndex]
			p2 := &manifoldLanes[2].Points[pointIndex]
			p3 := &manifoldLanes[3].Points[pointIndex]

			var rA, rB vec3W
			var separation, normalImpulse floatW
			rA.X, rA.Y, rA.Z, rB.X = transposeW(
				setW(p0.AnchorA.X, p0.AnchorA.Y, p0.AnchorA.Z, p0.AnchorB.X),
				setW(p1.AnchorA.X, p1.AnchorA.Y, p1.AnchorA.Z, p1.AnchorB.X),
				setW(p2.AnchorA.X, p2.AnchorA.Y, p2.AnchorA.Z, p2.AnchorB.X),
				setW(p3.AnchorA.X, p3.AnchorA.Y, p3.AnchorA.Z, p3.AnchorB.X))
			rB.Y, rB.Z, separation, normalImpulse = transposeW(
				setW(p0.AnchorB.Y, p0.AnchorB.Z, p0.Separation, p0.NormalImpulse),
				setW(p1.AnchorB.Y, p1.AnchorB.Z, p1.Separation, p1.NormalImpulse),
				setW(p2.AnchorB.Y, p2.AnchorB.Z, p2.Separation, p2.NormalImpulse),
				setW(p3.AnchorB.Y, p3.AnchorB.Z, p3.Separation, p3.NormalImpulse))

			rA.X = blendW(zeroW, rA.X, pointMask)
			rA.Y = blendW(zeroW, rA.Y, pointMask)
			rA.Z = blendW(zeroW, rA.Z, pointMask)
			rB.X = blendW(zeroW, rB.X, pointMask)
			rB.Y = blendW(zeroW, rB.Y, pointMask)
			rB.Z = blendW(zeroW, rB.Z, pointMask)
			separation = blendW(zeroW, separation, pointMask)
			normalImpulse = blendW(zeroW, normalImpulse, pointMask)

			// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
			// See details in b3PrepareContacts_Mesh. This code should stay in sync.
			weight := minW(maxW(subW(twoW, mulW(separation, invTau)), minFrictionWeight), oneW)
			weight = blendW(zeroW, weight, pointMask)
			centerA = mulAddSVW(centerA, weight, rA)
			centerB = mulAddSVW(centerB, weight, rB)
			totalFrictionWeight = addW(totalFrictionWeight, weight)

			cp.anchorAs = rA
			cp.anchorBs = rB
			cp.baseSeparations = subW(separation, dotW(subVW(rB, rA), normal))
			cp.normalImpulses = mulW(warmStartScale, normalImpulse)
			cp.totalNormalImpulses = zeroW
			cp.relativeVelocities = zeroW
			cp.restitutionImpulses = zeroW

			rnA := crossW(rA, normal)
			rnB := crossW(rB, normal)
			kNormal := addW(mA, mB)
			kNormal = addW(kNormal, dotW(rnA, mulMVW(iA, rnA)))
			kNormal = addW(kNormal, dotW(rnB, mulMVW(iB, rnB)))
			valid := andW(greaterThanW(kNormal, zeroW), pointMask)
			cp.normalMasses = blendW(zeroW, divW(oneW, kNormal), valid)
		}

		frictionValid := greaterThanW(totalFrictionWeight, zeroW)
		invWeight := blendW(zeroW, divW(oneW, totalFrictionWeight), frictionValid)
		centerA = mulSVW(invWeight, centerA)
		centerB = mulSVW(invWeight, centerB)
		cm.centerA = centerA
		cm.centerB = centerB

		for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
			cp := &cm.points[pointIndex]
			pointMask := greaterThanW(pointCountW, splatW(float32(pointIndex)))
			d := subVW(cp.anchorAs, centerA)
			cp.leverArms = blendW(zeroW, sqrtW(dotW(d, d)), pointMask)
		}

		{
			rtA1 := crossW(centerA, tangent1)
			rtA2 := crossW(centerA, tangent2)
			rtB1 := crossW(centerB, tangent1)
			rtB2 := crossW(centerB, tangent2)
			iArtA1 := mulMVW(iA, rtA1)
			iArtA2 := mulMVW(iA, rtA2)
			iBrtB1 := mulMVW(iB, rtB1)
			iBrtB2 := mulMVW(iB, rtB2)

			kxx := addW(addW(addW(mA, mB), dotW(rtA1, iArtA1)), dotW(rtB1, iBrtB1))
			kyy := addW(addW(addW(mA, mB), dotW(rtA2, iArtA2)), dotW(rtB2, iBrtB2))
			kxy := addW(dotW(rtA1, iArtA2), dotW(rtB1, iBrtB2))

			det := subW(mulW(kxx, kyy), mulW(kxy, kxy))
			valid := andW(greaterThanW(absW(det), minDet), frictionValid)
			invDet := blendW(zeroW, divW(oneW, det), valid)
			cm.tangentMass.cxx = mulW(invDet, kyy)
			cm.tangentMass.cxy = negW(mulW(invDet, kxy))
			cm.tangentMass.cyy = mulW(invDet, kxx)
		}

		{
			kTwist := dotW(normal, mulMVW(invIAB, normal))
			cm.twistMass = blendW(zeroW, divW(oneW, kTwist), greaterThanW(kTwist, zeroW))
		}
	}

	// Only sample contact point normal velocity if needed.
	haveRestitution := anyTrueW(greaterThanW(c.restitution, zeroW))
	if hitEventLanes != 0 || haveRestitution {
		bA := gatherBodies(states, &c.indexA)
		bB := gatherBodies(states, &c.indexB)

		for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
			cm := &manifolds[manifoldIndex]

			for pointIndex := 0; pointIndex < int(cm.pointCount); pointIndex++ {
				cp := &cm.points[pointIndex]

				vrA := addVW(bA.v, crossW(bA.w, cp.anchorAs))
				vrB := addVW(bB.v, crossW(bB.w, cp.anchorBs))
				vn := dotW(cm.normal, subVW(vrB, vrA))
				cp.relativeVelocities = vn

				if hitEventLanes != 0 {
					var normalVelocities [laneCount]float32
					storeW(normalVelocities[:], vn)

					for lane := range laneCount {
						if (hitEventLanes&(1<<lane)) == 0 || manifoldIndex >= manifoldCounts[lane] {
							continue
						}

						manifold := &c.contacts[lane].manifolds[manifoldIndex]
						if pointIndex < int(manifold.PointCount) {
							manifold.Points[pointIndex].NormalVelocity = normalVelocities[lane]
						}
					}
				}
			}
		}
	}

	return haveRestitution
}

// prepareContactsConvexW is b3PrepareContacts_ConvexW4.
func prepareContactsConvexW(block solverBlock, context *stepContext) {
	world := context.world
	spans := context.widePrepareSpans
	wideBase := context.wideConstraints
	warmStartScale := zeroW()
	if world.enableWarmStarting {
		warmStartScale = splatW(1.0)
	}
	anyRestitution := false

	wideIndex := int(block.startIndex)
	endWideIndex := int(block.startIndex) + int(block.count)

	// Find color for start index. Linear search but fast.
	colorIndex := 0
	for int(spans[colorIndex+1].start) <= wideIndex {
		colorIndex += 1
	}

	// Loop over block
	for wideIndex < endWideIndex {
		colorWideStart := int(spans[colorIndex].start)
		colorWideEndIndex := MinInt(int(spans[colorIndex+1].start), endWideIndex)
		colorContactCount := int(spans[colorIndex].count)
		contactIds := spans[colorIndex].contacts

		// Loop over color
		for ; wideIndex < colorWideEndIndex; wideIndex++ {
			ci, mi := getConstraintW(wideBase, nil, wideIndex, 1)
			localWideIndex := wideIndex - colorWideStart

			var contacts [laneCount]*contact
			for lane := range laneCount {
				contactIndex := laneCount*localWideIndex + lane
				if contactIndex < colorContactCount {
					contacts[lane] = arrayGet(world.contacts, int(contactIds[contactIndex]))
					assert(contacts[lane].manifoldCount == 1, "contacts[lane]->manifoldCount == 1")
				} else {
					contacts[lane] = nil
				}
			}

			haveRestitution := prepareConstraintW(wideBase, ci, mi, &contacts, 1, context, warmStartScale)
			anyRestitution = anyRestitution || haveRestitution
		}

		// Advance to next color
		colorIndex += 1
	}

	if anyRestitution {
		atomicStoreInt(&context.anyRestitution, 1)
	}
}

// prepareContactsMeshWideW is b3PrepareContacts_MeshWideW4.
func prepareContactsMeshWideW(block solverBlock, context *stepContext) {
	world := context.world
	spans := context.meshPrepareSpans
	wideBase := context.wideMeshConstraints
	manifoldStarts := context.wideMeshManifoldStarts
	warmStartScale := zeroW()
	if world.enableWarmStarting {
		warmStartScale = splatW(1.0)
	}
	anyRestitution := false

	wideIndex := int(block.startIndex)
	endWideIndex := int(block.startIndex) + int(block.count)

	colorIndex := 0
	for int(spans[colorIndex+1].start) <= wideIndex {
		colorIndex += 1
	}

	for wideIndex < endWideIndex {
		colorWideStart := int(spans[colorIndex].start)
		colorWideEndIndex := MinInt(int(spans[colorIndex+1].start), endWideIndex)
		colorContactCount := int(spans[colorIndex].count)
		specs := spans[colorIndex].contacts

		// This order is used to group contacts with similar manifold counts to keep the lanes full.
		order := spans[colorIndex].order

		for ; wideIndex < colorWideEndIndex; wideIndex++ {
			ci, mi := getConstraintW(wideBase, manifoldStarts, wideIndex, 0)
			localWideIndex := wideIndex - colorWideStart

			var contacts [laneCount]*contact
			for lane := range laneCount {
				contactIndex := laneCount*localWideIndex + lane
				if contactIndex < colorContactCount {
					spec := &specs[order[contactIndex]]
					contact := arrayGet(world.contacts, int(spec.contactId))
					assert(contact.contactId == spec.contactId, "contact->contactId == spec->contactId")
					assert(int(contact.manifoldCount) == int(spec.manifoldCount), "contact->manifoldCount == spec->manifoldCount")
					contacts[lane] = contact
				} else {
					contacts[lane] = nil
				}
			}

			manifoldCount := int(manifoldStarts[wideIndex+1] - manifoldStarts[wideIndex])
			haveRestitution := prepareConstraintW(wideBase, ci, mi, &contacts, manifoldCount, context, warmStartScale)
			anyRestitution = anyRestitution || haveRestitution
		}

		colorIndex += 1
	}

	if anyRestitution {
		atomicStoreInt(&context.anyRestitution, 1)
	}
}

// warmStartContactsW is b3WarmStartContactsW.
func warmStartContactsW(block solverBlock, context *stepContext, base wideConstraintBuffer, manifoldStarts []int32,
	fixedManifoldCount int,
) {
	states := context.states
	zeroW := zeroW()

	ci, mi := getConstraintW(base, manifoldStarts, int(block.startIndex), fixedManifoldCount)
	for i := 0; i < int(block.count); i++ {
		c := &base.constraints[ci]
		manifoldCount := fixedManifoldCount
		if fixedManifoldCount <= 0 {
			manifoldCount = int(c.manifoldCount)
		}
		anyRolling := allZeroW(c.rollingResistance) == false

		linearImpulse := vec3W{zeroW, zeroW, zeroW}
		angularImpulseA := vec3W{zeroW, zeroW, zeroW}
		angularImpulseB := vec3W{zeroW, zeroW, zeroW}

		manifolds := getManifoldsW(base, mi)
		for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
			cm := &manifolds[manifoldIndex]

			totalNormalImpulse := zeroW
			momentA := vec3W{zeroW, zeroW, zeroW}
			momentB := vec3W{zeroW, zeroW, zeroW}

			for pointIndex := 0; pointIndex < int(cm.pointCount); pointIndex++ {
				cp := &cm.points[pointIndex]
				normalImpulse := cp.normalImpulses
				totalNormalImpulse = addW(totalNormalImpulse, normalImpulse)
				momentA = mulAddSVW(momentA, normalImpulse, cp.anchorAs)
				momentB = mulAddSVW(momentB, normalImpulse, cp.anchorBs)
			}

			normal := cm.normal
			frictionImpulse := mulSVW(cm.frictionImpulse.x, cm.tangent1)
			frictionImpulse = mulAddSVW(frictionImpulse, cm.frictionImpulse.y, cm.tangent2)

			linear := mulAddSVW(frictionImpulse, totalNormalImpulse, normal)

			twistImpulse := mulSVW(cm.twistImpulse, normal)
			angularA :=
				addVW(addVW(crossW(momentA, normal), crossW(cm.centerA, frictionImpulse)), twistImpulse)
			angularB :=
				addVW(addVW(crossW(momentB, normal), crossW(cm.centerB, frictionImpulse)), twistImpulse)

			if anyRolling {
				angularA = addVW(angularA, cm.rollingImpulse)
				angularB = addVW(angularB, cm.rollingImpulse)
			}

			linearImpulse = addVW(linearImpulse, linear)
			angularImpulseA = addVW(angularImpulseA, angularA)
			angularImpulseB = addVW(angularImpulseB, angularB)
		}

		bA := gatherBodies(states, &c.indexA)
		bB := gatherBodies(states, &c.indexB)

		bA.w = mulSubMVW(bA.w, c.invIA, angularImpulseA)
		bA.v = mulSubSVW(bA.v, c.invMassA, linearImpulse)
		bB.w = mulAddMVW(bB.w, c.invIB, angularImpulseB)
		bB.v = mulAddSVW(bB.v, c.invMassB, linearImpulse)

		scatterBodies(states, &c.indexA, &bA)
		scatterBodies(states, &c.indexB, &bB)

		ci, mi = nextConstraintW(ci, mi, manifoldCount)
	}
}

// pushContactsW is b3PushContactsW.
// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
func pushContactsW(block solverBlock, context *stepContext, base wideConstraintBuffer, manifoldStarts []int32,
	fixedManifoldCount int,
) {
	states := context.states
	inv_h := splatW(context.inv_h)
	contactSpeed := splatW(-context.world.contactSpeed)
	oneW := splatW(1.0)
	zeroW := zeroW()

	// Stiffer for static contacts to avoid bodies getting pushed through the ground. Selected per
	// lane from the null body index instead of stored per constraint.
	dynamicBiasRate := splatW(float32(context.contactSoftness.massScale * context.contactSoftness.biasRate))
	dynamicMassScale := splatW(context.contactSoftness.massScale)
	dynamicImpulseScale := splatW(context.contactSoftness.impulseScale)
	staticBiasRate := splatW(float32(context.staticSoftness.massScale * context.staticSoftness.biasRate))
	staticMassScale := splatW(context.staticSoftness.massScale)
	staticImpulseScale := splatW(context.staticSoftness.impulseScale)

	ci, mi := getConstraintW(base, manifoldStarts, int(block.startIndex), fixedManifoldCount)
	for i := 0; i < int(block.count); i++ {
		c := &base.constraints[ci]
		manifoldCount := fixedManifoldCount
		if fixedManifoldCount <= 0 {
			manifoldCount = int(c.manifoldCount)
		}
		bA := gatherBodies(states, &c.indexA)
		bB := gatherBodies(states, &c.indexB)

		softMask := softMaskW(c.indexA[:], c.indexB[:])
		biasRate := blendW(dynamicBiasRate, staticBiasRate, softMask)
		massScale := blendW(dynamicMassScale, staticMassScale, softMask)
		impulseScale := blendW(dynamicImpulseScale, staticImpulseScale, softMask)

		dp := subVW(bB.dp, bA.dp)
		invMass := addW(c.invMassA, c.invMassB)

		manifolds := getManifoldsW(base, mi)
		for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
			cm := &manifolds[manifoldIndex]

			// Convert to normals to local space to reduce transform math.
			// Read the normal rather than put on stack to avoid register spills.
			normalSeparation := dotW(cm.normal, dp)
			normalA := invRotateVectorW(bA.dq, cm.normal)
			normalB := invRotateVectorW(bB.dq, cm.normal)

			linearNormalVelocity := dotW(cm.normal, subVW(bB.v, bA.v))
			manifoldImpulse := zeroW

			for pointIndex := 0; pointIndex < int(cm.pointCount); pointIndex++ {
				cp := &cm.points[pointIndex]

				// Fixed anchor points for applying impulses
				rA := cp.anchorAs
				rB := cp.anchorBs

				s := addW(addW(normalSeparation, subW(dotW(normalB, rB), dotW(normalA, rA))),
					cp.baseSeparations)

				// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
				separated := greaterThanW(s, zeroW)

				// Speculative bias - positive
				specBias := mulW(s, inv_h)

				// Overlap bias - negative
				overlapBias := maxW(mulW(biasRate, s), contactSpeed)
				velocityBias := blendW(overlapBias, specBias, separated)

				pointMassScale := blendW(massScale, oneW, separated)
				pointImpulseScale := blendW(impulseScale, zeroW, separated)

				uA := crossW(rA, cm.normal)
				uB := crossW(rB, cm.normal)
				gA := mulMVW(c.invIA, uA)
				gB := mulMVW(c.invIB, uB)

				vn := addW(linearNormalVelocity, subW(dotW(uB, bB.w), dotW(uA, bA.w)))

				negImpulse := addW(mulW(cp.normalMasses, addW(mulW(pointMassScale, vn), velocityBias)),
					mulW(pointImpulseScale, cp.normalImpulses))
				impulse := maxW(negW(negImpulse), negW(cp.normalImpulses))
				cp.normalImpulses = addW(cp.normalImpulses, impulse)

				linearNormalVelocity = mulAddW(linearNormalVelocity, invMass, impulse)
				manifoldImpulse = addW(manifoldImpulse, impulse)
				bA.w = mulSubSVW(bA.w, impulse, gA)
				bB.w = mulAddSVW(bB.w, impulse, gB)
			}

			P := mulSVW(manifoldImpulse, cm.normal)
			bA.v = mulSubSVW(bA.v, c.invMassA, P)
			bB.v = mulAddSVW(bB.v, c.invMassB, P)
		}

		scatterBodies(states, &c.indexA, &bA)
		scatterBodies(states, &c.indexB, &bB)

		ci, mi = nextConstraintW(ci, mi, manifoldCount)
	}
}

// solveContactsW is b3SolveContactsW.
// Solve the normal constraint, friction, and rolling resistance.
func solveContactsW(block solverBlock, context *stepContext, base wideConstraintBuffer, manifoldStarts []int32,
	fixedManifoldCount int,
) {
	states := context.states
	inv_h := splatW(context.inv_h)
	oneW := splatW(1.0)
	zeroW := zeroW()
	epsilonW := splatW(fltEpsilon)

	ci, mi := getConstraintW(base, manifoldStarts, int(block.startIndex), fixedManifoldCount)
	for i := 0; i < int(block.count); i++ {
		c := &base.constraints[ci]
		manifoldCount := fixedManifoldCount
		if fixedManifoldCount <= 0 {
			manifoldCount = int(c.manifoldCount)
		}
		bA := gatherBodies(states, &c.indexA)
		bB := gatherBodies(states, &c.indexB)

		anyRolling := allZeroW(c.rollingResistance) == false
		dp := subVW(bB.dp, bA.dp)
		invMass := addW(c.invMassA, c.invMassB)

		manifolds := getManifoldsW(base, mi)
		for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
			cm := &manifolds[manifoldIndex]

			// Read the normal rather than put on stack to avoid register spills.
			normalSeparation := dotW(cm.normal, dp)
			normalA := invRotateVectorW(bA.dq, cm.normal)
			normalB := invRotateVectorW(bB.dq, cm.normal)

			dv := subVW(bB.v, bA.v)
			linearNormalVelocity := dotW(cm.normal, dv)
			linearTangentVelocity1 := subW(dotW(cm.tangent1, dv), cm.tangentVelocity1)
			linearTangentVelocity2 := subW(dotW(cm.tangent2, dv), cm.tangentVelocity2)
			manifoldImpulse := zeroW

			totalNormalImpulse := zeroW
			totalTwistLimit := zeroW

			// Normal contraints
			for pointIndex := 0; pointIndex < int(cm.pointCount); pointIndex++ {
				cp := &cm.points[pointIndex]

				// Fixed anchor points for applying impulses
				rA := cp.anchorAs
				rB := cp.anchorBs

				s := addW(addW(normalSeparation, subW(dotW(normalB, rB), dotW(normalA, rA))),
					cp.baseSeparations)

				// Speculative bias, positive if separated and zero if overlapped
				velocityBias := maxW(zeroW, mulW(s, inv_h))

				uA := crossW(rA, cm.normal)
				uB := crossW(rB, cm.normal)
				gA := mulMVW(c.invIA, uA)
				gB := mulMVW(c.invIB, uB)

				vn := addW(linearNormalVelocity, subW(dotW(uB, bB.w), dotW(uA, bA.w)))

				negImpulse := mulW(cp.normalMasses, addW(vn, velocityBias))
				impulse := maxW(negW(negImpulse), negW(cp.normalImpulses))
				newImpulse := addW(cp.normalImpulses, impulse)
				cp.normalImpulses = newImpulse
				cp.totalNormalImpulses = addW(cp.totalNormalImpulses, newImpulse)
				totalNormalImpulse = addW(totalNormalImpulse, newImpulse)
				totalTwistLimit = addW(totalTwistLimit, mulW(cp.leverArms, newImpulse))

				linearNormalVelocity = mulAddW(linearNormalVelocity, invMass, impulse)
				manifoldImpulse = addW(manifoldImpulse, impulse)
				bA.w = mulSubSVW(bA.w, impulse, gA)
				bB.w = mulAddSVW(bB.w, impulse, gB)
			}

			// Rolling resistance. After normal constraints because it uses the normal impulse.
			if anyRolling {
				// flip A/B order to negate
				deltaImpulse := mulMVW(c.rollingMass, subVW(bA.w, bB.w))
				oldImpulse := cm.rollingImpulse
				cm.rollingImpulse = addVW(oldImpulse, deltaImpulse)

				maxImpulse := mulW(c.rollingResistance, totalNormalImpulse)
				lengthSquared := dotW(cm.rollingImpulse, cm.rollingImpulse)

				// if ( magSqr > maxLambda * maxLambda + FLT_EPSILON )
				//{
				//	c->rollingImpulse *= maxLambda / sqrtf( magSqr );
				// }

				mask := greaterThanW(lengthSquared, mulAddW(epsilonW, maxImpulse, maxImpulse))

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
				normalize := divW(maxImpulse, addW(sqrtW(lengthSquared), epsilonW))
				scale := blendW(oneW, normalize, mask)

				// Blend the result to ensure there are no +/-0 determinism problems, since 0 * -1 == -0.
				scaledImpulse := mulSVW(scale, cm.rollingImpulse)
				cm.rollingImpulse.X = blendW(zeroW, scaledImpulse.X, cm.rollingMask)
				cm.rollingImpulse.Y = blendW(zeroW, scaledImpulse.Y, cm.rollingMask)
				cm.rollingImpulse.Z = blendW(zeroW, scaledImpulse.Z, cm.rollingMask)

				deltaImpulse = subVW(cm.rollingImpulse, oldImpulse)

				bA.w = mulSubMVW(bA.w, c.invIA, deltaImpulse)
				bB.w = mulAddMVW(bB.w, c.invIB, deltaImpulse)
			}

			// Central twist friction. Friction goes last to improve stacking stability.
			{
				twistSpeed := dotW(cm.normal, subVW(bB.w, bA.w))
				maxLambda := mulW(c.friction, totalTwistLimit)
				oldImpulse := cm.twistImpulse
				impulse := negW(mulW(cm.twistMass, twistSpeed))
				impulse = maxW(subW(negW(maxLambda), oldImpulse), minW(impulse, subW(maxLambda, oldImpulse)))
				cm.twistImpulse = addW(oldImpulse, impulse)

				bA.w = mulSubSVW(bA.w, impulse, mulMVW(c.invIA, cm.normal))
				bB.w = mulAddSVW(bB.w, impulse, mulMVW(c.invIB, cm.normal))
			}

			// Central friction. Friction goes last to improve stacking stability.
			{
				tangent1 := cm.tangent1
				tangent2 := cm.tangent2

				// Fixed anchor points for applying impulses
				rA := cm.centerA
				rB := cm.centerB

				angularRelativeVelocity := subVW(crossW(bB.w, rB), crossW(bA.w, rA))

				// Relative tangent velocity at contact
				vt := vec2W{
					addW(linearTangentVelocity1, dotW(angularRelativeVelocity, tangent1)),
					addW(linearTangentVelocity2, dotW(angularRelativeVelocity, tangent2)),
				}

				// Incremental tangent impulse
				deltaImpulse := mulMV2W(cm.tangentMass, vt)
				deltaImpulse = vec2W{negW(deltaImpulse.x), negW(deltaImpulse.y)}
				newImpulse := addV2W(cm.frictionImpulse, deltaImpulse)

				maxImpulse := mulW(c.friction, totalNormalImpulse)

				// Clamp the accumulated impulse
				lengthSquared := addW(mulW(newImpulse.x, newImpulse.x), mulW(newImpulse.y, newImpulse.y))

				// Max impulse can be zero
				mask := greaterThanW(lengthSquared, mulW(maxImpulse, maxImpulse))

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by
				// zero.
				normalize := divW(maxImpulse, addW(sqrtW(lengthSquared), epsilonW))
				scale := blendW(oneW, normalize, mask)
				newImpulse = vec2W{
					mulW(scale, newImpulse.x),
					mulW(scale, newImpulse.y),
				}

				deltaImpulse = vec2W{
					subW(newImpulse.x, cm.frictionImpulse.x),
					subW(newImpulse.y, cm.frictionImpulse.y),
				}

				cm.frictionImpulse = newImpulse

				// Apply delta impulse
				P := addVW(mulSVW(deltaImpulse.x, tangent1), mulSVW(deltaImpulse.y, tangent2))
				bA.w = mulSubMVW(bA.w, c.invIA, crossW(rA, P))
				bB.w = mulAddMVW(bB.w, c.invIB, crossW(rB, P))

				P = mulAddSVW(P, manifoldImpulse, cm.normal)
				bA.v = mulSubSVW(bA.v, c.invMassA, P)
				bB.v = mulAddSVW(bB.v, c.invMassB, P)
			}
		}

		scatterBodies(states, &c.indexA, &bA)
		scatterBodies(states, &c.indexB, &bB)

		ci, mi = nextConstraintW(ci, mi, manifoldCount)
	}
}

// applyRestitutionW is b3ApplyRestitutionW.
func applyRestitutionW(block solverBlock, context *stepContext, base wideConstraintBuffer, manifoldStarts []int32,
	fixedManifoldCount int,
) {
	states := context.states
	inv_h := splatW(context.inv_h)
	negRestitutionThreshold := splatW(-context.world.restitutionThreshold)
	zeroW := zeroW()
	propagate := context.world.enableRestitutionPropagation

	nextC, nextM := getConstraintW(base, manifoldStarts, int(block.startIndex), fixedManifoldCount)
	for i := 0; i < int(block.count); i++ {
		ci, mi := nextC, nextM
		c := &base.constraints[ci]
		manifoldCount := fixedManifoldCount
		if fixedManifoldCount <= 0 {
			manifoldCount = int(c.manifoldCount)
		}
		nextC, nextM = nextConstraintW(ci, mi, manifoldCount)

		if propagate == false && allZeroW(c.restitution) {
			continue
		}

		bA := gatherBodies(states, &c.indexA)
		bB := gatherBodies(states, &c.indexB)

		restitutionMask := greaterThanW(c.restitution, zeroW)
		dp := subVW(bB.dp, bA.dp)

		manifolds := getManifoldsW(base, mi)
		for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
			cm := &manifolds[manifoldIndex]

			normalSeparation := dotW(cm.normal, dp)
			normalA := invRotateVectorW(bA.dq, cm.normal)
			normalB := invRotateVectorW(bB.dq, cm.normal)

			// Read the point count rather than put on stack to avoid register spills.
			for pointIndex := 0; pointIndex < int(cm.pointCount); pointIndex++ {
				cp := &cm.points[pointIndex]

				rA := cp.anchorAs
				rB := cp.anchorBs

				s := addW(addW(normalSeparation, subW(dotW(normalB, rB), dotW(normalA, rA))),
					cp.baseSeparations)

				normalMass := cp.normalMasses
				if !propagate {
					normalMass = blendW(zeroW, cp.normalMasses, restitutionMask)
				}

				compressionImpulse := subW(cp.totalNormalImpulses, cp.restitutionImpulses)
				armed :=
					andW(andW(restitutionMask, lessThanW(cp.relativeVelocities, negRestitutionThreshold)),
						greaterThanW(compressionImpulse, zeroW))

				specBias := maxW(zeroW, mulW(s, inv_h))
				velocityBias := blendW(specBias, mulW(c.restitution, cp.relativeVelocities), armed)

				vrA := addVW(bA.v, crossW(bA.w, rA))
				vrB := addVW(bB.v, crossW(bB.w, rB))
				vn := dotW(subVW(vrB, vrA), cm.normal)

				negImpulse := mulW(normalMass, addW(vn, velocityBias))

				newImpulse := maxW(subW(cp.normalImpulses, negImpulse), zeroW)
				impulse := subW(newImpulse, cp.normalImpulses)

				approachImpulse :=
					minW(maxW(negW(mulW(normalMass, vn)), zeroW), maxW(impulse, zeroW))
				allowance :=
					subW(mulW(c.restitution, addW(compressionImpulse, approachImpulse)), cp.restitutionImpulses)
				maxImpulse := addW(approachImpulse, maxW(allowance, zeroW))
				impulse = blendW(impulse, minW(impulse, maxImpulse), armed)

				cp.normalImpulses = addW(cp.normalImpulses, impulse)
				cp.restitutionImpulses = addW(cp.restitutionImpulses, subW(impulse, approachImpulse))
				cp.totalNormalImpulses = addW(cp.totalNormalImpulses, impulse)

				P := mulSVW(impulse, cm.normal)
				bA.w = mulSubMVW(bA.w, c.invIA, crossW(rA, P))
				bA.v = mulSubSVW(bA.v, c.invMassA, P)
				bB.w = mulAddMVW(bB.w, c.invIB, crossW(rB, P))
				bB.v = mulAddSVW(bB.v, c.invMassB, P)
			}
		}

		scatterBodies(states, &c.indexA, &bA)
		scatterBodies(states, &c.indexB, &bB)
	}
}

// storeImpulsesW is b3StoreImpulsesW.
// Store impulses by contact constraint
func storeImpulsesW(block solverBlock, context *stepContext, workerIndex int, base wideConstraintBuffer,
	manifoldStarts []int32, fixedManifoldCount int,
) {
	world := context.world
	taskContext := &world.taskContexts[workerIndex]
	hitEventBitSet := &taskContext.hitEventBitSet
	hasHitEvents := taskContext.hasHitEvents
	negHitThreshold := -world.hitEventThreshold

	ci, mi := getConstraintW(base, manifoldStarts, int(block.startIndex), fixedManifoldCount)
	for i := 0; i < int(block.count); i++ {
		c := &base.constraints[ci]
		manifoldCount := fixedManifoldCount
		if fixedManifoldCount <= 0 {
			manifoldCount = int(c.manifoldCount)
		}
		manifolds := getManifoldsW(base, mi)

		// Pay the cache misses up front.
		var laneManifolds [laneCount][]Manifold
		for lane := range laneCount {
			contact := c.contacts[lane]
			if contact != nil {
				laneManifolds[lane] = contact.manifolds
			} else {
				laneManifolds[lane] = nil
			}
		}

		for lane := range laneCount {
			contact := c.contacts[lane]
			if contact == nil {
				continue
			}

			laneManifoldCount := fixedManifoldCount
			if fixedManifoldCount <= 0 {
				laneManifoldCount = int(contact.manifoldCount)
			}
			assert(laneManifoldCount == int(contact.manifoldCount) && laneManifoldCount <= manifoldCount,
				"laneManifoldCount == contact->manifoldCount && laneManifoldCount <= manifoldCount")

			for manifoldIndex := 0; manifoldIndex < laneManifoldCount; manifoldIndex++ {
				cm := &manifolds[manifoldIndex]
				m := &laneManifolds[lane][manifoldIndex]

				// PORT: the float pointer views of the wide fields are lane indexes.
				f1 := cm.frictionImpulse.x.v[lane]
				f2 := cm.frictionImpulse.y.v[lane]
				m.FrictionImpulse = Vec3{
					float32(f1*cm.tangent1.X.v[lane]) + float32(f2*cm.tangent2.X.v[lane]),
					float32(f1*cm.tangent1.Y.v[lane]) + float32(f2*cm.tangent2.Y.v[lane]),
					float32(f1*cm.tangent1.Z.v[lane]) + float32(f2*cm.tangent2.Z.v[lane]),
				}
				m.TwistImpulse = cm.twistImpulse.v[lane]
				m.RollingImpulse = Vec3{
					cm.rollingImpulse.X.v[lane],
					cm.rollingImpulse.Y.v[lane],
					cm.rollingImpulse.Z.v[lane],
				}

				pointCount := int(m.PointCount)
				assert(pointCount <= int(cm.pointCount), "pointCount <= cm->pointCount")
				for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
					cp := &cm.points[pointIndex]

					mp := &m.Points[pointIndex]
					mp.NormalImpulse = cp.normalImpulses.v[lane]
					mp.TotalNormalImpulse = cp.totalNormalImpulses.v[lane]
				}
			}

			if (contact.flags & simEnableHitEvent) != 0 {
				flagged := false
				for manifoldIndex := 0; manifoldIndex < laneManifoldCount && flagged == false; manifoldIndex++ {
					m := &contact.manifolds[manifoldIndex]
					for pointIndex := 0; pointIndex < int(m.PointCount); pointIndex++ {
						mp := &m.Points[pointIndex]

						// Need to check total impulse because the point may be speculative and not colliding
						if mp.NormalVelocity < negHitThreshold && mp.TotalNormalImpulse > 0.0 {
							setBit(hitEventBitSet, uint32(contact.contactId))
							hasHitEvents = true
							flagged = true
							break
						}
					}
				}
			}
		}

		ci, mi = nextConstraintW(ci, mi, manifoldCount)
	}

	taskContext.hasHitEvents = hasHitEvents
}

// These wrappers allow the implementation be shared between convex and mesh contacts.

// warmStartContactsConvexW is b3WarmStartContacts_ConvexW4.
func warmStartContactsConvexW(block solverBlock, context *stepContext) {
	warmStartContactsW(block, context, context.graph.colors[block.colorIndex].wideConstraints, nil, 1)
}

// pushContactsConvexW is b3PushContacts_ConvexW4.
func pushContactsConvexW(block solverBlock, context *stepContext) {
	pushContactsW(block, context, context.graph.colors[block.colorIndex].wideConstraints, nil, 1)
}

// solveContactsConvexW is b3SolveContacts_ConvexW4.
func solveContactsConvexW(block solverBlock, context *stepContext) {
	solveContactsW(block, context, context.graph.colors[block.colorIndex].wideConstraints, nil, 1)
}

// applyRestitutionConvexW is b3ApplyRestitution_ConvexW4.
func applyRestitutionConvexW(block solverBlock, context *stepContext) {
	applyRestitutionW(block, context, context.graph.colors[block.colorIndex].wideConstraints, nil, 1)
}

// storeImpulsesConvexW is b3StoreImpulses_ConvexW4.
func storeImpulsesConvexW(block solverBlock, context *stepContext, workerIndex int) {
	storeImpulsesW(block, context, workerIndex, context.wideConstraints, nil, 1)
}

// warmStartContactsMeshWideW is b3WarmStartContacts_MeshWideW4.
func warmStartContactsMeshWideW(block solverBlock, context *stepContext) {
	color := &context.graph.colors[block.colorIndex]
	warmStartContactsW(block, context, color.wideMeshConstraints, color.wideMeshManifoldStarts, 0)
}

// pushContactsMeshWideW is b3PushContacts_MeshWideW4.
func pushContactsMeshWideW(block solverBlock, context *stepContext) {
	color := &context.graph.colors[block.colorIndex]
	pushContactsW(block, context, color.wideMeshConstraints, color.wideMeshManifoldStarts, 0)
}

// solveContactsMeshWideW is b3SolveContacts_MeshWideW4.
func solveContactsMeshWideW(block solverBlock, context *stepContext) {
	color := &context.graph.colors[block.colorIndex]
	solveContactsW(block, context, color.wideMeshConstraints, color.wideMeshManifoldStarts, 0)
}

// applyRestitutionMeshWideW is b3ApplyRestitution_MeshWideW4.
func applyRestitutionMeshWideW(block solverBlock, context *stepContext) {
	color := &context.graph.colors[block.colorIndex]
	applyRestitutionW(block, context, color.wideMeshConstraints, color.wideMeshManifoldStarts, 0)
}

// storeImpulsesMeshWideW is b3StoreImpulses_MeshWideW4.
func storeImpulsesMeshWideW(block solverBlock, context *stepContext, workerIndex int) {
	storeImpulsesW(block, context, workerIndex, context.wideMeshConstraints, context.wideMeshManifoldStarts, 0)
}
