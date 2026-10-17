// Port of the B3_SIMD_WIDTH == 4 regions of src/contact_solver_wide.inl from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "math"

// PORT: upstream keeps the lane-count specific code of the wide contact solver inline, behind
// `#if B3_SIMD_WIDTH == 8` (and `#if defined( B3_SIMD_NONE )`). The Go port moves every such
// region into the helpers of this file, so contact_solver_wide.go and convex_manifold_wide.go hold
// no code that only works at one lane count. This file holds the width 4 side of each region; a
// width 8 build supplies the same helpers for 8 lanes.

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
		simdBody.v.X = setW(s1.linearVelocity.X, s2.linearVelocity.X, s3.linearVelocity.X, s4.linearVelocity.X)
		simdBody.v.Y = setW(s1.linearVelocity.Y, s2.linearVelocity.Y, s3.linearVelocity.Y, s4.linearVelocity.Y)
		simdBody.v.Z = setW(s1.linearVelocity.Z, s2.linearVelocity.Z, s3.linearVelocity.Z, s4.linearVelocity.Z)
		simdBody.w.X = setW(s1.angularVelocity.X, s2.angularVelocity.X, s3.angularVelocity.X, s4.angularVelocity.X)
		simdBody.w.Y = setW(s1.angularVelocity.Y, s2.angularVelocity.Y, s3.angularVelocity.Y, s4.angularVelocity.Y)
		simdBody.w.Z = setW(s1.angularVelocity.Z, s2.angularVelocity.Z, s3.angularVelocity.Z, s4.angularVelocity.Z)
		simdBody.dp.X = setW(s1.deltaPosition.X, s2.deltaPosition.X, s3.deltaPosition.X, s4.deltaPosition.X)
		simdBody.dp.Y = setW(s1.deltaPosition.Y, s2.deltaPosition.Y, s3.deltaPosition.Y, s4.deltaPosition.Y)
		simdBody.dp.Z = setW(s1.deltaPosition.Z, s2.deltaPosition.Z, s3.deltaPosition.Z, s4.deltaPosition.Z)
		simdBody.dq.V.X = setW(s1.deltaRotation.V.X, s2.deltaRotation.V.X, s3.deltaRotation.V.X, s4.deltaRotation.V.X)
		simdBody.dq.V.Y = setW(s1.deltaRotation.V.Y, s2.deltaRotation.V.Y, s3.deltaRotation.V.Y, s4.deltaRotation.V.Y)
		simdBody.dq.V.Z = setW(s1.deltaRotation.V.Z, s2.deltaRotation.V.Z, s3.deltaRotation.V.Z, s4.deltaRotation.V.Z)
		simdBody.dq.S = setW(s1.deltaRotation.S, s2.deltaRotation.S, s3.deltaRotation.S, s4.deltaRotation.S)

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
	state.linearVelocity = Vec3{laneW(v, 0), laneW(v, 1), laneW(v, 2)}
	state.padding1 = laneW(v, 3)
	state.angularVelocity = Vec3{laneW(w, 0), laneW(w, 1), laneW(w, 2)}
	state.padding2 = laneW(w, 3)
}

// scatterBodies is b3ScatterBodies.
// This writes only the velocities back to the solver bodies
// PORT: `#if defined( B3_SIMD_NONE )` becomes a branch on simdNone, as in gatherBodies; the else
// branch is the B3_SIMD_WIDTH == 4 variant.
func scatterBodies(states []bodyState, indices *[laneCount]int32, simdBody *bodyStateW) {
	if simdNone {
		index1 := indices[0] - 1
		if index1 != -1 && (states[index1].flags&dynamicFlag) != 0 {
			state := &states[index1]
			state.linearVelocity.X = laneW(simdBody.v.X, 0)
			state.linearVelocity.Y = laneW(simdBody.v.Y, 0)
			state.linearVelocity.Z = laneW(simdBody.v.Z, 0)
			state.angularVelocity.X = laneW(simdBody.w.X, 0)
			state.angularVelocity.Y = laneW(simdBody.w.Y, 0)
			state.angularVelocity.Z = laneW(simdBody.w.Z, 0)
		}

		index2 := indices[1] - 1
		if index2 != -1 && (states[index2].flags&dynamicFlag) != 0 {
			state := &states[index2]
			state.linearVelocity.X = laneW(simdBody.v.X, 1)
			state.linearVelocity.Y = laneW(simdBody.v.Y, 1)
			state.linearVelocity.Z = laneW(simdBody.v.Z, 1)
			state.angularVelocity.X = laneW(simdBody.w.X, 1)
			state.angularVelocity.Y = laneW(simdBody.w.Y, 1)
			state.angularVelocity.Z = laneW(simdBody.w.Z, 1)
		}

		index3 := indices[2] - 1
		if index3 != -1 && (states[index3].flags&dynamicFlag) != 0 {
			state := &states[index3]
			state.linearVelocity.X = laneW(simdBody.v.X, 2)
			state.linearVelocity.Y = laneW(simdBody.v.Y, 2)
			state.linearVelocity.Z = laneW(simdBody.v.Z, 2)
			state.angularVelocity.X = laneW(simdBody.w.X, 2)
			state.angularVelocity.Y = laneW(simdBody.w.Y, 2)
			state.angularVelocity.Z = laneW(simdBody.w.Z, 2)
		}

		index4 := indices[3] - 1
		if index4 != -1 && (states[index4].flags&dynamicFlag) != 0 {
			state := &states[index4]
			state.linearVelocity.X = laneW(simdBody.v.X, 3)
			state.linearVelocity.Y = laneW(simdBody.v.Y, 3)
			state.linearVelocity.Z = laneW(simdBody.v.Z, 3)
			state.angularVelocity.X = laneW(simdBody.w.X, 3)
			state.angularVelocity.Y = laneW(simdBody.w.Y, 3)
			state.angularVelocity.Z = laneW(simdBody.w.Z, 3)
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

// PORT: B3_GATHER_LANES( wide, lanes, field ) becomes one gather<Field>W helper per field it is
// used with in b3PrepareConstraintW; each holds the B3_SIMD_WIDTH == 4 form of the macro,
// b3SetW( lanes[0]->field, lanes[1]->field, lanes[2]->field, lanes[3]->field ).

// gatherInvMassW is B3_GATHER_LANES( wide, simLanes, invMass ).
func gatherInvMassW(simLanes *[laneCount]*bodySim) floatW {
	return setW(simLanes[0].invMass, simLanes[1].invMass, simLanes[2].invMass, simLanes[3].invMass)
}

// gatherFrictionW is B3_GATHER_LANES( wide, contactLanes, friction ).
func gatherFrictionW(contactLanes *[laneCount]*contact) floatW {
	return setW(contactLanes[0].friction, contactLanes[1].friction, contactLanes[2].friction, contactLanes[3].friction)
}

// gatherRollingResistanceW is B3_GATHER_LANES( wide, contactLanes, rollingResistance ).
func gatherRollingResistanceW(contactLanes *[laneCount]*contact) floatW {
	return setW(contactLanes[0].rollingResistance, contactLanes[1].rollingResistance,
		contactLanes[2].rollingResistance, contactLanes[3].rollingResistance)
}

// gatherRestitutionW is B3_GATHER_LANES( wide, contactLanes, restitution ).
func gatherRestitutionW(contactLanes *[laneCount]*contact) floatW {
	return setW(contactLanes[0].restitution, contactLanes[1].restitution, contactLanes[2].restitution,
		contactLanes[3].restitution)
}

// gatherTangentVelocityW is B3_GATHER_LANES( tangentVelocity.X, contactLanes, tangentVelocity.x )
// and the same for .y and .z.
func gatherTangentVelocityW(contactLanes *[laneCount]*contact) vec3W {
	var tangentVelocity vec3W
	tangentVelocity.X = setW(contactLanes[0].tangentVelocity.X, contactLanes[1].tangentVelocity.X,
		contactLanes[2].tangentVelocity.X, contactLanes[3].tangentVelocity.X)
	tangentVelocity.Y = setW(contactLanes[0].tangentVelocity.Y, contactLanes[1].tangentVelocity.Y,
		contactLanes[2].tangentVelocity.Y, contactLanes[3].tangentVelocity.Y)
	tangentVelocity.Z = setW(contactLanes[0].tangentVelocity.Z, contactLanes[1].tangentVelocity.Z,
		contactLanes[2].tangentVelocity.Z, contactLanes[3].tangentVelocity.Z)
	return tangentVelocity
}

// gatherInvInertiaW is b3GatherInvInertiaW.
// PORT: b3LoadW( i + k ) reads invInertiaWorld as the float array cx.x cx.y cx.z cy.x | cy.y cy.z
// cz.x cz.y | cz.z. This is the B3_SIMD_WIDTH == 4 branch of its `#if B3_SIMD_WIDTH == 8`.
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

// intsToFloatW is b3IntsToFloatW.
// PORT: the B3_SIMD_WIDTH == 4 branch of its `#if B3_SIMD_WIDTH == 8`.
func intsToFloatW(v *[laneCount]int32) floatW {
	return setW(float32(v[0]), float32(v[1]), float32(v[2]), float32(v[3]))
}

// gatherManifoldHeadW is the `const float* m0 … m3` block of b3PrepareConstraintW: the manifold
// normal, twist, friction and rolling impulses of each lane, transposed into wide values.
// PORT: b3LoadW( &manifold->normal.x + k ) reads normal.x normal.y normal.z twistImpulse |
// frictionImpulse.x .y .z rollingImpulse.x | rollingImpulse.y | rollingImpulse.z. This is the
// B3_SIMD_WIDTH == 4 branch of the block's `#if B3_SIMD_WIDTH == 8`; the out-parameters of
// b3TransposeW become return values.
func gatherManifoldHeadW(lanes *[laneCount]*Manifold) (normal vec3W, twistImpulse floatW, frictionImpulse, rollingImpulse vec3W) {
	m0 := lanes[0]
	m1 := lanes[1]
	m2 := lanes[2]
	m3 := lanes[3]
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
	return normal, twistImpulse, frictionImpulse, rollingImpulse
}

// gatherManifoldPointW is the `const float* p0 … p3` block of b3PrepareConstraintW: manifold point
// pointIndex of each lane, transposed into wide values.
// PORT: b3LoadW( (const float*)( manifold->points + pointIndex ) + k ) reads anchorA.x .y .z
// anchorB.x | anchorB.y .z separation normalImpulse. This is the B3_SIMD_WIDTH == 4 branch of the
// block's `#if B3_SIMD_WIDTH == 8`; the out-parameters of b3TransposeW become return values.
func gatherManifoldPointW(lanes *[laneCount]*Manifold, pointIndex int) (rA, rB vec3W, separation, normalImpulse floatW) {
	p0 := &lanes[0].Points[pointIndex]
	p1 := &lanes[1].Points[pointIndex]
	p2 := &lanes[2].Points[pointIndex]
	p3 := &lanes[3].Points[pointIndex]

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
	return rA, rB, separation, normalImpulse
}
