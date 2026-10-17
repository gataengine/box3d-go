// Port of the B3_SIMD_WIDTH == 8 regions of src/contact_solver_wide.inl and the AVX2 detection
// of src/simd.c from Box3D (https://github.com/erincatto/box3d) at commit
// efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build box3d_simd && goexperiment.simd && amd64 && !box3d_no8

package box3d

import (
	"simd/archsimd"
	"unsafe"
)

// PORT: the width 8 side of wide_lanes.go: each helper holds the `#if B3_SIMD_WIDTH == 8` branch
// of the region wide_lanes.go holds the width 4 side of. The generated width 8 kernels
// (contact_solver_wide8_gen.go) call them under the names tools/widegen gives the width 4
// helpers. Upstream's B3_SIMD_NONE branches have no width 8 form (simdNone8 is false).
// convex_manifold_wide.inl has no width specific region, and the mesh contact code uses
// b3FloatW4 at every width (mesh_contact.go stays on floatW).

// PORT: B3_SIMD_HAS_WIDTH_8 && b3HasAVX2() (simd.c, cpuid / xgetbv) becomes archsimd's CPU
// feature check; the width 8 kernels are registered in wide8 (wide_kernels.go) only when it
// holds, so detectSIMDWidth picks width 8 exactly where upstream's b3DetectSIMDWidth does.
// Nothing reads wide8 during package initialization: GetSIMDWidth detects the width on first use.
func init() {
	if !archsimd.X86.AVX() || !archsimd.X86.AVX2() {
		return
	}

	wide8 = &wideKernels{
		prepareContactsConvex:      avx8(prepareContactsConvexW8),
		warmStartContactsConvex:    avx8(warmStartContactsConvexW8),
		pushContactsConvex:         avx8(pushContactsConvexW8),
		solveContactsConvex:        avx8(solveContactsConvexW8),
		applyRestitutionConvex:     avx8(applyRestitutionConvexW8),
		prepareContactsMeshWide:    avx8(prepareContactsMeshWideW8),
		warmStartContactsMeshWide:  avx8(warmStartContactsMeshWideW8),
		pushContactsMeshWide:       avx8(pushContactsMeshWideW8),
		solveContactsMeshWide:      avx8(solveContactsMeshWideW8),
		applyRestitutionMeshWide:   avx8(applyRestitutionMeshWideW8),
		storeImpulsesConvex:        avx8Store(storeImpulsesConvexW8),
		storeImpulsesMeshWide:      avx8Store(storeImpulsesMeshWideW8),
		computeSeparatingAxis:      computeSeparatingAxisAVX8,
		contactConstraintByteCount: getWideContactConstraintByteCountW8,
		meshConstraintByteCount:    getWideMeshConstraintByteCountW8,
		meshManifoldByteCount:      getWideMeshManifoldByteCountW8,
	}
}

// PORT: C compilers end every function that uses 256 bit registers (the W8 kernels of wide8.c)
// with VZEROUPPER, so the SSE encoded scalar code that follows runs without the AVX to SSE
// transition penalty / false dependency on the upper halves. The Go compiler does not insert it
// and encodes its scalar float code with SSE, so each registered W8 kernel ends with
// archsimd.ClearAVXUpperBits (VZEROUPPER); without it the width 8 solver ran 3-5x slower than
// width 4 on Zen 3 and the Windows host.

// avx8 wraps a W8 solver kernel: kernel, then VZEROUPPER.
func avx8(kernel func(solverBlock, *stepContext)) func(solverBlock, *stepContext) {
	return func(block solverBlock, context *stepContext) {
		kernel(block, context)
		archsimd.ClearAVXUpperBits()
	}
}

// avx8Store wraps a W8 store impulses kernel: kernel, then VZEROUPPER.
func avx8Store(kernel func(solverBlock, *stepContext, int)) func(solverBlock, *stepContext, int) {
	return func(block solverBlock, context *stepContext, workerIndex int) {
		kernel(block, context, workerIndex)
		archsimd.ClearAVXUpperBits()
	}
}

// computeSeparatingAxisAVX8 is computeSeparatingAxisW8, then VZEROUPPER.
func computeSeparatingAxisAVX8(hullA, hullB *HullData, xfB Transform, earlyReturn bool) axisQuery {
	query := computeSeparatingAxisW8(hullA, hullB, xfB, earlyReturn)
	archsimd.ClearAVXUpperBits()
	return query
}

// gatherBodies8 is b3GatherBodies (B3_SIMD_WIDTH == 8).
// PORT: the int pointer is a pointer to the lane index array.
func gatherBodies8(states []bodyState, indices *[laneCount8]int32) bodyStateW8 {
	identity := &identityBodyState

	// Indices are 0 for null
	var p [laneCount8]*bodyState
	for lane := range laneCount8 {
		p[lane] = identity
		if indices[lane] != 0 {
			p[lane] = &states[indices[lane]-1]
		}
	}

	var s bodyStateW8
	s.v.X, s.v.Y, s.v.Z, _, s.w.X, s.w.Y, s.w.Z, _ = transposeW8(
		bodyStateLoadW8(p[0], 0), bodyStateLoadW8(p[1], 0), bodyStateLoadW8(p[2], 0), bodyStateLoadW8(p[3], 0),
		bodyStateLoadW8(p[4], 0), bodyStateLoadW8(p[5], 0), bodyStateLoadW8(p[6], 0), bodyStateLoadW8(p[7], 0))
	s.dp.X, s.dp.Y, s.dp.Z, _, s.dq.V.X, s.dq.V.Y, s.dq.V.Z, s.dq.S = transposeW8(
		bodyStateLoadW8(p[0], 8), bodyStateLoadW8(p[1], 8), bodyStateLoadW8(p[2], 8), bodyStateLoadW8(p[3], 8),
		bodyStateLoadW8(p[4], 8), bodyStateLoadW8(p[5], 8), bodyStateLoadW8(p[6], 8), bodyStateLoadW8(p[7], 8))
	return s
}

// The b3BodyState layout _Static_asserts of contact_solver_wide.inl, which bodyStateLoadW8 and
// storeBodyVelocity8 rely on: 64 bytes, the rows at 0, 16, 32 and 48.
const (
	_ uintptr = unsafe.Sizeof(bodyState{}) - 64
	_ uintptr = 64 - unsafe.Sizeof(bodyState{})
	_ uintptr = unsafe.Offsetof(bodyState{}.angularVelocity) - 16
	_ uintptr = 16 - unsafe.Offsetof(bodyState{}.angularVelocity)
	_ uintptr = unsafe.Offsetof(bodyState{}.deltaPosition) - 32
	_ uintptr = 32 - unsafe.Offsetof(bodyState{}.deltaPosition)
	_ uintptr = unsafe.Offsetof(bodyState{}.deltaRotation) - 48
	_ uintptr = 48 - unsafe.Offsetof(bodyState{}.deltaRotation)
)

// bodyStateLoadW8 is b3LoadW( (const float*)state + offset ) for offset 0 or 8: one 32 byte half
// of the 64 byte b3BodyState (velocity + pad, angular velocity + pad | delta position + flags,
// delta rotation).
// PORT: as upstream, one unaligned 256 bit load (VMOVDQU) of the state's floats through an
// unsafe view. Building the row with setW8 (scalar stores, then a vector load that cannot be
// store forwarded) made the width 8 solver several times slower than width 4 on x86.
func bodyStateLoadW8(state *bodyState, offset int) floatW8 {
	return floatW8{archsimd.LoadFloat32x8Array((*[laneCount8]float32)(unsafe.Add(unsafe.Pointer(state), 4*offset)))}
}

// storeBodyVelocity8 is b3StoreBodyVelocity (B3_SIMD_WIDTH == 8).
func storeBodyVelocity8(states []bodyState, index int32, vw floatW8) {
	// Indices are 0 for null
	if index == 0 {
		return
	}

	state := &states[index-1]
	flags := state.flags
	if (flags & dynamicFlag) == 0 {
		return
	}

	// PORT: b3StoreW( (float*)state, vw ) writes the two velocity rows including the padding lanes,
	// one unaligned 256 bit store through an unsafe view as bodyStateLoadW8.
	vw.v.StoreArray((*[laneCount8]float32)(unsafe.Pointer(state)))
}

// scatterBodies8 is b3ScatterBodies (B3_SIMD_WIDTH == 8).
// This writes only the velocities back to the solver bodies
func scatterBodies8(states []bodyState, indices *[laneCount8]int32, simdBody *bodyStateW8) {
	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	zero := zeroW8()
	vw1, vw2, vw3, vw4, vw5, vw6, vw7, vw8 := transposeW8(simdBody.v.X, simdBody.v.Y, simdBody.v.Z, zero,
		simdBody.w.X, simdBody.w.Y, simdBody.w.Z, zero)

	storeBodyVelocity8(states, indices[0], vw1)
	storeBodyVelocity8(states, indices[1], vw2)
	storeBodyVelocity8(states, indices[2], vw3)
	storeBodyVelocity8(states, indices[3], vw4)
	storeBodyVelocity8(states, indices[4], vw5)
	storeBodyVelocity8(states, indices[5], vw6)
	storeBodyVelocity8(states, indices[6], vw7)
	storeBodyVelocity8(states, indices[7], vw8)
}

// PORT: the B3_SIMD_WIDTH == 8 form of B3_GATHER_LANES( wide, lanes, field ),
// b3SetW( lanes[0]->field, …, lanes[7]->field ), one helper per field as in wide_lanes.go.

// gatherInvMassW8 is B3_GATHER_LANES( wide, simLanes, invMass ).
func gatherInvMassW8(simLanes *[laneCount8]*bodySim) floatW8 {
	return setW8(simLanes[0].invMass, simLanes[1].invMass, simLanes[2].invMass, simLanes[3].invMass,
		simLanes[4].invMass, simLanes[5].invMass, simLanes[6].invMass, simLanes[7].invMass)
}

// gatherFrictionW8 is B3_GATHER_LANES( wide, contactLanes, friction ).
func gatherFrictionW8(contactLanes *[laneCount8]*contact) floatW8 {
	return setW8(contactLanes[0].friction, contactLanes[1].friction, contactLanes[2].friction, contactLanes[3].friction,
		contactLanes[4].friction, contactLanes[5].friction, contactLanes[6].friction, contactLanes[7].friction)
}

// gatherRollingResistanceW8 is B3_GATHER_LANES( wide, contactLanes, rollingResistance ).
func gatherRollingResistanceW8(contactLanes *[laneCount8]*contact) floatW8 {
	return setW8(contactLanes[0].rollingResistance, contactLanes[1].rollingResistance,
		contactLanes[2].rollingResistance, contactLanes[3].rollingResistance,
		contactLanes[4].rollingResistance, contactLanes[5].rollingResistance,
		contactLanes[6].rollingResistance, contactLanes[7].rollingResistance)
}

// gatherRestitutionW8 is B3_GATHER_LANES( wide, contactLanes, restitution ).
func gatherRestitutionW8(contactLanes *[laneCount8]*contact) floatW8 {
	return setW8(contactLanes[0].restitution, contactLanes[1].restitution, contactLanes[2].restitution,
		contactLanes[3].restitution, contactLanes[4].restitution, contactLanes[5].restitution,
		contactLanes[6].restitution, contactLanes[7].restitution)
}

// gatherTangentVelocityW8 is B3_GATHER_LANES( tangentVelocity.X, contactLanes, tangentVelocity.x )
// and the same for .y and .z.
func gatherTangentVelocityW8(contactLanes *[laneCount8]*contact) vec3W8 {
	l := contactLanes
	var tangentVelocity vec3W8
	tangentVelocity.X = setW8(l[0].tangentVelocity.X, l[1].tangentVelocity.X, l[2].tangentVelocity.X, l[3].tangentVelocity.X,
		l[4].tangentVelocity.X, l[5].tangentVelocity.X, l[6].tangentVelocity.X, l[7].tangentVelocity.X)
	tangentVelocity.Y = setW8(l[0].tangentVelocity.Y, l[1].tangentVelocity.Y, l[2].tangentVelocity.Y, l[3].tangentVelocity.Y,
		l[4].tangentVelocity.Y, l[5].tangentVelocity.Y, l[6].tangentVelocity.Y, l[7].tangentVelocity.Y)
	tangentVelocity.Z = setW8(l[0].tangentVelocity.Z, l[1].tangentVelocity.Z, l[2].tangentVelocity.Z, l[3].tangentVelocity.Z,
		l[4].tangentVelocity.Z, l[5].tangentVelocity.Z, l[6].tangentVelocity.Z, l[7].tangentVelocity.Z)
	return tangentVelocity
}

// matrixLoadW8 is b3LoadW( &m->cx.x ): the first eight floats cx.x cx.y cx.z cy.x cy.y cy.z cz.x
// cz.y of a b3Matrix3.
// PORT: Go-only helper replacing the float pointer view of b3Matrix3.
func matrixLoadW8(m *Matrix3) floatW8 {
	return setW8(m.Cx.X, m.Cx.Y, m.Cx.Z, m.Cy.X, m.Cy.Y, m.Cy.Z, m.Cz.X, m.Cz.Y)
}

// gatherInvInertiaW8 is b3GatherInvInertiaW (B3_SIMD_WIDTH == 8).
func gatherInvInertiaW8(simLanes *[laneCount8]*bodySim) symMatrix3W8 {
	i0 := &simLanes[0].invInertiaWorld
	i1 := &simLanes[1].invInertiaWorld
	i2 := &simLanes[2].invInertiaWorld
	i3 := &simLanes[3].invInertiaWorld
	i4 := &simLanes[4].invInertiaWorld
	i5 := &simLanes[5].invInertiaWorld
	i6 := &simLanes[6].invInertiaWorld
	i7 := &simLanes[7].invInertiaWorld

	var m symMatrix3W8
	m.cxx, m.cxy, m.cxz, _, m.cyy, m.cyz, _, _ = transposeW8(
		matrixLoadW8(i0), matrixLoadW8(i1), matrixLoadW8(i2), matrixLoadW8(i3),
		matrixLoadW8(i4), matrixLoadW8(i5), matrixLoadW8(i6), matrixLoadW8(i7))
	m.czz = setW8(i0.Cz.Z, i1.Cz.Z, i2.Cz.Z, i3.Cz.Z, i4.Cz.Z, i5.Cz.Z, i6.Cz.Z, i7.Cz.Z)
	return m
}

// intsToFloatW8 is b3IntsToFloatW (B3_SIMD_WIDTH == 8).
func intsToFloatW8(v *[laneCount8]int32) floatW8 {
	return setW8(float32(v[0]), float32(v[1]), float32(v[2]), float32(v[3]),
		float32(v[4]), float32(v[5]), float32(v[6]), float32(v[7]))
}

// gatherManifoldHeadW8 is the B3_SIMD_WIDTH == 8 branch of the `const float* m0 … m7` block of
// b3PrepareConstraintW (see gatherManifoldHeadW).
// PORT: b3LoadW( &manifold->normal.x ) reads normal.x normal.y normal.z twistImpulse
// frictionImpulse.x .y .z rollingImpulse.x; m[8] and m[9] are rollingImpulse.y and .z. The
// out-parameters of b3TransposeW become return values.
func gatherManifoldHeadW8(lanes *[laneCount8]*Manifold) (normal vec3W8, twistImpulse floatW8, frictionImpulse, rollingImpulse vec3W8) {
	var head [laneCount8]floatW8
	for lane, m := range lanes {
		head[lane] = setW8(m.Normal.X, m.Normal.Y, m.Normal.Z, m.TwistImpulse,
			m.FrictionImpulse.X, m.FrictionImpulse.Y, m.FrictionImpulse.Z, m.RollingImpulse.X)
	}
	normal.X, normal.Y, normal.Z, twistImpulse, frictionImpulse.X, frictionImpulse.Y, frictionImpulse.Z, rollingImpulse.X =
		transposeW8(head[0], head[1], head[2], head[3], head[4], head[5], head[6], head[7])
	m0, m1, m2, m3, m4, m5, m6, m7 := lanes[0], lanes[1], lanes[2], lanes[3], lanes[4], lanes[5], lanes[6], lanes[7]
	rollingImpulse.Y = setW8(m0.RollingImpulse.Y, m1.RollingImpulse.Y, m2.RollingImpulse.Y, m3.RollingImpulse.Y,
		m4.RollingImpulse.Y, m5.RollingImpulse.Y, m6.RollingImpulse.Y, m7.RollingImpulse.Y)
	rollingImpulse.Z = setW8(m0.RollingImpulse.Z, m1.RollingImpulse.Z, m2.RollingImpulse.Z, m3.RollingImpulse.Z,
		m4.RollingImpulse.Z, m5.RollingImpulse.Z, m6.RollingImpulse.Z, m7.RollingImpulse.Z)
	return normal, twistImpulse, frictionImpulse, rollingImpulse
}

// gatherManifoldPointW8 is the B3_SIMD_WIDTH == 8 branch of the `const float* p0 … p7` block of
// b3PrepareConstraintW (see gatherManifoldPointW).
// PORT: b3LoadW( (const float*)( manifold->points + pointIndex ) ) reads anchorA.x .y .z
// anchorB.x .y .z separation normalImpulse. The out-parameters of b3TransposeW become return
// values.
func gatherManifoldPointW8(lanes *[laneCount8]*Manifold, pointIndex int) (rA, rB vec3W8, separation, normalImpulse floatW8) {
	var rows [laneCount8]floatW8
	for lane, m := range lanes {
		p := &m.Points[pointIndex]
		rows[lane] = setW8(p.AnchorA.X, p.AnchorA.Y, p.AnchorA.Z, p.AnchorB.X, p.AnchorB.Y, p.AnchorB.Z,
			p.Separation, p.NormalImpulse)
	}
	rA.X, rA.Y, rA.Z, rB.X, rB.Y, rB.Z, separation, normalImpulse =
		transposeW8(rows[0], rows[1], rows[2], rows[3], rows[4], rows[5], rows[6], rows[7])
	return rA, rB, separation, normalImpulse
}
