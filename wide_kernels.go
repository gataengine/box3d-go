// Go-only: the runtime dispatch table for the width 8 kernels of Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: upstream compiles the W8 kernels (wide8.c, B3_SIMD_HAS_WIDTH_8) next to the W4 ones and
// the dispatchers in contact_solver.c, convex_manifold.c and simd.c pick them when
// world->simdWidth == 8. Go has no per-file target flags, so the width 8 kernels live in a build
// that registers them here from its init when the CPU has AVX2. In every other build wide8 stays
// nil and nothing selects width 8 (detectSIMDWidth reports laneCount).

// wideKernels holds the width 8 kernels the dispatchers call when the world runs at width 8.
type wideKernels struct {
	prepareContactsConvex, warmStartContactsConvex, pushContactsConvex, solveContactsConvex,
	applyRestitutionConvex, prepareContactsMeshWide, warmStartContactsMeshWide,
	pushContactsMeshWide, solveContactsMeshWide, applyRestitutionMeshWide func(solverBlock, *stepContext)

	storeImpulsesConvex, storeImpulsesMeshWide func(solverBlock, *stepContext, int)

	computeSeparatingAxis func(hullA, hullB *HullData, xfB Transform, earlyReturn bool) axisQuery

	contactConstraintByteCount, meshConstraintByteCount, meshManifoldByteCount func() int
}

var wide8 *wideKernels // set by the x8 backend's init when AVX2 is present
