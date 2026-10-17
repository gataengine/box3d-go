// Port of test/test_hull.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"bytes"

	. "github.com/onsi/ginkgo/v2"
)

var cubeCornersTest = [8]Vec3{
	{1.0, 1.0, 1.0}, {-1.0, 1.0, 1.0}, {-1.0, -1.0, 1.0}, {1.0, -1.0, 1.0},
	{1.0, 1.0, -1.0}, {-1.0, 1.0, -1.0}, {-1.0, -1.0, -1.0}, {1.0, -1.0, -1.0},
}

const sphereN = 6 // SPHERE_N

// fillSphereSample is FillSphereSample.
// Inlined XorShift32 + Shoemake unit-vector recipe matches shared/utils.h exactly so
// generated points are bit-identical to samples that share the seed.
// PORT: sinf/cosf come from float64 math (see math_test.go).
func fillSphereSample(points []Vec3, count int, seed uint32) {
	const randLimitLocal uint32 = 32767
	for i := range count {
		var u [3]float32
		for k := range 3 {
			seed ^= seed << 13
			seed ^= seed >> 17
			seed ^= seed << 5
			r := float32(seed & randLimitLocal)
			r /= float32(randLimitLocal)
			u[k] = r
		}
		u1 := u[0]
		u2 := float32(2.0 * Pi * u[1])
		u3 := float32(2.0 * Pi * u[2])
		sqrt1MinusU1 := sqrtf(1.0 - u1)
		sqrtU1 := sqrtf(u1)
		points[i].X = sqrt1MinusU1 * sinf(u2)
		points[i].Y = sqrt1MinusU1 * cosf(u2)
		points[i].Z = sqrtU1 * sinf(u3)
	}
}

// fillCubeSample is FillCubeSample: XorShift32 + uniform-cube point generator. Same engine as FillSphereSample.
func fillCubeSample(points []Vec3, count int, seed uint32) {
	const randLimitLocal uint32 = 32767
	for i := range count {
		var v [3]float32
		for k := range 3 {
			seed ^= seed << 13
			seed ^= seed >> 17
			seed ^= seed << 5
			r := float32(seed & randLimitLocal)
			r /= float32(randLimitLocal)
			v[k] = float32(2.0*r) - 1.0
		}
		points[i].X = v[0]
		points[i].Y = v[1]
		points[i].Z = v[2]
	}
}

// exactQuat is ExactQuat.
// b3ComputeCosSin is a coarse approximation, so build rotations from libm to keep the
// analytic corner and plane positions exact.
func exactQuat(axis Vec3, radians float32) Quat {
	half := float32(0.5 * radians)
	s := sinf(half)
	return Quat{Vec3{float32(s * axis.X), float32(s * axis.Y), float32(s * axis.Z)}, cosf(half)}
}

// boxCornerSignsTest is s_boxCornerSigns: corner sign pattern baked by MakeTransformedBoxHull, in order.
var boxCornerSignsTest = [8]Vec3{
	{1.0, 1.0, 1.0}, {-1.0, 1.0, 1.0}, {-1.0, -1.0, 1.0}, {1.0, -1.0, 1.0},
	{1.0, 1.0, -1.0}, {-1.0, 1.0, -1.0}, {-1.0, -1.0, -1.0}, {1.0, -1.0, -1.0},
}

// checkTransformedBox is CheckTransformedBox.
// The baked box hull is only exercised elsewhere through mass properties, which are analytic
// and never read boxPoints or boxPlanes. Pin the geometry itself against the transform so an
// axis swap in the point or plane bake cannot pass silently.
func checkTransformedBox(h Vec3, xf Transform) {
	GinkgoHelper()
	box := MakeTransformedBoxHull(h.X, h.Y, h.Z, xf)

	const tol float32 = 1e-5

	// Each corner is the transform of the signed half extent.
	for i := range 8 {
		local := Vec3{boxCornerSignsTest[i].X * h.X, boxCornerSignsTest[i].Y * h.Y, boxCornerSignsTest[i].Z * h.Z}
		expected := TransformPoint(xf, local)
		ensureSmall(box.BoxPoints[i].X-expected.X, tol)
		ensureSmall(box.BoxPoints[i].Y-expected.Y, tol)
		ensureSmall(box.BoxPoints[i].Z-expected.Z, tol)
	}

	// Face normals rotate with the box, offsets carry the rotated normal through the translation.
	localNormals := [6]Vec3{
		Neg(Vec3AxisX), Vec3AxisX, Neg(Vec3AxisY), Vec3AxisY, Neg(Vec3AxisZ), Vec3AxisZ,
	}
	localOffsets := [6]float32{h.X, h.X, h.Y, h.Y, h.Z, h.Z}
	for i := range 6 {
		n := RotateVector(xf.Q, localNormals[i])
		offset := localOffsets[i] + Dot(n, xf.P)
		ensureSmall(box.BoxPlanes[i].Normal.X-n.X, tol)
		ensureSmall(box.BoxPlanes[i].Normal.Y-n.Y, tol)
		ensureSmall(box.BoxPlanes[i].Normal.Z-n.Z, tol)
		ensureSmall(box.BoxPlanes[i].Offset-offset, tol)
	}

	// SoA vertex mirrors match the AoS points on all eight lanes.
	for i := range 8 {
		ensureSmall(box.Vx[i]-box.BoxPoints[i].X, tol)
		ensureSmall(box.Vy[i]-box.BoxPoints[i].Y, tol)
		ensureSmall(box.Vz[i]-box.BoxPoints[i].Z, tol)
	}

	// SoA normal mirrors carry the six real faces and zero the two pad lanes.
	for i := range 6 {
		ensureSmall(box.Nx[i]-box.BoxPlanes[i].Normal.X, tol)
		ensureSmall(box.Ny[i]-box.BoxPlanes[i].Normal.Y, tol)
		ensureSmall(box.Nz[i]-box.BoxPlanes[i].Normal.Z, tol)
	}
	ensure(box.Nx[6] == 0.0 && box.Nx[7] == 0.0)
	ensure(box.Ny[6] == 0.0 && box.Ny[7] == 0.0)
	ensure(box.Nz[6] == 0.0 && box.Nz[7] == 0.0)

	// The stored AABB bounds every baked corner.
	pointBox := MakeAABB(box.BoxPoints[:8], 0.0)
	ensure(box.Base.AABB.Contains(pointBox))
}

// fillPoints2D is FillPoints2D.
func fillPoints2D(pts []Point2D, input []Vec2, count int) {
	for i := range count {
		pts[i].P = input[i]
		pts[i].Separation = 0.0
		pts[i].OriginalIndex = int32(i)
	}
}

// hull2DArea is Hull2DArea.
func hull2DArea(hull []Point2D, count int) float32 {
	var sum float32 = 0.0
	for i := range count {
		next := (i + 1) % count
		sum += float32(hull[i].P.X*hull[next].P.Y) - float32(hull[i].P.Y*hull[next].P.X)
	}

	return 0.5 * sum
}

var _ = Describe("HullTest", func() {
	It("CreateHullCubeTest", func() {
		hull := CreateHull(cubeCornersTest[:], 8, 8)
		ensure(hull != nil)

		ensure(hull.VertexCount == 8)
		ensure(hull.EdgeCount == 24)
		ensure(hull.FaceCount == 6)

		// Euler's identity for convex polyhedron
		ensure(hull.VertexCount-hull.EdgeCount/2+hull.FaceCount == 2)

		ref := MakeBoxHull(1.0, 1.0, 1.0)

		ensureSmall(hull.Volume-ref.Base.Volume, 1e-4)
		ensureSmall(hull.SurfaceArea-ref.Base.SurfaceArea, 1e-4)
		ensureSmall(hull.InnerRadius-ref.Base.InnerRadius, fltEpsilon)

		ensureSmall(hull.Center.X-ref.Base.Center.X, 1e-5)
		ensureSmall(hull.Center.Y-ref.Base.Center.Y, 1e-5)
		ensureSmall(hull.Center.Z-ref.Base.Center.Z, 1e-5)

		ensureSmall(hull.AABB.LowerBound.X+1.0, fltEpsilon)
		ensureSmall(hull.AABB.LowerBound.Y+1.0, fltEpsilon)
		ensureSmall(hull.AABB.LowerBound.Z+1.0, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.X-1.0, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.Y-1.0, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.Z-1.0, fltEpsilon)

		d := SubMM(hull.CentralInertia, ref.Base.CentralInertia)
		ensureSmall(d.Cx.X, 1e-4)
		ensureSmall(d.Cy.Y, 1e-4)
		ensureSmall(d.Cz.Z, 1e-4)
		ensureSmall(d.Cx.Y, 1e-4)
		ensureSmall(d.Cx.Z, 1e-4)
		ensureSmall(d.Cy.Z, 1e-4)
		ensureSmall(d.Cy.X, 1e-4)
		ensureSmall(d.Cz.X, 1e-4)
		ensureSmall(d.Cz.Y, 1e-4)

		DestroyHull(hull)
	})

	It("CreateHullTetrahedronTest", func() {
		points := [4]Vec3{
			{0.0, 0.0, 0.0},
			{1.0, 0.0, 0.0},
			{0.0, 1.0, 0.0},
			{0.0, 0.0, 1.0},
		}

		hull := CreateHull(points[:], 4, 4)
		ensure(hull != nil)

		ensure(hull.VertexCount == 4)
		ensure(hull.EdgeCount == 12)
		ensure(hull.FaceCount == 4)
		ensure(hull.VertexCount-hull.EdgeCount/2+hull.FaceCount == 2)

		// Analytic values for the unit-corner tetrahedron at the origin.
		var expectedVolume float32 = 1.0 / 6.0
		expectedSurfaceArea := 1.5 + float32(0.5*sqrtf(3.0))
		expectedInnerRadius := 0.25 / sqrtf(3.0)

		ensureSmall(hull.Volume-expectedVolume, 1e-5)
		ensureSmall(hull.SurfaceArea-expectedSurfaceArea, 1e-5)
		ensureSmall(hull.InnerRadius-expectedInnerRadius, 1e-5)

		ensureSmall(hull.Center.X-0.25, 1e-5)
		ensureSmall(hull.Center.Y-0.25, 1e-5)
		ensureSmall(hull.Center.Z-0.25, 1e-5)

		ensureSmall(hull.AABB.LowerBound.X, fltEpsilon)
		ensureSmall(hull.AABB.LowerBound.Y, fltEpsilon)
		ensureSmall(hull.AABB.LowerBound.Z, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.X-1.0, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.Y-1.0, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.Z-1.0, fltEpsilon)

		DestroyHull(hull)
	})

	It("CreateHullDeterminismTest", func() {
		h1 := CreateHull(cubeCornersTest[:], 8, 8)
		h2 := CreateHull(cubeCornersTest[:], 8, 8)
		ensure(h1 != nil && h2 != nil)

		ensure(h1.ByteCount == h2.ByteCount)
		ensure(h1.Hash != 0)
		ensure(h1.Hash == h2.Hash)
		// PORT: memcmp( h1, h2, h1->byteCount ) compares the byte images.
		ensure(bytes.Equal(hullBytes(h1), hullBytes(h2)))

		DestroyHull(h1)
		DestroyHull(h2)
	})

	It("CreateHullMaxVertexTest", func() {
		// Sphere-sampled point cloud, dense enough that the builder has room to grow.
		var points [sphereN * sphereN]Vec3
		index := 0
		for i := range sphereN {
			theta := Pi * float32(i) / float32(sphereN-1)
			for j := range sphereN {
				phi := float32(2.0*Pi) * float32(j) / float32(sphereN)
				points[index].X = sinf(theta) * cosf(phi)
				points[index].Y = sinf(theta) * sinf(phi)
				points[index].Z = cosf(theta)
				index += 1
			}
		}
		count := sphereN * sphereN

		// maxVertexCount honored as a strict cap.
		h1 := CreateHull(points[:], count, 8)
		ensure(h1 != nil)
		ensure(h1.VertexCount <= 8)
		DestroyHull(h1)

		// Below the floor: clamps up to 4.
		h2 := CreateHull(points[:], count, 1)
		ensure(h2 != nil)
		ensure(h2.VertexCount >= 4 && h2.VertexCount <= MaxHullVertices)
		DestroyHull(h2)

		// Above the ceiling: clamps down to B3_MAX_HULL_VERTICES.
		h3 := CreateHull(points[:], count, 1000)
		ensure(h3 != nil)
		ensure(h3.VertexCount >= 4 && h3.VertexCount <= MaxHullVertices)
		DestroyHull(h3)
	})

	It("CreateHullRedundantInputTest", func() {
		// 8 cube corners + duplicates + interior points. Builder should produce the cube.
		points := [20]Vec3{
			{1.0, 1.0, 1.0}, // corners
			{-1.0, 1.0, 1.0},
			{-1.0, -1.0, 1.0},
			{1.0, -1.0, 1.0},
			{1.0, 1.0, -1.0},
			{-1.0, 1.0, -1.0},
			{-1.0, -1.0, -1.0},
			{1.0, -1.0, -1.0},
			{1.0, 1.0, 1.0}, // duplicates
			{1.0, 1.0, 1.0},
			{0.0, 0.0, 0.0}, // interior points
			{0.5, 0.0, 0.0},
			{0.0, 0.5, 0.0},
			{0.0, 0.0, 0.5},
			{-0.5, 0.0, 0.0},
			{0.0, -0.5, 0.0},
			{0.0, 0.0, -0.5},
			{0.25, 0.25, 0.25},
			{-0.25, -0.25, -0.25},
			{0.5, 0.5, 0.5},
		}

		hull := CreateHull(points[:], 20, 8)
		ensure(hull != nil)

		ensure(hull.VertexCount == 8)
		ensure(hull.EdgeCount == 24)
		ensure(hull.FaceCount == 6)

		ref := MakeBoxHull(1.0, 1.0, 1.0)

		ensureSmall(hull.Volume-ref.Base.Volume, 1e-4)
		ensureSmall(hull.SurfaceArea-ref.Base.SurfaceArea, 1e-4)
		ensureSmall(hull.InnerRadius-ref.Base.InnerRadius, fltEpsilon)

		ensureSmall(hull.Center.X-ref.Base.Center.X, 1e-5)
		ensureSmall(hull.Center.Y-ref.Base.Center.Y, 1e-5)
		ensureSmall(hull.Center.Z-ref.Base.Center.Z, 1e-5)

		ensureSmall(hull.AABB.LowerBound.X+1.0, fltEpsilon)
		ensureSmall(hull.AABB.LowerBound.Y+1.0, fltEpsilon)
		ensureSmall(hull.AABB.LowerBound.Z+1.0, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.X-1.0, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.Y-1.0, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.Z-1.0, fltEpsilon)

		DestroyHull(hull)
	})

	It("CreateHullCloneTest", func() {
		original := CreateHull(cubeCornersTest[:], 8, 8)
		ensure(original != nil)

		clone := CloneHull(original)
		ensure(clone != nil)
		ensure(clone.ByteCount == original.ByteCount)
		ensure(bytes.Equal(hullBytes(clone), hullBytes(original)))

		DestroyHull(clone)
		DestroyHull(original)
	})

	It("CreateHullCylinderTest", func() {
		const height float32 = 2.0
		const radius float32 = 1.0
		const sides = 8
		const yOffset float32 = 0.0

		hull := CreateCylinder(height, radius, yOffset, sides)
		ensure(hull != nil)

		ensure(hull.VertexCount == 2*sides)
		ensure(hull.EdgeCount == 6*sides)
		ensure(hull.FaceCount == sides+2)

		// Analytic n-gon prism values (exact targets, not the circular cylinder approximations).
		halfAngle := Pi / float32(sides)
		capArea := float32(sides) * 0.5 * radius * radius * sinf(2.0*halfAngle)
		chordLen := 2.0 * radius * sinf(halfAngle)
		lateralArea := float32(sides) * chordLen * height

		expectedVolume := capArea * height
		expectedSurfaceArea := float32(2.0*capArea) + lateralArea
		expectedInnerRadius := radius * cosf(halfAngle)

		ensureSmall((hull.Volume-expectedVolume)/expectedVolume, 1e-4)
		ensureSmall((hull.SurfaceArea-expectedSurfaceArea)/expectedSurfaceArea, 1e-4)
		ensureSmall(hull.InnerRadius-expectedInnerRadius, 1e-5)

		ensureSmall(hull.Center.X, 1e-5)
		ensureSmall(hull.Center.Y-(yOffset+float32(0.5*height)), 1e-5)
		ensureSmall(hull.Center.Z, 1e-5)

		ensureSmall(hull.AABB.LowerBound.X+radius, fltEpsilon)
		ensureSmall(hull.AABB.LowerBound.Y-yOffset, fltEpsilon)
		ensureSmall(hull.AABB.LowerBound.Z+radius, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.X-radius, fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.Y-(yOffset+height), fltEpsilon)
		ensureSmall(hull.AABB.UpperBound.Z-radius, fltEpsilon)

		DestroyHull(hull)
	})

	// Reproduces the HullReduction sample (Sphere, 64 points, count=20) that used to assert on
	// b->faceCount < b->faceCapacity. The free-list reclaim in NewFace/NewEdge keeps the bump
	// counts proportional to the live hull instead of cumulative creations.
	It("CreateHullSphereReductionTest", func() {
		var points [64]Vec3
		fillSphereSample(points[:], 64, 12345) // RAND_SEED

		hull := CreateHull(points[:], 64, 20)
		ensure(hull != nil)
		ensure(hull.VertexCount >= 4 && hull.VertexCount <= 20)
		ensure(hull.VertexCount-hull.EdgeCount/2+hull.FaceCount == 2)
		DestroyHull(hull)
	})

	// Pushes the working-stage bump capacities (faceCapacity = 5*M - 10, edgeCapacity =
	// 24*M - 48 in src/hull.c::b3ComputeHullWorkSizes) close to their peak by sweeping M
	// on a dense random sphere. If the peak face/edge count exceeds the cap and the free
	// list fails to reclaim slots in time, the assertion at src/hull.c::NewFace/NewEdge
	// fires (exit code 3). A dense sphere hull is fully triangulated, so F = 2M - 4 and the
	// final face limit B3_MAX_HULL_FACES binds well before the vertex or edge limits.
	It("CreateHullSphereStressTest", func() {
		const N = 512
		var points [N]Vec3

		// Multiple seeds exercise different horizon-size / merge-cascade sequences.
		seeds := []uint32{12345, 1, 0xdeadbeef, 0xcafef00d}

		// M kept <= 32 so the final face count 2M - 4 stays under B3_MAX_HULL_FACES
		mValues := []int{16, 24, 32}

		for s := range arrayCount(seeds) {
			fillSphereSample(points[:], N, seeds[s])

			for m := range arrayCount(mValues) {
				M := mValues[m]
				hull := CreateHull(points[:], N, M)
				ensure(hull != nil)
				ensure(hull.VertexCount >= 4 && int(hull.VertexCount) <= M)
				ensure(hull.VertexCount-hull.EdgeCount/2+hull.FaceCount == 2)
				ensure(hull.FaceCount >= 4)
				DestroyHull(hull)
			}
		}
	})

	// Random points inside a cube produce a small final hull (8 corners, 6 faces) but
	// generate heavy internal churn: every interior point gets fed to the conflict-list
	// machinery, and most cone faces created during apex insertion are then merged out
	// when their newly-created neighbors are coplanar. Exercises the merge cascade in
	// b3HullBuilder_ConnectEdges/ConnectFaces and the corresponding RetireFace/RetireEdge
	// reclaim path much harder than the sphere case (which has few merges).
	It("CreateHullMergeChurnStressTest", func() {
		const N = 4096
		points := make([]Vec3, N)

		seeds := []uint32{12345, 0xdeadbeef}

		for s := range arrayCount(seeds) {
			fillCubeSample(points, N, seeds[s])

			// Stamp the 8 corners last so they're guaranteed extremes.
			for c := range 8 {
				points[N-8+c].X = -1.0
				if c&1 != 0 {
					points[N-8+c].X = 1.0
				}
				points[N-8+c].Y = -1.0
				if c&2 != 0 {
					points[N-8+c].Y = 1.0
				}
				points[N-8+c].Z = -1.0
				if c&4 != 0 {
					points[N-8+c].Z = 1.0
				}
			}

			hull := CreateHull(points, N, 64)
			ensure(hull != nil)
			ensure(hull.VertexCount == 8)
			ensure(hull.EdgeCount == 24)
			ensure(hull.FaceCount == 6)
			DestroyHull(hull)
		}
	})

	It("CreateHullDegenerateTest", func() {
		// Real (non-null) buffer; pointCount < 4 cases are guarded inside Construct().
		collinear := [8]Vec3{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0},
			{4.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, {7.0, 0.0, 0.0},
		}

		// Empty input.
		ensure(CreateHull(collinear[:], 0, 8) == nil)

		// Fewer than 4 points.
		ensure(CreateHull(collinear[:], 3, 8) == nil)

		// 8 coincident points.
		coincident := [8]Vec3{
			{1.0, 2.0, 3.0}, {1.0, 2.0, 3.0}, {1.0, 2.0, 3.0}, {1.0, 2.0, 3.0},
			{1.0, 2.0, 3.0}, {1.0, 2.0, 3.0}, {1.0, 2.0, 3.0}, {1.0, 2.0, 3.0},
		}
		ensure(CreateHull(coincident[:], 8, 8) == nil)

		// Collinear (along x-axis).
		ensure(CreateHull(collinear[:], 8, 8) == nil)

		// Coplanar (in the xy-plane).
		coplanar := [6]Vec3{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0},
			{1.0, 1.0, 0.0}, {2.0, 0.5, 0.0}, {0.5, 2.0, 0.0},
		}
		ensure(CreateHull(coplanar[:], 6, 8) == nil)
	})

	It("TransformedBoxHullTest", func() {
		h := Vec3{0.25, 0.5, 0.3}

		// Identity.
		checkTransformedBox(h, TransformIdentity)

		// Translation only.
		translated := Transform{Vec3{0.4, -0.7, 0.1}, QuatIdentity}
		checkTransformedBox(h, translated)

		// Rotation only. A quarter turn about Y swaps the world X and Z extents.
		rotated := Transform{Vec3Zero, exactQuat(Vec3AxisY, 0.25*Pi)}
		checkTransformedBox(h, rotated)

		// Translation and rotation together.
		transformed := Transform{Vec3{3.0, -2.0, 1.5}, exactQuat(Vec3AxisZ, 0.25*Pi)}
		checkTransformedBox(h, transformed)
	})

	// Interior points contribute no area, so they must not survive the hull.
	It("Hull2DSquareTest", func() {
		input := [8]Vec2{
			{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0},
			{0.0, 0.0}, {0.5, 0.25}, {-0.3, 0.6}, {0.1, -0.4},
		}

		var pts [8]Point2D
		var hull [16]Point2D
		fillPoints2D(pts[:], input[:], 8)

		count := Hull2D(pts[:], 8, hull[:])
		ensure(count == 4)

		// The reduction code assumes counter clockwise winding
		ensure(hull2DArea(hull[:], count) > 0.0)

		var onHull [8]bool
		for i := range count {
			ensure(0 <= hull[i].OriginalIndex && hull[i].OriginalIndex < 8)
			onHull[hull[i].OriginalIndex] = true
		}

		for i := range 4 {
			ensure(onHull[i] == true)
		}

		for i := 4; i < 8; i++ {
			ensure(onHull[i] == false)
		}
	})

	It("Hull2DCollinearTest", func() {
		input := [5]Vec2{
			{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0},
		}

		var pts [5]Point2D
		var hull [10]Point2D
		fillPoints2D(pts[:], input[:], 5)

		ensure(Hull2D(pts[:], 5, hull[:]) == 2)
	})

	// Coincident points weld and the lower input index wins, so a sort order change cannot
	// swap which of the two survives.
	It("Hull2DWeldTest", func() {
		offset := float32(0.1 * LinearSlop())

		input := [5]Vec2{
			{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}, {-1.0 + offset, -1.0 + offset},
		}

		var pts [5]Point2D
		var hull [10]Point2D
		fillPoints2D(pts[:], input[:], 5)

		count := Hull2D(pts[:], 5, hull[:])
		ensure(count == 4)

		for i := range count {
			ensure(hull[i].OriginalIndex != 4)
		}
	})

	// The surviving point set must not depend on the order the points arrive in. The reduction
	// runs inside the solver, so an input order dependence would show up as a determinism bug.
	It("Hull2DOrderTest", func() {
		input := [8]Vec2{
			{-2.0, -1.0}, {0.0, -1.5}, {2.0, -1.0}, {2.5, 0.5},
			{1.0, 2.0}, {-1.0, 1.8}, {-2.5, 0.4}, {0.2, 0.1},
		}

		permutation := [8]int{5, 2, 7, 0, 4, 6, 1, 3}

		var pts [8]Point2D
		var hull [16]Point2D
		fillPoints2D(pts[:], input[:], 8)

		count1 := Hull2D(pts[:], 8, hull[:])
		ensure(count1 == 7)

		var onHull1 [8]bool
		for i := range count1 {
			onHull1[hull[i].OriginalIndex] = true
		}

		var shuffled [8]Point2D
		var shuffledHull [16]Point2D
		for i := range 8 {
			shuffled[i].P = input[permutation[i]]
			shuffled[i].Separation = 0.0
			shuffled[i].OriginalIndex = int32(i)
		}

		count2 := Hull2D(shuffled[:], 8, shuffledHull[:])
		ensure(count2 == count1)

		var onHull2 [8]bool
		for i := range count2 {
			onHull2[permutation[shuffledHull[i].OriginalIndex]] = true
		}

		for i := range 8 {
			ensure(onHull1[i] == onHull2[i])
		}
	})

	It("SimplifyHull2DTargetTest", func() {
		var pts [20]Point2D
		var hull [40]Point2D

		for i := range 20 {
			angle := float32(2.0*Pi) * float32(i) / 20.0
			pts[i].P = Vec2{cosf(angle), sinf(angle)}
			pts[i].Separation = 0.0
			pts[i].OriginalIndex = int32(i)
		}

		hullCount := Hull2D(pts[:], 20, hull[:])
		ensure(hullCount == 20)

		count := SimplifyHull2D(hull[:], hullCount, 6)
		ensure(count == 6)
		ensure(hull2DArea(hull[:], count) > 0.0)
	})

	It("SimplifyHull2DNoopTest", func() {
		input := [4]Vec2{
			{-1.0, -1.0},
			{1.0, -1.0},
			{1.0, 1.0},
			{-1.0, 1.0},
		}

		var pts [4]Point2D
		var hull [8]Point2D
		fillPoints2D(pts[:], input[:], 4)

		hullCount := Hull2D(pts[:], 4, hull[:])
		ensure(hullCount == 4)

		ensure(SimplifyHull2D(hull[:], hullCount, 8) == 4)
		ensure(SimplifyHull2D(hull[:], hullCount, 4) == 4)
	})
})
