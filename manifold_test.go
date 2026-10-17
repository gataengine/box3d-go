// Port of test/test_manifold.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// ensureFacePoints is ENSURE_FACE_POINTS( count ).
// A face contact keeps the whole clipped polygon when the manifold has room for it, so a build
// with a larger B3_MAX_MANIFOLD_POINTS legitimately reports more than four points. At the default
// this is still an exact count.
func ensureFacePoints(count int32) {
	GinkgoHelper()
	ensure(4 <= count && count <= MaxManifoldPoints)
}

// PORT: kRoot2, kAxisY, kAxisZ, ExactQuat and ExactRotation are shared with test_sat.c and defined
// in sat_test.go / hull_test.go.

var (
	kHalfRoot2 float32 = 0.70710678
	kAxisX             = Vec3{1.0, 0.0, 0.0}
)

// makeCrossedEdgeHulls is MakeCrossedEdgeHulls.
// Cube A yawed 45 degrees presents an edge along y at x = +h*root2.
// Cube B rolled 45 degrees presents an edge along z at x = -h*root2.
// Sliding B along x makes those two edges the closest features, so the axis of minimum
// penetration is x, the separation is d - 2*h*root2 and the contact point sits at x = d/2.
// Both hulls are far from a face axis here, which keeps the edge query in charge.
func makeCrossedEdgeHulls(hullA, hullB *BoxHull, halfWidth float32) {
	*hullA = MakeTransformedBoxHull(halfWidth, halfWidth, halfWidth, exactRotation(kAxisY, 0.25*Pi))
	*hullB = MakeTransformedBoxHull(halfWidth, halfWidth, halfWidth, exactRotation(kAxisZ, 0.25*Pi))
}

// slideX is SlideX.
func slideX(x float32) Transform {
	return Transform{Vec3{x, 0.0, 0.0}, QuatIdentity}
}

// slideY is SlideY.
func slideY(y float32) Transform {
	return Transform{Vec3{0.0, y, 0.0}, QuatIdentity}
}

// manifoldMinSeparation is MinSeparation.
func manifoldMinSeparation(manifold *LocalManifold) float32 {
	minSeparation := fltMax
	for i := 0; i < int(manifold.PointCount); i++ {
		minSeparation = MinFloat(minSeparation, manifold.Points[i].Separation)
	}

	return minSeparation
}

// kTiltAxes is kTiltAxes.
var kTiltAxes = []Vec3{
	{0.57735027, 0.57735027, 0.57735027},
	{0.70710678, 0.0, 0.70710678},
	{0.26726124, 0.53452248, 0.80178373},
	{-0.48507125, 0.72760688, -0.48507125},
}

// kTiltAngles: angles that straddle the 0.005 rejection threshold
var kTiltAngles = []float32{0.0, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 0.004, 0.005, 0.006, 0.01, 0.05}

// kHalfDiagonal: a cube corner is root3/2 from the center of rotation
var kHalfDiagonal float32 = 0.87

// manifoldSeed is g_seed in test_manifold.c.
var manifoldSeed uint32 = 12345

// manifoldNextFloat is NextFloat in test_manifold.c.
func manifoldNextFloat(lower, upper float32) float32 {
	manifoldSeed = 1664525*manifoldSeed + 1013904223
	t := float32(manifoldSeed>>8) / float32(1<<24)
	return mulAdd(lower, t, upper-lower)
}

// manifoldNextDirection is NextDirection in test_manifold.c.
func manifoldNextDirection() Vec3 {
	var v Vec3
	v.X = manifoldNextFloat(-1.0, 1.0)
	v.Y = manifoldNextFloat(-1.0, 1.0)
	v.Z = manifoldNextFloat(-1.0, 1.0)
	return Normalize(v)
}

// checkRoofFaceContact is CheckRoofFaceContact.
// A crossed ridge pair must land on a four point roof face contact. The clipped face
// separation can be no deeper than root2 times the vertical overlap.
func checkRoofFaceContact(manifold *LocalManifold, cache *SATCache, overlap float32) int {
	GinkgoHelper()
	ensureFacePoints(manifold.PointCount)
	ensure(SeparatingFeature(cache.Type) == FaceAxisA || SeparatingFeature(cache.Type) == FaceAxisB)

	// A roof face of one hull, so 45 degrees off the vertical
	ensureSmall(manifold.Normal.Y-kHalfRoot2, 1e-4)

	minSeparation := manifoldMinSeparation(manifold)
	ensure(minSeparation < float32(-kHalfRoot2*overlap)+1e-4)
	ensure(minSeparation > float32(-kRoot2*overlap)-1e-4)

	return 0
}

// checkEdgeContact is CheckEdgeContact.
// The edge pair axis produced by the arc intersection must match the classic edge cross product.
// Rebuild the axis, the separation and the contact point from the two contributing edges and the
// convex radius, then compare against the manifold. orientRef fixes the sign of the axis to match
// the manifold normal convention for the shape pair. e1 belongs to the shape whose contact point is
// pulled in by the radius (the hull or triangle when it meets a capsule), e2 to the other edge.
func checkEdgeContact(manifold *LocalManifold, p1, e1, p2, e2, orientRef Vec3,
	radius, normalTol, sepTol, pointTol float32) int {
	GinkgoHelper()
	axis := Normalize(Cross(e1, e2))
	if Dot(axis, orientRef) < 0.0 {
		axis = Neg(axis)
	}

	// Normal matches the cross product and is perpendicular to both edges
	ensureSmall(manifold.Normal.X-axis.X, normalTol)
	ensureSmall(manifold.Normal.Y-axis.Y, normalTol)
	ensureSmall(manifold.Normal.Z-axis.Z, normalTol)
	ensureSmall(Dot(manifold.Normal, Normalize(e1)), normalTol)
	ensureSmall(Dot(manifold.Normal, Normalize(e2)), normalTol)

	closest := LineDistance(p1, e1, p2, e2)

	// Signed gap between the edge lines along the axis, less the capsule radius
	expectedSeparation := Dot(axis, Sub(closest.Point2, closest.Point1)) - radius
	ensureSmall(manifold.Points[0].Separation-expectedSeparation, sepTol)

	// Midpoint of the closest approach, pulling the first point in by the radius
	expectedPoint := MulSV(0.5, Add(MulSub(closest.Point1, radius, axis), closest.Point2))
	ensureSmall(manifold.Points[0].Point.X-expectedPoint.X, pointTol)
	ensureSmall(manifold.Points[0].Point.Y-expectedPoint.Y, pointTol)
	ensureSmall(manifold.Points[0].Point.Z-expectedPoint.Z, pointTol)

	return 0
}

// hullEdgeSegment is HullEdgeSegment.
func hullEdgeSegment(hull *HullData, edgeIndex int, transform Transform, point, edge *Vec3) {
	edges := GetHullEdges(hull)
	points := GetHullPoints(hull)
	e := &edges[edgeIndex]
	tail := TransformPoint(transform, points[e.Origin])
	head := TransformPoint(transform, points[edges[e.Twin].Origin])
	*point = tail
	*edge = Sub(head, tail)
}

// checkSpeculativePoints is CheckSpeculativePoints.
func checkSpeculativePoints(manifold *LocalManifold, gap float32) int {
	GinkgoHelper()
	ensure(manifold.PointCount > 0)
	ensureSmall(manifoldMinSeparation(manifold)-gap, 1e-4)
	return 0
}

// degreesToRadians is the fixture expression `degrees * B3_PI / 180.0f`, rounded per operation.
func degreesToRadians(degrees float32) float32 {
	return float32(degrees*Pi) / 180.0
}

var _ = Describe("ManifoldTest", func() {
	// The library reports the point count it was built with. A mismatch against the manifold
	// this translation unit sees means the app and the library disagree on b3Manifold.
	It("MaxPointCountTest", func() {
		var manifold Manifold
		ensure(GetMaxManifoldPoints() == len(manifold.Points))
	})

	// The edge pair axis is built by intersecting the two Gauss map arcs. Walk the crossed edges from
	// speculative contact into deep overlap and check the axis, the separation and the point.
	It("CrossedEdgeTest", func() {
		var hullA, hullB BoxHull
		makeCrossedEdgeHulls(&hullA, &hullB, 0.5)

		distances := []float32{1.42, kRoot2, 1.41, 1.3}

		for i := 0; i < arrayCount(distances); i++ {
			d := distances[i]
			expected := d - kRoot2

			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SATCache{}
			CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, slideX(d), &cache)

			ensure(manifold.PointCount == 1)
			ensure(SeparatingFeature(cache.Type) == EdgePairAxis)
			ensureSmall(manifold.Normal.X-1.0, 1e-6)
			ensureSmall(manifold.Normal.Y, 1e-6)
			ensureSmall(manifold.Normal.Z, 1e-6)
			ensureSmall(manifold.Points[0].Separation-expected, 1e-5)
			ensureSmall(manifold.Points[0].Point.X-float32(0.5*d), 1e-5)
			ensureSmall(manifold.Points[0].Point.Y, 1e-5)
			ensureSmall(manifold.Points[0].Point.Z, 1e-5)

			// The forced edge query must agree with what the full solver chose
			manual := LocalManifold{}
			manual.Points = points[:]
			manualCache := SATCache{Type: uint8(ManualEdgePairAxis)}
			CollideHulls(&manual, 8, &hullA.Base, &hullB.Base, slideX(d), &manualCache)

			ensure(manual.PointCount == 1)
			ensureSmall(manual.Points[0].Separation-expected, 1e-5)
		}

		// Beyond the speculative distance the query reports the axis without building a contact.
		// The axis carries its own orientation now, so a sign error here would read as deep overlap.
		{
			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SATCache{}
			CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, slideX(1.5), &cache)

			ensure(manifold.PointCount == 0)
			ensure(SeparatingFeature(cache.Type) == EdgePairAxis)
			ensureSmall(cache.Separation-(1.5-kRoot2), 1e-5)
		}
	})

	// The parallel edge rejection compares dot products against the edge length, so it is a sine
	// threshold and must hold at any size.
	It("EdgeAxisScaleTest", func() {
		scales := []float32{100.0, 1.0, 0.2}

		for i := 0; i < arrayCount(scales); i++ {
			s := scales[i]
			var hullA, hullB BoxHull
			makeCrossedEdgeHulls(&hullA, &hullB, float32(0.5*s))

			expected := float32(-0.002)
			d := float32(s*kRoot2) + expected

			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SATCache{}
			CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, slideX(d), &cache)

			// Differencing coordinates of magnitude d costs precision proportional to the scale
			tolerance := float32(1e-5*s) + 1e-6

			ensure(manifold.PointCount == 1)
			ensure(SeparatingFeature(cache.Type) == EdgePairAxis)
			ensureSmall(manifold.Normal.X-1.0, 1e-6)
			ensureSmall(manifold.Points[0].Separation-expected, tolerance)
			ensureSmall(manifold.Points[0].Point.X-float32(0.5*d), tolerance)
			ensureSmall(manifold.Points[0].Point.Y, tolerance)
			ensureSmall(manifold.Points[0].Point.Z, tolerance)
		}
	})

	// The cached edge pair rebuilds the axis without a fresh query. An untouched cache proves the
	// cached branch answered rather than falling through to the full SAT.
	It("EdgeCacheTest", func() {
		var hullA, hullB BoxHull
		makeCrossedEdgeHulls(&hullA, &hullB, 0.5)

		var points [8]LocalManifoldPoint
		manifold := LocalManifold{}
		manifold.Points = points[:]
		cache := SATCache{}

		CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, slideX(1.41), &cache)
		ensure(manifold.PointCount == 1)
		ensure(SeparatingFeature(cache.Type) == EdgePairAxis)
		// Cached edges are the even half of each twin pair
		ensure((cache.IndexA&1) == 0 && int32(cache.IndexA) < hullA.Base.EdgeCount)
		ensure((cache.IndexB&1) == 0 && int32(cache.IndexB) < hullB.Base.EdgeCount)

		seededSeparation := cache.Separation
		ensureSmall(seededSeparation-(1.41-kRoot2), 1e-5)

		// Small motion, the cached features still describe the contact
		CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, slideX(1.4105), &cache)
		ensure(manifold.PointCount == 1)
		ensure(cache.Separation == seededSeparation)
		ensureSmall(manifold.Points[0].Separation-(1.4105-kRoot2), 1e-5)
		ensureSmall(manifold.Normal.X-1.0, 1e-6)

		// Jump past the speculative distance. The cached axis alone must report the separation.
		CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, slideX(1.5), &cache)
		ensure(manifold.PointCount == 0)
		ensure(cache.Separation == seededSeparation)
	})

	// Sliding B along the direction of edge A walks the closest point off the end of the segment.
	It("EdgeEndpointTest", func() {
		var hullA, hullB BoxHull
		makeCrossedEdgeHulls(&hullA, &hullB, 0.5)

		d := float32(1.41)
		expected := d - kRoot2

		// Just inside the end of edge A
		{
			transform := Transform{Vec3{d, 0.49, 0.0}, QuatIdentity}
			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SATCache{Type: uint8(ManualEdgePairAxis)}
			CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

			ensure(manifold.PointCount == 1)
			ensureSmall(manifold.Points[0].Separation-expected, 1e-5)
			ensureSmall(manifold.Points[0].Point.Y-0.49, 1e-5)
		}

		// Off the end. The edge pair no longer describes a contact, so the builder rejects it and
		// clears the cache rather than clamping to a point that is not on the hulls.
		{
			transform := Transform{Vec3{d, 0.55, 0.0}, QuatIdentity}
			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SATCache{Type: uint8(ManualEdgePairAxis)}
			CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

			ensure(manifold.PointCount == 0)
			ensure(SeparatingFeature(cache.Type) == InvalidAxis)

			// The true gap is a vertex to edge distance well past the speculative distance
			freshCache := SATCache{}
			CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &freshCache)
			ensure(manifold.PointCount == 0)
			ensure(freshCache.Separation > 0.0)
		}
	})

	// Cubes stacked face to face and tipped by a hair. A third of the edge pairs are then nearly
	// parallel, the angle between them is at the noise floor and the arc intersection carries no
	// information. The face contact has to survive that untouched.
	It("ParallelEdgeTest", func() {
		hullA := MakeBoxHull(0.5, 0.5, 0.5)
		hullB := MakeBoxHull(0.5, 0.5, 0.5)

		overlap := float32(0.01)

		for i := 0; i < arrayCount(kTiltAxes); i++ {
			for j := 0; j < arrayCount(kTiltAngles); j++ {
				angle := kTiltAngles[j]
				transform := Transform{Vec3{0.0, 1.0 - overlap, 0.0}, exactQuat(kTiltAxes[i], angle)}

				var points [8]LocalManifoldPoint
				manifold := LocalManifold{}
				manifold.Points = points[:]
				cache := SATCache{}
				CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

				ensureFacePoints(manifold.PointCount)
				ensure(SeparatingFeature(cache.Type) == FaceAxisA || SeparatingFeature(cache.Type) == FaceAxisB)
				ensure(Dot(manifold.Normal, kAxisY) > 0.998)

				// The tilt can only lift or sink a face point by the length of the arc it sweeps
				bound := float32(kHalfDiagonal*angle) + 1e-5
				for k := 0; k < int(manifold.PointCount); k++ {
					ensureSmall(manifold.Points[k].Separation+overlap, bound)
				}
			}
		}
	})

	// Same stack, but force the edge query to answer. With the edges exactly parallel no pair forms a
	// Minkowski face at all, and once a pair does form its separation can never be positive because
	// the hulls overlap.
	It("ParallelEdgeManualTest", func() {
		hullA := MakeBoxHull(0.5, 0.5, 0.5)
		hullB := MakeBoxHull(0.5, 0.5, 0.5)

		overlap := float32(0.01)

		for i := 0; i < arrayCount(kTiltAxes); i++ {
			for j := 0; j < arrayCount(kTiltAngles); j++ {
				angle := kTiltAngles[j]
				transform := Transform{Vec3{0.0, 1.0 - overlap, 0.0}, exactQuat(kTiltAxes[i], angle)}

				var points [8]LocalManifoldPoint
				manifold := LocalManifold{}
				manifold.Points = points[:]
				cache := SATCache{Type: uint8(ManualEdgePairAxis)}
				CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

				if angle == 0.0 {
					// Every pair is parallel so the query finds nothing and leaves the cache alone
					ensure(manifold.PointCount == 0)
					ensure(SeparatingFeature(cache.Type) == ManualEdgePairAxis)
					continue
				}

				// The closest points can fall off the ends of the segments
				if manifold.PointCount == 0 {
					continue
				}

				ensure(manifold.PointCount == 1)
				ensure(Dot(manifold.Normal, kAxisY) > 0.99)

				separation := manifold.Points[0].Separation
				ensure(separation <= 0.0)
				ensure(separation >= -overlap-float32(kHalfDiagonal*angle)-1e-4)
			}
		}
	})

	// Overlapping hulls admit no separating axis, so an edge separation that comes back positive is
	// always noise. It shows up as a manifold with no points, which the solver reads as no contact.
	It("OverlapNeverEmptyTest", func() {
		// PORT: upstream relies on g_seed still holding its initial value here (no earlier subtest
		// draws from it). Reset it so the spec does not depend on Ginkgo's spec order.
		manifoldSeed = 12345

		hullA := MakeBoxHull(0.5, 0.5, 0.5)
		hullB := MakeBoxHull(0.4, 0.6, 0.5)

		for i := range 2000 {
			axis := manifoldNextDirection()

			// Half the samples are nearly aligned, where the edge cross products are smallest
			var angle float32
			if i&1 != 0 {
				angle = manifoldNextFloat(-0.01, 0.01)
			} else {
				angle = manifoldNextFloat(-Pi, Pi)
			}

			// Shorter than the smallest half width, so the center of B is inside A
			offset := MulSV(0.4, manifoldNextDirection())

			transform := Transform{offset, exactQuat(axis, angle)}

			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SATCache{}
			CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

			ensure(manifold.PointCount > 0)
			ensureSmall(Length(manifold.Normal)-1.0, 1e-5)

			// Clipping keeps points that are separated, but the deepest one must penetrate
			ensure(manifoldMinSeparation(&manifold) < 0.0)
		}
	})

	// Two long roof ridges laid across each other. The axis of minimum penetration is the edge
	// pair, but a one point edge contact is weak for stacking. The collider builds the roof face
	// contact first and only switches to the edge contact when the edge axis beats the clipped
	// face separation by more than the slop. This pins all three regimes of that policy.
	It("RidgeCrossingTest", func() {
		hullA := MakeTransformedBoxHull(1.5, 0.1, 0.1, exactRotation(kAxisX, 0.25*Pi))
		hullB := MakeTransformedBoxHull(1.5, 0.1, 0.1, exactRotation(kAxisX, 0.25*Pi))

		ridgeY := float32(0.1 * kRoot2)

		// Shallow overlap. The edge axis is better by only ( root2 - 1 ) * overlap, inside the
		// slop, so the four point face contact carries the crossing at every angle.
		{
			overlap := float32(0.01)
			lift := float32(2.0*ridgeY) - overlap
			crossingAngles := []float32{0.0, 1e-3, 0.02, 0.1, 0.5}

			for i := 0; i < arrayCount(crossingAngles); i++ {
				transform := Transform{Vec3{0.0, lift, 0.0}, exactQuat(kAxisY, crossingAngles[i])}

				var points [8]LocalManifoldPoint
				manifold := LocalManifold{}
				manifold.Points = points[:]
				cache := SATCache{}
				CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

				if checkRoofFaceContact(&manifold, &cache, overlap) != 0 {
					return
				}
			}
		}

		// Deep overlap at a clear crossing. The edge axis now beats the clipped face separation
		// by more than the slop, so the edge contact replaces the face contact.
		{
			overlap := float32(0.05)
			lift := float32(2.0*ridgeY) - overlap
			crossingAngles := []float32{0.05, 0.1, 0.2, 0.5}

			for i := 0; i < arrayCount(crossingAngles); i++ {
				transform := Transform{Vec3{0.0, lift, 0.0}, exactQuat(kAxisY, crossingAngles[i])}

				var points [8]LocalManifoldPoint
				manifold := LocalManifold{}
				manifold.Points = points[:]
				cache := SATCache{}
				CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

				ensure(manifold.PointCount == 1)
				ensure(SeparatingFeature(cache.Type) == EdgePairAxis)
				ensureSmall(manifold.Normal.X, 1e-4)
				ensureSmall(manifold.Normal.Y-1.0, 1e-4)
				ensureSmall(manifold.Normal.Z, 1e-4)
				ensureSmall(manifold.Points[0].Separation+overlap, 1e-4)
				ensureSmall(manifold.Points[0].Point.Y-(ridgeY-float32(0.5*overlap)), 1e-4)

				// Only has to land near the crossing, not at the end of a three meter beam
				ensureSmall(manifold.Points[0].Point.X, 0.01)
				ensureSmall(manifold.Points[0].Point.Z, 0.01)
			}
		}

		// Deep overlap near parallel. A one point edge contact off a parallel pair would have a
		// normal built from noise, so the roof faces keep the contact.
		{
			overlap := float32(0.05)
			lift := float32(2.0*ridgeY) - overlap
			shallowAngles := []float32{0.0, 1e-4, 1e-3, 0.003}

			for i := 0; i < arrayCount(shallowAngles); i++ {
				transform := Transform{Vec3{0.0, lift, 0.0}, exactQuat(kAxisY, shallowAngles[i])}

				var points [8]LocalManifoldPoint
				manifold := LocalManifold{}
				manifold.Points = points[:]
				cache := SATCache{}
				CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

				if checkRoofFaceContact(&manifold, &cache, overlap) != 0 {
					return
				}
			}
		}
	})

	// A cube pitched 45 degrees rests on an edge along x at y = -h*root2. The two faces meeting there
	// have normals (0,-r,r) and (0,-r,-r), so the arc between them spans the whole lower quadrant.
	// A triangle edge crossing under it at an angle picks out an interior point of that arc, which is
	// where a wrong lerp parameter would show up.
	It("TriangleEdgeTest", func() {
		beta := degreesToRadians(20.0)
		gamma := degreesToRadians(30.0)

		hull := MakeTransformedBoxHull(0.5, 0.5, 0.5, exactRotation(kAxisX, 0.25*Pi))

		// Tipping the triangle plane about z keeps its normal off the hull edge, which the Minkowski
		// test needs. Tipping the edge within that plane moves the arc intersection off the midpoint.
		triNormal := Vec3{sinf(beta), cosf(beta), 0.0}
		triEdge := Vec3{float32(sinf(gamma) * cosf(beta)), float32(-sinf(gamma) * sinf(beta)), cosf(gamma)}

		// Perpendicular to both edges and pointing out of the hull
		axis := Normalize(Cross(kAxisX, triEdge))
		hullPoint := Vec3{0.0, -kHalfRoot2, 0.0}

		gaps := []float32{0.03, 0.01, 0.0, -0.01, -0.1}

		for i := 0; i < arrayCount(gaps); i++ {
			gap := gaps[i]
			trianglePoint := MulAdd(hullPoint, gap, axis)

			v1 := MulAdd(trianglePoint, -1.0, triEdge)
			v2 := MulAdd(trianglePoint, 1.0, triEdge)
			v3 := MulAdd(v1, 1.5, Cross(triNormal, triEdge))

			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SATCache{Type: uint8(ManualEdgePairAxis)}
			CollideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, &hull.Base, &cache, true)

			expectedNormal := Neg(axis)
			expectedPoint := MulAdd(hullPoint, float32(0.5*gap), axis)

			ensure(manifold.PointCount == 1)
			ensure(SeparatingFeature(cache.Type) == EdgePairAxis)
			ensureSmall(manifold.Normal.X-expectedNormal.X, 1e-5)
			ensureSmall(manifold.Normal.Y-expectedNormal.Y, 1e-5)
			ensureSmall(manifold.Normal.Z-expectedNormal.Z, 1e-5)
			ensureSmall(manifold.Points[0].Separation-gap, 1e-5)
			ensureSmall(manifold.Points[0].Point.X-expectedPoint.X, 1e-5)
			ensureSmall(manifold.Points[0].Point.Y-expectedPoint.Y, 1e-5)
			ensureSmall(manifold.Points[0].Point.Z-expectedPoint.Z, 1e-5)
		}

		// The tipped triangle plane buries a corner of the hull, so neither face axis separates and
		// the edge axis has to carry the speculative cull on its own.
		culled := []float32{0.03, 0.05}

		for i := 0; i < arrayCount(culled); i++ {
			gap := culled[i]
			trianglePoint := MulAdd(hullPoint, gap, axis)

			v1 := MulAdd(trianglePoint, -1.0, triEdge)
			v2 := MulAdd(trianglePoint, 1.0, triEdge)
			v3 := MulAdd(v1, 1.5, Cross(triNormal, triEdge))

			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SATCache{}
			CollideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, &hull.Base, &cache, true)

			ensure(manifold.PointCount == 0)
			ensure(SeparatingFeature(cache.Type) == EdgePairAxis)
			ensureSmall(cache.Separation-gap, 1e-5)
		}
	})

	// The same cube resting its bottom edge on a triangle whose first edge runs along x. Tipping the
	// triangle takes that pair from exactly parallel through the rejection threshold.
	It("TriangleParallelEdgeTest", func() {
		hull := MakeTransformedBoxHull(0.5, 0.5, 0.5, exactRotation(kAxisX, 0.25*Pi))

		overlap := float32(0.01)
		y := -kHalfRoot2 + overlap

		for i := 0; i < arrayCount(kTiltAxes); i++ {
			for j := 0; j < arrayCount(kTiltAngles); j++ {
				q := exactQuat(kTiltAxes[i], kTiltAngles[j])

				v1 := RotateVector(q, Vec3{-2.0, y, -1.0})
				v2 := RotateVector(q, Vec3{0.0, y, 2.0})
				v3 := RotateVector(q, Vec3{2.0, y, -1.0})

				var points [8]LocalManifoldPoint
				manifold := LocalManifold{}
				manifold.Points = points[:]
				cache := SATCache{}
				CollideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, &hull.Base, &cache, true)

				ensureFacePoints(manifold.PointCount)
				ensure(SeparatingFeature(cache.Type) == FaceAxisA)
				ensure(Dot(manifold.Normal, kAxisY) > 0.99)

				// The tilt can only sink the contact by the length of the arc it sweeps
				bound := float32(kHalfDiagonal*kTiltAngles[j]) + 1e-5
				ensureSmall(manifoldMinSeparation(&manifold)+overlap, bound)
			}
		}
	})

	// Two boxes crossing edge to edge. A holds a vertical edge, B is rolled to present a crossing edge
	// and yawed so the arc intersection walks off the midpoint. For every configuration that resolves
	// to an edge pair the recovered axis, separation and point must match the cross product oracle to
	// tight tolerance. The oracle reads the edges the solver actually latched onto, so the check is
	// exact regardless of which pair wins.
	It("EdgeAxisOracleTest", func() {
		hullA := MakeTransformedBoxHull(0.5, 0.5, 0.5, exactRotation(kAxisY, 0.25*Pi))

		rolls := []float32{float32(0.18) * Pi, float32(0.25) * Pi, float32(0.32) * Pi}
		yaws := []float32{-0.35, -0.15, 0.0, 0.15, 0.35}
		distances := []float32{1.38, 1.40, kRoot2, 1.44}

		edgeContacts := 0

		for i := 0; i < arrayCount(rolls); i++ {
			hullB := MakeTransformedBoxHull(0.5, 0.5, 0.5, exactRotation(kAxisZ, rolls[i]))

			for j := 0; j < arrayCount(yaws); j++ {
				for k := 0; k < arrayCount(distances); k++ {
					transform := Transform{Vec3{distances[k], 0.0, 0.0}, exactQuat(kAxisY, yaws[j])}

					var points [8]LocalManifoldPoint
					manifold := LocalManifold{}
					manifold.Points = points[:]
					cache := SATCache{}
					CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

					if SeparatingFeature(cache.Type) != EdgePairAxis || manifold.PointCount != 1 {
						continue
					}

					var p1, e1, p2, e2 Vec3
					hullEdgeSegment(&hullA.Base, int(cache.IndexA), TransformIdentity, &p1, &e1)
					hullEdgeSegment(&hullB.Base, int(cache.IndexB), transform, &p2, &e2)

					centerA := hullA.Base.Center
					centerB := TransformPoint(transform, hullB.Base.Center)
					orientRef := Sub(centerB, centerA)

					if checkEdgeContact(&manifold, p1, e1, p2, e2, orientRef, 0.0, 2e-4, 2e-4, 2e-3) != 0 {
						return
					}

					edgeContacts++
				}
			}
		}

		// The sweep is only meaningful if it actually drove the edge path
		ensure(edgeContacts >= 15)
	})

	// The same oracle over randomly oriented box pairs. Whenever the solver reports an edge pair the
	// recovered axis must be perpendicular to both edges and match the cross product. This casts a wide
	// net over the arc that the structured sweep cannot reach.
	It("EdgeAxisRandomOracleTest", func() {
		manifoldSeed = 246813579

		edgeContacts := 0

		for range 2000 {
			angleA := manifoldNextFloat(0.2, 0.5) * Pi
			angleB := manifoldNextFloat(0.2, 0.5) * Pi
			axisA := manifoldNextDirection()
			hullA := MakeTransformedBoxHull(0.5, 0.5, 0.5, exactRotation(axisA, angleA))
			hullB := MakeBoxHull(0.5, 0.5, 0.5)

			d := manifoldNextFloat(1.2, 1.55)
			var transform Transform
			transform.P = MulSV(d, manifoldNextDirection())
			axisB := manifoldNextDirection()
			transform.Q = exactQuat(axisB, angleB)

			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SATCache{}
			CollideHulls(&manifold, 8, &hullA.Base, &hullB.Base, transform, &cache)

			if SeparatingFeature(cache.Type) != EdgePairAxis || manifold.PointCount != 1 {
				continue
			}

			var p1, e1, p2, e2 Vec3
			hullEdgeSegment(&hullA.Base, int(cache.IndexA), TransformIdentity, &p1, &e1)
			hullEdgeSegment(&hullB.Base, int(cache.IndexB), transform, &p2, &e2)

			// Skip crossings near parallel where the closest point solve is ill conditioned. The
			// parallel rejection itself is covered by ParallelEdgeTest.
			sine := Length(Cross(Normalize(e1), Normalize(e2)))
			if sine < 0.1 {
				continue
			}

			orientRef := Sub(TransformPoint(transform, hullB.Base.Center), hullA.Base.Center)

			if checkEdgeContact(&manifold, p1, e1, p2, e2, orientRef, 0.0, 1e-3, 1e-3, 5e-3) != 0 {
				return
			}

			edgeContacts++
		}

		ensure(edgeContacts >= 100)
	})

	// A thin capsule stabbed through the +x +y edge of a box so the edge pair is the axis of minimum
	// penetration. This drives the isolated edge axis (arc versus circle on the Gauss map) that a
	// capsule presents. The edge is nearly parallel to a box face normal, exactly where the old center
	// based orientation flickered, so the axis, the penetration and the point are all checked.
	It("HullCapsuleEdgeDeepTest", func() {
		hull := MakeBoxHull(0.5, 0.5, 0.5)

		// The +x +y edge runs along z between the +x and +y faces
		edgePoint := Vec3{0.5, 0.5, 0.0}
		edgeDir := Vec3{0.0, 0.0, 1.0}
		outward := Normalize(Vec3{1.0, 1.0, 0.0})

		// Penetrate far enough that the core segment clearly overlaps the box so the deep path runs,
		// but keep the radius small enough that the edge stays the axis of minimum penetration.
		depths := []float32{0.12, 0.18, 0.25}
		radii := []float32{0.05, 0.1, 0.2}
		tilts := []float32{0.0, 0.25, -0.25}

		count := 0

		for i := 0; i < arrayCount(depths); i++ {
			for j := 0; j < arrayCount(radii); j++ {
				for k := 0; k < arrayCount(tilts); k++ {
					capsuleDir := Normalize(Vec3{1.0, -1.0, tilts[k]})
					mid := MulAdd(edgePoint, -depths[i], outward)
					c1 := MulAdd(mid, -0.5, capsuleDir)
					c2 := MulAdd(mid, 0.5, capsuleDir)
					capsule := Capsule{c1, c2, radii[j]}

					var points [8]LocalManifoldPoint
					manifold := LocalManifold{}
					manifold.Points = points[:]
					cache := SimplexCache{}
					CollideHullAndCapsule(&manifold, 8, &hull.Base, &capsule, TransformIdentity, &cache)

					ensure(manifold.PointCount == 1)
					ensure(manifold.Points[0].Separation < 0.0)

					// Hull edge is e1, capsule axis is e2, normal points out of the hull
					if checkEdgeContact(&manifold, edgePoint, edgeDir, c1, Sub(c2, c1), outward, radii[j], 1e-4, 1e-4,
						1e-4) != 0 {
						return
					}

					count++
				}
			}
		}

		ensure(count == arrayCount(depths)*arrayCount(radii)*arrayCount(tilts))
	})

	// Force the triangle versus hull edge query over a broad sweep of crossing geometries. A cube tipped
	// 45 degrees rests an edge along x at y = -h*root2. A triangle edge is laid across it at a range of
	// yaws, plane tips and gaps so the arc intersection lands all over the arc. The manual axis hands
	// the winning pair to the builder, and the recovered axis must match the cross product of the chosen
	// triangle and hull edges and point from the triangle into the hull.
	It("TriangleHullEdgeSweepTest", func() {
		hull := MakeTransformedBoxHull(0.5, 0.5, 0.5, exactRotation(kAxisX, 0.25*Pi))
		hullEdgePoint := Vec3{0.0, -kHalfRoot2, 0.0}

		// Degrees: triangle plane tip about z, and triangle edge yaw
		betas := []float32{8.0, 20.0, 32.0}
		gammas := []float32{20.0, 35.0, 50.0, 70.0}
		gaps := []float32{0.02, 0.0, -0.03, -0.08}

		edgeContacts := 0

		for a := 0; a < arrayCount(betas); a++ {
			for b := 0; b < arrayCount(gammas); b++ {
				for c := 0; c < arrayCount(gaps); c++ {
					beta := degreesToRadians(betas[a])
					gamma := degreesToRadians(gammas[b])

					// Tip the plane off the hull edge so the Minkowski test holds, then yaw the edge
					triNormal := Vec3{sinf(beta), cosf(beta), 0.0}
					triEdge := Vec3{float32(sinf(gamma) * cosf(beta)), float32(-sinf(gamma) * sinf(beta)), cosf(gamma)}

					// Perpendicular to both edges and pointing out of the hull
					axis := Normalize(Cross(kAxisX, triEdge))
					trianglePoint := MulAdd(hullEdgePoint, gaps[c], axis)

					v1 := MulAdd(trianglePoint, -1.0, triEdge)
					v2 := MulAdd(trianglePoint, 1.0, triEdge)
					v3 := MulAdd(v1, 1.5, Cross(triNormal, triEdge))

					triangleVerts := [3]Vec3{v1, v2, v3}
					triangleEdges := [3]Vec3{Sub(v2, v1), Sub(v3, v2), Sub(v1, v3)}
					triangleCenter := MulSV(1.0/3.0, Add(v1, Add(v2, v3)))

					var points [8]LocalManifoldPoint
					manifold := LocalManifold{}
					manifold.Points = points[:]
					cache := SATCache{Type: uint8(ManualEdgePairAxis)}
					CollideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, &hull.Base, &cache, true)

					if SeparatingFeature(cache.Type) != EdgePairAxis || manifold.PointCount != 1 {
						continue
					}

					p1 := triangleVerts[cache.IndexA]
					e1 := triangleEdges[cache.IndexA]

					var p2, e2 Vec3
					hullEdgeSegment(&hull.Base, int(cache.IndexB), TransformIdentity, &p2, &e2)

					// Normal points from the triangle into the hull
					orientRef := Sub(hull.Base.Center, triangleCenter)

					if checkEdgeContact(&manifold, p1, e1, p2, e2, orientRef, 0.0, 1e-4, 1e-4, 1e-3) != 0 {
						return
					}

					edgeContacts++
				}
			}
		}

		ensure(edgeContacts >= 30)
	})

	// A capsule laid nearly in a triangle plane and pushed across one edge so the edge pair drives the
	// deep contact. This exercises the two sided triangle edge, where the side normal trick chooses
	// which half of the arc holds the axis. Validate every edge contact the sweep produces.
	It("CapsuleTriangleEdgeDeepTest", func() {
		// Triangle in the y = 0 plane. The v1 v2 edge runs along x at z = 0, the interior lies at z < 0.
		v1 := Vec3{-2.0, 0.0, 0.0}
		v2 := Vec3{2.0, 0.0, 0.0}
		v3 := Vec3{0.0, 0.0, -2.0}
		triangle := [3]Vec3{v1, v2, v3}
		triangleEdges := [3]Vec3{Sub(v2, v1), Sub(v3, v2), Sub(v1, v3)}
		triangleCenter := MulSV(1.0/3.0, Add(v1, Add(v2, v3)))

		// A nearly in plane core crossing the edge at (0,0,z0) with a small out of plane tilt. The core
		// pierces the triangle just inside the edge so the deep path runs and the tilted edge pair wins.
		z0s := []float32{-0.05, -0.03, -0.01}
		tilts := []float32{0.2, 0.3, 0.4}
		yaws := []float32{0.4, 0.6, 0.8}
		radii := []float32{0.05, 0.1}

		edgeContacts := 0

		for i := 0; i < arrayCount(z0s); i++ {
			for j := 0; j < arrayCount(tilts); j++ {
				for y := 0; y < arrayCount(yaws); y++ {
					for r := 0; r < arrayCount(radii); r++ {
						capsuleDir := Normalize(Vec3{sinf(yaws[y]), tilts[j], cosf(yaws[y])})
						mid := Vec3{0.0, 0.0, z0s[i]}
						c1 := MulAdd(mid, -0.6, capsuleDir)
						c2 := MulAdd(mid, 0.6, capsuleDir)
						capsule := Capsule{c1, c2, radii[r]}

						var points [8]LocalManifoldPoint
						manifold := LocalManifold{}
						manifold.Points = points[:]
						cache := SimplexCache{}
						CollideTriangleAndCapsule(&manifold, 8, triangle[:], &capsule, &cache)

						// Only the edge contacts exercise the new axis. Face contacts are handled elsewhere.
						if manifold.PointCount != 1 || manifold.Feature < FeatureEdge1 || manifold.Feature > FeatureEdge3 {
							continue
						}

						edgeIndex := int(manifold.Feature - FeatureEdge1)
						p1 := triangle[edgeIndex]
						e1 := triangleEdges[edgeIndex]

						// Normal points from the triangle toward the capsule
						capsuleEdge := Sub(c2, c1)
						capsuleCenter := Lerp(c1, c2, 0.5)
						orientRef := Sub(capsuleCenter, triangleCenter)

						if checkEdgeContact(&manifold, p1, e1, c1, capsuleEdge, orientRef, radii[r], 1e-4, 1e-4, 2e-4) != 0 {
							return
						}

						edgeContacts++
					}
				}
			}
		}

		// The sweep must actually reach the edge path
		ensure(edgeContacts >= 15)
	})

	// A sphere driven straight through a box face, from separated, across the surface where the collider
	// switches from GJK closest points to the SAT face pick, and on into deep overlap. The separation
	// must stay the analytic gap the whole way and the normal must not flip. A jump at the seam would
	// read as a pop in the solver. The sweep is fine enough to land samples on both sides of the seam.
	It("SphereHullSeamTest", func() {
		hull := MakeBoxHull(0.5, 0.5, 0.5)
		radius := float32(0.15)

		yStart := 0.5 + radius + float32(0.4*SpeculativeDistance())
		yEnd := float32(0.1)
		steps := 400
		dy := (yStart - yEnd) / float32(steps)

		previous := float32(0.0)
		shallowSamples := 0
		deepSamples := 0

		for i := 0; i <= steps; i++ {
			y := yStart - float32(float32(i)*dy)
			sphere := Sphere{Vec3{0.0, y, 0.0}, radius}

			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SimplexCache{}
			CollideHullAndSphere(&manifold, 8, &hull.Base, &sphere, TransformIdentity, &cache)

			ensure(manifold.PointCount == 1)

			separation := manifold.Points[0].Separation
			expected := (y - 0.5) - radius

			// Separation is the analytic gap on both sides of the seam
			ensureSmall(separation-expected, 1e-5)

			// Normal holds the face direction with no flip
			ensureSmall(manifold.Normal.X, 1e-5)
			ensureSmall(manifold.Normal.Y-1.0, 1e-5)
			ensureSmall(manifold.Normal.Z, 1e-5)

			// No jump across the seam: consecutive separations track the step
			if i > 0 {
				ensureSmall((previous-separation)-dy, 1e-5)
			}
			previous = separation

			if y > 0.5 {
				shallowSamples += 1
			} else {
				deepSamples += 1
			}
		}

		// The sweep must straddle the surface so both the GJK and the SAT branch run
		ensure(shallowSamples > 0 && deepSamples > 0)
	})

	// The same seam for a capsule laid parallel to the face. Above the surface the shallow path clips two
	// points, in overlap the face path builds two, and every point must sit at the analytic gap through
	// the transition with the normal fixed on the face.
	It("CapsuleHullSeamTest", func() {
		hull := MakeBoxHull(0.5, 0.5, 0.5)
		radius := float32(0.15)
		halfLength := float32(0.3)

		yStart := 0.5 + radius + float32(0.4*SpeculativeDistance())
		yEnd := float32(0.1)
		steps := 400
		dy := (yStart - yEnd) / float32(steps)

		previous := float32(0.0)
		shallowSamples := 0
		deepSamples := 0

		for i := 0; i <= steps; i++ {
			y := yStart - float32(float32(i)*dy)
			capsule := Capsule{Vec3{-halfLength, y, 0.0}, Vec3{halfLength, y, 0.0}, radius}

			var points [8]LocalManifoldPoint
			manifold := LocalManifold{}
			manifold.Points = points[:]
			cache := SimplexCache{}
			CollideHullAndCapsule(&manifold, 8, &hull.Base, &capsule, TransformIdentity, &cache)

			ensure(manifold.PointCount >= 1)

			expected := (y - 0.5) - radius

			// Every point sits at the analytic gap
			for k := 0; k < int(manifold.PointCount); k++ {
				ensureSmall(manifold.Points[k].Separation-expected, 1e-5)
			}

			// Normal holds the face direction with no flip
			ensureSmall(manifold.Normal.X, 1e-5)
			ensureSmall(manifold.Normal.Y-1.0, 1e-5)
			ensureSmall(manifold.Normal.Z, 1e-5)

			// No jump across the seam
			minSeparation := manifoldMinSeparation(&manifold)
			if i > 0 {
				ensureSmall((previous-minSeparation)-dy, 1e-5)
			}
			previous = minSeparation

			if y > 0.5 {
				shallowSamples += 1
			} else {
				deepSamples += 1
			}
		}

		ensure(shallowSamples > 0 && deepSamples > 0)
	})

	// A capsule core straddling a triangle face inside the interior. The core pierces the plane so the
	// deep path runs, and with the tilt kept small the face stays the axis of minimum penetration, so the
	// clip must return two points on the triangle face. This is the branch that had no coverage.
	It("CapsuleTriangleFaceDeepTest", func() {
		// Triangle in the y = 0 plane, normal +y, centroid at the origin
		v1 := Vec3{-3.0, 0.0, -2.0}
		v2 := Vec3{0.0, 0.0, 4.0}
		v3 := Vec3{3.0, 0.0, -2.0}
		triangle := [3]Vec3{v1, v2, v3}

		yaws := []float32{0.0, 0.6, 1.2, 1.8, 2.4}
		tilts := []float32{0.06, 0.1, 0.15}
		radii := []float32{0.05, 0.1, 0.2}

		// Bias the center just above the plane so the back side cull passes while the lower endpoint dips through
		bias := float32(0.01)
		halfLength := float32(1.0)

		faceContacts := 0

		for i := 0; i < arrayCount(yaws); i++ {
			for j := 0; j < arrayCount(tilts); j++ {
				for r := 0; r < arrayCount(radii); r++ {
					// Axis is the in-plane heading tipped up so the segment straddles the plane
					axis := Vec3{float32(cosf(tilts[j]) * cosf(yaws[i])), sinf(tilts[j]), float32(cosf(tilts[j]) * sinf(yaws[i]))}
					center := Vec3{0.0, bias, 0.0}
					c1 := MulAdd(center, -halfLength, axis)
					c2 := MulAdd(center, halfLength, axis)
					capsule := Capsule{c1, c2, radii[r]}

					var points [8]LocalManifoldPoint
					manifold := LocalManifold{}
					manifold.Points = points[:]
					cache := SimplexCache{}
					CollideTriangleAndCapsule(&manifold, 8, triangle[:], &capsule, &cache)

					// Two points on the triangle face with the plane normal
					ensure(manifold.PointCount == 2)
					ensure(manifold.Feature == FeatureTriangleFace)
					ensureSmall(manifold.Normal.X, 1e-5)
					ensureSmall(manifold.Normal.Y-1.0, 1e-5)
					ensureSmall(manifold.Normal.Z, 1e-5)

					// Separations are the endpoint heights pulled in by the radius. The lower endpoint is below
					// the plane, so the deepest separation is negative.
					lower := MinFloat(c1.Y, c2.Y) - radii[r]
					upper := MaxFloat(c1.Y, c2.Y) - radii[r]
					minSep := MinFloat(manifold.Points[0].Separation, manifold.Points[1].Separation)
					maxSep := MaxFloat(manifold.Points[0].Separation, manifold.Points[1].Separation)
					ensureSmall(minSep-lower, 1e-5)
					ensureSmall(maxSep-upper, 1e-5)
					ensure(minSep < 0.0)

					faceContacts++
				}
			}
		}

		// Every configuration must reach the face path
		ensure(faceContacts == arrayCount(yaws)*arrayCount(tilts)*arrayCount(radii))
	})

	// A capsule laid flat with its core below a box top face. The core sits inside the box so the deep
	// path runs, and across a sweep of depths, headings and radii the face clip must return two points.
	It("HullCapsuleFaceDeepTest", func() {
		hull := MakeBoxHull(0.5, 0.5, 0.5)

		depths := []float32{0.1, 0.2, 0.3, 0.4, 0.45}
		yaws := []float32{0.0, 0.4, 0.8, 1.2}
		radii := []float32{0.1, 0.15, 0.2}
		offsets := []float32{-0.1, 0.0, 0.1}
		halfLength := float32(0.3)

		faceContacts := 0

		for i := 0; i < arrayCount(depths); i++ {
			for j := 0; j < arrayCount(yaws); j++ {
				for r := 0; r < arrayCount(radii); r++ {
					for o := 0; o < arrayCount(offsets); o++ {
						y := depths[i]
						dir := Vec3{cosf(yaws[j]), 0.0, sinf(yaws[j])}
						center := Vec3{offsets[o], y, 0.0}
						c1 := MulAdd(center, -halfLength, dir)
						c2 := MulAdd(center, halfLength, dir)
						capsule := Capsule{c1, c2, radii[r]}

						var points [8]LocalManifoldPoint
						manifold := LocalManifold{}
						manifold.Points = points[:]
						cache := SimplexCache{}
						CollideHullAndCapsule(&manifold, 8, &hull.Base, &capsule, TransformIdentity, &cache)

						// Two points on the top face. The hull path does not tag a feature, so the face is
						// identified by the normal and the point count.
						ensure(manifold.PointCount == 2)
						ensureSmall(manifold.Normal.X, 1e-5)
						ensureSmall(manifold.Normal.Y-1.0, 1e-5)
						ensureSmall(manifold.Normal.Z, 1e-5)

						// Flat capsule, so both points sit at the same analytic gap
						expected := (y - 0.5) - radii[r]
						ensureSmall(manifold.Points[0].Separation-expected, 1e-5)
						ensureSmall(manifold.Points[1].Separation-expected, 1e-5)
						ensure(expected < 0.0)

						faceContacts++
					}
				}
			}
		}

		ensure(faceContacts == arrayCount(depths)*arrayCount(yaws)*arrayCount(radii)*arrayCount(offsets))
	})

	// Contact recycling treats a pair without points as separated by at least the speculative distance.
	// Every manifold routine must report a pair inside that distance or recycling can skip real overlap.
	// Discussion #175.
	It("SpeculativeMarginTest", func() {
		gaps := []float32{0.0, float32(0.25 * SpeculativeDistance()), float32(0.75 * SpeculativeDistance())}
		gapCount := len(gaps)

		sphere := Sphere{Vec3Zero, 0.25}
		bigSphere := Sphere{Vec3Zero, 0.5}
		capsuleX := Capsule{Vec3{-0.5, 0.0, 0.0}, Vec3{0.5, 0.0, 0.0}, 0.25}
		capsuleZ := Capsule{Vec3{0.0, 0.0, -0.5}, Vec3{0.0, 0.0, 0.5}, 0.25}
		box := MakeBoxHull(0.5, 0.5, 0.5)

		// Normal is +y
		triangle := [3]Vec3{{-2.0, 0.0, -2.0}, {0.0, 0.0, 2.0}, {2.0, 0.0, -2.0}}

		var points [MaxManifoldPoints]LocalManifoldPoint
		manifold := LocalManifold{}
		manifold.Points = points[:]

		for i := range gapCount {
			gap := gaps[i]

			manifold.PointCount = 0
			CollideSpheres(&manifold, MaxManifoldPoints, &bigSphere, &sphere, slideY(0.75+gap))
			ensure(checkSpeculativePoints(&manifold, gap) == 0)

			manifold.PointCount = 0
			CollideCapsuleAndSphere(&manifold, MaxManifoldPoints, &capsuleX, &sphere, slideY(0.5+gap))
			ensure(checkSpeculativePoints(&manifold, gap) == 0)

			manifold.PointCount = 0
			CollideCapsules(&manifold, MaxManifoldPoints, &capsuleX, &capsuleZ, slideY(0.5+gap))
			ensure(checkSpeculativePoints(&manifold, gap) == 0)

			{
				manifold.PointCount = 0
				cache := SimplexCache{}
				CollideHullAndSphere(&manifold, MaxManifoldPoints, &box.Base, &sphere, slideY(0.75+gap), &cache)
				ensure(checkSpeculativePoints(&manifold, gap) == 0)
			}

			{
				manifold.PointCount = 0
				cache := SimplexCache{}
				CollideHullAndCapsule(&manifold, MaxManifoldPoints, &box.Base, &capsuleX, slideY(0.75+gap), &cache)
				ensure(checkSpeculativePoints(&manifold, gap) == 0)
			}

			{
				manifold.PointCount = 0
				cache := SATCache{}
				CollideHulls(&manifold, MaxManifoldPoints, &box.Base, &box.Base, slideY(1.0+gap), &cache)
				ensure(checkSpeculativePoints(&manifold, gap) == 0)
			}

			{
				manifold.PointCount = 0
				raised := Sphere{Vec3{0.0, 0.25 + gap, 0.0}, 0.25}
				CollideTriangleAndSphere(&manifold, MaxManifoldPoints, triangle[:], &raised)
				ensure(checkSpeculativePoints(&manifold, gap) == 0)
			}

			{
				manifold.PointCount = 0
				raised := capsuleX
				raised.Center1.Y = 0.25 + gap
				raised.Center2.Y = 0.25 + gap
				cache := SimplexCache{}
				CollideTriangleAndCapsule(&manifold, MaxManifoldPoints, triangle[:], &raised, &cache)
				ensure(checkSpeculativePoints(&manifold, gap) == 0)
			}

			{
				manifold.PointCount = 0
				raised := MakeTransformedBoxHull(0.5, 0.5, 0.5, slideY(0.5+gap))
				cache := SATCache{}
				CollideTriangleAndHull(&manifold, MaxManifoldPoints, triangle[0], triangle[1], triangle[2], 0, &raised.Base,
					&cache, true)
				ensure(checkSpeculativePoints(&manifold, gap) == 0)
			}
		}

		// Beyond the speculative distance the closed form routines report nothing
		farGap := float32(1.5 * SpeculativeDistance())

		manifold.PointCount = 0
		CollideSpheres(&manifold, MaxManifoldPoints, &bigSphere, &sphere, slideY(0.75+farGap))
		ensure(manifold.PointCount == 0)

		manifold.PointCount = 0
		CollideCapsuleAndSphere(&manifold, MaxManifoldPoints, &capsuleX, &sphere, slideY(0.5+farGap))
		ensure(manifold.PointCount == 0)
	})
})
