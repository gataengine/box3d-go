// Port of test/test_sat.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// PORT: test_sat.c and test_manifold.c both define kRoot2, kAxisY, kAxisZ, ExactQuat and
// ExactRotation with identical bodies. One package holds both test files, so they are defined once:
// here (constants, exactRotation) and in hull_test.go (exactQuat, same body as here).

var (
	kRoot2 float32 = 1.41421356
	kAxisY         = Vec3{0.0, 1.0, 0.0}
	kAxisZ         = Vec3{0.0, 0.0, 1.0}
)

// exactRotation is ExactRotation.
// b3ComputeCosSin is a coarse rational approximation, so the fixtures build their rotations from
// libm to keep the analytic edge and face positions exact.
func exactRotation(axis Vec3, radians float32) Transform {
	return Transform{Vec3Zero, exactQuat(axis, radians)}
}

// sepAlong is SepAlong.
// Directed separation along a unit axis n pointing from A to B. Positive means B clears A along n.
// This is the projection gap min over B minus max over A, which is exactly what a valid separating
// axis measures, so the winning face or edge axis must reproduce it.
func sepAlong(hullA, hullB *HullData, xfB Transform, n Vec3) float32 {
	pointsA := GetHullPoints(hullA)
	maxA := -fltMax
	for i := 0; i < int(hullA.VertexCount); i++ {
		maxA = MaxFloat(maxA, Dot(n, pointsA[i]))
	}

	pointsB := GetHullPoints(hullB)
	minB := fltMax
	for i := 0; i < int(hullB.VertexCount); i++ {
		minB = MinFloat(minB, Dot(n, TransformPoint(xfB, pointsB[i])))
	}

	return minB - maxA
}

// tryAxis is TryAxis.
func tryAxis(hullA, hullB *HullData, xfB Transform, n Vec3, best *float32, bestNormal *Vec3) {
	s := sepAlong(hullA, hullB, xfB, n)
	if s > *best {
		*best = s
		*bestNormal = n
	}
}

// oracleSeparation is OracleSeparation.
// Brute force SAT. The maximum separation over every face normal and every edge cross product is
// the true answer the SIMD query has to match. Each undirected axis is tried both ways so the
// winner comes out oriented from A to B with no sign bookkeeping.
func oracleSeparation(hullA, hullB *HullData, xfB Transform, outNormal *Vec3) float32 {
	best := -fltMax
	bestNormal := Vec3{0.0, 0.0, 0.0}

	planesA := GetHullPlanes(hullA)
	for i := 0; i < int(hullA.FaceCount); i++ {
		tryAxis(hullA, hullB, xfB, planesA[i].Normal, &best, &bestNormal)
		tryAxis(hullA, hullB, xfB, Neg(planesA[i].Normal), &best, &bestNormal)
	}

	planesB := GetHullPlanes(hullB)
	for i := 0; i < int(hullB.FaceCount); i++ {
		n := RotateVector(xfB.Q, planesB[i].Normal)
		tryAxis(hullA, hullB, xfB, n, &best, &bestNormal)
		tryAxis(hullA, hullB, xfB, Neg(n), &best, &bestNormal)
	}

	edgesA := GetHullEdges(hullA)
	pointsA := GetHullPoints(hullA)
	edgesB := GetHullEdges(hullB)
	pointsB := GetHullPoints(hullB)

	// Half edges are stored as adjacent twin pairs, so the even index and its successor bound one edge.
	for i := 0; i < int(hullA.EdgeCount); i += 2 {
		dirA := Sub(pointsA[edgesA[i+1].Origin], pointsA[edgesA[i].Origin])
		for j := 0; j < int(hullB.EdgeCount); j += 2 {
			dirB := RotateVector(xfB.Q, Sub(pointsB[edgesB[j+1].Origin], pointsB[edgesB[j].Origin]))
			cross := Cross(dirA, dirB)

			// Near parallel edges give no useful axis and cannot be the true maximum.
			if Dot(cross, cross) < 1e-10 {
				continue
			}

			cross = Normalize(cross)
			tryAxis(hullA, hullB, xfB, cross, &best, &bestNormal)
			tryAxis(hullA, hullB, xfB, Neg(cross), &best, &bestNormal)
		}
	}

	*outNormal = bestNormal
	return best
}

// satSeed is g_seed in test_sat.c.
var satSeed uint32 = 987654321

// satNextFloat is NextFloat in test_sat.c.
func satNextFloat(lower, upper float32) float32 {
	satSeed = 1664525*satSeed + 1013904223
	t := float32(satSeed>>8) / float32(1<<24)
	return mulAdd(lower, t, upper-lower)
}

// satNextDirection is NextDirection in test_sat.c.
func satNextDirection() Vec3 {
	var v Vec3
	v.X = satNextFloat(-1.0, 1.0)
	v.Y = satNextFloat(-1.0, 1.0)
	v.Z = satNextFloat(-1.0, 1.0)
	return Normalize(v)
}

var _ = Describe("SeparatingAxisTest", func() {
	// Two axis aligned cubes pushed apart along x. A's plus x face is the separating axis, the gap is
	// large enough to trip the speculative early out, and the reference vertex on B is its minus x side.
	It("FaceAxisASeparatedTest", func() {
		hullA := MakeBoxHull(0.5, 0.5, 0.5)
		hullB := MakeBoxHull(0.5, 0.5, 0.5)

		xfB := Transform{Vec3{1.2, 0.0, 0.0}, QuatIdentity}
		aq := computeSeparatingAxis(&hullA.Base, &hullB.Base, xfB, true)
		q := getBestAxis(&aq)

		ensure(q.typ == FaceAxisA)
		ensureSmall(q.separation-0.2, 1e-5)
		ensureSmall(q.normal.X-1.0, 1e-6)
		ensureSmall(q.normal.Y, 1e-6)
		ensureSmall(q.normal.Z, 1e-6)

		planesA := GetHullPlanes(&hullA.Base)
		ensureSmall(planesA[q.indexA].Normal.X-1.0, 1e-6)

		pointsB := GetHullPoints(&hullB.Base)
		ensureSmall(pointsB[q.indexB].X+0.5, 1e-6)

		ensureSmall(sepAlong(&hullA.Base, &hullB.Base, xfB, q.normal)-q.separation, 1e-5)
	})

	// A yawed 45 degrees about z so all of its faces sit oblique to x. Only B still has a face square to
	// the gap, so the reference has to be B's minus x face. This is the case that ties when both hulls
	// are aligned, and the tie is what a face A bias would hide.
	It("FaceAxisBSeparatedTest", func() {
		hullA := MakeTransformedBoxHull(0.5, 0.5, 0.5, exactRotation(kAxisZ, 0.25*Pi))
		hullB := MakeBoxHull(0.5, 0.5, 0.5)

		aExtent := float32(0.5 * kRoot2)
		gap := float32(0.2)
		d := aExtent + 0.5 + gap
		xfB := Transform{Vec3{d, 0.0, 0.0}, QuatIdentity}

		aq := computeSeparatingAxis(&hullA.Base, &hullB.Base, xfB, true)
		q := getBestAxis(&aq)

		ensure(q.typ == FaceAxisB)
		ensureSmall(q.separation-gap, 1e-5)
		ensureSmall(q.normal.X-1.0, 1e-5)
		ensureSmall(q.normal.Y, 1e-5)
		ensureSmall(q.normal.Z, 1e-5)

		planesB := GetHullPlanes(&hullB.Base)
		ensureSmall(planesB[q.indexB].Normal.X+1.0, 1e-5)

		pointsA := GetHullPoints(&hullA.Base)
		ensureSmall(pointsA[q.indexA].X-aExtent, 1e-4)

		ensureSmall(sepAlong(&hullA.Base, &hullB.Base, xfB, q.normal)-q.separation, 1e-4)
	})

	// Well beyond the speculative distance the face phase must report the axis and separation without
	// running the edge phase. A sign slip in the early out would read as deep overlap.
	It("FaceFarSeparatedTest", func() {
		hullA := MakeBoxHull(0.5, 0.5, 0.5)
		hullB := MakeBoxHull(0.5, 0.5, 0.5)

		xfB := Transform{Vec3{3.0, 0.0, 0.0}, QuatIdentity}
		aq := computeSeparatingAxis(&hullA.Base, &hullB.Base, xfB, true)
		q := getBestAxis(&aq)

		ensure(q.typ == FaceAxisA)
		ensureSmall(q.separation-2.0, 1e-5)
		ensureSmall(q.normal.X-1.0, 1e-6)

		var oracleNormal Vec3
		oracleSep := oracleSeparation(&hullA.Base, &hullB.Base, xfB, &oracleNormal)
		ensureSmall(q.separation-oracleSep, 1e-5)
	})

	// The support bias is derived from each hull's AABB, not from an origin assumed to sit inside it. A
	// box built far from its own local origin is the case the old diagonal bias got wrong. A is yawed
	// and pushed out to x = 5, so the face B query has to run getSupport over vertices clustered near
	// x = 5 and still pick the right one.
	It("OffsetFaceAxisBTest", func() {
		cx := float32(5.0)
		placeA := Transform{Vec3{cx, 0.0, 0.0}, exactQuat(kAxisZ, 0.25*Pi)}
		hullA := MakeTransformedBoxHull(0.5, 0.5, 0.5, placeA)
		hullB := MakeBoxHull(0.5, 0.5, 0.5)

		aExtent := cx + float32(0.5*kRoot2)
		gap := float32(0.2)
		d := aExtent + 0.5 + gap
		xfB := Transform{Vec3{d, 0.0, 0.0}, QuatIdentity}

		aq := computeSeparatingAxis(&hullA.Base, &hullB.Base, xfB, true)
		q := getBestAxis(&aq)

		ensure(q.typ == FaceAxisB)
		ensureSmall(q.separation-gap, 1e-4)
		ensureSmall(q.normal.X-1.0, 1e-5)
		ensureSmall(q.normal.Y, 1e-5)
		ensureSmall(q.normal.Z, 1e-5)

		planesB := GetHullPlanes(&hullB.Base)
		ensureSmall(planesB[q.indexB].Normal.X+1.0, 1e-5)

		pointsA := GetHullPoints(&hullA.Base)
		ensureSmall(pointsA[q.indexA].X-aExtent, 1e-4)

		var oracleNormal Vec3
		oracleSep := oracleSeparation(&hullA.Base, &hullB.Base, xfB, &oracleNormal)
		ensureSmall(q.separation-oracleSep, 1e-4)
		ensureSmall(sepAlong(&hullA.Base, &hullB.Base, xfB, q.normal)-q.separation, 1e-4)
	})

	// Cube A yawed 45 about y presents an edge along y at x = h*root2. Cube B rolled 45 about z presents
	// an edge along z at x = -h*root2. Sliding B along x makes those edges the closest features, so the
	// axis is x and the separation is d - root2. The sweep straddles contact into overlap while staying
	// under the speculative distance, so no face axis can short circuit the edge phase. The overlap rows
	// pin the maximum separation, which is where a premature edge early out shows up.
	It("EdgePairSweepTest", func() {
		hullA := MakeTransformedBoxHull(0.5, 0.5, 0.5, exactRotation(kAxisY, 0.25*Pi))
		hullB := MakeTransformedBoxHull(0.5, 0.5, 0.5, exactRotation(kAxisZ, 0.25*Pi))

		distances := []float32{1.43, 1.42, kRoot2, 1.40, 1.38, 1.35, 1.30}

		for i := 0; i < arrayCount(distances); i++ {
			d := distances[i]
			expected := d - kRoot2

			xfB := Transform{Vec3{d, 0.0, 0.0}, QuatIdentity}
			aq := computeSeparatingAxis(&hullA.Base, &hullB.Base, xfB, true)
			q := getBestAxis(&aq)

			ensure(q.typ == EdgePairAxis)
			ensureSmall(q.separation-expected, 1e-4)
			ensureSmall(q.normal.X-1.0, 1e-4)
			ensureSmall(q.normal.Y, 1e-4)
			ensureSmall(q.normal.Z, 1e-4)

			// Half edge indices are the even twin of each pair.
			ensure((q.indexA & 1) == 0)
			ensure((q.indexB & 1) == 0)

			var oracleNormal Vec3
			oracleSep := oracleSeparation(&hullA.Base, &hullB.Base, xfB, &oracleNormal)
			ensureSmall(q.separation-oracleSep, 1e-4)
			ensureSmall(sepAlong(&hullA.Base, &hullB.Base, xfB, q.normal)-q.separation, 1e-4)
		}
	})

	// A wide net over randomly oriented box pairs, from clearly separated to deep overlap. The brute
	// force oracle is the source of truth. Whatever axis the query returns it must be a unit vector that
	// reproduces its own separation, it must not exceed the true maximum, and it must not fall short of
	// it. The short fall bound is what a suboptimal edge pick trips. Thin boxes are mixed in because a
	// crossed pair of long edges stays the axis of minimum penetration even deep in overlap, which is
	// the hardest case for the edge phase.
	It("SeparatingAxisOracleTest", func() {
		satSeed = 987654321

		separated := 0
		penetrating := 0
		edgeWins := 0
		worstBandShortfall := float32(0.0)
		worstExcess := float32(0.0)

		for i := range 4000 {
			var halfA, halfB Vec3
			var reach float32

			if i&1 != 0 {
				// Thin crossed beams driven into deep overlap.
				halfA.X = satNextFloat(1.0, 1.6)
				halfA.Y = satNextFloat(0.08, 0.15)
				halfA.Z = satNextFloat(0.08, 0.15)
				halfB.X = satNextFloat(1.0, 1.6)
				halfB.Y = satNextFloat(0.08, 0.15)
				halfB.Z = satNextFloat(0.08, 0.15)
				reach = satNextFloat(0.0, 0.4)
			} else {
				// Chunky boxes across the whole separated to overlapping range.
				halfA.X = satNextFloat(0.3, 0.8)
				halfA.Y = satNextFloat(0.3, 0.8)
				halfA.Z = satNextFloat(0.3, 0.8)
				halfB.X = satNextFloat(0.3, 0.8)
				halfB.Y = satNextFloat(0.3, 0.8)
				halfB.Z = satNextFloat(0.3, 0.8)
				reach = satNextFloat(0.0, 1.7)
			}

			// Rotate A in place so it stays centered on the origin, which keeps the support bias valid.
			axisA := satNextDirection()
			angleA := satNextFloat(0.0, Pi)
			hullA := MakeTransformedBoxHull(halfA.X, halfA.Y, halfA.Z, exactRotation(axisA, angleA))
			hullB := MakeBoxHull(halfB.X, halfB.Y, halfB.Z)

			var xfB Transform
			xfB.P = MulSV(reach, satNextDirection())
			axisB := satNextDirection()
			angleB := satNextFloat(0.0, Pi)
			xfB.Q = exactQuat(axisB, angleB)

			aq := computeSeparatingAxis(&hullA.Base, &hullB.Base, xfB, true)
			q := getBestAxis(&aq)

			var oracleNormal Vec3
			oracleSep := oracleSeparation(&hullA.Base, &hullB.Base, xfB, &oracleNormal)

			ensureSmall(Length(q.normal)-1.0, 1e-3)
			ensureSmall(sepAlong(&hullA.Base, &hullB.Base, xfB, q.normal)-q.separation, 2e-3)

			worstExcess = MaxFloat(worstExcess, q.separation-oracleSep)

			// Never overstate the true maximum. This holds on every path since the reported axis is real.
			ensure(q.separation <= oracleSep+2e-3)

			// The scan stops at the first axis that clears the speculative band, so a widely separated pair
			// reports some separating axis rather than the deepest one. That is all the collide path needs,
			// since past the band it drops the pair either way. Inside the band nothing stops the scan, so
			// the result there is the exhaustive maximum, and that is what a manifold gets built from.
			if q.separation <= SpeculativeDistance() {
				worstBandShortfall = MaxFloat(worstBandShortfall, oracleSep-q.separation)

				// A suboptimal axis pick lands a discrete step below the true maximum. The bound sits well
				// under that step yet clear of the edge normalization noise floor near 6e-4.
				ensure(q.separation >= oracleSep-4e-3)
			}

			if oracleSep > 0.0 {
				separated += 1
			} else {
				penetrating += 1
			}

			if q.typ == EdgePairAxis {
				edgeWins += 1
			}
		}

		GinkgoWriter.Printf("    oracle: separated=%d penetrating=%d edgeWins=%d worstBandShortfall=%.2e worstExcess=%.2e\n", separated,
			penetrating, edgeWins, worstBandShortfall, worstExcess)

		// The sweep has to cover both regimes and actually drive the edge path.
		ensure(separated > 100)
		ensure(penetrating > 100)
		ensure(edgeWins > 20)
	})

	// The oracle over hulls whose vertices sit far from their own local origin, which the AABB based
	// bias must handle. Half the pairs offset A in its own frame, half offset B, and B is placed so the
	// two centers land near each other across the separated to overlapping range. Any bias that assumed
	// the origin was inside the hull would pick the wrong support and blow the separation.
	It("OffsetHullOracleTest", func() {
		satSeed = 24681012

		separated := 0
		penetrating := 0
		edgeWins := 0
		worstBandShortfall := float32(0.0)
		worstExcess := float32(0.0)
		worstConsistency := float32(0.0)

		for i := range 3000 {
			var halfA, halfB Vec3
			halfA.X = satNextFloat(0.3, 0.8)
			halfA.Y = satNextFloat(0.3, 0.8)
			halfA.Z = satNextFloat(0.3, 0.8)
			halfB.X = satNextFloat(0.3, 0.8)
			halfB.Y = satNextFloat(0.3, 0.8)
			halfB.Z = satNextFloat(0.3, 0.8)

			offsetScale := satNextFloat(2.0, 6.0)
			offset := MulSV(offsetScale, satNextDirection())
			reach := satNextFloat(0.0, 1.7)
			dir := satNextDirection()

			var hullA BoxHull
			var hullB BoxHull
			var xfB Transform

			if i&1 != 0 {
				// A carries the offset in its own frame. B is centered and dropped near A's center.
				axisA := satNextDirection()
				angleA := satNextFloat(0.0, Pi)
				placeA := Transform{offset, exactQuat(axisA, angleA)}
				hullA = MakeTransformedBoxHull(halfA.X, halfA.Y, halfA.Z, placeA)
				hullB = MakeBoxHull(halfB.X, halfB.Y, halfB.Z)
				axisB := satNextDirection()
				angleB := satNextFloat(0.0, Pi)
				xfB = Transform{MulAdd(offset, reach, dir), exactQuat(axisB, angleB)}
			} else {
				// B carries the offset in its own frame. Place it so its world center lands near the origin.
				axisB := satNextDirection()
				angleB := satNextFloat(0.0, Pi)
				qB := exactQuat(axisB, angleB)
				hullA = MakeBoxHull(halfA.X, halfA.Y, halfA.Z)
				hullB = MakeOffsetBoxHull(halfB.X, halfB.Y, halfB.Z, offset)
				xfB = Transform{Sub(MulSV(reach, dir), RotateVector(qB, offset)), qB}
			}

			aq := computeSeparatingAxis(&hullA.Base, &hullB.Base, xfB, true)
			q := getBestAxis(&aq)

			var oracleNormal Vec3
			oracleSep := oracleSeparation(&hullA.Base, &hullB.Base, xfB, &oracleNormal)

			consistency := AbsFloat(sepAlong(&hullA.Base, &hullB.Base, xfB, q.normal) - q.separation)
			worstConsistency = MaxFloat(worstConsistency, consistency)
			worstExcess = MaxFloat(worstExcess, q.separation-oracleSep)

			// Consistency is the sharp bias check: a wrong support pick on an offset hull would throw the
			// returned normal off its own separation by a vertex spacing, not a noise floor. It holds on
			// every path, early out or not, so it stays unconditional. The oracle bounds are looser since
			// differencing coordinates out at radius six costs a few more digits.
			ensureSmall(Length(q.normal)-1.0, 1e-3)
			ensure(consistency < 1e-3)
			ensure(q.separation <= oracleSep+3e-3)

			// Only the in band result feeds a manifold, and only there does the scan run to completion.
			if q.separation <= SpeculativeDistance() {
				worstBandShortfall = MaxFloat(worstBandShortfall, oracleSep-q.separation)
				ensure(q.separation >= oracleSep-8e-3)
			}

			if oracleSep > 0.0 {
				separated += 1
			} else {
				penetrating += 1
			}

			if q.typ == EdgePairAxis {
				edgeWins += 1
			}
		}

		GinkgoWriter.Printf(
			"    offset: separated=%d penetrating=%d edgeWins=%d worstBandShortfall=%.2e worstExcess=%.2e worstConsistency=%.2e\n",
			separated, penetrating, edgeWins, worstBandShortfall, worstExcess, worstConsistency)

		ensure(separated > 100)
		ensure(penetrating > 100)
		ensure(edgeWins > 20)
	})
})
