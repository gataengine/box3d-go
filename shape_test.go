// Port of test/test_shape.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"math"

	. "github.com/onsi/ginkgo/v2"
)

// PORT: the file statics capsule, sphere and box are prefixed (shapeTest…) so they cannot clash
// with other test files; box is made in a BeforeEach as ShapeTest does before its subtests.
// ShapeNameTest and ShapeFlagsTest need the world (body.c, physics_world.c and, for names,
// name_cache.c); they live in shape_world_test.go behind the box3d_m3_integration build tag and
// are pending (PIt) without it (shape_world_pending_test.go). The CheckCastHit / CheckInitialOverlap
// helpers fail the spec directly, so the `if ( Check… ) return 1;` wrappers are dropped.

var (
	shapeTestCapsule = Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 1.0}
	shapeTestSphere  = Sphere{Vec3{1.0, 0.0, 0.0}, 1.0}
	shapeTestBox     BoxHull
)

const shapeTestN = 4 // N

// shapeCheckCastHit is CheckCastHit.
// Shared assertions for a surface hit. The normal points outward toward the ray,
// the point sits on the surface, and the point lies on the ray at the reported fraction.
func shapeCheckCastHit(out CastOutput, origin, translation, point, normal Vec3, fraction, tol float32) {
	GinkgoHelper()
	ensure(out.Hit)
	ensureSmall(out.Fraction-fraction, tol)
	ensureSmall(out.Point.X-point.X, tol)
	ensureSmall(out.Point.Y-point.Y, tol)
	ensureSmall(out.Point.Z-point.Z, tol)
	ensureSmall(out.Normal.X-normal.X, tol)
	ensureSmall(out.Normal.Y-normal.Y, tol)
	ensureSmall(out.Normal.Z-normal.Z, tol)

	onRay := MulAdd(origin, out.Fraction, translation)
	ensureSmall(Distance(out.Point, onRay), tol)
}

// shapeCheckInitialOverlap is CheckInitialOverlap.
// The shared initial overlap convention: a ray starting inside a solid reports the origin
// with zero fraction and no normal.
func shapeCheckInitialOverlap(out CastOutput, origin Vec3) {
	GinkgoHelper()
	ensure(out.Hit)
	ensure(out.Fraction == 0.0)
	ensureSmall(Distance(out.Point, origin), fltEpsilon)
	ensure(out.Normal.X == 0.0 && out.Normal.Y == 0.0 && out.Normal.Z == 0.0)
}

// shapeSphereHitError is SphereHitError.
// Distance, in double precision, from a single precision hit point to the analytic first
// ray/sphere intersection of the same float ray. Isolates the single precision method error:
// the reference carries no float rounding, so what remains is purely what the method lost.
func shapeSphereHitError(shape Sphere, input RayCastInput, point Vec3) float64 {
	r := float64(shape.Radius)
	sx := float64(input.Origin.X) - float64(shape.Center.X)
	sy := float64(input.Origin.Y) - float64(shape.Center.Y)
	sz := float64(input.Origin.Z) - float64(shape.Center.Z)
	tx, ty, tz := float64(input.Translation.X), float64(input.Translation.Y), float64(input.Translation.Z)
	length := math.Sqrt(tx*tx + ty*ty + tz*tz)
	dx, dy, dz := tx/length, ty/length, tz/length
	b := sx*dx + sy*dy + sz*dz
	c := sx*sx + sy*sy + sz*sz - r*r
	t := -b - math.Sqrt(b*b-c)
	ex := float64(point.X) - (float64(input.Origin.X) + t*dx)
	ey := float64(point.Y) - (float64(input.Origin.Y) + t*dy)
	ez := float64(point.Z) - (float64(input.Origin.Z) + t*dz)
	return math.Sqrt(ex*ex + ey*ey + ez*ez)
}

// shapeCapsuleHitError is CapsuleHitError.
// Same idea for the ray/infinite-cylinder intersection, used where the hit lands on the side.
func shapeCapsuleHitError(shape Capsule, input RayCastInput, point Vec3) float64 {
	ax := float64(shape.Center2.X) - float64(shape.Center1.X)
	ay := float64(shape.Center2.Y) - float64(shape.Center1.Y)
	az := float64(shape.Center2.Z) - float64(shape.Center1.Z)
	alen := math.Sqrt(ax*ax + ay*ay + az*az)
	ax /= alen
	ay /= alen
	az /= alen

	sx := float64(input.Origin.X) - float64(shape.Center1.X)
	sy := float64(input.Origin.Y) - float64(shape.Center1.Y)
	sz := float64(input.Origin.Z) - float64(shape.Center1.Z)
	tx, ty, tz := float64(input.Translation.X), float64(input.Translation.Y), float64(input.Translation.Z)
	tlen := math.Sqrt(tx*tx + ty*ty + tz*tz)
	dx, dy, dz := tx/tlen, ty/tlen, tz/tlen

	sa := sx*ax + sy*ay + sz*az
	da := dx*ax + dy*ay + dz*az
	spx, spy, spz := sx-sa*ax, sy-sa*ay, sz-sa*az
	dpx, dpy, dpz := dx-da*ax, dy-da*ay, dz-da*az
	A := dpx*dpx + dpy*dpy + dpz*dpz
	B := 2.0 * (spx*dpx + spy*dpy + spz*dpz)
	r := float64(shape.Radius)
	C := spx*spx + spy*spy + spz*spz - r*r
	tau := (-B - math.Sqrt(B*B-4.0*A*C)) / (2.0 * A)
	ex := float64(point.X) - (float64(input.Origin.X) + tau*dx)
	ey := float64(point.Y) - (float64(input.Origin.Y) + tau*dy)
	ez := float64(point.Z) - (float64(input.Origin.Z) + tau*dz)
	return math.Sqrt(ex*ex + ey*ey + ez*ez)
}

// A miss is the worst possible outcome, so fold it into the error as a large sentinel.
const shapeTestRayMiss = 1.0e30 // RAY_MISS

// Capsule along x from -2 to 2, radius 1. Reused by the capsule ray cast subtests.
var shapeTestRayCapsule = Capsule{Vec3{-2.0, 0.0, 0.0}, Vec3{2.0, 0.0, 0.0}, 1.0}

var _ = Describe("ShapeTest", func() {
	BeforeEach(func() {
		shapeTestBox = MakeBoxHull(1.0, 1.0, 1.0)
	})

	It("ShapeMassTest", func() {
		// PORT: the expected values are computed in float32 steps at run time, as the C float
		// expressions are (Go would fold a constant expression exactly).
		four, two, one := float32(4.0), float32(2.0), float32(1.0)

		// Sphere
		{
			md := ComputeSphereMass(&shapeTestSphere, 1.0)
			mass := four / 3.0 * Pi
			ensureSmall(md.Mass-mass, fltEpsilon)
			ensure(md.Center.X == 1.0 && md.Center.Y == 0.0)

			// Inertia is now about the shape center of mass, so the offset does not appear.
			inertia := two / 5.0 * mass
			ensureSmall(md.Inertia.Cx.X-inertia, fltEpsilon)
			ensureSmall(md.Inertia.Cy.Y-inertia, fltEpsilon)
			ensureSmall(md.Inertia.Cz.Z-inertia, fltEpsilon)
		}

		// Analytic box hull
		{
			md := ComputeHullMass(&shapeTestBox.Base, 1.0)
			var mass float32 = 2.0 * 2.0 * 2.0
			ensureSmall(md.Mass-mass, fltEpsilon)
			ensureSmall(md.Center.X, fltEpsilon)
			ensureSmall(md.Center.Y, fltEpsilon)
			ensureSmall(md.Center.Z, fltEpsilon)
			inertia := (one / 12.0) * mass * (2.0*2.0 + 2.0*2.0)
			ensureSmall(md.Inertia.Cx.X-inertia, 2.0*fltEpsilon)
			ensureSmall(md.Inertia.Cy.Y-inertia, 2.0*fltEpsilon)
			ensureSmall(md.Inertia.Cz.Z-inertia, 2.0*fltEpsilon)
		}

		// Translated box
		{
			offset := Vec3{0.4, -0.7, 0.1}
			transform := Transform{
				P: offset,
				Q: QuatIdentity,
			}
			h := Vec3{0.25, 0.5, 0.3}
			b1 := MakeBoxHull(h.X, h.Y, h.Z)
			b2 := MakeTransformedBoxHull(h.X, h.Y, h.Z, transform)

			m1 := ComputeHullMass(&b1.Base, 1.0)
			m2 := ComputeHullMass(&b2.Base, 1.0)

			ensureSmall(m1.Mass-m2.Mass, fltEpsilon)

			d := SubMM(b1.Base.CentralInertia, b2.Base.CentralInertia)
			ensureSmall(d.Cx.X, fltEpsilon)
			ensureSmall(d.Cx.Y, fltEpsilon)
			ensureSmall(d.Cx.Z, fltEpsilon)
			ensureSmall(d.Cy.X, fltEpsilon)
			ensureSmall(d.Cy.Y, fltEpsilon)
			ensureSmall(d.Cy.Z, fltEpsilon)
			ensureSmall(d.Cz.X, fltEpsilon)
			ensureSmall(d.Cz.Y, fltEpsilon)
			ensureSmall(d.Cz.Z, fltEpsilon)

			ensureSmall(m2.Center.X-offset.X, fltEpsilon)
			ensureSmall(m2.Center.Y-offset.Y, fltEpsilon)
			ensureSmall(m2.Center.Z-offset.Z, fltEpsilon)
		}

		// Rotated box
		{
			h1 := Vec3{0.25, 0.5, 0.3}
			h2 := Vec3{0.25, 0.3, 0.5}
			q := ComputeQuatBetweenUnitVectors(Vec3AxisY, Vec3AxisZ)
			transform := Transform{
				P: Vec3Zero,
				Q: q,
			}
			b1 := MakeTransformedBoxHull(h1.X, h1.Y, h1.Z, transform)
			b2 := MakeBoxHull(h2.X, h2.Y, h2.Z)

			m1 := ComputeHullMass(&b1.Base, 1.0)
			m2 := ComputeHullMass(&b2.Base, 1.0)

			ensureSmall(m1.Mass-m2.Mass, fltEpsilon)

			d := SubMM(b1.Base.CentralInertia, b2.Base.CentralInertia)
			ensureSmall(d.Cx.X, fltEpsilon)
			ensureSmall(d.Cx.Y, fltEpsilon)
			ensureSmall(d.Cx.Z, fltEpsilon)
			ensureSmall(d.Cy.X, fltEpsilon)
			ensureSmall(d.Cy.Y, fltEpsilon)
			ensureSmall(d.Cy.Z, fltEpsilon)
			ensureSmall(d.Cz.X, fltEpsilon)
			ensureSmall(d.Cz.Y, fltEpsilon)
			ensureSmall(d.Cz.Z, fltEpsilon)

			ensureSmall(m1.Center.X-m2.Center.X, fltEpsilon)
			ensureSmall(m1.Center.Y-m2.Center.Y, fltEpsilon)
			ensureSmall(m1.Center.Z-m2.Center.Z, fltEpsilon)
		}

		// Transformed box
		{
			offset := Vec3{0.4, -0.7, 0.1}
			h1 := Vec3{0.25, 0.5, 0.3}
			h2 := Vec3{0.25, 0.3, 0.5}
			q := ComputeQuatBetweenUnitVectors(Vec3AxisY, Vec3AxisZ)
			transform := Transform{
				P: offset,
				Q: q,
			}
			b1 := MakeTransformedBoxHull(h1.X, h1.Y, h1.Z, transform)
			b2 := MakeBoxHull(h2.X, h2.Y, h2.Z)

			m1 := ComputeHullMass(&b1.Base, 1.0)
			m2 := ComputeHullMass(&b2.Base, 1.0)

			ensureSmall(m1.Mass-m2.Mass, fltEpsilon)

			d := SubMM(b1.Base.CentralInertia, b2.Base.CentralInertia)
			ensureSmall(d.Cx.X, fltEpsilon)
			ensureSmall(d.Cx.Y, fltEpsilon)
			ensureSmall(d.Cx.Z, fltEpsilon)
			ensureSmall(d.Cy.X, fltEpsilon)
			ensureSmall(d.Cy.Y, fltEpsilon)
			ensureSmall(d.Cy.Z, fltEpsilon)
			ensureSmall(d.Cz.X, fltEpsilon)
			ensureSmall(d.Cz.Y, fltEpsilon)
			ensureSmall(d.Cz.Z, fltEpsilon)

			ensureSmall(m1.Center.X-offset.X, fltEpsilon)
			ensureSmall(m1.Center.Y-offset.Y, fltEpsilon)
			ensureSmall(m1.Center.Z-offset.Z, fltEpsilon)
		}

		// Capsule
		{
			radius := shapeTestCapsule.Radius
			length := Distance(shapeTestCapsule.Center1, shapeTestCapsule.Center2)

			// Capsule along x-axis
			md := ComputeCapsuleMass(&shapeTestCapsule, 1.0)

			// Box that fully contains capsule. Upper bound on capsule mass.
			r := MakeBoxHull(mulAdd(radius, 0.5, length), radius, radius)
			mdUpper := ComputeHullMass(&r.Base, 1.0)

			// Approximate capsule using convex hull. This should be a lower bound on the
			// capsule mass.
			var points [2 * shapeTestN * shapeTestN]Vec3
			var d float32 = Pi / (shapeTestN - 1.0)
			var angle1 float32 = -0.5 * Pi
			index := 0
			for range shapeTestN {
				s1 := sinf(angle1)
				c1 := cosf(angle1)
				var angle2 float32 = -0.5 * Pi
				for range shapeTestN {
					points[index].X = mulAdd(1.0, radius, c1)
					points[index].Y = radius * s1 * cosf(angle2)
					points[index].Z = radius * s1 * sinf(angle2)
					angle2 += d
					index += 1
				}

				angle1 += d
			}

			angle1 = 0.5 * Pi
			for range shapeTestN {
				s1 := sinf(angle1)
				c1 := cosf(angle1)
				var angle2 float32 = -0.5 * Pi
				for range shapeTestN {
					points[index].X = mulAdd(-1.0, radius, c1)
					points[index].Y = radius * s1 * cosf(angle2)
					points[index].Z = radius * s1 * sinf(angle2)
					angle2 += d
					index += 1
				}

				angle1 += d
			}

			ensure(index == 2*shapeTestN*shapeTestN)

			hull := CreateHull(points[:], 2*shapeTestN*shapeTestN, 2*shapeTestN*shapeTestN)
			mdLower := ComputeHullMass(hull, 1.0)

			ensure(mdLower.Mass < md.Mass && md.Mass < mdUpper.Mass)
			ensure(mdLower.Inertia.Cx.X < md.Inertia.Cx.X && md.Inertia.Cx.X < mdUpper.Inertia.Cx.X)
			ensure(mdLower.Inertia.Cy.Y < md.Inertia.Cy.Y && md.Inertia.Cy.Y < mdUpper.Inertia.Cy.Y)
			ensure(mdLower.Inertia.Cz.Z < md.Inertia.Cz.Z && md.Inertia.Cz.Z < mdUpper.Inertia.Cz.Z)

			DestroyHull(hull)
		}
	})

	It("ShapeAABBTest", func() {
		{
			b := ComputeSphereAABB(&shapeTestSphere, TransformIdentity)
			ensureSmall(b.LowerBound.X, fltEpsilon)
			ensureSmall(b.LowerBound.Y+1.0, fltEpsilon)
			ensureSmall(b.LowerBound.Z+1.0, fltEpsilon)
			ensureSmall(b.UpperBound.X-2.0, fltEpsilon)
			ensureSmall(b.UpperBound.Y-1.0, fltEpsilon)
			ensureSmall(b.UpperBound.Z-1.0, fltEpsilon)
		}

		{
			b := ComputeCapsuleAABB(&shapeTestCapsule, TransformIdentity)
			ensureSmall(b.LowerBound.X+2.0, fltEpsilon)
			ensureSmall(b.LowerBound.Y+1.0, fltEpsilon)
			ensureSmall(b.LowerBound.Z+1.0, fltEpsilon)
			ensureSmall(b.UpperBound.X-2.0, fltEpsilon)
			ensureSmall(b.UpperBound.Y-1.0, fltEpsilon)
			ensureSmall(b.UpperBound.Z-1.0, fltEpsilon)
		}
		{
			b := ComputeHullAABB(&shapeTestBox.Base, TransformIdentity)
			ensureSmall(b.LowerBound.X+1.0, fltEpsilon)
			ensureSmall(b.LowerBound.Y+1.0, fltEpsilon)
			ensureSmall(b.LowerBound.Z+1.0, fltEpsilon)
			ensureSmall(b.UpperBound.X-1.0, fltEpsilon)
			ensureSmall(b.UpperBound.Y-1.0, fltEpsilon)
			ensureSmall(b.UpperBound.Z-1.0, fltEpsilon)
		}
	})

	// PORT: PointInShapeTest is #if 0 upstream (and commented out of ShapeTest); not ported.

	shapeWorldSpecs()

	It("RayCastShapeTest", func() {
		input := RayCastInput{
			Origin:      Vec3{-4.0, 0.0, 0.0},
			Translation: Vec3{8.0, 0.0, 0.0},
			MaxFraction: 1.0,
		}

		{
			output := RayCastSphere(&shapeTestSphere, &input)
			ensure(output.Hit)
			ensureSmall(output.Normal.X+1.0, fltEpsilon)
			ensureSmall(output.Normal.Y, fltEpsilon)
			ensureSmall(output.Normal.Z, fltEpsilon)
			ensureSmall(output.Fraction-0.5, fltEpsilon)
		}

		{
			output := RayCastCapsule(&shapeTestCapsule, &input)
			ensure(output.Hit)
			ensureSmall(output.Normal.X+1.0, fltEpsilon)
			ensureSmall(output.Normal.Y, fltEpsilon)
			ensureSmall(output.Normal.Z, fltEpsilon)
			ensureSmall(output.Fraction-1.0/4.0, fltEpsilon)
		}

		{
			output := RayCastHull(&shapeTestBox.Base, &input)
			ensure(output.Hit)
			ensureSmall(output.Normal.X+1.0, fltEpsilon)
			ensureSmall(output.Normal.Y, fltEpsilon)
			ensureSmall(output.Normal.Z, fltEpsilon)
			ensureSmall(output.Fraction-3.0/8.0, fltEpsilon)
		}
	})

	It("RayCastSphereHitTest", func() {
		s := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}

		// Hit along each principal axis. Surface at distance 3 over a length 8 ray.
		{
			input := RayCastInput{Vec3{-4.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 1.0}
			out := RayCastSphere(&s, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{-1.0, 0.0, 0.0}, Vec3{-1.0, 0.0, 0.0},
				3.0/8.0, 1e-5)
		}
		{
			input := RayCastInput{Vec3{0.0, 4.0, 0.0}, Vec3{0.0, -8.0, 0.0}, 1.0}
			out := RayCastSphere(&s, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 1.0, 0.0},
				3.0/8.0, 1e-5)
		}
		{
			input := RayCastInput{Vec3{0.0, 0.0, -4.0}, Vec3{0.0, 0.0, 8.0}, 1.0}
			out := RayCastSphere(&s, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{0.0, 0.0, -1.0}, Vec3{0.0, 0.0, -1.0},
				3.0/8.0, 1e-5)
		}

		// Offset center, hit partway along the ray.
		{
			s2 := Sphere{Vec3{5.0, 0.0, 0.0}, 2.0}
			input := RayCastInput{Vec3{0.0, 0.0, 0.0}, Vec3{10.0, 0.0, 0.0}, 1.0}
			out := RayCastSphere(&s2, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{3.0, 0.0, 0.0}, Vec3{-1.0, 0.0, 0.0},
				0.3, 1e-5)
		}

		// Diagonal ray straight through the center.
		{
			var k float32 = 0.70710678
			input := RayCastInput{Vec3{-3.0, -3.0, 0.0}, Vec3{6.0, 6.0, 0.0}, 1.0}
			out := RayCastSphere(&s, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{-k, -k, 0.0}, Vec3{-k, -k, 0.0}, 0.382149,
				1e-4)
		}
	})

	It("RayCastSphereMissTest", func() {
		s := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}

		// Pointing away.
		{
			input := RayCastInput{Vec3{-4.0, 0.0, 0.0}, Vec3{-8.0, 0.0, 0.0}, 1.0}
			ensure(RayCastSphere(&s, &input).Hit == false)
		}
		// Passes wide of the sphere.
		{
			input := RayCastInput{Vec3{-4.0, 3.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 1.0}
			ensure(RayCastSphere(&s, &input).Hit == false)
		}
		// Aimed at the sphere but the translation stops short.
		{
			input := RayCastInput{Vec3{-4.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 0.3}
			ensure(RayCastSphere(&s, &input).Hit == false)
		}
	})

	It("RayCastSphereClipTest", func() {
		s := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}

		// The surface is reached at fraction 3/8. Straddle it with maxFraction.
		{
			input := RayCastInput{Vec3{-4.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 0.374}
			ensure(RayCastSphere(&s, &input).Hit == false)
		}
		{
			input := RayCastInput{Vec3{-4.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 0.376}
			out := RayCastSphere(&s, &input)
			ensure(out.Hit)
			ensureSmall(out.Fraction-3.0/8.0, 1e-5)
		}
	})

	It("RayCastSphereInteriorTest", func() {
		s := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}

		// Origin inside reports the origin with zero fraction.
		{
			input := RayCastInput{Vec3{0.3, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 1.0}
			out := RayCastSphere(&s, &input)
			ensure(out.Hit)
			ensure(out.Fraction == 0.0)
			ensureSmall(Distance(out.Point, input.Origin), fltEpsilon)
		}
		// Zero length ray inside.
		{
			input := RayCastInput{Vec3{0.5, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, 1.0}
			out := RayCastSphere(&s, &input)
			ensure(out.Hit)
			ensureSmall(Distance(out.Point, input.Origin), fltEpsilon)
		}
		// Zero length ray outside.
		{
			input := RayCastInput{Vec3{3.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, 1.0}
			ensure(RayCastSphere(&s, &input).Hit == false)
		}
	})

	It("RayCastSphereGrazeTest", func() {
		s := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}

		// Just inside the radius grazes a hit, just outside misses.
		{
			input := RayCastInput{Vec3{-4.0, 0.999, 0.0}, Vec3{8.0, 0.0, 0.0}, 1.0}
			ensure(RayCastSphere(&s, &input).Hit)
		}
		{
			input := RayCastInput{Vec3{-4.0, 1.001, 0.0}, Vec3{8.0, 0.0, 0.0}, 1.0}
			ensure(RayCastSphere(&s, &input).Hit == false)
		}
	})

	It("RayCastCapsuleSideTest", func() {
		// Perpendicular hit on the cylindrical side. Surface at distance 2 over a length 6 ray.
		{
			input := RayCastInput{Vec3{0.0, 3.0, 0.0}, Vec3{0.0, -6.0, 0.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 1.0, 0.0},
				1.0/3.0, 1e-5)
		}
		// Same from +z to exercise the other transverse direction.
		{
			input := RayCastInput{Vec3{0.0, 0.0, 3.0}, Vec3{0.0, 0.0, -6.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 1.0},
				1.0/3.0, 1e-5)
		}
		// Side hit nearer the c1 end.
		{
			input := RayCastInput{Vec3{-1.0, 3.0, 0.0}, Vec3{0.0, -6.0, 0.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{-1.0, 1.0, 0.0}, Vec3{0.0, 1.0, 0.0},
				1.0/3.0, 1e-5)
		}
	})

	It("RayCastCapsuleObliqueTest", func() {
		// Oblique ray in the z=0 plane. It crosses y=1 inside the cylinder span, so the
		// normal stays transverse. Exercises the non perpendicular ray/axis solve where
		// dot(axis, rayAxis) != 0.
		input := RayCastInput{Vec3{-3.0, 3.0, 0.0}, Vec3{4.0, -4.0, 0.0}, 1.0}
		out := RayCastCapsule(&shapeTestRayCapsule, &input)
		shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{-1.0, 1.0, 0.0}, Vec3{0.0, 1.0, 0.0}, 0.5,
			1e-4)
	})

	It("RayCastCapsuleCapTest", func() {
		var k float32 = 0.70710678

		// Collinear ray hits the c2 hemisphere from beyond the end.
		{
			input := RayCastInput{Vec3{5.0, 0.0, 0.0}, Vec3{-8.0, 0.0, 0.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{3.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0},
				1.0/4.0, 1e-5)
		}
		// Off-axis ray through the c2 cap center, approaching from outside the cylinder.
		{
			input := RayCastInput{Vec3{4.0, 2.0, 0.0}, Vec3{-4.0, -4.0, 0.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{2.0 + k, k, 0.0}, Vec3{k, k, 0.0}, 0.323223,
				1e-4)
		}
		// Mirror through the c1 cap center.
		{
			input := RayCastInput{Vec3{-4.0, 2.0, 0.0}, Vec3{4.0, -4.0, 0.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{-2.0 - k, k, 0.0}, Vec3{-k, k, 0.0},
				0.323223, 1e-4)
		}
	})

	It("RayCastCapsuleMissTest", func() {
		// Pointing away.
		{
			input := RayCastInput{Vec3{0.0, 3.0, 0.0}, Vec3{0.0, 4.0, 0.0}, 1.0}
			ensure(RayCastCapsule(&shapeTestRayCapsule, &input).Hit == false)
		}
		// Crosses above the axis more than a radius away.
		{
			input := RayCastInput{Vec3{0.0, 4.0, 2.0}, Vec3{0.0, -8.0, 0.0}, 1.0}
			ensure(RayCastCapsule(&shapeTestRayCapsule, &input).Hit == false)
		}
		// Aimed at the side but the translation stops short.
		{
			input := RayCastInput{Vec3{0.0, 5.0, 0.0}, Vec3{0.0, -1.0, 0.0}, 1.0}
			ensure(RayCastCapsule(&shapeTestRayCapsule, &input).Hit == false)
		}
		// Parallel to the axis and outside the cylinder.
		{
			input := RayCastInput{Vec3{0.0, 3.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 1.0}
			ensure(RayCastCapsule(&shapeTestRayCapsule, &input).Hit == false)
		}
		// Descends past the rounded c2 end, beyond cap reach.
		{
			input := RayCastInput{Vec3{4.0, 3.0, 0.0}, Vec3{0.0, -6.0, 0.0}, 1.0}
			ensure(RayCastCapsule(&shapeTestRayCapsule, &input).Hit == false)
		}
	})

	It("RayCastCapsuleInteriorTest", func() {
		// Origin on the axis between the caps.
		{
			input := RayCastInput{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, -5.0, 0.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			ensure(out.Hit)
			ensure(out.Fraction == 0.0)
			ensureSmall(Distance(out.Point, input.Origin), fltEpsilon)
		}
		// Origin inside the c2 hemisphere, past the cylinder end.
		{
			input := RayCastInput{Vec3{2.5, 0.0, 0.0}, Vec3{0.0, 0.0, 5.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			ensure(out.Hit)
			ensure(out.Fraction == 0.0)
			ensureSmall(Distance(out.Point, input.Origin), fltEpsilon)
		}
		// Zero length ray inside.
		{
			input := RayCastInput{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			ensure(out.Hit)
			ensureSmall(Distance(out.Point, input.Origin), fltEpsilon)
		}
		// Zero length ray outside.
		{
			input := RayCastInput{Vec3{0.0, 3.0, 0.0}, Vec3{0.0, 0.0, 0.0}, 1.0}
			ensure(RayCastCapsule(&shapeTestRayCapsule, &input).Hit == false)
		}
	})

	It("RayCastCapsuleDegenerateTest", func() {
		// Coincident centers collapse to a sphere.
		c := Capsule{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, 1.0}
		input := RayCastInput{Vec3{-4.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 1.0}
		out := RayCastCapsule(&c, &input)
		shapeCheckCastHit(out, input.Origin, input.Translation, Vec3{-1.0, 0.0, 0.0}, Vec3{-1.0, 0.0, 0.0},
			3.0/8.0, 1e-5)
	})

	It("RayCastCapsuleClipTest", func() {
		// The side hit occurs at fraction 1/3. Straddle it with maxFraction.
		{
			input := RayCastInput{Vec3{0.0, 3.0, 0.0}, Vec3{0.0, -6.0, 0.0}, 0.3}
			ensure(RayCastCapsule(&shapeTestRayCapsule, &input).Hit == false)
		}
		{
			input := RayCastInput{Vec3{0.0, 3.0, 0.0}, Vec3{0.0, -6.0, 0.0}, 0.5}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			ensure(out.Hit)
			ensureSmall(out.Fraction-1.0/3.0, 1e-5)
		}
	})

	// A ray within a hair of the capsule axis must still hit when it slowly converges onto the
	// surface. The closest point solver is ill conditioned in this band, so this guards the near
	// parallel fallback that intersects the infinite cylinder directly.
	It("RayCastCapsuleParallelTest", func() {
		// Capsule along y. A long ray almost parallel to the axis drifts inward from just outside the
		// cylinder and dips through the far endcap. The naive solve loses this hit to a determinant of zero.
		axisY := Capsule{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 10.0, 0.0}, 1.0}
		{
			input := RayCastInput{Vec3{1.0001, 100.0, 0.0}, Vec3{-0.001, -200.0, 0.0}, 1.0}
			out := RayCastCapsule(&axisY, &input)
			ensure(out.Hit)

			// The hit lands on the capsule surface and on the ray.
			onSeg := PointToSegmentDistance(axisY.Center1, axisY.Center2, out.Point)
			ensureSmall(Distance(out.Point, onSeg)-axisY.Radius, 1e-3)
			onRay := MulAdd(input.Origin, out.Fraction, input.Translation)
			ensureSmall(Distance(out.Point, onRay), 1e-3)
		}

		// Near parallel ray converging onto the x-axis capsule from far away.
		{
			input := RayCastInput{Vec3{-1000.0, 1.0001, 0.0}, Vec3{2000.0, -0.001, 0.0}, 1.0}
			out := RayCastCapsule(&shapeTestRayCapsule, &input)
			ensure(out.Hit)
			onSeg := PointToSegmentDistance(shapeTestRayCapsule.Center1, shapeTestRayCapsule.Center2, out.Point)
			ensureSmall(Distance(out.Point, onSeg)-shapeTestRayCapsule.Radius, 1e-3)
		}

		// Exactly parallel and outside the cylinder still misses.
		{
			input := RayCastInput{Vec3{0.0, 3.0, 0.0}, Vec3{8.0, 0.0, 0.0}, 1.0}
			ensure(RayCastCapsule(&shapeTestRayCapsule, &input).Hit == false)
		}
	})

	// Zero length rays and initial overlap behave the same across the solid shapes. A moving ray
	// and a zero length ray that both start inside report the origin with zero fraction, and a zero
	// length ray that starts outside misses.
	It("RayCastOverlapConventionTest", func() {
		zero := Vec3{0.0, 0.0, 0.0}
		ray := Vec3{8.0, 0.0, 0.0}

		// Sphere
		{
			s := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}
			inside := Vec3{0.2, 0.0, 0.0}
			outside := Vec3{3.0, 0.0, 0.0}
			moving := RayCastInput{inside, ray, 1.0}
			pointInside := RayCastInput{inside, zero, 1.0}
			pointOutside := RayCastInput{outside, zero, 1.0}
			shapeCheckInitialOverlap(RayCastSphere(&s, &moving), inside)
			shapeCheckInitialOverlap(RayCastSphere(&s, &pointInside), inside)
			ensure(RayCastSphere(&s, &pointOutside).Hit == false)
		}

		// Capsule
		{
			inside := Vec3{0.0, 0.0, 0.0}
			outside := Vec3{0.0, 3.0, 0.0}
			moving := RayCastInput{inside, ray, 1.0}
			pointInside := RayCastInput{inside, zero, 1.0}
			pointOutside := RayCastInput{outside, zero, 1.0}
			shapeCheckInitialOverlap(RayCastCapsule(&shapeTestRayCapsule, &moving), inside)
			shapeCheckInitialOverlap(RayCastCapsule(&shapeTestRayCapsule, &pointInside), inside)
			ensure(RayCastCapsule(&shapeTestRayCapsule, &pointOutside).Hit == false)
		}

		// Hull
		{
			inside := Vec3{0.3, 0.2, 0.1}
			outside := Vec3{3.0, 0.0, 0.0}
			moving := RayCastInput{inside, ray, 1.0}
			pointInside := RayCastInput{inside, zero, 1.0}
			pointOutside := RayCastInput{outside, zero, 1.0}
			shapeCheckInitialOverlap(RayCastHull(&shapeTestBox.Base, &moving), inside)
			shapeCheckInitialOverlap(RayCastHull(&shapeTestBox.Base, &pointInside), inside)
			ensure(RayCastHull(&shapeTestBox.Base, &pointOutside).Hit == false)
		}
	})

	It("RayCastFarOriginTest", func() {
		s := Sphere{Vec3{0.0, 0.0, 0.0}, 1.0}
		c := Capsule{Vec3{-2.0, 0.0, 0.0}, Vec3{2.0, 0.0, 0.0}, 1.0}

		// (0,0,1) lies on the unit sphere and on the capsule side. The ray dives in from a far origin
		// along H + D*u for a fan of directions u skewed off the surface normal, so the capsule solve
		// sees a real perpendicular gap rather than a free exact cancellation.
		h := Vec3{0.0, 0.0, 1.0}
		offsets := []float32{-0.7, 0.0, 0.7}
		distances := []float32{1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7}

		worstSphere := make([]float64, arrayCount(distances))
		worstCapsule := make([]float64, arrayCount(distances))

		GinkgoWriter.Printf("    worst hit point error over a fan of skew rays, by origin distance:\n")
		GinkgoWriter.Printf("    %-9s %-13s %-13s\n", "distance", "sphere", "capsule")
		for i := range arrayCount(distances) {
			d := distances[i]
			maxS, maxC := 0.0, 0.0

			for ia := range arrayCount(offsets) {
				for ib := range arrayCount(offsets) {
					u := Normalize(Vec3{offsets[ia], offsets[ib], 1.0})
					origin := MulAdd(h, d, u)
					translation := MulSV(-2.0*d, u)
					input := RayCastInput{origin, translation, 1.0}

					os := RayCastSphere(&s, &input)
					oc := RayCastCapsule(&c, &input)

					errS := shapeTestRayMiss
					if os.Hit {
						errS = shapeSphereHitError(s, input, os.Point)
					}
					errC := shapeTestRayMiss
					if oc.Hit {
						errC = shapeCapsuleHitError(c, input, oc.Point)
					}

					maxS = max(errS, maxS)
					maxC = max(errC, maxC)
				}
			}

			worstSphere[i] = maxS
			worstCapsule[i] = maxC
			GinkgoWriter.Printf("    %-9.0e %-13.3e %-13.3e\n", d, maxS, maxC)
		}

		// The closest point formulation keeps the error at the single precision floor: it grows only
		// linearly with origin distance, error ~ distance * FLT_EPSILON, with no catastrophic loss. This
		// holds out to a million units. At ten million the origin coordinates carry a meter sized ULP,
		// larger than the unit radius, so the ray genuinely drops the hit. That row is printed for insight
		// but left unasserted rather than baking in the breakdown.
		for i := range arrayCount(distances) {
			if distances[i] < 1.0e7 {
				floor := 16.0*float64(distances[i])*float64(fltEpsilon) + 2.0e-6
				ensure(worstSphere[i] < floor)
				ensure(worstCapsule[i] < floor)
			}
		}

		// Still a clean sub-meter hit at a million units out.
		ensure(worstSphere[5] < 0.5 && worstCapsule[5] < 0.5)
	})
})
