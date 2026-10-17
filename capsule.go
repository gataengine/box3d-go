// Port of src/capsule.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "unsafe"

// capsulePoints is the two point proxy cloud of a capsule: &shape->center1 with count 2.
// PORT: Center1 and Center2 are adjacent fields, so the slice spans both as the C pointer does.
func capsulePoints(shape *Capsule) []Vec3 {
	return unsafe.Slice(&shape.Center1, 2)
}

// ComputeCapsuleMass computes mass properties of a capsule.
func ComputeCapsuleMass(shape *Capsule, density float32) MassData {
	c1 := shape.Center1
	c2 := shape.Center2
	r := shape.Radius

	// Cylinder
	cylinderHeight := Distance(c1, c2)
	cylinderVolume := float32(Pi * r * r * cylinderHeight)
	cylinderMass := float32(cylinderVolume * density)

	// Sphere
	// PORT: ( 4.0f / 3.0f ) is rounded first, then multiplied at run time, as in C.
	fourThirds := float32(4.0 / 3.0)
	sphereVolume := float32(fourThirds * Pi * r * r * r)
	sphereMass := float32(sphereVolume * density)

	// Local accumulated inertia
	inertia := AddMM(cylinderInertia(cylinderMass, r, cylinderHeight), sphereInertia(sphereMass, r))

	steiner := float32(float32(0.125*sphereMass) * (float32(3.0*r) + float32(2.0*cylinderHeight)) * cylinderHeight)
	inertia.Cx.X += steiner
	inertia.Cz.Z += steiner

	// Align capsule axis with chosen up-axis
	rotation := Mat3Identity
	if cylinderHeight*cylinderHeight > 1000.0*fltMin {
		direction := Normalize(Sub(c2, c1))
		q := ComputeQuatBetweenUnitVectors(Vec3AxisY, direction)
		rotation = MakeMatrixFromQuat(q)
	}

	mass := sphereMass + cylinderMass
	center := MulSV(0.5, Add(c1, c2))

	var out MassData
	out.Mass = mass
	out.Center = center

	// Rotate the central inertia into the shape frame
	out.Inertia = MulMM(rotation, MulMM(inertia, Transpose(rotation)))

	return out
}

// ComputeCapsuleAABB computes the bounding box of a transformed capsule.
func ComputeCapsuleAABB(shape *Capsule, transform Transform) AABB {
	r := shape.Radius

	center1 := TransformPoint(transform, shape.Center1)
	center2 := TransformPoint(transform, shape.Center2)
	extent := Vec3{r, r, r}

	var aabb AABB
	aabb.LowerBound = Sub(Min(center1, center2), extent)
	aabb.UpperBound = Add(Max(center1, center2), extent)
	return aabb
}

// computeSweptCapsuleAABB is b3ComputeSweptCapsuleAABB.
func computeSweptCapsuleAABB(shape *Capsule, xf1, xf2 Transform) AABB {
	r := Vec3{shape.Radius, shape.Radius, shape.Radius}
	a := TransformPoint(xf1, shape.Center1)
	b := TransformPoint(xf1, shape.Center2)
	c := TransformPoint(xf2, shape.Center1)
	d := TransformPoint(xf2, shape.Center2)

	aabb := AABB{
		LowerBound: Sub(Min(Min(a, b), Min(c, d)), r),
		UpperBound: Add(Max(Max(a, b), Max(c, d)), r),
	}
	return aabb
}

// OverlapCapsule overlaps a shape versus a capsule.
func OverlapCapsule(shape *Capsule, shapeTransform Transform, proxy *ShapeProxy) bool {
	var input DistanceInput
	input.ProxyA = ShapeProxy{capsulePoints(shape), 2, shape.Radius}
	input.ProxyB = *proxy
	input.Transform = InvMulTransforms(shapeTransform, TransformIdentity)
	input.UseRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil)
	return output.Distance < OverlapSlop()
}

// RayCastCapsule ray casts versus a capsule in local space. A zero length ray is a point query. Initial overlap
// reports a hit at the ray origin with zero fraction and zero normal.
// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
func RayCastCapsule(shape *Capsule, input *RayCastInput) CastOutput {
	assert(IsValidRay(input), "b3IsValidRay( input )")

	c1 := shape.Center1
	c2 := shape.Center2
	r := shape.Radius

	// Initialize result structure
	output := CastOutput{}

	d := Sub(c2, c1)

	// Fall back to sphere if the capsule is short
	tol := float32(0.01 * LinearSlop())
	lengthSquared := LengthSquared(d)
	if lengthSquared < tol*tol {
		sphereCenter := MulSV(0.5, Add(shape.Center1, shape.Center2))
		sphere := Sphere{sphereCenter, shape.Radius}
		return RayCastSphere(&sphere, input)
	}

	// Vector from first center to ray origin.
	s := Sub(input.Origin, c1)

	// Capsule axis
	length := sqrtf(lengthSquared)
	axis := MulSV(1.0/length, d)

	// Project ray origin onto capsule axis.
	u := Dot(s, axis)

	// Closest point on infinite capsule axis, relative to c1.
	c := MulSV(u, axis)

	// Vector from closest point to ray origin
	sc := Sub(s, c)

	// Squared distance from ray origin to capsule axis
	sc2 := LengthSquared(sc)

	// Is the ray origin within the infinite cylinder along the capsule axis?
	if sc2 < r*r {
		// Clamped barycentric coordinate of ray origin projected onto capsule axis.
		uClamped := ClampFloat(u, 0.0, length)

		// The closest point on the bounded capsule segment, relative to c1.
		cp := MulSV(uClamped, axis)

		// Vector from ray origin to closest point on segment.
		scp := Sub(s, cp)

		// Squared distance of ray origin from capsule segment.
		scp2 := LengthSquared(scp)

		// Is the ray origin within the capsule?
		if scp2 < r*r {
			output.Hit = true
			output.Point = input.Origin
			return output
		}

		// The ray can hit an endcap.
		sphere := Sphere{
			Center: Add(c1, cp),
			Radius: r,
		}

		return RayCastSphere(&sphere, input)
	}

	// Ray axis. A zero length ray reaching here starts outside the capsule, so it misses.
	// Same zero length convention as b3RayCastSphere.
	dr := input.Translation
	rayAxis, rayLength := GetLengthAndNormalize(dr)
	if rayLength == 0.0 {
		return output
	}

	// Barycentric coordinate of ray end point.
	v := mulAdd(u, input.MaxFraction, Dot(dr, axis))

	// Early out: does the projected ray fall outside the capsule?
	if (u < -r && v < -r) || (length+r < u && length+r < v) {
		return output
	}

	// Compute the closest point between the ray segment and the capsule segment.
	// See Real-Time Collision Detection, section 5.1.9

	// Closest point on capsule : a1 = segment unit axis, t1 = unknown fraction
	// p1 = t1 * a1

	// Closet point on ray : a2 = ray unit axis, t2 = unknown fraction
	// p2 = s + t2 * a2

	// Closest point perpendicularity conditions.
	// dot(p2 - p1, a1) = 0
	// dot(p2 - p1, a2) = 0

	// Expand
	// dot(t1 * a1 - s - t2 * a2, a1) = 0
	// dot(t1 * a1 - s - t2 * a2, a2) = 0

	// Expand
	// t1 - dot(s, a1) - t2 * dot(a1, a2) = 0
	// t1 * dot(a1, a2) - dot(s, a2) - t2 = 0

	// Group : a12 = dot(a1, a2), sa1 = dot(s, a1), sa2 = dot(s, a2)
	// t1       - a12 * t2 = sa1
	// a12 * t1 -       t2 = sa2

	// Solve
	// https://en.wikipedia.org/wiki/Cramer%27s_rule
	// I've flipped the signs of the numerator and denominator to give a positive determinant.
	// det = 1 - a12 * a12
	// t1 = (sa1 - a12 * sa2) / det
	// t2 = (a12 * sa1 - sa2) / det

	a1 := axis
	a2 := rayAxis
	a12 := Dot(a1, a2)

	// Ray distance to the near intersection with the infinite cylinder. Length units.
	var tr float32

	det := mulSub(1.0, a12, a12)
	if det < fltEpsilon {
		// Solve the 2D problem of ray versus circle starting at the ray origin, where the circle is
		// the axial view of the infinite capsule cylinder. This works well when the ray origin is
		// not too far from the capsule axis.

		// Instead of a cross product, subtract the parallel part to get a perpendicular vector. Non-dimensional.
		perp := MulSub(a2, a12, a1)
		perp2 := LengthSquared(perp)

		// Project to origin to infinite capsule axis vector onto the perpendicular vector. beta has length units.
		beta := Dot(sc, perp)

		// Setup quadratic root finder.
		gamma := mulSub(sc2, r, r)

		// Discriminant
		disc := float32(beta*beta) - float32(perp2*gamma)

		// Casting away from the axis, or the perpendicular gap never closes to the radius.
		if beta >= 0.0 || disc < 0.0 {
			return output
		}

		// Quadratic near root. Expressed in an alternate form to avoid the (-beta - sqrt) cancellation as
		// the ray nears parallel.
		tr = gamma / (-beta + sqrtf(disc))
	} else {
		// Ray and capsules axes are not parallel.

		// Closest points between the infinite ray and the infinite capsule axis.
		invDet := 1.0 / det
		sa1 := u
		sa2 := Dot(s, a2)

		t1 := float32(mulSub(sa1, a12, sa2) * invDet)
		t2 := float32((float32(a12*sa1) - sa2) * invDet)

		// Closest points
		p1 := MulSV(t1, a1)
		p2 := MulAdd(s, t2, a2)

		// Vector from closest point on infinite capsule to infinite ray.
		g := Sub(p2, p1)

		g2 := LengthSquared(g)
		if g2 > r*r {
			// Early out: closest point on infinite ray is outside infinite cylinder.
			return output
		}

		// Intersect the infinite ray with the infinite cylinder. Like ray versus sphere this is done
		// relative to the closest point to avoid round-off errors. Length units, not a fraction.
		// https://en.wikipedia.org/wiki/Line-cylinder_intersection
		h := sqrtf(float32((float32(r*r) - g2) * invDet))

		tr = t2 - h
	}

	// Outside ray?
	if tr < 0.0 || input.MaxFraction*rayLength < tr {
		return output
	}

	// The corresponding distance on the capsule axis. Length units.
	tc := mulAdd(u, tr, a12)

	// Outside c1 end?
	if tc < 0.0 {
		// Ray cast sphere 1.
		sphere := Sphere{
			Center: c1,
			Radius: r,
		}

		return RayCastSphere(&sphere, input)
	}

	// Outside c2 end?
	if length < tc {
		// Ray cast sphere 2.
		sphere := Sphere{
			Center: c2,
			Radius: r,
		}

		return RayCastSphere(&sphere, input)
	}

	// Hit point on capsule side, relative to c1.
	p := MulAdd(s, tr, rayAxis)

	// Hit normal.
	normal := MulSub(p, tc, axis)
	normal = Normalize(normal)

	output.Point = Add(c1, p)
	output.Normal = normal
	output.Fraction = ClampFloat(tr/rayLength, 0.0, input.MaxFraction)
	output.Hit = true
	return output
}

// ShapeCastCapsule shape casts versus a capsule. Initial overlap is treated as a miss.
func ShapeCastCapsule(capsule *Capsule, input *ShapeCastInput) CastOutput {
	var pairInput ShapeCastPairInput
	pairInput.ProxyA = ShapeProxy{capsulePoints(capsule), 2, capsule.Radius}
	pairInput.ProxyB = input.Proxy
	pairInput.Transform = TransformIdentity
	pairInput.TranslationB = input.Translation
	pairInput.MaxFraction = input.MaxFraction
	pairInput.CanEncroach = input.CanEncroach

	output := ShapeCast(&pairInput)
	return output
}

// collideMoverAndCapsule is b3CollideMoverAndCapsule.
func collideMoverAndCapsule(result *PlaneResult, shape *Capsule, mover *Capsule) int {
	totalRadius := mover.Radius + shape.Radius

	approach := SegmentDistance(shape.Center1, shape.Center2, mover.Center1, mover.Center2)

	// The normal points from the shape toward the mover.
	normal, distance := GetLengthAndNormalize(Sub(approach.Point2, approach.Point1))

	if distance > totalRadius {
		return 0
	}

	linearSlop := LinearSlop()
	if distance < linearSlop {
		// Deep overlap: the core segments intersect. Pick an arbitrary direction perpendicular
		// the to capsule axis.
		moverAxis, moverLength := GetLengthAndNormalize(Sub(mover.Center2, mover.Center1))
		if moverLength > linearSlop {
			normal = Perp(moverAxis)
		} else {
			normal = Vec3AxisY
		}
		distance = 0.0
	}

	plane := Plane{normal, totalRadius - distance}
	*result = PlaneResult{plane, approach.Point1, 0, 0, 0}
	return 1
}
