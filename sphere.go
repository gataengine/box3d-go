// Port of src/sphere.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.
//
// Dirk Gregorius contributed portions of this code

package box3d

import "unsafe"

// spherePoints is the one point proxy cloud of a sphere: &shape->center with count 1.
func spherePoints(shape *Sphere) []Vec3 {
	return unsafe.Slice(&shape.Center, 1)
}

// ComputeSphereMass computes mass properties of a sphere.
func ComputeSphereMass(shape *Sphere, density float32) MassData {
	center := shape.Center
	radius := shape.Radius

	// PORT: 4.0f / 3.0f is rounded first, then multiplied at run time, as in C.
	fourThirds := float32(4.0 / 3.0)
	volume := float32(fourThirds * Pi * radius * radius * radius)
	mass := float32(volume * density)
	ixx := float32(0.4 * mass * radius * radius)

	var out MassData
	out.Mass = mass
	out.Center = center

	// Inertia about the center of mass
	out.Inertia = makeDiagonalMatrix(ixx, ixx, ixx)
	return out
}

// ComputeSphereAABB computes the bounding box of a transformed sphere.
func ComputeSphereAABB(shape *Sphere, transform Transform) AABB {
	center := TransformPoint(transform, shape.Center)
	radius := shape.Radius
	extent := Vec3{radius, radius, radius}
	return AABB{Sub(center, extent), Add(center, extent)}
}

// computeSweptSphereAABB is b3ComputeSweptSphereAABB.
func computeSweptSphereAABB(shape *Sphere, xf1, xf2 Transform) AABB {
	r := Vec3{shape.Radius, shape.Radius, shape.Radius}
	center1 := TransformPoint(xf1, shape.Center)
	center2 := TransformPoint(xf2, shape.Center)
	return AABB{Sub(Min(center1, center2), r), Add(Max(center1, center2), r)}
}

// OverlapSphere overlaps a shape versus a sphere.
func OverlapSphere(shape *Sphere, shapeTransform Transform, proxy *ShapeProxy) bool {
	var input DistanceInput
	input.ProxyA = ShapeProxy{spherePoints(shape), 1, shape.Radius}
	input.ProxyB = *proxy
	input.Transform = InvMulTransforms(shapeTransform, TransformIdentity)
	input.UseRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil)
	return output.Distance < OverlapSlop()
}

// RayCastSphere ray casts versus a sphere in local space. A zero length ray is a point query. Initial overlap
// reports a hit at the ray origin with zero fraction and zero normal.
// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
func RayCastSphere(shape *Sphere, input *RayCastInput) CastOutput {
	assert(IsValidRay(input), "b3IsValidRay( input )")
	output := CastOutput{}

	p := shape.Center

	// Shift ray so sphere center is the origin
	s := Sub(input.Origin, p)

	r := shape.Radius
	rr := float32(r * r)

	d, length := GetLengthAndNormalize(input.Translation)
	if length == 0.0 {
		// zero length ray

		if LengthSquared(s) < rr {
			// initial overlap
			output.Point = input.Origin
			output.Hit = true
		}

		return output
	}

	// Find closest point on ray to origin

	// solve: dot(s + t * d, d) = 0
	t := -Dot(s, d)

	// c is the closest point on the line to the origin
	c := MulAdd(s, t, d)

	cc := Dot(c, c)

	if cc > rr {
		// closest point is outside the sphere
		return output
	}

	// Pythagoras
	h := sqrtf(rr - cc)

	fraction := t - h

	if fraction < 0.0 || float32(input.MaxFraction*length) < fraction {
		// intersection is point outside the range of the ray segment

		if LengthSquared(s) < rr {
			// initial overlap
			output.Point = input.Origin
			output.Hit = true
		}

		return output
	}

	hitPoint := MulAdd(s, fraction, d)

	output.Fraction = fraction / length

	if output.Fraction > input.MaxFraction {
		log("sphere input fraction = %g, output fraction = %g", input.MaxFraction, output.Fraction)
		output.Fraction = input.MaxFraction
	}

	output.Normal = Normalize(hitPoint)
	output.Point = MulAdd(p, shape.Radius, output.Normal)
	output.Hit = true

	return output
}

// RayCastHollowSphere ray casts versus a hollow sphere shell in local space. Unlike the solid sphere a ray starting
// inside is not an overlap: it passes through and hits the far wall.
// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
// This will do interior hits.
func RayCastHollowSphere(sphere *Sphere, input *RayCastInput) CastOutput {
	p := sphere.Center

	output := CastOutput{}

	// Shift ray so sphere center is the origin
	s := Sub(input.Origin, p)
	d := Normalize(input.Translation)

	// Find closest point on ray to origin

	// solve: dot(s + t * d, d) = 0
	t := -Dot(s, d)

	// c is the closest point on the line to the origin
	c := MulAdd(s, t, d)

	cc := Dot(c, c)
	r := sphere.Radius
	rr := float32(r * r)

	if cc > rr {
		// closest point is outside the sphere
		return output
	}

	// Pythagoras
	h := sqrtf(rr - cc)

	fraction := t - h

	if fraction < 0.0 {
		fraction = t + h
	}

	if fraction < 0.0 {
		// behind the ray
		return output
	}

	if fraction > input.MaxFraction {
		return output
	}

	hitPoint := MulAdd(s, fraction, d)

	output.Fraction = fraction
	output.Normal = Normalize(hitPoint)
	output.Point = MulAdd(p, sphere.Radius, output.Normal)
	output.Hit = true

	return output
}

// ShapeCastSphere shape casts versus a sphere. Initial overlap is treated as a miss.
func ShapeCastSphere(sphere *Sphere, input *ShapeCastInput) CastOutput {
	var pairInput ShapeCastPairInput
	pairInput.ProxyA = ShapeProxy{spherePoints(sphere), 1, sphere.Radius}
	pairInput.ProxyB = input.Proxy
	pairInput.Transform = TransformIdentity
	pairInput.TranslationB = input.Translation
	pairInput.MaxFraction = input.MaxFraction
	pairInput.CanEncroach = input.CanEncroach

	output := ShapeCast(&pairInput)
	return output
}

// collideMoverAndSphere is b3CollideMoverAndSphere.
func collideMoverAndSphere(result *PlaneResult, shape *Sphere, mover *Capsule) int {
	totalRadius := mover.Radius + shape.Radius
	closest := PointToSegmentDistance(mover.Center1, mover.Center2, shape.Center)

	// The normal points from the sphere toward the mover.
	normal, distance := GetLengthAndNormalize(Sub(closest, shape.Center))

	if distance > totalRadius {
		return 0
	}

	linearSlop := LinearSlop()
	if distance < linearSlop {
		// Deep overlap: the mover axis passes through the sphere center, so no
		// direction is preferred. Push perpendicular to the mover axis.
		axis, length := GetLengthAndNormalize(Sub(mover.Center2, mover.Center1))
		if length > linearSlop {
			normal = Perp(axis)
		} else {
			normal = Vec3AxisY
		}
		distance = 0.0
	}

	plane := Plane{normal, totalRadius - distance}
	*result = PlaneResult{plane, shape.Center, 0, 0, 0}
	return 1
}
