// Port of src/math_internal.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

const (
	twoPi      float32 = 6.283185307 // B3_TWO_PI
	piOverTwo  float32 = 1.570796327 // B3_PI_OVER_TWO
	piOverFour float32 = 0.785398163 // B3_PI_OVER_FOUR
	sqrt3      float32 = 1.732050808 // B3_SQRT3
)

// todo eliminate this
var bounds3Empty = AABB{Vec3{fltMax, fltMax, fltMax}, Vec3{-fltMax, -fltMax, -fltMax}}

// matrix2 is b3Matrix2.
type matrix2 struct {
	cx, cy Vec2
}

// triangle is b3Triangle.
type triangle struct {
	vertices   [3]Vec3
	i1, i2, i3 int32
	flags      int32
}

// trianglePoint is b3TrianglePoint.
type trianglePoint struct {
	point   Vec3
	feature TriangleFeature
}

// shapeExtent is b3ShapeExtent.
type shapeExtent struct {
	minExtent float32
	maxExtent Vec3
}

// closestPointOnTriangle is implemented in math_functions.go.

// PORT: the following declarations are implemented with the upstream file that defines them:
// b3IntersectSegmentTriangle, b3IntersectSegmentSphere, b3ComputeMassProperties,
// b3IsValidMassData, b3GetProxySupport, b3GetPointSupport and b3IsSweepNormalized.
// sphereInertia, cylinderInertia and boxInertia are in math_functions.go.

// alignUp8 aligns up to 8 byte alignment.
func alignUp8(x int) int {
	return (x + 7) &^ 7
}

// ceilingInt is b3CeilingInt.
// https://en.wikipedia.org/wiki/Floor_and_ceiling_functions
func ceilingInt(numerator, denominator int) int {
	validate(denominator > 0, "denominator > 0")
	return (numerator + denominator - 1) / denominator
}

// ceilingPow2 assumes denominator == 2^exponent
func ceilingPow2(numerator, denominator, exponent int) int {
	validate(exponent > 0 && (denominator == 1<<exponent), "exponent > 0 && ( denominator == 1 << exponent )")
	return (numerator + denominator - 1) >> exponent
}

func dot2(v1, v2 Vec2) float32 {
	return float32(v1.X*v2.X) + float32(v1.Y*v2.Y)
}

func length2(v Vec2) float32 {
	return sqrtf(dot2(v, v))
}

func lengthSquared2(v Vec2) float32 {
	return dot2(v, v)
}

func minVec2(v1, v2 Vec2) Vec2 {
	var v Vec2
	v.X = MinFloat(v1.X, v2.X)
	v.Y = MinFloat(v1.Y, v2.Y)
	return v
}

func maxVec2(v1, v2 Vec2) Vec2 {
	var v Vec2
	v.X = MaxFloat(v1.X, v2.X)
	v.Y = MaxFloat(v1.Y, v2.Y)
	return v
}

// store is b3Store.
func store(dst []float32, src Vec3) {
	dst[0] = src.X
	dst[1] = src.Y
	dst[2] = src.Z
}

func clampLength(v Vec3, maxLength float32) Vec3 {
	lengthSq := LengthSquared(v)
	if lengthSq <= maxLength*maxLength {
		return v
	}

	length := sqrtf(lengthSq)
	return MulSV(maxLength/length, v)
}

// arbitraryPerp assumes v is a unit vector
func arbitraryPerp(v Vec3) Vec3 {
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	var p Vec3
	if v.X < -0.5 || 0.5 < v.X {
		// x is non-zero and it should not go into the x component
		// dot([ay + bz, cx, dx], [x, y, z]) = ayx + bzx + cxy + dzx
		// for the dot product to be zero need: c = -a, d = -b
		var a float32 = 0.67
		var b float32 = -0.42
		p = Vec3{float32(a*v.Y) + float32(b*v.Z), -a * v.X, -b * v.X}
	} else if v.Y < -0.5 || 0.5 < v.Y {
		// y is non-zero and it should not go into the y component
		// p = [ay, bx + cz, dy]
		// axy + bxy + cyz + dyz = 0
		// b = -a, d = -c
		var a float32 = 0.67
		var c float32 = -0.42
		p = Vec3{a * v.Y, float32(-a*v.X) + float32(c*v.Z), -c * v.Y}
	} else {
		// This would trip if the input is not a unit vector
		validate(v.Z < -0.5 || 0.5 < v.Z, "v.z < -0.5f || 0.5f < v.z")

		// z is non-zero and it should not go into the z component
		// p = [az, bz, cx + dy]
		// axz + byz + cxz + dyz = 0
		// c = -a, d = -b
		var a float32 = 0.67
		var b float32 = -0.42
		p = Vec3{a * v.Z, b * v.Z, float32(-a*v.X) - float32(b*v.Y)}
	}

	validate(LengthSquared(p) > 0.1, "b3LengthSquared( p ) > 0.1f")
	validate(AbsFloat(Dot(p, v)) < 100.0*fltEpsilon, "b3AbsFloat( b3Dot( p, v ) ) < 100.0f * FLT_EPSILON")

	return Normalize(p)
}

func quatFromExponentialMap(v Vec3) Quat {
	// Exponential map (Grassia)
	var threshold float32 = 0.018581361

	angle := Length(v)
	if angle < threshold {
		// Taylor expansion
		var out Quat
		out.V = MulSV(0.5+angle*angle/48.0, v)
		out.S = Cos(0.5 * angle)

		return out
	}

	return MakeQuatFromAxisAngle(MulSV(1.0/angle, v), angle)
}

// integrateRotation integrates rotation from angular velocity
//
//	q1 initial rotation
//	deltaRotation the angular displacement vector in radians (angular velocity multiplied by the time step)
//
// q2 = q1 + 0.5 * omega * q1
func integrateRotation(q1 Quat, deltaRotation Vec3) Quat {
	// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
	qd := Quat{MulSV(0.5, deltaRotation), 0.0}
	qd = MulQuat(qd, q1)
	q2 := Quat{Add(q1.V, qd.V), qd.S + q1.S}
	q2 = NormalizeQuat(q2)
	return q2
}

// deltaQuatToRotation is the pseudo angular velocity from a quaternion target
// w = 2 * (target - q) * conj(q)
func deltaQuatToRotation(q, target Quat) Vec3 {
	s := q
	if DotQuat(q, target) < 0.0 {
		// Correct polarity
		s = NegateQuat(q)
	}

	diff := Quat{Sub(target.V, s.V), target.S - s.S}
	product := MulQuat(diff, Conjugate(s))
	return MulSV(2.0, product.V)
}

func scalarTripleProduct(a, b, c Vec3) float32 {
	var d Vec3
	d.X = float32(b.Y*c.Z) - float32(b.Z*c.Y)
	d.Y = float32(b.Z*c.X) - float32(b.X*c.Z)
	d.Z = float32(b.X*c.Y) - float32(b.Y*c.X)
	return float32(a.X*d.X) + float32(a.Y*d.Y) + float32(a.Z*d.Z)
}

// getByIndex gets a value by index. Avoid undefined behavior of code like (&v.x)[2].
func getByIndex(v Vec3, index int) float32 {
	validate(0 <= index && index < 3, "0 <= index && index < 3")
	temp := [3]float32{v.X, v.Y, v.Z}
	return temp[index]
}

func majorAxis(v Vec3) int {
	if v.X < v.Y {
		if v.Y < v.Z {
			return 2
		}
		return 1
	}
	if v.X < v.Z {
		return 2
	}
	return 0
}

func minElement(v Vec3) float32 {
	return MinFloat(v.X, MinFloat(v.Y, v.Z))
}

func maxElement(v Vec3) float32 {
	return MaxFloat(v.X, MaxFloat(v.Y, v.Z))
}

func maxElementIndex(v Vec3) int {
	if v.X < v.Y {
		if v.Y < v.Z {
			return 2
		}
		return 1
	}
	if v.X < v.Z {
		return 2
	}
	return 0
}

func add2(a, b Vec2) Vec2 {
	c := Vec2{a.X + b.X, a.Y + b.Y}
	return c
}

func sub2(a, b Vec2) Vec2 {
	c := Vec2{a.X - b.X, a.Y - b.Y}
	return c
}

func neg2(v Vec2) Vec2 {
	c := Vec2{-v.X, -v.Y}
	return c
}

func mulSV2(s float32, v Vec2) Vec2 {
	c := Vec2{float32(s * v.X), float32(s * v.Y)}
	return c
}

// mulAdd2 returns a + s * b
func mulAdd2(a Vec2, s float32, b Vec2) Vec2 {
	c := Vec2{a.X + float32(s*b.X), a.Y + float32(s*b.Y)}
	return c
}

// mulSub2 returns a - s * b
func mulSub2(a Vec2, s float32, b Vec2) Vec2 {
	c := Vec2{a.X - float32(s*b.X), a.Y - float32(s*b.Y)}
	return c
}

func cross2(a, b Vec2) float32 {
	return float32(a.X*b.Y) - float32(a.Y*b.X)
}

func distanceSquared2(a, b Vec2) float32 {
	dx := b.X - a.X
	dy := b.Y - a.Y
	return float32(dx*dx) + float32(dy*dy)
}

func mulMV2(m matrix2, a Vec2) Vec2 {
	b := Vec2{float32(m.cx.X*a.X) + float32(m.cy.X*a.Y), float32(m.cx.Y*a.X) + float32(m.cy.Y*a.Y)}
	return b
}

func mulMM2(m1, m2 matrix2) matrix2 {
	var out matrix2
	out.cx = mulMV2(m1, m2.cx)
	out.cy = mulMV2(m1, m2.cy)
	return out
}

func det2(m matrix2) float32 {
	return float32(m.cx.X*m.cy.Y) - float32(m.cx.Y*m.cy.X)
}

func invert2(m matrix2) matrix2 {
	det := det2(m)
	if AbsFloat(det) > 1000.0*fltMin {
		invDet := 1.0 / det
		return matrix2{
			Vec2{float32(invDet * m.cy.Y), float32(-invDet * m.cx.Y)},
			Vec2{float32(-invDet * m.cy.X), float32(invDet * m.cx.X)},
		}
	}

	return matrix2{Vec2{0.0, 0.0}, Vec2{0.0, 0.0}}
}

// solve2 assumes positive semi-definite
func solve2(m matrix2, b Vec2) Vec2 {
	det := det2(m)
	if det > 1000.0*fltMin {
		invDet := 1.0 / det
		return Vec2{
			float32(invDet*m.cy.Y*b.X) - float32(invDet*m.cy.X*b.Y),
			float32(-invDet*m.cx.Y*b.X) + float32(invDet*m.cx.X*b.Y),
		}
	}

	return Vec2{0.0, 0.0}
}

// blend3 is a convenience function: s * a + t * b + u * c
func blend3(s float32, a Vec3, t float32, b Vec3, u float32, c Vec3) Vec3 {
	d := Vec3{
		float32(s*a.X) + float32(t*b.X) + float32(u*c.X),
		float32(s*a.Y) + float32(t*b.Y) + float32(u*c.Y),
		float32(s*a.Z) + float32(t*b.Z) + float32(u*c.Z),
	}
	return d
}

func modifiedCross(a, b Vec3) Vec3 {
	var c Vec3
	c.X = float32(a.Y*b.Z) + float32(a.Z*b.Y)
	c.Y = float32(a.Z*b.X) + float32(a.X*b.Z)
	c.Z = float32(a.X*b.Y) + float32(a.Y*b.X)
	return c
}

func makeDiagonalMatrix(a, b, c float32) Matrix3 {
	return Matrix3{Vec3{a, 0.0, 0.0}, Vec3{0.0, b, 0.0}, Vec3{0.0, 0.0, c}}
}

func normalizePlane(plane Plane) Plane {
	invLength := 1.0 / Length(plane.Normal)
	return Plane{MulSV(invLength, plane.Normal), float32(invLength * plane.Offset)}
}

func makePlaneFromNormalAndPoint(normal, point Vec3) Plane {
	return Plane{normal, Dot(normal, point)}
}

func makePlaneFromPoints(point1, point2, point3 Vec3) Plane {
	var plane Plane
	plane.Normal = Cross(Sub(point2, point1), Sub(point3, point1))
	plane.Normal = Normalize(plane.Normal)
	plane.Offset = Dot(plane.Normal, point1)
	return plane
}

func makeNormalFromPoints(point1, point2, point3 Vec3) Vec3 {
	normal := Cross(Sub(point2, point1), Sub(point3, point1))
	return Normalize(normal)
}

// transformPlane computes
// normal2 = q * normal1
// offset2 = dot(normal2, p) + offset1
func transformPlane(transform Transform, plane Plane) Plane {
	normal := RotateVector(transform.Q, plane.Normal)
	return Plane{normal, plane.Offset + Dot(normal, transform.P)}
}

// planeSeparation is the signed separation of a point from a plane
func planeSeparation(plane Plane, point Vec3) float32 {
	return Dot(plane.Normal, point) - plane.Offset
}

// signedVolume is negative if p is below the triangle v1-v2-v3
func signedVolume(v1, v2, v3, p Vec3) float32 {
	e1 := Sub(v2, v1)
	e2 := Sub(v3, v1)
	n := Cross(e1, e2)
	return Dot(n, Sub(p, v1))
}

// isWithinSegments is b3IsWithinSegments.
// todo eliminate this
func isWithinSegments(result *SegmentDistanceResult) bool {
	return (0.0 <= result.Fraction1 && result.Fraction1 <= 1.0) &&
		(0.0 <= result.Fraction2 && result.Fraction2 <= 1.0)
}

func rotateInertia(q Quat, centralInertia Matrix3) Matrix3 {
	rotationMatrix := MakeMatrixFromQuat(q)
	inertia := MulMM(rotationMatrix, MulMM(centralInertia, Transpose(rotationMatrix)))
	return inertia
}

func transformInertia(transform Transform, centralInertia Matrix3, mass float32) Matrix3 {
	inertia := rotateInertia(transform.Q, centralInertia)
	inertia = AddMM(inertia, Steiner(mass, transform.P))
	return inertia
}

// addPoint is b3AABB_AddPoint: add a point to an AABB.
func (a AABB) addPoint(point Vec3) AABB {
	return AABB{Min(a.LowerBound, point), Max(a.UpperBound, point)}
}
