// Port of include/box3d/math_functions.h and src/math_functions.c from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "math"

// ---- Go helpers for float determinism (no upstream counterpart) ----

// PORT: Go may fuse x*y + z (and z - x*y, x*y - z) into a single FMA instruction on arm64 and
// on amd64 with GOAMD64=v3, even across statements. Upstream builds with -ffp-contract=off.
// An explicit float32() conversion rounds the product and prevents fusion, so every product
// that feeds an addition or subtraction is written float32(x*y) or goes through mulAdd/mulSub.
// fma_guard_test.go fails the build if a fused instruction appears in the package.

// mulAdd returns a + b*c with the product rounded before the add.
func mulAdd(a, b, c float32) float32 {
	return a + float32(b*c)
}

// mulSub returns a - b*c with the product rounded before the subtract.
func mulSub(a, b, c float32) float32 {
	return a - float32(b*c)
}

// sqrtf is the C sqrtf. The float64 square root of a float32 rounded back to float32 is
// correctly rounded, so it matches sqrtf bit for bit.
func sqrtf(x float32) float32 {
	return float32(math.Sqrt(float64(x)))
}

// remainderf is the C remainderf. The IEEE remainder is exact, so computing it in float64 and
// converting back is bit identical.
func remainderf(x, y float32) float32 {
	return float32(math.Remainder(float64(x), float64(y)))
}

// float.h
const (
	fltMin     float32 = 0x1p-126 // FLT_MIN
	fltEpsilon float32 = 0x1p-23  // FLT_EPSILON
	fltMax     float32 = math.MaxFloat32
)

// ---- math_functions.h ----

// Math: vector math types and functions

// Pi is B3_PI. https://en.wikipedia.org/wiki/Pi
const Pi float32 = 3.14159265359

// DegToRad is a convenience constant to convert from degrees to radians.
const DegToRad float32 = 0.01745329251

// RadToDeg is a convenience constant to convert from radians to degrees.
const RadToDeg float32 = 57.2957795131

// MinScale is the minimum scale used for scaling collision meshes, etc.
const MinScale float32 = 0.01

// Vec2 is a 2D vector.
type Vec2 struct {
	X float32
	Y float32
}

// Vec3 is a 3D vector.
type Vec3 struct {
	X float32
	Y float32
	Z float32
}

// CosSin is a cosine and sine pair.
// This uses a custom implementation designed for cross-platform determinism.
type CosSin struct {
	// cosine and sine
	Cosine float32
	Sine   float32
}

// Quat is a quaternion.
type Quat struct {
	V Vec3
	S float32
}

// Transform is a rigid transform.
type Transform struct {
	P Vec3
	Q Quat
}

// PORT: BOX3D_DOUBLE_PRECISION is not ported. In single precision mode upstream makes b3Pos and
// b3WorldTransform typedefs of b3Vec3 and b3Transform. Here they are distinct types with the
// same layout so a double precision mode can be added later without touching call sites.
// Convert with ToPos / ToVec3 / MakeWorldTransform as upstream does.

// Pos is a world position. In single precision mode this has the same layout as Vec3.
type Pos struct {
	X, Y, Z float32
}

// WorldTransform is a world transform. In single precision mode this has the same layout as Transform.
type WorldTransform struct {
	P Pos
	Q Quat
}

// Matrix3 is a 3x3 matrix.
type Matrix3 struct {
	Cx, Cy, Cz Vec3
}

// AABB is an axis aligned bounding box.
type AABB struct {
	LowerBound Vec3
	UpperBound Vec3
}

// Plane is a plane.
// separation = dot(normal, point) - offset
type Plane struct {
	Normal Vec3
	Offset float32
}

var (
	Vec3Zero          = Vec3{0.0, 0.0, 0.0}
	Vec3One           = Vec3{1.0, 1.0, 1.0}
	Vec3AxisX         = Vec3{1.0, 0.0, 0.0}
	Vec3AxisY         = Vec3{0.0, 1.0, 0.0}
	Vec3AxisZ         = Vec3{0.0, 0.0, 1.0}
	QuatIdentity      = Quat{Vec3{0.0, 0.0, 0.0}, 1.0}
	TransformIdentity = Transform{Vec3{0.0, 0.0, 0.0}, Quat{Vec3{0.0, 0.0, 0.0}, 1.0}}
	Mat3Zero          = Matrix3{
		Vec3{0.0, 0.0, 0.0},
		Vec3{0.0, 0.0, 0.0},
		Vec3{0.0, 0.0, 0.0},
	}
	Mat3Identity = Matrix3{
		Vec3{1.0, 0.0, 0.0},
		Vec3{0.0, 1.0, 0.0},
		Vec3{0.0, 0.0, 1.0},
	}

	// Valid in both modes: 0.0f promotes to double, the identity rotation stays float
	PosZero                = Pos{0.0, 0.0, 0.0}
	WorldTransformIdentity = WorldTransform{Pos{0.0, 0.0, 0.0}, Quat{Vec3{0.0, 0.0, 0.0}, 1.0}}
)

// MinInt returns the minimum of two integers.
func MinInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}

// MaxInt returns the maximum of two integers.
func MaxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

// ClampInt returns an integer clamped between a lower and upper bound.
func ClampInt(a, lower, upper int) int {
	if a < lower {
		return lower
	}
	if upper < a {
		return upper
	}
	return a
}

// AbsFloat returns the absolute value of a float.
func AbsFloat(a float32) float32 {
	if a < 0 {
		return -a
	}
	return a
}

// MinFloat returns the minimum of two floats.
// PORT: written as upstream's ternary, not the min builtin, so NaN handling matches.
func MinFloat(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

// MaxFloat returns the maximum of two floats.
func MaxFloat(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

// ClampFloat returns a float clamped between a lower and upper bound.
func ClampFloat(a, lower, upper float32) float32 {
	if a < lower {
		return lower
	}
	if upper < a {
		return upper
	}
	return a
}

// LerpFloat interpolates a scalar.
func LerpFloat(a, b, alpha float32) float32 {
	return float32((1.0-alpha)*a) + float32(alpha*b)
}

// Atan2 computes an approximate arctangent in the range [-pi, pi]
// This is hand coded for cross-platform determinism. The atan2f
// function in the standard library is not cross-platform deterministic.
// Accurate to around 0.0023 degrees.
// Implemented below with math_functions.c.

// ComputeCosSin computes the cosine and sine of an angle in radians. Implemented
// for cross-platform determinism. Implemented below with math_functions.c.

// Sin is deprecated.
func Sin(radians float32) float32 {
	cs := ComputeCosSin(radians)
	return cs.Sine
}

// Cos is deprecated.
func Cos(radians float32) float32 {
	cs := ComputeCosSin(radians)
	return cs.Cosine
}

// UnwindAngle converts any angle into the range [-pi, pi].
func UnwindAngle(radians float32) float32 {
	// Assuming this is deterministic
	return remainderf(radians, 2.0*Pi)
}

// Add is vector addition.
func Add(a, b Vec3) Vec3 {
	return Vec3{a.X + b.X, a.Y + b.Y, a.Z + b.Z}
}

// Sub is vector subtraction.
func Sub(a, b Vec3) Vec3 {
	return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z}
}

// Mul is vector component-wise multiplication.
func Mul(a, b Vec3) Vec3 {
	return Vec3{float32(a.X * b.X), float32(a.Y * b.Y), float32(a.Z * b.Z)}
}

// Neg is vector negation.
func Neg(a Vec3) Vec3 {
	return Vec3{-a.X, -a.Y, -a.Z}
}

// Dot is the vector dot product.
func Dot(a, b Vec3) float32 {
	return float32(a.X*b.X) + float32(a.Y*b.Y) + float32(a.Z*b.Z)
}

// Length is the vector length.
func Length(v Vec3) float32 {
	return sqrtf(Dot(v, v))
}

// LengthSquared is the vector length squared.
func LengthSquared(a Vec3) float32 {
	return float32(a.X*a.X) + float32(a.Y*a.Y) + float32(a.Z*a.Z)
}

// Distance between two points.
func Distance(a, b Vec3) float32 {
	dv := Vec3{b.X - a.X, b.Y - a.Y, b.Z - a.Z}
	return Length(dv)
}

// DistanceSquared is the squared distance between two points.
func DistanceSquared(a, b Vec3) float32 {
	dv := Vec3{b.X - a.X, b.Y - a.Y, b.Z - a.Z}
	return float32(dv.X*dv.X) + float32(dv.Y*dv.Y) + float32(dv.Z*dv.Z)
}

// Normalize a vector. Returns a zero vector if the input vector is very small.
func Normalize(a Vec3) Vec3 {
	lengthSquared := float32(a.X*a.X) + float32(a.Y*a.Y) + float32(a.Z*a.Z)
	if lengthSquared > 1000.0*fltMin {
		s := 1.0 / sqrtf(lengthSquared)
		u := Vec3{float32(s * a.X), float32(s * a.Y), float32(s * a.Z)}
		return u
	}
	return Vec3{0.0, 0.0, 0.0}
}

// GetLengthAndNormalize normalizes a vector and returns the length. Returns a zero vector
// if the input is very small.
// PORT: the length out-parameter is the second return value.
func GetLengthAndNormalize(a Vec3) (Vec3, float32) {
	lengthSquared := float32(a.X*a.X) + float32(a.Y*a.Y) + float32(a.Z*a.Z)
	if lengthSquared > 1000.0*fltMin {
		length := sqrtf(lengthSquared)
		s := 1.0 / length
		u := Vec3{float32(s * a.X), float32(s * a.Y), float32(s * a.Z)}
		return u, length
	}

	return Vec3{0.0, 0.0, 0.0}, 0.0
}

// Perp gets a unit vector that is perpendicular to the supplied vector.
func Perp(a Vec3) Vec3 {
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	var p Vec3
	if a.X < -0.5 || 0.5 < a.X {
		p = Vec3{a.Y, -a.X, 0.0}
	} else {
		p = Vec3{0.0, a.Z, -a.Y}
	}

	return Normalize(p)
}

// IsNormalized: is a vector normalized? In other words, does it have unit length?
func IsNormalized(a Vec3) bool {
	aa := Dot(a, a)
	return AbsFloat(1.0-aa) < 100.0*fltEpsilon
}

// MulAdd returns a + s * b.
func MulAdd(a Vec3, s float32, b Vec3) Vec3 {
	return Vec3{a.X + float32(s*b.X), a.Y + float32(s*b.Y), a.Z + float32(s*b.Z)}
}

// MulSub returns a - s * b.
func MulSub(a Vec3, s float32, b Vec3) Vec3 {
	return Vec3{a.X - float32(s*b.X), a.Y - float32(s*b.Y), a.Z - float32(s*b.Z)}
}

// MulSV returns s * a.
func MulSV(s float32, a Vec3) Vec3 {
	return Vec3{float32(s * a.X), float32(s * a.Y), float32(s * a.Z)}
}

// Cross is the vector cross product. https://en.wikipedia.org/wiki/Cross_product
func Cross(a, b Vec3) Vec3 {
	var c Vec3
	c.X = float32(a.Y*b.Z) - float32(a.Z*b.Y)
	c.Y = float32(a.Z*b.X) - float32(a.X*b.Z)
	c.Z = float32(a.X*b.Y) - float32(a.Y*b.X)
	return c
}

// Lerp linearly interpolates between two vectors.
func Lerp(a, b Vec3, alpha float32) Vec3 {
	assert(0.0 <= alpha && alpha <= 1.0, "0.0f <= alpha && alpha <= 1.0f")

	c := Vec3{
		float32((1.0-alpha)*a.X) + float32(alpha*b.X),
		float32((1.0-alpha)*a.Y) + float32(alpha*b.Y),
		float32((1.0-alpha)*a.Z) + float32(alpha*b.Z),
	}
	return c
}

// Blend2 blends two vectors: s * a + t * b
func Blend2(s float32, a Vec3, t float32, b Vec3) Vec3 {
	d := Vec3{
		float32(s*a.X) + float32(t*b.X),
		float32(s*a.Y) + float32(t*b.Y),
		float32(s*a.Z) + float32(t*b.Z),
	}
	return d
}

// Abs is the component-wise absolute value.
func Abs(a Vec3) Vec3 {
	return Vec3{
		AbsFloat(a.X),
		AbsFloat(a.Y),
		AbsFloat(a.Z),
	}
}

// signFloat is the ternary in b3Sign.
func signFloat(a float32) float32 {
	if a >= 0.0 {
		return 1.0
	}
	return -1.0
}

// Sign is the component-wise -1 or 1 (1 if zero).
func Sign(a Vec3) Vec3 {
	return Vec3{
		signFloat(a.X),
		signFloat(a.Y),
		signFloat(a.Z),
	}
}

// Min is the component-wise minimum value.
func Min(a, b Vec3) Vec3 {
	return Vec3{
		MinFloat(a.X, b.X),
		MinFloat(a.Y, b.Y),
		MinFloat(a.Z, b.Z),
	}
}

// Max is the component-wise maximum value.
func Max(a, b Vec3) Vec3 {
	return Vec3{
		MaxFloat(a.X, b.X),
		MaxFloat(a.Y, b.Y),
		MaxFloat(a.Z, b.Z),
	}
}

// Clamp is the component-wise clamped value.
func Clamp(a, lower, upper Vec3) Vec3 {
	var b Vec3
	b.X = ClampFloat(a.X, lower.X, upper.X)
	b.Y = ClampFloat(a.Y, lower.Y, upper.Y)
	b.Z = ClampFloat(a.Z, lower.Z, upper.Z)
	return b
}

// SafeScale creates a safe scaling value for scaling collision. This allows
// negative scale, but keeps scale sufficiently far from zero.
func SafeScale(a Vec3) Vec3 {
	absScale := Abs(a)
	minScale := Vec3{MinScale, MinScale, MinScale}
	safeScale := Mul(Sign(a), Max(absScale, minScale))
	return safeScale
}

// IsNormalizedQuat: does the supplied quaternion have unit length?
func IsNormalizedQuat(q Quat) bool {
	qq := float32(q.V.X*q.V.X) + float32(q.V.Y*q.V.Y) + float32(q.V.Z*q.V.Z) + float32(q.S*q.S)
	return 1.0-20.0*fltEpsilon < qq && qq < 1.0+20.0*fltEpsilon
}

// RotateVector rotates a vector.
func RotateVector(q Quat, v Vec3) Vec3 {
	// v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
	// B3_ASSERT( b3IsNormalizedQuat( q ) );
	t1 := Cross(q.V, v)
	t2 := MulAdd(t1, q.S, v)
	t3 := Cross(q.V, t2)
	return MulAdd(v, 2.0, t3)
}

// InvRotateVector inverse rotates a vector.
func InvRotateVector(q Quat, v Vec3) Vec3 {
	// v + 2 * cross(q.v, cross(q.v, v) - q.s * v)
	// B3_ASSERT( b3IsNormalizedQuat( q ) );
	t1 := Cross(q.V, v)
	t2 := MulSub(t1, q.S, v)
	t3 := Cross(q.V, t2)
	return MulAdd(v, 2.0, t3)
}

// DotQuat computes dot product of two quaternions. Useful for polarity tests.
func DotQuat(a, b Quat) float32 {
	return float32(a.V.X*b.V.X) + float32(a.V.Y*b.V.Y) + float32(a.V.Z*b.V.Z) + float32(a.S*b.S)
}

// MulQuat multiplies two quaternions.
func MulQuat(q1, q2 Quat) Quat {
	t1 := Cross(q1.V, q2.V)
	t2 := MulAdd(t1, q1.S, q2.V)
	t3 := MulAdd(t2, q2.S, q1.V)
	q := Quat{t3, float32(q1.S*q2.S) - Dot(q1.V, q2.V)}
	return q
}

// InvMulQuat computes a relative quaternion.
// inv(q1) * q2
func InvMulQuat(q1, q2 Quat) Quat {
	t1 := Cross(q2.V, q1.V)
	t2 := MulAdd(t1, q1.S, q2.V)
	t3 := MulSub(t2, q2.S, q1.V)
	q := Quat{t3, float32(q1.S*q2.S) + Dot(q1.V, q2.V)}
	return q
}

// Conjugate is the quaternion conjugate (cheap inverse).
func Conjugate(q Quat) Quat {
	return Quat{Vec3{-q.V.X, -q.V.Y, -q.V.Z}, q.S}
}

// NegateQuat is the component-wise quaternion negation.
func NegateQuat(q Quat) Quat {
	return Quat{Vec3{-q.V.X, -q.V.Y, -q.V.Z}, -q.S}
}

// NormalizeQuat normalizes a quaternion.
func NormalizeQuat(q Quat) Quat {
	lengthSq := DotQuat(q, q)
	if lengthSq > 1000.0*fltMin {
		s := 1.0 / sqrtf(lengthSq)
		qn := Quat{Vec3{float32(s * q.V.X), float32(s * q.V.Y), float32(s * q.V.Z)}, float32(s * q.S)}
		return qn
	}

	return QuatIdentity
}

// MakeQuatFromAxisAngle makes a quaternion that is equivalent to rotating around an axis by a specified angle.
func MakeQuatFromAxisAngle(axis Vec3, radians float32) Quat {
	assert(IsNormalized(axis), "b3IsNormalized( axis )")
	cs := ComputeCosSin(0.5 * radians)
	q := Quat{Vec3{float32(cs.Sine * axis.X), float32(cs.Sine * axis.Y), float32(cs.Sine * axis.Z)}, cs.Cosine}
	return q
}

// GetAxisAngle gets the axis and angle from a quaternion. Assumes the quaternion is normalized.
// PORT: the radians out-parameter is the second return value.
func GetAxisAngle(q Quat) (Vec3, float32) {
	length := sqrtf(float32(q.V.X*q.V.X) + float32(q.V.Y*q.V.Y) + float32(q.V.Z*q.V.Z))
	radians := float32(2.0 * Atan2(length, q.S))
	if length > 0.0 {
		invLength := 1.0 / length
		axis := Vec3{float32(invLength * q.V.X), float32(invLength * q.V.Y), float32(invLength * q.V.Z)}
		return axis, radians
	}

	return Vec3Zero, radians
}

// GetQuatAngle gets the angle for a quaternion in radians
func GetQuatAngle(q Quat) float32 {
	length := sqrtf(float32(q.V.X*q.V.X) + float32(q.V.Y*q.V.Y) + float32(q.V.Z*q.V.Z))
	return float32(2.0 * Atan2(length, q.S))
}

// MakeQuatFromMatrix extracts a quaternion from a rotation matrix. Implemented below with math_functions.c.

// ComputeQuatBetweenUnitVectors finds a quaternion that rotates one vector to another.
// Implemented below with math_functions.c.

// GetTwistAngle is the twist angle around the z-axis, used for twist limit and revolute angle limit
func GetTwistAngle(q Quat) float32 {
	// Account for polarity to keep the twist angle in range.
	// This is simpler than asking the user to check polarity or unwinding.
	var twist float32
	if q.S < 0.0 {
		twist = Atan2(-q.V.Z, -q.S)
	} else {
		twist = Atan2(q.V.Z, q.S)
	}
	twist *= 2.0
	assert(-Pi <= twist && twist <= Pi, "-B3_PI <= twist && twist <= B3_PI")
	return twist
}

// GetSwingAngle is the swing angle used for cone limit
func GetSwingAngle(q Quat) float32 {
	// Polarity should not matter because all terms are squared.
	x := sqrtf(float32(q.V.Z*q.V.Z) + float32(q.S*q.S))
	y := sqrtf(float32(q.V.X*q.V.X) + float32(q.V.Y*q.V.Y))
	swing := float32(2.0 * Atan2(y, x))
	assert(0.0 <= swing && swing <= Pi, "0.0f <= swing && swing <= B3_PI")
	return swing
}

// NLerp linearly interpolates and normalizes between two quaternions
func NLerp(q1, q2 Quat, alpha float32) Quat {
	validate(0.0 <= alpha && alpha <= 1.0, "0.0f <= alpha && alpha <= 1.0f")
	if DotQuat(q1, q2) < 0.0 {
		q1 = Quat{Vec3{-q1.V.X, -q1.V.Y, -q1.V.Z}, -q1.S}
	}

	var q Quat
	q.V = Lerp(q1.V, q2.V, alpha)
	q.S = float32((1.0-alpha)*q1.S) + float32(alpha*q2.S)

	return NormalizeQuat(q)
}

// MulTransforms multiplies two transforms. If the result is applied to a point p local to frame B,
// the transform would first convert p to a point local to frame A, then into a point
// in the world frame. This is useful if frame B is a child of frame A.
func MulTransforms(a, b Transform) Transform {
	var out Transform
	out.P = Add(RotateVector(a.Q, b.P), a.P)
	out.Q = MulQuat(a.Q, b.Q)
	return out
}

// InvMulTransforms creates a transform that converts a local point in frame B to a local point in frame A.
// This is useful for transforming points between the local spaces of two frames that are
// in world space.
func InvMulTransforms(a, b Transform) Transform {
	var out Transform
	out.P = InvRotateVector(a.Q, Sub(b.P, a.P))
	out.Q = InvMulQuat(a.Q, b.Q)
	return out
}

// InvertTransform gets the inverse of a transform.
func InvertTransform(t Transform) Transform {
	var out Transform
	out.P = InvRotateVector(t.Q, Neg(t.P))
	out.Q = Conjugate(t.Q)
	return out
}

// TransformPoint transforms a point.
func TransformPoint(t Transform, v Vec3) Vec3 {
	rv := RotateVector(t.Q, v)
	return Add(rv, t.P)
}

// InvTransformPoint inverse transforms a point.
func InvTransformPoint(t Transform, v Vec3) Vec3 {
	return InvRotateVector(t.Q, Sub(v, t.P))
}

// World position boundary. These cross between the double precision world space at the public
// boundary and the float interior. One set of bodies serves both modes: the typedefs collapse
// the types in float mode and the explicit float casts become no-ops.

// ToPos converts a vector to a world position.
func ToPos(v Vec3) Pos {
	return Pos{v.X, v.Y, v.Z}
}

// ToVec3 is a lossy conversion of a world position to a float vector.
func ToVec3(p Pos) Vec3 {
	return Vec3{p.X, p.Y, p.Z}
}

// RoundDownFloat narrows a world coordinate to float, rounding toward negative infinity. Use with
// RoundUpFloat to build a conservative float box that always contains the double bounds,
// where plain rounding far from the origin could clip. With large world mode off this is a plain conversion.
// PORT: the parameter is float32 because large world mode is not ported; upstream takes double.
func RoundDownFloat(x float32) float32 {
	return x
}

// RoundUpFloat narrows a world coordinate to float, rounding toward positive infinity.
// PORT: the parameter is float32 because large world mode is not ported; upstream takes double.
func RoundUpFloat(x float32) float32 {
	return x
}

// SubPos returns a - b, demoted to float. The primary precision boundary operation.
func SubPos(a, b Pos) Vec3 {
	return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z}
}

// OffsetPos returns p + d.
func OffsetPos(p Pos, d Vec3) Pos {
	return Pos{p.X + d.X, p.Y + d.Y, p.Z + d.Z}
}

// LerpPosition is world position interpolation for sweeps and sampling.
func LerpPosition(a, b Pos, t float32) Pos {
	return Pos{
		float32((1.0-t)*a.X) + float32(t*b.X),
		float32((1.0-t)*a.Y) + float32(t*b.Y),
		float32((1.0-t)*a.Z) + float32(t*b.Z),
	}
}

// TransformWorldPoint transforms a local point to a world position. Rotation in float, translation in double.
func TransformWorldPoint(t WorldTransform, p Vec3) Pos {
	r := RotateVector(t.Q, p)
	return Pos{t.P.X + r.X, t.P.Y + r.Y, t.P.Z + r.Z}
}

// InvTransformWorldPoint transforms a world position to a local point. One double subtraction, then float.
func InvTransformWorldPoint(t WorldTransform, p Pos) Vec3 {
	d := Vec3{p.X - t.P.X, p.Y - t.P.Y, p.Z - t.P.Z}
	return InvRotateVector(t.Q, d)
}

// MulWorldTransforms composes a world transform with a local transform.
func MulWorldTransforms(A WorldTransform, B Transform) WorldTransform {
	var C WorldTransform
	C.Q = MulQuat(A.Q, B.Q)
	r := RotateVector(A.Q, B.P)
	C.P = Pos{A.P.X + r.X, A.P.Y + r.Y, A.P.Z + r.Z}
	return C
}

// InvMulWorldTransforms is the relative transform of frame B in frame A. The narrow phase boundary.
func InvMulWorldTransforms(A, B WorldTransform) Transform {
	var C Transform
	C.Q = InvMulQuat(A.Q, B.Q)
	d := Vec3{B.P.X - A.P.X, B.P.Y - A.P.Y, B.P.Z - A.P.Z}
	C.P = InvRotateVector(A.Q, d)
	return C
}

// ToRelativeTransform shifts a world transform into the frame of a base position.
func ToRelativeTransform(t WorldTransform, base Pos) Transform {
	var r Transform
	r.Q = t.Q
	r.P = Vec3{t.P.X - base.X, t.P.Y - base.Y, t.P.Z - base.Z}
	return r
}

// MakeWorldTransform promotes a float transform to a world transform. Lossless.
func MakeWorldTransform(t Transform) WorldTransform {
	var w WorldTransform
	w.P = ToPos(t.P)
	w.Q = t.Q
	return w
}

// OffsetAABB translates a local AABB by a world position, rounding outward so the single precision box
// encloses the double precision box. Far from the origin a plain conversion could clip a
// shape out of its own box.
// In single precision mode the the rounding is a no-op.
func OffsetAABB(localBox AABB, origin Pos) AABB {
	var out AABB
	out.LowerBound.X = RoundDownFloat(origin.X + localBox.LowerBound.X)
	out.LowerBound.Y = RoundDownFloat(origin.Y + localBox.LowerBound.Y)
	out.LowerBound.Z = RoundDownFloat(origin.Z + localBox.LowerBound.Z)
	out.UpperBound.X = RoundUpFloat(origin.X + localBox.UpperBound.X)
	out.UpperBound.Y = RoundUpFloat(origin.Y + localBox.UpperBound.Y)
	out.UpperBound.Z = RoundUpFloat(origin.Z + localBox.UpperBound.Z)
	return out
}

// Det computes the determinant of a 3-by-3 matrix.
func Det(m Matrix3) float32 {
	return Dot(m.Cx, Cross(m.Cy, m.Cz))
}

// MulMV multiplies a matrix times a column vector.
func MulMV(m Matrix3, a Vec3) Vec3 {
	b := Vec3{
		float32(m.Cx.X*a.X) + float32(m.Cy.X*a.Y) + float32(m.Cz.X*a.Z),
		float32(m.Cx.Y*a.X) + float32(m.Cy.Y*a.Y) + float32(m.Cz.Y*a.Z),
		float32(m.Cx.Z*a.X) + float32(m.Cy.Z*a.Y) + float32(m.Cz.Z*a.Z),
	}
	return b
}

// NegateMat3 negates a matrix.
func NegateMat3(a Matrix3) Matrix3 {
	return Matrix3{
		Vec3{-a.Cx.X, -a.Cx.Y, -a.Cx.Z},
		Vec3{-a.Cy.X, -a.Cy.Y, -a.Cy.Z},
		Vec3{-a.Cz.X, -a.Cz.Y, -a.Cz.Z},
	}
}

// AddMM is matrix addition.
// Returns a + b
func AddMM(a, b Matrix3) Matrix3 {
	return Matrix3{
		Vec3{a.Cx.X + b.Cx.X, a.Cx.Y + b.Cx.Y, a.Cx.Z + b.Cx.Z},
		Vec3{a.Cy.X + b.Cy.X, a.Cy.Y + b.Cy.Y, a.Cy.Z + b.Cy.Z},
		Vec3{a.Cz.X + b.Cz.X, a.Cz.Y + b.Cz.Y, a.Cz.Z + b.Cz.Z},
	}
}

// SubMM is matrix subtraction.
// Returns a - b
func SubMM(a, b Matrix3) Matrix3 {
	return Matrix3{
		Vec3{a.Cx.X - b.Cx.X, a.Cx.Y - b.Cx.Y, a.Cx.Z - b.Cx.Z},
		Vec3{a.Cy.X - b.Cy.X, a.Cy.Y - b.Cy.Y, a.Cy.Z - b.Cy.Z},
		Vec3{a.Cz.X - b.Cz.X, a.Cz.Y - b.Cz.Y, a.Cz.Z - b.Cz.Z},
	}
}

// MulSM multiplies a matrix by a scalar, component-wise.
func MulSM(s float32, a Matrix3) Matrix3 {
	return Matrix3{
		Vec3{float32(s * a.Cx.X), float32(s * a.Cx.Y), float32(s * a.Cx.Z)},
		Vec3{float32(s * a.Cy.X), float32(s * a.Cy.Y), float32(s * a.Cy.Z)},
		Vec3{float32(s * a.Cz.X), float32(s * a.Cz.Y), float32(s * a.Cz.Z)},
	}
}

// MulMM is matrix multiplication.
// Returns a * b
func MulMM(a, b Matrix3) Matrix3 {
	var out Matrix3
	out.Cx = MulMV(a, b.Cx)
	out.Cy = MulMV(a, b.Cy)
	out.Cz = MulMV(a, b.Cz)
	return out
}

// Transpose is the matrix transpose.
func Transpose(m Matrix3) Matrix3 {
	var out Matrix3
	out.Cx = Vec3{m.Cx.X, m.Cy.X, m.Cz.X}
	out.Cy = Vec3{m.Cx.Y, m.Cy.Y, m.Cz.Y}
	out.Cz = Vec3{m.Cx.Z, m.Cy.Z, m.Cz.Z}

	return out
}

// InvertMatrix is the general matrix inverse.
func InvertMatrix(m Matrix3) Matrix3 {
	det := Det(m)
	if AbsFloat(det) > 1000.0*fltMin {
		invDet := 1.0 / det
		var out Matrix3
		out.Cx = MulSV(invDet, Cross(m.Cy, m.Cz))
		out.Cy = MulSV(invDet, Cross(m.Cz, m.Cx))
		out.Cz = MulSV(invDet, Cross(m.Cx, m.Cy))

		return Transpose(out)
	}

	return Mat3Zero
}

// Solve3 solves a matrix equation.
// Returns inv(m) * a
func Solve3(m Matrix3, a Vec3) Vec3 {
	det := Det(m)
	if AbsFloat(det) > 1000.0*fltMin {
		invDet := 1.0 / det
		var s Matrix3
		s.Cx = Cross(m.Cy, m.Cz)
		s.Cy = Cross(m.Cz, m.Cx)
		s.Cz = Cross(m.Cx, m.Cy)

		b := Vec3{
			float32(invDet * Dot(s.Cx, a)),
			float32(invDet * Dot(s.Cy, a)),
			float32(invDet * Dot(s.Cz, a)),
		}

		return b
	}

	return Vec3Zero
}

// InvertT inverts a matrix.
func InvertT(m Matrix3) Matrix3 {
	det := Det(m)
	if AbsFloat(det) > 1000.0*fltMin {
		invDet := 1.0 / det
		var out Matrix3
		out.Cx = MulSV(invDet, Cross(m.Cy, m.Cz))
		out.Cy = MulSV(invDet, Cross(m.Cz, m.Cx))
		out.Cz = MulSV(invDet, Cross(m.Cx, m.Cy))
		return out
	}

	return Mat3Zero
}

// AbsMatrix3 gets the component-wise absolute value of a matrix.
func AbsMatrix3(m Matrix3) Matrix3 {
	var out Matrix3
	out.Cx = Abs(m.Cx)
	out.Cy = Abs(m.Cy)
	out.Cz = Abs(m.Cz)

	return out
}

// MakeMatrixFromQuat makes a matrix from a quaternion. This is useful if you need to
// rotate many vectors.
// The force inline improves the performance of b3ShapeDistance.
func MakeMatrixFromQuat(q Quat) Matrix3 {
	// PORT: every product is consumed by an add, so each is rounded explicitly.
	xx := float32(q.V.X * q.V.X)
	yy := float32(q.V.Y * q.V.Y)
	zz := float32(q.V.Z * q.V.Z)
	xy := float32(q.V.X * q.V.Y)
	xz := float32(q.V.X * q.V.Z)
	xw := float32(q.V.X * q.S)
	yz := float32(q.V.Y * q.V.Z)
	yw := float32(q.V.Y * q.S)
	zw := float32(q.V.Z * q.S)

	return Matrix3{
		Vec3{1.0 - float32(2.0*(yy+zz)), 2.0 * (xy + zw), 2.0 * (xz - yw)},
		Vec3{2.0 * (xy - zw), 1.0 - float32(2.0*(xx+zz)), 2.0 * (yz + xw)},
		Vec3{2.0 * (xz + yw), 2.0 * (yz - xw), 1.0 - float32(2.0*(xx+yy))},
	}
}

// Steiner gets the inertia tensor of an offset point.
// https://en.wikipedia.org/wiki/Parallel_axis_theorem
// Implemented below with math_functions.c.

// MakeAABB gets the AABB of a point cloud.
// PORT: the pointer + count pair is a slice.
func MakeAABB(points []Vec3, radius float32) AABB {
	count := len(points)
	assert(count > 0, "count > 0")
	a := AABB{points[0], points[0]}
	for i := 1; i < count; i++ {
		a.LowerBound = Min(a.LowerBound, points[i])
		a.UpperBound = Max(a.UpperBound, points[i])
	}

	r := Vec3{radius, radius, radius}
	a.LowerBound = Sub(a.LowerBound, r)
	a.UpperBound = Add(a.UpperBound, r)

	return a
}

// Contains is b3AABB_Contains: does a fully contain b?
func (a AABB) Contains(b AABB) bool {
	if a.LowerBound.X > b.LowerBound.X || b.UpperBound.X > a.UpperBound.X {
		return false
	}
	if a.LowerBound.Y > b.LowerBound.Y || b.UpperBound.Y > a.UpperBound.Y {
		return false
	}
	if a.LowerBound.Z > b.LowerBound.Z || b.UpperBound.Z > a.UpperBound.Z {
		return false
	}

	return true
}

// Area is b3AABB_Area: get the surface area of an axis-aligned bounding box.
func (a AABB) Area() float32 {
	delta := Sub(a.UpperBound, a.LowerBound)
	return 2.0 * (float32(delta.X*delta.Y) + float32(delta.Y*delta.Z) + float32(delta.Z*delta.X))
}

// Center is b3AABB_Center: get the center of an axis-aligned bounding box.
func (a AABB) Center() Vec3 {
	return MulSV(0.5, Add(a.UpperBound, a.LowerBound))
}

// Extents is b3AABB_Extents: get the extents (half-widths) of an axis-aligned bounding box.
func (a AABB) Extents() Vec3 {
	return MulSV(0.5, Sub(a.UpperBound, a.LowerBound))
}

// Union is b3AABB_Union: get the union of two axis-aligned bounding boxes.
func (a AABB) Union(b AABB) AABB {
	var out AABB
	out.LowerBound = Min(a.LowerBound, b.LowerBound)
	out.UpperBound = Max(a.UpperBound, b.UpperBound)
	return out
}

// Inflate is b3AABB_Inflate: add uniform padding to an axis-aligned bounding box.
func (a AABB) Inflate(extension float32) AABB {
	radius := Vec3{extension, extension, extension}

	var out AABB
	out.LowerBound = Sub(a.LowerBound, radius)
	out.UpperBound = Add(a.UpperBound, radius)
	return out
}

// Overlaps is b3AABB_Overlaps: do two axis-aligned boxes overlap?
func (a AABB) Overlaps(b AABB) bool {
	// No intersection if separated along one axis
	if a.UpperBound.X < b.LowerBound.X || a.LowerBound.X > b.UpperBound.X {
		return false
	}
	if a.UpperBound.Y < b.LowerBound.Y || a.LowerBound.Y > b.UpperBound.Y {
		return false
	}
	if a.UpperBound.Z < b.LowerBound.Z || a.LowerBound.Z > b.UpperBound.Z {
		return false
	}

	// Overlapping on all axis means bounds are intersecting
	return true
}

// Transform is b3AABB_Transform( transform, a ): transform an axis-aligned bounding box. This can
// create a larger box than if you recomputed the AABB of the original shape with the transform
// applied.
func (a AABB) Transform(transform Transform) AABB {
	center := TransformPoint(transform, a.Center())
	m := MakeMatrixFromQuat(transform.Q)
	extent := MulMV(AbsMatrix3(m), a.Extents())
	out := AABB{Sub(center, extent), Add(center, extent)}
	return out
}

// ClosestPointToAABB gets the closest point on an axis-aligned bounding box.
func ClosestPointToAABB(point Vec3, a AABB) Vec3 {
	return Clamp(point, a.LowerBound, a.UpperBound)
}

// SegmentDistanceResult holds the closest points between to segments or infinite lines.
type SegmentDistanceResult struct {
	Point1    Vec3
	Fraction1 float32
	Point2    Vec3
	Fraction2 float32
}

// PointToSegmentDistance, LineDistance, SegmentDistance and the IsValid* functions are
// implemented below with math_functions.c.

// PORT: the C++ operator overloads (math_cpp group) are not ported.

// ---- math_functions.c ----

// IsValidFloat: is this a valid number? Not NaN or infinity.
func IsValidFloat(a float32) bool {
	if isnan(a) {
		return false
	}

	if isinf(a) {
		return false
	}

	return true
}

// isnan is the C isnan for float.
func isnan(a float32) bool {
	return a != a
}

// isinf is the C isinf for float.
func isinf(a float32) bool {
	return a > fltMax || a < -fltMax
}

// IsValidVec3: is this a valid vector? Not NaN or infinity.
func IsValidVec3(a Vec3) bool {
	if isnan(a.X) || isnan(a.Y) || isnan(a.Z) {
		return false
	}

	if isinf(a.X) || isinf(a.Y) || isinf(a.Z) {
		return false
	}

	return true
}

// IsValidQuat: is this a valid quaternion? Not NaN or infinity. Is normalized.
func IsValidQuat(a Quat) bool {
	if isnan(a.V.X) || isnan(a.V.Y) || isnan(a.V.Z) || isnan(a.S) {
		return false
	}

	if isinf(a.V.X) || isinf(a.V.Y) || isinf(a.V.Z) || isinf(a.S) {
		return false
	}

	return IsNormalizedQuat(a)
}

// IsValidTransform: is this a valid transform? Not NaN or infinity. Is normalized.
func IsValidTransform(a Transform) bool {
	return IsValidVec3(a.P) && IsValidQuat(a.Q)
}

// IsValidMatrix3: is this a valid matrix? Not NaN or infinity.
func IsValidMatrix3(a Matrix3) bool {
	return IsValidVec3(a.Cx) && IsValidVec3(a.Cy) && IsValidVec3(a.Cz)
}

// IsValidAABB: is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
func IsValidAABB(a AABB) bool {
	if IsValidVec3(a.LowerBound) == false {
		return false
	}

	if IsValidVec3(a.UpperBound) == false {
		return false
	}

	if a.LowerBound.X > a.UpperBound.X {
		return false
	}

	if a.LowerBound.Y > a.UpperBound.Y {
		return false
	}

	if a.LowerBound.Z > a.UpperBound.Z {
		return false
	}

	return true
}

// IsBoundedAABB: is this AABB reasonably close to the origin? See Huge.
func IsBoundedAABB(a AABB) bool {
	if a.LowerBound.X < -Huge() || a.LowerBound.Y < -Huge() || a.LowerBound.Z < -Huge() {
		return false
	}

	if a.UpperBound.X > Huge() || a.UpperBound.Y > Huge() || a.UpperBound.Z > Huge() {
		return false
	}

	return true
}

// IsSaneAABB: is this AABB valid and reasonable?
func IsSaneAABB(a AABB) bool {
	if IsValidAABB(a) == false {
		return false
	}

	if a.LowerBound.X < -Huge() || a.LowerBound.Y < -Huge() || a.LowerBound.Z < -Huge() {
		return false
	}

	if a.UpperBound.X > Huge() || a.UpperBound.Y > Huge() || a.UpperBound.Z > Huge() {
		return false
	}

	return true
}

// IsValidPlane: is this a valid plane? Normal is a unit vector. Not Nan or infinity.
func IsValidPlane(a Plane) bool {
	if IsValidVec3(a.Normal) == false {
		return false
	}

	if IsNormalized(a.Normal) == false {
		return false
	}

	return IsValidFloat(a.Offset)
}

// IsValidPosition: is this a valid world position? Not NaN or infinity.
func IsValidPosition(p Pos) bool {
	if isnan(p.X) || isnan(p.Y) || isnan(p.Z) {
		return false
	}

	if isinf(p.X) || isinf(p.Y) || isinf(p.Z) {
		return false
	}

	return true
}

// IsValidWorldTransform: is this a valid world transform? Not NaN or infinity. Rotation is normalized.
func IsValidWorldTransform(t WorldTransform) bool {
	return IsValidPosition(t.P) && IsValidQuat(t.Q)
}

// Atan2 computes an approximate arctangent in the range [-pi, pi].
// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
func Atan2(y, x float32) float32 {
	// Added check for (0,0) to match atan2f and avoid NaN
	if x == 0.0 && y == 0.0 {
		return 0.0
	}

	ax := AbsFloat(x)
	ay := AbsFloat(y)
	mx := MaxFloat(ay, ax)
	mn := MinFloat(ay, ax)
	a := mn / mx

	// Minimax polynomial approximation to atan(a) on [0,1]
	s := a * a
	c := s * a
	q := s * s
	r := float32(0.024840285*q) + 0.18681418
	t := float32(-0.094097948*q) - 0.33213072
	r = float32(r*s) + t
	r = float32(r*c) + a

	// Map to full circle
	if ay > ax {
		r = 1.57079637 - r
	}

	if x < 0 {
		r = 3.14159274 - r
	}

	if y < 0 {
		r = -r
	}

	return r
}

// ComputeCosSin computes the cosine and sine of an angle in radians.
// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
// this result.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
func ComputeCosSin(radians float32) CosSin {
	x := UnwindAngle(radians)
	var pi2 float32 = Pi * Pi

	// cosine needs angle in [-pi/2, pi/2]
	var c float32
	if x < -0.5*Pi {
		y := x + Pi
		y2 := float32(y * y)
		c = -(pi2 - float32(4.0*y2)) / (pi2 + y2)
	} else if x > 0.5*Pi {
		y := x - Pi
		y2 := float32(y * y)
		c = -(pi2 - float32(4.0*y2)) / (pi2 + y2)
	} else {
		y2 := float32(x * x)
		c = (pi2 - float32(4.0*y2)) / (pi2 + y2)
	}

	// sine needs angle in [0, pi]
	var s float32
	if x < 0.0 {
		y := x + Pi
		s = -16.0 * y * (Pi - y) / (float32(5.0*pi2) - float32(4.0*y*(Pi-y)))
	} else {
		s = 16.0 * x * (Pi - x) / (float32(5.0*pi2) - float32(4.0*x*(Pi-x)))
	}

	mag := sqrtf(float32(s*s) + float32(c*c))
	var invMag float32
	if mag > 0.0 {
		invMag = 1.0 / mag
	}
	cs := CosSin{float32(c * invMag), float32(s * invMag)}
	return cs
}

// MakeQuatFromMatrix extracts a quaternion from a rotation matrix.
func MakeQuatFromMatrix(m *Matrix3) Quat {
	c1 := m.Cx
	c2 := m.Cy
	c3 := m.Cz

	var q Quat

	trace := m.Cx.X + m.Cy.Y + m.Cz.Z
	if trace >= 0.0 {
		q.V.X = c2.Z - c3.Y
		q.V.Y = c3.X - c1.Z
		q.V.Z = c1.Y - c2.X
		q.S = trace + 1.0
	} else {
		if c1.X > c2.Y && c1.X > c3.Z {
			q.V.X = c1.X - c2.Y - c3.Z + 1.0
			q.V.Y = c2.X + c1.Y
			q.V.Z = c3.X + c1.Z
			q.S = c2.Z - c3.Y
		} else if c2.Y > c3.Z {
			q.V.X = c1.Y + c2.X
			q.V.Y = c2.Y - c3.Z - c1.X + 1.0
			q.V.Z = c3.Y + c2.Z
			q.S = c3.X - c1.Z
		} else {
			q.V.X = c1.Z + c3.X
			q.V.Y = c2.Z + c3.Y
			q.V.Z = c3.Z - c1.X - c2.Y + 1.0
			q.S = c1.Y - c2.X
		}
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return NormalizeQuat(q)
}

// ComputeQuatBetweenUnitVectors finds a quaternion that rotates one vector to another.
func ComputeQuatBetweenUnitVectors(v1, v2 Vec3) Quat {
	assert(IsNormalized(v1), "b3IsNormalized( v1 )")
	assert(IsNormalized(v2), "b3IsNormalized( v2 )")

	var out Quat

	m := Lerp(v1, v2, 0.5)
	tolerance := 100.0 * fltEpsilon
	if LengthSquared(m) > tolerance*tolerance {
		out.V = Cross(v1, m)
		out.S = Dot(v1, m)
	} else {
		// Anti-parallel: Use a perpendicular vector
		if AbsFloat(v1.X) > 0.5 {
			out.V.X = v1.Y
			out.V.Y = -v1.X
			out.V.Z = 0.0
		} else {
			out.V.X = 0.0
			out.V.Y = v1.Z
			out.V.Z = -v1.Y
		}

		out.S = 0.0
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return NormalizeQuat(out)
}

// LineDistance computes the closest points on two infinite lines.
func LineDistance(p1, d1, p2, d2 Vec3) SegmentDistanceResult {
	var result SegmentDistanceResult

	// Solve A*x = b
	a11 := Dot(d1, d1)
	a12 := -Dot(d1, d2)
	a21 := Dot(d2, d1)
	a22 := -Dot(d2, d2)

	w := Sub(p1, p2)
	b1 := -Dot(d1, w)
	b2 := -Dot(d2, w)

	det := float32(a11*a22) - float32(a12*a21)
	if det*det < 1000.0*fltMin {
		// Lines are parallel - project p2 onto line L1: x1 = p1 + s1 * d1
		s1 := Dot(Sub(p2, p1), d1) / Dot(d1, d1)
		var s2 float32 = 0.0

		result.Point1 = MulAdd(p1, s1, d1)
		result.Fraction1 = s1
		result.Point2 = MulAdd(p2, s2, d2)
		result.Fraction2 = s2

		return result
	}

	s1 := (float32(a22*b1) - float32(a12*b2)) / det
	s2 := (float32(a11*b2) - float32(a21*b1)) / det

	result.Point1 = MulAdd(p1, s1, d1)
	result.Fraction1 = s1
	result.Point2 = MulAdd(p2, s2, d2)
	result.Fraction2 = s2
	return result
}

// SegmentDistance computes the closest points on two line segments.
func SegmentDistance(p1, q1, p2, q2 Vec3) SegmentDistanceResult {
	var result SegmentDistanceResult

	d1 := Sub(q1, p1)
	d2 := Sub(q2, p2)
	r := Sub(p1, p2)

	a := Dot(d1, d1)
	b := Dot(d1, d2)
	c := Dot(d1, r)
	e := Dot(d2, d2)
	f := Dot(d2, r)

	// Check if one of the segments degenerates into a point
	if a < 100.0*fltEpsilon && e < 100.0*fltEpsilon {
		// Both segments degenerate into points
		result.Point1 = p1
		result.Fraction1 = 0.0
		result.Point2 = p2
		result.Fraction2 = 0.0

		return result
	}

	if a < 100.0*fltEpsilon {
		// First segment degenerates into a point
		s2 := ClampFloat(f/e, 0.0, 1.0)

		result.Point1 = p1
		result.Fraction1 = 0.0
		result.Point2 = MulAdd(p2, s2, d2)
		result.Fraction2 = s2

		return result
	}

	if e < 100.0*fltEpsilon {
		// Second segment degenerates into a point
		s1 := ClampFloat(-c/a, 0.0, 1.0)

		result.Point1 = MulAdd(p1, s1, d1)
		result.Fraction1 = s1
		result.Point2 = p2
		result.Fraction2 = 0.0

		return result
	}

	// Non-degenerate case
	denom := float32(a*e) - float32(b*b)
	var s1 float32
	if denom > 1000.0*fltMin {
		s1 = ClampFloat((float32(b*f)-float32(c*e))/denom, 0.0, 1.0)
	} else {
		s1 = 0.0
	}
	s2 := (float32(b*s1) + f) / e

	// Clamp lambda2 and recompute lambda1 if necessary
	if s2 < 0.0 {
		s1 = ClampFloat(-c/a, 0.0, 1.0)
		s2 = 0.0
	} else if s2 > 1.0 {
		s1 = ClampFloat((b-c)/a, 0.0, 1.0)
		s2 = 1.0
	}

	result.Point1 = MulAdd(p1, s1, d1)
	result.Fraction1 = s1
	result.Point2 = MulAdd(p2, s2, d2)
	result.Fraction2 = s2

	return result
}

// PointToSegmentDistance computes the closest point on the segment a-b to the target q.
func PointToSegmentDistance(a, b, q Vec3) Vec3 {
	ab := Sub(b, a)
	aq := Sub(q, a)

	alpha := Dot(ab, aq)

	if alpha <= 0.0 {
		// q projects outside interval [a, b] on the side of a
		return a
	} else {
		denominator := Dot(ab, ab)
		if alpha > denominator {
			// q projects outside interval [a, b] on the side of b
			return b
		} else {
			// q projects inside interval [a, b]
			alpha /= denominator
			return MulAdd(a, alpha, ab)
		}
	}
}

// closestPointOnTriangle is b3ClosestPointOnTriangle.
func closestPointOnTriangle(a, b, c, q Vec3) trianglePoint {
	// Check if P lies in vertex region of A
	ab := Sub(b, a)
	ac := Sub(c, a)
	aq := Sub(q, a)

	d1 := Dot(ab, aq)
	d2 := Dot(ac, aq)
	if d1 <= 0.0 && d2 <= 0.0 {
		return trianglePoint{a, FeatureVertex1}
	}

	// Check if P lies in vertex region of B
	bq := Sub(q, b)

	d3 := Dot(ab, bq)
	d4 := Dot(ac, bq)
	if d3 > 0.0 && d4 <= d3 {
		return trianglePoint{b, FeatureVertex2}
	}

	// Check if P lies in edge region AB
	vc := float32(d1*d4) - float32(d3*d2)
	if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
		t := d1 / (d1 - d3)
		return trianglePoint{MulAdd(a, t, ab), FeatureEdge1}
	}

	// Check if P lies in vertex region of C
	cq := Sub(q, c)

	d5 := Dot(ab, cq)
	d6 := Dot(ac, cq)
	if d6 >= 0.0 && d5 <= d6 {
		return trianglePoint{c, FeatureVertex3}
	}

	// Check if P lies in edge region AC
	vb := float32(d5*d2) - float32(d1*d6)
	if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
		t := d2 / (d2 - d6)
		return trianglePoint{MulAdd(a, t, ac), FeatureEdge3}
	}

	// Check if P lies in edge region of BC
	va := float32(d3*d6) - float32(d5*d4)
	if va <= 0.0 && d4 >= d3 && d5 >= d6 {
		bc := Sub(c, b)

		t := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		return trianglePoint{MulAdd(b, t, bc), FeatureEdge2}
	}

	// P inside face region ABC
	t1 := vb / (va + vb + vc)
	t2 := vc / (va + vb + vc)

	p := MulAdd(a, t1, ab)
	p = MulAdd(p, t2, ac)
	return trianglePoint{p, FeatureTriangleFace}
}

// sphereInertia is b3SphereInertia.
func sphereInertia(mass, radius float32) Matrix3 {
	i := float32(0.4 * mass * radius * radius)
	return makeDiagonalMatrix(i, i, i)
}

// cylinderInertia is b3CylinderInertia.
func cylinderInertia(mass, radius, height float32) Matrix3 {
	ixx := mass * (float32(3*radius*radius) + float32(height*height)) / 12.0
	iyy := 0.5 * mass * radius * radius
	return makeDiagonalMatrix(ixx, iyy, ixx)
}

// boxInertia is b3BoxInertia.
func boxInertia(mass float32, min, max Vec3) Matrix3 {
	delta := Sub(max, min)
	ixx := mass * (float32(delta.Y*delta.Y) + float32(delta.Z*delta.Z)) / 12.0
	iyy := mass * (float32(delta.X*delta.X) + float32(delta.Z*delta.Z)) / 12.0
	izz := mass * (float32(delta.X*delta.X) + float32(delta.Y*delta.Y)) / 12.0

	return makeDiagonalMatrix(ixx, iyy, izz)
}

// Steiner gets the inertia tensor of an offset point.
// https://en.wikipedia.org/wiki/Parallel_axis_theorem
func Steiner(mass float32, origin Vec3) Matrix3 {
	// Usage: Io = Ic + Is and Ic = Io - Is
	ixx := mass * (float32(origin.Y*origin.Y) + float32(origin.Z*origin.Z))
	iyy := mass * (float32(origin.X*origin.X) + float32(origin.Z*origin.Z))
	izz := mass * (float32(origin.X*origin.X) + float32(origin.Y*origin.Y))
	ixy := -mass * origin.X * origin.Y
	ixz := -mass * origin.X * origin.Z
	iyz := -mass * origin.Y * origin.Z

	// Write
	var out Matrix3
	out.Cx.X = ixx
	out.Cy.X = ixy
	out.Cz.X = ixz
	out.Cx.Y = ixy
	out.Cy.Y = iyy
	out.Cz.Y = iyz
	out.Cx.Z = ixz
	out.Cy.Z = iyz
	out.Cz.Z = izz

	return out
}

// IsValidRay: use this to ensure your ray cast input is valid and avoid internal assertions.
func IsValidRay(input *RayCastInput) bool {
	isValid := IsValidVec3(input.Origin) && IsValidVec3(input.Translation) &&
		IsValidFloat(input.MaxFraction) && 0.0 <= input.MaxFraction && input.MaxFraction < Huge()
	return isValid
}
