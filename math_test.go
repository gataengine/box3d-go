// Port of test/test_math.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"math"
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
)

// 0.0023 degrees
const atanTol float32 = 0.00004

// PORT: libm cosf, sinf and atan2f are computed in float64 and rounded. They are only reference
// values for tolerance checks.
func cosf(x float32) float32      { return float32(math.Cos(float64(x))) }
func sinf(x float32) float32      { return float32(math.Sin(float64(x))) }
func atan2f(y, x float32) float32 { return float32(math.Atan2(float64(y), float64(x))) }

var _ = Describe("MathTest", func() {
	It("MathTest", func() {
		for t := float32(-10.0); t < 10.0; t += 0.01 {
			angle := Pi * t
			cs := ComputeCosSin(angle)
			c := cosf(angle)
			s := sinf(angle)

			// The cosine and sine approximations are accurate to about 0.1 degrees (0.002 radians)
			// printf( "%g %g\n", r.c - c, r.s - s );
			ensureSmall(cs.Cosine-c, 0.002)
			ensureSmall(cs.Sine-s, 0.002)

			xn := UnwindAngle(angle)
			a := Atan2(s, c)
			ensure(IsValidFloat(a))

			diff := AbsFloat(a - xn)

			// The two results can be off by 360 degrees (-pi and pi)
			if diff > Pi {
				diff -= 2.0 * Pi
			}

			// The approximate atan2 is quite accurate
			ensureSmall(diff, atanTol)
		}

		for y := float32(-1.0); y <= 1.0; y += 0.01 {
			for x := float32(-1.0); x <= 1.0; x += 0.01 {
				a1 := Atan2(y, x)
				a2 := atan2f(y, x)
				diff := AbsFloat(a1 - a2)
				ensure(IsValidFloat(a1))
				ensureSmall(diff, atanTol)
			}
		}

		{
			a1 := Atan2(1.0, 0.0)
			a2 := atan2f(1.0, 0.0)
			diff := AbsFloat(a1 - a2)
			ensure(IsValidFloat(a1))
			ensureSmall(diff, atanTol)
		}

		{
			a1 := Atan2(-1.0, 0.0)
			a2 := atan2f(-1.0, 0.0)
			diff := AbsFloat(a1 - a2)
			ensure(IsValidFloat(a1))
			ensureSmall(diff, atanTol)
		}

		{
			a1 := Atan2(0.0, 1.0)
			a2 := atan2f(0.0, 1.0)
			diff := AbsFloat(a1 - a2)
			ensure(IsValidFloat(a1))
			ensureSmall(diff, atanTol)
		}

		{
			a1 := Atan2(0.0, -1.0)
			a2 := atan2f(0.0, -1.0)
			diff := AbsFloat(a1 - a2)
			ensure(IsValidFloat(a1))
			ensureSmall(diff, atanTol)
		}

		{
			a1 := Atan2(0.0, 0.0)
			a2 := atan2f(0.0, 0.0)
			diff := AbsFloat(a1 - a2)
			ensure(IsValidFloat(a1))
			ensureSmall(diff, atanTol)
		}

		zero := Vec3{}
		one := Vec3{1.0, 1.0, 1.0}
		two := Vec3{2.0, 2.0, 2.0}

		v := Add(one, two)
		ensure(v.X == 3.0 && v.Y == 3.0)

		v = Sub(zero, two)
		ensure(v.X == -2.0 && v.Y == -2.0)

		v = Add(two, two)
		ensure(v.X != 5.0 && v.Y != 5.0)

		axis := Normalize(Vec3{-0.75, 0.5, 1.0})
		transform1 := Transform{Vec3{-2.0, 3.0, 0.0}, QuatIdentity}
		transform2 := Transform{Vec3{1.0, 0.0, 0.0}, MakeQuatFromAxisAngle(axis, Pi)}

		transform := MulTransforms(transform2, transform1)

		v = TransformPoint(transform2, TransformPoint(transform1, two))

		u := TransformPoint(transform, two)

		ensureSmall(u.X-v.X, 10.0*fltEpsilon)
		ensureSmall(u.Y-v.Y, 10.0*fltEpsilon)

		v = TransformPoint(transform1, two)
		v = InvTransformPoint(transform1, v)

		ensureSmall(v.X-two.X, 8.0*fltEpsilon)
		ensureSmall(v.Y-two.Y, 8.0*fltEpsilon)

		relTransform := InvMulTransforms(transform1, transform2)
		v = InvTransformPoint(transform1, TransformPoint(transform2, two))
		u = TransformPoint(relTransform, two)
		ensureSmall(u.X-v.X, 10.0*fltEpsilon)
		ensureSmall(u.Y-v.Y, 10.0*fltEpsilon)

		{
			axis = Vec3{0.0, 0.0, 1.0}
			q1 := MakeQuatFromAxisAngle(axis, -0.5*Pi)
			q2 := ComputeQuatBetweenUnitVectors(Vec3{1.0, 0.0, 0.0}, Vec3{0.0, -1.0, 0.0})

			ensureSmall(q1.V.X-q2.V.X, fltEpsilon)
			ensureSmall(q1.V.Y-q2.V.Y, fltEpsilon)
			ensureSmall(q1.V.Z-q2.V.Z, fltEpsilon)
			ensureSmall(q1.S-q2.S, fltEpsilon)

			q3 := NormalizeQuat(Quat{Vec3{1.0, -2.0, 3.0}, 4.0})
			q4 := InvMulQuat(q3, q1)
			q5 := MulQuat(q3, q4)
			ensureSmall(q1.V.X-q5.V.X, fltEpsilon)
			ensureSmall(q1.V.Y-q5.V.Y, fltEpsilon)
			ensureSmall(q1.V.Z-q5.V.Z, fltEpsilon)
			ensureSmall(q1.S-q5.S, fltEpsilon)

			q6 := ComputeQuatBetweenUnitVectors(Vec3{0.0, 1.0, 0.0}, Vec3{0.0, -1.0, 0.0})
			ensureSmall(q6.S, fltEpsilon)
		}

		v = Normalize(Vec3{0.2, -0.5, 3.0})
		for z := float32(-1.0); z <= 1.0; z += 0.02 {
			for y := float32(-1.0); y <= 1.0; y += 0.02 {
				for x := float32(-1.0); x <= 1.0; x += 0.02 {
					if x == 0.0 && y == 0.0 && z == 0.0 {
						continue
					}

					u = Normalize(Vec3{x, y, z})

					r := ComputeQuatBetweenUnitVectors(v, u)
					ensure(IsValidQuat(r))

					w := RotateVector(r, v)

					ensureSmall(Dot(r.V, Cross(u, w))-scalarTripleProduct(r.V, u, w), fltEpsilon)

					// The quaternion between vectors can have lots of round off error at large angles.
					ensureSmall(w.X-u.X, 0.001)
					ensureSmall(w.Y-u.Y, 0.001)
					ensureSmall(w.Z-u.Z, 0.001)

					// Twist angle testing
					var twist float32
					if r.S < 0.0 {
						twist = Atan2(-r.V.Z, -r.S)
					} else {
						twist = Atan2(r.V.Z, r.S)
					}
					twist *= 2.0
					ensure(-Pi <= twist && twist <= Pi)
				}
			}
		}

		{
			// More twist angle testing
			q := Quat{V: Vec3{-0.0558656752, -0.188799798, 0.00689807534}, S: -0.980401039}
			var twist float32
			if q.S < 0.0 {
				twist = Atan2(-q.V.Z, -q.S)
			} else {
				twist = Atan2(q.V.Z, q.S)
			}
			twist *= 2.0
			ensure(-Pi <= twist && twist <= Pi)
		}

		{
			m := Matrix3{Vec3{3.0, 1.0, -1.0}, Vec3{-1.0, 3.0, 1.0}, Vec3{1.0, -1.0, 3.0}}
			invM := InvertMatrix(m)
			a := MulMM(m, invM)
			ensureSmall(a.Cx.X-1.0, fltEpsilon)
			ensureSmall(a.Cx.Y, fltEpsilon)
			ensureSmall(a.Cx.Z, fltEpsilon)
			ensureSmall(a.Cy.X, fltEpsilon)
			ensureSmall(a.Cy.Y-1.0, fltEpsilon)
			ensureSmall(a.Cy.Z, fltEpsilon)
			ensureSmall(a.Cz.X, fltEpsilon)
			ensureSmall(a.Cz.Y, fltEpsilon)
			ensureSmall(a.Cz.Z-1.0, fltEpsilon)

			v = Vec3{1.0, -2.0, 3.0}
			u = MulMV(invM, MulMV(m, v))
			ensureSmall(v.X-u.X, fltEpsilon)
			ensureSmall(v.Y-u.Y, fltEpsilon)
			ensureSmall(v.Z-u.Z, fltEpsilon)

			w := MulMV(invM, v)
			u = Solve3(m, v)
			ensureSmall(w.X-u.X, fltEpsilon)
			ensureSmall(w.Y-u.Y, fltEpsilon)
			ensureSmall(w.Z-u.Z, fltEpsilon)
		}

		{
			m := matrix2{Vec2{3.0, 1.0}, Vec2{-1.0, 3.0}}
			invM := invert2(m)
			a := mulMM2(m, invM)
			ensureSmall(a.cx.X-1.0, fltEpsilon)
			ensureSmall(a.cx.Y, fltEpsilon)
			ensureSmall(a.cy.X, fltEpsilon)
			ensureSmall(a.cy.Y-1.0, fltEpsilon)

			v2 := Vec2{1.0, -2.0}
			u2 := mulMV2(invM, mulMV2(m, v2))
			ensureSmall(v2.X-u2.X, fltEpsilon)
			ensureSmall(v2.Y-u2.Y, fltEpsilon)

			w := mulMV2(invM, v2)
			u2 = solve2(m, v2)
			ensureSmall(w.X-u2.X, fltEpsilon)
			ensureSmall(w.Y-u2.Y, fltEpsilon)

			w = mulMV2(m, u2)
			ensureSmall(w.X-v2.X, 10.0*fltEpsilon)
			ensureSmall(w.Y-v2.Y, 10.0*fltEpsilon)
		}

		for range 100 {
			a := randomFloat()
			b := float64(a)
			c := float32(b)
			ensure(c == a)
		}

		q1 := QuatIdentity
		q2 := MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)
		n := 100
		for i := 0; i <= n; i++ {
			alpha := float32(i) / float32(n)
			q := NLerp(q1, q2, alpha)
			angle := GetTwistAngle(q)
			ensureSmall(alpha*0.5*Pi-angle, 1.0*DegToRad)
			//printf( "angle = [%g %g %g]\n", alpha, alpha * 0.5f * B3_PI, angle );
		}

		{
			normal := Vec3{0.504055440, 0.621548057, 0.599671543}
			perp := arbitraryPerp(normal)
			ensureSmall(Dot(normal, perp), 2.0*fltEpsilon)
		}

		{
			// World position boundary helpers. The query agrees with the built type sizes.
			ensure(IsDoublePrecision() == (unsafe.Sizeof(Pos{}) > unsafe.Sizeof(Vec3{})))

			// Deltas and offsets round trip exactly for representable inputs in both modes.
			a := Vec3{3.0, -5.0, 2.0}
			b := Vec3{1.0, 4.0, -6.0}
			pa := ToPos(a)
			pb := ToPos(b)

			d := SubPos(pa, pb)
			sub := Sub(a, b)
			ensure(d.X == sub.X && d.Y == sub.Y && d.Z == sub.Z)

			back := SubPos(OffsetPos(pb, sub), pa)
			ensure(back.X == 0.0 && back.Y == 0.0 && back.Z == 0.0)

			r := ToVec3(pa)
			ensure(r.X == a.X && r.Y == a.Y && r.Z == a.Z)

			ensure(IsValidPosition(pa))

			// World transform relative ops match the pure float transform ops. Float mode is
			// bit identical, double mode keeps the relative result in float.
			axis := Normalize(Vec3{0.3, -0.7, 0.5})
			tA := Transform{a, MakeQuatFromAxisAngle(axis, 0.4)}
			tB := Transform{b, MakeQuatFromAxisAngle(axis, -1.1)}
			wA := MakeWorldTransform(tA)
			wB := MakeWorldTransform(tB)
			ensure(IsValidWorldTransform(wA))

			relRef := InvMulTransforms(tA, tB)
			rel := InvMulWorldTransforms(wA, wB)
			ensureSmall(rel.P.X-relRef.P.X, 1.0e-5)
			ensureSmall(rel.P.Y-relRef.P.Y, 1.0e-5)
			ensureSmall(rel.P.Z-relRef.P.Z, 1.0e-5)
			ensureSmall(rel.Q.S-relRef.Q.S, 1.0e-5)

			// Local point to world and back.
			local := Vec3{0.5, -0.25, 1.5}
			back2 := InvTransformWorldPoint(wA, TransformWorldPoint(wA, local))
			ensureSmall(back2.X-local.X, 1.0e-5)
			ensureSmall(back2.Y-local.Y, 1.0e-5)
			ensureSmall(back2.Z-local.Z, 1.0e-5)

			// Compose with a local transform, then strip it back off.
			relAB := InvMulWorldTransforms(wA, MulWorldTransforms(wA, tB))
			ensureSmall(relAB.P.X-tB.P.X, 1.0e-5)
			ensureSmall(relAB.P.Y-tB.P.Y, 1.0e-5)
			ensureSmall(relAB.P.Z-tB.P.Z, 1.0e-5)
		}

		// PORT: the BOX3D_DOUBLE_PRECISION block is not ported (double precision is not ported).
	})
})
