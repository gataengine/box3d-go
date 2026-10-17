// Port of test/test_distance.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// encroachCastInput builds a unit box with a segment resting 2mm from its +x face: inside the cast
// target distance, not overlapped
func encroachCastInput(translationB Vec3) ShapeCastPairInput {
	vas := []Vec3{{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}}
	vbs := []Vec3{
		{1.002, -1.0, 0.0},
		{1.002, 1.0, 0.0},
	}

	var input ShapeCastPairInput
	input.ProxyA = ShapeProxy{vas, int32(arrayCount(vas)), 0.0}
	input.ProxyB = ShapeProxy{vbs, int32(arrayCount(vbs)), 0.0}
	input.Transform = TransformIdentity
	input.TranslationB = translationB
	input.MaxFraction = 1.0
	input.CanEncroach = true
	return input
}

var _ = Describe("DistanceTest", func() {
	It("SegmentDistanceTest", func() {
		p1 := Vec3{-1.0, -1.0, 0.0}
		q1 := Vec3{-1.0, 1.0, 0.0}
		p2 := Vec3{2.0, 0.0, 0.0}
		q2 := Vec3{1.0, 0.0, 0.0}

		result := SegmentDistance(p1, q1, p2, q2)

		ensureSmall(result.Fraction1-0.5, fltEpsilon)
		ensureSmall(result.Fraction2-1.0, fltEpsilon)
		ensureSmall(result.Point1.X+1.0, fltEpsilon)
		ensureSmall(result.Point1.Y, fltEpsilon)
		ensureSmall(result.Point1.Z, fltEpsilon)
		ensureSmall(result.Point2.X-1.0, fltEpsilon)
		ensureSmall(result.Point2.Y, fltEpsilon)
		ensureSmall(result.Point2.Z, fltEpsilon)
	})

	It("ShapeDistanceTest", func() {
		vas := []Vec3{{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}}

		vbs := []Vec3{
			{2.0, -1.0, 0.0},
			{2.0, 1.0, 0.0},
		}

		var input DistanceInput
		input.ProxyA = ShapeProxy{vas, int32(arrayCount(vas)), 0.0}
		input.ProxyB = ShapeProxy{vbs, int32(arrayCount(vbs)), 0.0}
		input.Transform = TransformIdentity
		input.UseRadii = false

		cache := SimplexCache{}
		output := ShapeDistance(&input, &cache, nil)

		ensureSmall(output.Distance-1.0, fltEpsilon)
	})

	It("ShapeCastTest", func() {
		vas := []Vec3{{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}}

		vbs := []Vec3{
			{2.0, -1.0, 0.0},
			{2.0, 1.0, 0.0},
		}

		var input ShapeCastPairInput
		input.ProxyA = ShapeProxy{vas, int32(arrayCount(vas)), 0.0}
		input.ProxyB = ShapeProxy{vbs, int32(arrayCount(vbs)), 0.0}
		input.Transform = TransformIdentity
		input.TranslationB = Vec3{-2.0, 0.0, 0.0}
		input.MaxFraction = 1.0
		input.CanEncroach = false

		output := ShapeCast(&input)

		ensure(output.Hit)
		ensureSmall(output.Fraction-0.5, 0.005)
	})

	It("ShapeCastEncroachRecedeTest", func() {
		input := encroachCastInput(Vec3{1.0, 0.0, 0.0})
		output := ShapeCast(&input)
		ensure(output.Hit == false)
	})

	It("ShapeCastEncroachSlideTest", func() {
		input := encroachCastInput(Vec3{0.0, 2.0, 0.0})
		output := ShapeCast(&input)
		ensure(output.Hit == false)
	})

	It("ShapeCastEncroachClosingTest", func() {
		input := encroachCastInput(Vec3{-1.0, 0.0, 0.0})
		output := ShapeCast(&input)
		ensure(output.Hit)
		ensureSmall(output.Fraction, fltEpsilon)
		ensureSmall(output.Normal.X-1.0, 0.001)
	})

	It("ShapeCastEncroachGradedTest", func() {
		// 5mm gap: closing consumes the gap down to the retained rest separation
		vbs := []Vec3{
			{1.005, -1.0, 0.0},
			{1.005, 1.0, 0.0},
		}

		input := encroachCastInput(Vec3{-1.0, 0.0, 0.0})
		input.ProxyB = ShapeProxy{vbs, int32(arrayCount(vbs)), 0.0}
		output := ShapeCast(&input)
		ensure(output.Hit)
		ensureSmall(output.Fraction-0.0025, 0.0005)
		ensureSmall(output.Normal.X-1.0, 0.001)
	})

	It("ShapeCastEncroachMaxFractionTest", func() {
		// 5mm gap closes at fraction 0.0025
		vbs := []Vec3{
			{1.005, -1.0, 0.0},
			{1.005, 1.0, 0.0},
		}

		input := encroachCastInput(Vec3{-1.0, 0.0, 0.0})
		input.ProxyB = ShapeProxy{vbs, int32(arrayCount(vbs)), 0.0}
		input.MaxFraction = 0.002
		output := ShapeCast(&input)
		ensure(output.Hit == false)

		input.MaxFraction = 0.003
		output = ShapeCast(&input)
		ensure(output.Hit)
		ensureSmall(output.Fraction-0.0025, 0.0005)
	})

	It("ShapeCastEncroachGrazeTest", func() {
		// mostly tangential slide with a slight closing component: free
		vbs := []Vec3{
			{1.005, -1.0, 0.0},
			{1.005, 1.0, 0.0},
		}

		input := encroachCastInput(Vec3{-0.001, 2.0, 0.0})
		input.ProxyB = ShapeProxy{vbs, int32(arrayCount(vbs)), 0.0}
		output := ShapeCast(&input)
		ensure(output.Hit == false)
	})

	It("ShapeCastStartTouchingDefaultTest", func() {
		input := encroachCastInput(Vec3{1.0, 0.0, 0.0})
		input.CanEncroach = false
		output := ShapeCast(&input)
		ensure(output.Hit)
	})

	It("ShapeCastEncroachOverlapTest", func() {
		vcs := []Vec3{
			{0.5, -1.0, 0.0},
			{0.5, 1.0, 0.0},
		}

		input := encroachCastInput(Vec3{1.0, 0.0, 0.0})
		input.ProxyB = ShapeProxy{vcs, int32(arrayCount(vcs)), 0.0}
		output := ShapeCast(&input)
		ensure(output.Hit)
	})

	It("TimeOfImpactTest", func() {
		vas := []Vec3{{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}}

		vbs := []Vec3{
			{2.0, -1.0, 0.0},
			{2.0, 1.0, 0.0},
		}

		var input TOIInput
		input.ProxyA = ShapeProxy{vas, int32(arrayCount(vas)), 0.0}
		input.ProxyB = ShapeProxy{vbs, int32(arrayCount(vbs)), 0.0}
		input.SweepA = Sweep{Vec3Zero, Vec3Zero, Vec3Zero, QuatIdentity, QuatIdentity}
		input.SweepB = Sweep{Vec3Zero, Vec3Zero, Vec3{-2.0, 0.0, 0.0}, QuatIdentity, QuatIdentity}
		input.MaxFraction = 1.0

		output := TimeOfImpact(&input)

		ensure(output.State == TOIStateHit)
		ensureSmall(output.Fraction-0.5, 0.005)
	})
})
