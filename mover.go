// Port of src/mover.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// SolvePlanes solves the position of a mover that satisfies the given collision planes.
// targetDelta is the desired translation from the position used to generate the collision planes.
// PORT: b3SolvePlanes( targetDelta, planes, count ) takes the planes as a slice; the push
// results are written back into it.
func SolvePlanes(targetDelta Vec3, planes []CollisionPlane) PlaneSolverResult {
	count := len(planes)
	for i := 0; i < count; i++ {
		planes[i].Push = 0.0
	}

	delta := targetDelta
	tolerance := LinearSlop()

	var iteration int
	for iteration = 0; iteration < 20; iteration++ {
		totalPush := float32(0.0)
		for planeIndex := 0; planeIndex < count; planeIndex++ {
			plane := &planes[planeIndex]

			// Add slop to prevent jitter
			separation := planeSeparation(plane.Plane, delta) + LinearSlop()

			push := -separation

			// Clamp accumulated push
			accumulatedPush := plane.Push
			plane.Push = ClampFloat(plane.Push+push, 0.0, plane.PushLimit)
			push = plane.Push - accumulatedPush
			delta = MulAdd(delta, push, plane.Plane.Normal)

			// Track maximum push for convergence
			totalPush += AbsFloat(push)
		}

		if totalPush < tolerance {
			break
		}
	}

	return PlaneSolverResult{
		Delta:          delta,
		IterationCount: int32(iteration),
	}
}

// ClipVector clips the velocity against the given collision planes. Planes with zero push or
// ClipVelocity set to false are skipped.
// PORT: b3ClipVector( vector, planes, count ) takes the planes as a slice.
func ClipVector(vector Vec3, planes []CollisionPlane) Vec3 {
	v := vector

	count := len(planes)
	for planeIndex := 0; planeIndex < count; planeIndex++ {
		plane := &planes[planeIndex]
		if plane.Push == 0.0 || plane.ClipVelocity == false {
			continue
		}

		v = MulSub(v, MinFloat(0.0, Dot(v, plane.Plane.Normal)), plane.Plane.Normal)
	}

	return v
}
