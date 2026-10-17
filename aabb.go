// Port of src/aabb.h and src/aabb.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- aabb.h ----

// rayCastAABB: ray cast an AABB. This is a custom function used by height fields. Implemented below.

// perimeter gets the surface area (perimeter)
func perimeter(a AABB) float32 {
	wx := a.UpperBound.X - a.LowerBound.X
	wy := a.UpperBound.Y - a.LowerBound.Y
	wz := a.UpperBound.Z - a.LowerBound.Z
	return 2.0 * (float32(wx*wz) + float32(wy*wx) + float32(wz*wy))
}

// enlargeAABB enlarges a to contain b
// Returns true if the AABB grew
func enlargeAABB(a *AABB, b AABB) bool {
	changed := false
	if b.LowerBound.X < a.LowerBound.X {
		a.LowerBound.X = b.LowerBound.X
		changed = true
	}

	if b.LowerBound.Y < a.LowerBound.Y {
		a.LowerBound.Y = b.LowerBound.Y
		changed = true
	}

	if b.LowerBound.Z < a.LowerBound.Z {
		a.LowerBound.Z = b.LowerBound.Z
		changed = true
	}

	if a.UpperBound.X < b.UpperBound.X {
		a.UpperBound.X = b.UpperBound.X
		changed = true
	}

	if a.UpperBound.Y < b.UpperBound.Y {
		a.UpperBound.Y = b.UpperBound.Y
		changed = true
	}

	if a.UpperBound.Z < b.UpperBound.Z {
		a.UpperBound.Z = b.UpperBound.Z
		changed = true
	}

	return changed
}

// PORT: the disabled (#if 0) b3OverlapAABBs is not ported.

func farthestPointOnAABB(b AABB, p Vec3) Vec3 {
	var out Vec3
	if (p.X - b.LowerBound.X) > (b.UpperBound.X - p.X) {
		out.X = b.LowerBound.X
	} else {
		out.X = b.UpperBound.X
	}
	if (p.Y - b.LowerBound.Y) > (b.UpperBound.Y - p.Y) {
		out.Y = b.LowerBound.Y
	} else {
		out.Y = b.UpperBound.Y
	}
	if (p.Z - b.LowerBound.Z) > (b.UpperBound.Z - p.Z) {
		out.Z = b.LowerBound.Z
	} else {
		out.Z = b.UpperBound.Z
	}
	return out
}

// ---- aabb.c ----

// rayCastAABB is b3RayCastAABB.
// Similar to Real-time Collision Detection, p179.
// todo try
// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
// PORT: the minFraction and maxFraction out-parameters are the second and third return values.
// They are zero when the ray misses (upstream leaves them unwritten).
func rayCastAABB(a AABB, p1, p2 Vec3) (bool, float32, float32) {
	// Ray direction and length
	d := Sub(p2, p1)
	rayLength := Length(d)

	// Handle degenerate ray
	if rayLength < fltEpsilon {
		// Check if point is inside AABB
		if p1.X >= a.LowerBound.X && p1.X <= a.UpperBound.X && p1.Y >= a.LowerBound.Y && p1.Y <= a.UpperBound.Y &&
			p1.Z >= a.LowerBound.Z && p1.Z <= a.UpperBound.Z {
			return true, 0.0, 0.0
		}

		return false, 0.0, 0.0
	}

	rayDir := MulSV(1.0/rayLength, d)

	// Slab method for ray-AABB intersection
	var tMin float32 = 0.0
	tMax := rayLength

	// x-axis
	{
		rayComponent := rayDir.X
		rayStart := p1.X
		boxMin := a.LowerBound.X
		boxMax := a.UpperBound.X

		if AbsFloat(rayComponent) < fltEpsilon {
			// Ray is parallel to slab, check if ray origin is within slab
			if rayStart < boxMin || rayStart > boxMax {
				return false, 0.0, 0.0
			}
		} else {
			// Compute intersection distances
			t1 := (boxMin - rayStart) / rayComponent
			t2 := (boxMax - rayStart) / rayComponent

			// Ensure t1 <= t2
			if t1 > t2 {
				temp := t1
				t1 = t2
				t2 = temp
			}

			// Update intersection interval
			tMin = MaxFloat(tMin, t1)
			tMax = MinFloat(tMax, t2)

			// Check for no intersection
			if tMin > tMax {
				return false, 0.0, 0.0
			}
		}
	}

	// y-axis
	{
		rayComponent := rayDir.Y
		rayStart := p1.Y
		boxMin := a.LowerBound.Y
		boxMax := a.UpperBound.Y

		if AbsFloat(rayComponent) < fltEpsilon {
			// Ray is parallel to slab, check if ray origin is within slab
			if rayStart < boxMin || rayStart > boxMax {
				return false, 0.0, 0.0
			}
		} else {
			// Compute intersection distances
			t1 := (boxMin - rayStart) / rayComponent
			t2 := (boxMax - rayStart) / rayComponent

			// Ensure t1 <= t2
			if t1 > t2 {
				temp := t1
				t1 = t2
				t2 = temp
			}

			// Update intersection interval
			tMin = MaxFloat(tMin, t1)
			tMax = MinFloat(tMax, t2)

			// Check for no intersection
			if tMin > tMax {
				return false, 0.0, 0.0
			}
		}
	}

	// z-axis
	{
		rayComponent := rayDir.Z
		rayStart := p1.Z
		boxMin := a.LowerBound.Z
		boxMax := a.UpperBound.Z

		if AbsFloat(rayComponent) < fltEpsilon {
			// Ray is parallel to slab, check if ray origin is within slab
			if rayStart < boxMin || rayStart > boxMax {
				return false, 0.0, 0.0
			}
		} else {
			// Compute intersection distances
			t1 := (boxMin - rayStart) / rayComponent
			t2 := (boxMax - rayStart) / rayComponent

			// Ensure t1 <= t2
			if t1 > t2 {
				temp := t1
				t1 = t2
				t2 = temp
			}

			// Update intersection interval
			tMin = MaxFloat(tMin, t1)
			tMax = MinFloat(tMax, t2)

			// Check for no intersection
			if tMin > tMax {
				return false, 0.0, 0.0
			}
		}
	}

	// Check if intersection is behind ray start
	if tMax < 0.0 {
		return false, 0.0, 0.0
	}

	// Convert distances to fractions
	minFraction := ClampFloat(tMin/rayLength, 0.0, 1.0)
	maxFraction := ClampFloat(tMax/rayLength, 0.0, 1.0)

	return true, minFraction, maxFraction
}
