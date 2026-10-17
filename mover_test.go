// Port of test/test_mover.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
)

// PORT: the tests that need world, body, shape, mesh, height field or compound code are pending
// (PIt) until those files land; each names the missing upstream files. Port their bodies from
// test/test_mover.c then.

// moverMakeTwoMaterialMesh is MakeTwoMaterialMesh from test_mover.c.
// PORT: renamed; test_compound.c has a static function of the same name.
func moverMakeTwoMaterialMesh() *MeshData {
	vertices := [6]Vec3{
		{-3.0, 0.0, -1.0}, {-2.0, 0.0, 1.0}, {-1.0, 0.0, -1.0},
		{1.0, 0.0, -1.0}, {2.0, 0.0, 1.0}, {3.0, 0.0, -1.0},
	}
	indices := [6]int32{0, 1, 2, 3, 4, 5}
	materialIndices := [2]uint8{0, 1}

	def := MeshDef{}
	def.Vertices = vertices[:]
	def.Stride = int(unsafe.Sizeof(Vec3{}))
	def.Indices = indices[:]
	def.MaterialIndices = materialIndices[:]
	def.VertexCount = 6
	def.TriangleCount = 2

	return CreateMesh(&def, nil)
}

var _ = Describe("MoverTest", func() {
	It("GamePlanes", func() {
		// This scenario takes many iterations because the target is deep into the plane.
		var planes [3]CollisionPlane
		planes[0].Plane.Normal = Vec3{0.0, -0.23941046, 0.970918416}
		planes[0].Plane.Offset = 0.390724182
		planes[0].PushLimit = fltMax
		planes[1].Plane.Normal = Vec3{0.0, 0.0, 1.0}
		planes[1].Plane.Offset = 1.49998093
		planes[1].PushLimit = fltMax

		target := Vec3{-2.5390625, 0.0, -73.6880798}

		planes[0].Plane.Offset -= Dot(planes[0].Plane.Normal, target)
		planes[1].Plane.Offset -= Dot(planes[1].Plane.Normal, target)
		target = Vec3Zero

		result := SolvePlanes(target, planes[:2])

		ensure(result.IterationCount == 20)
	})

	It("ParallelPlanes", func() {
		var planes [3]CollisionPlane
		planes[0].Plane.Normal = Vec3{0.0, 0.0, 1.0}
		planes[0].Plane.Offset = 0.5
		planes[0].PushLimit = fltMax
		planes[1].Plane.Normal = Vec3{0.0, 0.0, 1.0}
		planes[1].Plane.Offset = 1.0
		planes[1].PushLimit = fltMax
		// planes[2].plane.normal = b3Normalize((b3Vec3){ 0.2f, 0.0f, 0.9f });
		// planes[2].plane.offset = 0.25f;
		// planes[2].pushLimit = FLT_MAX;

		target := Vec3{0.0, 0.0, 0.0}
		result := SolvePlanes(target, planes[:2])

		ensure(result.IterationCount == 2)
		ensureSmall(result.Delta.Z-1.0, 0.0055)
	})

	// Mover-collide overlap handling
	// b3CollideMoverAndSphere / Capsule / Hull must never emit a plane with a
	// degenerate (zero) normal, even when the mover deeply penetrates the shape.
	// On deep overlap the GJK path returns a {0,0,0} normal; these tests guard the
	// fix that replaces it with an analytic (sphere/capsule) or dropped (hull) result.
	It("MoverSphereSeparated", func() {
		shape := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
		mover := Capsule{Vec3{4.0, 3.0, 0.0}, Vec3{6.0, 3.0, 0.0}, 0.2}

		var result PlaneResult
		count := collideMoverAndSphere(&result, &shape, &mover)
		ensure(count == 0)
	})

	It("MoverSphereTouching", func() {
		shape := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}

		// Mover core segment runs along X at y = 0.6, leaving it 0.1 inside the
		// 0.7 combined radius.
		mover := Capsule{Vec3{-1.0, 0.6, 0.0}, Vec3{1.0, 0.6, 0.0}, 0.2}

		var result PlaneResult
		count := collideMoverAndSphere(&result, &shape, &mover)
		ensure(count == 1)
		ensure(IsNormalized(result.Plane.Normal))

		// Push-out points from the sphere straight up toward the mover.
		ensure(result.Plane.Normal.Y > 0.99)
		ensureSmall(result.Plane.Offset-0.1, 1e-5)
	})

	It("MoverSphereDeepOverlap", func() {
		shape := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}

		// Mover axis runs straight through the sphere center: the bug case where
		// GJK reports a zero normal.
		mover := Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 0.2}

		var result PlaneResult
		count := collideMoverAndSphere(&result, &shape, &mover)
		ensure(count == 1)

		// The normal must still be a valid unit vector.
		ensure(IsNormalized(result.Plane.Normal))

		// The fallback axis is perpendicular to the mover axis (X).
		ensureSmall(result.Plane.Normal.X, 1e-5)

		// Deepest possible penetration: the full combined radius.
		ensureSmall(result.Plane.Offset-0.7, 1e-5)
	})

	It("MoverCapsuleSeparated", func() {
		shape := Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 0.3}
		mover := Capsule{Vec3{-1.0, 5.0, 0.0}, Vec3{1.0, 5.0, 0.0}, 0.2}

		var result PlaneResult
		count := collideMoverAndCapsule(&result, &shape, &mover)
		ensure(count == 0)
	})

	It("MoverCapsuleTouching", func() {
		shape := Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 0.3}

		// Parallel mover 0.4 above, leaving it 0.1 inside the 0.5 combined radius.
		mover := Capsule{Vec3{-1.0, 0.4, 0.0}, Vec3{1.0, 0.4, 0.0}, 0.2}

		var result PlaneResult
		count := collideMoverAndCapsule(&result, &shape, &mover)
		ensure(count == 1)
		ensure(IsNormalized(result.Plane.Normal))
		ensure(result.Plane.Normal.Y > 0.99)
		ensureSmall(result.Plane.Offset-0.1, 1e-5)
	})

	It("MoverCapsuleDeepOverlap", func() {
		// Shape capsule along X, mover capsule along Z; their core segments cross
		// exactly at the origin, so GJK reports a zero normal.
		shape := Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 0.3}
		mover := Capsule{Vec3{0.0, 0.0, -1.0}, Vec3{0.0, 0.0, 1.0}, 0.2}

		var result PlaneResult
		count := collideMoverAndCapsule(&result, &shape, &mover)
		ensure(count == 1)
		ensure(IsNormalized(result.Plane.Normal))

		// The separating axis of two crossing segments is perpendicular to both.
		ensureSmall(result.Plane.Normal.X, 1e-5)
		ensureSmall(result.Plane.Normal.Z, 1e-5)
		ensureSmall(result.Plane.Offset-0.5, 1e-5)
	})

	It("MoverCapsuleParallelOverlap", func() {
		// Mover core segment coincides with the shape core segment: the cross-product
		// axis degenerates, so a perpendicular of the mover axis is used instead.
		shape := Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 0.3}
		mover := Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 0.2}

		var result PlaneResult
		count := collideMoverAndCapsule(&result, &shape, &mover)
		ensure(count == 1)
		ensure(IsNormalized(result.Plane.Normal))

		// The fallback axis is perpendicular to the mover axis (X).
		ensureSmall(result.Plane.Normal.X, 1e-5)
		ensureSmall(result.Plane.Offset-0.5, 1e-5)
	})

	It("MoverHullSeparated", func() {
		box := MakeBoxHull(0.5, 0.5, 0.5)
		mover := Capsule{Vec3{-0.3, 5.0, 0.0}, Vec3{0.3, 5.0, 0.0}, 0.2}

		var result PlaneResult
		count := collideMoverAndHull(&result, &box.Base, &mover)
		ensure(count == 0)
	})

	It("MoverHullTouching", func() {
		box := MakeBoxHull(0.5, 0.5, 0.5)

		// Mover core segment above the +Y face; the 0.2 radius reaches 0.1 into it.
		mover := Capsule{Vec3{-0.3, 0.6, 0.0}, Vec3{0.3, 0.6, 0.0}, 0.2}

		var result PlaneResult
		count := collideMoverAndHull(&result, &box.Base, &mover)
		ensure(count == 1)
		ensure(IsNormalized(result.Plane.Normal))
		ensure(result.Plane.Normal.Y > 0.99)
		ensureSmall(result.Plane.Offset-0.1, 1e-4)
	})

	It("MoverHullDeepOverlap", func() {
		box := MakeBoxHull(0.5, 0.5, 0.5)

		// Mover core segment lies entirely inside the box, so GJK reports overlap.
		mover := Capsule{Vec3{-0.2, 0.0, 0.0}, Vec3{0.2, 0.0, 0.0}, 0.1}

		var result PlaneResult
		count := collideMoverAndHull(&result, &box.Base, &mover)

		// The overlap guard drops the plane rather than emit a zero normal.
		// todo replace with SAT once b3CollideMoverAndHull resolves overlaps.
		ensure(count == 0)
	})

	// Mover queries report which material a contact plane came from
	// b3PlaneResult::materialIndex follows different paths per shape type. Meshes
	// report the per triangle material index. Compounds remap the child result
	// through the child material table. Convex shapes report index 0.
	PIt("MoverWorldMeshMaterials (needs body.c, shape.c, physics_world.c)")
	PIt("MoverWorldCompoundMeshMaterials (needs body.c, shape.c, physics_world.c)")
	PIt("MoverBodyMaterialIndices (needs body.c, shape.c, physics_world.c)")
	PIt("MoverBodySkipsMeshAndCompound (needs body.c, shape.c, physics_world.c)")

	// One sided mover collision.
	// Mover queries keep only triangles facing the mover. The front side follows the
	// baked winding: up for a default mesh or height field, down when the height
	// field carries clockwiseWinding. A mirror in the mesh scale must not flip it.
	It("MoverMeshBackside", func() {
		mesh := moverMakeTwoMaterialMesh()
		shape := Mesh{Data: mesh, Scale: Vec3{1.0, 1.0, 1.0}}

		var planes [4]PlaneResult

		// On the front of the left triangle, so a plane comes back
		above := Capsule{Vec3{-2.0, 0.15, 0.0}, Vec3{-2.0, 0.35, 0.0}, 0.2}
		count := collideMoverAndMesh(planes[:4], &shape, &above)
		ensure(count == 1)
		ensure(planes[0].Plane.Normal.Y > 0.99)
		ensure(planes[0].ChildIndex == 0)
		ensure(planes[0].TriangleIndex >= 0 && planes[0].TriangleIndex < mesh.TriangleCount)

		// The same spot seen from behind the face is culled
		below := Capsule{Vec3{-2.0, -0.35, 0.0}, Vec3{-2.0, -0.15, 0.0}, 0.2}
		count = collideMoverAndMesh(planes[:4], &shape, &below)
		ensure(count == 0)

		DestroyMesh(mesh)
	})

	It("MoverMeshMirroredScale", func() {
		mesh := moverMakeTwoMaterialMesh()

		// Reflecting the scale flips the triangle winding, the collision swaps it back,
		// so the front stays up. Local x = -2 maps onto the source right triangle, material 1.
		shape := Mesh{Data: mesh, Scale: Vec3{-1.0, 1.0, 1.0}}

		var planes [4]PlaneResult

		above := Capsule{Vec3{-2.0, 0.15, 0.0}, Vec3{-2.0, 0.35, 0.0}, 0.2}
		count := collideMoverAndMesh(planes[:4], &shape, &above)
		ensure(count == 1)
		ensure(planes[0].Plane.Normal.Y > 0.99)
		ensure(planes[0].MaterialIndex == 1)
		ensure(GetMeshMaterialIndices(mesh)[planes[0].TriangleIndex] == 1)

		below := Capsule{Vec3{-2.0, -0.35, 0.0}, Vec3{-2.0, -0.15, 0.0}, 0.2}
		count = collideMoverAndMesh(planes[:4], &shape, &below)
		ensure(count == 0)

		DestroyMesh(mesh)
	})
	It("MoverHeightFieldBackside", func() {
		materials := [4]uint8{0, 0, 0, 0}
		hf := makeFlatField(materials[:], false)

		var planes [4]PlaneResult

		// Standing on the front (upper) face of the default winding
		above := Capsule{Vec3{0.3, 0.15, 0.25}, Vec3{0.3, 0.35, 0.25}, 0.2}
		count := collideMoverAndHeightField(planes[:4], hf, &above)
		ensure(count == 1)
		ensure(planes[0].Plane.Normal.Y > 0.99)
		ensureSmall(planes[0].Plane.Offset-0.05, 1e-4)

		// Under the surface is the back side and gets culled
		below := Capsule{Vec3{0.3, -0.35, 0.25}, Vec3{0.3, -0.15, 0.25}, 0.2}
		count = collideMoverAndHeightField(planes[:4], hf, &below)
		ensure(count == 0)

		DestroyHeightField(hf)
	})

	It("MoverHeightFieldReport", func() {
		materials := [4]uint8{1, 2, 0, 0}
		hf := makeFlatField(materials[:], false)

		var planes [4]PlaneResult

		// (0.3, 0.25) sits on the x + z <= 1 side of cell (0,0), which holds triangle 0
		first := Capsule{Vec3{0.3, 0.15, 0.25}, Vec3{0.3, 0.35, 0.25}, 0.2}
		count := collideMoverAndHeightField(planes[:4], hf, &first)
		ensure(count == 1)
		ensure(planes[0].TriangleIndex == 0)
		ensure(planes[0].ChildIndex == 0)
		ensure(planes[0].MaterialIndex == 1)

		// (1.3, 0.3) sits on the x + z <= 2 side of cell (0,1), which holds triangle 2
		second := Capsule{Vec3{1.3, 0.15, 0.3}, Vec3{1.3, 0.35, 0.3}, 0.2}
		count = collideMoverAndHeightField(planes[:4], hf, &second)
		ensure(count == 1)
		ensure(planes[0].TriangleIndex == 2)
		ensure(planes[0].MaterialIndex == 2)

		DestroyHeightField(hf)
	})

	// A clockwise height field faces down, see HeightFieldWinding. Backside culling
	// must respect the flag: below the surface is the front side, above is the back.
	It("MoverHeightFieldClockwise", func() {
		materials := [4]uint8{0, 0, 0, 0}
		hf := makeFlatField(materials[:], true)

		var planes [4]PlaneResult

		below := Capsule{Vec3{0.3, -0.35, 0.25}, Vec3{0.3, -0.15, 0.25}, 0.2}
		count := collideMoverAndHeightField(planes[:4], hf, &below)
		ensure(count == 1)
		ensure(planes[0].Plane.Normal.Y < -0.99)
		ensureSmall(planes[0].Plane.Offset-0.05, 1e-4)
		ensure(planes[0].TriangleIndex == 0 || planes[0].TriangleIndex == 1)

		above := Capsule{Vec3{0.3, 0.15, 0.25}, Vec3{0.3, 0.35, 0.25}, 0.2}
		count = collideMoverAndHeightField(planes[:4], hf, &above)
		ensure(count == 0)

		DestroyHeightField(hf)
	})

	// A mesh can be baked with more materials than the shape carries. The reported index must stay
	// inside the shape's material array.
	PIt("MoverWorldMeshMaterialClamp (needs body.c, shape.c, physics_world.c)")
})
