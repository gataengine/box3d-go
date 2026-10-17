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

// PORT: the static helpers of test_mover.c are prefixed with mover (or shared, see makeFlatField)
// so they cannot clash in the one test package.

// moverMakeMaterial is MakeMaterial from test_mover.c.
// PORT: renamed; test_compound.c has a static function of the same name.
func moverMakeMaterial(friction float32, userId uint64) SurfaceMaterial {
	m := DefaultSurfaceMaterial()
	m.Friction = friction
	m.UserMaterialId = userId
	return m
}

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

// moverPlaneCapture is PlaneCapture.
type moverPlaneCapture struct {
	planes [16]PlaneResult
	count  int
}

// moverCapturePlaneFcn is CapturePlaneFcn.
func moverCapturePlaneFcn(shapeId ShapeID, planes []PlaneResult, context any) bool {
	_ = shapeId
	capture := context.(*moverPlaneCapture)
	planeCount := len(planes)
	for i := 0; i < planeCount && capture.count < 16; i++ {
		capture.planes[capture.count] = planes[i]
		capture.count++
	}
	return true
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
	It("MoverWorldMeshMaterials", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyId := worldId.CreateBody(&bodyDef)

		mesh := moverMakeTwoMaterialMesh()
		ensure(mesh != nil)
		ensure(mesh.MaterialCount == 2)

		materials := [2]SurfaceMaterial{moverMakeMaterial(0.2, 1), moverMakeMaterial(0.8, 2)}
		shapeDef := DefaultShapeDef()
		shapeDef.Materials = materials[:]
		shapeDef.MaterialCount = 2
		bodyId.CreateMeshShape(&shapeDef, mesh, Vec3{1.0, 1.0, 1.0})

		worldId.Step(1.0/60.0, 1)

		bakedMaterialIndices := GetMeshMaterialIndices(mesh)

		// Mover hanging just above the first triangle so its radius reaches 0.05 into the surface
		mover := Capsule{Vec3{-2.0, 0.15, 0.0}, Vec3{-2.0, 0.35, 0.0}, 0.2}

		capture := moverPlaneCapture{}
		worldId.CollideMover(PosZero, &mover, DefaultQueryFilter(), moverCapturePlaneFcn, &capture)

		ensure(capture.count == 1)
		ensure(capture.planes[0].Plane.Normal.Y > 0.99)
		ensureSmall(capture.planes[0].Plane.Offset-0.05, 1e-4)
		ensureSmall(capture.planes[0].Point.Y, 1e-4)
		ensure(capture.planes[0].MaterialIndex == 0)
		ensure(capture.planes[0].TriangleIndex >= 0)
		ensure(capture.planes[0].TriangleIndex < mesh.TriangleCount)
		ensure(bakedMaterialIndices[capture.planes[0].TriangleIndex] == 0)

		// Same mover over the second triangle
		mover2 := Capsule{Vec3{2.0, 0.15, 0.0}, Vec3{2.0, 0.35, 0.0}, 0.2}

		capture2 := moverPlaneCapture{}
		worldId.CollideMover(PosZero, &mover2, DefaultQueryFilter(), moverCapturePlaneFcn, &capture2)

		ensure(capture2.count == 1)
		ensure(capture2.planes[0].Plane.Normal.Y > 0.99)
		ensure(capture2.planes[0].MaterialIndex == 1)
		ensure(bakedMaterialIndices[capture2.planes[0].TriangleIndex] == 1)

		worldId.Destroy()
		DestroyMesh(mesh)
	})

	It("MoverWorldCompoundMeshMaterials", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyId := worldId.CreateBody(&bodyDef)

		mesh := moverMakeTwoMaterialMesh()

		// The hull material and the two mesh materials are distinct, so the compound material
		// table gets one slot per material. Children bake in hull then mesh order.
		meshMaterials := [2]SurfaceMaterial{moverMakeMaterial(0.3, 101), moverMakeMaterial(0.6, 202)}
		hullMaterial := moverMakeMaterial(0.9, 303)

		meshChild := CompoundMeshDef{
			MeshData:      mesh,
			Transform:     TransformIdentity,
			Scale:         Vec3{1.0, 1.0, 1.0},
			Materials:     meshMaterials[:],
			MaterialCount: 2,
		}

		box := MakeBoxHull(0.5, 0.5, 0.5)
		hullChild := CompoundHullDef{
			Hull:      &box.Base,
			Transform: Transform{P: Vec3{-8.0, 0.0, 0.0}, Q: QuatIdentity},
			Material:  hullMaterial,
		}

		compoundDef := CompoundDef{
			Hulls:     []CompoundHullDef{hullChild},
			HullCount: 1,
			Meshes:    []CompoundMeshDef{meshChild},
			MeshCount: 1,
		}
		compound := CreateCompound(&compoundDef)
		ensure(compound != nil)
		ensure(compound.MaterialCount == 3)

		bakedMaterials := GetCompoundMaterials(compound)
		ensure(bakedMaterials[0].UserMaterialId == 303)
		ensure(bakedMaterials[1].UserMaterialId == 101)
		ensure(bakedMaterials[2].UserMaterialId == 202)

		shapeDef := DefaultShapeDef()
		bodyId.CreateBakedCompoundShape(&shapeDef, compound)

		worldId.Step(1.0/60.0, 1)

		// Mover on top of the hull child face at y = 0.5
		hullMover := Capsule{Vec3{-8.0, 0.65, 0.0}, Vec3{-8.0, 0.85, 0.0}, 0.2}

		hullCapture := moverPlaneCapture{}
		worldId.CollideMover(PosZero, &hullMover, DefaultQueryFilter(), moverCapturePlaneFcn, &hullCapture)

		ensure(hullCapture.count == 1)
		ensure(hullCapture.planes[0].Plane.Normal.Y > 0.99)
		ensure(hullCapture.planes[0].ChildIndex == 0)
		ensure(hullCapture.planes[0].MaterialIndex == 0)
		ensure(bakedMaterials[hullCapture.planes[0].MaterialIndex].UserMaterialId == 303)

		// Mover over the second mesh triangle. The mesh reports triangle material 1, which the
		// compound remaps through the child material table to the shared slot of meshMaterials[1].
		meshMover := Capsule{Vec3{2.0, 0.15, 0.0}, Vec3{2.0, 0.35, 0.0}, 0.2}

		meshCapture := moverPlaneCapture{}
		worldId.CollideMover(PosZero, &meshMover, DefaultQueryFilter(), moverCapturePlaneFcn, &meshCapture)

		ensure(meshCapture.count == 1)
		ensure(meshCapture.planes[0].Plane.Normal.Y > 0.99)
		ensure(meshCapture.planes[0].ChildIndex == 1)
		ensure(meshCapture.planes[0].MaterialIndex == 2)
		ensure(bakedMaterials[meshCapture.planes[0].MaterialIndex].UserMaterialId == 202)

		worldId.Destroy()
		DestroyCompound(compound)
		DestroyMesh(mesh)
	})

	It("MoverBodyMaterialIndices", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyId := worldId.CreateBody(&bodyDef)

		// One shape of each convex type, spaced out along X
		shapeDef := DefaultShapeDef()

		sphere := Sphere{Vec3{0.0, 0.5, 0.0}, 0.5}
		bodyId.CreateSphereShape(&shapeDef, &sphere)

		box := MakeTransformedBoxHull(0.5, 0.5, 0.5, Transform{P: Vec3{5.0, 0.0, 0.0}, Q: QuatIdentity})
		bodyId.CreateHullShape(&shapeDef, &box.Base)

		capsule := Capsule{Vec3{9.0, 0.0, 0.0}, Vec3{11.0, 0.0, 0.0}, 0.3}
		bodyId.CreateCapsuleShape(&shapeDef, &capsule)

		bodyTransform := WorldTransform{P: PosZero, Q: QuatIdentity}

		// Convex shapes report the base material as index 0
		movers := [3]Capsule{
			{Vec3{0.0, 1.15, 0.0}, Vec3{0.0, 1.35, 0.0}, 0.2},
			{Vec3{5.0, 0.65, 0.0}, Vec3{5.0, 0.85, 0.0}, 0.2},
			{Vec3{10.0, 0.45, 0.0}, Vec3{10.0, 0.65, 0.0}, 0.2},
		}

		for i := 0; i < 3; i++ {
			var planes [4]BodyPlaneResult
			count := bodyId.CollideMover(planes[:4], PosZero, &movers[i], DefaultQueryFilter(), bodyTransform)

			ensure(count == 1)
			ensure(planes[0].ShapeId.IsValid())
			ensure(planes[0].Result.Plane.Normal.Y > 0.99)
			ensure(planes[0].Result.MaterialIndex == 0)
			ensure(planes[0].Result.ChildIndex == 0)
		}

		worldId.Destroy()
	})

	It("MoverBodySkipsMeshAndCompound", func() {
		// The body level mover query handles convex shapes only. Mesh and compound shapes are
		// skipped, so their material indices are only available through b3World_CollideMover.
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyId := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()

		mesh := moverMakeTwoMaterialMesh()
		ensure(mesh != nil)
		bodyId.CreateMeshShape(&shapeDef, mesh, Vec3{1.0, 1.0, 1.0})

		box := MakeBoxHull(0.5, 0.5, 0.5)
		hullChild := CompoundHullDef{
			Hull:      &box.Base,
			Transform: Transform{P: Vec3{10.0, 0.0, 0.0}, Q: QuatIdentity},
		}
		compoundDef := CompoundDef{
			Hulls:     []CompoundHullDef{hullChild},
			HullCount: 1,
		}
		compound := CreateCompound(&compoundDef)
		ensure(compound != nil)
		bodyId.CreateBakedCompoundShape(&shapeDef, compound)

		bodyTransform := WorldTransform{P: PosZero, Q: QuatIdentity}

		// Mover over the mesh triangle and mover over the compound hull child both find nothing
		meshMover := Capsule{Vec3{-2.0, 0.15, 0.0}, Vec3{-2.0, 0.35, 0.0}, 0.2}
		compoundMover := Capsule{Vec3{10.0, 0.65, 0.0}, Vec3{10.0, 0.85, 0.0}, 0.2}

		movers := [2]Capsule{meshMover, compoundMover}
		for i := 0; i < 2; i++ {
			var planes [4]BodyPlaneResult
			count := bodyId.CollideMover(planes[:4], PosZero, &movers[i], DefaultQueryFilter(), bodyTransform)
			ensure(count == 0)
		}

		worldId.Destroy()
		DestroyCompound(compound)
		DestroyMesh(mesh)
	})

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
	It("MoverWorldMeshMaterialClamp", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = StaticBody
		bodyId := worldId.CreateBody(&bodyDef)

		mesh := moverMakeTwoMaterialMesh()
		ensure(mesh.MaterialCount == 2)

		// Base material only
		shapeDef := DefaultShapeDef()
		shapeId := bodyId.CreateMeshShape(&shapeDef, mesh, Vec3{1.0, 1.0, 1.0})
		shapeMaterialCount := shapeId.GetMeshMaterialCount()
		ensure(shapeMaterialCount == 1)

		worldId.Step(1.0/60.0, 1)

		// Over the triangle baked with material 1
		mover := Capsule{Vec3{2.0, 0.15, 0.0}, Vec3{2.0, 0.35, 0.0}, 0.2}
		capture := moverPlaneCapture{}
		worldId.CollideMover(PosZero, &mover, DefaultQueryFilter(), moverCapturePlaneFcn, &capture)

		ensure(capture.count == 1)
		ensure(capture.planes[0].MaterialIndex == int32(shapeMaterialCount-1))

		worldId.Destroy()
		DestroyMesh(mesh)
	})
})
