// Port of test/test_compound.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"bytes"
	"math"
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
)

// compoundMakeMaterial is MakeMaterial (static in test_compound.c).
func compoundMakeMaterial(friction float32, userId uint64) SurfaceMaterial {
	m := DefaultSurfaceMaterial()
	m.Friction = friction
	m.UserMaterialId = userId
	return m
}

// compoundMakeTwoMaterialMesh is MakeTwoMaterialMesh (static in test_compound.c).
// Two separated upward facing triangles on the y = 0 plane, one per material. The bake may
// reorder triangles but always keeps a triangle paired with its material index.
func compoundMakeTwoMaterialMesh() *MeshData {
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

// makeSixMaterialMesh is MakeSixMaterialMesh.
// Six separated upward facing triangles, one per material index 0 to 5
func makeSixMaterialMesh() *MeshData {
	var vertices [18]Vec3
	var indices [18]int32
	var materialIndices [6]uint8

	for i := range 6 {
		x := 3.0 * float32(i)
		vertices[3*i+0] = Vec3{x - 1.0, 0.0, -1.0}
		vertices[3*i+1] = Vec3{x, 0.0, 1.0}
		vertices[3*i+2] = Vec3{x + 1.0, 0.0, -1.0}

		indices[3*i+0] = int32(3*i + 0)
		indices[3*i+1] = int32(3*i + 1)
		indices[3*i+2] = int32(3*i + 2)

		materialIndices[i] = uint8(i)
	}

	def := MeshDef{}
	def.Vertices = vertices[:]
	def.Stride = int(unsafe.Sizeof(Vec3{}))
	def.Indices = indices[:]
	def.MaterialIndices = materialIndices[:]
	def.VertexCount = 18
	def.TriangleCount = 6

	return CreateMesh(&def, nil)
}

// queryAccumulator is struct QueryAccumulator.
type queryAccumulator struct {
	childIndices [8]int
	count        int
	stopAfter    int // -1 = never stop early
}

// compoundQueryCallback is QueryCallback.
func compoundQueryCallback(compound *CompoundData, childIndex int, context any) bool {
	_ = compound
	acc := context.(*queryAccumulator)
	if acc.count < len(acc.childIndices) {
		acc.childIndices[acc.count] = childIndex
		acc.count++
	}
	if acc.stopAfter >= 0 && acc.count >= acc.stopAfter {
		return false
	}
	return true
}

// buildSerializableCompound is BuildSerializableCompound.
func buildSerializableCompound(md *MeshData) *CompoundData {
	mat := DefaultSurfaceMaterial()
	box := MakeBoxHull(0.5, 0.5, 0.5)

	capsule := CompoundCapsuleDef{Capsule: Capsule{Vec3{-2, 0, 0}, Vec3{-1, 0, 0}, 0.2}, Material: mat}
	hull := CompoundHullDef{Hull: &box.Base, Transform: Transform{P: Vec3{5, 0, 0}, Q: QuatIdentity}, Material: mat}
	mesh := CompoundMeshDef{
		MeshData:      md,
		Transform:     Transform{P: Vec3{0, 0, 5}, Q: QuatIdentity},
		Scale:         Vec3{1, 1, 1},
		Materials:     []SurfaceMaterial{mat},
		MaterialCount: 1,
	}
	sph := CompoundSphereDef{Sphere: Sphere{Vec3{-5, 0, 0}, 0.5}, Material: mat}

	def := CompoundDef{
		Capsules:     []CompoundCapsuleDef{capsule},
		CapsuleCount: 1,
		Hulls:        []CompoundHullDef{hull},
		HullCount:    1,
		Meshes:       []CompoundMeshDef{mesh},
		MeshCount:    1,
		Spheres:      []CompoundSphereDef{sph},
		SphereCount:  1,
	}
	return CreateCompound(&def)
}

// cloneCompoundBytes is the malloc + memcpy of the serialize tests.
func cloneCompoundBytes(compound *CompoundData) []byte {
	buffer := make([]byte, compound.ByteCount)
	copy(buffer, compoundBytes(compound))
	return buffer
}

var _ = Describe("CompoundTest", func() {
	It("CompoundCreateMixed", func() {
		mat := DefaultSurfaceMaterial()

		capsules := []CompoundCapsuleDef{
			{Capsule: Capsule{Vec3{-1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 0.25}, Material: mat},
		}

		box := MakeBoxHull(0.5, 0.5, 0.5)
		hulls := []CompoundHullDef{
			{Hull: &box.Base, Transform: TransformIdentity, Material: mat},
		}

		meshData := CreateBoxMesh(Vec3Zero, Vec3{0.5, 0.5, 0.5}, false)
		meshes := []CompoundMeshDef{
			{
				MeshData:      meshData,
				Transform:     TransformIdentity,
				Scale:         Vec3{1.0, 1.0, 1.0},
				Materials:     []SurfaceMaterial{mat},
				MaterialCount: 1,
			},
		}

		spheres := []CompoundSphereDef{
			{Sphere: Sphere{Vec3{5.0, 0.0, 0.0}, 0.5}, Material: mat},
			{Sphere: Sphere{Vec3{-5.0, 0.0, 0.0}, 0.5}, Material: mat},
		}

		def := CompoundDef{
			Capsules:     capsules,
			CapsuleCount: 1,
			Hulls:        hulls,
			HullCount:    1,
			Meshes:       meshes,
			MeshCount:    1,
			Spheres:      spheres,
			SphereCount:  2,
		}

		compound := CreateCompound(&def)
		ensure(compound != nil)
		ensure(compound.Version == CompoundVersion)
		ensure(compound.ByteCount > int32(unsafe.Sizeof(CompoundData{})))

		ensure(compound.CapsuleCount == 1)
		ensure(compound.HullCount == 1)
		ensure(compound.MeshCount == 1)
		ensure(compound.SphereCount == 2)

		ensure(compound.MaterialCount == 1)
		ensure(compound.SharedHullCount == 1)
		ensure(compound.SharedMeshCount == 1)

		ensure(compound.NodeCount >= 2)
		ensure(compound.NodeOffset > 0)
		ensure(compound.Bounds.LowerBound.X < compound.Bounds.UpperBound.X)

		DestroyCompound(compound)
		DestroyMesh(meshData)
	})

	It("CompoundCreateSingleType", func() {
		mat := DefaultSurfaceMaterial()

		// Capsule only
		{
			capsule := CompoundCapsuleDef{Capsule: Capsule{Vec3{0, 0, 0}, Vec3{1, 0, 0}, 0.5}, Material: mat}
			def := CompoundDef{Capsules: []CompoundCapsuleDef{capsule}, CapsuleCount: 1}
			c := CreateCompound(&def)
			ensure(c != nil && c.CapsuleCount == 1 && c.HullCount == 0 && c.MeshCount == 0 && c.SphereCount == 0)
			DestroyCompound(c)
		}

		// Hull only
		{
			box := MakeBoxHull(1, 1, 1)
			h := CompoundHullDef{Hull: &box.Base, Transform: TransformIdentity, Material: mat}
			def := CompoundDef{Hulls: []CompoundHullDef{h}, HullCount: 1}
			c := CreateCompound(&def)
			ensure(c != nil && c.HullCount == 1 && c.SharedHullCount == 1 && c.CapsuleCount == 0)
			DestroyCompound(c)
		}

		// Mesh only
		{
			md := CreateBoxMesh(Vec3Zero, Vec3{1, 1, 1}, false)
			m := CompoundMeshDef{
				MeshData:      md,
				Transform:     TransformIdentity,
				Scale:         Vec3{1, 1, 1},
				Materials:     []SurfaceMaterial{mat},
				MaterialCount: 1,
			}
			def := CompoundDef{Meshes: []CompoundMeshDef{m}, MeshCount: 1}
			c := CreateCompound(&def)
			ensure(c != nil && c.MeshCount == 1 && c.SharedMeshCount == 1)
			DestroyCompound(c)
			DestroyMesh(md)
		}

		// Sphere only
		{
			s := CompoundSphereDef{Sphere: Sphere{Vec3{0, 0, 0}, 1.0}, Material: mat}
			def := CompoundDef{Spheres: []CompoundSphereDef{s}, SphereCount: 1}
			c := CreateCompound(&def)
			ensure(c != nil && c.SphereCount == 1)
			DestroyCompound(c)
		}
	})

	It("CompoundMaterialDedup", func() {
		mat := compoundMakeMaterial(0.4, 7)

		var caps [3]CompoundCapsuleDef
		for i := range 3 {
			caps[i].Capsule = Capsule{Vec3{float32(i), 0, 0}, Vec3{float32(i) + 1, 0, 0}, 0.25}
			caps[i].Material = mat
		}

		def := CompoundDef{Capsules: caps[:], CapsuleCount: 3}
		c := CreateCompound(&def)
		ensure(c.MaterialCount == 1)
		for i := range 3 {
			cc := GetCompoundCapsule(c, i)
			ensure(cc.MaterialIndex == 0)
		}
		DestroyCompound(c)
	})

	It("CompoundMaterialDistinct", func() {
		var caps [3]CompoundCapsuleDef
		for i := range 3 {
			caps[i].Capsule = Capsule{Vec3{float32(i), 0, 0}, Vec3{float32(i) + 1, 0, 0}, 0.25}
			caps[i].Material = compoundMakeMaterial(0.1*float32(i+1), uint64(i+1))
		}

		def := CompoundDef{Capsules: caps[:], CapsuleCount: 3}
		c := CreateCompound(&def)
		ensure(c.MaterialCount == 3)

		mats := GetCompoundMaterials(c)
		ensure(mats != nil)

		for i := range 3 {
			cc := GetCompoundCapsule(c, i)
			ensure(cc.MaterialIndex < 3)
			ensure(mats[cc.MaterialIndex].UserMaterialId == uint64(i+1))
		}

		DestroyCompound(c)
	})

	It("CompoundMaterialCrossShape", func() {
		// One material shared across capsule, hull, and sphere -> 1 material slot.
		mat := compoundMakeMaterial(0.5, 99)

		capsule := CompoundCapsuleDef{Capsule: Capsule{Vec3{0, 0, 0}, Vec3{1, 0, 0}, 0.25}, Material: mat}
		box := MakeBoxHull(0.5, 0.5, 0.5)
		hull := CompoundHullDef{Hull: &box.Base, Transform: TransformIdentity, Material: mat}
		sph := CompoundSphereDef{Sphere: Sphere{Vec3{5, 0, 0}, 0.5}, Material: mat}

		def := CompoundDef{
			Capsules:     []CompoundCapsuleDef{capsule},
			CapsuleCount: 1,
			Hulls:        []CompoundHullDef{hull},
			HullCount:    1,
			Spheres:      []CompoundSphereDef{sph},
			SphereCount:  1,
		}
		c := CreateCompound(&def)
		ensure(c.MaterialCount == 1)

		ensure(GetCompoundCapsule(c, 0).MaterialIndex == 0)
		ensure(GetCompoundHull(c, 0).MaterialIndex == 0)
		ensure(GetCompoundSphere(c, 0).MaterialIndex == 0)

		DestroyCompound(c)
	})

	It("CompoundMaterialMeshShared", func() {
		// Mesh material entries are routed through the same material map as convex
		// materials, so an identical material is deduped across mesh and convex.
		// (The comment in compound.c about meshes "not being shared" refers to the
		// per-instance materialIndices arrays, not the b3SurfaceMaterial table.)
		mat := compoundMakeMaterial(0.3, 11)

		md := CreateBoxMesh(Vec3Zero, Vec3{1, 1, 1}, false)
		ensure(md.MaterialCount == 1)

		sph := CompoundSphereDef{Sphere: Sphere{Vec3{5, 0, 0}, 0.5}, Material: mat}
		mesh := CompoundMeshDef{
			MeshData:      md,
			Transform:     TransformIdentity,
			Scale:         Vec3{1, 1, 1},
			Materials:     []SurfaceMaterial{mat},
			MaterialCount: 1,
		}

		def := CompoundDef{
			Meshes:      []CompoundMeshDef{mesh},
			MeshCount:   1,
			Spheres:     []CompoundSphereDef{sph},
			SphereCount: 1,
		}
		c := CreateCompound(&def)
		ensure(c.MaterialCount == 1)

		DestroyCompound(c)
		DestroyMesh(md)
	})

	// ---------------------------------------------------------------------------
	// Hull / mesh blob sharing
	// ---------------------------------------------------------------------------

	It("CompoundHullSharingPointer", func() {
		mat := DefaultSurfaceMaterial()
		box := MakeBoxHull(1, 1, 1)

		var hulls [3]CompoundHullDef
		for i := range 3 {
			hulls[i].Hull = &box.Base
			hulls[i].Transform = TransformIdentity
			hulls[i].Transform.P.X = float32(4 * i)
			hulls[i].Material = mat
		}

		def := CompoundDef{Hulls: hulls[:], HullCount: 3}
		c := CreateCompound(&def)
		ensure(c.HullCount == 3)
		ensure(c.SharedHullCount == 1)
		DestroyCompound(c)
	})

	It("CompoundHullSharingContent", func() {
		mat := DefaultSurfaceMaterial()

		// Two box hulls built independently with identical args are byte-identical
		// (b3MakeBoxHull is deterministic and the hash is computed over the bytes).
		boxA := MakeBoxHull(1, 1, 1)
		boxB := MakeBoxHull(1, 1, 1)
		ensure(&boxA != &boxB)

		hulls := [2]CompoundHullDef{
			{Hull: &boxA.Base, Transform: TransformIdentity, Material: mat},
			{Hull: &boxB.Base, Transform: TransformIdentity, Material: mat},
		}
		hulls[1].Transform.P.X = 5.0

		def := CompoundDef{Hulls: hulls[:], HullCount: 2}
		c := CreateCompound(&def)
		ensure(c.SharedHullCount == 1)
		DestroyCompound(c)
	})

	It("CompoundHullDistinct", func() {
		mat := DefaultSurfaceMaterial()
		boxA := MakeBoxHull(1, 1, 1)
		boxB := MakeBoxHull(2, 1, 1)

		hulls := [2]CompoundHullDef{
			{Hull: &boxA.Base, Transform: TransformIdentity, Material: mat},
			{Hull: &boxB.Base, Transform: TransformIdentity, Material: mat},
		}
		hulls[1].Transform.P.X = 5.0

		def := CompoundDef{Hulls: hulls[:], HullCount: 2}
		c := CreateCompound(&def)
		ensure(c.SharedHullCount == 2)
		DestroyCompound(c)
	})

	It("CompoundMeshSharingPointer", func() {
		mat := DefaultSurfaceMaterial()
		md := CreateBoxMesh(Vec3Zero, Vec3{1, 1, 1}, false)

		var meshes [3]CompoundMeshDef
		for i := range 3 {
			meshes[i].MeshData = md
			meshes[i].Transform = TransformIdentity
			meshes[i].Transform.P.X = float32(4 * i)
			meshes[i].Scale = Vec3{1, 1, 1}
			meshes[i].Materials = []SurfaceMaterial{mat}
			meshes[i].MaterialCount = 1
		}

		def := CompoundDef{Meshes: meshes[:], MeshCount: 3}
		c := CreateCompound(&def)
		ensure(c.MeshCount == 3)
		ensure(c.SharedMeshCount == 1)

		DestroyCompound(c)
		DestroyMesh(md)
	})

	It("CompoundMeshSharingContent", func() {
		mat := DefaultSurfaceMaterial()
		mdA := CreateBoxMesh(Vec3Zero, Vec3{1, 1, 1}, false)
		mdB := CreateBoxMesh(Vec3Zero, Vec3{1, 1, 1}, false)
		ensure(mdA != mdB)

		meshes := [2]CompoundMeshDef{
			{MeshData: mdA, Transform: TransformIdentity, Scale: Vec3{1, 1, 1}, Materials: []SurfaceMaterial{mat}, MaterialCount: 1},
			{MeshData: mdB, Transform: TransformIdentity, Scale: Vec3{1, 1, 1}, Materials: []SurfaceMaterial{mat}, MaterialCount: 1},
		}
		meshes[1].Transform.P.X = 5.0

		def := CompoundDef{Meshes: meshes[:], MeshCount: 2}
		c := CreateCompound(&def)
		ensure(c.SharedMeshCount == 1)

		DestroyCompound(c)
		DestroyMesh(mdA)
		DestroyMesh(mdB)
	})

	It("CompoundMeshDistinct", func() {
		mat := DefaultSurfaceMaterial()
		mdA := CreateBoxMesh(Vec3Zero, Vec3{1, 1, 1}, false)
		mdB := CreateBoxMesh(Vec3Zero, Vec3{2, 1, 1}, false)

		meshes := [2]CompoundMeshDef{
			{MeshData: mdA, Transform: TransformIdentity, Scale: Vec3{1, 1, 1}, Materials: []SurfaceMaterial{mat}, MaterialCount: 1},
			{MeshData: mdB, Transform: TransformIdentity, Scale: Vec3{1, 1, 1}, Materials: []SurfaceMaterial{mat}, MaterialCount: 1},
		}
		meshes[1].Transform.P.X = 5.0

		def := CompoundDef{Meshes: meshes[:], MeshCount: 2}
		c := CreateCompound(&def)
		ensure(c.SharedMeshCount == 2)

		DestroyCompound(c)
		DestroyMesh(mdA)
		DestroyMesh(mdB)
	})

	It("CompoundChildDispatch", func() {
		mat := DefaultSurfaceMaterial()
		box := MakeBoxHull(0.5, 0.5, 0.5)
		md := CreateBoxMesh(Vec3Zero, Vec3{0.5, 0.5, 0.5}, false)

		caps := []CompoundCapsuleDef{
			{Capsule: Capsule{Vec3{0, 0, 0}, Vec3{1, 0, 0}, 0.2}, Material: mat},
			{Capsule: Capsule{Vec3{0, 2, 0}, Vec3{1, 2, 0}, 0.2}, Material: mat},
		}
		hulls := []CompoundHullDef{{Hull: &box.Base, Transform: TransformIdentity, Material: mat}}
		hulls[0].Transform.P = Vec3{5, 0, 0}
		meshes := []CompoundMeshDef{{
			MeshData:      md,
			Transform:     TransformIdentity,
			Scale:         Vec3{1, 1, 1},
			Materials:     []SurfaceMaterial{mat},
			MaterialCount: 1,
		}}
		meshes[0].Transform.P = Vec3{0, 0, 5}
		spheres := []CompoundSphereDef{{Sphere: Sphere{Vec3{-5, 0, 0}, 0.5}, Material: mat}}

		def := CompoundDef{
			Capsules:     caps,
			CapsuleCount: 2,
			Hulls:        hulls,
			HullCount:    1,
			Meshes:       meshes,
			MeshCount:    1,
			Spheres:      spheres,
			SphereCount:  1,
		}
		c := CreateCompound(&def)

		// Index ordering is capsules -> hulls -> meshes -> spheres.
		ensure(GetCompoundChild(c, 0).Type == CapsuleShape)
		ensure(GetCompoundChild(c, 1).Type == CapsuleShape)
		ensure(GetCompoundChild(c, 2).Type == HullShape)
		ensure(GetCompoundChild(c, 3).Type == MeshShape)
		ensure(GetCompoundChild(c, 4).Type == SphereShape)

		// Capsule and sphere children always report identity transform. The position
		// is encoded in the shape itself (capsule->center{1,2}, sphere->center).
		cap0 := GetCompoundChild(c, 0)
		ensureSmall(cap0.Transform.P.X, fltEpsilon)
		ensureSmall(cap0.Transform.P.Y, fltEpsilon)
		ensureSmall(cap0.Transform.P.Z, fltEpsilon)
		ensureSmall(cap0.Capsule.Center2.X-1.0, fltEpsilon)

		sph := GetCompoundChild(c, 4)
		ensureSmall(sph.Transform.P.X, fltEpsilon)
		ensureSmall(sph.Sphere.Center.X+5.0, fltEpsilon)

		// Hull and mesh children carry their stored transform.
		hull := GetCompoundChild(c, 2)
		ensureSmall(hull.Transform.P.X-5.0, fltEpsilon)

		mesh := GetCompoundChild(c, 3)
		ensureSmall(mesh.Transform.P.Z-5.0, fltEpsilon)
		ensure(mesh.Mesh.Data != nil)

		DestroyCompound(c)
		DestroyMesh(md)
	})

	It("CompoundAABBContainsChildren", func() {
		mat := DefaultSurfaceMaterial()
		spheres := []CompoundSphereDef{
			{Sphere: Sphere{Vec3{-3, 0, 0}, 1.0}, Material: mat},
			{Sphere: Sphere{Vec3{4, 0, 0}, 0.5}, Material: mat},
		}
		def := CompoundDef{Spheres: spheres, SphereCount: 2}
		c := CreateCompound(&def)

		local := ComputeCompoundAABB(c, TransformIdentity)
		ensure(local.LowerBound.X <= -4.0+1e-5)
		ensure(local.UpperBound.X >= 4.5-1e-5)
		ensure(local.LowerBound.Y <= -1.0+1e-5)
		ensure(local.UpperBound.Y >= 1.0-1e-5)

		// Translation commutes through the bounding-box transform.
		xf := Transform{P: Vec3{10, 20, 30}, Q: QuatIdentity}
		world := ComputeCompoundAABB(c, xf)
		ensureSmall(world.LowerBound.X-(local.LowerBound.X+10.0), 1e-4)
		ensureSmall(world.UpperBound.Y-(local.UpperBound.Y+20.0), 1e-4)
		ensureSmall(world.LowerBound.Z-(local.LowerBound.Z+30.0), 1e-4)

		DestroyCompound(c)
	})

	It("CompoundRayCastMiss", func() {
		mat := DefaultSurfaceMaterial()
		sph := CompoundSphereDef{Sphere: Sphere{Vec3{0, 0, 0}, 0.5}, Material: mat}
		def := CompoundDef{Spheres: []CompoundSphereDef{sph}, SphereCount: 1}
		c := CreateCompound(&def)

		// Ray well above the sphere on a parallel path.
		input := RayCastInput{Origin: Vec3{-5, 5, 0}, Translation: Vec3{10, 0, 0}, MaxFraction: 1.0}
		out := RayCastCompound(c, &input)
		ensure(out.Hit == false)

		DestroyCompound(c)
	})

	It("CompoundRayCastClosest", func() {
		matA := compoundMakeMaterial(0.4, 100)
		matB := compoundMakeMaterial(0.4, 200)

		// Two unit spheres along +X. Ray from origin must hit the nearer one first.
		spheres := []CompoundSphereDef{
			{Sphere: Sphere{Vec3{5, 0, 0}, 1.0}, Material: matA},
			{Sphere: Sphere{Vec3{10, 0, 0}, 1.0}, Material: matB},
		}
		def := CompoundDef{Spheres: spheres, SphereCount: 2}
		c := CreateCompound(&def)

		input := RayCastInput{Origin: Vec3{0, 0, 0}, Translation: Vec3{20, 0, 0}, MaxFraction: 1.0}
		out := RayCastCompound(c, &input)
		ensure(out.Hit == true)

		// Front face of the nearer sphere is at x=4 -> fraction 4/20 = 0.2.
		ensureSmall(out.Fraction-0.2, 1e-4)
		ensureSmall(out.Normal.X+1.0, 1e-4)
		ensure(out.ChildIndex == 0)

		mats := GetCompoundMaterials(c)
		ensure(mats[out.MaterialIndex].UserMaterialId == 100)

		DestroyCompound(c)
	})

	It("CompoundRayCastHullNormalRotation", func() {
		// A unit box rotated 90 degrees about Z, placed at compound +X. The ray hits the
		// face that, in compound space, points back toward -X. Verifies that the
		// normal returned by the cast has been rotated from hull-local space back
		// into compound space.
		mat := DefaultSurfaceMaterial()
		box := MakeBoxHull(1, 1, 1)

		hull := CompoundHullDef{
			Hull:      &box.Base,
			Transform: Transform{P: Vec3{5, 0, 0}, Q: MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)},
			Material:  mat,
		}
		def := CompoundDef{Hulls: []CompoundHullDef{hull}, HullCount: 1}
		c := CreateCompound(&def)

		input := RayCastInput{Origin: Vec3{0, 0, 0}, Translation: Vec3{20, 0, 0}, MaxFraction: 1.0}
		out := RayCastCompound(c, &input)
		ensure(out.Hit == true)
		// Box has |hx|=1; with the rotation a face still intersects the +X ray at x=4.
		ensureSmall(out.Fraction-0.2, 1e-4)
		ensureSmall(out.Normal.X+1.0, 1e-3)
		ensureSmall(out.Normal.Y, 1e-3)
		ensureSmall(out.Normal.Z, 1e-3)

		DestroyCompound(c)
	})

	It("CompoundShapeCastClosest", func() {
		mat := DefaultSurfaceMaterial()
		spheres := []CompoundSphereDef{
			{Sphere: Sphere{Vec3{5, 0, 0}, 1.0}, Material: mat},
			{Sphere: Sphere{Vec3{10, 0, 0}, 1.0}, Material: mat},
		}
		def := CompoundDef{Spheres: spheres, SphereCount: 2}
		c := CreateCompound(&def)

		point := Vec3{0, 0, 0}
		input := ShapeCastInput{
			Proxy:       pointProxy(&point, 0.25),
			Translation: Vec3{20, 0, 0},
			MaxFraction: 1.0,
			CanEncroach: false,
		}
		out := ShapeCastCompound(c, &input)
		ensure(out.Hit == true)
		// Closest contact: caster radius 0.25 + sphere radius 1.0 -> first contact at x ~= 3.75.
		ensureSmall(out.Fraction-3.75/20.0, 1e-3)
		ensure(out.ChildIndex == 0)

		DestroyCompound(c)
	})

	It("CompoundShapeCastMiss", func() {
		mat := DefaultSurfaceMaterial()
		sph := CompoundSphereDef{Sphere: Sphere{Vec3{5, 0, 0}, 1.0}, Material: mat}
		def := CompoundDef{Spheres: []CompoundSphereDef{sph}, SphereCount: 1}
		c := CreateCompound(&def)

		point := Vec3{0, 5, 0}
		input := ShapeCastInput{
			Proxy:       pointProxy(&point, 0.25),
			Translation: Vec3{20, 0, 0},
			MaxFraction: 1.0,
			CanEncroach: false,
		}
		ensure(ShapeCastCompound(c, &input).Hit == false)

		// An empty proxy has nothing to sweep
		input.Proxy.Count = 0
		ensure(ShapeCastCompound(c, &input).Hit == false)

		DestroyCompound(c)
	})

	It("CompoundShapeCastHullNormalRotation", func() {
		// Same setup as the ray cast rotation test. The result comes back in compound space, so the
		// hull local face the sweep lands on says nothing about the reported normal and point.
		mat := DefaultSurfaceMaterial()
		box := MakeBoxHull(1, 1, 1)

		hull := CompoundHullDef{
			Hull:      &box.Base,
			Transform: Transform{P: Vec3{5, 0, 0}, Q: MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)},
			Material:  mat,
		}
		def := CompoundDef{Hulls: []CompoundHullDef{hull}, HullCount: 1}
		c := CreateCompound(&def)

		point := Vec3{0, 0, 0}
		input := ShapeCastInput{
			Proxy:       pointProxy(&point, 0.25),
			Translation: Vec3{20, 0, 0},
			MaxFraction: 1.0,
			CanEncroach: false,
		}
		out := ShapeCastCompound(c, &input)
		ensure(out.Hit == true)

		// Near face at x=4, caster radius 0.25 so contact at x=3.75
		ensureSmall(out.Fraction-3.75/20.0, 1e-3)
		ensureSmall(out.Normal.X+1.0, 1e-3)
		ensureSmall(out.Normal.Y, 1e-3)
		ensureSmall(out.Normal.Z, 1e-3)
		ensureSmall(out.Point.X-4.0, 1e-3)
		ensureSmall(out.Point.Y, 1e-3)
		ensureSmall(out.Point.Z, 1e-3)
		ensure(out.ChildIndex == 0)

		DestroyCompound(c)
	})

	It("CompoundMeshMaterialRemap", func() {
		// A mesh child carries its own small material table. Casts report the triangle material
		// after it has been remapped into the compound material array.
		matA := compoundMakeMaterial(0.3, 100)
		matB := compoundMakeMaterial(0.7, 200)
		materials := []SurfaceMaterial{matA, matB}

		md := compoundMakeTwoMaterialMesh()
		mesh := CompoundMeshDef{
			MeshData:      md,
			Transform:     TransformIdentity,
			Scale:         Vec3{1, 1, 1},
			Materials:     materials,
			MaterialCount: 2,
		}
		def := CompoundDef{Meshes: []CompoundMeshDef{mesh}, MeshCount: 1}
		c := CreateCompound(&def)

		table := GetCompoundMaterials(c)

		// Straight down onto the material 0 triangle
		rayA := RayCastInput{Origin: Vec3{-2, 5, 0}, Translation: Vec3{0, -10, 0}, MaxFraction: 1.0}
		outA := RayCastCompound(c, &rayA)
		ensure(outA.Hit == true)
		ensure(table[outA.MaterialIndex].UserMaterialId == 100)

		rayB := RayCastInput{Origin: Vec3{2, 5, 0}, Translation: Vec3{0, -10, 0}, MaxFraction: 1.0}
		outB := RayCastCompound(c, &rayB)
		ensure(outB.Hit == true)
		ensure(table[outB.MaterialIndex].UserMaterialId == 200)

		// The shape cast path does the same remap
		point := Vec3{2, 5, 0}
		castInput := ShapeCastInput{
			Proxy:       pointProxy(&point, 0.1),
			Translation: Vec3{0, -10, 0},
			MaxFraction: 1.0,
			CanEncroach: false,
		}
		outCast := ShapeCastCompound(c, &castInput)
		ensure(outCast.Hit == true)
		ensure(table[outCast.MaterialIndex].UserMaterialId == 200)

		DestroyCompound(c)
		DestroyMesh(md)
	})

	It("CompoundMeshManyMaterials", func() {
		// The capsule material bakes first so the mesh map is not the identity
		capsuleMat := compoundMakeMaterial(0.4, 1000)

		md := makeSixMaterialMesh()
		ensure(md.MaterialCount == 6)

		var meshMats [6]SurfaceMaterial
		for i := range 6 {
			meshMats[i] = compoundMakeMaterial(mulAdd(0.2, 0.1, float32(i)), uint64(i+1))
		}

		capsule := CompoundCapsuleDef{Capsule: Capsule{Vec3{-10.0, 0.0, 0.0}, Vec3{-9.0, 0.0, 0.0}, 0.25}, Material: capsuleMat}
		mesh := CompoundMeshDef{
			MeshData:      md,
			Transform:     TransformIdentity,
			Scale:         Vec3{1.0, 1.0, 1.0},
			Materials:     meshMats[:],
			MaterialCount: 6,
		}

		def := CompoundDef{
			Capsules:     []CompoundCapsuleDef{capsule},
			CapsuleCount: 1,
			Meshes:       []CompoundMeshDef{mesh},
			MeshCount:    1,
		}
		c := CreateCompound(&def)
		ensure(c != nil)
		ensure(c.MaterialCount == 7)

		table := GetCompoundMaterials(c)

		capChild := GetCompoundChild(c, 0)
		ensure(capChild.Type == CapsuleShape)
		ensure(capChild.MaterialCount == 1)
		ensure(table[capChild.MaterialIndices[0]].UserMaterialId == 1000)

		meshChild := GetCompoundChild(c, 1)
		ensure(meshChild.Type == MeshShape)
		ensure(meshChild.MaterialCount == 6)
		for k := range 6 {
			ensure(table[meshChild.MaterialIndices[k]].UserMaterialId == uint64(k+1))
		}

		// Sixth triangle, material index 5
		triangleMaterials := GetMeshMaterialIndices(md)
		ray := RayCastInput{Origin: Vec3{15.0, 5.0, 0.0}, Translation: Vec3{0.0, -10.0, 0.0}, MaxFraction: 1.0}
		out := RayCastCompound(c, &ray)
		ensure(out.Hit == true)
		ensure(out.ChildIndex == 1)
		rawIndex := int(triangleMaterials[out.TriangleIndex])
		ensure(rawIndex >= 4)
		ensure(table[out.MaterialIndex].UserMaterialId == uint64(rawIndex+1))

		DestroyCompound(c)
		DestroyMesh(md)
	})

	PIt("CompoundMeshContactMaterial (needs body.c, shape.c, contact.c, mesh_contact.c, physics_world.c)")

	It("CompoundOverlap", func() {
		mat := DefaultSurfaceMaterial()
		spheres := []CompoundSphereDef{
			{Sphere: Sphere{Vec3{-3, 0, 0}, 0.5}, Material: mat},
			{Sphere: Sphere{Vec3{3, 0, 0}, 0.5}, Material: mat},
		}
		def := CompoundDef{Spheres: spheres, SphereCount: 2}
		c := CreateCompound(&def)

		// Proxy at the origin lies in the gap between the two spheres.
		origin := Vec3{0, 0, 0}
		gap := pointProxy(&origin, 0.25)
		ensure(OverlapCompound(c, TransformIdentity, &gap) == false)

		// Proxy at the center of the second sphere overlaps it.
		onSecond := Vec3{3, 0, 0}
		hit := pointProxy(&onSecond, 0.1)
		ensure(OverlapCompound(c, TransformIdentity, &hit) == true)

		DestroyCompound(c)
	})

	It("CompoundOverlapTransformed", func() {
		// The tree and the child geometry live in the compound frame while the proxy arrives in world
		// space, so a compound that is moved and turned has to pull the proxy back before querying.
		mat := DefaultSurfaceMaterial()
		spheres := []CompoundSphereDef{
			{Sphere: Sphere{Vec3{-3, 0, 0}, 0.5}, Material: mat},
			{Sphere: Sphere{Vec3{3, 0, 0}, 0.5}, Material: mat},
		}
		def := CompoundDef{Spheres: spheres, SphereCount: 2}
		c := CreateCompound(&def)

		transform := Transform{P: Vec3{10, 20, 30}, Q: MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)}

		gapPoint := TransformPoint(transform, Vec3{0, 0, 0})
		gap := pointProxy(&gapPoint, 0.25)
		ensure(OverlapCompound(c, transform, &gap) == false)

		hitPoint := TransformPoint(transform, Vec3{3, 0, 0})
		hit := pointProxy(&hitPoint, 0.1)
		ensure(OverlapCompound(c, transform, &hit) == true)

		// A proxy left in the compound frame must not register against the moved compound
		localPoint := Vec3{3, 0, 0}
		stale := pointProxy(&localPoint, 0.1)
		ensure(OverlapCompound(c, transform, &stale) == false)

		DestroyCompound(c)
	})

	It("CompoundOverlapChildTypes", func() {
		// One child of every type, spaced far enough apart that a hit names the type it came from.
		// Hull and mesh children add a child transform underneath the compound transform.
		mat := DefaultSurfaceMaterial()
		box := MakeBoxHull(0.5, 0.5, 0.5)
		md := CreateBoxMesh(Vec3Zero, Vec3{0.5, 0.5, 0.5}, false)

		capsule := CompoundCapsuleDef{Capsule: Capsule{Vec3{-10, 0, 0}, Vec3{-9, 0, 0}, 0.25}, Material: mat}
		hull := CompoundHullDef{
			Hull:      &box.Base,
			Transform: Transform{P: Vec3{0, 0, 0}, Q: MakeQuatFromAxisAngle(Vec3AxisZ, 0.25*Pi)},
			Material:  mat,
		}
		mesh := CompoundMeshDef{
			MeshData:      md,
			Transform:     Transform{P: Vec3{10, 0, 0}, Q: MakeQuatFromAxisAngle(Vec3AxisY, 0.5*Pi)},
			Scale:         Vec3{1, 1, 1},
			Materials:     []SurfaceMaterial{mat},
			MaterialCount: 1,
		}
		sphere := CompoundSphereDef{Sphere: Sphere{Vec3{20, 0, 0}, 0.5}, Material: mat}

		def := CompoundDef{
			Capsules:     []CompoundCapsuleDef{capsule},
			CapsuleCount: 1,
			Hulls:        []CompoundHullDef{hull},
			HullCount:    1,
			Meshes:       []CompoundMeshDef{mesh},
			MeshCount:    1,
			Spheres:      []CompoundSphereDef{sphere},
			SphereCount:  1,
		}
		c := CreateCompound(&def)

		transform := Transform{P: Vec3{100, 200, 300}, Q: MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)}

		// Points sitting on or inside each child, then carried out to world space
		localPoints := [4]Vec3{
			{-9.5, 0, 0},  // capsule axis
			{0, 0, 0},     // hull center
			{10, 0.45, 0}, // just under a mesh face
			{20, 0, 0},    // sphere center
		}

		for i := range 4 {
			worldPoint := TransformPoint(transform, localPoints[i])
			proxy := pointProxy(&worldPoint, 0.1)
			ensure(OverlapCompound(c, transform, &proxy) == true)
		}

		// A gap between the hull and the mesh
		gapPoint := TransformPoint(transform, Vec3{5, 0, 0})
		gap := pointProxy(&gapPoint, 0.1)
		ensure(OverlapCompound(c, transform, &gap) == false)

		DestroyCompound(c)
		DestroyMesh(md)
	})

	It("CompoundOverlapSegmentProxy", func() {
		// A multi point proxy has to be carried into the compound frame point by point
		mat := DefaultSurfaceMaterial()
		spheres := []CompoundSphereDef{
			{Sphere: Sphere{Vec3{-3, 0, 0}, 0.5}, Material: mat},
			{Sphere: Sphere{Vec3{3, 0, 0}, 0.5}, Material: mat},
		}
		def := CompoundDef{Spheres: spheres, SphereCount: 2}
		c := CreateCompound(&def)

		transform := Transform{P: Vec3{-40, 15, 7}, Q: MakeQuatFromAxisAngle(Vec3AxisY, 0.5*Pi)}

		// Segment that stays inside the gap
		inGap := []Vec3{
			TransformPoint(transform, Vec3{-1, 0, 0}),
			TransformPoint(transform, Vec3{1, 0, 0}),
		}
		gap := ShapeProxy{Points: inGap, Count: 2, Radius: 0.25}
		ensure(OverlapCompound(c, transform, &gap) == false)

		// Same segment stretched until one end reaches the second sphere
		reaching := []Vec3{
			TransformPoint(transform, Vec3{-1, 0, 0}),
			TransformPoint(transform, Vec3{2.9, 0, 0}),
		}
		hit := ShapeProxy{Points: reaching, Count: 2, Radius: 0.25}
		ensure(OverlapCompound(c, transform, &hit) == true)

		DestroyCompound(c)
	})

	It("CompoundQuery", func() {
		mat := DefaultSurfaceMaterial()
		spheres := []CompoundSphereDef{
			{Sphere: Sphere{Vec3{-10, 0, 0}, 0.5}, Material: mat},
			{Sphere: Sphere{Vec3{0, 0, 0}, 0.5}, Material: mat},
			{Sphere: Sphere{Vec3{10, 0, 0}, 0.5}, Material: mat},
		}
		def := CompoundDef{Spheres: spheres, SphereCount: 3}
		c := CreateCompound(&def)

		// Tight box around the middle sphere. Only it should be reported.
		middle := AABB{Vec3{-1, -1, -1}, Vec3{1, 1, 1}}
		acc := queryAccumulator{stopAfter: -1}
		QueryCompound(c, middle, compoundQueryCallback, &acc)
		ensure(acc.count == 1)
		ensure(acc.childIndices[0] == 1)

		// Wide box overlapping all three; early-exit after the first reported child.
		wide := AABB{Vec3{-20, -1, -1}, Vec3{20, 1, 1}}
		stop := queryAccumulator{stopAfter: 1}
		QueryCompound(c, wide, compoundQueryCallback, &stop)
		ensure(stop.count == 1)

		// Without early-exit, all three are visited.
		all := queryAccumulator{stopAfter: -1}
		QueryCompound(c, wide, compoundQueryCallback, &all)
		ensure(all.count == 3)

		DestroyCompound(c)
	})

	It("CompoundMover", func() {
		mat := DefaultSurfaceMaterial()
		box := MakeBoxHull(0.5, 0.5, 0.5)

		// Two boxes side-by-side along X, gap of 1 between them.
		hulls := []CompoundHullDef{
			{Hull: &box.Base, Transform: Transform{P: Vec3{-1.0, 0, 0}, Q: QuatIdentity}, Material: mat},
			{Hull: &box.Base, Transform: Transform{P: Vec3{1.0, 0, 0}, Q: QuatIdentity}, Material: mat},
		}
		def := CompoundDef{Hulls: hulls, HullCount: 2}
		c := CreateCompound(&def)

		// Small capsule mover sitting on top of the boxes, low enough to penetrate both.
		mover := Capsule{Vec3{-1.0, 0.6, 0}, Vec3{1.0, 0.6, 0}, 0.2}

		var planes [8]PlaneResult
		planeCount := collideMoverAndCompound(planes[:8], c, &mover)

		// Both boxes contribute at least one plane each; the +Y face of each box
		// should produce a plane whose normal points roughly +Y in compound space.
		ensure(planeCount >= 2)

		upPlanes := 0
		for i := range planeCount {
			if planes[i].Plane.Normal.Y > 0.9 {
				upPlanes += 1
			}
		}
		ensure(upPlanes >= 2)

		// Capacity cap is honored: ask for 1 and the call must not exceed it.
		var one [1]PlaneResult
		capped := collideMoverAndCompound(one[:1], c, &mover)
		ensure(capped <= 1)

		DestroyCompound(c)
	})

	It("CompoundMoverRotatedChild", func() {
		// The child solves the mover in its own frame, so the plane it returns has to be carried
		// back into the compound frame before the caller sees it.
		mat := compoundMakeMaterial(0.25, 77)
		box := MakeBoxHull(0.5, 0.5, 0.5)

		// Quarter turn about z, so the compound space top face is a side face of the hull
		hull := CompoundHullDef{
			Hull:      &box.Base,
			Transform: Transform{P: Vec3{0, 0, 0}, Q: MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)},
			Material:  mat,
		}
		def := CompoundDef{Hulls: []CompoundHullDef{hull}, HullCount: 1}
		c := CreateCompound(&def)

		mover := Capsule{Vec3{-0.1, 0.6, 0}, Vec3{0.1, 0.6, 0}, 0.2}

		var planes [8]PlaneResult
		planeCount := collideMoverAndCompound(planes[:8], c, &mover)
		ensure(planeCount >= 1)

		ensureSmall(planes[0].Plane.Normal.X, 1e-3)
		ensureSmall(planes[0].Plane.Normal.Y-1.0, 1e-3)
		ensureSmall(planes[0].Plane.Normal.Z, 1e-3)
		ensureSmall(planes[0].Point.Y-0.5, 1e-3)

		ensure(planes[0].ChildIndex == 0)
		table := GetCompoundMaterials(c)
		ensure(table[planes[0].MaterialIndex].UserMaterialId == 77)

		DestroyCompound(c)
	})

	It("CompoundSerializeRoundtrip", func() {
		md := CreateBoxMesh(Vec3Zero, Vec3{0.5, 0.5, 0.5}, false)
		a := buildSerializableCompound(md)

		// Snapshot a few queries on the original.
		aabbA := ComputeCompoundAABB(a, TransformIdentity)
		rayInput := RayCastInput{Origin: Vec3{0, 0, 0}, Translation: Vec3{20, 0, 0}, MaxFraction: 1.0}
		rayA := RayCastCompound(a, &rayInput)
		ensure(rayA.Hit)

		byteCount := int(a.ByteCount)
		buffer := make([]byte, byteCount)
		ensure(buffer != nil)

		reference := make([]byte, byteCount)
		ensure(reference != nil)

		copy(buffer, compoundBytes(a))
		copy(reference, compoundBytes(a))
		DestroyCompound(a)

		b := ValidateCompound(buffer, byteCount)
		ensure(b != nil)
		ensure(unsafe.Pointer(b) == unsafe.Pointer(&buffer[0]))
		ensure(int(b.ByteCount) == byteCount)
		ensure(b.CapsuleCount == 1)
		ensure(b.HullCount == 1)
		ensure(b.MeshCount == 1)
		ensure(b.SphereCount == 1)

		// Validation must not touch the bytes
		ensure(bytes.Equal(buffer, reference))

		// Bounds and ray cast on the deserialized compound match the original.
		aabbB := ComputeCompoundAABB(b, TransformIdentity)
		ensureSmall(aabbA.LowerBound.X-aabbB.LowerBound.X, 1e-5)
		ensureSmall(aabbA.UpperBound.Z-aabbB.UpperBound.Z, 1e-5)

		rayB := RayCastCompound(b, &rayInput)
		ensure(rayB.Hit == rayA.Hit)
		ensureSmall(rayB.Fraction-rayA.Fraction, 1e-5)
		ensure(rayB.ChildIndex == rayA.ChildIndex)

		DestroyMesh(md)
	})

	It("CompoundSerializeBadVersion", func() {
		md := CreateBoxMesh(Vec3Zero, Vec3{0.5, 0.5, 0.5}, false)
		a := buildSerializableCompound(md)

		byteCount := int(a.ByteCount)
		buffer := cloneCompoundBytes(a)
		DestroyCompound(a)

		// Corrupt the version word (first 8 bytes of b3Compound).
		(*CompoundData)(unsafe.Pointer(&buffer[0])).Version ^= 0x1

		b := ValidateCompound(buffer, byteCount)
		ensure(b == nil)

		DestroyMesh(md)
	})

	It("CompoundSerializeWrongByteCount", func() {
		md := CreateBoxMesh(Vec3Zero, Vec3{0.5, 0.5, 0.5}, false)
		a := buildSerializableCompound(md)

		byteCount := int(a.ByteCount)
		buffer := cloneCompoundBytes(a)
		DestroyCompound(a)

		// Off-by-one in the declared length is rejected.
		b := ValidateCompound(buffer, byteCount+1)
		ensure(b == nil)

		DestroyMesh(md)
	})

	It("CompoundSerializeBadNodeCount", func() {
		md := CreateBoxMesh(Vec3Zero, Vec3{0.5, 0.5, 0.5}, false)
		a := buildSerializableCompound(md)

		byteCount := int(a.ByteCount)
		buffer := cloneCompoundBytes(a)
		DestroyCompound(a)

		header := (*CompoundData)(unsafe.Pointer(&buffer[0]))
		nodeCount := header.NodeCount

		// Node array runs off the end of the buffer
		header.NodeCount = int32(byteCount)
		ensure(ValidateCompound(buffer, byteCount) == nil)

		// Overflow in 32 bit arithmetic must still be caught
		header.NodeCount = math.MaxInt32
		ensure(ValidateCompound(buffer, byteCount) == nil)

		header.NodeCount = nodeCount
		ensure(ValidateCompound(buffer, byteCount) != nil)

		DestroyMesh(md)
	})
})
