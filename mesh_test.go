// Port of test/test_mesh.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
)

// Two quads meeting at a concave crease along the shared edge 1-4. The crease exercises
// edge identification, which reads the baked winding rather than the input winding.
const (
	valleyVertexCount   = 6 // VALLEY_VERTEX_COUNT
	valleyTriangleCount = 4 // VALLEY_TRIANGLE_COUNT
)

func makeValley(vertices []Vec3, indices []int32) {
	v := [valleyVertexCount]Vec3{
		{-1.0, 1.0, -1.0}, {0.0, 0.0, -1.0}, {1.0, 1.0, -1.0},
		{-1.0, 1.0, 1.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 1.0},
	}

	i := [3 * valleyTriangleCount]int32{
		0, 3, 1, // left quad
		3, 4, 1,
		1, 4, 2, // right quad
		4, 5, 2,
	}

	copy(vertices, v[:])
	copy(indices, i[:])
}

// fatVertex is FatVertex: a vertex laid out the way a renderer would hand it over. The position
// sits off the front of the struct so a non-zero base offset gets exercised too.
type fatVertex struct {
	weight   float32
	position Vec3
	uv       [2]float32
}

func makeFatVertices(fat []fatVertex, vertices []Vec3, count int) {
	for i := range count {
		// Poison the padding so a stride mistake bakes obvious garbage instead of near misses
		fat[i].weight = 1000.0 + float32(i)
		fat[i].position = vertices[i]
		fat[i].uv[0] = -1000.0
		fat[i].uv[1] = -2000.0
	}
}

// fatPositions is &fat[0].position as a strided vertex base for MeshDef.Vertices.
// PORT: the slice spans every byte of the strided positions without running past the array.
func fatPositions(fat []fatVertex) []Vec3 {
	span := len(fat)*int(unsafe.Sizeof(fatVertex{})) - int(unsafe.Offsetof(fatVertex{}.position))
	return unsafe.Slice(&fat[0].position, span/int(unsafe.Sizeof(Vec3{})))
}

func reverseWinding(indices []int32, triangleCount int) {
	for i := range triangleCount {
		temp := indices[3*i+1]
		indices[3*i+1] = indices[3*i+2]
		indices[3*i+2] = temp
	}
}

// The hash covers every byte of the mesh block, so this compares the tree, the vertices,
// the baked triangles and the edge flags in one shot.
func meshesMatch(mesh1, mesh2 *MeshData) bool {
	return mesh1.ByteCount == mesh2.ByteCount && mesh1.Hash == mesh2.Hash
}

func verticesMatch(mesh *MeshData, expected []Vec3, count int) bool {
	if int(mesh.VertexCount) != count {
		return false
	}

	vertices := GetMeshVertices(mesh)
	for i := range count {
		if DistanceSquared(vertices[i], expected[i]) > fltEpsilon {
			return false
		}
	}

	return true
}

// The valley faces up, so every baked triangle should wind counter clockwise about +Y
func facesUp(mesh *MeshData) bool {
	vertices := GetMeshVertices(mesh)
	triangles := GetMeshTriangles(mesh)

	for i := range int(mesh.TriangleCount) {
		v1 := vertices[triangles[i].Index1]
		v2 := vertices[triangles[i].Index2]
		v3 := vertices[triangles[i].Index3]

		normal := Cross(Sub(v2, v1), Sub(v3, v1))
		if normal.Y <= 0.0 {
			return false
		}
	}

	return true
}

func hasConcaveEdge(mesh *MeshData) bool {
	flags := GetMeshFlags(mesh)

	for i := range int(mesh.TriangleCount) {
		if (MeshEdgeFlags(flags[i]) & AllConcaveEdges) != 0 {
			return true
		}
	}

	return false
}

func makeValleyDef(vertices []Vec3, indices []int32) MeshDef {
	def := MeshDef{}
	def.Vertices = vertices
	def.Stride = int(unsafe.Sizeof(Vec3{}))
	def.Indices = indices
	def.VertexCount = valleyVertexCount
	def.TriangleCount = valleyTriangleCount
	def.IdentifyEdges = true
	return def
}

var _ = Describe("MeshTest", func() {
	// Dense input is the reference the other cases are measured against
	It("MeshDenseStride", func() {
		var vertices [valleyVertexCount]Vec3
		var indices [3 * valleyTriangleCount]int32
		makeValley(vertices[:], indices[:])

		def := makeValleyDef(vertices[:], indices[:])
		mesh := CreateMesh(&def, nil)
		ensure(mesh != nil)

		ensure(mesh.VertexCount == valleyVertexCount)
		ensure(mesh.TriangleCount == valleyTriangleCount)
		ensure(mesh.DegenerateCount == 0)
		ensure(verticesMatch(mesh, vertices[:], valleyVertexCount))
		ensure(facesUp(mesh))
		ensure(hasConcaveEdge(mesh))

		DestroyMesh(mesh)
	})

	// Interleaved input must bake to exactly the same mesh as dense input
	It("MeshFatStride", func() {
		var vertices [valleyVertexCount]Vec3
		var indices [3 * valleyTriangleCount]int32
		makeValley(vertices[:], indices[:])

		denseDef := makeValleyDef(vertices[:], indices[:])
		denseMesh := CreateMesh(&denseDef, nil)
		ensure(denseMesh != nil)

		var fat [valleyVertexCount]fatVertex
		makeFatVertices(fat[:], vertices[:], valleyVertexCount)

		fatDef := makeValleyDef(fatPositions(fat[:]), indices[:])
		fatDef.Stride = int(unsafe.Sizeof(fatVertex{}))
		fatMesh := CreateMesh(&fatDef, nil)
		ensure(fatMesh != nil)

		ensure(verticesMatch(fatMesh, vertices[:], valleyVertexCount))
		ensure(facesUp(fatMesh))
		ensure(meshesMatch(denseMesh, fatMesh))

		DestroyMesh(fatMesh)
		DestroyMesh(denseMesh)
	})

	// Welding reads the source vertices through the stride as well
	It("MeshStrideWeld", func() {
		var vertices [valleyVertexCount]Vec3
		var indices [3 * valleyTriangleCount]int32
		makeValley(vertices[:], indices[:])

		// Split the crease so each quad owns a copy of the shared edge
		var splitVertices [8]Vec3
		copy(splitVertices[:], vertices[:])
		splitVertices[6] = vertices[1]
		splitVertices[7] = vertices[4]

		// Point the right quad at the copies
		var splitIndices [3 * valleyTriangleCount]int32
		copy(splitIndices[:], indices[:])
		splitIndices[6] = 6
		splitIndices[7] = 7
		splitIndices[9] = 7

		var fat [8]fatVertex
		makeFatVertices(fat[:], splitVertices[:], 8)

		def := makeValleyDef(fatPositions(fat[:]), splitIndices[:])
		def.Stride = int(unsafe.Sizeof(fatVertex{}))
		def.VertexCount = 8
		def.WeldVertices = true
		def.WeldTolerance = 0.01

		mesh := CreateMesh(&def, nil)
		ensure(mesh != nil)

		ensure(mesh.VertexCount == valleyVertexCount)
		ensure(mesh.TriangleCount == valleyTriangleCount)
		ensure(verticesMatch(mesh, vertices[:], valleyVertexCount))
		ensure(facesUp(mesh))
		ensure(hasConcaveEdge(mesh))

		DestroyMesh(mesh)
	})

	// Clockwise input plus the flag must bake to the same mesh as counter clockwise input
	It("MeshClockWise", func() {
		var vertices [valleyVertexCount]Vec3
		var indices [3 * valleyTriangleCount]int32
		makeValley(vertices[:], indices[:])

		ccwDef := makeValleyDef(vertices[:], indices[:])
		ccwMesh := CreateMesh(&ccwDef, nil)
		ensure(ccwMesh != nil)

		var reversed [3 * valleyTriangleCount]int32
		copy(reversed[:], indices[:])
		reverseWinding(reversed[:], valleyTriangleCount)

		cwDef := makeValleyDef(vertices[:], reversed[:])
		cwDef.ClockWiseWinding = true
		cwMesh := CreateMesh(&cwDef, nil)
		ensure(cwMesh != nil)

		ensure(facesUp(cwMesh))
		ensure(hasConcaveEdge(cwMesh))
		ensure(meshesMatch(ccwMesh, cwMesh))

		DestroyMesh(cwMesh)
		DestroyMesh(ccwMesh)
	})

	// Without the flag the same clockwise input must bake inside out
	It("MeshClockWiseIgnored", func() {
		var vertices [valleyVertexCount]Vec3
		var indices [3 * valleyTriangleCount]int32
		makeValley(vertices[:], indices[:])
		reverseWinding(indices[:], valleyTriangleCount)

		def := makeValleyDef(vertices[:], indices[:])
		mesh := CreateMesh(&def, nil)
		ensure(mesh != nil)

		ensure(facesUp(mesh) == false)

		DestroyMesh(mesh)
	})

	// Winding, stride and welding have to compose
	It("MeshClockWiseStrideWeld", func() {
		var vertices [valleyVertexCount]Vec3
		var indices [3 * valleyTriangleCount]int32
		makeValley(vertices[:], indices[:])

		ccwDef := makeValleyDef(vertices[:], indices[:])
		ccwDef.WeldVertices = true
		ccwDef.WeldTolerance = 0.01
		ccwMesh := CreateMesh(&ccwDef, nil)
		ensure(ccwMesh != nil)

		var reversed [3 * valleyTriangleCount]int32
		copy(reversed[:], indices[:])
		reverseWinding(reversed[:], valleyTriangleCount)

		var fat [valleyVertexCount]fatVertex
		makeFatVertices(fat[:], vertices[:], valleyVertexCount)

		cwDef := makeValleyDef(fatPositions(fat[:]), reversed[:])
		cwDef.Stride = int(unsafe.Sizeof(fatVertex{}))
		cwDef.ClockWiseWinding = true
		cwDef.WeldVertices = true
		cwDef.WeldTolerance = 0.01
		cwMesh := CreateMesh(&cwDef, nil)
		ensure(cwMesh != nil)

		ensure(facesUp(cwMesh))
		ensure(meshesMatch(ccwMesh, cwMesh))

		DestroyMesh(cwMesh)
		DestroyMesh(ccwMesh)
	})

	// The built in creators fill their own def, so a missed stride shows up here
	It("MeshCreators", func() {
		center := Vec3{1.0, 2.0, 3.0}
		extent := Vec3{0.5, 1.0, 1.5}

		meshes := []*MeshData{
			CreateGridMesh(4, 4, 1.0, 2, true),
			CreateWaveMesh(4, 4, 1.0, 0.5, 1.0, 1.0),
			CreateTorusMesh(8, 6, 2.0, 0.5),
			CreateBoxMesh(center, extent, true),
			CreateHollowBoxMesh(center, extent),
			CreatePlatformMesh(center, 2.0, 1.0, 2.0),
		}

		for i := range arrayCount(meshes) {
			ensure(meshes[i] != nil)
			ensure(meshes[i].VertexCount >= 3)
			ensure(meshes[i].TriangleCount >= 1)
			ensure(IsSaneAABB(meshes[i].Bounds))

			DestroyMesh(meshes[i])
		}
	})
})
