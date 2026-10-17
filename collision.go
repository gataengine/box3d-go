// Port of include/box3d/collision.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "unsafe"

// PORT: collision.h declares functions implemented in dynamic_tree.c, hull.c, mesh.c,
// height_field.c, compound.c, sphere.c, capsule.c, distance.c, manifold.c, mover.c and others;
// each is ported with its .c file.

// ---- Geometry blobs (Go helpers, no upstream counterpart) ----

// PORT: geometry blobs (b3HullData, b3MeshData, b3HeightFieldData, b3CompoundData) keep
// upstream's byte image: a header struct followed by arrays addressed by byte offsets from the
// header address, 8-byte aligned and lane padded exactly as in C. A blob is allocated as one
// []uint64 (8-byte aligned, pointer free so the GC never scans it) and the *HullData (etc.)
// points at its first word. The header structs have C layout (checked by size asserts), so
// offsets, byte counts, hashes and memcmp style comparisons match upstream byte for byte. A
// value type that embeds the header followed by its arrays (b3BoxHull) works the same way:
// the offsets reach past the header into the enclosing struct.
//
// The accessors return slices into the image. Interior pointers keep the whole allocation
// alive. The headers and arrays must stay pointer free.

// blobAlloc allocates a zeroed, 8-byte aligned blob of byteCount bytes. This is
// b3Alloc + memset( 0 ) for geometry blobs.
func blobAlloc(byteCount int) unsafe.Pointer {
	words := make([]uint64, (byteCount+7)/8)
	return unsafe.Pointer(unsafe.SliceData(words))
}

// blobSlice returns count elements of T at byte offset from base, or nil for a zero offset
// (the C accessors return NULL).
func blobSlice[T any](base unsafe.Pointer, offset int32, count int) []T {
	if offset == 0 {
		return nil
	}
	if count == 0 {
		// An empty array may sit at the very end of the blob; a pointer one past the
		// allocation is invalid in Go (checkptr), so hand out a non-nil empty slice instead.
		return []T{}
	}
	return unsafe.Slice((*T)(unsafe.Add(base, offset)), count)
}

// blobBytes views the first byteCount bytes of a blob. This is the (uint8_t*) cast upstream
// uses for hashing, cloning and comparing blobs.
func blobBytes(base unsafe.Pointer, byteCount int32) []byte {
	return unsafe.Slice((*byte)(base), byteCount)
}

// Dynamic tree

// GetUserData is b3DynamicTree_GetUserData: get proxy user data.
func (tree *DynamicTree) GetUserData(proxyId int) uint64 {
	return tree.Proxies[proxyId].UserData
}

// GetAABB is b3DynamicTree_GetAABB: get the AABB of a proxy.
func (tree *DynamicTree) GetAABB(proxyId int) AABB {
	return tree.Nodes[tree.Proxies[proxyId].Node].AABB
}

// Hull

// hullBytes is the byte image of a hull: (uint8_t*)hull with hull->byteCount bytes.
func hullBytes(hull *HullData) []byte {
	return blobBytes(unsafe.Pointer(hull), hull.ByteCount)
}

// GetHullVertices gets read only hull vertices.
// PORT: returns a slice of VertexCount elements into the hull image (nil when the offset is 0).
// The slices are writable in Go; treat them as read only as upstream's const does.
func GetHullVertices(hull *HullData) []HullVertex {
	return blobSlice[HullVertex](unsafe.Pointer(hull), hull.VertexOffset, int(hull.VertexCount))
}

// GetHullPoints gets read only hull points.
func GetHullPoints(hull *HullData) []Vec3 {
	return blobSlice[Vec3](unsafe.Pointer(hull), hull.PointOffset, int(hull.VertexCount))
}

// GetHullEdges gets read only hull half edges.
func GetHullEdges(hull *HullData) []HullHalfEdge {
	return blobSlice[HullHalfEdge](unsafe.Pointer(hull), hull.EdgeOffset, int(hull.EdgeCount))
}

// GetHullPlanes gets read only hull planes.
func GetHullPlanes(hull *HullData) []Plane {
	return blobSlice[Plane](unsafe.Pointer(hull), hull.PlaneOffset, int(hull.FaceCount))
}

// GetHullFaces gets read only hull faces.
func GetHullFaces(hull *HullData) []HullFace {
	return blobSlice[HullFace](unsafe.Pointer(hull), hull.FaceOffset, int(hull.FaceCount))
}

// GetHullSoaVertices gets read only SOA vertices. This is an array of vertices with all x values,
// y values, and z values as separate arrays. The array lengths are padded to
// a multiple of 8. The padded values are repeats of the first value.
// PORT: the slice holds all three arrays: 3 * getHullSoaStride( VertexCount ) floats.
func GetHullSoaVertices(hull *HullData) []float32 {
	return blobSlice[float32](unsafe.Pointer(hull), hull.SoaVertexOffset, 3*getHullSoaStride(int(hull.VertexCount)))
}

// GetHullSoaNormals gets read only SOA unit normal vectors. This is an array of normals with all x values,
// y values, and z values as separate arrays. The array lengths are padded to
// a multiple of 8. The padded values are 0.
// PORT: the slice holds all three arrays: 3 * getHullSoaStride( FaceCount ) floats.
func GetHullSoaNormals(hull *HullData) []float32 {
	return blobSlice[float32](unsafe.Pointer(hull), hull.SoaNormalOffset, 3*getHullSoaStride(int(hull.FaceCount)))
}

// GetHullEdgeCosines gets the dot products of the two adjacent face normals for each full edge.
// PORT: EdgeCount / 2 floats.
func GetHullEdgeCosines(hull *HullData) []float32 {
	return blobSlice[float32](unsafe.Pointer(hull), hull.EdgeCosineOffset, int(hull.EdgeCount/2))
}

// CreateCylinder, CreateCone, CreateRock, CreateComplexHull, CreateHull, CloneHull,
// CloneAndTransformHull, DestroyHull, MakeCubeHull, MakeBoxHull, MakeOffsetBoxHull,
// MakeTransformedBoxHull, MakeScaledBoxHull and ScaleBox are implemented in hull.go.

// Mesh

// meshBytes is the byte image of a mesh: (uint8_t*)mesh with mesh->byteCount bytes.
func meshBytes(mesh *MeshData) []byte {
	return blobBytes(unsafe.Pointer(mesh), mesh.ByteCount)
}

// GetMeshNodes gets read only mesh BVH nodes.
// PORT: returns NodeCount nodes (nil when the offset is 0).
func GetMeshNodes(mesh *MeshData) []MeshNode {
	return blobSlice[MeshNode](unsafe.Pointer(mesh), mesh.NodeOffset, int(mesh.NodeCount))
}

// GetMeshVertices gets read only mesh vertices.
// PORT: returns VertexCount vertices (nil when the offset is 0).
func GetMeshVertices(mesh *MeshData) []Vec3 {
	return blobSlice[Vec3](unsafe.Pointer(mesh), mesh.VertexOffset, int(mesh.VertexCount))
}

// GetMeshTriangles gets read only mesh triangles.
// PORT: returns TriangleCount triangles (nil when the offset is 0).
func GetMeshTriangles(mesh *MeshData) []MeshTriangle {
	return blobSlice[MeshTriangle](unsafe.Pointer(mesh), mesh.TriangleOffset, int(mesh.TriangleCount))
}

// GetMeshMaterialIndices gets read only mesh materials. The count is equal to the triangle count.
func GetMeshMaterialIndices(mesh *MeshData) []uint8 {
	return blobSlice[uint8](unsafe.Pointer(mesh), mesh.MaterialOffset, int(mesh.TriangleCount))
}

// GetMeshFlags gets read only mesh flags. The count is equal to the triangle count.
func GetMeshFlags(mesh *MeshData) []uint8 {
	return blobSlice[uint8](unsafe.Pointer(mesh), mesh.FlagsOffset, int(mesh.TriangleCount))
}

// CreateGridMesh, CreateWaveMesh, CreateTorusMesh, CreateBoxMesh, CreateHollowBoxMesh,
// CreatePlatformMesh, CreateMesh, DestroyMesh and GetHeight are implemented in mesh.go.

// Height field

// heightFieldBytes is the byte image of a height field: (uint8_t*)hf with hf->byteCount bytes.
func heightFieldBytes(hf *HeightFieldData) []byte {
	return blobBytes(unsafe.Pointer(hf), hf.ByteCount)
}

// GetHeightFieldCompressedHeights gets read only compressed heights. One uint16_t per grid point.
// PORT: returns ColumnCount * RowCount heights (nil when the offset is 0).
func GetHeightFieldCompressedHeights(hf *HeightFieldData) []uint16 {
	return blobSlice[uint16](unsafe.Pointer(hf), hf.HeightsOffset, int(hf.ColumnCount)*int(hf.RowCount))
}

// GetHeightFieldMaterialIndices gets read only material indices. One uint8_t per cell.
func GetHeightFieldMaterialIndices(hf *HeightFieldData) []uint8 {
	return blobSlice[uint8](unsafe.Pointer(hf), hf.MaterialOffset, int(hf.ColumnCount-1)*int(hf.RowCount-1))
}

// GetHeightFieldFlags gets read only triangle flags. One uint8_t per triangle.
func GetHeightFieldFlags(hf *HeightFieldData) []uint8 {
	return blobSlice[uint8](unsafe.Pointer(hf), hf.FlagsOffset, 2*int(hf.ColumnCount-1)*int(hf.RowCount-1))
}

// CreateHeightField, CreateGrid, CreateWave, DestroyHeightField, DumpHeightData and
// LoadHeightField are implemented in height_field.go.

// Query

// MeshQueryFcn is the query callback.
type MeshQueryFcn func(a, b, c Vec3, triangleIndex int, context any) bool

// Collision

// Point2D is used for computing 2D hulls in service of contact manifold simplification.
// Placed here for testing. Internal.
type Point2D struct {
	P             Vec2    // 2D point
	Separation    float32 // separation of the associated contact point
	OriginalIndex int32   // original index of the associated contact point
}
