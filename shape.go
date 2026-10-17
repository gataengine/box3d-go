// Port of src/shape.h (and later src/shape.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "unsafe"

// ---- shape.h ----

// PORT: b3BroadPhase and b3World forward declarations are not needed in Go.

// shapeFlags is b3ShapeFlags.
type shapeFlags int32

const (
	enableSensorEvents    shapeFlags = 0x01
	enableContactEvents   shapeFlags = 0x02
	enableCustomFiltering shapeFlags = 0x04
	enableHitEvents       shapeFlags = 0x08
	enablePreSolveEvents  shapeFlags = 0x10
	// 0x20 is free, it was the enlarged AABB flag before the broad-phase marked the tree
	enableSpeculative shapeFlags = 0x40
)

// shape is b3Shape.
type shape struct {
	id             int32
	bodyId         int32
	prevShapeId    int32
	nextShapeId    int32
	sensorIndex    int32
	proxyKey       int32
	typ            ShapeType // PORT: type is a Go keyword
	density        float32
	explosionScale float32
	aabbMargin     float32

	aabb          AABB
	localCentroid Vec3

	materialCount int32
	material      SurfaceMaterial
	materials     []SurfaceMaterial // PORT: b3SurfaceMaterial* heap array; nil for a single material shape

	filter    Filter
	userData  any
	userShape any

	nameId     uint32
	generation uint16

	// b3ShapeFlags
	flags uint8

	// PORT: tagged union { capsule; sphere; hull; mesh; heightField; compound }. All members are
	// present; only the one selected by typ is meaningful.
	capsule     Capsule
	sphere      Sphere
	hull        *HullData
	mesh        Mesh
	heightField *HeightFieldData
	compound    *CompoundData
}

// getShapeMaterials is b3GetShapeMaterials.
// A single material shape keeps its material inline. Multi material meshes and compounds own a heap
// array. Reach the materials the same way for both: a single material shape presents its inline
// material as a one element array. Do not cache the pointer, the shapes array can move.
func getShapeMaterials(shape *shape) []SurfaceMaterial {
	if shape.materials != nil {
		return shape.materials
	}
	return unsafe.Slice(&shape.material, 1)
}

// PORT: the functions declared in shape.h (b3CreateShapeProxy, b3DestroyShapeProxy,
// b3DestroyShapeAllocations, b3ComputeShapeMass, b3ComputeShapeExtent, b3ComputeShapeMinExtent,
// b3ComputeSweptSphereAABB, b3ComputeSweptCapsuleAABB, b3ComputeShapeAABB, b3ComputeFatShapeAABB,
// b3ComputeSweptShapeAABB, b3GetShapeCentroid, b3GetShapeArea, b3GetShapeProjectedArea,
// b3GetShapeUserMaterialId, b3MakeShapeProxy, b3MakeLocalProxy, b3ComputeProxyAABB, b3RayCastShape,
// b3ShapeCastShape, b3OverlapShape, b3ShapeTimeOfImpact, b3CollideMover) are defined in shape.c
// and land with it (milestone 3). b3CollideMoverAndSphere / Capsule / Hull / Mesh / HeightField,
// the hull functions (b3FindHullSupportVertex, …) and the height field / mesh functions are defined
// in sphere.c, capsule.c, hull.c, mesh.c and height_field.c and are ported with those files.

// Height field

// getHeightFieldTriangleCount is b3GetHeightFieldTriangleCount.
func getHeightFieldTriangleCount(heightField *HeightFieldData) int {
	cellCount := int(heightField.RowCount-1) * int(heightField.ColumnCount-1)
	return 2 * cellCount
}

// Mesh

// getMeshTriangle is b3GetMeshTriangle.
// Making this inline made the tree benchmarks faster.
func getMeshTriangle(mesh *Mesh, triangleIndex int) triangle {
	assert(0 <= triangleIndex && triangleIndex < int(mesh.Data.TriangleCount), "0 <= triangleIndex && triangleIndex < mesh->data->triangleCount")

	triangles := GetMeshTriangles(mesh.Data)
	flags := GetMeshFlags(mesh.Data)
	vertices := GetMeshVertices(mesh.Data)

	var result triangle
	tri := triangles[triangleIndex]
	triangleFlags := flags[triangleIndex]

	scale := mesh.Scale

	result.vertices[0] = Mul(scale, vertices[tri.Index1])
	result.i1 = tri.Index1

	if scale.X*scale.Y*scale.Z < 0.0 {
		result.vertices[1] = Mul(scale, vertices[tri.Index3])
		result.vertices[2] = Mul(scale, vertices[tri.Index2])

		result.i2 = tri.Index3
		result.i3 = tri.Index2

		// mesh is inverted, so concave edges are now convex
		result.flags = 0
		if MeshEdgeFlags(triangleFlags)&InverseConcaveEdge1 != 0 {
			result.flags |= int32(ConcaveEdge1)
		}
		if MeshEdgeFlags(triangleFlags)&InverseConcaveEdge2 != 0 {
			result.flags |= int32(ConcaveEdge2)
		}
		if MeshEdgeFlags(triangleFlags)&InverseConcaveEdge3 != 0 {
			result.flags |= int32(ConcaveEdge3)
		}
	} else {
		result.vertices[1] = Mul(scale, vertices[tri.Index2])
		result.vertices[2] = Mul(scale, vertices[tri.Index3])

		result.i2 = tri.Index2
		result.i3 = tri.Index3
		result.flags = int32(triangleFlags)
	}

	return result
}

// isValidMesh (b3IsValidMesh) is implemented in mesh.go.

// shouldShapesCollide is b3ShouldShapesCollide.
func shouldShapesCollide(filterA, filterB Filter) bool {
	if filterA.GroupIndex == filterB.GroupIndex && filterA.GroupIndex != 0 {
		return filterA.GroupIndex > 0
	}

	return (filterA.MaskBits&filterB.CategoryBits) != 0 && (filterA.CategoryBits&filterB.MaskBits) != 0
}

// shouldQueryCollide is b3ShouldQueryCollide.
func shouldQueryCollide(shapeFilter *Filter, queryFilter *QueryFilter) bool {
	return (shapeFilter.CategoryBits&queryFilter.MaskBits) != 0 &&
		(shapeFilter.MaskBits&queryFilter.CategoryBits) != 0
}

// isConvex is b3IsConvex.
func isConvex(typ ShapeType) bool {
	return typ == SphereShape || typ == CapsuleShape || typ == HullShape
}

// ---- shape.c ----

// PORT: only b3MakeLocalProxy and b3ComputeProxyAABB are ported so far. mesh.c, height_field.c
// and compound.c need them in milestone 2; the rest of shape.c lands in milestone 3 and goes
// around these two in upstream order.

// makeLocalProxy is b3MakeLocalProxy.
// PORT: buffer is the caller's [MaxShapeCastPoints]Vec3 array; the returned proxy points into it.
func makeLocalProxy(proxy *ShapeProxy, transform Transform, buffer []Vec3) ShapeProxy {
	invTransform := InvertTransform(transform)
	R := MakeMatrixFromQuat(invTransform.Q)

	count := MinInt(int(proxy.Count), MaxShapeCastPoints)
	for i := range count {
		buffer[i] = Add(MulMV(R, proxy.Points[i]), invTransform.P)
	}

	return ShapeProxy{
		Points: buffer,
		Count:  int32(count),
		Radius: proxy.Radius,
	}
}

// computeProxyAABB is b3ComputeProxyAABB.
func computeProxyAABB(proxy *ShapeProxy) AABB {
	points := proxy.Points
	aabb := AABB{
		LowerBound: points[0],
		UpperBound: points[0],
	}

	for i := 1; i < int(proxy.Count); i++ {
		aabb.LowerBound = Min(aabb.LowerBound, points[i])
		aabb.UpperBound = Max(aabb.UpperBound, points[i])
	}

	r := Vec3{proxy.Radius, proxy.Radius, proxy.Radius}
	aabb.LowerBound = Sub(aabb.LowerBound, r)
	aabb.UpperBound = Add(aabb.UpperBound, r)
	return aabb
}
