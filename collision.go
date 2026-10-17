// Port of include/box3d/collision.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: collision.h declares functions implemented in dynamic_tree.c, hull.c, mesh.c,
// height_field.c, compound.c, sphere.c, capsule.c, distance.c, manifold.c, mover.c and others;
// each is ported with its .c file. The inline blob accessors (b3GetHullVertices,
// b3GetMeshNodes, b3GetHeightFieldCompressedHeights, …) are ported in milestone 2 together with
// the geometry byte-image layout.

// Dynamic tree

// GetUserData is b3DynamicTree_GetUserData: get proxy user data.
func (tree *DynamicTree) GetUserData(proxyId int) uint64 {
	return tree.Proxies[proxyId].UserData
}

// GetAABB is b3DynamicTree_GetAABB: get the AABB of a proxy.
func (tree *DynamicTree) GetAABB(proxyId int) AABB {
	return tree.Nodes[tree.Proxies[proxyId].Node].AABB
}

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
