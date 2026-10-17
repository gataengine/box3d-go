// Port of src/broad_phase.h and src/broad_phase.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- broad_phase.h ----

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.

// proxyType is B3_PROXY_TYPE( KEY ).
func proxyType(key int) BodyType {
	return BodyType(key & 3)
}

// proxyId is B3_PROXY_ID( KEY ).
func proxyId(key int) int {
	return key >> 2
}

// proxyKey is B3_PROXY_KEY( ID, TYPE ).
func proxyKey(id int, typ BodyType) int {
	return (id << 2) | int(typ)
}

// broadPhase is b3BroadPhase.
// The broad-phase is used for computing pairs and performing volume queries and ray casts.
// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
// It is up to the client to consume the new pairs and to track subsequent overlap.
type broadPhase struct {
	// One tree for each body type.
	trees [BodyTypeCount]DynamicTree

	// The moved siblings gathered from the dynamic body tree.
	// PORT: int* from the world stack allocator becomes a slice.
	movedSiblings []int32

	// Tracks shape pairs that have a b3Contact
	// todo pairSet can grow quite large on the first time step and remain large
	pairSet hashSet
}

// markProxyMovedSerial is b3BroadPhase_MarkProxyMovedSerial.
func (bp *broadPhase) markProxyMovedSerial(proxyKey int) {
	proxyType := proxyType(proxyKey)
	proxyId := proxyId(proxyKey)
	bp.trees[proxyType].markProxyMovedSerial(proxyId)
}

// markProxyMoved is b3BroadPhase_MarkProxyMoved.
func (bp *broadPhase) markProxyMoved(proxyKey int, aabb AABB) {
	proxyType := proxyType(proxyKey)
	proxyId := proxyId(proxyKey)
	bp.trees[proxyType].markProxyMoved(proxyId, aabb)
}
