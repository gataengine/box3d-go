// Cross-agent stubs for world_snapshot.go: replay.h declarations ported by the replay.c agent.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: cross-agent stub, integrator deletes. registrySlot is b3RegistrySlot (replay.h).
type registrySlot struct {
	kind      geometryKind
	byteCount int32
	bytes     []byte // raw bytes from the file (always freed at teardown)
}

// PORT: cross-agent stub, integrator deletes. recReader is b3RecReader (replay.h); only the
// preloaded geometry registry that b3DeserializeIntoShell reads is declared here.
type recReader struct {
	// Preloaded geometry registry
	slots     []registrySlot
	slotCount int
}

// PORT: cross-agent stub, integrator deletes. recGetSlot is b3RecGetSlot (replay.c), written out
// as upstream instead of panicking so the snapshot round trip is testable before the merge.
// Get a geometry slot with kind validation.
func recGetSlot(rdr *recReader, id uint32, kind geometryKind) *registrySlot {
	if rdr == nil || id >= uint32(rdr.slotCount) || rdr.slots[id].kind != kind {
		return nil
	}
	return &rdr.slots[id]
}
