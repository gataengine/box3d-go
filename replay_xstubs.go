// Cross-agent stubs for replay.go: functions owned by recording.go (src/recording.c) and
// world_snapshot.go (src/world_snapshot.c) that replay.c calls and that do not exist yet on this
// branch. Signatures follow the upstream declarations through the spec naming rules.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// CreateRecording is b3CreateRecording (box3d.h, recording.c).
// PORT: cross-agent stub, integrator deletes
func CreateRecording(byteCapacity int) *Recording {
	panic("box3d: CreateRecording is a cross-agent stub (recording.c)")
}

// Destroy is b3DestroyRecording (box3d.h, recording.c).
// PORT: cross-agent stub, integrator deletes
func (recording *Recording) Destroy() {
	panic("box3d: Recording.Destroy is a cross-agent stub (recording.c)")
}

// serializeWorld is b3SerializeWorld (world_snapshot.h).
// PORT: cross-agent stub, integrator deletes
func serializeWorld(world *world, buf *recBuffer, rec *Recording) int {
	panic("box3d: serializeWorld is a cross-agent stub (world_snapshot.c)")
}

// deserializeIntoShell is b3DeserializeIntoShell (world_snapshot.h).
// PORT: cross-agent stub, integrator deletes
func deserializeIntoShell(data []byte, world *world, rdr *recReader) bool {
	panic("box3d: deserializeIntoShell is a cross-agent stub (world_snapshot.c)")
}
