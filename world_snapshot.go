// Port of src/world_snapshot.h and src/world_snapshot.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// serializeWorld is b3SerializeWorld.
// Serialize the live world into buf, interning shape geometry into rec->registry.
// On success buf holds a self-contained snapshot image. Returns the byte count.
// PORT: stub until world_snapshot.c is ported (next commit): writes nothing.
func serializeWorld(world *world, buf *recBuffer, rec *Recording) int {
	return 0
}
