// Port of src/mesh_contact.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- mesh_contact.c ----

// PORT: stubs until mesh_contact.c lands in milestone 3. Other milestone 3 files (contact.c) call
// these; the owner of mesh_contact.c replaces each stub with the real function, keeping the
// signature, in upstream order.

// computeMeshManifolds is b3ComputeMeshManifolds.
func computeMeshManifolds(world *world, workerIndex int, contact *contact, shapeA *shape, materialMap []uint16,
	xfA WorldTransform, shapeB *shape, xfB WorldTransform, isFast bool, arena arena) bool {
	panic("box3d: b3ComputeMeshManifolds not ported yet (mesh_contact.c, milestone 3)")
}
