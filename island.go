// Port of src/island.h (and later src/island.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- island.h ----

// PORT: header types only; island.c lands in milestone 3.

// contactLink is b3ContactLink.
// Cached contact data stored in the island for fast contiguous iteration.
// Avoids touching b3Contact during union-find in b3SplitIsland.
type contactLink struct {
	contactId int32
	bodyIdA   int32
	bodyIdB   int32
}

// jointLink is b3JointLink.
// Cached joint data stored in the island for fast contiguous iteration.
type jointLink struct {
	jointId int32
	bodyIdA int32
	bodyIdB int32
}

// Deterministic solver
//
// Collide all awake contacts
// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
// created in a deterministic order. bit-wise OR together bit arrays and issue changes:
// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root
// - stop touching: increment constraintRemoveCount

// island is b3Island.
// Persistent island for awake bodies, joints, and contacts.
// Contacts are touching.
// Contacts and joints may connect to static bodies, but static bodies are not in the island.
// https://en.wikipedia.org/wiki/Component_(graph_theory)
// https://en.wikipedia.org/wiki/Dynamic_connectivity
type island struct {
	// index of solver set stored in b3World
	// may be B3_NULL_INDEX
	setIndex int32

	// island index within set
	// may be B3_NULL_INDEX
	localIndex int32

	islandId int32

	// Keeps track of how many contacts have been removed from this island.
	// This is used to determine if an island is a candidate for splitting.
	constraintRemoveCount int32

	// I tried using a stack array for this but the data pointer goes out of
	// sync when the world island array grows.
	bodies []int32

	// Contacts and joints that belong to this island. May connect to static
	// bodies not in the island.
	// Each link has the two body ids so that b3SplitIsland's union-find pass
	// never needs to touch b3Contact/b3Joint.
	contacts []contactLink
	joints   []jointLink
}

// islandSim is b3IslandSim: this is used to move islands across solver sets
type islandSim struct {
	islandId int32
}

// ---- island.c ----

// PORT: stubs until island.c lands in milestone 3. Other milestone 3 files (body.c, contact.c,
// solver.c and physics_world.c) call these; the owner of island.c replaces each stub with the real
// function, keeping the signature, in upstream order.

// createIsland is b3CreateIsland.
func createIsland(world *world, setIndex int) *island {
	panic("box3d: b3CreateIsland not ported yet (island.c, milestone 3)")
}

// destroyIsland is b3DestroyIsland.
func destroyIsland(world *world, islandId int) {
	panic("box3d: b3DestroyIsland not ported yet (island.c, milestone 3)")
}

// linkContact is b3LinkContact.
func linkContact(world *world, contact *contact) {
	panic("box3d: b3LinkContact not ported yet (island.c, milestone 3)")
}

// unlinkContact is b3UnlinkContact.
func unlinkContact(world *world, contact *contact) {
	panic("box3d: b3UnlinkContact not ported yet (island.c, milestone 3)")
}

// linkJoint is b3LinkJoint.
func linkJoint(world *world, joint *joint) {
	panic("box3d: b3LinkJoint not ported yet (island.c, milestone 3)")
}

// unlinkJoint is b3UnlinkJoint.
func unlinkJoint(world *world, joint *joint) {
	panic("box3d: b3UnlinkJoint not ported yet (island.c, milestone 3)")
}

// splitIsland is b3SplitIsland.
func splitIsland(world *world, baseId int) {
	panic("box3d: b3SplitIsland not ported yet (island.c, milestone 3)")
}

// splitIslandTask is b3SplitIslandTask.
func splitIslandTask(context any) {
	panic("box3d: b3SplitIslandTask not ported yet (island.c, milestone 3)")
}

// validateIsland is b3ValidateIsland.
func validateIsland(world *world, islandId int) {
	panic("box3d: b3ValidateIsland not ported yet (island.c, milestone 3)")
}
