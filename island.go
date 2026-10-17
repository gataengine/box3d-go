// Port of src/island.h and src/island.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- island.h ----

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

// createIsland is b3CreateIsland.
func createIsland(world *world, setIndex int) *island {
	assert(setIndex == awakeSet || setIndex >= firstSleepingSet, "setIndex == b3_awakeSet || setIndex >= b3_firstSleepingSet")

	islandId := allocId(&world.islandIdPool)

	if islandId == len(world.islands) {
		emptyIsland := island{}
		arrayPush(&world.islands, emptyIsland)
	} else {
		assert(world.islands[islandId].setIndex == NullIndex, "world->islands.data[islandId].setIndex == B3_NULL_INDEX")
	}

	set := arrayGet(world.solverSets, setIndex)

	island := arrayGet(world.islands, islandId)
	island.setIndex = int32(setIndex)
	island.localIndex = int32(len(set.islandSims))
	island.islandId = int32(islandId)
	arrayCreate(&island.bodies)
	arrayCreate(&island.contacts)
	arrayCreate(&island.joints)
	island.constraintRemoveCount = 0

	islandSim := arrayEmplace(&set.islandSims)
	islandSim.islandId = int32(islandId)

	return island
}

// destroyIsland is b3DestroyIsland.
func destroyIsland(world *world, islandId int) {
	if int(world.splitIslandId) == islandId {
		world.splitIslandId = NullIndex
	}

	// assume island is empty
	island := arrayGet(world.islands, islandId)
	set := arrayGet(world.solverSets, int(island.setIndex))
	{
		localIndex := int(island.localIndex)
		lastIndex := len(set.islandSims) - 1
		assert(0 <= localIndex && localIndex <= lastIndex, "0 <= localIndex && localIndex <= lastIndex")
		moveIslandId := set.islandSims[lastIndex].islandId
		set.islandSims[localIndex] = set.islandSims[lastIndex]
		world.islands[moveIslandId].localIndex = int32(localIndex)
		set.islandSims = set.islandSims[:lastIndex]
	}

	// Free island and id (preserve island generation)
	arrayDestroy(&island.bodies)
	arrayDestroy(&island.contacts)
	arrayDestroy(&island.joints)
	island.constraintRemoveCount = 0
	island.localIndex = NullIndex
	island.islandId = NullIndex
	island.setIndex = NullIndex

	freeId(&world.islandIdPool, islandId)
}

// mergeIslands is b3MergeIslands.
func mergeIslands(world *world, islandIdA, islandIdB int) int {
	if islandIdA == islandIdB {
		return islandIdA
	}

	if islandIdA == NullIndex {
		assert(islandIdB != NullIndex, "islandIdB != B3_NULL_INDEX")
		return islandIdB
	}

	if islandIdB == NullIndex {
		assert(islandIdA != NullIndex, "islandIdA != B3_NULL_INDEX")
		return islandIdA
	}

	var smallIsland *island
	var bigIsland *island
	{
		islandA := arrayGet(world.islands, islandIdA)
		islandB := arrayGet(world.islands, islandIdB)

		// Keep the biggest island to reduce cache misses
		if len(islandA.bodies) >= len(islandB.bodies) {
			bigIsland = islandA
			smallIsland = islandB
		} else {
			bigIsland = islandB
			smallIsland = islandA
		}
	}

	bigIslandId := bigIsland.islandId
	arrayReserve(&bigIsland.bodies, len(bigIsland.bodies)+len(smallIsland.bodies))

	// Move bodies from smaller island to larger island
	for i := 0; i < len(smallIsland.bodies); i++ {
		bodyId := smallIsland.bodies[i]
		body := arrayGet(world.bodies, int(bodyId))
		validate(body.islandId == smallIsland.islandId, "body->islandId == smallIsland->islandId")
		body.islandId = bigIslandId
		body.islandIndex = int32(len(bigIsland.bodies))
		arrayPush(&bigIsland.bodies, bodyId)
	}

	// Migrate contacts from smaller island to larger island
	if len(smallIsland.contacts) > 0 {
		arrayReserve(&bigIsland.contacts, len(bigIsland.contacts)+len(smallIsland.contacts))

		for i := 0; i < len(smallIsland.contacts); i++ {
			link := &smallIsland.contacts[i]
			contact := arrayGet(world.contacts, int(link.contactId))
			contact.islandId = bigIslandId
			contact.islandIndex = int32(len(bigIsland.contacts))
			arrayPush(&bigIsland.contacts, *link)
		}
	}

	// Migrate joints from smaller island to larger island
	if len(smallIsland.joints) > 0 {
		arrayReserve(&bigIsland.joints, len(bigIsland.joints)+len(smallIsland.joints))

		for i := 0; i < len(smallIsland.joints); i++ {
			link := &smallIsland.joints[i]
			joint := arrayGet(world.joints, int(link.jointId))
			joint.islandId = bigIslandId
			joint.islandIndex = int32(len(bigIsland.joints))
			arrayPush(&bigIsland.joints, *link)
		}
	}

	// Track removed constraints
	bigIsland.constraintRemoveCount += smallIsland.constraintRemoveCount

	destroyIsland(world, int(smallIsland.islandId))

	validateIsland(world, int(bigIslandId))

	return int(bigIslandId)
}

// addContactToIsland is b3AddContactToIsland.
func addContactToIsland(world *world, islandId int, contact *contact) {
	assert(contact.islandId == NullIndex, "contact->islandId == B3_NULL_INDEX")
	assert(contact.islandIndex == NullIndex, "contact->islandIndex == B3_NULL_INDEX")

	island := arrayGet(world.islands, islandId)

	contact.islandId = int32(islandId)
	contact.islandIndex = int32(len(island.contacts))

	var link contactLink
	link.contactId = contact.contactId
	link.bodyIdA = contact.edges[0].bodyId
	link.bodyIdB = contact.edges[1].bodyId
	arrayPush(&island.contacts, link)

	validateIsland(world, islandId)
}

// linkContact is b3LinkContact.
// Link a contact into an island.
func linkContact(world *world, contact *contact) {
	assert((contact.flags&contactTouchingFlag) != 0, "( contact->flags & b3_contactTouchingFlag ) != 0")

	bodyIdA := int(contact.edges[0].bodyId)
	bodyIdB := int(contact.edges[1].bodyId)

	bodyA := arrayGet(world.bodies, bodyIdA)
	bodyB := arrayGet(world.bodies, bodyIdB)

	assert(bodyA.setIndex != disabledSet && bodyB.setIndex != disabledSet, "bodyA->setIndex != b3_disabledSet && bodyB->setIndex != b3_disabledSet")
	assert(bodyA.setIndex != staticSet || bodyB.setIndex != staticSet, "bodyA->setIndex != b3_staticSet || bodyB->setIndex != b3_staticSet")

	// Wake bodyB if bodyA is awake and bodyB is sleeping
	if bodyA.setIndex == awakeSet && bodyB.setIndex >= firstSleepingSet {
		wakeSolverSet(world, int(bodyB.setIndex))
	}

	// Wake bodyA if bodyB is awake and bodyA is sleeping
	if bodyB.setIndex == awakeSet && bodyA.setIndex >= firstSleepingSet {
		wakeSolverSet(world, int(bodyA.setIndex))
	}

	islandIdA := int(bodyA.islandId)
	islandIdB := int(bodyB.islandId)

	// Static bodies have null island indices.
	assert(bodyA.setIndex != staticSet || islandIdA == NullIndex, "bodyA->setIndex != b3_staticSet || islandIdA == B3_NULL_INDEX")
	assert(bodyB.setIndex != staticSet || islandIdB == NullIndex, "bodyB->setIndex != b3_staticSet || islandIdB == B3_NULL_INDEX")
	assert(islandIdA != NullIndex || islandIdB != NullIndex, "islandIdA != B3_NULL_INDEX || islandIdB != B3_NULL_INDEX")

	// Merge islands. This will destroy one of the islands.
	finalIslandId := mergeIslands(world, islandIdA, islandIdB)

	// Add contact to the island that survived
	addContactToIsland(world, finalIslandId, contact)
}

// unlinkContact is b3UnlinkContact.
// This is called when a contact no longer has contact points or when a contact is destroyed.
func unlinkContact(world *world, contact *contact) {
	assert(contact.islandId != NullIndex, "contact->islandId != B3_NULL_INDEX")

	// remove from island
	islandId := int(contact.islandId)
	island := arrayGet(world.islands, islandId)

	removeIndex := int(contact.islandIndex)
	assert(0 <= removeIndex && removeIndex < len(island.contacts), "0 <= removeIndex && removeIndex < island->contacts.count")
	assert(island.contacts[removeIndex].contactId == contact.contactId, "island->contacts.data[removeIndex].contactId == contact->contactId")

	movedIndex := arrayRemoveSwap(&island.contacts, removeIndex)
	if movedIndex != NullIndex {
		// Fix islandIndex on the contact that was swapped into removeIndex
		movedLink := &island.contacts[removeIndex]
		movedContact := arrayGet(world.contacts, int(movedLink.contactId))
		assert(int(movedContact.islandIndex) == movedIndex, "movedContact->islandIndex == movedIndex")
		movedContact.islandIndex = int32(removeIndex)
	}

	contact.islandId = NullIndex
	contact.islandIndex = NullIndex
	island.constraintRemoveCount += 1

	validateIsland(world, islandId)
}

// addJointToIsland is b3AddJointToIsland.
func addJointToIsland(world *world, islandId int, joint *joint) {
	assert(joint.islandId == NullIndex, "joint->islandId == B3_NULL_INDEX")
	assert(joint.islandIndex == NullIndex, "joint->islandIndex == B3_NULL_INDEX")

	island := arrayGet(world.islands, islandId)

	joint.islandId = int32(islandId)
	joint.islandIndex = int32(len(island.joints))

	var link jointLink
	link.jointId = joint.jointId
	link.bodyIdA = joint.edges[0].bodyId
	link.bodyIdB = joint.edges[1].bodyId
	arrayPush(&island.joints, link)

	validateIsland(world, islandId)
}

// linkJoint is b3LinkJoint.
func linkJoint(world *world, joint *joint) {
	bodyA := arrayGet(world.bodies, int(joint.edges[0].bodyId))
	bodyB := arrayGet(world.bodies, int(joint.edges[1].bodyId))

	assert(bodyA.typ == DynamicBody || bodyB.typ == DynamicBody, "bodyA->type == b3_dynamicBody || bodyB->type == b3_dynamicBody")

	if bodyA.setIndex == awakeSet && bodyB.setIndex >= firstSleepingSet {
		wakeSolverSet(world, int(bodyB.setIndex))
	} else if bodyB.setIndex == awakeSet && bodyA.setIndex >= firstSleepingSet {
		wakeSolverSet(world, int(bodyA.setIndex))
	}

	islandIdA := int(bodyA.islandId)
	islandIdB := int(bodyB.islandId)

	assert(islandIdA != NullIndex || islandIdB != NullIndex, "islandIdA != B3_NULL_INDEX || islandIdB != B3_NULL_INDEX")

	// Merge islands. This will destroy one of the islands.
	finalIslandId := mergeIslands(world, islandIdA, islandIdB)

	// Add joint the island that survived
	addJointToIsland(world, finalIslandId, joint)
}

// unlinkJoint is b3UnlinkJoint.
func unlinkJoint(world *world, joint *joint) {
	if joint.islandId == NullIndex {
		return
	}

	// remove from island
	islandId := int(joint.islandId)
	island := arrayGet(world.islands, islandId)

	removeIndex := int(joint.islandIndex)
	assert(0 <= removeIndex && removeIndex < len(island.joints), "0 <= removeIndex && removeIndex < island->joints.count")
	assert(island.joints[removeIndex].jointId == joint.jointId, "island->joints.data[removeIndex].jointId == joint->jointId")

	movedIndex := arrayRemoveSwap(&island.joints, removeIndex)
	if movedIndex != NullIndex {
		// Fix islandIndex on the joint that was swapped into removeIndex
		movedLink := &island.joints[removeIndex]
		movedJoint := arrayGet(world.joints, int(movedLink.jointId))
		assert(int(movedJoint.islandIndex) == movedIndex, "movedJoint->islandIndex == movedIndex")
		movedJoint.islandIndex = int32(removeIndex)
	}

	joint.islandId = NullIndex
	joint.islandIndex = NullIndex
	island.constraintRemoveCount += 1

	validateIsland(world, islandId)
}

// islandFindParent is b3IslandFindParent.
// Find parent of a node. Use path halving to speed up further queries.
func islandFindParent(parents []int32, node int) int {
	// Walk the chain of parents to find the node that is its own parent (the root)
	for int(parents[node]) != node {
		grandParent := parents[parents[node]]
		parents[node] = grandParent
		node = int(grandParent)
	}

	return node
}

// islandUnion is b3IslandUnion.
// Connect the components containing node1 and node2.
// Uses rank to keep tree balanced. Tracks per-component contact and joint counts.
func islandUnion(parents []int32, ranks []int32, node1, node2 int, contactCounts []int32, jointCounts []int32) {
	root1 := islandFindParent(parents, node1)
	root2 := islandFindParent(parents, node2)
	if root1 != root2 {
		if ranks[root1] < ranks[root2] {
			parents[root1] = int32(root2)
			contactCounts[root2] += contactCounts[root1]
			jointCounts[root2] += jointCounts[root1]
		} else if ranks[root1] > ranks[root2] {
			parents[root2] = int32(root1)
			contactCounts[root1] += contactCounts[root2]
			jointCounts[root1] += jointCounts[root2]
		} else {
			parents[root2] = int32(root1)
			ranks[root1] += 1
			contactCounts[root1] += contactCounts[root2]
			jointCounts[root1] += jointCounts[root2]
		}
	}
}

// splitIsland is b3SplitIsland.
// This uses union-find.
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
func splitIsland(world *world, baseId int) {
	baseIsland := arrayGet(world.islands, baseId)
	assert(baseIsland.constraintRemoveCount > 0, "baseIsland->constraintRemoveCount > 0")
	assert(baseIsland.setIndex == awakeSet, "baseIsland->setIndex == b3_awakeSet")

	validateIsland(world, baseId)

	// Cache base island fields before b3CreateIsland, which may reallocate
	// world->islands and invalidate the baseIsland pointer.
	// PORT: the cached slices carry data, count and capacity together.
	baseBodyCount := len(baseIsland.bodies)
	baseBodyIds := baseIsland.bodies

	baseContactCount := len(baseIsland.contacts)
	baseContacts := baseIsland.contacts

	baseJointCount := len(baseIsland.joints)
	baseJoints := baseIsland.joints

	alloc := &world.stack

	// No lock is needed because I ensure the allocator is not used while this task is active.
	// Allocate contactCounts and jointCounts before ranks so ranks can be freed first (LIFO arena).
	parents := stackAlloc[int32](alloc, baseBodyCount, "parents")
	contactCounts := stackAlloc[int32](alloc, baseBodyCount, "contact counts")
	jointCounts := stackAlloc[int32](alloc, baseBodyCount, "joint counts")
	ranks := stackAlloc[int32](alloc, baseBodyCount, "ranks")
	for i := 0; i < baseBodyCount; i++ {
		parents[i] = int32(i)
		ranks[i] = 0
		contactCounts[i] = 0
		jointCounts[i] = 0
	}

	bodies := world.bodies

	// Union over contacts, tracking per-component contact counts
	for i := 0; i < baseContactCount; i++ {
		bodyIdA := int(baseContacts[i].bodyIdA)
		bodyIdB := int(baseContacts[i].bodyIdB)
		validate(0 <= bodyIdA && bodyIdA < len(world.bodies), "0 <= bodyIdA && bodyIdA < world->bodies.count")
		validate(0 <= bodyIdB && bodyIdB < len(world.bodies), "0 <= bodyIdB && bodyIdB < world->bodies.count")
		bodyA := &bodies[bodyIdA]
		bodyB := &bodies[bodyIdB]
		islandIndexA := int(bodyA.islandIndex)
		islandIndexB := int(bodyB.islandIndex)

		// Only connect non-static bodies
		if islandIndexA != NullIndex && islandIndexB != NullIndex {
			validate(0 <= islandIndexA && islandIndexA < baseBodyCount, "0 <= islandIndexA && islandIndexA < baseBodyCount")
			validate(0 <= islandIndexB && islandIndexB < baseBodyCount, "0 <= islandIndexB && islandIndexB < baseBodyCount")
			islandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts)
			root := islandFindParent(parents, islandIndexA)
			contactCounts[root] += 1
		} else {
			islandIndex := islandIndexB
			if islandIndexA != NullIndex {
				islandIndex = islandIndexA
			}
			root := islandFindParent(parents, islandIndex)
			contactCounts[root] += 1
		}
	}

	// Union over joints, tracking per-component joint counts
	for i := 0; i < baseJointCount; i++ {
		bodyIdA := int(baseJoints[i].bodyIdA)
		bodyIdB := int(baseJoints[i].bodyIdB)
		validate(0 <= bodyIdA && bodyIdA < len(world.bodies), "0 <= bodyIdA && bodyIdA < world->bodies.count")
		validate(0 <= bodyIdB && bodyIdB < len(world.bodies), "0 <= bodyIdB && bodyIdB < world->bodies.count")
		bodyA := &bodies[bodyIdA]
		bodyB := &bodies[bodyIdB]
		islandIndexA := int(bodyA.islandIndex)
		islandIndexB := int(bodyB.islandIndex)

		// Only connect non-static bodies
		if islandIndexA != NullIndex && islandIndexB != NullIndex {
			validate(0 <= islandIndexA && islandIndexA < baseBodyCount, "0 <= islandIndexA && islandIndexA < baseBodyCount")
			validate(0 <= islandIndexB && islandIndexB < baseBodyCount, "0 <= islandIndexB && islandIndexB < baseBodyCount")
			islandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts)
			root := islandFindParent(parents, islandIndexA)
			jointCounts[root] += 1
		} else {
			islandIndex := islandIndexB
			if islandIndexA != NullIndex {
				islandIndex = islandIndexA
			}
			root := islandFindParent(parents, islandIndex)
			jointCounts[root] += 1
		}
	}

	// Done with ranks
	stackFree(alloc, ranks)
	ranks = nil

	// Flatten all parent indices and count connected components.
	componentCount := 0
	for i := 0; i < baseBodyCount; i++ {
		parents[i] = int32(islandFindParent(parents, i))
		if int(parents[i]) == i {
			componentCount += 1
		}
	}

	// Island is still fully connected, no split needed.
	if componentCount == 1 {
		baseIsland.constraintRemoveCount = 0
		stackFree(alloc, jointCounts)
		stackFree(alloc, contactCounts)
		stackFree(alloc, parents)
		return
	}

	// Detach body/contact/joint arrays from base island so b3DestroyIsland won't free them
	// PORT: data, count and capacity reset together by clearing the slice.
	baseIsland.bodies = nil

	baseIsland.contacts = nil

	baseIsland.joints = nil

	// Null so code below doesn't accidentally use this.
	baseIsland = nil

	// Map from body index to new island index. Only set for root bodies.
	rootMap := stackAlloc[int32](alloc, baseBodyCount, "root map")
	for i := 0; i < baseBodyCount; i++ {
		rootMap[i] = NullIndex
	}

	componentBodyCounts := stackAlloc[int32](alloc, componentCount, "component body counts")
	componentContactCounts := stackAlloc[int32](alloc, componentCount, "component contact counts")
	componentJointCounts := stackAlloc[int32](alloc, componentCount, "component joint counts")
	islandCount := 0

	// Find the root body for each body and create islands as needed.
	// Extract per-component counts from the root nodes' accumulated counts.
	for i := 0; i < baseBodyCount; i++ {
		rootIndex := parents[i]
		if rootMap[rootIndex] == NullIndex {
			rootMap[rootIndex] = int32(islandCount)
			componentBodyCounts[islandCount] = 0
			componentContactCounts[islandCount] = contactCounts[rootIndex]
			componentJointCounts[islandCount] = jointCounts[rootIndex]
			islandCount += 1
		}

		componentBodyCounts[rootMap[rootIndex]] += 1
	}

	assert(islandCount == componentCount, "islandCount == componentCount")

	// Map from new island index to island id
	islandIds := stackAlloc[int32](alloc, islandCount, "island ids")

	// Create new islands and reserve body/contact/joint arrays
	for i := 0; i < islandCount; i++ {
		// WARNING: this invalidates baseIsland pointer
		newIsland := createIsland(world, awakeSet)
		islandIds[i] = newIsland.islandId

		// Reserve arrays to avoid wasteful growth and memcpy.
		arrayReserve(&newIsland.bodies, int(componentBodyCounts[i]))
		arrayReserve(&newIsland.contacts, int(componentContactCounts[i]))
		arrayReserve(&newIsland.joints, int(componentJointCounts[i]))
	}

	// Assign bodies to new islands
	for i := 0; i < baseBodyCount; i++ {
		bodyId := baseBodyIds[i]
		root := islandFindParent(parents, i)
		newIslandId := int(islandIds[rootMap[root]])

		body := arrayGet(world.bodies, int(bodyId))
		newIsland := arrayGet(world.islands, newIslandId)

		body.islandId = int32(newIslandId)
		body.islandIndex = int32(len(newIsland.bodies))

		// Ensure the array has the correct capacity
		validate(len(newIsland.bodies) < cap(newIsland.bodies), "newIsland->bodies.count < newIsland->bodies.capacity")
		arrayPush(&newIsland.bodies, bodyId)
	}

	// Assign contacts to the island of their bodies
	for i := 0; i < baseContactCount; i++ {
		link := &baseContacts[i]
		contact := arrayGet(world.contacts, int(link.contactId))

		// Static bodies don't have an island id.
		bodyA := arrayGet(world.bodies, int(link.bodyIdA))
		bodyB := arrayGet(world.bodies, int(link.bodyIdB))
		targetIslandId := int(bodyB.islandId)
		if bodyA.islandId != NullIndex {
			targetIslandId = int(bodyA.islandId)
		}

		targetIsland := arrayGet(world.islands, targetIslandId)
		contact.islandId = int32(targetIslandId)
		contact.islandIndex = int32(len(targetIsland.contacts))

		// Ensure the array has the correct capacity
		validate(len(targetIsland.contacts) < cap(targetIsland.contacts), "targetIsland->contacts.count < targetIsland->contacts.capacity")
		arrayPush(&targetIsland.contacts, *link)
	}

	// Assign joints to the island of their bodies
	for i := 0; i < baseJointCount; i++ {
		link := &baseJoints[i]
		joint := arrayGet(world.joints, int(link.jointId))

		// Static bodies don't have an island id.
		bodyA := arrayGet(world.bodies, int(link.bodyIdA))
		bodyB := arrayGet(world.bodies, int(link.bodyIdB))
		targetIslandId := int(bodyB.islandId)
		if bodyA.islandId != NullIndex {
			targetIslandId = int(bodyA.islandId)
		}

		targetIsland := arrayGet(world.islands, targetIslandId)
		joint.islandId = int32(targetIslandId)
		joint.islandIndex = int32(len(targetIsland.joints))

		// Ensure the array has the correct capacity
		validate(len(targetIsland.joints) < cap(targetIsland.joints), "targetIsland->joints.count < targetIsland->joints.capacity")
		arrayPush(&targetIsland.joints, *link)
	}

	// Destroy the base island
	destroyIsland(world, baseId)

	// Free the detached arrays manually
	// PORT: b3Free dropped; the GC reclaims the detached slices (baseBodyIds, baseContacts,
	// baseJoints).

	// Free arena items in LIFO order
	stackFree(alloc, islandIds)
	stackFree(alloc, componentJointCounts)
	stackFree(alloc, componentContactCounts)
	stackFree(alloc, componentBodyCounts)
	stackFree(alloc, rootMap)
	stackFree(alloc, jointCounts)
	stackFree(alloc, contactCounts)
	stackFree(alloc, parents)
}

// splitIslandTask is b3SplitIslandTask.
// Split an island because some contacts and/or joints have been removed.
// This is called during the constraint solve while islands are not being touched. This uses union find and
// touches a lot of memory, so it can be slow.
// Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
// Note: static bodies are never in an island
// Note: this task interacts with some allocators without locks under the assumption that no other tasks
// are interacting with these data structures.
func splitIslandTask(context any) {
	// PORT: Tracy zones dropped.
	ticks := GetTicks()
	world := context.(*world)

	assert(world.splitIslandId != NullIndex, "world->splitIslandId != B3_NULL_INDEX")

	splitIsland(world, int(world.splitIslandId))

	world.splitIslandId = NullIndex
	world.profile.SplitIslands += GetMilliseconds(ticks)
}

// validateIsland is b3ValidateIsland.
// PORT: #if B3_ENABLE_VALIDATION becomes `if !validateEnabled { return }`.
func validateIsland(world *world, islandId int) {
	if !validateEnabled {
		return
	}

	if islandId == NullIndex {
		return
	}

	island := arrayGet(world.islands, islandId)
	assert(int(island.islandId) == islandId, "island->islandId == islandId")
	assert(island.setIndex != NullIndex, "island->setIndex != B3_NULL_INDEX")

	{
		assert(len(island.bodies) > 0, "island->bodies.count > 0")
		assert(len(island.bodies) <= getIdCount(&world.bodyIdPool), "island->bodies.count <= b3GetIdCount( &world->bodyIdPool )")

		for i := 0; i < len(island.bodies); i++ {
			body := arrayGet(world.bodies, int(island.bodies[i]))
			assert(int(body.islandId) == islandId, "body->islandId == islandId")
			assert(int(body.islandIndex) == i, "body->islandIndex == i")
			assert(body.setIndex == island.setIndex, "body->setIndex == island->setIndex")
		}
	}

	if len(island.contacts) > 0 {
		assert(len(island.contacts) <= getIdCount(&world.contactIdPool), "island->contacts.count <= b3GetIdCount( &world->contactIdPool )")

		for i := 0; i < len(island.contacts); i++ {
			link := &island.contacts[i]
			contact := arrayGet(world.contacts, int(link.contactId))
			assert(contact.setIndex == island.setIndex, "contact->setIndex == island->setIndex")
			assert(int(contact.islandId) == islandId, "contact->islandId == islandId")
			assert(int(contact.islandIndex) == i, "contact->islandIndex == i")
		}
	}

	if len(island.joints) > 0 {
		assert(len(island.joints) <= getIdCount(&world.jointIdPool), "island->joints.count <= b3GetIdCount( &world->jointIdPool )")

		for i := 0; i < len(island.joints); i++ {
			link := &island.joints[i]
			joint := arrayGet(world.joints, int(link.jointId))
			assert(joint.setIndex == island.setIndex, "joint->setIndex == island->setIndex")
			assert(int(joint.islandId) == islandId, "joint->islandId == islandId")
			assert(int(joint.islandIndex) == i, "joint->islandIndex == i")
		}
	}
}
