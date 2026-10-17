// Port of src/solver_set.h and src/solver_set.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- solver_set.h ----

// solverSet is b3SolverSet.
// This holds solver set data. The following sets are used:
// - static set for all static bodies and joints between static bodies
// - active set for all active bodies with body states (no
// contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
// The purpose of solver sets is to achieve high memory locality.
// https://www.youtube.com/watch?v=nZNd5FjSquk
type solverSet struct {
	// Body array. Empty for unused set.
	bodySims []bodySim

	// Body state only exists for active set
	bodyStates []bodyState

	// This holds sleeping/disabled joints. Empty for static/active set.
	jointSims []jointSim

	// This holds all contacts for sleeping sets.
	// This holds non-touching contacts for the awake set.
	// This should be empty for the static and disabled sets.
	contactIndices []int32

	// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
	// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
	// islands will be naturally merged with the set is woken.
	// The static and disabled sets have no islands.
	// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
	islandSims []islandSim

	// Aligns with b3World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
	setIndex int32
}

// ---- solver_set.c ----

// destroySolverSet is b3DestroySolverSet.
func destroySolverSet(world *world, setIndex int) {
	set := arrayGet(world.solverSets, setIndex)

	arrayDestroy(&set.bodySims)
	arrayDestroy(&set.bodyStates)
	arrayDestroy(&set.contactIndices)
	arrayDestroy(&set.jointSims)
	arrayDestroy(&set.islandSims)

	freeId(&world.solverSetIdPool, setIndex)
	*set = solverSet{}
	set.setIndex = NullIndex
}

// wakeSolverSet is b3WakeSolverSet.
// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
func wakeSolverSet(world *world, setIndex int) {
	assert(setIndex >= firstSleepingSet, "setIndex >= b3_firstSleepingSet")
	set := arrayGet(world.solverSets, setIndex)
	awakeSet_ := arrayGet(world.solverSets, awakeSet)       // PORT: awakeSet is the set index constant
	disabledSet_ := arrayGet(world.solverSets, disabledSet) // PORT: disabledSet is the set index constant

	bodies := world.bodies

	bodyCount := len(set.bodySims)
	for i := 0; i < bodyCount; i++ {
		simSrc := &set.bodySims[i]

		body := &bodies[simSrc.bodyId]
		assert(int(body.setIndex) == setIndex, "body->setIndex == setIndex")
		body.setIndex = awakeSet
		body.localIndex = int32(len(awakeSet_.bodySims))

		// Reset sleep timer
		body.sleepTime = 0.0

		simDst := arrayEmplace(&awakeSet_.bodySims)
		*simDst = *simSrc

		state := arrayEmplace(&awakeSet_.bodyStates)
		*state = identityBodyState
		state.flags = body.flags

		// Move non-touching contacts from disabled set to awake set.
		encodedBodySimIndex := body.localIndex
		contactKey := int(body.headContactKey)
		for contactKey != NullIndex {
			edgeIndex := contactKey & 1
			contactId := contactKey >> 1

			contact := arrayGet(world.contacts, contactId)
			contactKey = int(contact.edges[edgeIndex].nextKey)

			if contact.setIndex != disabledSet {
				assert(contact.setIndex == awakeSet || int(contact.setIndex) == setIndex, "contact->setIndex == b3_awakeSet || contact->setIndex == setIndex")

				// If this contact is awake but not touching I need to fixup the body sim index here.
				if contact.setIndex == awakeSet {
					assert(contact.colorIndex == NullIndex, "contact->colorIndex == B3_NULL_INDEX")
					if edgeIndex == 0 {
						contact.encodedBodySimA = encodedBodySimIndex
					} else {
						contact.encodedBodySimB = encodedBodySimIndex
					}
				}

				continue
			}

			localIndex := int(contact.localIndex)

			assert(0 <= localIndex && localIndex < len(disabledSet_.contactIndices), "0 <= localIndex && localIndex < disabledSet->contactIndices.count")
			assert(int(disabledSet_.contactIndices[localIndex]) == contactId, "disabledSet->contactIndices.data[localIndex] == contactId")

			assert((contact.flags&contactTouchingFlag) == 0 && contact.manifoldCount == 0, "( contact->flags & b3_contactTouchingFlag ) == 0 && contact->manifoldCount == 0")

			contact.setIndex = awakeSet
			contact.localIndex = int32(len(awakeSet_.contactIndices))
			arrayPush(&awakeSet_.contactIndices, int32(contactId))

			if edgeIndex == 0 {
				contact.encodedBodySimA = encodedBodySimIndex
			} else {
				contact.encodedBodySimB = encodedBodySimIndex
			}

			movedLocalIndex := arrayRemoveSwap(&disabledSet_.contactIndices, localIndex)
			if movedLocalIndex != NullIndex {
				// fix moved element
				movedContactIndex := int(disabledSet_.contactIndices[localIndex])
				movedContact := arrayGet(world.contacts, movedContactIndex)
				assert(int(movedContact.localIndex) == movedLocalIndex, "movedContact->localIndex == movedLocalIndex")
				movedContact.localIndex = int32(localIndex)
			}
		}
	}

	// Transfer touching contacts from sleeping set to constraint graph.
	{
		contactCount := len(set.contactIndices)
		for i := 0; i < contactCount; i++ {
			contactIndex := int(set.contactIndices[i])
			contact := arrayGet(world.contacts, contactIndex)
			assert(contact.flags&contactTouchingFlag != 0, "contact->flags & b3_contactTouchingFlag")
			assert(contact.flags&simTouchingFlag != 0, "contact->flags & b3_simTouchingFlag")
			assert(int(contact.setIndex) == setIndex, "contact->setIndex == setIndex")
			addContactToGraph(world, contact)
			contact.setIndex = awakeSet
		}
	}

	// transfer joints from sleeping set to awake set
	{
		jointCount := len(set.jointSims)
		for i := 0; i < jointCount; i++ {
			jointSim := &set.jointSims[i]
			joint := arrayGet(world.joints, int(jointSim.jointId))
			assert(int(joint.setIndex) == setIndex, "joint->setIndex == setIndex")
			addJointToGraph(world, jointSim, joint)
			joint.setIndex = awakeSet
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		islandCount := len(set.islandSims)
		for i := 0; i < islandCount; i++ {
			islandSrc := &set.islandSims[i]
			island := arrayGet(world.islands, int(islandSrc.islandId))
			island.setIndex = awakeSet
			island.localIndex = int32(len(awakeSet_.islandSims))
			islandDst := arrayEmplace(&awakeSet_.islandSims)
			*islandDst = *islandSrc
		}
	}

	// destroy the sleeping set
	destroySolverSet(world, setIndex)
}

// trySleepIsland is b3TrySleepIsland.
func trySleepIsland(world *world, islandId int) {
	island := arrayGet(world.islands, islandId)
	assert(island.setIndex == awakeSet, "island->setIndex == b3_awakeSet")

	// Cannot put an island to sleep while it has a pending split and more than one body.
	if island.constraintRemoveCount > 0 && len(island.bodies) > 1 {
		return
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	sleepSetId := allocId(&world.solverSetIdPool)
	if sleepSetId == len(world.solverSets) {
		set := solverSet{}
		set.setIndex = NullIndex
		arrayPush(&world.solverSets, set)
	}

	sleepSet := arrayGet(world.solverSets, sleepSetId)
	*sleepSet = solverSet{}

	// grab awake set after creating the sleep set because the solver set array may have been resized
	awakeSet_ := arrayGet(world.solverSets, awakeSet)       // PORT: awakeSet is the set index constant
	disabledSet_ := arrayGet(world.solverSets, disabledSet) // PORT: disabledSet is the set index constant
	assert(0 <= island.localIndex && int(island.localIndex) < len(awakeSet_.islandSims), "0 <= island->localIndex && island->localIndex < awakeSet->islandSims.count")

	sleepSet.setIndex = int32(sleepSetId)
	arrayReserve(&sleepSet.bodySims, len(island.bodies))
	arrayReserve(&sleepSet.contactIndices, len(island.contacts))
	arrayReserve(&sleepSet.jointSims, len(island.joints))

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		for i := 0; i < len(island.bodies); i++ {
			bodyId := int(island.bodies[i])
			body := arrayGet(world.bodies, bodyId)
			assert(body.setIndex == awakeSet, "body->setIndex == b3_awakeSet")
			assert(int(body.islandId) == islandId, "body->islandId == islandId")
			assert(int(body.islandIndex) == i, "body->islandIndex == i")

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			if body.bodyMoveIndex != NullIndex {
				moveEvent := arrayGet(world.bodyMoveEvents, int(body.bodyMoveIndex))
				assert(int(moveEvent.BodyId.Index1)-1 == bodyId, "moveEvent->bodyId.index1 - 1 == bodyId")
				assert(moveEvent.BodyId.Generation == body.generation, "moveEvent->bodyId.generation == body->generation")
				moveEvent.FellAsleep = true
				body.bodyMoveIndex = NullIndex
			}

			awakeBodyIndex := int(body.localIndex)
			awakeSim := arrayGet(awakeSet_.bodySims, awakeBodyIndex)

			// move body sim to sleep set
			sleepBodyIndex := len(sleepSet.bodySims)
			sleepBodySim := arrayEmplace(&sleepSet.bodySims)
			*sleepBodySim = *awakeSim

			movedIndex := arrayRemoveSwap(&awakeSet_.bodySims, awakeBodyIndex)
			if movedIndex != NullIndex {
				// fix local index on moved element
				movedSim := &awakeSet_.bodySims[awakeBodyIndex]
				movedId := int(movedSim.bodyId)
				movedBody := arrayGet(world.bodies, movedId)
				assert(int(movedBody.localIndex) == movedIndex, "movedBody->localIndex == movedIndex")
				movedBody.localIndex = int32(awakeBodyIndex)

				if int(movedBody.islandId) != islandId {
					refreshBodyContactIndices(world, movedBody)
				}
			}

			// destroy state, no need to clone
			arrayRemoveSwap(&awakeSet_.bodyStates, awakeBodyIndex)

			body.setIndex = int32(sleepSetId)
			body.localIndex = int32(sleepBodyIndex)

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			contactKey := int(body.headContactKey)
			for contactKey != NullIndex {
				contactId := contactKey >> 1
				edgeIndex := contactKey & 1

				contact := arrayGet(world.contacts, contactId)

				assert(contact.setIndex == awakeSet, "contact->setIndex == b3_awakeSet")

				contactKey = int(contact.edges[edgeIndex].nextKey)

				if contact.colorIndex != NullIndex {
					// contact is touching and will be moved separately
					assert((contact.flags&contactTouchingFlag) != 0, "( contact->flags & b3_contactTouchingFlag ) != 0")
					continue
				}

				if edgeIndex == 0 {
					contact.encodedBodySimA = NullIndex
				} else {
					contact.encodedBodySimB = NullIndex
				}

				// the other body may still be awake, it still may go to sleep and then it will be responsible
				// for moving this contact to the disabled set.
				otherEdgeIndex := edgeIndex ^ 1
				otherBodyId := int(contact.edges[otherEdgeIndex].bodyId)
				otherBody := arrayGet(world.bodies, otherBodyId)
				if otherBody.setIndex == awakeSet {
					continue
				}

				localIndex := int(contact.localIndex)
				assert(int(awakeSet_.contactIndices[localIndex]) == contactId, "awakeSet->contactIndices.data[localIndex] == contactId")

				assert(contact.manifoldCount == 0, "contact->manifoldCount == 0")
				assert((contact.flags&contactTouchingFlag) == 0, "( contact->flags & b3_contactTouchingFlag ) == 0")

				// Move the non-touching contact to the disabled set.
				contact.setIndex = disabledSet

				// This is mandatory for validation to work correctly
				contact.localIndex = int32(len(disabledSet_.contactIndices))
				arrayPush(&disabledSet_.contactIndices, contact.contactId)

				movedLocalIndex := arrayRemoveSwap(&awakeSet_.contactIndices, localIndex)
				if movedLocalIndex != NullIndex {
					// fix moved element
					movedContactIndex := int(awakeSet_.contactIndices[localIndex])
					movedContact := arrayGet(world.contacts, movedContactIndex)
					assert(int(movedContact.localIndex) == movedLocalIndex, "movedContact->localIndex == movedLocalIndex")
					movedContact.localIndex = int32(localIndex)
				}
			}
		}
	}

	// move touching contacts to sleeping set
	// this shuffles contacts in the awake set
	{
		for i := 0; i < len(island.contacts); i++ {
			contactId := int(island.contacts[i].contactId)
			contact := arrayGet(world.contacts, contactId)
			assert(contact.setIndex == awakeSet, "contact->setIndex == b3_awakeSet")
			assert(int(contact.islandId) == islandId, "contact->islandId == islandId")
			assert(int(contact.islandIndex) == i, "contact->islandIndex == i")
			colorIndex := int(contact.colorIndex)
			assert(0 <= colorIndex && colorIndex < GraphColorCount, "0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT")

			color := &world.constraintGraph.colors[colorIndex]

			// Remove bodies from graph coloring associated with this constraint
			if colorIndex != overflowIndex {
				// might clear a bit for a static body, but this has no effect
				clearBit(&color.bodySet, uint32(contact.edges[0].bodyId))
				clearBit(&color.bodySet, uint32(contact.edges[1].bodyId))
			}

			sleepContactIndex := len(sleepSet.contactIndices)
			arrayPush(&sleepSet.contactIndices, int32(contactId))

			localIndex := int(contact.localIndex)
			if (contact.flags&simMeshContact) != 0 || colorIndex == overflowIndex {
				movedLocalIndex := arrayRemoveSwap(&color.contacts, localIndex)
				if movedLocalIndex != NullIndex {
					// fix moved element
					movedContactId := int(color.contacts[localIndex].contactId)
					movedContact := arrayGet(world.contacts, movedContactId)
					assert(int(movedContact.localIndex) == movedLocalIndex, "movedContact->localIndex == movedLocalIndex")
					movedContact.localIndex = int32(localIndex)
				}
			} else {
				movedLocalIndex := arrayRemoveSwap(&color.convexContacts, localIndex)
				if movedLocalIndex != NullIndex {
					// fix moved element
					movedContactId := int(color.convexContacts[localIndex])
					movedContact := arrayGet(world.contacts, movedContactId)
					assert(int(movedContact.localIndex) == movedLocalIndex, "movedContact->localIndex == movedLocalIndex")
					movedContact.localIndex = int32(localIndex)
				}
			}

			contact.setIndex = int32(sleepSetId)
			contact.colorIndex = NullIndex
			contact.localIndex = int32(sleepContactIndex)
			contact.encodedBodySimA = int32(sleepBodySimIndex(int(contact.encodedBodySimA)))
			contact.encodedBodySimB = int32(sleepBodySimIndex(int(contact.encodedBodySimB)))
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		for i := 0; i < len(island.joints); i++ {
			jointId := int(island.joints[i].jointId)
			joint := arrayGet(world.joints, jointId)
			assert(joint.setIndex == awakeSet, "joint->setIndex == b3_awakeSet")
			assert(int(joint.islandId) == islandId, "joint->islandId == islandId")
			assert(int(joint.islandIndex) == i, "joint->islandIndex == i")
			colorIndex := int(joint.colorIndex)
			localIndex := int(joint.localIndex)

			assert(0 <= colorIndex && colorIndex < GraphColorCount, "0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT")

			color := &world.constraintGraph.colors[colorIndex]

			awakeJointSim := arrayGet(color.jointSims, localIndex)

			if colorIndex != overflowIndex {
				// might clear a bit for a static body, but this has no effect
				clearBit(&color.bodySet, uint32(joint.edges[0].bodyId))
				clearBit(&color.bodySet, uint32(joint.edges[1].bodyId))
			}

			sleepJointIndex := len(sleepSet.jointSims)
			sleepJointSim := arrayEmplace(&sleepSet.jointSims)
			*sleepJointSim = *awakeJointSim

			movedIndex := arrayRemoveSwap(&color.jointSims, localIndex)
			if movedIndex != NullIndex {
				// fix moved element
				movedJointSim := &color.jointSims[localIndex]
				movedId := int(movedJointSim.jointId)
				movedJoint := arrayGet(world.joints, movedId)
				assert(int(movedJoint.localIndex) == movedIndex, "movedJoint->localIndex == movedIndex")
				movedJoint.localIndex = int32(localIndex)
			}

			joint.setIndex = int32(sleepSetId)
			joint.colorIndex = NullIndex
			joint.localIndex = int32(sleepJointIndex)
		}
	}

	// move island struct
	{
		assert(island.setIndex == awakeSet, "island->setIndex == b3_awakeSet")

		islandIndex := int(island.localIndex)
		sleepIsland := arrayEmplace(&sleepSet.islandSims)
		sleepIsland.islandId = int32(islandId)

		movedIslandIndex := arrayRemoveSwap(&awakeSet_.islandSims, islandIndex)
		if movedIslandIndex != NullIndex {
			// fix index on moved element
			movedIslandSim := &awakeSet_.islandSims[islandIndex]
			movedIslandId := int(movedIslandSim.islandId)
			movedIsland := arrayGet(world.islands, movedIslandId)
			assert(int(movedIsland.localIndex) == movedIslandIndex, "movedIsland->localIndex == movedIslandIndex")
			movedIsland.localIndex = int32(islandIndex)
		}

		island.setIndex = int32(sleepSetId)
		island.localIndex = 0
	}

	if int(world.splitIslandId) == islandId {
		world.splitIslandId = NullIndex
	}

	validateSolverSets(world)
}

// mergeSolverSets is b3MergeSolverSets.
// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is woke.
func mergeSolverSets(world *world, setId1, setId2 int) {
	assert(setId1 >= firstSleepingSet, "setId1 >= b3_firstSleepingSet")
	assert(setId2 >= firstSleepingSet, "setId2 >= b3_firstSleepingSet")
	set1 := arrayGet(world.solverSets, setId1)
	set2 := arrayGet(world.solverSets, setId2)

	// Move the fewest number of bodies
	if len(set1.bodySims) < len(set2.bodySims) {
		tempSet := set1
		set1 = set2
		set2 = tempSet

		tempId := setId1
		setId1 = setId2
		setId2 = tempId
	}

	// transfer bodies
	{
		bodies := world.bodies
		bodyCount := len(set2.bodySims)
		for i := 0; i < bodyCount; i++ {
			simSrc := &set2.bodySims[i]

			body := &bodies[simSrc.bodyId]
			assert(int(body.setIndex) == setId2, "body->setIndex == setId2")
			body.setIndex = int32(setId1)
			body.localIndex = int32(len(set1.bodySims))

			simDst := arrayEmplace(&set1.bodySims)
			*simDst = *simSrc
		}
	}

	// transfer contacts
	{
		contactCount := len(set2.contactIndices)
		for i := 0; i < contactCount; i++ {
			contactIndex := set2.contactIndices[i]
			contact := arrayGet(world.contacts, int(contactIndex))
			assert(int(contact.setIndex) == setId2, "contact->setIndex == setId2")
			contact.setIndex = int32(setId1)
			contact.localIndex = int32(len(set1.contactIndices))
			arrayPush(&set1.contactIndices, contactIndex)
		}
	}

	// transfer joints
	{
		jointCount := len(set2.jointSims)
		for i := 0; i < jointCount; i++ {
			jointSrc := &set2.jointSims[i]

			joint := arrayGet(world.joints, int(jointSrc.jointId))
			assert(int(joint.setIndex) == setId2, "joint->setIndex == setId2")
			joint.setIndex = int32(setId1)
			joint.localIndex = int32(len(set1.jointSims))

			jointDst := arrayEmplace(&set1.jointSims)
			*jointDst = *jointSrc
		}
	}

	// transfer islands
	{
		islandCount := len(set2.islandSims)
		for i := 0; i < islandCount; i++ {
			islandSrc := &set2.islandSims[i]
			islandId := int(islandSrc.islandId)

			island := arrayGet(world.islands, islandId)
			island.setIndex = int32(setId1)
			island.localIndex = int32(len(set1.islandSims))

			islandDst := arrayEmplace(&set1.islandSims)
			*islandDst = *islandSrc
		}
	}

	// destroy the merged set
	// Warning: need to be careful not to destroy things that got transferred, like triangle caches.
	destroySolverSet(world, setId2)

	validateSolverSets(world)
}

// transferBody is b3TransferBody.
func transferBody(world *world, targetSet, sourceSet *solverSet, body *body) {
	assert(body.headContactKey == NullIndex, "body->headContactKey == B3_NULL_INDEX")

	if targetSet == sourceSet {
		return
	}

	sourceIndex := int(body.localIndex)
	sourceSim := arrayGet(sourceSet.bodySims, sourceIndex)

	targetIndex := len(targetSet.bodySims)
	targetSim := arrayEmplace(&targetSet.bodySims)
	*targetSim = *sourceSim

	// Clear transient body flags
	body.flags &^= bodyTransientFlags
	targetSim.flags &^= (isFast | bodyTransientFlags)

	// Remove body sim from solver set that owns it
	movedIndex := arrayRemoveSwap(&sourceSet.bodySims, sourceIndex)
	if movedIndex != NullIndex {
		// Fix moved body index
		movedSim := &sourceSet.bodySims[sourceIndex]
		movedId := int(movedSim.bodyId)
		movedBody := arrayGet(world.bodies, movedId)
		assert(int(movedBody.localIndex) == movedIndex, "movedBody->localIndex == movedIndex")
		movedBody.localIndex = int32(sourceIndex)
		refreshBodyContactIndices(world, movedBody)
	}

	if sourceSet.setIndex == awakeSet {
		arrayRemoveSwap(&sourceSet.bodyStates, sourceIndex)
	} else if targetSet.setIndex == awakeSet {
		state := arrayEmplace(&targetSet.bodyStates)
		*state = identityBodyState
		state.flags = body.flags
	}

	body.setIndex = targetSet.setIndex
	body.localIndex = int32(targetIndex)
}

// transferJoint is b3TransferJoint.
func transferJoint(world *world, targetSet, sourceSet *solverSet, joint *joint) {
	if targetSet == sourceSet {
		return
	}

	localIndex := int(joint.localIndex)
	colorIndex := int(joint.colorIndex)

	// Retrieve source.
	var sourceSim *jointSim
	if sourceSet.setIndex == awakeSet {
		assert(0 <= colorIndex && colorIndex < GraphColorCount, "0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT")
		color := &world.constraintGraph.colors[colorIndex]

		sourceSim = arrayGet(color.jointSims, localIndex)
	} else {
		assert(colorIndex == NullIndex, "colorIndex == B3_NULL_INDEX")
		sourceSim = arrayGet(sourceSet.jointSims, localIndex)
	}

	// Create target and copy. Fix joint.
	if targetSet.setIndex == awakeSet {
		addJointToGraph(world, sourceSim, joint)
		joint.setIndex = awakeSet
	} else {
		joint.setIndex = targetSet.setIndex
		joint.localIndex = int32(len(targetSet.jointSims))
		joint.colorIndex = NullIndex

		targetSim := arrayEmplace(&targetSet.jointSims)
		*targetSim = *sourceSim
	}

	// Destroy source.
	if sourceSet.setIndex == awakeSet {
		removeJointFromGraph(world, int(joint.edges[0].bodyId), int(joint.edges[1].bodyId), colorIndex, localIndex)
	} else {
		movedIndex := arrayRemoveSwap(&sourceSet.jointSims, localIndex)
		if movedIndex != NullIndex {
			// fix swapped element
			movedJointSim := &sourceSet.jointSims[localIndex]
			movedId := int(movedJointSim.jointId)
			movedJoint := arrayGet(world.joints, movedId)
			movedJoint.localIndex = int32(localIndex)
		}
	}
}
