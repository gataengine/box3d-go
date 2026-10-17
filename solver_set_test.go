// Go-only test: no upstream counterpart (solver_set.c has no unit test upstream). Puts an island
// to sleep and wakes it again on a hand-built world, checking where bodies, touching and
// non-touching contacts, joints and islands end up; then checks set merging and body/joint
// transfer between sets.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// solverSetTestWorld has the static, disabled and awake sets, one static body (id 0) and
// dynamicCount awake dynamic bodies (ids 1..n) with sims, states and their own islands.
func solverSetTestWorld(dynamicCount int) *world {
	w := &world{}
	w.splitIslandId = NullIndex
	w.bodyIdPool = createIdPool()
	w.contactIdPool = createIdPool()
	w.jointIdPool = createIdPool()
	w.islandIdPool = createIdPool()
	w.solverSetIdPool = createIdPool()
	createGraph(&w.constraintGraph, 16)
	w.broadPhase.pairSet = createSet(32)
	for i := 0; i < firstSleepingSet; i++ {
		id := allocId(&w.solverSetIdPool)
		w.solverSets = append(w.solverSets, solverSet{setIndex: int32(id)})
	}

	for i := 0; i <= dynamicCount; i++ {
		id := allocId(&w.bodyIdPool)
		b := body{
			id: int32(id), islandId: NullIndex, islandIndex: NullIndex, headContactKey: NullIndex,
			headJointKey: NullIndex, bodyMoveIndex: NullIndex, headShapeId: NullIndex,
		}
		var set *solverSet
		if i == 0 {
			b.setIndex = staticSet
			b.typ = StaticBody
			set = &w.solverSets[staticSet]
		} else {
			b.setIndex = awakeSet
			b.typ = DynamicBody
			b.flags = isSpeedCapped | dynamicFlag
			set = &w.solverSets[awakeSet]
			state := arrayEmplace(&set.bodyStates)
			*state = identityBodyState
			state.flags = b.flags
		}
		b.localIndex = int32(len(set.bodySims))
		arrayPush(&set.bodySims, bodySim{bodyId: int32(id), flags: b.flags})
		w.bodies = append(w.bodies, b)
		if i > 0 {
			isl := createIsland(w, awakeSet)
			w.bodies[id].islandId = isl.islandId
			w.bodies[id].islandIndex = 0
			arrayPush(&isl.bodies, int32(id))
		}
	}
	return w
}

// solverSetTestContact creates an awake contact and pushes it on both bodies' contact lists.
func solverSetTestContact(w *world, bodyIdA, bodyIdB int32, touching bool) int {
	id := allocId(&w.contactIdPool)
	w.contacts = append(w.contacts, contact{})
	c := &w.contacts[id]
	c.contactId = int32(id)
	c.setIndex = awakeSet
	c.colorIndex = NullIndex
	c.localIndex = NullIndex
	c.islandId = NullIndex
	c.islandIndex = NullIndex
	c.edges[0].bodyId = bodyIdA
	c.edges[1].bodyId = bodyIdB
	for edge, bodyId := range [2]int32{bodyIdA, bodyIdB} {
		b := &w.bodies[bodyId]
		c.edges[edge].prevKey = NullIndex
		c.edges[edge].nextKey = b.headContactKey
		b.headContactKey = int32(id<<1 | edge)
		b.contactCount += 1
	}
	// b3ValidateSolverSets counts one broad-phase pair per contact; the key only has to be unique.
	addKey(&w.broadPhase.pairSet, shapePairKey(2*id, 2*id+1, 0))
	c.encodedBodySimA = int32(encodeBodySimIndex(&w.bodies[bodyIdA]))
	c.encodedBodySimB = int32(encodeBodySimIndex(&w.bodies[bodyIdB]))

	if touching {
		c.flags = contactTouchingFlag | simTouchingFlag
		c.manifoldCount = 1
		addContactToGraph(w, c)
		linkContact(w, c)
	} else {
		set := &w.solverSets[awakeSet]
		c.localIndex = int32(len(set.contactIndices))
		arrayPush(&set.contactIndices, int32(id))
	}
	return id
}

func solverSetTestJoint(w *world, bodyIdA, bodyIdB int32) int {
	id := allocId(&w.jointIdPool)
	w.joints = append(w.joints, joint{})
	j := &w.joints[id]
	j.jointId = int32(id)
	j.setIndex = awakeSet
	j.islandId = NullIndex
	j.islandIndex = NullIndex
	j.edges[0].bodyId = bodyIdA
	j.edges[1].bodyId = bodyIdB
	addJointToGraph(w, &jointSim{jointId: int32(id)}, j)
	linkJoint(w, j)
	return id
}

var _ = Describe("SolverSetTest", func() {
	It("SleepAndWake", func() {
		w := solverSetTestWorld(2)
		c12 := solverSetTestContact(w, 1, 2, true)
		c02 := solverSetTestContact(w, 0, 2, false)
		j12 := solverSetTestJoint(w, 1, 2)
		islandId := int(w.bodies[1].islandId)
		ensure(int(w.bodies[2].islandId) == islandId)

		awake := &w.solverSets[awakeSet]
		ensure(len(awake.islandSims) == 1)
		ensure(len(awake.contactIndices) == 1)

		// A pending split with more than one body keeps the island awake.
		w.islands[islandId].constraintRemoveCount = 1
		trySleepIsland(w, islandId)
		ensure(len(w.solverSets) == firstSleepingSet)
		w.islands[islandId].constraintRemoveCount = 0

		w.splitIslandId = int32(islandId)
		trySleepIsland(w, islandId)
		ensure(w.splitIslandId == NullIndex)
		ensure(len(w.solverSets) == firstSleepingSet+1)
		sleepSetId := firstSleepingSet
		awake = &w.solverSets[awakeSet]
		sleep := &w.solverSets[sleepSetId]
		disabled := &w.solverSets[disabledSet]
		ensure(sleep.setIndex == int32(sleepSetId))

		ensure(len(awake.bodySims) == 0 && len(awake.bodyStates) == 0)
		ensure(len(sleep.bodySims) == 2)
		for b := 1; b <= 2; b++ {
			ensure(int(w.bodies[b].setIndex) == sleepSetId)
			ensure(sleep.bodySims[w.bodies[b].localIndex].bodyId == int32(b))
		}

		// The non-touching contact went to the disabled set because body 0 is static.
		ensure(len(awake.contactIndices) == 0)
		ensure(len(disabled.contactIndices) == 1 && int(disabled.contactIndices[0]) == c02)
		ensure(w.contacts[c02].setIndex == disabledSet)
		ensure(w.contacts[c02].encodedBodySimB == NullIndex)

		// The touching contact and the joint left the graph for the sleeping set.
		ensure(len(sleep.contactIndices) == 1 && int(sleep.contactIndices[0]) == c12)
		ensure(int(w.contacts[c12].setIndex) == sleepSetId && w.contacts[c12].colorIndex == NullIndex)
		ensure(w.contacts[c12].encodedBodySimA == NullIndex && w.contacts[c12].encodedBodySimB == NullIndex)
		ensure(len(sleep.jointSims) == 1 && int(sleep.jointSims[0].jointId) == j12)
		ensure(int(w.joints[j12].setIndex) == sleepSetId && w.joints[j12].colorIndex == NullIndex)
		for i := 0; i < GraphColorCount; i++ {
			color := &w.constraintGraph.colors[i]
			ensure(len(color.convexContacts) == 0 && len(color.contacts) == 0 && len(color.jointSims) == 0)
			if i != overflowIndex {
				ensure(getBit(&color.bodySet, 1) == false && getBit(&color.bodySet, 2) == false)
			}
		}

		ensure(len(awake.islandSims) == 0)
		ensure(len(sleep.islandSims) == 1 && int(sleep.islandSims[0].islandId) == islandId)
		ensure(int(w.islands[islandId].setIndex) == sleepSetId && w.islands[islandId].localIndex == 0)

		// Wake it again.
		wakeSolverSet(w, sleepSetId)
		awake = &w.solverSets[awakeSet]
		disabled = &w.solverSets[disabledSet]
		ensure(w.solverSets[sleepSetId].setIndex == NullIndex)
		ensure(len(awake.bodySims) == 2 && len(awake.bodyStates) == 2)
		for b := 1; b <= 2; b++ {
			body := &w.bodies[b]
			ensure(body.setIndex == awakeSet)
			ensure(awake.bodySims[body.localIndex].bodyId == int32(b))
			ensure(awake.bodyStates[body.localIndex].flags == body.flags)
		}
		ensure(len(disabled.contactIndices) == 0)
		ensure(len(awake.contactIndices) == 1 && int(awake.contactIndices[0]) == c02)
		ensure(w.contacts[c02].setIndex == awakeSet && w.contacts[c02].localIndex == 0)
		ensure(w.contacts[c02].encodedBodySimB == w.bodies[2].localIndex)
		ensure(w.contacts[c12].setIndex == awakeSet && w.contacts[c12].colorIndex != NullIndex)
		ensure(w.joints[j12].setIndex == awakeSet && w.joints[j12].colorIndex != NullIndex)
		ensure(len(awake.islandSims) == 1 && w.islands[islandId].setIndex == awakeSet)

		// The sleeping set id is recycled by the next sleep.
		trySleepIsland(w, islandId)
		ensure(len(w.solverSets) == firstSleepingSet+1)
		ensure(w.solverSets[sleepSetId].setIndex == int32(sleepSetId))
	})

	It("MergeAndTransfer", func() {
		w := solverSetTestWorld(3)
		// Bodies 3 and 2 sleep in their own sets (each the last awake body sim, so no other body
		// moves); body 1 stays awake.
		island3 := int(w.bodies[3].islandId)
		island2 := int(w.bodies[2].islandId)
		trySleepIsland(w, island3)
		trySleepIsland(w, island2)
		set3 := int(w.bodies[3].setIndex)
		set2 := int(w.bodies[2].setIndex)
		ensure(set3 >= firstSleepingSet && set2 >= firstSleepingSet && set3 != set2)

		mergeSolverSets(w, set3, set2)
		// Equal body counts: the second set merges into the first.
		ensure(int(w.bodies[3].setIndex) == set3 && int(w.bodies[2].setIndex) == set3)
		ensure(w.solverSets[set2].setIndex == NullIndex)
		merged := &w.solverSets[set3]
		ensure(len(merged.bodySims) == 2 && len(merged.islandSims) == 2)
		ensure(int(w.islands[island2].setIndex) == set3 && w.islands[island2].localIndex == 1)
		ensure(w.bodies[2].localIndex == 1)

		// Transfer awake body 1 (no contacts) to the disabled set and back.
		awake := &w.solverSets[awakeSet]
		disabled := &w.solverSets[disabledSet]
		body1 := &w.bodies[1]
		ensure(body1.localIndex == 0 && len(awake.bodySims) == 1)
		body1.flags = isSpeedCapped | hadTimeOfImpact
		awake.bodySims[body1.localIndex].flags = isFast | isSpeedCapped
		transferBody(w, disabled, awake, body1)
		ensure(body1.setIndex == disabledSet && body1.localIndex == 0)
		ensure(len(awake.bodySims) == 0 && len(awake.bodyStates) == 0)
		ensure(body1.flags == 0 && disabled.bodySims[0].flags == 0)
		transferBody(w, awake, disabled, body1)
		ensure(body1.setIndex == awakeSet && len(awake.bodyStates) == 1)
		transferBody(w, awake, awake, body1)
		ensure(len(awake.bodySims) == 1)

		// Transfer a joint between awake body 1 and static body 0 out of the graph and back.
		jointId := solverSetTestJoint(w, 0, 1)
		jt := &w.joints[jointId]
		color := int(jt.colorIndex)
		ensure(color != NullIndex)
		transferJoint(w, disabled, awake, jt)
		ensure(jt.setIndex == disabledSet && jt.colorIndex == NullIndex && jt.localIndex == 0)
		ensure(len(w.constraintGraph.colors[color].jointSims) == 0)
		ensure(int(disabled.jointSims[0].jointId) == jointId)
		transferJoint(w, awake, disabled, jt)
		ensure(jt.setIndex == awakeSet && int(jt.colorIndex) == color)
		ensure(len(disabled.jointSims) == 0)

		destroySolverSet(w, set3)
		ensure(w.solverSets[set3].setIndex == NullIndex && len(w.solverSets[set3].bodySims) == 0)
	})
})
