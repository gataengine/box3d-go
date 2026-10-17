// Go-only test: no upstream counterpart (island.c has no unit test upstream). Checks island
// merging through contacts and joints, unlinking with the swap fix-up, union-find splitting and
// the per-island contact/joint assignment, with validateIsland on under box3d_validate.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"slices"

	. "github.com/onsi/ginkgo/v2"
)

// islandTestWorld has one static body (id 0) and dynamicCount awake dynamic bodies (ids 1..n),
// each dynamic body alone in its own awake island as body creation leaves it.
func islandTestWorld(dynamicCount int) *world {
	w := &world{}
	w.splitIslandId = NullIndex
	w.bodyIdPool = createIdPool()
	w.contactIdPool = createIdPool()
	w.jointIdPool = createIdPool()
	w.islandIdPool = createIdPool()
	w.solverSets = make([]solverSet, firstSleepingSet)
	for i := range w.solverSets {
		w.solverSets[i].setIndex = int32(i)
	}

	for i := 0; i <= dynamicCount; i++ {
		id := allocId(&w.bodyIdPool)
		b := body{id: int32(id), islandId: NullIndex, islandIndex: NullIndex}
		if i == 0 {
			b.setIndex = staticSet
			b.typ = StaticBody
		} else {
			b.setIndex = awakeSet
			b.typ = DynamicBody
		}
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

func islandTestLinkContact(w *world, bodyIdA, bodyIdB int32) int {
	id := allocId(&w.contactIdPool)
	w.contacts = append(w.contacts, contact{})
	c := &w.contacts[id]
	c.contactId = int32(id)
	c.setIndex = awakeSet
	c.flags = contactTouchingFlag
	c.islandId = NullIndex
	c.islandIndex = NullIndex
	c.edges[0].bodyId = bodyIdA
	c.edges[1].bodyId = bodyIdB
	linkContact(w, c)
	return id
}

func islandTestLinkJoint(w *world, bodyIdA, bodyIdB int32) int {
	id := allocId(&w.jointIdPool)
	w.joints = append(w.joints, joint{})
	j := &w.joints[id]
	j.jointId = int32(id)
	j.setIndex = awakeSet
	j.islandId = NullIndex
	j.islandIndex = NullIndex
	j.edges[0].bodyId = bodyIdA
	j.edges[1].bodyId = bodyIdB
	linkJoint(w, j)
	return id
}

func islandTestBodies(w *world, islandId int32) []int32 {
	ids := slices.Clone(w.islands[islandId].bodies)
	slices.Sort(ids)
	return ids
}

var _ = Describe("IslandTest", func() {
	It("LinkAndSplit", func() {
		w := islandTestWorld(5)
		awake := &w.solverSets[awakeSet]
		ensure(len(awake.islandSims) == 5)

		c12 := islandTestLinkContact(w, 1, 2)
		c23 := islandTestLinkContact(w, 2, 3)
		c34 := islandTestLinkContact(w, 3, 4)
		c04 := islandTestLinkContact(w, 0, 4)
		j12 := islandTestLinkJoint(w, 1, 2)
		j05 := islandTestLinkJoint(w, 0, 5)

		// Bodies 1..4 merged into one island; body 5 stays alone with its static joint.
		ensure(len(awake.islandSims) == 2)
		root := w.bodies[1].islandId
		for b := 1; b <= 4; b++ {
			ensure(w.bodies[b].islandId == root)
		}
		ensure(slices.Equal(islandTestBodies(w, root), []int32{1, 2, 3, 4}))
		ensure(len(w.islands[root].contacts) == 4)
		ensure(len(w.islands[root].joints) == 1)
		ensure(w.joints[j05].islandId == w.bodies[5].islandId)
		ensure(w.contacts[c04].islandId == root)
		validateIsland(w, int(root))

		// Merging keeps the bigger island and recycles the smaller one's id.
		for i := range w.islands {
			isl := &w.islands[i]
			if isl.setIndex != NullIndex {
				ensure(int(isl.localIndex) < len(awake.islandSims))
				ensure(awake.islandSims[isl.localIndex].islandId == isl.islandId)
			}
		}

		// Unlinking c23 swaps the last contact into its slot.
		removeIndex := w.contacts[c23].islandIndex
		unlinkContact(w, &w.contacts[c23])
		ensure(w.contacts[c23].islandId == NullIndex && w.contacts[c23].islandIndex == NullIndex)
		ensure(w.islands[root].constraintRemoveCount == 1)
		moved := w.islands[root].contacts[removeIndex].contactId
		ensure(w.contacts[moved].islandIndex == removeIndex)

		// Split: {1,2} with c12 and j12, {3,4} with c34 and the static contact c04.
		w.splitIslandId = root
		splitIslandTask(w)
		ensure(w.splitIslandId == NullIndex)
		ensure(len(awake.islandSims) == 3)
		islandA := w.bodies[1].islandId
		islandB := w.bodies[3].islandId
		ensure(islandA != islandB)
		ensure(w.islands[root].setIndex == NullIndex || root == islandA || root == islandB)
		ensure(slices.Equal(islandTestBodies(w, islandA), []int32{1, 2}))
		ensure(slices.Equal(islandTestBodies(w, islandB), []int32{3, 4}))
		ensure(w.contacts[c12].islandId == islandA)
		ensure(w.joints[j12].islandId == islandA)
		ensure(w.contacts[c34].islandId == islandB)
		ensure(w.contacts[c04].islandId == islandB)
		ensure(len(w.islands[islandA].contacts) == 1 && len(w.islands[islandA].joints) == 1)
		ensure(len(w.islands[islandB].contacts) == 2 && len(w.islands[islandB].joints) == 0)
		ensure(w.islands[islandA].constraintRemoveCount == 0)
		validateIsland(w, int(islandA))
		validateIsland(w, int(islandB))
		ensure(w.stack.entryCount == 0)

		// A split of a still connected island only resets the remove count.
		unlinkJoint(w, &w.joints[j12])
		ensure(w.islands[islandA].constraintRemoveCount == 1)
		islandTestLinkJoint(w, 1, 2)
		splitIsland(w, int(islandA))
		ensure(w.islands[islandA].constraintRemoveCount == 0)
		ensure(len(awake.islandSims) == 3)
		ensure(w.stack.entryCount == 0)

		// Unlinking a joint without an island is a no-op.
		unlinkJoint(w, &joint{islandId: NullIndex})
	})
})
