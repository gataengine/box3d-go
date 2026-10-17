// Go-only test: no upstream counterpart (constraint_graph.c has no unit test upstream). Checks
// graph coloring of dynamic/static contacts and joints, the overflow color, mesh contacts going
// to the scalar list, and the local index fix-up on swap removal.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// graphTestWorld has one static body (id 0) and dynamicCount awake dynamic bodies (ids 1..n).
func graphTestWorld(dynamicCount int) *world {
	w := &world{}
	createGraph(&w.constraintGraph, 4)
	w.bodies = make([]body, dynamicCount+1)
	w.bodies[0] = body{setIndex: staticSet, localIndex: 0, typ: StaticBody, id: 0}
	for i := 1; i <= dynamicCount; i++ {
		w.bodies[i] = body{setIndex: awakeSet, localIndex: int32(i - 1), typ: DynamicBody, id: int32(i)}
	}
	return w
}

func graphTestAddContact(w *world, bodyIdA, bodyIdB int, flags uint32) *contact {
	id := len(w.contacts)
	w.contacts = append(w.contacts, contact{})
	c := &w.contacts[id]
	c.contactId = int32(id)
	c.setIndex = awakeSet
	c.manifoldCount = 1
	c.flags = contactTouchingFlag | flags
	c.edges[0].bodyId = int32(bodyIdA)
	c.edges[1].bodyId = int32(bodyIdB)
	addContactToGraph(w, c)
	return c
}

var _ = Describe("ConstraintGraphTest", func() {
	It("ContactColoring", func() {
		w := graphTestWorld(40)
		graph := &w.constraintGraph
		ensure(graph.colors[overflowIndex].bodySet.bits == nil)

		// A chain of dynamic pairs: 1-2 takes color 0, 2-3 conflicts and takes color 1, 3-4 color 0.
		c12 := graphTestAddContact(w, 1, 2, 0)
		c23 := graphTestAddContact(w, 2, 3, 0)
		c34 := graphTestAddContact(w, 3, 4, 0)
		ensure(c12.colorIndex == 0 && c12.localIndex == 0)
		ensure(c23.colorIndex == 1 && c23.localIndex == 0)
		ensure(c34.colorIndex == 0 && c34.localIndex == 1)
		ensure(c12.encodedBodySimA == 0 && c12.encodedBodySimB == 1)

		// Static contacts fill colors from the end (overflow - 1 downward).
		s1 := graphTestAddContact(w, 0, 1, 0)
		s1b := graphTestAddContact(w, 1, 0, 0)
		ensure(s1.colorIndex == overflowIndex-1)
		ensure(s1b.colorIndex == overflowIndex-2)
		ensure(s1.encodedBodySimA == -2)

		// Mesh contacts go to the scalar contact list of their color.
		m := graphTestAddContact(w, 0, 5, simMeshContact)
		ensure(m.colorIndex == overflowIndex-1)
		ensure(m.localIndex == 0)
		ensure(len(graph.colors[overflowIndex-1].contacts) == 1)
		ensure(graph.colors[overflowIndex-1].contacts[0].contactId == m.contactId)
		ensure(len(graph.colors[overflowIndex-1].convexContacts) == 1)

		// Body 6 against many dynamic bodies exhausts the dynamic colors and overflows.
		var last *contact
		for i := 0; i < dynamicColorCount+2; i++ {
			last = graphTestAddContact(w, 6, 7+i, 0)
		}
		ensure(last.colorIndex == overflowIndex)
		ensure(len(graph.colors[overflowIndex].contacts) == 2)

		// Static contacts on body 8 can use colors 1..overflow-1 but never 0.
		for i := 0; i < overflowIndex+1; i++ {
			graphTestAddContact(w, 0, 8, 0)
		}
		ensure(len(graph.colors[0].convexContacts) >= 2)
		for _, id := range graph.colors[0].convexContacts {
			c := &w.contacts[id]
			ensure(c.edges[0].bodyId != 0 && c.edges[1].bodyId != 0)
		}

		// Removing c12 swaps the last color 0 contact (6-7) into local index 0 and clears the body bits.
		removeContactFromGraph(w, 1, 2, int(c12.colorIndex), int(c12.localIndex), false)
		ensure(len(graph.colors[0].convexContacts) >= 1)
		ensure(graph.colors[0].convexContacts[0] != c12.contactId)
		moved := &w.contacts[graph.colors[0].convexContacts[0]]
		ensure(moved.localIndex == 0)
		ensure(getBit(&graph.colors[0].bodySet, 1) == false)
		ensure(getBit(&graph.colors[0].bodySet, 2) == false)

		// Overflow removal keeps the overflow color without a bit set.
		overflow := &graph.colors[overflowIndex]
		first := &w.contacts[overflow.contacts[0].contactId]
		removeContactFromGraph(w, int(first.edges[0].bodyId), int(first.edges[1].bodyId), overflowIndex, 0, false)
		ensure(len(overflow.contacts) == 4)
		ensure(w.contacts[overflow.contacts[0].contactId].localIndex == 0)
		ensure(overflow.bodySet.bits == nil)

		destroyGraph(graph)
	})

	It("JointColoring", func() {
		w := graphTestWorld(4)
		graph := &w.constraintGraph
		w.joints = make([]joint, 3)
		pairs := [3][2]int32{{1, 2}, {2, 3}, {0, 3}}
		for i := range w.joints {
			j := &w.joints[i]
			j.jointId = int32(i)
			j.setIndex = awakeSet
			j.edges[0].bodyId = pairs[i][0]
			j.edges[1].bodyId = pairs[i][1]
			sim := jointSim{jointId: int32(i)}
			addJointToGraph(w, &sim, j)
		}
		ensure(w.joints[0].colorIndex == 0 && w.joints[0].localIndex == 0)
		ensure(w.joints[1].colorIndex == 1 && w.joints[1].localIndex == 0)
		ensure(w.joints[2].colorIndex == overflowIndex-1)
		ensure(graph.colors[1].jointSims[0].jointId == 1)

		// Add a second joint into color 0 then remove the first to check the swap fix-up.
		w.joints = append(w.joints, joint{jointId: 3, setIndex: awakeSet})
		j3 := &w.joints[3]
		j3.edges[0].bodyId = 3
		j3.edges[1].bodyId = 4
		addJointToGraph(w, &jointSim{jointId: 3}, j3)
		ensure(j3.colorIndex == 0 && j3.localIndex == 1)
		removeJointFromGraph(w, 1, 2, 0, 0)
		ensure(len(graph.colors[0].jointSims) == 1)
		ensure(w.joints[3].localIndex == 0)

		destroyGraph(graph)
	})
})
