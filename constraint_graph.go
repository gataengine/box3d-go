// Port of src/constraint_graph.h and src/constraint_graph.c from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "math"

// ---- constraint_graph.h ----

// overflowIndex is B3_OVERFLOW_INDEX.
// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
const overflowIndex = GraphColorCount - 1

// dynamicColorCount is B3_DYNAMIC_COLOR_COUNT.
// This keeps constraints involving two dynamic bodies at a lower solver priority than constraints
// involving a dynamic and static bodies. This reduces tunneling due to push through.
const dynamicColorCount = GraphColorCount - 4

// todo optimize mesh contact constraints
// They can be lumped with convex contacts and I can use the bitset event to re-link if the manifold count increases
// Could also create a group for two wide manifolds and use bitset event
// This could create ping-pong jitter so may need to pin to high water mark or introduce hysteresis somehow
//
// Dirk has the idea to do graph coloring based on manifolds. This suggests mesh contact will have manifolds
// in multiple graph colors. So each manifold with have a color and local index.
// Some concerns about this:
// - manifolds don't have a strong identity, would this affect stability/jitter?
// - this creates a lot of static graph colors and can overflow

// graphColor is b3GraphColor.
type graphColor struct {
	// This bitset is indexed by bodyId so this is over-sized to encompass static bodies
	// however I never traverse these bits or use the bit count for anything
	// This bitset is unused on the overflow color.
	bodySet bitSet

	// cache friendly arrays
	jointSims []jointSim

	convexContacts []int32
	contacts       []contactSpec

	// These are used for convex contacts
	// PORT: the void* wide constraint buffers are wideConstraintBuffer (contact_solver_wide.go);
	// the `8` fields hold them when the world runs at width 8 (wideConstraintBuffer8). The counts
	// and manifold starts do not depend on the width.
	wideConstraints     wideConstraintBuffer
	wideConstraints8    wideConstraintBuffer8
	wideConstraintCount int32

	wideMeshConstraints     wideConstraintBuffer
	wideMeshConstraints8    wideConstraintBuffer8
	wideMeshManifoldStarts  []int32
	wideMeshConstraintCount int32

	// These are used for mesh and overflow contacts
	// PORT: pointers to stack allocated arrays become slices.
	manifoldConstraints []manifoldConstraint
	contactConstraints  []contactConstraint
}

// constraintGraph is b3ConstraintGraph.
type constraintGraph struct {
	// including overflow at the end
	colors [GraphColorCount]graphColor
}

// ---- constraint_graph.c ----

// Solver using graph coloring. Islands are only used for sleep.
// High-Performance Physical Simulations on Next-Generation Architecture with Many Cores
// http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf

// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver
// body for kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body
// scatter would write to the same kinematic body from multiple threads. Even if these writes don't modify the body, they will
// cause horrible cache stalls. To make this feasible I would need a way to block these writes.

// forceOverflow is B3_FORCE_OVERFLOW.
// This is used for debugging by making all constraints be assigned to overflow.
// PORT: the #if becomes a constant condition.
const forceOverflow = 0

var graphColors = [GraphColorCount]HexColor{
	ColorRed, ColorOrange, ColorYellow, ColorLimeGreen, ColorSpringGreen,
	ColorAqua, ColorDodgerBlue, ColorBlueViolet, ColorMagenta, ColorDeepPink,
	ColorCrimson, ColorCoral, ColorGold, ColorGreenYellow, ColorMediumSeaGreen,
	ColorTurquoise, ColorDeepSkyBlue, ColorCornflowerBlue, ColorMediumSlateBlue, ColorMediumOrchid,
	ColorHotPink, ColorTomato, ColorKhaki, ColorSilver,
}

// GetGraphColor is b3GetGraphColor: get the visualization color assigned to a constraint graph
// color slot. The last index (GraphColorCount - 1) is the overflow color.
func GetGraphColor(index int) HexColor {
	assert(0 <= index && index < GraphColorCount, "0 <= index && index < B3_GRAPH_COLOR_COUNT")
	return graphColors[index]
}

// createGraph is b3CreateGraph.
func createGraph(graph *constraintGraph, bodyCapacity int) {
	// PORT: _Static_assert becomes a constant expression that fails to compile when false.
	const _ = uint(GraphColorCount - 2)                   // must have at least two constraint graph colors
	const _ = uint(overflowIndex - (GraphColorCount - 1)) // bad over flow index
	const _ = uint((GraphColorCount - 1) - overflowIndex) // bad over flow index

	*graph = constraintGraph{}

	bodyCapacity = MaxInt(bodyCapacity, 8)

	// Initialize graph color bit set.
	// No bitset for overflow color.
	for i := 0; i < overflowIndex; i++ {
		color := &graph.colors[i]
		color.bodySet = createBitSet(uint32(bodyCapacity))
		setBitCountAndClear(&color.bodySet, uint32(bodyCapacity))
	}
}

// destroyGraph is b3DestroyGraph.
func destroyGraph(graph *constraintGraph) {
	for i := 0; i < GraphColorCount; i++ {
		color := &graph.colors[i]

		// The bit set should never be used on the overflow color
		assert(i != overflowIndex || color.bodySet.bits == nil, "i != B3_OVERFLOW_INDEX || color->bodySet.bits == NULL")

		destroyBitSet(&color.bodySet)

		arrayDestroy(&color.convexContacts)
		arrayDestroy(&color.contacts)
		arrayDestroy(&color.jointSims)
	}
}

// addContactToGraph is b3AddContactToGraph.
// Contacts are always created as non-touching. They get cloned into the constraint
// graph once they are found to be touching.
func addContactToGraph(world *world, contact *contact) {
	assert(contact.manifoldCount > 0, "contact->manifoldCount > 0")
	assert(contact.flags&contactTouchingFlag != 0, "contact->flags & b3_contactTouchingFlag")

	graph := &world.constraintGraph
	colorIndex := overflowIndex

	bodyIdA := int(contact.edges[0].bodyId)
	bodyIdB := int(contact.edges[1].bodyId)
	bodyA := arrayGet(world.bodies, bodyIdA)
	bodyB := arrayGet(world.bodies, bodyIdB)

	typeA := bodyA.typ
	typeB := bodyB.typ
	assert(typeA == DynamicBody || typeB == DynamicBody, "typeA == b3_dynamicBody || typeB == b3_dynamicBody")

	if forceOverflow == 0 {
		if typeA == DynamicBody && typeB == DynamicBody {
			// Dynamic constraint colors cannot encroach on colors reserved for static constraints
			for i := 0; i < dynamicColorCount; i++ {
				color := &graph.colors[i]
				if getBit(&color.bodySet, uint32(bodyIdA)) || getBit(&color.bodySet, uint32(bodyIdB)) {
					continue
				}

				setBitGrow(&color.bodySet, uint32(bodyIdA))
				setBitGrow(&color.bodySet, uint32(bodyIdB))
				colorIndex = i
				break
			}
		} else if typeA == DynamicBody {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for i := overflowIndex - 1; i >= 1; i-- {
				color := &graph.colors[i]
				if getBit(&color.bodySet, uint32(bodyIdA)) {
					continue
				}

				setBitGrow(&color.bodySet, uint32(bodyIdA))
				colorIndex = i
				break
			}
		} else if typeB == DynamicBody {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for i := overflowIndex - 1; i >= 1; i-- {
				color := &graph.colors[i]
				if getBit(&color.bodySet, uint32(bodyIdB)) {
					continue
				}

				setBitGrow(&color.bodySet, uint32(bodyIdB))
				colorIndex = i
				break
			}
		}
	}

	isScalar := (contact.flags&simMeshContact) != 0 || colorIndex == overflowIndex

	color := &graph.colors[colorIndex]
	contact.colorIndex = int32(colorIndex)
	if isScalar {
		contact.localIndex = int32(len(color.contacts))
	} else {
		contact.localIndex = int32(len(color.convexContacts))
	}
	contact.encodedBodySimA = int32(encodeBodySimIndex(bodyA))
	contact.encodedBodySimB = int32(encodeBodySimIndex(bodyB))

	if isScalar {
		assert(contact.manifoldCount < math.MaxUint16, "contact->manifoldCount < UINT16_MAX")
		spec := contactSpec{
			contactId:     contact.contactId,
			manifoldStart: 0,
			manifoldCount: uint16(contact.manifoldCount),
		}
		arrayPush(&color.contacts, spec)
	} else {
		arrayPush(&color.convexContacts, contact.contactId)
	}
}

// removeContactFromGraph is b3RemoveContactFromGraph.
func removeContactFromGraph(world *world, bodyIdA, bodyIdB, colorIndex, localIndex int, meshContact bool) {
	graph := &world.constraintGraph

	assert(0 <= colorIndex && colorIndex < GraphColorCount, "0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT")
	color := &graph.colors[colorIndex]

	if colorIndex != overflowIndex {
		// This might clear a bit for a static body, but this has no effect
		clearBit(&color.bodySet, uint32(bodyIdA))
		clearBit(&color.bodySet, uint32(bodyIdB))
	}

	if meshContact || colorIndex == overflowIndex {
		movedIndex := arrayRemoveSwap(&color.contacts, localIndex)
		if movedIndex != NullIndex {
			// Fix index on swapped contact
			movedContactId := int(color.contacts[localIndex].contactId)
			movedContact := arrayGet(world.contacts, movedContactId)
			assert(movedContact.setIndex == awakeSet, "movedContact->setIndex == b3_awakeSet")
			assert(int(movedContact.colorIndex) == colorIndex, "movedContact->colorIndex == colorIndex")
			assert(int(movedContact.localIndex) == movedIndex, "movedContact->localIndex == movedIndex")
			movedContact.localIndex = int32(localIndex)
		}
	} else {
		movedIndex := arrayRemoveSwap(&color.convexContacts, localIndex)
		if movedIndex != NullIndex {
			// Fix index on swapped contact
			movedContactId := int(color.convexContacts[localIndex])
			movedContact := arrayGet(world.contacts, movedContactId)
			assert(movedContact.setIndex == awakeSet, "movedContact->setIndex == b3_awakeSet")
			assert(int(movedContact.colorIndex) == colorIndex, "movedContact->colorIndex == colorIndex")
			assert(int(movedContact.localIndex) == movedIndex, "movedContact->localIndex == movedIndex")
			assert((movedContact.flags&simMeshContact) == 0, "( movedContact->flags & b3_simMeshContact ) == 0")
			movedContact.localIndex = int32(localIndex)
		}
	}
}

// assignJointColor is b3AssignJointColor.
func assignJointColor(graph *constraintGraph, bodyIdA, bodyIdB int, typeA, typeB BodyType) int {
	assert(typeA == DynamicBody || typeB == DynamicBody, "typeA == b3_dynamicBody || typeB == b3_dynamicBody")

	if forceOverflow == 0 {
		if typeA == DynamicBody && typeB == DynamicBody {
			// Dynamic constraint colors cannot encroach on colors reserved for static constraints
			for i := 0; i < dynamicColorCount; i++ {
				color := &graph.colors[i]
				if getBit(&color.bodySet, uint32(bodyIdA)) || getBit(&color.bodySet, uint32(bodyIdB)) {
					continue
				}

				setBitGrow(&color.bodySet, uint32(bodyIdA))
				setBitGrow(&color.bodySet, uint32(bodyIdB))
				return i
			}
		} else if typeA == DynamicBody {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for i := overflowIndex - 1; i >= 1; i-- {
				color := &graph.colors[i]
				if getBit(&color.bodySet, uint32(bodyIdA)) {
					continue
				}

				setBitGrow(&color.bodySet, uint32(bodyIdA))
				return i
			}
		} else if typeB == DynamicBody {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for i := overflowIndex - 1; i >= 1; i-- {
				color := &graph.colors[i]
				if getBit(&color.bodySet, uint32(bodyIdB)) {
					continue
				}

				setBitGrow(&color.bodySet, uint32(bodyIdB))
				return i
			}
		}
	}

	return overflowIndex
}

// createJointInGraph is b3CreateJointInGraph.
func createJointInGraph(world *world, joint *joint) *jointSim {
	graph := &world.constraintGraph

	bodyIdA := int(joint.edges[0].bodyId)
	bodyIdB := int(joint.edges[1].bodyId)
	bodyA := arrayGet(world.bodies, bodyIdA)
	bodyB := arrayGet(world.bodies, bodyIdB)

	colorIndex := assignJointColor(graph, bodyIdA, bodyIdB, bodyA.typ, bodyB.typ)

	jointSims := &graph.colors[colorIndex].jointSims
	jointSim := arrayEmplace(jointSims)
	// PORT: memset of the new element; the local name shadows the type, so zero it by slice.
	arrayMemZero((*jointSims)[len(*jointSims)-1:])

	joint.colorIndex = int32(colorIndex)
	joint.localIndex = int32(len(graph.colors[colorIndex].jointSims) - 1)
	return jointSim
}

// addJointToGraph is b3AddJointToGraph.
func addJointToGraph(world *world, jointSim *jointSim, joint *joint) {
	jointDst := createJointInGraph(world, joint)
	*jointDst = *jointSim
}

// removeJointFromGraph is b3RemoveJointFromGraph.
func removeJointFromGraph(world *world, bodyIdA, bodyIdB, colorIndex, localIndex int) {
	graph := &world.constraintGraph

	assert(0 <= colorIndex && colorIndex < GraphColorCount, "0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT")
	color := &graph.colors[colorIndex]

	if colorIndex != overflowIndex {
		// May clear static bodies, no effect
		clearBit(&color.bodySet, uint32(bodyIdA))
		clearBit(&color.bodySet, uint32(bodyIdB))
	}

	movedIndex := arrayRemoveSwap(&color.jointSims, localIndex)
	if movedIndex != NullIndex {
		// Fix moved joint
		movedJointSim := &color.jointSims[localIndex]
		movedId := int(movedJointSim.jointId)
		movedJoint := arrayGet(world.joints, movedId)
		assert(movedJoint.setIndex == awakeSet, "movedJoint->setIndex == b3_awakeSet")
		assert(int(movedJoint.colorIndex) == colorIndex, "movedJoint->colorIndex == colorIndex")
		assert(int(movedJoint.localIndex) == movedIndex, "movedJoint->localIndex == movedIndex")
		movedJoint.localIndex = int32(localIndex)
	}
}
