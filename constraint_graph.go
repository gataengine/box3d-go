// Port of src/constraint_graph.h (and later src/constraint_graph.c) from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- constraint_graph.h ----

// PORT: header types only; constraint_graph.c lands in milestone 3.

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
	// PORT: void* wide constraint buffers; their Go type is decided with contact_solver_wide.go
	// in milestone 3.
	wideConstraints     any
	wideConstraintCount int32

	wideMeshConstraints     any
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
