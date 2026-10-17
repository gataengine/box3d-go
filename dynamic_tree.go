// Port of src/dynamic_tree.h and src/dynamic_tree.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"math"
	"os"
	"unsafe"
)

// ---- dynamic_tree.h ----

// PORT: the traversal stacks (int stack[B3_TREE_STACK_SIZE]) are fixed int32 arrays.
const treeStackSize = 512

// Used to mark a node as being moved such that pairs need to be generated and the tree
// may need to be rebuilt. This mark comes from a few sources:
// - updating the transform and AABB in the solver from movement
// - a joint disabling collision
// - creating a proxy
// - setting the transform on a body

const (
	movedNode     uint32 = 1 << 30
	leafNode      uint32 = 1 << 31
	nodeIndexMask uint32 = 0xFFFFFFFF &^ (movedNode | leafNode)

	// Used to indicate empty nodes. It also passes as a leaf so that internal node
	// processing will naturally skip it.
	emptyNode uint32 = nodeIndexMask | leafNode

	rootNode = 0
)

func isLeaf(node *TreeNode) bool {
	return (node.FlagIndex & leafNode) == leafNode
}

func isNodeMoved(node *TreeNode) bool {
	return (node.FlagIndex & movedNode) == movedNode
}

func isEmptyNode(node *TreeNode) bool {
	return node.FlagIndex == emptyNode
}

func getLeftChild(node *TreeNode) int {
	return int(node.FlagIndex & nodeIndexMask)
}

func getProxyId(node *TreeNode) int {
	return int(node.FlagIndex & nodeIndexMask)
}

func getRootPair(nodes []TreeNode) int {
	root := &nodes[rootNode]
	if isLeaf(root) {
		return rootNode
	}
	return getLeftChild(root)
}

func makeEmptyNode() TreeNode {
	return TreeNode{
		AABB: AABB{
			LowerBound: Vec3{inf32, inf32, inf32},
			UpperBound: Vec3{-inf32, -inf32, -inf32},
		},
		FlagIndex: emptyNode,
		Height:    0,
	}
}

// treeView is b3TreeView.
// PORT: the node and proxy pointers become slices; a nil proxies slice is the NULL view.
type treeView struct {
	nodes   []TreeNode
	proxies []TreeProxy
}

func makeTreeView(tree *DynamicTree) treeView {
	view := treeView{
		nodes:   tree.Nodes,
		proxies: tree.Proxies,
	}
	return view
}

// treeView.query, treeView.queryClosest, treeView.rayCast and treeView.boxCast are implemented
// below with dynamic_tree.c.

func hasTreeMoved(tree *DynamicTree) bool {
	return isNodeMoved(&tree.Nodes[rootNode])
}

func needsRebuild(tree *DynamicTree) bool {
	return isNodeMoved(&tree.Nodes[rootNode]) || tree.DfsOrdered == false
}

// createTreeProxyInternal, moveProxyInternal, markProxyMovedSerial, markProxyMoved,
// clearMoved, gatherMovedProxies and refit are implemented below with dynamic_tree.c.

// ---- dynamic_tree.c ----

// inf32 is INFINITY.
var inf32 = float32(math.Inf(1))

// PORT: _Static_assert( sizeof( b3TreeNode ) == 32 ) and sizeof( b3TreeProxy ) == 24 as
// overflowing constant pairs.
var (
	_ = [unsafe.Sizeof(TreeNode{}) - 32]struct{}{}
	_ = [32 - unsafe.Sizeof(TreeNode{})]struct{}{}
	_ = [unsafe.Sizeof(TreeProxy{}) - 24]struct{}{}
	_ = [24 - unsafe.Sizeof(TreeProxy{})]struct{}{}
)

// The free sibling pair list is chained through the parent index of their first node. A pair comes from the
// free list when there is one and from the bump pointer otherwise. A rebuild empties the free list, so
// allocations can just bump.
func allocateSiblingPair(tree *DynamicTree) int {
	// Allocate from free list.
	if tree.PairFreeList != NullIndex {
		pair := int(tree.PairFreeList)
		tree.PairFreeList = tree.Parents[pair]
		return pair
	}

	// Extend the bump allocator if needed.
	if tree.NodeEnd+2 > tree.NodeCapacity {
		oldCapacity := tree.NodeCapacity
		newCapacity := oldCapacity + (oldCapacity >> 1)
		newCapacity += newCapacity & 1
		// PORT: B3_GROW_ZERO is make + copy.
		nodes := make([]TreeNode, newCapacity)
		copy(nodes, tree.Nodes[:oldCapacity])
		tree.Nodes = nodes
		parents := make([]int32, newCapacity)
		copy(parents, tree.Parents[:oldCapacity])
		tree.Parents = parents
		tree.NodeCapacity = newCapacity

		// The spare has to match, the rebuild allocates it again.
		tree.SwapNodes = nil
	}

	pair := int(tree.NodeEnd)
	tree.NodeEnd += 2
	return pair
}

func freePair(tree *DynamicTree, pair int) {
	assert((pair&1) == 0 && 2 <= pair && pair < int(tree.NodeEnd), "( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd")
	tree.Nodes[pair] = makeEmptyNode()
	tree.Nodes[pair+1] = makeEmptyNode()
	tree.Parents[pair] = tree.PairFreeList
	tree.Parents[pair+1] = NullIndex
	tree.PairFreeList = int32(pair)
}

// CreateDynamicTree is b3DynamicTree_Create: constructing the tree initializes the node pool.
// PORT: a free function (no receiver yet); the other b3DynamicTree_* functions are methods.
func CreateDynamicTree(proxyCapacity int) DynamicTree {
	capacity := MaxInt(proxyCapacity, 16)

	// PORT: the memset for deterministic serialization is the Go zero value.
	var tree DynamicTree

	tree.Version = DynamicTreeVersion

	// A tree of n proxies has 2n - 1 nodes plus the empty node beside the root
	tree.NodeCapacity = int32(2 * capacity)
	tree.Nodes = make([]TreeNode, tree.NodeCapacity)
	tree.Parents = make([]int32, tree.NodeCapacity)
	tree.PairFreeList = NullIndex

	// The root and the empty node always exist
	tree.Nodes[rootNode] = makeEmptyNode()
	tree.Nodes[rootNode+1] = makeEmptyNode()
	tree.Parents[rootNode] = NullIndex
	tree.Parents[rootNode+1] = NullIndex

	// This is the bump index.
	tree.NodeEnd = 2
	tree.DfsOrdered = true

	tree.ProxyCapacity = int32(capacity)
	tree.ProxyCount = 0
	tree.Proxies = make([]TreeProxy, tree.ProxyCapacity)

	// Build a linked list for the free list.
	for i := int32(0); i < tree.ProxyCapacity-1; i++ {
		tree.Proxies[i].Node = NullIndex
		tree.Proxies[i].Next = i + 1
	}

	tree.Proxies[tree.ProxyCapacity-1].Node = NullIndex
	tree.Proxies[tree.ProxyCapacity-1].Next = NullIndex
	tree.ProxyFreeList = 0

	tree.SwapNodes = nil
	tree.LeafIndices = nil
	tree.LeafNodes = nil
	tree.LeafBoxes = nil
	tree.LeafCenters = nil
	tree.BinIndices = nil
	tree.RebuildCapacity = 0

	return tree
}

// Destroy is b3DynamicTree_Destroy: destroy the tree, freeing the node pool.
func (tree *DynamicTree) Destroy() {
	// PORT: the GC owns the arrays.
	*tree = DynamicTree{}
}

// Allocate a proxy from the pool. Grow the pool if necessary.
func allocateProxy(tree *DynamicTree) int {
	// Expand the pool as needed.
	if tree.ProxyFreeList == NullIndex {
		assert(tree.ProxyCount == tree.ProxyCapacity, "tree->proxyCount == tree->proxyCapacity")

		// The free list is empty. Rebuild a bigger pool.
		// A restored pool can have capacity 1, so guarantee strict growth.
		oldCapacity := tree.ProxyCapacity
		tree.ProxyCapacity += int32(MaxInt(int(oldCapacity>>1), 1))
		proxies := make([]TreeProxy, tree.ProxyCapacity)
		copy(proxies, tree.Proxies[:oldCapacity])
		tree.Proxies = proxies

		// Build a linked list for the free list.
		for i := oldCapacity; i < tree.ProxyCapacity-1; i++ {
			tree.Proxies[i].Node = NullIndex
			tree.Proxies[i].Next = i + 1
		}

		tree.Proxies[tree.ProxyCapacity-1].Node = NullIndex
		tree.Proxies[tree.ProxyCapacity-1].Next = NullIndex
		tree.ProxyFreeList = oldCapacity
	}

	// Peel a proxy off the free list.
	proxyIndex := tree.ProxyFreeList
	tree.ProxyFreeList = tree.Proxies[proxyIndex].Next
	tree.Proxies[proxyIndex] = TreeProxy{}
	tree.Proxies[proxyIndex].Node = NullIndex
	tree.Proxies[proxyIndex].Next = NullIndex
	tree.ProxyCount++
	return int(proxyIndex)
}

// Return a proxy to the pool.
func freeProxy(tree *DynamicTree, proxyId int) {
	assert(0 <= proxyId && proxyId < int(tree.ProxyCapacity), "0 <= proxyId && proxyId < tree->proxyCapacity")
	assert(0 < tree.ProxyCount, "0 < tree->proxyCount")
	tree.Proxies[proxyId].Node = NullIndex
	tree.Proxies[proxyId].Next = tree.ProxyFreeList
	tree.ProxyFreeList = int32(proxyId)
	tree.ProxyCount--
}

func getNodeHeight(node *TreeNode) int {
	if isLeaf(node) {
		return 0
	}
	return int(node.Height)
}

// The internal node above a children pair
func makeInternalNode(nodes []TreeNode, pair int) TreeNode {
	c1 := &nodes[pair]
	c2 := &nodes[pair+1]

	var node TreeNode
	storeAABBV(&node.AABB, unionPairV(nodes[pair:pair+2]), true)
	node.FlagIndex = uint32(pair) | ((c1.FlagIndex | c2.FlagIndex) & movedNode)
	node.Height = int32(1 + MaxInt(getNodeHeight(c1), getNodeHeight(c2)))
	return node
}

func makeLeafNode(aabb AABB, proxyId int, userData uint64, moved bool) TreeNode {
	var node TreeNode
	node.AABB = aabb
	node.FlagIndex = uint32(proxyId) | leafNode
	if moved {
		node.FlagIndex |= movedNode
	}
	// PORT: node.shapeIndex is the Height field (union).
	node.Height = int32(uint32(userData))
	return node
}

// A node landed at a new index, so tell what hangs below it
func linkChildren(tree *DynamicTree, nodeIndex int) {
	node := &tree.Nodes[nodeIndex]
	if isLeaf(node) {
		tree.Proxies[getProxyId(node)].Node = int32(nodeIndex)
	} else {
		pair := getLeftChild(node)
		tree.Parents[pair] = int32(nodeIndex)
		tree.Parents[pair+1] = int32(nodeIndex)
	}
}

// The sweep refit visits indices from high to low, so it needs every child above its parent
func isNodeOrdered(nodes []TreeNode, nodeIndex int) bool {
	node := &nodes[nodeIndex]
	return isLeaf(node) || nodeIndex < getLeftChild(node)
}

// Greedy algorithm for sibling selection using the SAH
// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//
//	a: B is a leaf : A-(E-(B,D), C)
//	b: B is an internal node: A-(B{D},C)
//
// 3: associate D with C
//
//	a: C is a leaf : A-(B, E-(C,D))
//	b: C is an internal node: A-(B, C{D})
//
// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
// cost of sibling H = area(union(H, D)) + increased area of ancestors

// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
// case1: D becomes a sibling of B
// case2: D becomes a descendant of B along with a new internal node of area(D).
func findBestSibling(tree *DynamicTree, boxD AABB) int {
	centerD := boxD.Center()
	areaD := perimeter(boxD)

	nodes := tree.Nodes
	nodeIndex := rootNode

	if isLeaf(&nodes[nodeIndex]) {
		return nodeIndex
	}

	rootBox := nodes[nodeIndex].AABB

	// Area of current node
	areaBase := perimeter(rootBox)

	// Area of inflated node
	directCost := perimeter(rootBox.Union(boxD))
	inheritedCost := float32(0.0)

	bestSibling := nodeIndex
	bestCost := directCost

	// Descend the tree, following a single greedy path.
	for {
		child1 := getLeftChild(&nodes[nodeIndex])
		child2 := child1 + 1

		// Cost of creating a new parent for this node and the new leaf
		cost := directCost + inheritedCost

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if cost < bestCost {
			bestSibling = nodeIndex
			bestCost = cost
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase

		leaf1 := isLeaf(&nodes[child1])
		leaf2 := isLeaf(&nodes[child2])

		// Cost of descending into child 1
		lowerCost1 := fltMax
		box1 := nodes[child1].AABB
		directCost1 := perimeter(box1.Union(boxD))
		area1 := float32(0.0)
		if leaf1 {
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			cost1 := directCost1 + inheritedCost
			if cost1 < bestCost {
				bestSibling = child1
				bestCost = cost1
			}
		} else {
			// Child 1 is an internal node
			area1 = perimeter(box1)

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendant of child1 could be the sibling with the lower bound cost of
			//       cost1 = inheritedCost + (directCost1 - area1) + areaD
			// This minimum here leads to the minimum of these two costs.
			lowerCost1 = inheritedCost + directCost1 + MinFloat(areaD-area1, 0.0)
		}

		// Cost of descending into child 2
		lowerCost2 := fltMax
		box2 := nodes[child2].AABB
		directCost2 := perimeter(box2.Union(boxD))
		area2 := float32(0.0)
		if leaf2 {
			cost2 := directCost2 + inheritedCost
			if cost2 < bestCost {
				bestSibling = child2
				bestCost = cost2
			}
		} else {
			area2 = perimeter(box2)
			lowerCost2 = inheritedCost + directCost2 + MinFloat(areaD-area2, 0.0)
		}

		if leaf1 && leaf2 {
			break
		}

		// Can the cost possibly be decreased?
		if bestCost <= lowerCost1 && bestCost <= lowerCost2 {
			break
		}

		if lowerCost1 == lowerCost2 && leaf1 == false {
			assert(lowerCost1 < fltMax, "lowerCost1 < FLT_MAX")
			assert(lowerCost2 < fltMax, "lowerCost2 < FLT_MAX")

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain D. Fall back to node distance.
			d1 := Sub(box1.Center(), centerD)
			d2 := Sub(box2.Center(), centerD)
			lowerCost1 = LengthSquared(d1)
			lowerCost2 = LengthSquared(d2)
		}

		// Descend
		if lowerCost1 < lowerCost2 && leaf1 == false {
			nodeIndex = child1
			areaBase = area1
			directCost = directCost1
		} else {
			nodeIndex = child2
			areaBase = area2
			directCost = directCost2
		}
	}

	return bestSibling
}

// Swap a child of A with a grandchild under the other child.
// Example swapping B with G:
// (A (B (D E) C (F G)) -> (A (G) C (F B (D E))
func swapNodes(tree *DynamicTree, iDown, iUp int) {
	nodes := tree.Nodes
	nodes[iDown], nodes[iUp] = nodes[iUp], nodes[iDown]
	linkChildren(tree, iDown)
	linkChildren(tree, iUp)

	if isNodeOrdered(nodes, iDown) == false || isNodeOrdered(nodes, iUp) == false {
		tree.DfsOrdered = false
	}

	// The sibling of the node that went down holds a different subtree now
	iC := iDown ^ 1
	nodes[iC] = makeInternalNode(nodes, getLeftChild(&nodes[iC]))
}

// Perform a left or right rotation if node A is imbalanced.
// Tree: (A (B (D E) C (F G))
func rotateNodes(tree *DynamicTree, iA int) {
	nodes := tree.Nodes
	A := &nodes[iA]
	assert(isLeaf(A) == false, "b3IsLeaf( A ) == false")

	iB := getLeftChild(A)
	iC := iB + 1
	B := &nodes[iB]
	C := &nodes[iC]

	leafB := isLeaf(B)
	leafC := isLeaf(C)
	if leafB && leafC {
		return
	}

	bestDown := NullIndex
	bestUp := NullIndex
	bestDelta := float32(0.0)

	if leafC == false {
		// Swap B with F or G
		iF := getLeftChild(C)
		iG := iF + 1
		areaC := perimeter(C.AABB)

		// B <-> F then C (B G)
		deltaBF := perimeter(unionV(B.AABB, nodes[iG].AABB)) - areaC
		if deltaBF < bestDelta {
			bestDown = iB
			bestUp = iF
			bestDelta = deltaBF
		}

		// B <-> G then C (F B)
		deltaBG := perimeter(unionV(B.AABB, nodes[iF].AABB)) - areaC
		if deltaBG < bestDelta {
			bestDown = iB
			bestUp = iG
			bestDelta = deltaBG
		}
	}

	if leafB == false {
		// Swap C with D or E
		iD := getLeftChild(B)
		iE := iD + 1
		areaB := perimeter(B.AABB)

		// C <-> D then B (C E)
		deltaCD := perimeter(unionV(C.AABB, nodes[iE].AABB)) - areaB
		if deltaCD < bestDelta {
			bestDown = iC
			bestUp = iD
			bestDelta = deltaCD
		}

		// C <-> E then B (D C)
		deltaCE := perimeter(unionV(C.AABB, nodes[iD].AABB)) - areaB
		if deltaCE < bestDelta {
			bestDown = iC
			bestUp = iE
			bestDelta = deltaCE
		}
	}

	if bestDown != NullIndex {
		swapNodes(tree, bestDown, bestUp)
	}
}

func insertLeaf(tree *DynamicTree, aabb AABB, proxyId int, moved bool, shouldRotate bool) {
	proxy := &tree.Proxies[proxyId]
	leaf := makeLeafNode(aabb, proxyId, proxy.UserData, moved)

	if isEmptyNode(&tree.Nodes[rootNode]) {
		tree.Nodes[rootNode] = leaf
		proxy.Node = rootNode
		return
	}

	sibling := findBestSibling(tree, aabb)

	// The sibling's position becomes the new parent. The sibling moves down into a new pair
	// beside the leaf, which puts it below its own children when it is internal.
	pair := allocateSiblingPair(tree)
	nodes := tree.Nodes
	parents := tree.Parents

	nodes[pair] = nodes[sibling]
	nodes[pair+1] = leaf
	parents[pair] = int32(sibling)
	parents[pair+1] = int32(sibling)
	linkChildren(tree, pair)
	proxy.Node = int32(pair + 1)

	nodes[sibling] = makeInternalNode(nodes, pair)

	if isNodeOrdered(nodes, sibling) == false || isNodeOrdered(nodes, pair) == false {
		tree.DfsOrdered = false
	}

	// Walk back up the tree refitting ancestors, the root included.
	index := sibling
	for index != NullIndex {
		if shouldRotate {
			rotateNodes(tree, index)
		}

		nodes[index] = makeInternalNode(nodes, getLeftChild(&nodes[index]))
		index = int(parents[index])
	}
}

func removeLeaf(tree *DynamicTree, proxyId int) {
	nodes := tree.Nodes
	parents := tree.Parents

	leaf := int(tree.Proxies[proxyId].Node)
	assert(0 <= leaf && leaf < int(tree.NodeEnd), "0 <= leaf && leaf < tree->nodeEnd")
	assert(isLeaf(&nodes[leaf]) && getProxyId(&nodes[leaf]) == proxyId, "b3IsLeaf( nodes + leaf ) && b3GetProxyId( nodes + leaf ) == proxyId")

	if leaf == rootNode {
		nodes[rootNode] = makeEmptyNode()
		return
	}

	// The sibling takes the parent's position and the pair is freed. This keeps the order,
	// the sibling only moves down.
	parent := int(parents[leaf])
	nodes[parent] = nodes[leaf^1]
	linkChildren(tree, parent)
	freePair(tree, leaf&^1)

	// Update ancestors.
	index := int(parents[parent])
	for index != NullIndex {
		nodes[index] = makeInternalNode(nodes, getLeftChild(&nodes[index]))
		index = int(parents[index])
	}
}

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
func createTreeProxyInternal(tree *DynamicTree, aabb AABB, categoryBits uint64, userData uint64, markMoved bool) int {
	validate(IsValidAABB(aabb), "b3IsValidAABB( aabb )")

	proxyId := allocateProxy(tree)

	proxy := &tree.Proxies[proxyId]
	proxy.CategoryBits = categoryBits
	proxy.UserData = userData

	shouldRotate := true
	insertLeaf(tree, aabb, proxyId, markMoved, shouldRotate)

	return proxyId
}

// CreateProxy is b3DynamicTree_CreateProxy: create a proxy. Provide an AABB and a userData value.
func (tree *DynamicTree) CreateProxy(aabb AABB, categoryBits uint64, userData uint64) int {
	return createTreeProxyInternal(tree, aabb, categoryBits, userData, false)
}

// DestroyProxy is b3DynamicTree_DestroyProxy: destroy a proxy. This asserts if the id is invalid.
func (tree *DynamicTree) DestroyProxy(proxyId int) {
	assert(0 <= proxyId && proxyId < int(tree.ProxyCapacity), "0 <= proxyId && proxyId < tree->proxyCapacity")

	removeLeaf(tree, proxyId)
	freeProxy(tree, proxyId)
}

// GetProxyCount is b3DynamicTree_GetProxyCount: get the number of proxies created.
func (tree *DynamicTree) GetProxyCount() int {
	return int(tree.ProxyCount)
}

// moveProxyInternal is b3DynamicTree_MoveProxyInternal.
func (tree *DynamicTree) moveProxyInternal(proxyId int, aabb AABB, markMoved bool) {
	validate(IsValidAABB(aabb), "b3IsValidAABB( aabb )")
	validate(aabb.UpperBound.X-aabb.LowerBound.X < Huge(), "aabb.upperBound.x - aabb.lowerBound.x < B3_HUGE")
	validate(aabb.UpperBound.Y-aabb.LowerBound.Y < Huge(), "aabb.upperBound.y - aabb.lowerBound.y < B3_HUGE")
	validate(aabb.UpperBound.Z-aabb.LowerBound.Z < Huge(), "aabb.upperBound.z - aabb.lowerBound.z < B3_HUGE")
	assert(0 <= proxyId && proxyId < int(tree.ProxyCapacity), "0 <= proxyId && proxyId < tree->proxyCapacity")

	removeLeaf(tree, proxyId)

	shouldRotate := false
	insertLeaf(tree, aabb, proxyId, markMoved, shouldRotate)
}

// MoveProxy is b3DynamicTree_MoveProxy: move a proxy to a new AABB by removing and reinserting into the tree.
func (tree *DynamicTree) MoveProxy(proxyId int, aabb AABB) {
	tree.moveProxyInternal(proxyId, aabb, false)
}

// EnlargeProxy is b3DynamicTree_EnlargeProxy: enlarge a proxy and enlarge ancestors as necessary.
func (tree *DynamicTree) EnlargeProxy(proxyId int, aabb AABB) {
	validate(IsValidAABB(aabb), "b3IsValidAABB( aabb )")
	validate(aabb.UpperBound.X-aabb.LowerBound.X < Huge(), "aabb.upperBound.x - aabb.lowerBound.x < B3_HUGE")
	validate(aabb.UpperBound.Y-aabb.LowerBound.Y < Huge(), "aabb.upperBound.y - aabb.lowerBound.y < B3_HUGE")
	validate(aabb.UpperBound.Z-aabb.LowerBound.Z < Huge(), "aabb.upperBound.z - aabb.lowerBound.z < B3_HUGE")
	assert(0 <= proxyId && proxyId < int(tree.ProxyCapacity), "0 <= proxyId && proxyId < tree->proxyCapacity")

	nodes := tree.Nodes
	parents := tree.Parents

	index := int(tree.Proxies[proxyId].Node)
	node := &nodes[index]
	assert(isLeaf(node), "b3IsLeaf( node )")

	// Caller must ensure this
	validate(node.AABB.Contains(aabb) == false, "b3AABB_Contains( node->aabb, aabb ) == false")

	node.AABB = aabb
	node.FlagIndex |= movedNode

	index = int(parents[index])
	for index != NullIndex {
		node = &nodes[index]
		changed := enlargeAABB(&node.AABB, aabb)

		// This is marked to ensure the root is marked in this loop or the one below.
		node.FlagIndex |= movedNode

		index = int(parents[index])

		if changed == false {
			break
		}
	}

	// Mark all the way up to the root.
	for index != NullIndex {
		node = &nodes[index]
		if node.FlagIndex&movedNode != 0 {
			// Early out because this ancestor was previously ascended and marked as moved.
			break
		}

		node.FlagIndex |= movedNode
		index = int(parents[index])
	}
}

// SetCategoryBits is b3DynamicTree_SetCategoryBits: modify the category bits on a proxy.
// This is an expensive operation.
func (tree *DynamicTree) SetCategoryBits(proxyId int, categoryBits uint64) {
	assert(0 <= proxyId && proxyId < int(tree.ProxyCapacity), "0 <= proxyId && proxyId < tree->proxyCapacity")
	tree.Proxies[proxyId].CategoryBits = categoryBits
}

// GetCategoryBits is b3DynamicTree_GetCategoryBits: get the category bits on a proxy.
func (tree *DynamicTree) GetCategoryBits(proxyId int) uint64 {
	assert(0 <= proxyId && proxyId < int(tree.ProxyCapacity), "0 <= proxyId && proxyId < tree->proxyCapacity")
	return tree.Proxies[proxyId].CategoryBits
}

// GetHeight is b3DynamicTree_GetHeight: get the height of the binary tree.
func (tree *DynamicTree) GetHeight() int {
	return getNodeHeight(&tree.Nodes[rootNode])
}

// The area ratio is the thing that SAH seeks to minimize. SAH
// cannot do anything about leaf boxes or the root box. It seeks
// to minimize the area of all non-root internal nodes. Divide this
// by the root area to make the metric non-dimensional.
// So this becomes a meaningful measure of tree quality.

// GetAreaRatio is b3DynamicTree_GetAreaRatio: get the ratio of the sum of the node areas to
// the root area.
func (tree *DynamicTree) GetAreaRatio() float32 {
	if tree.ProxyCount == 0 {
		return 0.0
	}

	nodes := tree.Nodes
	rootArea := perimeter(nodes[rootNode].AABB)
	if rootArea <= 0.0 {
		return 0.0
	}

	// Free nodes and the empty node are leaf tagged.
	internalArea := float32(0.0)
	nodeEnd := int(tree.NodeEnd)
	for i := 2; i < nodeEnd; i++ {
		if isLeaf(&nodes[i]) == false {
			internalArea += perimeter(nodes[i].AABB)
		}
	}

	return internalArea / rootArea
}

// GetRootBounds is b3DynamicTree_GetRootBounds: get the bounding box that contains the entire tree.
func (tree *DynamicTree) GetRootBounds() AABB {
	if tree.ProxyCount == 0 {
		return AABB{Vec3Zero, Vec3Zero}
	}

	return tree.Nodes[rootNode].AABB
}

// PORT: #if B3_ENABLE_VALIDATION becomes `if !validateEnabled { return }`; the helpers are
// always compiled.
func validateSubtree(tree *DynamicTree, nodeIndex int, leafCount *int) int {
	assert(0 <= nodeIndex && nodeIndex < int(tree.NodeEnd), "0 <= nodeIndex && nodeIndex < tree->nodeEnd")
	node := &tree.Nodes[nodeIndex]
	assert(isEmptyNode(node) == false, "b3IsEmptyNode( node ) == false")
	assert(IsValidAABB(node.AABB), "b3IsValidAABB( node->aabb )")

	if isLeaf(node) {
		proxyId := getProxyId(node)
		assert(0 <= proxyId && proxyId < int(tree.ProxyCapacity), "0 <= proxyId && proxyId < tree->proxyCapacity")
		assert(int(tree.Proxies[proxyId].Node) == nodeIndex, "tree->proxies[proxyId].node == nodeIndex")
		assert(int32(uint32(tree.Proxies[proxyId].UserData)) == node.Height, "(int32_t)(uint32_t)tree->proxies[proxyId].userData == node->shapeIndex")
		*leafCount += 1
		return 0
	}

	pair := getLeftChild(node)
	assert((pair&1) == 0 && 2 <= pair && pair < int(tree.NodeEnd), "( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd")
	assert(int(tree.Parents[pair]) == nodeIndex, "tree->parents[pair] == nodeIndex")
	assert(int(tree.Parents[pair+1]) == nodeIndex, "tree->parents[pair + 1] == nodeIndex")
	assert(tree.DfsOrdered == false || nodeIndex < pair, "tree->dfsOrdered == false || nodeIndex < pair")

	c1 := &tree.Nodes[pair]
	c2 := &tree.Nodes[pair+1]
	assert(node.AABB.Contains(c1.AABB), "b3AABB_Contains( node->aabb, c1->aabb )")
	assert(node.AABB.Contains(c2.AABB), "b3AABB_Contains( node->aabb, c2->aabb )")
	assert(isNodeMoved(node) == (isNodeMoved(c1) || isNodeMoved(c2)), "b3IsNodeMoved( node ) == ( b3IsNodeMoved( c1 ) || b3IsNodeMoved( c2 ) )")

	// A bad tree can stack overflow, but that is validation on its own.
	height1 := validateSubtree(tree, pair, leafCount)
	height2 := validateSubtree(tree, pair+1, leafCount)
	height := 1 + MaxInt(height1, height2)
	assert(int(node.Height) == height, "node->height == height")
	return height
}

// Validate is b3DynamicTree_Validate: validate this tree. For testing.
func (tree *DynamicTree) Validate() {
	if !validateEnabled {
		return
	}

	assert(2 <= tree.NodeEnd && tree.NodeEnd <= tree.NodeCapacity, "2 <= tree->nodeEnd && tree->nodeEnd <= tree->nodeCapacity")
	assert((tree.NodeEnd&1) == 0, "( tree->nodeEnd & 1 ) == 0")
	assert(tree.Parents[rootNode] == NullIndex, "tree->parents[B3_ROOT_NODE] == B3_NULL_INDEX")
	assert(isEmptyNode(&tree.Nodes[rootNode+1]), "b3IsEmptyNode( tree->nodes + B3_ROOT_NODE + 1 )")

	freePairCount := 0
	pair := int(tree.PairFreeList)
	for pair != NullIndex {
		assert((pair&1) == 0 && 2 <= pair && pair < int(tree.NodeEnd), "( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd")
		assert(isEmptyNode(&tree.Nodes[pair]), "b3IsEmptyNode( tree->nodes + pair )")
		assert(isEmptyNode(&tree.Nodes[pair+1]), "b3IsEmptyNode( tree->nodes + pair + 1 )")
		pair = int(tree.Parents[pair])
		freePairCount++
		assert(2*freePairCount < int(tree.NodeEnd), "2 * freePairCount < tree->nodeEnd")
	}

	// Validate proxy free list.
	freeProxyCount := 0
	freeIndex := int(tree.ProxyFreeList)
	for freeIndex != NullIndex {
		assert(0 <= freeIndex && freeIndex < int(tree.ProxyCapacity), "0 <= freeIndex && freeIndex < tree->proxyCapacity")
		assert(tree.Proxies[freeIndex].Node == NullIndex, "tree->proxies[freeIndex].node == B3_NULL_INDEX")
		freeIndex = int(tree.Proxies[freeIndex].Next)
		freeProxyCount++
	}
	assert(int(tree.ProxyCount)+freeProxyCount == int(tree.ProxyCapacity), "tree->proxyCount + freeProxyCount == tree->proxyCapacity")

	assert(int(tree.NodeEnd) == 2*MaxInt(int(tree.ProxyCount), 1)+2*freePairCount, "tree->nodeEnd == 2 * b3MaxInt( tree->proxyCount, 1 ) + 2 * freePairCount")

	if tree.ProxyCount == 0 {
		assert(isEmptyNode(&tree.Nodes[rootNode]), "b3IsEmptyNode( tree->nodes + B3_ROOT_NODE )")
		return
	}

	leafCount := 0
	validateSubtree(tree, rootNode, &leafCount)
	assert(leafCount == int(tree.ProxyCount), "leafCount == tree->proxyCount")
}

// ValidateNoMoved is b3DynamicTree_ValidateNoMoved: validate this tree has no moved nodes.
// For testing.
func (tree *DynamicTree) ValidateNoMoved() {
	if !validateEnabled {
		return
	}

	nodes := tree.Nodes
	nodeEnd := int(tree.NodeEnd)
	for i := 0; i < nodeEnd; i++ {
		assert(isNodeMoved(&nodes[i]) == false, "b3IsNodeMoved( nodes + i ) == false")
	}
}

// Sizes of the C structs for b3DynamicTree_GetByteCount and the save format.
const (
	dynamicTreeCSize = 112 // sizeof( b3DynamicTree ) on 64-bit targets
	treeNodeCSize    = 32
	treeProxyCSize   = 24
	aabbCSize        = 24
	vec3CSize        = 12
)

// GetByteCount is b3DynamicTree_GetByteCount: get the number of bytes used by this tree.
// PORT: counts upstream's C sizes (sizeof( b3DynamicTree ) on a 64-bit target).
func (tree *DynamicTree) GetByteCount() int {
	size := dynamicTreeCSize
	size += int(tree.NodeCapacity) * treeNodeCSize
	size += int(tree.NodeCapacity) * 4
	size += int(tree.ProxyCapacity) * treeProxyCSize
	if tree.SwapNodes != nil {
		size += int(tree.NodeCapacity) * treeNodeCSize
	}

	// leafIndices
	size += int(tree.RebuildCapacity) * 4
	// leafNodes
	size += int(tree.RebuildCapacity) * treeNodeCSize
	// leafBoxes
	size += int(tree.RebuildCapacity) * aabbCSize
	// leafCenters
	size += int(tree.RebuildCapacity) * vec3CSize
	// binIndices
	size += int(tree.RebuildCapacity) * 4

	return size
}

func testCategory(categoryBits, maskBits uint64, requireAllBits bool) bool {
	if requireAllBits {
		return (categoryBits & maskBits) == maskBits
	}
	return (categoryBits & maskBits) != 0
}

func queryTreeView(view *treeView, aabb AABB, maskBits uint64, requireAllBits bool,
	callback TreeQueryCallbackFcn, context any, hasProxies bool) TreeStats {
	var result TreeStats

	if isEmptyNode(&view.nodes[rootNode]) {
		return result
	}

	nodes := view.nodes

	var stack [treeStackSize]int32
	stackCount := 0
	stack[stackCount] = int32(getRootPair(nodes))
	stackCount++

	boxv := loadAABBV(&aabb)

	for stackCount > 0 {
		stackCount--
		pair := int(stack[stackCount])
		result.NodeVisits += 1

		for i := 0; i < 2; i++ {
			node := &nodes[pair+i]
			if overlapNode(boxv, node) == false {
				continue
			}

			if isLeaf(node) {
				proxyId := getProxyId(node)
				userData := uint64(uint32(node.Height))
				if hasProxies {
					proxy := &view.proxies[proxyId]
					if testCategory(proxy.CategoryBits, maskBits, requireAllBits) == false {
						continue
					}
					userData = proxy.UserData
				}

				proceed := callback(proxyId, userData, context)
				result.LeafVisits += 1

				if proceed == false {
					return result
				}
			} else {
				if stackCount < treeStackSize-1 {
					stack[stackCount] = int32(getLeftChild(node))
					stackCount++
				} else {
					assert(stackCount < treeStackSize-1, "stackCount < B3_TREE_STACK_SIZE - 1")
				}
			}
		}
	}

	return result
}

func distanceToNodeSqr(point Vec3, node *TreeNode) float32 {
	r := Sub(point, Clamp(point, node.AABB.LowerBound, node.AABB.UpperBound))
	return Dot(r, r)
}

type queryClosestItem struct {
	nodeIndex         int32
	distanceToNodeSqr float32
}

func queryClosestTreeView(view *treeView, point Vec3, maskBits uint64, requireAllBits bool,
	callback TreeQueryClosestCallbackFcn, context any, minDistanceSqr *float32, hasProxies bool) TreeStats {
	var result TreeStats

	if isEmptyNode(&view.nodes[rootNode]) {
		return result
	}

	nodes := view.nodes

	minSqr := *minDistanceSqr
	var stack [treeStackSize]queryClosestItem
	stackCount := 0

	rootPair := getRootPair(nodes)
	seed1 := queryClosestItem{
		nodeIndex:         int32(rootPair),
		distanceToNodeSqr: distanceToNodeSqr(point, &nodes[rootPair]),
	}

	if rootPair == rootNode {
		stack[stackCount] = seed1
		stackCount++
	} else {
		seed2 := queryClosestItem{
			nodeIndex:         int32(rootPair + 1),
			distanceToNodeSqr: distanceToNodeSqr(point, &nodes[rootPair+1]),
		}

		// Ensure we iterate the closest child first as we pop off the stack
		if seed2.distanceToNodeSqr < seed1.distanceToNodeSqr {
			stack[stackCount] = seed1
			stackCount++
			stack[stackCount] = seed2
			stackCount++
		} else {
			stack[stackCount] = seed2
			stackCount++
			stack[stackCount] = seed1
			stackCount++
		}
	}

	for stackCount > 0 {
		stackCount--
		item := stack[stackCount]
		node := &nodes[item.nodeIndex]
		result.NodeVisits += 1

		if item.distanceToNodeSqr >= minSqr {
			continue
		}

		if isLeaf(node) {
			proxyId := getProxyId(node)
			userData := uint64(uint32(node.Height))
			if hasProxies {
				proxy := &view.proxies[proxyId]
				if testCategory(proxy.CategoryBits, maskBits, requireAllBits) == false {
					continue
				}
				userData = proxy.UserData
			}

			// callback to user code with minimum distance squared so far and proxy id
			dd := callback(minSqr, proxyId, userData, context)

			if dd < minSqr {
				minSqr = dd
			}

			result.LeafVisits += 1
			continue
		}

		if stackCount+2 > treeStackSize {
			assert(stackCount+2 <= treeStackSize, "stackCount + 2 <= B3_TREE_STACK_SIZE")
			continue
		}

		pair := getLeftChild(node)

		// Store the distance to node in the stack instead of recomputing after pop
		item1 := queryClosestItem{
			nodeIndex:         int32(pair),
			distanceToNodeSqr: distanceToNodeSqr(point, &nodes[pair]),
		}

		item2 := queryClosestItem{
			nodeIndex:         int32(pair + 1),
			distanceToNodeSqr: distanceToNodeSqr(point, &nodes[pair+1]),
		}

		// Ensure we iterate the closest child first as we pop off the stack
		if item2.distanceToNodeSqr < item1.distanceToNodeSqr {
			stack[stackCount] = item1
			stackCount++
			stack[stackCount] = item2
			stackCount++
		} else {
			stack[stackCount] = item2
			stackCount++
			stack[stackCount] = item1
			stackCount++
		}
	}

	*minDistanceSqr = minSqr

	return result
}

// vec3Floats views a Vec3 as float[3], the &v.x argument of b3LoadV.
// PORT: unsafe.Slice stands in for the pointer to the first component.
func vec3Floats(v *Vec3) []float32 {
	return unsafe.Slice(&v.X, 3)
}

// A lot of optimization work went into this. The children of a popped pair are both tested
// before either is followed, then the survivors are ordered so the closer one narrows first.
func rayCastTreeView(view *treeView, input *RayCastInput, maskBits uint64, requireAllBits bool,
	callback TreeRayCastCallbackFcn, context any, hasProxies bool) TreeStats {
	var result TreeStats

	if isEmptyNode(&view.nodes[rootNode]) {
		return result
	}

	p1 := input.Origin
	d := input.Translation

	pv1 := loadV(vec3Floats(&p1))
	dv := loadV(vec3Floats(&d))

	maxFraction := input.MaxFraction

	p2 := MulAdd(p1, maxFraction, d)

	// Build a bounding box for the segment.
	segmentAABB := AABB{Min(p1, p2), Max(p1, p2)}
	boxv := loadAABBV(&segmentAABB)

	nodes := view.nodes

	var stack [treeStackSize]int32
	stackCount := 0
	stack[stackCount] = int32(getRootPair(nodes))
	stackCount++

	subInput := *input

	for stackCount > 0 {
		stackCount--
		pair := int(stack[stackCount])
		result.NodeVisits += 1

		// PORT: hit holds node pointers as in C.
		var hit [2]*TreeNode
		var isLeafHit [2]bool
		hitCount := 0
		for i := 0; i < 2; i++ {
			node := &nodes[pair+i]

			if overlapNode(boxv, node) == false {
				continue
			}

			nodeAABB := node.AABB
			lower := loadV(vec3Floats(&nodeAABB.LowerBound))
			upper := loadV(vec3Floats(&nodeAABB.UpperBound))
			if testBoundsRayOverlap(lower, upper, pv1, dv) == false {
				continue
			}

			isLeafHit[hitCount] = isLeaf(node)
			hit[hitCount] = node
			hitCount += 1
		}

		if hitCount == 2 && isLeafHit[0] == false && isLeafHit[1] == false {
			center1 := hit[0].AABB.Center()
			center2 := hit[1].AABB.Center()
			d1 := DistanceSquared(center1, p1)
			d2 := DistanceSquared(center2, p1)

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if d1 < d2 {
				hit[0], hit[1] = hit[1], hit[0]
			}
		}

		for i := 0; i < hitCount; i++ {
			if isLeafHit[i] {
				proxyId := getProxyId(hit[i])
				userData := uint64(uint32(hit[i].Height))
				if hasProxies {
					proxy := &view.proxies[proxyId]
					if testCategory(proxy.CategoryBits, maskBits, requireAllBits) == false {
						continue
					}
					userData = proxy.UserData
				}

				subInput.MaxFraction = maxFraction

				value := callback(&subInput, proxyId, userData, context)
				result.LeafVisits += 1

				// The user may return -1 to indicate this shape should be skipped

				if value == 0.0 {
					// The client has terminated the ray cast.
					return result
				}

				if 0.0 < value && value <= maxFraction {
					// Update segment bounding box.
					maxFraction = value
					p2 = MulAdd(p1, maxFraction, d)
					segmentAABB.LowerBound = Min(p1, p2)
					segmentAABB.UpperBound = Max(p1, p2)
					boxv = loadAABBV(&segmentAABB)
				}
			} else {
				if stackCount < treeStackSize-1 {
					stack[stackCount] = int32(getLeftChild(hit[i]))
					stackCount++
				} else {
					assert(stackCount < treeStackSize-1, "stackCount < B3_TREE_STACK_SIZE - 1")
				}
			}
		}
	}

	return result
}

// Follows structure of ray cast with small tweaks to handle a swept box.
func boxCastTreeView(view *treeView, input *BoxCastInput, maskBits uint64, requireAllBits bool,
	callback TreeBoxCastCallbackFcn, context any, hasProxies bool) TreeStats {
	var result TreeStats

	if isEmptyNode(&view.nodes[rootNode]) {
		return result
	}

	// The caller folds the shape radius and the world origin into the box
	originAABB := input.Box

	p1 := originAABB.Center()
	extension := originAABB.Extents()

	d := input.Translation

	pv1 := loadV(vec3Floats(&p1))
	dv := loadV(vec3Floats(&d))
	ev := loadV(vec3Floats(&extension))

	maxFraction := input.MaxFraction

	// Build total box for the cast
	t := MulSV(maxFraction, input.Translation)
	totalAABB := AABB{
		Min(originAABB.LowerBound, Add(originAABB.LowerBound, t)),
		Max(originAABB.UpperBound, Add(originAABB.UpperBound, t)),
	}
	boxv := loadAABBV(&totalAABB)

	nodes := view.nodes

	var stack [treeStackSize]int32
	stackCount := 0
	stack[stackCount] = int32(getRootPair(nodes))
	stackCount++

	subInput := *input

	for stackCount > 0 {
		stackCount--
		pair := int(stack[stackCount])
		result.NodeVisits += 1

		var hit [2]*TreeNode
		var isLeafHit [2]bool
		hitCount := 0
		for i := 0; i < 2; i++ {
			node := &nodes[pair+i]

			if overlapNode(boxv, node) == false {
				continue
			}

			// radius extension is added to the node in this case
			nodeAABB := node.AABB
			lower := subV(loadV(vec3Floats(&nodeAABB.LowerBound)), ev)
			upper := addV(loadV(vec3Floats(&nodeAABB.UpperBound)), ev)
			if testBoundsRayOverlap(lower, upper, pv1, dv) == false {
				continue
			}

			isLeafHit[hitCount] = isLeaf(node)
			hit[hitCount] = node
			hitCount += 1
		}

		if hitCount == 2 && isLeafHit[0] == false && isLeafHit[1] == false {
			center1 := hit[0].AABB.Center()
			center2 := hit[1].AABB.Center()
			d1 := DistanceSquared(center1, p1)
			d2 := DistanceSquared(center2, p1)

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if d1 < d2 {
				hit[0], hit[1] = hit[1], hit[0]
			}
		}

		for i := 0; i < hitCount; i++ {
			if isLeafHit[i] {
				proxyId := getProxyId(hit[i])
				userData := uint64(uint32(hit[i].Height))
				if hasProxies {
					proxy := &view.proxies[proxyId]
					if testCategory(proxy.CategoryBits, maskBits, requireAllBits) == false {
						continue
					}
					userData = proxy.UserData
				}

				subInput.MaxFraction = maxFraction

				value := callback(&subInput, proxyId, userData, context)
				result.LeafVisits += 1

				if value == 0.0 {
					// The client has terminated the cast.
					return result
				}

				if 0.0 < value && value < maxFraction {
					maxFraction = value
					t = MulSV(maxFraction, input.Translation)
					totalAABB.LowerBound = Min(originAABB.LowerBound, Add(originAABB.LowerBound, t))
					totalAABB.UpperBound = Max(originAABB.UpperBound, Add(originAABB.UpperBound, t))
					boxv = loadAABBV(&totalAABB)
				}
			} else {
				if stackCount < treeStackSize-1 {
					stack[stackCount] = int32(getLeftChild(hit[i]))
					stackCount++
				} else {
					assert(stackCount < treeStackSize-1, "stackCount < B3_TREE_STACK_SIZE - 1")
				}
			}
		}
	}

	return result
}

// Query is b3DynamicTree_Query: query an AABB for overlapping proxies. The callback class is
// called for each proxy that overlaps the supplied AABB.
func (tree *DynamicTree) Query(aabb AABB, maskBits uint64, requireAllBits bool,
	callback TreeQueryCallbackFcn, context any) TreeStats {
	if tree.ProxyCount == 0 {
		return TreeStats{}
	}

	view := makeTreeView(tree)
	return queryTreeView(&view, aabb, maskBits, requireAllBits, callback, context, true)
}

// QueryClosest is b3DynamicTree_QueryClosest: query the tree for the proxy closest to a point.
// minDistanceSqr is the initial squared distance bound on input and the closest squared
// distance found on output.
func (tree *DynamicTree) QueryClosest(point Vec3, maskBits uint64, requireAllBits bool,
	callback TreeQueryClosestCallbackFcn, context any, minDistanceSqr *float32) TreeStats {
	if tree.ProxyCount == 0 {
		return TreeStats{}
	}

	view := makeTreeView(tree)
	return queryClosestTreeView(&view, point, maskBits, requireAllBits, callback, context, minDistanceSqr, true)
}

// RayCast is b3DynamicTree_RayCast: ray cast against the proxies in the tree. This relies on
// the callback to perform an exact ray cast in the case where the proxy contains a shape.
// The callback also performs any collision filtering. This has performance roughly equal to
// k * log(n), where k is the number of collisions and n is the number of proxies in the tree.
func (tree *DynamicTree) RayCast(input *RayCastInput, maskBits uint64, requireAllBits bool,
	callback TreeRayCastCallbackFcn, context any) TreeStats {
	if tree.ProxyCount == 0 {
		return TreeStats{}
	}

	view := makeTreeView(tree)
	return rayCastTreeView(&view, input, maskBits, requireAllBits, callback, context, true)
}

// BoxCast is b3DynamicTree_BoxCast: sweep an AABB against the proxies in the tree.
func (tree *DynamicTree) BoxCast(input *BoxCastInput, maskBits uint64, requireAllBits bool,
	callback TreeBoxCastCallbackFcn, context any) TreeStats {
	if tree.ProxyCount == 0 {
		return TreeStats{}
	}

	view := makeTreeView(tree)
	return boxCastTreeView(&view, input, maskBits, requireAllBits, callback, context, true)
}

// query is b3TreeView_Query.
func (view *treeView) query(aabb AABB, maskBits uint64, requireAllBits bool,
	callback TreeQueryCallbackFcn, context any) TreeStats {
	if view.proxies == nil {
		return queryTreeView(view, aabb, maskBits, requireAllBits, callback, context, false)
	}

	return queryTreeView(view, aabb, maskBits, requireAllBits, callback, context, true)
}

// queryClosest is b3TreeView_QueryClosest.
func (view *treeView) queryClosest(point Vec3, maskBits uint64, requireAllBits bool,
	callback TreeQueryClosestCallbackFcn, context any, minDistanceSqr *float32) TreeStats {
	if view.proxies == nil {
		return queryClosestTreeView(view, point, maskBits, requireAllBits, callback, context, minDistanceSqr, false)
	}

	return queryClosestTreeView(view, point, maskBits, requireAllBits, callback, context, minDistanceSqr, true)
}

// rayCast is b3TreeView_RayCast.
func (view *treeView) rayCast(input *RayCastInput, maskBits uint64, requireAllBits bool,
	callback TreeRayCastCallbackFcn, context any) TreeStats {
	if view.proxies == nil {
		return rayCastTreeView(view, input, maskBits, requireAllBits, callback, context, false)
	}

	return rayCastTreeView(view, input, maskBits, requireAllBits, callback, context, true)
}

// boxCast is b3TreeView_BoxCast.
func (view *treeView) boxCast(input *BoxCastInput, maskBits uint64, requireAllBits bool,
	callback TreeBoxCastCallbackFcn, context any) TreeStats {
	if view.proxies == nil {
		return boxCastTreeView(view, input, maskBits, requireAllBits, callback, context, false)
	}

	return boxCastTreeView(view, input, maskBits, requireAllBits, callback, context, true)
}

// Median split == 0, Surface area heuristic == 1
// PORT: B3_TREE_HEURISTIC is 0 upstream. The surface area heuristic branch (b3TreeBin,
// b3TreePlane, b3PartitionSAH) is compiled out there and not ported.

// Median split heuristic
func partitionMid(indices []int32, centers []Vec3, count int) int {
	// Handle trivial case
	if count <= 2 {
		return count / 2
	}

	lowerBound := centers[0]
	upperBound := centers[0]

	for i := 1; i < count; i++ {
		lowerBound = Min(lowerBound, centers[i])
		upperBound = Max(upperBound, centers[i])
	}

	d := Sub(upperBound, lowerBound)
	c := MulSV(0.5, Add(lowerBound, upperBound))

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	i1, i2 := 0, count
	if d.X >= d.Y && d.X >= d.Z {
		pivot := c.X

		for i1 < i2 {
			for i1 < i2 && centers[i1].X < pivot {
				i1 += 1
			}

			for i1 < i2 && centers[i2-1].X >= pivot {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap indices
				indices[i1], indices[i2-1] = indices[i2-1], indices[i1]

				// Swap centers
				centers[i1], centers[i2-1] = centers[i2-1], centers[i1]

				i1 += 1
				i2 -= 1
			}
		}
	} else if d.Y >= d.Z {
		pivot := c.Y

		for i1 < i2 {
			for i1 < i2 && centers[i1].Y < pivot {
				i1 += 1
			}

			for i1 < i2 && centers[i2-1].Y >= pivot {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap indices
				indices[i1], indices[i2-1] = indices[i2-1], indices[i1]

				// Swap centers
				centers[i1], centers[i2-1] = centers[i2-1], centers[i1]

				i1 += 1
				i2 -= 1
			}
		}
	} else {
		pivot := c.Z

		for i1 < i2 {
			for i1 < i2 && centers[i1].Z < pivot {
				i1 += 1
			}

			for i1 < i2 && centers[i2-1].Z >= pivot {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap indices
				indices[i1], indices[i2-1] = indices[i2-1], indices[i1]

				// Swap centers
				centers[i1], centers[i2-1] = centers[i2-1], centers[i1]

				i1 += 1
				i2 -= 1
			}
		}
	}
	assert(i1 == i2, "i1 == i2")

	if i1 > 0 && i1 < count {
		return i1
	}

	return count / 2
}

// Temporary data used to track the rebuild of a tree node.
type rebuildItem struct {
	// Where this node is written and the pair its children go in.
	nodeIndex  int32
	pair       int32
	childCount int32

	// Leaf indices
	startIndex int32
	splitIndex int32
	endIndex   int32
}

type copyItem struct {
	oldPair  int32
	newIndex int32
}

// Bump allocate a pair of sibling nodes. Returns index to the first one.
func bumpPair(tree *DynamicTree, parent int) int {
	pair := int(tree.NodeEnd)
	assert(pair+2 <= int(tree.NodeCapacity), "pair + 2 <= tree->nodeCapacity")
	tree.NodeEnd += 2
	tree.Parents[pair] = int32(parent)
	tree.Parents[pair+1] = int32(parent)
	return pair
}

func setLeftChild(node *TreeNode, pair int) {
	node.FlagIndex = (node.FlagIndex &^ nodeIndexMask) | uint32(pair)
}

// Copy a retained subtree from the old array into the rebuilt DFS array using
// a bump allocator.
func copySubtree(tree *DynamicTree, node TreeNode, newIndex int) {
	oldNodes := tree.Nodes
	newNodes := tree.SwapNodes
	proxies := tree.Proxies

	if isLeaf(&node) {
		newNodes[newIndex] = node
		proxies[getProxyId(&node)].Node = int32(newIndex)
		return
	}

	var stack [treeStackSize]copyItem
	stackCount := 0

	oldPair := getLeftChild(&node)
	newPair := bumpPair(tree, newIndex)
	setLeftChild(&node, newPair)

	// Copy the subtree root. This can be a left or right sibling (even or odd).
	newNodes[newIndex] = node

	for {
		pair := newNodes[newPair : newPair+2]

		// Copy the siblings.
		pair[0] = oldNodes[oldPair]
		pair[1] = oldNodes[oldPair+1]

		if isLeaf(&pair[1]) {
			// Hook up proxy on right sibling.
			proxyId := getProxyId(&pair[1])
			proxies[proxyId].Node = int32(newPair + 1)
		} else {
			// Push the left child of the right sibling.
			if stackCount < treeStackSize {
				leftChild := getLeftChild(&pair[1])
				stack[stackCount] = copyItem{int32(leftChild), int32(newPair + 1)}
				stackCount++
			} else {
				// todo fail gracefully if the stack runs out in release
				assert(stackCount < treeStackSize, "stackCount < B3_TREE_STACK_SIZE")
			}
		}

		if isLeaf(&pair[0]) == false {
			// Descend into left subtree.
			leftIndex := newPair
			oldPair = getLeftChild(&pair[0])
			newPair = bumpPair(tree, leftIndex)
			setLeftChild(&newNodes[leftIndex], newPair)
			continue
		}

		// Hook up proxy on left sibling.
		proxies[getProxyId(&pair[0])].Node = int32(newPair)

		if stackCount == 0 {
			break
		}

		// Descend a right subtree that was previously pushed.
		stackCount--
		item := stack[stackCount]
		oldPair = int(item.oldPair)
		newPair = bumpPair(tree, int(item.newIndex))
		setLeftChild(&newNodes[item.newIndex], newPair)
	}
}

func placeLeaf(tree *DynamicTree, leaf TreeNode, newIndex int) {
	if isLeaf(&leaf) {
		tree.SwapNodes[newIndex] = leaf
		tree.Proxies[getProxyId(&leaf)].Node = int32(newIndex)
	} else {
		copySubtree(tree, leaf, newIndex)
	}
}

func buildTree(tree *DynamicTree, leafCount int) {
	nodes := tree.SwapNodes
	leaves := tree.LeafNodes
	leafIndices := tree.LeafIndices

	leafCenters := tree.LeafCenters

	// Bump allocation into the swap nodes.
	tree.NodeEnd = 2
	nodes[rootNode+1] = makeEmptyNode()
	tree.Parents[rootNode] = NullIndex
	tree.Parents[rootNode+1] = NullIndex

	if leafCount == 1 {
		placeLeaf(tree, leaves[leafIndices[0]], rootNode)
		return
	}

	var stack [treeStackSize]rebuildItem
	top := 0

	stack[0].nodeIndex = rootNode
	stack[0].pair = int32(bumpPair(tree, rootNode))
	stack[0].childCount = -1
	stack[0].startIndex = 0
	stack[0].endIndex = int32(leafCount)
	stack[0].splitIndex = int32(partitionMid(leafIndices, leafCenters, leafCount))

	for {
		item := &stack[top]
		item.childCount += 1

		if item.childCount == 2 {
			// Both children written, so the parent node can be finalized.
			nodes[item.nodeIndex] = makeInternalNode(nodes, int(item.pair))
			if top == 0 {
				break
			}

			top -= 1
			continue
		}

		slot := int(item.childCount)
		startIndex := int(item.splitIndex)
		endIndex := int(item.endIndex)
		if slot == 0 {
			startIndex = int(item.startIndex)
			endIndex = int(item.splitIndex)
		}
		count := endIndex - startIndex
		assert(count > 0, "count > 0")

		nodeIndex := int(item.pair) + slot
		if count == 1 {
			placeLeaf(tree, leaves[leafIndices[startIndex]], nodeIndex)
			continue
		}

		// todo fail gracefully if the stack runs out in release
		assert(top < treeStackSize-1, "top < B3_TREE_STACK_SIZE - 1")

		top += 1
		newItem := &stack[top]
		newItem.nodeIndex = int32(nodeIndex)
		newItem.pair = int32(bumpPair(tree, nodeIndex))
		newItem.childCount = -1
		newItem.startIndex = int32(startIndex)
		newItem.endIndex = int32(endIndex)
		newItem.splitIndex = int32(startIndex)
		newItem.splitIndex += int32(partitionMid(leafIndices[startIndex:], leafCenters[startIndex:], count))
	}
}

// Rebuild is b3DynamicTree_Rebuild: rebuild the tree while retaining subtrees that haven't
// changed. Returns the number of boxes sorted.
//
// Rebuild the stale parts of the tree. The entire tree is put into DFS order. This makes
// refitting much faster. This is done async with threading, so the cost is hidden.
// Not safe to access tree during this operation.
func (tree *DynamicTree) Rebuild(fullBuild bool) int {
	proxyCount := int(tree.ProxyCount)
	if proxyCount == 0 {
		return 0
	}

	// An unordered tree is rebuilt even when nothing moved, the sweep refit needs the order
	nodes := tree.Nodes
	root := &nodes[rootNode]
	if fullBuild == false && isNodeMoved(root) == false && tree.DfsOrdered {
		return 0
	}

	if tree.SwapNodes == nil {
		tree.SwapNodes = make([]TreeNode, tree.NodeCapacity)
	}

	if proxyCount > int(tree.RebuildCapacity) {
		newCapacity := proxyCount + proxyCount/2

		// PORT: b3Free + b3Alloc become make; the SAH arrays (leafBoxes, binIndices) stay nil.
		tree.LeafIndices = make([]int32, newCapacity)
		tree.LeafNodes = make([]TreeNode, newCapacity)
		tree.LeafCenters = make([]Vec3, newCapacity)
		tree.RebuildCapacity = int32(newCapacity)
	}

	leafIndices := tree.LeafIndices
	leaves := tree.LeafNodes
	leafCenters := tree.LeafCenters

	// Gather build leaves. A marked internal node is descended and abandoned in the old array.
	// Everything else is a build leaf, kept subtrees included. A leaf root, or an unmarked root
	// on an unordered array, is the one build leaf.
	leafCount := 0
	var stack [treeStackSize]int32
	stackCount := 0
	if isLeaf(root) == false && (fullBuild || isNodeMoved(root)) {
		stack[stackCount] = int32(getLeftChild(root))
		stackCount++
	} else {
		node := *root
		if fullBuild == false {
			node.FlagIndex &^= movedNode
		}
		leafIndices[0] = 0
		leaves[0] = node
		leafCenters[0] = node.AABB.Center()
		leafCount = 1
	}

	for stackCount > 0 {
		stackCount--
		pair := int(stack[stackCount])
		for i := 0; i < 2; i++ {
			node := nodes[pair+i]
			if isLeaf(&node) == false && (fullBuild || isNodeMoved(&node)) {
				// todo fail gracefully if the stack runs out in release
				assert(stackCount < treeStackSize, "stackCount < B3_TREE_STACK_SIZE")
				stack[stackCount] = int32(getLeftChild(&node))
				stackCount++
				continue
			}

			if fullBuild == false {
				node.FlagIndex &^= movedNode
			}
			leafIndices[leafCount] = int32(leafCount)
			leaves[leafCount] = node
			leafCenters[leafCount] = node.AABB.Center()
			leafCount += 1
		}
	}

	assert(0 < leafCount && leafCount <= proxyCount, "0 < leafCount && leafCount <= proxyCount")

	buildTree(tree, leafCount)

	// The spare is now the tree and the old array is the spare. The build is dense, so the
	// stale tail of the old array sits above the end and is never read.
	tree.Nodes, tree.SwapNodes = tree.SwapNodes, tree.Nodes
	tree.PairFreeList = NullIndex
	tree.DfsOrdered = true

	tree.Validate()
	if fullBuild == false {
		tree.ValidateNoMoved()
	}

	return leafCount
}

// markProxyMovedSerial is b3DynamicTree_MarkProxyMovedSerial.
// Set the moved flag on the ancestors of the proxy. Serial use case.
func (tree *DynamicTree) markProxyMovedSerial(proxyId int) {
	validate(0 <= proxyId && proxyId < int(tree.ProxyCapacity), "0 <= proxyId && proxyId < tree->proxyCapacity")

	nodes := tree.Nodes
	parents := tree.Parents

	index := int(tree.Proxies[proxyId].Node)
	validate(0 <= index && index < int(tree.NodeEnd), "0 <= index && index < tree->nodeEnd")
	validate(!validateEnabled || isLeaf(&nodes[index]), "b3IsLeaf( nodes + index )")

	for index != NullIndex {
		nodes[index].FlagIndex |= movedNode
		index = int(parents[index])
	}
}

// markProxyMoved is b3DynamicTree_MarkProxyMoved.
// Update a proxy AABB and flag the ancestors as moved. Thread-safe using atomics.
func (tree *DynamicTree) markProxyMoved(proxyId int, aabb AABB) {
	validate(0 <= proxyId && proxyId < int(tree.ProxyCapacity), "0 <= proxyId && proxyId < tree->proxyCapacity")

	nodes := tree.Nodes
	parents := tree.Parents

	index := int(tree.Proxies[proxyId].Node)
	validate(0 <= index && index < int(tree.NodeEnd), "0 <= index && index < tree->nodeEnd")

	node := &nodes[index]
	validate(isLeaf(node), "b3IsLeaf( node )")
	validate(node.AABB.Contains(aabb) == false, "b3AABB_Contains( node->aabb, aabb ) == false")

	// This is not raced since it is the leaf.
	node.AABB = aabb
	node.FlagIndex |= movedNode

	index = int(parents[index])
	for index != NullIndex {
		node = &nodes[index]

		// Read first to avoid the FetchOr if possible.
		if atomicLoadU32Raw(&node.FlagIndex)&movedNode != 0 {
			break
		}

		previousFlags := atomicFetchOrU32(&node.FlagIndex, movedNode)
		if previousFlags&movedNode != 0 {
			// Ancestor already visited.
			break
		}

		index = int(parents[index])
	}
}

// clearMoved is b3DynamicTree_ClearMoved.
// Clear the moved flags from the entire tree.
func (tree *DynamicTree) clearMoved() {
	nodes := tree.Nodes

	root := &nodes[rootNode]
	if isNodeMoved(root) == false {
		return
	}

	root.FlagIndex &^= movedNode
	if isLeaf(root) {
		return
	}

	var stack [treeStackSize]int32
	stackCount := 0
	stack[stackCount] = int32(getLeftChild(root))
	stackCount++

	for stackCount > 0 {
		stackCount--
		pair := int(stack[stackCount])
		for i := 0; i < 2; i++ {
			node := &nodes[pair+i]
			if node.FlagIndex&movedNode != 0 {
				node.FlagIndex &^= movedNode
				if isLeaf(node) == false {
					if stackCount < treeStackSize {
						stack[stackCount] = int32(getLeftChild(node))
						stackCount++
					} else {
						// Bad stuff will happen if the moved flags don't get cleared.
						assert(stackCount < treeStackSize, "stackCount < B3_TREE_STACK_SIZE")
					}
				}
			}
		}
	}
}

// gatherMovedProxies is b3DynamicTree_GatherMovedProxies.
// PORT: int* proxyIds becomes a slice the caller sizes for every proxy.
func (tree *DynamicTree) gatherMovedProxies(proxyIds []int32) int {
	nodes := tree.Nodes
	root := &nodes[rootNode]
	if isNodeMoved(root) == false {
		return 0
	}

	if isLeaf(root) {
		proxyIds[0] = int32(getProxyId(root))
		return 1
	}

	count := 0
	var stack [treeStackSize]int32
	stackCount := 0
	stack[stackCount] = int32(getLeftChild(root))
	stackCount++

	for stackCount > 0 {
		stackCount--
		pair := int(stack[stackCount])
		for i := 0; i < 2; i++ {
			node := &nodes[pair+i]
			if isNodeMoved(node) == false {
				continue
			}

			if isLeaf(node) {
				proxyIds[count] = int32(getProxyId(node))
				count++
			} else {
				if stackCount < treeStackSize {
					stack[stackCount] = int32(getLeftChild(node))
					stackCount++
				} else {
					assert(stackCount < treeStackSize, "stackCount < B3_TREE_STACK_SIZE")
				}
			}
		}
	}

	return count
}

// Slow refit for unit tests.
func refitSubtree(nodes []TreeNode, rootIndex int) {
	root := &nodes[rootIndex]
	if isLeaf(root) || isNodeMoved(root) == false {
		return
	}

	// Negative entries indicate deferred unions. A parent is pushed before its children so the
	// LIFO order guarantees both children are finished when the union runs.
	var stack [treeStackSize]int32
	stackCount := 0
	// ~0 == -1
	stack[stackCount] = ^int32(rootIndex)
	stackCount++
	stack[stackCount] = int32(getLeftChild(root))
	stackCount++
	stack[stackCount] = int32(getLeftChild(root) + 1)
	stackCount++

	for stackCount > 0 {
		stackCount--
		item := stack[stackCount]
		if item < 0 {
			node := &nodes[^item]
			pair := getLeftChild(node)
			node.AABB = unionV(nodes[pair].AABB, nodes[pair+1].AABB)
			continue
		}

		node := &nodes[item]
		if isLeaf(node) || isNodeMoved(node) == false {
			continue
		}

		if stackCount+3 <= treeStackSize {
			pair := getLeftChild(node)
			stack[stackCount] = ^item
			stackCount++
			stack[stackCount] = int32(pair)
			stackCount++
			stack[stackCount] = int32(pair + 1)
			stackCount++
		} else {
			assert(stackCount+3 <= treeStackSize, "stackCount + 3 <= B3_TREE_STACK_SIZE")
		}
	}
}

// refit is b3DynamicTree_Refit.
func (tree *DynamicTree) refit() {
	if hasTreeMoved(tree) == false {
		return
	}

	nodes := tree.Nodes

	if tree.DfsOrdered == false {
		// This only happens in unit tests.
		refitSubtree(nodes, rootNode)
		return
	}

	for pair := int(tree.NodeEnd) - 2; pair >= 0; pair -= 2 {
		node := nodes[pair : pair+2]
		flags1 := node[0].FlagIndex
		flags2 := node[1].FlagIndex

		// Did either move?
		if ((flags1 | flags2) & movedNode) == 0 {
			continue
		}

		// Is the node internal and moved?
		refit1 := (flags1 & (leafNode | movedNode)) == movedNode
		refit2 := (flags2 & (leafNode | movedNode)) == movedNode

		// If the node needs a refit then get the child index. Otherwise use a valid dummy index.
		// The store function won't actually modify the node at the dummy index. This
		// avoids an unpredictable branch. SIMD FTW.
		children1 := pair
		if refit1 {
			children1 = int(flags1 & nodeIndexMask)
		}
		children2 := pair
		if refit2 {
			children2 = int(flags2 & nodeIndexMask)
		}

		// Conditionally store the union.
		storeAABBV(&node[0].AABB, unionPairV(nodes[children1:children1+2]), refit1)
		storeAABBV(&node[1].AABB, unionPairV(nodes[children2:children2+2]), refit2)
	}
}

// PORT: b3OpenTreeFile (fopen / fopen_s) is replaced by os.Create and os.ReadFile.

// treeHeaderBytes encodes the C image of a b3DynamicTree with the pointer slots zeroed.
// Offsets are those of the 64-bit C struct (see dynamicTreeCSize).
func treeHeaderBytes(tree *DynamicTree) []byte {
	b := make([]byte, dynamicTreeCSize)
	e := binary.NativeEndian
	e.PutUint64(b[0:], tree.Version)
	// 8 nodes, 16 parents, 24 proxies: zero
	e.PutUint32(b[32:], uint32(tree.NodeEnd))
	e.PutUint32(b[36:], uint32(tree.NodeCapacity))
	e.PutUint32(b[40:], uint32(tree.PairFreeList))
	e.PutUint32(b[44:], uint32(tree.ProxyCount))
	e.PutUint32(b[48:], uint32(tree.ProxyCapacity))
	e.PutUint32(b[52:], uint32(tree.ProxyFreeList))
	// 56 swapNodes .. 96 binIndices: zero
	e.PutUint32(b[104:], uint32(tree.RebuildCapacity))
	if tree.DfsOrdered {
		b[108] = 1
	}
	return b
}

func treeNodesBytes(nodes []TreeNode, count int) []byte {
	if count == 0 {
		return nil
	}
	return unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(nodes))), count*treeNodeCSize)
}

func treeInt32Bytes(values []int32, count int) []byte {
	if count == 0 {
		return nil
	}
	return unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(values))), count*4)
}

func treeProxiesBytes(proxies []TreeProxy, count int) []byte {
	if count == 0 {
		return nil
	}
	return unsafe.Slice((*byte)(unsafe.Pointer(unsafe.SliceData(proxies))), count*treeProxyCSize)
}

// Save is b3DynamicTree_Save: save the tree to a file. For debugging.
// PORT: the file has upstream's layout: the 64-bit C image of b3DynamicTree (pointers zeroed),
// then the node, parent and proxy arrays in native byte order.
func (tree *DynamicTree) Save(fileName string) {
	if tree.Nodes == nil || tree.Parents == nil || tree.Proxies == nil {
		return
	}

	file, err := os.Create(fileName)
	if err != nil {
		return
	}

	// Copy to allow setting some fields to zero
	temp := *tree

	// Zero pointers and temp data
	temp.Nodes = nil
	temp.Parents = nil
	temp.Proxies = nil
	temp.SwapNodes = nil
	temp.LeafIndices = nil
	temp.LeafNodes = nil
	temp.LeafBoxes = nil
	temp.LeafCenters = nil
	temp.BinIndices = nil
	temp.RebuildCapacity = 0

	temp.NodeCapacity = tree.NodeEnd

	_, _ = file.Write(treeHeaderBytes(&temp))

	if tree.NodeEnd > 0 {
		_, _ = file.Write(treeNodesBytes(tree.Nodes, int(tree.NodeEnd)))
		_, _ = file.Write(treeInt32Bytes(tree.Parents, int(tree.NodeEnd)))
	}

	if tree.ProxyCapacity > 0 {
		_, _ = file.Write(treeProxiesBytes(tree.Proxies, int(tree.ProxyCapacity)))
	}

	_ = file.Close()
}

// LoadDynamicTree is b3DynamicTree_Load: load a tree from a file. For debugging.
// The scale is applied to every node box.
// PORT: a free function like CreateDynamicTree; reads the C image written by Save.
func LoadDynamicTree(fileName string, scale float32) DynamicTree {
	var tree DynamicTree

	data, err := os.ReadFile(fileName)
	if err != nil {
		return tree
	}

	if len(data) < dynamicTreeCSize {
		return DynamicTree{}
	}

	e := binary.NativeEndian
	tree.Version = e.Uint64(data[0:])
	tree.NodeEnd = int32(e.Uint32(data[32:]))
	tree.NodeCapacity = int32(e.Uint32(data[36:]))
	tree.PairFreeList = int32(e.Uint32(data[40:]))
	tree.ProxyCount = int32(e.Uint32(data[44:]))
	tree.ProxyCapacity = int32(e.Uint32(data[48:]))
	tree.ProxyFreeList = int32(e.Uint32(data[52:]))
	tree.RebuildCapacity = int32(e.Uint32(data[104:]))
	tree.DfsOrdered = data[108] != 0
	data = data[dynamicTreeCSize:]

	if tree.Version != DynamicTreeVersion {
		return DynamicTree{}
	}

	tree.Nodes = nil
	tree.Parents = nil
	tree.Proxies = nil
	tree.SwapNodes = nil
	tree.LeafIndices = nil
	tree.LeafNodes = nil
	tree.LeafBoxes = nil
	tree.LeafCenters = nil
	tree.BinIndices = nil
	tree.RebuildCapacity = 0

	ok := tree.NodeCapacity >= 2 && tree.ProxyCapacity >= 1 && (tree.NodeEnd&1) == 0 && tree.NodeEnd >= 2 &&
		tree.NodeEnd <= tree.NodeCapacity

	if ok {
		tree.Nodes = make([]TreeNode, tree.NodeCapacity)
		tree.Parents = make([]int32, tree.NodeCapacity)
		tree.Proxies = make([]TreeProxy, tree.ProxyCapacity)

		// PORT: fread becomes copies out of the file image; a short image fails like a short read.
		readInto := func(dst []byte) bool {
			if len(data) < len(dst) {
				return false
			}
			copy(dst, data)
			data = data[len(dst):]
			return true
		}
		ok = readInto(treeNodesBytes(tree.Nodes, int(tree.NodeCapacity)))
		ok = ok && readInto(treeInt32Bytes(tree.Parents, int(tree.NodeCapacity)))
		ok = ok && readInto(treeProxiesBytes(tree.Proxies, int(tree.ProxyCapacity)))
	}

	if ok == false {
		return DynamicTree{}
	}

	// An empty node has infinite bounds, so the scale has to skip it.
	for i := int32(0); i < tree.NodeEnd; i++ {
		node := &tree.Nodes[i]
		if isEmptyNode(node) {
			continue
		}

		node.AABB.LowerBound = MulSV(scale, node.AABB.LowerBound)
		node.AABB.UpperBound = MulSV(scale, node.AABB.UpperBound)
	}

	return tree
}
