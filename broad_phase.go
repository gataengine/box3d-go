// Port of src/broad_phase.h and src/broad_phase.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- broad_phase.h ----

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.

// proxyType is B3_PROXY_TYPE( KEY ).
func proxyType(key int) BodyType {
	return BodyType(key & 3)
}

// proxyId is B3_PROXY_ID( KEY ).
func proxyId(key int) int {
	return key >> 2
}

// proxyKey is B3_PROXY_KEY( ID, TYPE ).
func proxyKey(id int, typ BodyType) int {
	return (id << 2) | int(typ)
}

// broadPhase is b3BroadPhase.
// The broad-phase is used for computing pairs and performing volume queries and ray casts.
// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
// It is up to the client to consume the new pairs and to track subsequent overlap.
type broadPhase struct {
	// One tree for each body type.
	trees [BodyTypeCount]DynamicTree

	// The moved siblings gathered from the dynamic body tree.
	// PORT: int* from the world stack allocator becomes a slice.
	movedSiblings []int32

	// Tracks shape pairs that have a b3Contact
	// todo pairSet can grow quite large on the first time step and remain large
	pairSet hashSet
}

// markProxyMovedSerial is b3BroadPhase_MarkProxyMovedSerial.
func (bp *broadPhase) markProxyMovedSerial(proxyKey int) {
	proxyType := proxyType(proxyKey)
	proxyId := proxyId(proxyKey)
	bp.trees[proxyType].markProxyMovedSerial(proxyId)
}

// markProxyMoved is b3BroadPhase_MarkProxyMoved.
func (bp *broadPhase) markProxyMoved(proxyKey int, aabb AABB) {
	proxyType := proxyType(proxyKey)
	proxyId := proxyId(proxyKey)
	bp.trees[proxyType].markProxyMoved(proxyId, aabb)
}

// ---- broad_phase.c ----

// createBroadPhase is b3CreateBroadPhase.
func createBroadPhase(bp *broadPhase, capacity *Capacity) {
	// PORT: _Static_assert( b3_bodyTypeCount == 3 ) as a constant expression.
	var _ = [1]struct{}{}[BodyTypeCount-3] // must be three body types

	bp.pairSet = createSet(int32(MaxInt(32, 2*int(capacity.ContactCount))))

	staticCapacity := MaxInt(16, int(capacity.StaticShapeCount))
	bp.trees[StaticBody] = CreateDynamicTree(staticCapacity)

	kinematicCapacity := 16
	bp.trees[KinematicBody] = CreateDynamicTree(kinematicCapacity)

	dynamicCapacity := MaxInt(16, int(capacity.DynamicShapeCount))
	bp.trees[DynamicBody] = CreateDynamicTree(dynamicCapacity)

	bp.movedSiblings = nil
}

// destroyBroadPhase is b3DestroyBroadPhase.
func destroyBroadPhase(bp *broadPhase) {
	for i := range BodyTypeCount {
		bp.trees[i].Destroy()
	}

	destroySet(&bp.pairSet)

	*bp = broadPhase{}
}

// createProxy is b3BroadPhase_CreateProxy.
func (bp *broadPhase) createProxy(proxyType BodyType, aabb AABB, categoryBits uint64, shapeIndex int,
	forcePairCreation bool) int {
	assert(0 <= proxyType && proxyType < BodyTypeCount, "0 <= proxyType && proxyType < b3_bodyTypeCount")

	mark := (proxyType != StaticBody || forcePairCreation)

	proxyId := createTreeProxyInternal(&bp.trees[proxyType], aabb, categoryBits, uint64(shapeIndex), mark)
	proxyKey := proxyKey(proxyId, proxyType)
	return proxyKey
}

// destroyProxy is b3BroadPhase_DestroyProxy.
func (bp *broadPhase) destroyProxy(proxyKey int) {
	proxyType := proxyType(proxyKey)
	proxyId := proxyId(proxyKey)

	assert(0 <= proxyType && proxyType < BodyTypeCount, "0 <= proxyType && proxyType < b3_bodyTypeCount")
	bp.trees[proxyType].DestroyProxy(proxyId)
}

// moveProxy is b3BroadPhase_MoveProxy.
func (bp *broadPhase) moveProxy(proxyKey int, aabb AABB) {
	proxyType := proxyType(proxyKey)
	proxyId := proxyId(proxyKey)

	mark := true
	bp.trees[proxyType].moveProxyInternal(proxyId, aabb, mark)
}

// gatherMovedSiblings is b3GatherMovedSiblings.
// Gather the sibling pairs with a moved node. This is done serially, it is cache friendly.
func gatherMovedSiblings(tree *DynamicTree, pairIndices []int32) int {
	nodes := tree.Nodes
	nodeEnd := int(tree.NodeEnd)

	count := 0

	// Skip the root.
	for pair := 2; pair < nodeEnd; pair += 2 {
		// Push when either sibling moved.
		if (nodes[pair].FlagIndex|nodes[pair+1].FlagIndex)&movedNode != 0 {
			pairIndices[count] = int32(pair)
			count++
		}
	}

	return count
}

// candidateBatch is B3_CANDIDATE_BATCH.
const candidateBatch = 32

// candidatePair is b3CandidatePair.
type candidatePair struct {
	shapeIdA int32
	shapeIdB int32
}

// pairContext is b3PairContext.
type pairContext struct {
	world      *world
	pairKeys   *[]uint64
	batch      [candidateBatch]candidatePair
	batchCount int

	// Compound shapes are static only, so only the static cross pass can meet one. Elsewhere
	// this stays false and the batch never reads a shape type.
	checkCompounds bool
}

// nodePair is b3NodePair.
type nodePair struct {
	a TreeNode
	b TreeNode
}

// indexPair is b3IndexPair.
type indexPair struct {
	a, b int32
}

// prefetchHash is b3PrefetchHash.
// PORT: b3Prefetch is dropped (no Go intrinsic), so this computes nothing; kept for the file
// mapping.
func prefetchHash(set *hashSet, hash uint32) {
}

// shouldCreatePair is b3ShouldCreatePair.
// The shape pair gauntlet. Everything here is a property of the pair, not of a compound child,
// so a compound runs it once for the whole pair.
func shouldCreatePair(world *world, shapeIdA, shapeIdB int) bool {
	shapeA := arrayGet(world.shapes, shapeIdA)
	shapeB := arrayGet(world.shapes, shapeIdB)

	bodyIdA := shapeA.bodyId
	bodyIdB := shapeB.bodyId

	// Are the shapes on the same body?
	if bodyIdA == bodyIdB {
		return false
	}

	// Sensors are handled elsewhere
	if shapeA.sensorIndex != NullIndex || shapeB.sensorIndex != NullIndex {
		return false
	}

	if shouldShapesCollide(shapeA.filter, shapeB.filter) == false {
		return false
	}

	// Does a joint override collision?
	bodyA := arrayGet(world.bodies, int(bodyIdA))
	bodyB := arrayGet(world.bodies, int(bodyIdB))
	if shouldBodiesCollide(world, bodyA, bodyB) == false {
		return false
	}

	// Custom user filter
	if (shapeA.flags&uint8(enableCustomFiltering)) != 0 || (shapeB.flags&uint8(enableCustomFiltering)) != 0 {
		customFilterFcn := world.customFilterFcn
		if customFilterFcn != nil {
			idA := ShapeID{int32(shapeIdA + 1), world.worldId, shapeA.generation}
			idB := ShapeID{int32(shapeIdB + 1), world.worldId, shapeB.generation}
			if customFilterFcn(idA, idB, world.customFilterContext) == false {
				return false
			}
		}
	}

	return true
}

// compoundPairContext is b3CompoundPairContext.
type compoundPairContext struct {
	pairContext     *pairContext
	pairSet         *hashSet
	compoundShapeId int
	otherShapeId    int
}

// compoundChildCallback is b3CompoundChildCallback.
func compoundChildCallback(proxyId int, userData uint64, context any) bool {
	compoundContext := context.(*compoundPairContext)
	childIndex := int(int32(userData))

	key := shapePairKey(compoundContext.compoundShapeId, compoundContext.otherShapeId, childIndex)
	if containsKey(compoundContext.pairSet, key) == false {
		arrayPush(compoundContext.pairContext.pairKeys, key)
	}

	return true
}

// emitCompoundPairs is b3EmitCompoundPairs.
// One of the two shapes is a compound, so the pair becomes one pair per overlapping child. The
// child boxes live in the compound frame, so the other shape's box has to be demoted and pulled
// into that frame first.
func emitCompoundPairs(context *pairContext, compoundShapeId, otherShapeId int) {
	world := context.world

	compoundShape := arrayGet(world.shapes, compoundShapeId)
	otherShape := arrayGet(world.shapes, otherShapeId)

	otherProxyKey := int(otherShape.proxyKey)
	assert(otherProxyKey != NullIndex, "otherProxyKey != B3_NULL_INDEX")
	otherAABB := world.broadPhase.trees[proxyType(otherProxyKey)].GetAABB(proxyId(otherProxyKey))

	// Query bounds are float world space, so the demoted transform is the matching float frame
	compoundTransform := ToRelativeTransform(getBodyTransform(world, int(compoundShape.bodyId)), PosZero)
	localAABB := otherAABB.Transform(InvertTransform(compoundTransform))

	compoundContext := compoundPairContext{
		pairContext:     context,
		pairSet:         &world.broadPhase.pairSet,
		compoundShapeId: compoundShapeId,
		otherShapeId:    otherShapeId,
	}

	startCount := len(*context.pairKeys)

	view := getCompoundTreeView(compoundShape.compound)
	view.query(localAABB, DefaultMaskBits, false, compoundChildCallback, &compoundContext)

	if len(*context.pairKeys) > startCount && shouldCreatePair(world, compoundShapeId, otherShapeId) == false {
		arrayResize(context.pairKeys, startCount)
	}
}

// flushCandidatePairs is b3FlushCandidatePairs.
func flushCandidatePairs(context *pairContext) {
	world := context.world
	bp := &world.broadPhase

	count1 := context.batchCount
	context.batchCount = 0

	var plain [candidateBatch]candidatePair
	plainCount := 0

	if context.checkCompounds {
		// A compound expands into one pair per child, so its key is not known yet and it cannot
		// ride the batched cull below.
		shapes := world.shapes
		for i := 0; i < count1; i++ {
			candidate := context.batch[i]
			compoundA := shapes[candidate.shapeIdA].typ == CompoundShape
			compoundB := shapes[candidate.shapeIdB].typ == CompoundShape

			if compoundA {
				// Compound versus compound is not supported
				assert(compoundB == false, "compoundB == false")
				emitCompoundPairs(context, int(candidate.shapeIdA), int(candidate.shapeIdB))
			} else if compoundB {
				emitCompoundPairs(context, int(candidate.shapeIdB), int(candidate.shapeIdA))
			} else {
				plain[plainCount] = candidate
				plainCount += 1
			}
		}
	} else {
		copy(plain[:count1], context.batch[:count1])
		plainCount = count1
	}

	// Prefetch hash set entries. Less than 1% gain in Box2D but the 3D shape is fatter.
	var keys [candidateBatch]uint64
	var hashes [candidateBatch]uint32
	for i := 0; i < plainCount; i++ {
		keys[i] = shapePairKey(int(plain[i].shapeIdA), int(plain[i].shapeIdB), 0)
		hashes[i] = keyHash(keys[i])
		prefetchHash(&bp.pairSet, hashes[i])
	}

	// Cull existing pairs.
	var candidates [candidateBatch]candidatePair
	count2 := 0
	for i := 0; i < plainCount; i++ {
		if containsHashedKey(&bp.pairSet, keys[i], hashes[i]) == false {
			candidates[count2] = plain[i]
			keys[count2] = keys[i]
			count2 += 1
		}
	}

	// Prefetch shapes.
	// PORT: b3Prefetch is dropped, so the prefetch loop over the candidate shapes is too.

	// Filter candidates.
	for i := 0; i < count2; i++ {
		if shouldCreatePair(world, int(candidates[i].shapeIdA), int(candidates[i].shapeIdB)) {
			// The pair passed the gauntlet. A new contact will be created.
			arrayPush(context.pairKeys, keys[i])
		}
	}
}

// addCandidatePair is b3AddCandidatePair.
func addCandidatePair(shapeIdA, shapeIdB int, context *pairContext) {
	// Follow shape index order.
	candidate := &context.batch[context.batchCount]
	candidate.shapeIdA = int32(MinInt(shapeIdA, shapeIdB))
	candidate.shapeIdB = int32(MaxInt(shapeIdA, shapeIdB))
	context.batchCount += 1
	if context.batchCount == candidateBatch {
		flushCandidatePairs(context)
	}
}

// testPair is b3TestPair.
// Did either move and if so do they overlap?
func testPair(a, b *TreeNode) bool {
	if ((a.FlagIndex | b.FlagIndex) & movedNode) == 0 {
		return false
	}

	return overlapV(&a.AABB, &b.AABB)
}

// collideProxyAndSubtree is b3CollideProxyAndSubtree.
// PORT: node->shapeIndex is the Height field of TreeNode (the union member for leaves).
func collideProxyAndSubtree(proxy *TreeNode, nodes []TreeNode, pair int, context *pairContext) {
	proxyMark := proxy.FlagIndex & movedNode
	boxv := loadAABBV(&proxy.AABB)
	shapeId := int(proxy.Height)

	var stack [treeStackSize]int32
	stackCount := 0
	stack[stackCount] = int32(pair)
	stackCount++

	for stackCount > 0 {
		stackCount--
		pair = int(stack[stackCount])
		for i := 0; i < 2; i++ {
			node := &nodes[pair+i]
			if ((node.FlagIndex | proxyMark) & movedNode) == 0 {
				continue
			}

			if overlapNode(boxv, node) == false {
				continue
			}

			if isLeaf(node) {
				addCandidatePair(shapeId, int(node.Height), context)
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
}

// visitPair is b3VisitPair.
// Helper for b3CollideCrossPairs to avoid code duplication.
func visitPair(arrayA, arrayB []TreeNode, nodeA, nodeB *TreeNode, stack *[treeStackSize]indexPair, stackCount *int,
	context *pairContext) {
	if testPair(nodeA, nodeB) == false {
		return
	}

	leafA := isLeaf(nodeA)
	leafB := isLeaf(nodeB)
	if leafA && leafB {
		addCandidatePair(int(nodeA.Height), int(nodeB.Height), context)
	} else if leafA {
		collideProxyAndSubtree(nodeA, arrayB, getLeftChild(nodeB), context)
	} else if leafB {
		collideProxyAndSubtree(nodeB, arrayA, getLeftChild(nodeA), context)
	} else {
		if *stackCount < treeStackSize {
			stack[*stackCount] = indexPair{a: int32(getLeftChild(nodeA)), b: int32(getLeftChild(nodeB))}
			*stackCount += 1
		} else {
			assert(*stackCount < treeStackSize, "*stackCount < B3_TREE_STACK_SIZE")
		}
	}
}

// collideCrossPairs is b3CollideCrossPairs.
// This collides two sub-trees against each other. They can live in the same dynamic tree.
// This can only generate pairs cross sub-tree, but not within a sub-tree. This fact means
// this does not generate duplicate pairs.
// For example consider the full binary tree A (B (D  E) C (F G))
// Colliding children of A (B and C) can give pairs (D,F) (D,G) (E,F) and (E,G).
// Then colliding children of B can give the pair (D,E) and for C (F,G).
// So no duplicates even when used for self-collision.
// Whenever a proxy is moved, the flag is propagated up the hierarchy to the root. So
// self collision gathers all those moved internal nodes and collides their subtrees together.
// See Real-time collision detection section 6.3.2. This is faster than querying every moved
// proxy against the whole tree. Scaling is linear instead of linear * log.
// Many other physics engines do this (Bepu, Rapier, etc). So nothing new here.
func collideCrossPairs(arrayA, arrayB []TreeNode, subtreeA, subtreeB *TreeNode, context *pairContext) {
	var stack [treeStackSize]indexPair
	stackCount := 0

	// Seed the stack.
	visitPair(arrayA, arrayB, subtreeA, subtreeB, &stack, &stackCount, context)

	for stackCount > 0 {
		stackCount--
		pair := stack[stackCount]
		for i := 0; i < 2; i++ {
			for j := 0; j < 2; j++ {
				visitPair(arrayA, arrayB, &arrayA[int(pair.a)+i], &arrayB[int(pair.b)+j], &stack, &stackCount, context)
			}
		}
	}
}

// selfPairsTask is b3SelfPairsTask.
// This takes moved internal nodes and collides their sub-trees against each other.
func selfPairsTask(startIndex, endIndex, workerIndex int, context any) {
	world := context.(*world)
	bp := &world.broadPhase
	tree := &bp.trees[DynamicBody]
	nodes := tree.Nodes
	siblingIndices := bp.movedSiblings

	pairContext := pairContext{
		world:    world,
		pairKeys: &world.taskContexts[workerIndex].pairKeys,
	}

	for i := startIndex; i < endIndex; i++ {
		nodeIndex := int(siblingIndices[i])
		collideCrossPairs(nodes, nodes, &nodes[nodeIndex], &nodes[nodeIndex+1], &pairContext)
	}

	flushCandidatePairs(&pairContext)
}

// crossSeedCount is B3_CROSS_SEED_COUNT.
const crossSeedCount = 64

// PORT: _Static_assert( ( B3_CROSS_SEED_COUNT & ( B3_CROSS_SEED_COUNT - 1 ) ) == 0 ) as a
// constant expression.
var _ = [1]struct{}{}[crossSeedCount&(crossSeedCount-1)] // must be power of 2

// gatherCrossSeeds is b3GatherCrossSeeds.
// This does a serial cross-tree breadth first search until the queue is full. Then it returns
// the queue pairs as seeds for a parallel search.
func gatherCrossSeeds(treeA, treeB *DynamicTree, seeds []nodePair) int {
	nodesA := treeA.Nodes
	nodesB := treeB.Nodes

	// Breadth-first search from the two roots as one pair. An empty tree has a sentinel root that
	// survives nothing.
	var queue [2 * crossSeedCount]nodePair
	mask := 2*crossSeedCount - 1
	head := 0
	tail := 0

	rootA := &nodesA[rootNode]
	rootB := &nodesB[rootNode]
	if testPair(rootA, rootB) {
		queue[tail&mask] = nodePair{a: *rootA, b: *rootB}
		tail += 1
	}

	seedCount := 0
	for head < tail && seedCount+(tail-head)+3 < crossSeedCount {
		pair := queue[head&mask]
		head += 1

		if isLeaf(&pair.a) || isLeaf(&pair.b) {
			seeds[seedCount] = pair
			seedCount++
			continue
		}

		a := nodesA[getLeftChild(&pair.a):]
		b := nodesB[getLeftChild(&pair.b):]

		// Nodes have two children each, so four combinations.
		for i := 0; i < 2; i++ {
			for j := 0; j < 2; j++ {
				if testPair(&a[i], &b[j]) {
					queue[tail&mask] = nodePair{a[i], b[j]}
					tail += 1
				}
			}
		}
	}

	for head < tail {
		seeds[seedCount] = queue[head&mask]
		seedCount += 1
		head += 1
	}

	return seedCount
}

// crossContext is b3CrossContext.
type crossContext struct {
	world           *world
	seeds           []nodePair
	staticSeedCount int
	checkCompounds  bool
}

// crossPairsTask is b3CrossPairsTask.
func crossPairsTask(startIndex, endIndex, workerIndex int, context any) {
	crossContext := context.(*crossContext)
	world := crossContext.world
	bp := &world.broadPhase
	staticNodes := bp.trees[StaticBody].Nodes
	kinematicNodes := bp.trees[KinematicBody].Nodes
	dynamicNodes := bp.trees[DynamicBody].Nodes

	pairContext := pairContext{
		world:    world,
		pairKeys: &world.taskContexts[workerIndex].pairKeys,
	}

	// Static seeds come first, so the two passes are contiguous sub-ranges. The batch is drained
	// between them because only the static pass can meet a compound.
	staticEnd := MinInt(endIndex, crossContext.staticSeedCount)

	pairContext.checkCompounds = crossContext.checkCompounds
	for i := startIndex; i < staticEnd; i++ {
		seed := crossContext.seeds[i]
		collideCrossPairs(dynamicNodes, staticNodes, &seed.a, &seed.b, &pairContext)
	}
	flushCandidatePairs(&pairContext)

	pairContext.checkCompounds = false
	for i := MaxInt(startIndex, crossContext.staticSeedCount); i < endIndex; i++ {
		seed := crossContext.seeds[i]
		collideCrossPairs(dynamicNodes, kinematicNodes, &seed.a, &seed.b, &pairContext)
	}
	flushCandidatePairs(&pairContext)
}

// updateTreesTask is b3UpdateTreesTask.
func updateTreesTask(context any) {
	world := context.(*world)
	world.broadPhase.trees[DynamicBody].Rebuild(false)
	world.broadPhase.trees[KinematicBody].Rebuild(false)
}

// enqueueTreeUpdate is b3EnqueueTreeUpdate.
// Task that can be done in parallel with the narrow-phase
// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
func enqueueTreeUpdate(world *world) {
	if world.taskCount < MaxTasks {
		world.userTreeTask = world.enqueueTaskFcn(updateTreesTask, world, world.userTaskContext, "rebuild tree")
		world.taskCount += 1
		if world.userTreeTask != nil {
			world.activeTaskCount += 1
		}
	} else {
		world.userTreeTask = nil
		updateTreesTask(world)
	}
}

// updateBroadPhasePairs is b3UpdateBroadPhasePairs.
func updateBroadPhasePairs(world *world) {
	bp := &world.broadPhase

	needUpdate := hasTreeMoved(&bp.trees[StaticBody])
	needUpdate = needUpdate || needsRebuild(&bp.trees[KinematicBody])
	needUpdate = needUpdate || needsRebuild(&bp.trees[DynamicBody])

	if needUpdate == false {
		return
	}

	alloc := &world.stack

	for i := 0; i < int(world.workerCount); i++ {
		arrayClear(&world.taskContexts[i].pairKeys)
	}

	// Generate pairs by querying the dynamic body tree against itself and against
	// the kinematic and static trees.
	{
		// Get the sibling pairs of the dynamic body tree that have moved.
		dynamicTree := &bp.trees[DynamicBody]
		pairCapacity := MaxInt(int(dynamicTree.NodeEnd)/2, 1)
		bp.movedSiblings = stackAlloc[int32](alloc, pairCapacity, "moved pairs")
		dynamicMoveCount := gatherMovedSiblings(dynamicTree, bp.movedSiblings)

		// Get seeds for colliding against the static and kinematic trees.
		var crossSeeds [2 * crossSeedCount]nodePair
		staticSeedCount := gatherCrossSeeds(dynamicTree, &bp.trees[StaticBody], crossSeeds[:])
		assert(staticSeedCount <= crossSeedCount, "staticSeedCount <= B3_CROSS_SEED_COUNT")
		kinematicSeedCount := gatherCrossSeeds(dynamicTree, &bp.trees[KinematicBody], crossSeeds[staticSeedCount:])
		assert(kinematicSeedCount <= crossSeedCount, "kinematicSeedCount <= B3_CROSS_SEED_COUNT")
		crossMoveCount := staticSeedCount + kinematicSeedCount

		// Collide the dynamic body tree against the static and kinematic trees.
		crossContext := crossContext{
			world:           world,
			seeds:           crossSeeds[:],
			staticSeedCount: staticSeedCount,
			checkCompounds:  world.compoundShapeCount > 0,
		}
		parallelFor(world, crossPairsTask, crossMoveCount, 1, &crossContext, "cross pairs")

		// Collide the dynamic body tree against itself.
		parallelFor(world, selfPairsTask, dynamicMoveCount, 64, world, "self pairs")
	}

	bp.trees[StaticBody].clearMoved()

	// Update stale trees.
	enqueueTreeUpdate(world)

	// Pairs arrive in deterministic order but scrambled relative to body and shape order.
	// Sorting them here improves solver performance and makes contact order independent of
	// tree structure.
	pairCount := 0
	for i := 0; i < int(world.workerCount); i++ {
		pairCount += len(world.taskContexts[i].pairKeys)
	}

	pairKeys := stackAlloc[uint64](alloc, MaxInt(pairCount, 1), "pair keys")
	keyCount := 0
	for i := 0; i < int(world.workerCount); i++ {
		workerKeys := world.taskContexts[i].pairKeys
		if len(workerKeys) > 0 {
			copy(pairKeys[keyCount:], workerKeys)
			keyCount += len(workerKeys)
		}
	}

	assert(keyCount == pairCount, "keyCount == pairCount")

	{
		// PORT: the LESS and SWAP macros become closures.
		qsort(pairCount, func(i, j int) bool {
			return pairKeys[i] < pairKeys[j]
		}, func(i, j int) {
			pairKeys[i], pairKeys[j] = pairKeys[j], pairKeys[i]
		})
	}

	for i := 0; i < keyCount; i++ {
		key := pairKeys[i]

		// The traversal is cross subtree and the seeds are disjoint, so a duplicate key means a
		// real bug. Trap it in a validation build, and skip rather than double add to the pair
		// set, which a later destroy would then remove once.
		duplicate := i > 0 && key == pairKeys[i-1]
		validate(duplicate == false, "duplicate == false")
		if duplicate {
			continue
		}

		shapeIdA := int((key >> (64 - ShapePower)) & shapeMask)
		shapeIdB := int((key >> (64 - 2*ShapePower)) & shapeMask)
		childIndex := int(key & childMask)

		shapeA := arrayGet(world.shapes, shapeIdA)
		shapeB := arrayGet(world.shapes, shapeIdB)

		createContact(world, shapeA, shapeB, childIndex)
	}

	stackFree(alloc, pairKeys)

	stackFree(alloc, bp.movedSiblings)
	bp.movedSiblings = nil

	validateSolverSets(world)
}

// testOverlap is b3BroadPhase_TestOverlap.
func (bp *broadPhase) testOverlap(proxyKeyA, proxyKeyB int) bool {
	typeIndexA := proxyType(proxyKeyA)
	proxyIdA := proxyId(proxyKeyA)
	typeIndexB := proxyType(proxyKeyB)
	proxyIdB := proxyId(proxyKeyB)

	aabbA := bp.trees[typeIndexA].GetAABB(proxyIdA)
	aabbB := bp.trees[typeIndexB].GetAABB(proxyIdB)
	return aabbA.Overlaps(aabbB)
}

// getShapeIndex is b3BroadPhase_GetShapeIndex.
func (bp *broadPhase) getShapeIndex(proxyKey int) int {
	typeIndex := proxyType(proxyKey)
	proxyId := proxyId(proxyKey)

	return int(bp.trees[typeIndex].GetUserData(proxyId))
}

// validateBroadPhase is b3ValidateBroadPhase.
func validateBroadPhase(bp *broadPhase) {
	bp.trees[DynamicBody].Validate()
	bp.trees[KinematicBody].Validate()

	// todo validate every shape AABB is contained in tree AABB
}

// validateNoMoved is b3ValidateNoMoved.
func validateNoMoved(bp *broadPhase) {
	if !validateEnabled {
		return
	}

	for j := range BodyTypeCount {
		tree := &bp.trees[j]
		tree.ValidateNoMoved()
	}
}
