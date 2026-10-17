# Loose ends

Every place the Go port is not a literal transliteration of upstream Box3D
(`efca0c184d40fd6a82016983f1601c4d23965867`). Each entry has a matching `// PORT:` comment at the
site. Upstream bugs are ported as is and listed here too.

| Go file | upstream site | what differs | why |
|---|---|---|---|
| test_macros_test.go | `RUN_TEST`, `RUN_SUBTEST` | replaced by Ginkgo `Describe`/`It` | Go test runner |
| box3d_suite_test.go | `test/main.c` | filter argument is `-ginkgo.focus`; Windows CRT and Tracy setup dropped | Go test runner |
| rapidhash.go | `rapid_read*`, pointer `p` | `p` is an offset into the key slice; only COMPACT + FAST variants ported | Go slices cannot index before their start; upstream compiles only these variants |
| base.go | `B3_ASSERT`, `B3_VALIDATE` | `assert(cond, "cond text")` / `validate(...)`; build tags `box3d_noassert`, `box3d_validate`; the condition is still evaluated when compiled out | Go has no macros; conditions must stay side-effect free |
| base.go | `B3_BREAKPOINT` | panics after the assert handler returns non-zero | no portable debug trap in Go |
| base.go, core.go | `b3AllocFcn`, `b3FreeFcn`, `b3SetAllocator`, `b3Alloc`, `b3Free`, `b3GrowAlloc*`, `b3AllocZero`, `B3_ALLOC/FREE/GROW*` | not ported; `GetByteCount` always returns 0 | memory is owned by the Go GC (spec: allocators dropped) |
| core.go | `b3Log` | no 512 byte truncation | `fmt.Sprintf` |
| core.go | `b3IsDoublePrecision`, `BOX3D_DOUBLE_PRECISION` | always false; double precision not ported | spec non-goal |
| core.go | platform/CPU/SIMD/compiler macros, Tracy, `B3_RESTRICT`, `B3_UNUSED`, `b3StrCpy` | dropped | no Go counterpart / unused |
| core.go | `B3_DEBUG` | `debugBuild` equals `assertEnabled` (NDEBUG ≙ `box3d_noassert`) | one build switch |
| core.go, platform.go | `b3AtomicInt/U32/I64` and helpers | aliases of `sync/atomic` types; helpers kept; `b3AtomicLoadU32Raw` is a seq-cst load | spec: atomics → sync/atomic; Go has no relaxed load |
| platform.go | `b3Prefetch` | dropped | no Go intrinsic |
| constants.go | `B3_HUGE`, `B3_LINEAR_SLOP` and other macros using `b3GetLengthUnitsPerMeter()` | functions (`Huge()`, `LinearSlop()`, …) | runtime value |
| constants.go | `#ifndef` overridable limits (`B3_MAX_MANIFOLD_POINTS`, `B3_MAX_WORLDS`, …) | fixed constants | no compile-time override in Go |
| math_functions.go | every `a + b*c` | products feeding an add/sub are wrapped in `float32()` or use `mulAdd`/`mulSub` | Go fuses into FMA on arm64; upstream uses `-ffp-contract=off`. Checked by `fma_guard_test.go` |
| math_functions.go | `sqrtf`, `remainderf` | float64 `math.Sqrt` / `math.Remainder` rounded to float32 | both exact (correctly rounded / exact remainder) |
| math_functions.go | `b3GetLengthAndNormalize(&length, v)`, `b3GetAxisAngle(&radians, q)` | out-parameter becomes the second return value | Go multiple returns |
| math_functions.go | `b3MakeAABB(points, count, radius)` | `MakeAABB(points []Vec3, radius)` | pointer + count parameter becomes a slice |
| math_functions.go | `b3AABB_*` | methods on `AABB` (`a.Contains(b)`, `a.Transform(t)`, …) | spec naming rule `b3Type_Method` |
| math_functions.go | `b3Pos`, `b3WorldTransform` | distinct structs with the float layout, not aliases | spec: keep `Pos` distinct for a later double mode |
| math_functions.go | `b3RoundDownFloat(double)`, `b3RoundUpFloat(double)` | take float32 | double precision not ported |
| math_functions.go | C++ operator overloads | not ported | no operator overloading in Go |
| math_internal.go | declarations of functions defined in other files | ported with the defining file | file mapping |
| types.go | public structs | `void*` → `any`, `const char*` → `string` (empty = NULL), pointer + count → slice + count field, stored `int` → int32, callback `int` params → `int`, `internalValue` unexported | translation rules |
| types.go | `b3WorldCastOutput` | distinct struct with `Point Pos` instead of a typedef of `b3CastOutput` | same reason as `Pos` |
| types.go | `b3TreeNode` union `height`/`shapeIndex` | single `Height int32` field | Go has no unions; same storage |
| types.go | `b3MeshNode` bit-field union | single `Data uint32` (low 2 bits axis/type, high 30 bits offset/count) | Go has no bit fields; accessors with mesh.c |
| types.go | `b3ChildShape`, `b3DebugShape` unions | all members present; the type tag says which is valid | Go has no unions |
| types.go | `b3HullData`, `b3BoxHull`, `b3MeshData`, `b3HeightFieldData`, `b3CompoundData` | header fields only; byte-image ownership and offset accessors decided in milestone 2 | spec leaves blob typing to the builder |
| types.go, mesh.go | `b3MeshDef.vertices` + `stride` | `Vertices []Vec3` is the base of the strided data and `Stride` stays a byte stride (0 = dense); elements are read `stride` bytes apart through `unsafe.Add`, so the slice must span every strided byte (asserted). Interleaved callers pass `unsafe.Slice(&fat[0].position, n)` (see `fatPositions` in mesh_test.go) | no pointer arithmetic on Go slices; keeps upstream's stride semantics |
| types.go | `b3PlaneResultFcn( shapeId, plane, planeCount, context )` | `func(shapeId, planes []PlaneResult, context)` | pointer + count parameter becomes a slice |
| math_test.go | `cosf`, `sinf`, `atan2f` reference values | float64 `math` rounded to float32 | Go has no float32 libm; reference only |
| utils_test.go | `shared/utils.h` | test-only helpers in package box3d; `RandomIntRange` keeps the float return | upstream tests are internal tests |
| ctz.go | MSVC/GCC intrinsic variants | `math/bits` | same instructions; `clz32(0)` is 32 (UB in the GCC path) |
| bitset.go | `b3BitSet.bits` | `[]uint64` of length `blockCapacity`; destroy drops the slice | allocators dropped |
| bitset_test.go | `floorf( log2f( (float)i ) )` | float64 `math.Floor(math.Log2(...))` | no float32 libm; exact for these integers |
| container.go | `b3Array( T )`, `b3Array_*` macros | `[]T` (count = len, capacity = cap) with generic `arrayPush`, `arrayPop`, `arrayGet`, `arrayEmplace`, `arrayAddIndex`, `arrayAppend`, `arrayReserve`, `arrayResize`, `arrayRemoveSwap`, `arrayClear`, `arrayMemZero`, `arrayByteCount`; growth policies kept; emplaced elements are zeroed | spec: slices replace b3Array |
| timer.go | per-platform `b3GetTicks`, threads, mutex, semaphore | one Go implementation: ticks are monotonic ns since package init, threads are goroutines (names stored only), mutex/semaphore wrap `sync` | Go runtime is the platform layer |
| timer.go | `b3Hash( hash, data, count )` | `Hash(hash, data []byte)`; always little endian loads | pointer + count is a slice; matches upstream's big-endian byte swap |
| table.go | `b3_probeCount`, `#if B3_DEBUG` | counted when `debugBuild` (assertions on) | NDEBUG ≙ `box3d_noassert` |
| table_test.go | `printf` timing lines | `GinkgoWriter.Printf` | shown with `-v` / on failure |
| id_pool.go | `b3FreeId` todo comment | ported as is: the `id == nextIndex` branch is unreachable under the assertion above it | upstream bug kept |
| arena_allocator.go | `b3Stack`, `b3Arena`, `b3StackAlloc( stack, size, name )`, `b3Bump( arena, size )` | typed `stackAlloc[T](s, count, name)`, `bump[T](a, count)` return zeroed slices from `make`; index/watermark/overflow accounting kept so counters and grow decisions match; no backing memory | spec: allocators dropped. Revisit for no-allocation `Step` |
| block_allocator.go | `b3BlockAllocator` | generic `blockAllocator[T]`; elements are `[]T` of `elementSize` values (not bytes); stride not rounded to 16 bytes; the intrusive free list is a LIFO slice stack (same reuse order) | allocators dropped; GC-safe typed memory |
| allocator_test.go | `TestBlockAlignment` byte sizes and `B3_BLOCK_ALIGNMENT` check | element types `byte`×12, `Manifold`×1, `Manifold`×3; alignment checked against `unsafe.Alignof(T)` | issue #78 (free-list link in the element) cannot occur in Go |
| aabb.go | `b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction )` | returns `(hit, minFraction, maxFraction)`; fractions are 0 on a miss | out-parameters become trailing returns |
| collision.go | `b3DynamicTree_GetUserData`, `b3DynamicTree_GetAABB` | methods on `*DynamicTree` | spec naming rule `b3Type_Method` |
| simd_scalar.go, simd.go, simd_wide.go | `simd_neon.h`, `simd_sse2.h`, `simd_avx2.h`, `B3_WIDE`, W8 functions | scalar lane backend only; `W4` suffix dropped (`b3AddW4` → `addW`); `floatW` is `struct{ v [laneCount]float32 }` | spec: scalar lanes first, SIMD backend later behind a build tag |
| simd_scalar.go | `b3LoadW4( const float* )`, `b3StoreW4`, `b3SoftMaskW4( const int* … )`, `b3UnionPairV( const b3TreeNode* )` | slices instead of pointers | Go has no pointer arithmetic |
| simd_scalar.go | `b3TransposeW4( …, b3FloatW4* c0, … )` | returns `(c0, c1, c2, c3)` | out-parameters become returns |
| simd.go | `b3HasAVX2`, `b3CpuId`, `b3GetExtendedControlRegister` | not ported; `detectSIMDWidth` returns `laneCount`, `IsAVX2Available` is false, `SetSIMDWidth(8)` stores 4 | width 8 not ported |
| simd.go | `b3128` union, `b3GetV` | switch on the index | Go has no unions |
| math_functions.go, math_internal.go, constants.go | helpers returning a bare product (`b3MulSV`, `b3Mul`, `b3MulSM`, `B3_LINEAR_SLOP`, …) | the returned product is wrapped in `float32()` | after inlining Go fused the product with the caller's add (found by the FMA guard when distance.go landed); values unchanged |
| distance.go | `b3ShapeDistance( input, cache, simplexes, simplexCapacity )` | `ShapeDistance(input, cache, simplexes []Simplex)`; capacity is `len(simplexes)`, nil for none | pointer + count is a slice |
| distance.go | `b3ComputeWitnessPoints( simplex, &a, &b )`, `b3FindMinSeparation( fcn, &indexA, &indexB, t )` | out-parameters become returns; the unreachable default case returns indices 0 | Go multiple returns |
| distance.go | `b3GetPointSupport( points, count, axis )` | `getPointSupport(points []Vec3, count int, axis)` | slice + explicit count (proxies carry a count) |
| distance.go | `b3SeparationFunction.type` | field `typ` | `type` is a Go keyword |
| distance.go | `#if B3_ENABLE_VALIDATION && 0` block in `b3TimeOfImpact` | not ported | compiled out upstream |
| verstable.go | `verstable.h` (Verstable v2.2.1) | API-level port: generic `verstable[K, V]` with linear probing and backward shift deletion; bucket placement, iteration order and `total_alloc_size` differ; `erase_itr` returns nothing | third party macro library; upstream only does lookups/inserts and an order independent sum |
| collision.go | geometry blob layout (`b3HullData` …), `(intptr_t)hull + offset` accessors | blob = one `[]uint64` allocation (8-byte aligned, pointer free) viewed through the header pointer; `blobSlice[T]` uses `unsafe.Add` from the header; accessors return slices with the element count (`3 * stride` for SoA arrays, `edgeCount / 2` for cosines); a zero count gives a non-nil empty slice (an empty array can sit one past the end of the blob, which checkptr rejects) | spec: byte image canonical; decision recorded in PROGRESS.md |
| collision.go, hull.go | `b3GetHullVertices` … return `const T*` | return writable slices; the `*Write` variants are thin wrappers | Go has no const slices |
| hull.go | `b3Alloc`/`b3Free` of hulls, `b3DestroyHull` | `blobAlloc`; `DestroyHull` is a no-op | GC owns memory |
| hull.go | `b3HullBuilder` single work block (`b3HullBuilder_Init( b, mem, sizes )`) | typed slices per pool with the same capacities; `computeHullWorkSizes` keeps the byte offsets (Go sizes); `hullBuilderInit` returns the shifted points buffer | allocators dropped |
| hull.go | `(b3QHVertex*)node`, `(b3QHFace*)node` intrusive list casts | `vertexOfNode` / `faceOfNode` / `nodeOfFace` unsafe casts (link is the first field) | keeps the intrusive lists literal |
| hull.go | `#if B3_DEBUG` consistency checks, `#if B3_ENABLE_VALIDATION` `b3IsValidHull` | `if debugBuild` / `if !validateEnabled { return true }` | build tags instead of macros |
| hull.go | `_Static_assert( sizeof( b3HullData ) == 152 )`, `== 696` | overflowing `uintptr` constant pair | Go has no static assert |
| hull.go | `b3FindFarthestPointsAlongCardinalAxes( &i1, &i2, … )` | returns `(index1, index2)` | out-parameters become returns |
| hull.go | `b3CreateHull( points, pointCount, max )`, `b3Hull2D( pts, count, hull )`, `b3SimplifyHull2D( hull, count, target )` | slices plus the explicit count | count may be less than the slice length (tests rely on it) |
| hull.go | unrolled SoA copies in `b3MakeTransformedBoxHull` | loops over the same indices | same stores, same order |
| hull.go | `memmove` in `b3SimplifyHull2D` | `copy` | same semantics |
| sphere.go | `( b3ShapeProxy ){ &shape->center, 1, … }` | `spherePoints(shape)` = `unsafe.Slice(&shape.Center, 1)` | no allocation, aliases the shape as in C |
| sphere.go | `4.0f / 3.0f * B3_PI * …` | `fourThirds := float32(4.0 / 3.0)` then run-time multiplies | Go folds constant chains exactly (one rounding); C rounds each step |
| capsule.go, hull.go | `( b3ShapeProxy ){ &capsule->center1, 2, … }` | `capsulePoints(c)` = `unsafe.Slice(&c.Center1, 2)` spanning the adjacent Center1/Center2 fields | no allocation, aliases the capsule as in C |
| manifold.go | `b3SeparatingAxis.type` | field `typ` | `type` is a Go keyword |
| manifold.go | `#if B3_ENABLE_VALIDATION b3ValidatePolygon` | always compiled; the call is `validate(!validateEnabled \|\| validatePolygon(...))` so it is skipped when validation is off | `validate` evaluates its argument |
| manifold.go | `b3ClipPolygon( out, polygon, count, … )` | slices plus the explicit count | pointer + count |
| shape.go | `b3Shape` anonymous union `{ capsule; sphere; hull; mesh; heightField; compound }` | all members present; `typ` (the shape type) says which is valid | Go has no unions |
| shape.go | `b3Shape.type`, `b3Shape.materials`, `userData`, `userShape` | field `typ`; `materials` is a `[]SurfaceMaterial` (nil for single material shapes); `void*` → `any` | `type` is a Go keyword; pointer + count is a slice |
| shape.go | `b3GetShapeMaterials` `(b3SurfaceMaterial*)&shape->material` | `unsafe.Slice(&shape.material, 1)` | presents the inline material as a one element slice without copying, as in C |
| shape.go | b3MakeLocalProxy( proxy, transform, b3Vec3* buffer ) | buffer is a slice | pointer + count is a slice |
| shape.go | `b3ComputeFatShapeAABB` `#if defined( BOX3D_DOUBLE_PRECISION )` branch | only the float branch; the world transform is converted with `Transform{ToVec3(p), q}` | double precision not ported; upstream's float `b3WorldTransform` is a typedef of `b3Transform` |
| simd_scalar.go | `B3_SIMD_NONE` (core.h) | `var simdNone = true`; kernels with a `#if defined( B3_SIMD_NONE )` variant port both variants and branch on it | the scalar backend is upstream's no-SIMD configuration; a variable so tests can run the lane variant too |
| convex_manifold.go | `b3ClipSegment( b3ClipVertex segment[2], … )`, `b3ClipSegmentToHullFace` | `*[2]clipVertex` | C array parameter is a pointer |
| convex_manifold.go | `( b3ShapeProxy ){ &center, 1, 0.0f }` in `b3CollideHullAndSphere` | `unsafe.Slice(&center, 1)` | aliases the local as in C, no allocation |
| convex_manifold.go | `#else` variant of `b3ReduceManifoldPoints` (`B3_MAX_MANIFOLD_POINTS` > 4, 2D hull simplification) | not ported | `MaxManifoldPoints` is fixed at 4, so upstream compiles it out too |
| convex_manifold.go | `b3CollideHullsAtWidth` edge contact: `edgeManifold.points = &edgePoint`, then copy out and restore the point buffer | `edgeManifold.Points` is the caller's buffer and the struct is copied out directly | same result (the edge builder writes the point only when it reports one); a local buffer copied into `*manifold` escapes to the heap |
| convex_manifold.go | `b3ComputeSeparatingAxisAtWidth`, `b3CollideHullsAtWidth` `simdWidth` | ignored; always the one lane kernel | width 8 not ported |
| convex_manifold.go, convex_manifold_wide.go | `_Static_assert`s | constant expressions that overflow unless the condition holds | Go has no static assert |
| convex_manifold_wide.go | `B3_WIDE( b3ComputeSeparatingAxis )` compiled per width in wide4.c / wide8.c | `computeSeparatingAxisW`, compiled once; `_Alignas` dropped; float pointers are slices | spec: scalar lanes, width suffix dropped |
| convex_manifold_wide.go | `b3GetSupportWide( …, &support, &vertexIndex )`, `b3GetFaceSeparation( …, &vertexIndex )` | returns `(support, vertexIndex)` / `(separation, vertexIndex)` | out-parameters become returns |
| convex_manifold_wide.go | `int x = a >= b;` in `b3TestEdgeCandidateSorted` | `boolInt(a >= b)` | Go bools do not convert to int |
| sat_test.go | `kRoot2`, `kAxisY`, `kAxisZ`, `ExactQuat`, `ExactRotation` (also in test_manifold.c) | defined once for the package (`exactQuat` in hull_test.go) | one package holds all test files |
| sat_test.go, manifold_test.go | `g_seed` / `NextFloat` / `NextDirection` | `satSeed`/`satNextFloat`/…, `manifoldSeed`/`manifoldNextFloat`/… | one package holds all test files |
| sat_test.go, manifold_test.go | `NextFloat()` calls inside initializer lists and argument lists | explicit temporaries in left to right order | C leaves the order unspecified; clang (the reference build) evaluates left to right |
| sat_test.go | `printf` statistics | `GinkgoWriter.Printf` | shown with `-ginkgo.v` |
| triangle_manifold.go | `b3CollideTriangleAndSphere( …, const b3Vec3* triangleA, … )`, `b3CollideTriangleAndCapsule` | `triangleA []Vec3` (at least three vertices) | pointer to an array becomes a slice |
| triangle_manifold.go | `#if B3_FORCE_GHOST_COLLISIONS` in `b3QueryTriangleAndHullEdges` | only the default (`triangle->flags`) branch | the macro is not defined upstream |
| triangle_manifold.go | `b3_triangleConvexCalls`, `b3_triangleCacheHits` | package `atomicInt` variables, unused as upstream | kept for the file mapping |
| manifold_test.go | `MinSeparation` | `manifoldMinSeparation` | avoids shadowing by the many `minSeparation` locals |
| manifold_test.go | `OverlapNeverEmptyTest` draws from `g_seed` without reseeding | reseeds to the initial 12345 first | Ginkgo may run specs alone or in another order; upstream reaches this test with the seed untouched |
| manifold_test.go | `x * B3_PI / 180.0f` | `degreesToRadians(x)`, rounded per operation as the C float expression | Go folds constant chains exactly |
| dynamic_tree.go | `b3DynamicTree_Create( capacity )`, `b3DynamicTree_Load( fileName, scale )` | free functions `CreateDynamicTree`, `LoadDynamicTree`; every other `b3DynamicTree_*` is a method on `*DynamicTree` (`tree.CreateProxy`, `tree.Query`, `tree.Rebuild`, …) | no receiver for constructors |
| dynamic_tree.go | internal `b3DynamicTree_MoveProxyInternal`, `_MarkProxyMovedSerial`, `_MarkProxyMoved`, `_ClearMoved`, `_GatherMovedProxies`, `_Refit`, `b3TreeView_*`, `b3CreateTreeProxyInternal` | unexported methods `moveProxyInternal`, `markProxyMovedSerial`, `markProxyMoved`, `clearMoved`, `gatherMovedProxies`, `refit`; `(*treeView).query/queryClosest/rayCast/boxCast`; free `createTreeProxyInternal` | spec naming rules for internal functions |
| dynamic_tree.go | node, parent, proxy and rebuild arrays (`B3_GROW_ZERO`, `b3Alloc`, `b3Free`) | slices of length == capacity grown with make + copy; the capacity fields are kept; `Destroy` zeroes the struct | allocators dropped |
| dynamic_tree.go | `B3_TREE_HEURISTIC == 1` (`b3PartitionSAH`, `b3TreeBin`, `b3TreePlane`) | not ported; `LeafBoxes` / `BinIndices` stay nil | compiled out upstream (median split is used) |
| dynamic_tree.go | `#if B3_ENABLE_VALIDATION` in `b3DynamicTree_Validate`, `_ValidateNoMoved`, `b3ValidateSubtree` | `if !validateEnabled { return }`; helper always compiled | build tags instead of macros |
| dynamic_tree.go | `b3DynamicTree_GetByteCount` `sizeof` terms | upstream's 64-bit C sizes as constants (`dynamicTreeCSize` = 112, node 32, proxy 24, …) | Go struct sizes differ (slice headers) |
| dynamic_tree.go | `b3DynamicTree_Save` / `_Load` (`fwrite` / `fread` of the struct, `b3OpenTreeFile`) | explicit 112-byte C image of `b3DynamicTree` (pointer slots zero) then native-endian node/parent/proxy arrays; `os.Create` / `os.ReadFile`; a short file fails like a short read | byte compatible with C tree files (golden hashes the saved file) |
| dynamic_tree.go | `b3DynamicTree_GatherMovedProxies( tree, int* proxyIds )` | `gatherMovedProxies(proxyIds []int32)` | pointer becomes a slice |
| dynamic_tree.go | `b3LoadV( &p.x )` on a `b3Vec3` | `loadV(vec3Floats(&p))`, an `unsafe.Slice` of the three components | `loadV` takes a slice |
| dynamic_tree_test.go | `TreeSaveLoadRoundtrip` writes `test_dynamic_tree_roundtrip.dat` in the working directory | writes into `GinkgoT().TempDir()` | test hygiene |
| dynamic_tree_test.go | `RandomBox` initializer list, `qsort` in `SameSet`, `(int)userData` | sequenced calls (left to right like clang/gcc), `slices.Sort`, `int(int32(userData))` | unspecified C evaluation order; same truncation |
| mover.go | `b3SolvePlanes( targetDelta, planes, count )`, `b3ClipVector( vector, planes, count )` | `SolvePlanes(targetDelta, planes []CollisionPlane)`, `ClipVector(vector, planes []CollisionPlane)` | pointer + count becomes a slice |
| mesh.go | `const b3MeshNode*` traversal (`node + 1`, `node + childOffset`, node stacks) | node indices into `GetMeshNodes(mesh)`; the static helpers are prefixed `mesh` (`meshIsLeaf`, `meshGetLeftChild`, `meshGetRoot`, `meshLeafNode`, …) | Go has no pointer arithmetic; dynamic_tree.go already owns `isLeaf`, `getLeftChild`, `leafNode` in the one package |
| mesh.go | `b3MeshNode::data` bit-field union (`asNode.axis`/`childOffset`, `asLeaf.type`/`triangleCount`) | `meshNodeType`, `meshNodeAxis`, `meshNodeChildOffset`, `meshNodeTriangleCount`, `setMeshNodeLow`, `setMeshNodeHigh` on `MeshNode.Data` (low 2 bits, high 30 bits) | Go has no bit fields; same bits as the C layout (checked by the golden hashes) |
| mesh.go | `b3CreateMesh( def, int* degenerateTriangleIndices, int degenerateCapacity )` | `CreateMesh(def, degenerateTriangleIndices []int32)`; capacity is the slice length, nil for none | pointer + count is a slice |
| mesh.go | `b3BuildRecursive( nodes, count, primitives, base, … )` (`primitives - base` leaf offset) | sub-slice plus an explicit `baseOffset` | Go has no pointer difference |
| mesh.go | `b3VertexMap`, `b3EdgeMap` (`vt_hash_integer`) | `verstable[uint64, int]` with `vtHashInteger` | verstable port (see verstable.go); only lookups/inserts |
| mesh.go | `#if B3_ENABLE_VALIDATION == 1` `b3IsNonDegenerate`, `b3IsConsistent`, `b3ValidateSplit`, two `b3IsValidMesh` bodies | helpers always compiled; `isValidMesh` branches on `validateEnabled`; `B3_VALIDATE` sites use `validate(!validateEnabled \|\| …)` | build tags instead of macros |
| mesh.go | `b3Alloc`/`b3Free` of meshes, `B3_ALLOC` scratch in `b3IdentifyEdges`, `b3DestroyMesh` | `blobAlloc` / `make`; `DestroyMesh` is a no-op | GC owns memory |
| mesh.go | `b3CollideMoverAndMesh( planes, capacity, shape, mover )` | `collideMoverAndMesh(planes []PlaneResult, shape, mover)`; capacity is `len(planes)` | pointer + count is a slice |
| mesh.go | `area = 0.5f * b3Length( normal )` feeding `surfaceArea += area` | product wrapped in `float32()` | Go fused it into an FMA (FMA guard) |
| mesh_test.go, mover_test.go | `MakeTwoMaterialMesh` (static in test_mover.c and test_compound.c), `FatVertex` | `moverMakeTwoMaterialMesh` / `compoundMakeTwoMaterialMesh`; `fatVertex` + `fatPositions` | one package holds all test files; strided base as a slice |
| height_field.go | `b3HeightFieldData::clockwise` (`uint8_t`) set from `bool` | `Clockwise uint8` set to 0/1 and tested with `!= 0` | Go bools do not convert to integers |
| height_field.go | `b3Alloc`/`b3Free` of the blob and the decompressed height scratch, `b3DestroyHeightField` | `blobAlloc` / `make`; `DestroyHeightField` is a no-op | GC owns memory |
| height_field.go | `b3GetHeightFieldCellCorners( hf, row, column, b3Vec3 corners[4] )` | `corners *[4]Vec3` | C array parameter is a pointer |
| height_field.go | `( b3ShapeProxy ){ &input->origin, 1, 0.0f }` in `b3RayCastHeightField` | `unsafe.Slice(&input.Origin, 1)` | aliases the input as in C, no allocation |
| height_field.go | `b3CollideMoverAndHeightField( planes, capacity, shape, mover )` | `collideMoverAndHeightField(planes []PlaneResult, shape, mover)`; capacity is `len(planes)` | pointer + count is a slice |
| height_field.go | `b3DumpHeightData` (`fopen`/`fprintf`), `b3LoadHeightField` (`fscanf`) | `os.Create` + `fmt.Fprintf` with the same formats (`%.9f` of the float promoted to double); `fmt.Fscan` (whitespace separated, float32 parse correctly rounded like `strtof`) | Go I/O; the golden test checks a dump/load round trip gives the C blob hash |
| height_field.go | `_Static_assert( b3_concaveEdge3 == 4 * b3_concaveEdge1 )` and the inverse variant | overflowing constant pairs | Go has no static assert |
| height_field.go | `gridFractionOffset = input->maxFraction * minFraction` (later added to a product) | product wrapped in `float32()` | Go fused it into an FMA through the inlined `mulAdd` (FMA guard) |
| mesh.go | `xWidth`/`zWidth`/`hb`/`ht`/`hy` half-width products in the creators | wrapped in `float32()` | rule: products feeding adds are rounded explicitly |
| height_field_test.go | `MakeFlatField` (static in test_height_field.c and test_mover.c), `BruteForceShapeCast`, `BruteForceRayCast`, `printf` mismatch lines | `makeFlatField` defined once (mover_test.go uses it), `heightFieldBruteForce*`, `GinkgoWriter.Printf` | one package holds all test files |
| height_field_test.go | `FileRoundtrip` writes `test_height_field_roundtrip.dat` in the working directory | writes into `GinkgoT().TempDir()` | test hygiene |
| compound.go | compound.h + compound.c | one Go file (header/source pair) | file mapping rule |
| compound.go | `b3CompoundTimeOfImpact` (declared in compound.h, defined only under `#if 0`), the `#if 0` TOI block | not ported | compiled out upstream |
| compound.go | `b3HashMesh`, `b3HashMaterial` (`vt_wyhash`) | `rapidhash` over the same bytes | the hash only places buckets in the Go verstable (bucket order differs anyway) |
| compound.go | `memcmp` of materials and meshes | `bytes.Equal` over `surfaceMaterialBytes` / `meshBytes` | same comparison |
| compound.go | `b3AllocZero`/`b3Free` scratch arrays, `b3DestroyCompound` | `make`; `DestroyCompound` is a no-op | GC owns memory |
| compound.go | `b3ChildShape.materialIndices = &capsules[i].materialIndex` etc. | `unsafe.Slice(&….MaterialIndex, 1)` into the blob; mesh children get a `MaterialCount` slice | pointer becomes a slice aliasing the image |
| compound.go | `b3ValidateCompound( const uint8_t* bytes, int byteCount )` | `ValidateCompound(bytes []byte, byteCount int) *CompoundData`; the result aliases `bytes` (8-byte aligned, as `make([]byte, n)` is for n >= 8) | slice plus the explicit count (tests pass a count that differs from the buffer) |
| compound.go | `b3CollideMoverAndCompound( planes, capacity, … )`, `b3CompoundMoverContext.planes + planeCount` | `collideMoverAndCompound(planes []PlaneResult, …)`; capacity is `len(planes)`; children get `planes[planeCount:capacity]` (convex ones `&planes[0]`) | pointer + count is a slice |
| compound.go | `void* context` callbacks (`b3CompoundOverlapContext`, …) | `any` holding a pointer to the context struct, type-asserted in the callback | translation rules |
| compound.go | `b3GetCompoundMesh` asserts `meshOffset >= … * sizeof( b3HullInstance )` | kept (`hullInstance` size) | upstream quirk ported as is |
| compound.go | size checks for `b3MeshInstance` (52) and `b3TreeNode` (32) | extra overflowing constant pairs | the blob copies those arrays verbatim; no upstream counterpart |
| compound_test.go | `MakeMaterial`, `MakeTwoMaterialMesh`, `QueryCallback`, `malloc`/`memcpy`/`memcmp` | `compoundMakeMaterial`, `compoundMakeTwoMaterialMesh`, `compoundQueryCallback`, `make`/`copy`/`bytes.Equal` | one package holds all test files |
| geometry_golden_test.go | testdata/geometry_golden_gen.c defines `b3MakeLocalProxy` / `b3ComputeProxyAABB` (verbatim from shape.c) | the generator links them itself | shape.c needs the whole world |
| collision_test.go | `#if defined( BOX3D_DOUBLE_PRECISION )` halves of `LargeWorldManifoldTest`, `LargeWorldAABBTest` | not ported | double precision not ported; upstream's float build compiles them out too |
| qsort.go | `QSORT( N, LESS, SWAP )` and the `Q_*` helper macros | `qsort(n, less, swap)` with closures; `Q_SUBFILES` is a local closure; indices are `int` instead of `unsigned long` (never wraps); stack size fixed at 32 (the size for a 32-bit `Q_N`, which every call site passes) | Go has no macros |
| joint.go | joint.h ported early (milestone 3 prep) | struct types and enums; the joint.c functions land in milestone 4 | `b3World` and the solver reach the joint types; kept field for field so `b3World` is faithful |
| body.go, contact.go | `b3BodyFlags`, `b3ContactFlags` enums | untyped constants | they are or-ed into `uint32` fields |
| body.go, contact.go, joint.go | `b3Body.type`, `b3Joint.type`, `b3JointSim.type` | field `typ` | `type` is a Go keyword |
| contact.go | `b3ContactCache` union, `b3Contact` anonymous union `{ convexContact; meshContact }` | all members present | Go has no unions |
| contact.go | `b3Contact.manifolds` (`b3Manifold*`) | `[]Manifold` plus the kept `manifoldCount` | pointer + count is a slice |
| joint.go | `b3JointSim` anonymous union of the eight joint structs | all members present; `typ` says which is valid | Go has no unions; revisit the size if it matters (M4) |
| constraint_graph.go | `b3GraphColor.wideConstraints`, `wideMeshConstraints` (`void*`), `manifoldConstraints`, `contactConstraints`, `wideMeshManifoldStarts` (pointers) | `wideConstraintBuffer` for the `void*` buffers (see contact_solver_wide.go); the typed pointers are slices | pointer arrays become slices |
| contact_solver.go | `b3ContactConstraint.constraints` (`b3ManifoldConstraint*`) | `[]manifoldConstraint` | pointer + count is a slice |
| name_cache.go | b3NameEntry.name (char*), b3NameCache.map (void*) | string; nameMap *nameMap (verstable[uint32, int]) | map is a Go keyword |
| physics_world.go | `b3World.hullDatabase` (`void*` holding a `b3HullMap*`) | `*hullMap` | upstream type-erases only to keep verstable out of the header |
| physics_world.go | `b3World.manifoldAllocators` (`b3Array( b3BlockAllocator )`) | `[]blockAllocator[Manifold]` | typed block allocator (see block_allocator.go) |
| physics_world.go | `b3World` callbacks and `void*` contexts | the public callback types; contexts are `any`; `userTreeTask` is `any` | translation rules |
| broad_phase.go | `B3_PROXY_TYPE`, `B3_PROXY_ID`, `B3_PROXY_KEY` macros; `b3BroadPhase_MarkProxyMoved*` inlines | functions `proxyType`, `proxyId`, `proxyKey`; methods on `*broadPhase` | Go has no macros; spec naming rule `b3Type_Method` |
| parallel_for.go | `b3ParallelForShared shared`, `b3ParallelForTask tasks[]` on the C stack | Go locals that escape to the heap (one small allocation per call) | the task system receives pointers to them; revisit for no-allocation `Step` |
| broad_phase.go | `b3BroadPhase_CreateProxy`, `_DestroyProxy`, `_MoveProxy`, `_TestOverlap`, `_GetShapeIndex` | methods on `*broadPhase`; `b3CreateBroadPhase`, `b3DestroyBroadPhase`, `b3UpdateBroadPhasePairs`, `b3ValidateBroadPhase`, `b3ValidateNoMoved` stay free functions | spec naming rule `b3Type_Method` |
| broad_phase.go | `b3BroadPhase.movedSiblings` (`int*` from `b3StackAlloc`), `b3CrossContext.seeds`, `b3PairContext.pairKeys` | `[]int32`, `[]nodePair`, `*[]uint64` | pointers become slices |
| broad_phase.go | `node->shapeIndex` | `node.Height` | `TreeNode` keeps the `height`/`shapeIndex` union as one field (see types.go) |
| broad_phase.go | `b3PrefetchHash`, `b3Prefetch( shapes + … )` loop in `b3FlushCandidatePairs` | `prefetchHash` is an empty function; the shape prefetch loop is dropped | `b3Prefetch` is dropped (platform.go) |
| broad_phase.go | `QSORT( pairCount, LESS, SWAP )` with `#define LESS/SWAP` | `qsort` with closures | see qsort.go |
| broad_phase.go | `_Static_assert`s | constant index expressions that fail to compile unless the condition holds | Go has no static assert |
| broad_phase.go | `b3TracyCZone*` | dropped | Tracy is a non-goal |
| broad_phase.go | `void* context` of the parallel-for and task callbacks | `any`, type-asserted (`*crossContext`, `*world`) | translation rules |
| broad_phase_golden_test.go | testdata/broad_phase_golden_gen.c stubs `b3CreateContact` (records the key, adds it to the pair set) and `b3ValidateSolverSets` | the Go test runs the real createContact and reads the created pairs back from the contacts; under `box3d_validate` the golden is skipped | the hand assembled world has no solver set bodies, islands or id pools; Go cannot replace a function for one test |
| broad_phase_test.go | `BroadPhaseStressTest` | pending `PIt` | needs weld joints (joint.c, weld_joint.c, M4); port it with CheckPairSet, RandomState and the stress context then |
| broad_phase_test.go | `b3SetAssertFcn( FlushingAssert )` in `BroadPhaseTest` | not ported | it only flushes stdout before a crash in the C runner |
| solver.go | `b3StepContext` pointers (`states`, `sims`, `enlargedShapes`, `bulletBodies`, `awakeContactIndices`, the span arrays, `stages`), `b3SolverStage.blocks`, span `contacts`/`order`/`joints` | slices, count fields kept | pointer + count is a slice |
| solver.go | `b3SolverBlockType` | untyped constants (`bodyBlock` … `overflowBlockType`; `b3_overflowBlock` would clash with the `overflowBlock` arena type) | stored in the `uint8_t` `blockType` |
| solver.go | `b3SolverStage.type` | field `typ` | `type` is a Go keyword |
| contact_solver_wide.go, solver.go, constraint_graph.go | `void* wideConstraints`, `void* wideMeshConstraints` (heterogeneous byte stream of `b3ContactConstraintWide` each followed by its `b3ContactManifoldWide`s, walked by `b3GetConstraintW` / `b3NextConstraintW` / `b3GetManifoldsW` with byte offsets) | `wideConstraintBuffer{constraints []contactConstraintWide; manifolds []contactManifoldWide}`: constraint i is `constraints[i]`, its manifolds start at `manifolds[i*fixedManifoldCount]` (convex) or `manifolds[manifoldStarts[i]-manifoldStarts[0]]` (mesh); a per-color base pointer is the pair of sub-slices; NULL is the zero value | typed buffers without unsafe; the kernels index the two slices exactly where upstream computes byte offsets |
| contact_solver_wide.go | `_Alignas( B3_WIDE_ALIGNMENT )` on `b3ContactManifoldWide`, `b3ContactConstraintWide`, and the layout `_Static_assert`s | dropped | scalar lanes need no 16/32 byte alignment; the stream is two typed slices, so no byte layout to assert |
| recording.go | `b3Recording` | exported opaque `Recording` (all fields unexported) | the public API hands it out (`b3CreateRecording`, `b3World_StartRecording`) |
| recording.go | `b3RecBuffer.data`, `b3GeometryEntry.bytes`, `b3Recording.tags` (pointers), `void* dedupMap`, `void* tagMap`, `b3RecQueryWriter.userFcn` union, `const char* tagName` | slices with counts kept; `any`; all union members present; `string` | translation rules; maps typed with recording.c (M5) |
| recording.go | `_Static_assert( sizeof( b3RecHeader ) == 48 )` | two constant index expressions on `unsafe.Sizeof` | Go has no static assert |
| recording.go | `b3FnvMixPosition` `BOX3D_DOUBLE_PRECISION` branch | float branch only | double precision not ported |
| recording_ops.go | X-macro expansions of `recording_ops.inl` in recording.h (pass 1a arg structs `b3RecArgs_Name`, `b3_recOpName` enum) | written out as Go structs `recArgsName` and untyped constants `recOpName`; ARG tags map to Go types (table in the file header); field `type` is `typ` | Go has no X-macros; keep in sync with the manifest on upstream bumps |
| recording.go | `B3_REC( world, Name, ... )`, `B3_REC_CREATE( world, Name, id, ... )` | expanded at the call site: `if world.recording != nil { recArgs := recArgsName{...}; recWriteName(world.recording, &recArgs) }` (and `recWriteRetName` with the id) | Go has no macros |
| recording.go | every function declared in recording.h: `b3RecBufAppend`, `b3RecBufFree`, `b3RecW_*` (43 write primitives), `b3RecBeginRecord`, `b3RecEndRecord`, `b3RecWrite_Name` (168 ops), `b3RecWriteRet_Name` (16 create ops), `b3RecReserveU32`, `b3RecPatchU32`, `b3RecCommitRecord`, `b3RecQueryBegin`, `b3RecQueryCommit`, `b3RecOverlapTrampoline`, `b3RecCastTrampoline`, `b3RecPlaneTrampoline`, `b3InternGeometry`, `b3AppendGeometry`, `b3FreeRegistry`, `b3RecWriteRegistry`, `b3HashQueryTag`, `b3RecInternTag`, `b3RecInternHull`, `b3RecInternMesh`, `b3RecInternHeightField`, `b3RecInternCompound`, `b3StartRecordingIntoBuffer`, `b3StopRecordingInternal`, `b3RecAccumulateBounds` | stubs (`// PORT: stub until milestone 5 (recording.c)`): no-ops returning zero values; `b3RecWriteArgs_Name` is not stubbed (only recording.c calls it) | body.c, shape.c, joint.c and physics_world.c call them in M3/M4; `startRecordingIntoBuffer` leaves `world.recording` nil, so the hooks never fire. `b3HashWorldState` is ported (next row) |
| recording.go | `b3HashWorldState` (recording.c) | ported in milestone 3, ahead of the rest of recording.c; `B3_HASH_FLOAT` written out with `math.Float32bits` | test_world.c, test_restitution.c and world_golden_test.go compare it |
| joint.go | `b3GetJointSim`, `b3DestroyJointInternal`, `b3GetJointReaction`, `b3PrepareJoint`, `b3WarmStartJoint`, `b3SolveJoint`, `b3DrawJoint` (joint.c) | placeholders that panic (`// PORT: placeholder until milestone 4 (joint.c)`); `getJointReaction` returns `(force, torque)` | M3 files call them; no joint can exist before joint.c (M4) |
| joint.go | `b3PrepareJoints_Overflow`, `b3WarmStartJoints_Overflow`, `b3SolveJoints_Overflow` (joint.c) | placeholders whose body is upstream's loop over the overflow joints (a no-op while there are none) | solver.c calls them every step in M3 |
| joint.go | `b3DefaultExplosionDef` (joint.c) | ported in milestone 3, ahead of the rest of joint.c | `b3World_Explode` tests need it |
| name_cache.go | `const char*` returns of `b3FindName`, `b3FindNameWithDefault` | `string`, NULL is "" | translation rules |
| physics_world.go | `b3AllocateManifolds` / `b3FreeManifolds` (`b3Manifold*`, `memset`) | `[]Manifold` of count elements from allocator count - 1, zeroed with `clear` | typed block allocator |
| body.go | `b3EncodeBodySimIndex`, `b3IsStaticSimIndex`, `b3DecodeAwakeIndex`, `b3SleepBodySimIndex` | take and return `int` | callers convert to the `int32` contact fields |
| contact.go | (b3Contact){ 0 } in b3CreateContact | package var zeroContact (contact_solver_wide.go's b3_zeroContact, never written) assigned to the slot | the local contact shadows the type name; same effect |
| contact.go | s_registers, s_initialized | package vars contactRegisters, contactRegistersInitialized | process-global as upstream; contact_test.go saves/restores them |
| contact.go | b3ContactData.manifolds pointer | slice contact.manifolds[:manifoldCount] (nil when 0) | pointer + count is a slice |
| contact.go | memcpy of b3Shape into childShapeA in b3UpdateContact | struct copy childShapeA := *shapeA | same; materials set to nil so getShapeMaterials uses the inline material |
| scheduler.go | b3Thread, b3Semaphore, b3Yield in the scheduler | worker threads are goroutines (timer.go createThread), the semaphore is timer.go's sync.Cond semaphore; a nil interface and a nil *schedulerTask both count as NULL in schedulerFinishTask | spec: goroutine worker pool |
| constraint_graph.go | `B3_FORCE_OVERFLOW`, `_Static_assert` | constant `forceOverflow` tested with `if`; static asserts are uint conversions of constant expressions | no preprocessor |
| constraint_graph.go | `memset( jointSim, 0, ... )` in b3CreateJointInGraph | `arrayMemZero` on the one-element tail slice | local variable name shadows the type |
| island.go | detached base arrays and `b3Free` in b3SplitIsland | cached slices, base island slices set to nil, the GC frees them | allocators dropped |
| island.go | Tracy zone in b3SplitIslandTask | dropped | spec non-goal |
| solver_set.go | locals `awakeSet` / `disabledSet` shadowing the set enum | named `awakeSet_` / `disabledSet_` | Go would shadow the constants used in the same function |
| body.go | `b3Body_SetType` early return for compound/height shapes | ported as is: returns with `world->locked` still true | upstream bug |
| body.go | `b3Body_GetContactData`, `b3Body_GetShapes`, `b3Body_GetJoints`, `b3Body_CollideMover` | pointer + capacity become one slice; capacity is len | translation rules |
| body.go | `b3Body_GetClosestPoint( bodyId, b3Vec3* result, target )` | `GetClosestPoint(target) (distance float32, result Vec3)` | out-parameter becomes a return value |
| body.go | `b3Body_GetClosestPoint` proxy `{ &target, 1, 0 }`; `b3Body_CollideMover` single `b3PlaneResult` | one element slice / `[1]PlaneResult` | Go slices instead of pointers |
| body.go | `*bodySim = (b3BodySim){ 0 }`, `*bodyState = (b3BodyState){ 0 }` in b3CreateBody | omitted (arrayEmplace zeroes) | the local names shadow the types |
| body.go | `b3Body_EnableSleep` parameter `enableSleep`; locals `staticSet`/`awakeSet`/`disabledSet` in SetType/Disable/Enable | renamed `flag`, `staticSolverSet`, `awakeSolverSet`, `disabledSolverSet` | they would shadow the constants |
| solver.go | `b3Pause` (pause / yield instructions) | empty function (upstream's fallback branch) | Go has no pause intrinsic; asynchronous preemption keeps spin loops fair |
| solver.go | `b3TracyCZone*` | dropped | Tracy is a non-goal |
| solver.go | `b3InitStage( stage, … )` returning `stage + 1`, `b3InitColorStages`, `b3InitBlocks( b3SyncBlock* … )` | stage pointer is an index into `stages` (returns the next index); block pointers are sub-slices | Go has no pointer arithmetic |
| solver.go | `wideConstraints` / `wideMeshConstraints` byte buffers sized with `b3GetWide*ByteCount`; per-colour byte offsets | two `stackAlloc`s each (constraints, manifolds) forming a `wideConstraintBuffer`; per-colour buffers are sub-slices starting at constraint index `wideBase` / `meshGroupBase` and manifold index `wideBase` / `meshManifoldStarts[meshGroupBase]`; the byte count functions are not called | typed buffers |
| solver.go | `enableSleep` local in `b3FinalizeBodiesTask`, `awakeSet` local in `b3Solve` | renamed `worldEnableSleep`, `awakeSolverSet` | would shadow constants |
| solver.go | `snprintf( buffer, "solve[%d]", i )` | precomputed `solverTaskNames` | no allocation when enqueueing |
| solver.go | C stack locals `widePrepareSpans`, `meshPrepareSpans`, `jointPrepareSpans`, `overflowSpans`, `workerContext[B3_MAX_WORKERS]`, `b3ContinuousContext` | Go locals that escape to the heap | revisit for no-allocation `Step` (M6), together with `stackAlloc` reusing its memory |
| solver_golden_test.go | testdata/solver_golden_gen.c `#include "solver.c"` and links every other src/*.c except wide8.c | the generator calls the static kernels directly | Go-only tooling |
| sensor.go | qsort( overlaps2, …, b3CompareVisitors ) (libc) | ported QSORT (qsort.go) | both unstable; entries with equal shapeId are identical visitors, so the de-duplicated result is the same |
| mesh_contact.go | static bool s_once in b3RefreshCache | package-level atomic.Bool | avoids a data race between narrow-phase workers; same log-once behaviour |
| name_cache.go | b3Free of interned and dropped names, byte-count bookkeeping | dropped (GC) | allocator not ported; GetByteCount stays 0, so test_name_cache.c's byte-count checks pass trivially |
| name_cache_test.go | NameRoundTrip, RollbackNames | pending PIt | need recording.c, replay.c, world_snapshot.c (M5) |
| contact_solver_wide.go | b3LoadW on (const float*)&b3BodyState / &invInertiaWorld.cx.x / &manifold->normal.x / manifold->points + i, and the layout _Static_asserts | setW of the fields at those float offsets (bodyStateLoadW for the 16-byte body state rows; storeBodyVelocity writes padding1/padding2 as b3StoreW does) | no float-pointer views of Go structs |
| contact_solver_wide.go | B3_GATHER_LANES macro | written out as setW( lanes[0].field, … ) at each site | no macros |
| contact_solver_wide.go | b3GetConstraintW / b3NextConstraintW / b3GetManifoldsW return pointers into the byte stream | return/advance (constraint index, manifold index) positions in wideConstraintBuffer; getManifoldsW(base, m) | typed buffers |
| contact_solver.go, contact_solver_wide.go | b3GetWide*ByteCount(W4) return C sizeof | return unsafe.Sizeof of the Go types (informational; the buffers are typed slices) | |
| contact_solver.go, contact_solver_wide.go | B3_SIMD_HAS_WIDTH_8 dispatch and the W8 kernels | dropped; dispatchers always call the single W kernel | one lane width |
| contact_solver.go | b3ContactConstraint.constraints = manifoldBase + manifoldStart | slice manifoldConstraints[manifoldStart : manifoldStart+manifoldCount] | |
| shape.go | b3CreateShape / b3CreateShapeInternal `const void* geometry` | `geometry any` holding the typed pointer, asserted per shape type | Go has no void* |
| shape.go | B3_REC / B3_REC_CREATE in shape.c | expanded at the call site | Go has no macros |
| shape.go | shape materials b3Alloc + memcpy / b3Free | make + copy / nil | allocators dropped |
| shape.go | b3CollideMover( planes, planeCapacity, … ), b3Shape_GetContactData( id, data, capacity ), b3Shape_GetSensorData( id, ids, capacity ) | slices; capacity is len | pointer + count is a slice |
| shape.go | `( b3ShapeProxy ){ &target, 1, 0 }` in b3Shape_GetClosestPoint, `&toiContext->localCentroidB` in b3MeshTimeOfImpactFcn | unsafe.Slice aliasing the variable | no allocation, aliases as in C |
| shape.go | `b3Vec3 triangle[3]` local in b3MeshTimeOfImpactFcn | `triangle [3]Vec3` field in meshImpactContext | avoids a heap allocation per triangle |
| shape.go | b3Shape_ApplyWind hull loop half-edge pointers | indices into the edges slice | Go slices |
| shape.go | `#if B3_DEBUG_WIND`, `#if 0` convex TOI fallback in b3ShapeTimeOfImpact | `const debugWind = false` block ported; `#if 0` block not ported | compile-time switches |
| shape.go | b3Shape_SetFilter `destroyProxy = filter.categoryBits == shape->filter.categoryBits` after `shape->filter = filter` | always true; ported as is | upstream bug |
| shape.go | b3Shape_ApplyWind reads `sim` before b3WakeBodyWithLock | force/torque land on the old sleeping-set sim (dangling in C, a detached copy in Go); ported as is | upstream bug |
| shape_golden_test.go | testdata/shape_golden_gen.c builds b3Shape values by hand and links all of src/*.c | world-free shape.c functions only | shape.c otherwise needs the world |
| physics_world.go, physics_world_queries.go | physics_world.c | split in two Go files: the world queries (WorldQueryContext … b3World_CastMover) in physics_world_queries.go, everything else in physics_world.go, both in upstream order with the upstream header | 4.2k lines (spec: left to the builder) |
| physics_world.go | physics_world.h declarations of physics_world.c functions (b3GetWorldFromId, b3ValidateSolverSets, b3AddHullToDatabase, …) | defined once, in the physics_world.c section | file mapping |
| physics_world.go | B3_MAX_WORLDS and graph color count `_Static_assert`s | overflowing constant pairs | Go has no static assert |
| physics_world.go | `memset( world, 0, sizeof( b3World ) )` in b3CreateWorld / b3DestroyWorld | `clear(unsafe.Slice(world, 1))` | same effect on the Go struct |
| physics_world.go | `b3Alloc( sizeof( b3HullMap ) )` + `b3HullMap_init` | `new(hullMap)` + `hullMapInit` | allocators dropped |
| physics_world.go | `b3Prefetch` of contacts in the contact update | `prefetchContact` is empty | b3Prefetch is dropped (platform.go) |
| physics_world.go | `staticSims - encodedIndex - 2` pointer arithmetic when resolving a contact's body sim | index into the static set slice (`resolveContactBodySim`) | Go has no pointer arithmetic |
| physics_world.go | `b3StepContext context = { 0 }` on the C stack in b3World_Step | `new(stepContext)` (escapes) | the parallel-for tasks receive a pointer; revisit for no-allocation `Step` (M6) |
| physics_world.go | `snprintf` into 32-byte buffers in b3World_Draw; `%#X` of a feature id | `fmt.Sprintf` (no truncation); Go's `%#X` prints `0X0` for zero where C prints `0` | debug draw text only |
| physics_world.go | b3World_Draw assigns `contactKey` twice; the island branch's `continue` skips clearing `debugIslandSet` | ported as is | upstream quirks |
| physics_world.go | b3GetWorldByteCount `sizeof` of internal structs and array capacities | Go sizes (`unsafe.Sizeof`, `arrayByteCount`; manifold blocks count manifolds) | diagnostic only; totals differ from C |
| physics_world.go | `#if B3_ENABLE_VALIDATION` validation bodies / empty functions | always compiled; each returns early unless `validateEnabled` (box3d_validate) | build tags instead of macros |
| physics_world.go, physics_world_queries.go | `( b3ShapeProxy ){ &localPosition, 1, 0.0f }`, `{ &mover->center1, 2, mover->radius }` | `unsafe.Slice` of the local / `capsulePoints(mover)` | aliases as in C, no allocation |
| scenes_benchmarks_test.go | shared/benchmarks.c, shared/overflow_color.c | only CreateJunkyard / GetJunkyardCapacity / StepJunkyard / the ground shape id and CreateOverflowColorPile, in a _test.go file of package box3d; `cosf` / `sinf` are float64 `math` rounded to float32 | test_world.c needs them; internal tests cannot import a package that imports box3d; the rest of benchmarks.c lands in M6 |
| scenes_human_test.go, scenes_determinism_test.go | shared/human.c, shared/determinism.c | in _test.go files of package box3d behind the `box3d_m4_joints` build tag; the #if 0 pose control block of human.c is not ported | the ragdolls need joint.c (M4); remove the tag when it lands |
| world_test.go | BodySimLocatorTest (LocatorTransferScene creates a spherical joint) | pending `PIt`; the joint lines are commented out behind a panic (`worldNeedsJoints`) and the Go-only BodySimLocatorTestWithoutJoints runs the other four scenes | joint.c (M4); restore the lines and delete the stand-in then |
| world_golden_test.go | testdata/world_golden_gen.c links every src/*.c except wide8.c and steps a joint-free scene | Go-only tooling | world-level parity check before the M5 determinism goldens |
| mover_test.go | `MakeMaterial`, `PlaneCapture`, `CapturePlaneFcn` (static in test_mover.c) | `moverMakeMaterial`, `moverPlaneCapture`, `moverCapturePlaneFcn` | one package holds all test files |
| compound_test.go | `MakeFlatMaterial5Mesh`, `s_compoundContactMaterialId`, `CompoundContactFrictionCallback` | `makeFlatMaterial5Mesh`, `compoundContactMaterialId`, `compoundContactFrictionCallback` | one package holds all test files |
| broad_phase_test.go | `RunStaticRebuildKeepsMarks` | `broadPhaseRunStaticRebuildKeepsMarks` | one package holds all test files |
| body_query_test.go | `MoverTOIMatchesSweep` calls such as `b3MakeQuatFromAxisAngle( RandomDirection( &seed ), RandomFloat( &seed, … ) )` | the two seed-advancing arguments are evaluated left to right | C leaves argument evaluation order unspecified; Go is left to right like clang, so the random scenarios match a clang build (gcc may differ) |
| restitution_test.go | `RUN_MEASUREMENT`, `RestitutionTest` failureCount | one `It` per subtest; each prints the passed/failed line and ensures the subtest returned 0 | Ginkgo runs each spec on its own instead of counting failures |
| large_world_test.go | `#if defined( BOX3D_DOUBLE_PRECISION )` blocks in LargeWorldStackTest, LargeWorldBulletTest, LargeWorldQueryTest | not ported; only the origin runs remain | double precision is a non-goal; upstream's float build compiles them out |
| hash_test.go | `ProbeAdd( probe, hash, bytes, byteCount )`, `B3_ALLOC`/`B3_FREE`, `(const uint8_t*)` casts | bytes passed as a slice (nil = NULL, length = byteCount); make + GC replace alloc/free, so ProbeEnd only gathers the result; blobs via hullBytes/meshBytes/heightFieldBytes, float arrays viewed with unsafe.Slice | pointer + count becomes a slice; allocators dropped |
