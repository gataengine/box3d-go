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
| types.go | `b3MeshDef.stride` | kept as a byte stride on a `[]Vec3` | revisit with mesh.c |
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
| collision.go | geometry blob layout (`b3HullData` …), `(intptr_t)hull + offset` accessors | blob = one `[]uint64` allocation (8-byte aligned, pointer free) viewed through the header pointer; `blobSlice[T]` uses `unsafe.Add` from the header; accessors return slices with the element count (`3 * stride` for SoA arrays, `edgeCount / 2` for cosines) | spec: byte image canonical; decision recorded in PROGRESS.md |
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
| shape.go | `b3GetMeshTriangle` (static inline in shape.h) | not yet ported | needs the mesh blob accessors that land with mesh.c |
