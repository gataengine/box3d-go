# Parity report

Source of truth: upstream [erincatto/box3d](https://github.com/erincatto/box3d) at commit
`efca0c184d40fd6a82016983f1601c4d23965867` (`b3GetVersion` 0.2.0), the `third_party/box3d`
submodule of the gata monorepo.

## API surface audit

```bash
libs/box3d-go/tools/parity_audit.sh [-v] [path/to/box3d]   # from the repo root
./tools/parity_audit.sh [-v] [path/to/box3d]               # from libs/box3d-go
```

The upstream path defaults to the repo's `third_party/box3d` (initialise it with
`git submodule update --init third_party/box3d`). The script reads `include/box3d/*.h`, maps each
public symbol to its Go name by the naming rules below and checks the package's non-test `.go`
files declare it. `-v` prints the whole mapping. Exit status 2 means something is missing.
Runs on bash 3.2 (macOS `/bin/bash`) and Linux bash 5; needs perl.

Current result (darwin/arm64, 2026-10-17):

```
b3_api: total=596 mapped=593 omitted=3 missing=0
b3_inline: total=103 mapped=103 omitted=0 missing=0
types: total=146 mapped=144 omitted=2 missing=0
enums: total=194 mapped=194 omitted=0 missing=0
constants: total=46 mapped=45 omitted=1 missing=0
---
go_exported_funcs_and_methods=736
mapped=1079 omitted=6 missing=0
```

| symbol class | upstream | mapped | intentionally omitted | missing |
|---|---:|---:|---:|---:|
| `B3_API` functions | 596 | 593 | 3 | 0 |
| `B3_INLINE` functions of the public headers | 103 | 103 | 0 | 0 |
| public types (`typedef struct/enum/union`, plain and function typedefs) | 146 | 144 | 2 | 0 |
| enum values (`b3_staticBody`, …) | 194 | 194 | 0 | 0 |
| `B3_` constants with a value | 46 | 45 | 1 | 0 |
| **total** | **1085** | **1079** | **6** | **0** |

The audit checks names, not behaviour; behaviour is covered by the ported upstream tests and the
goldens (see Determinism below and `PROGRESS.md`).

### Missing

None: every public symbol is either ported or listed under "Intentionally not ported" below
(`b3GetGraphColor` is exported as `GetGraphColor`).

### Intentionally not ported

| symbol | reason |
|---|---|
| `b3SetAllocator`, `b3AllocFcn`, `b3FreeFcn` | allocator hooks dropped: Go memory is owned by the GC; `GetByteCount` always returns 0 |
| `b3InternalAssert` | assert plumbing behind the `B3_ASSERT` macro; Go's `assert` (base_assert.go) calls the handler set by `SetAssertFcn` directly |
| `b3World_DumpShapeBounds` | declared in box3d.h but never defined upstream at this commit |
| `B3_ENABLE_VALIDATION` | compile switch; the `box3d_validate` build tag sets `validateEnabled` |

Present in Go but with reduced semantics (they map, so the audit counts them):

| symbol | Go behaviour | reason |
|---|---|---|
| `b3IsDoublePrecision` | always `false` | double precision (`BOX3D_DOUBLE_PRECISION`) not ported; `Pos` / `WorldTransform` stay distinct types so it can be added |
| `b3SetSIMDWidth`, `b3GetSIMDWidth`, `b3IsAVX2Available` | width 8 only in the `box3d_simd` build on amd64 with AVX2 (the AVX2 backend, `wide8.c`); elsewhere the width is 4, `SetSIMDWidth(8)` stores 4 as on a CPU without AVX2 and `IsAVX2Available` is `false` | the default build has the scalar 4-lane backend only; `IsAVX2Available` reports whether the width 8 backend is in use, not the CPU |
| `b3GetByteCount` | always 0 | allocators dropped |
| `b3RoundDownFloat`, `b3RoundUpFloat` | take `float32` instead of `double` | double precision not ported |

Not part of the audit because upstream has no public symbol for them: Tracy profiler zones
(`b3TracyCZone…`), the platform/CPU macros, `B3_RESTRICT` / `B3_FORCE_INLINE` / `B3_LITERAL`,
the C++ operator overloads of math_functions.h, and the samples app (`samples/`, C++).

## Naming rules

| C | Go | example |
|---|---|---|
| `b3Foo` | `Foo` | `b3MakeBoxHull` → `MakeBoxHull` |
| `b3World_/Body_/Shape_/Joint_/Contact_Method(id, …)` | method on the id type | `b3Body_SetTransform(id, …)` → `BodyID.SetTransform` |
| `b3<Kind>Joint_Method(id, …)` | `JointID.<Kind>Joint<Method>` | `b3RevoluteJoint_SetSpringHertz` → `JointID.RevoluteJointSetSpringHertz` |
| `b3Create<Kind>Joint`, `b3CreateBody` | method on `WorldID` | `b3CreateBody(worldId, &def)` → `worldId.CreateBody(&def)` |
| `b3Create<Kind>Shape` | method on `BodyID` | `b3CreateHullShape` → `BodyID.CreateHullShape` |
| `b3DestroyWorld/Body/Shape/Joint` | `<Type>ID.Destroy` | `b3DestroyBody(id)` → `id.Destroy()` |
| `b3DestroyPlayer`, `b3DestroyRecording` | `(*RecPlayer).Destroy`, `(*Recording).Destroy` | |
| `b3RecPlayer_*`, `b3Recording_*`, `b3DynamicTree_*`, `b3AABB_*` | methods on `*RecPlayer`, `*Recording`, `*DynamicTree`, `AABB` | `b3RecPlayer_StepFrame(p)` → `p.StepFrame()` |
| `b3DynamicTree_Create`, `b3DynamicTree_Load` | constructors `CreateDynamicTree`, `LoadDynamicTree` | |
| geometry `b3CreateHull/Mesh/HeightField/Compound`, `b3DestroyHull/…` | free functions | `b3CreateMesh(&def)` → `CreateMesh(&def)` |
| `b3Thing` type | `Thing`, with Go initialisms | `b3BodyId` → `BodyID`, `b3TOIOutput` → `TOIOutput` |
| enum value `b3_fooBar` | typed constant `FooBar` | `b3_dynamicBody` → `DynamicBody`, `b3_toiStateHit` → `TOIStateHit` |
| `B3_FOO_BAR` | `FooBar` constant | `B3_MAX_MANIFOLD_POINTS` → `MaxManifoldPoints` |
| `B3_*` scaled by `b3GetLengthUnitsPerMeter()` | function | `B3_LINEAR_SLOP` → `LinearSlop()`, `B3_HUGE` → `Huge()` |

## API shape deviations

These follow the spec's translation rules; every instance is in `LOOSE_ENDS.md`.

- **Pointer + count / capacity → slice.** `b3Body_GetShapes(id, shapeArray, capacity)` →
  `id.GetShapes(shapes []ShapeID) int` (capacity is `len`); likewise `GetJoints`,
  `GetContactData`, `ShapeDistance(…, simplexes []Simplex)`, `MakeAABB(points []Vec3, radius)`,
  `Hash(hash, data []byte)`, `CreatePlayer(data []byte, workers)`, `ValidateReplay(data, …)`,
  geometry accessors (`GetHullPoints(hull) []Vec3`, …) and callbacks such as `PlaneResultFcn`.
  Where upstream lets the count be less than the buffer (hull builders), the count stays explicit.
- **Out-parameters → multiple returns.** `b3GetLengthAndNormalize(&length, v)` →
  `GetLengthAndNormalize(v) (Vec3, float32)`; `b3GetAxisAngle` likewise.
- **Destroy methods.** `b3DestroyX(id)` → `id.Destroy()`; `b3DestroyJoint(id, wake)` →
  `id.Destroy(wake)`, `b3DestroyShape(id, updateMass)` → `id.Destroy(updateMass)`.
- **Strings.** `const char*` → `string`; `""` stands for `NULL` (`GetName()` returns `""` for no
  name; recordings encode `""` as upstream's NULL).
- **`void*`.** `userData` and callback `context` are `any`.
- **Opaque handles.** `b3Recording*` and `b3RecPlayer*` are `*Recording` / `*RecPlayer` with
  unexported fields; the `RecPlayer` methods are nil-safe like upstream's NULL checks.
- **Geometry blobs.** `b3HullData`, `b3MeshData`, `b3HeightFieldData`, `b3CompoundData` keep
  upstream's byte image (C layout, 8-byte aligned, lane padding); the Go value is the header with
  the arrays following it in one allocation, and the accessors return slices into it.
- **Compile-time options.** `B3_ASSERT` / `B3_VALIDATE` are the `box3d_noassert` /
  `box3d_validate` build tags; the `#ifndef` overridable limits are fixed constants. The SIMD
  backends (`simd_neon.h`, `simd_sse2.h`, `simd_avx2.h`) are opt-in: the default build runs the
  scalar lanes (upstream's `BOX3D_DISABLE_SIMD` code path), `GOEXPERIMENT=simd` with
  `-tags box3d_simd` runs the NEON / SSE 4-lane backend and, on amd64 with AVX2, the AVX2 8-lane
  one; `box3d_no8` is upstream's `BOX3D_DISABLE_AVX2`.
- **Errors.** None added: upstream reports failure with null ids and bools, so does the port.

## Determinism and byte compatibility

From `PROGRESS.md` (port complete, milestones 1-6 done):

- `test_determinism.c` passes with upstream's float golden hashes verbatim — ragdoll at 1–5
  workers, wave pile 1–4, query spawn 1–4, mesh drop 1 and 4, cross-platform at 1 — on
  darwin/arm64 and darwin/amd64 (Rosetta), and under `box3d_noassert`, `box3d_validate` and
  `-race`.
- Cross-platform: the whole suite (415 specs, every golden and `test_determinism.c` hash) passes
  unchanged on darwin/arm64, windows/amd64 and linux/amd64, the last also with `GOAMD64=v3`
  (AVX2/FMA hardware) and with `box3d_validate` (413 specs, the two broad phase golden specs skip
  by design), via `tools/xcheck` on 2026-10-17. No platform-specific goldens.
- Go-vs-C goldens (generators in `testdata/*_gen.c`, built against the submodule with
  `-ffp-contract=off`) match bit for bit for foundation math, hulls, manifolds, mesh / height
  field / compound geometry, broad phase, dynamic tree, contact solver, solver kernels, every joint
  type and a 240-step world scene, at 1, 2 and 4 workers. The C output is itself identical across
  NEON, SSE2, `BOX3D_DISABLE_SIMD` and `BOX3D_VALIDATE` builds, so the scalar Go lanes reproduce
  upstream's SIMD builds.
- SIMD build (`GOEXPERIMENT=simd`, `-tags box3d_simd`): the same suite, every golden and
  `test_determinism.c` hash unchanged, passes on darwin/arm64 (width 4, NEON), and on windows/amd64
  and linux/amd64 at native width 8 (AVX2) and forced width 4 (`BOX3D_TEST_FORCE_WIDTH4=1`), via
  `tools/xcheck` on 2026-10-17. `SIMDWidthTest` and `CrossWidthReplay` run (width 4 vs 8 bit for
  bit) on amd64 and take upstream's native-width-4 skip elsewhere.
- FMA: every multiply-add is split with explicit `float32()` rounding; `fma_guard_test.go` checks
  the compiled package for arm64 and amd64 v3.
- Recording / replay: the `.b3rec` op stream and the geometry registry are byte identical to C
  (joint ops included); C-made recordings replay in Go bit for bit. World snapshots and player
  keyframes are Go-native and rejected across implementations by the layout hash.
- Geometry blobs and dynamic tree files (`Save` / `LoadDynamicTree`) are byte compatible with C.

## Tests

```bash
cd libs/box3d-go
go vet ./... && go test ./...
go test -tags box3d_validate ./...
go test -tags box3d_noassert ./...
GOEXPERIMENT=simd go test -tags box3d_simd ./...
```

Every upstream `test/test_*.c` file has a `*_test.go` counterpart with every assertion; see the
`test/` rows of `PROGRESS.md`. Benchmarks are in `PERFORMANCE.md`.
