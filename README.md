# box3d-go

Pure-Go port of [Box3D](https://github.com/erincatto/box3d), Erin Catto's 3D rigid body physics
engine, at upstream commit `efca0c184d40fd6a82016983f1601c4d23965867` (`b3GetVersion` 0.2.0).

It is a file-by-file transliteration, not a reimplementation: every upstream source and test file
has a Go counterpart with the same functions in the same order, so upstream changes can be
re-applied per file. No cgo, no assembly. Upstream's determinism tests pass with its own golden
hashes.

- [PROGRESS.md](PROGRESS.md): file map, status of every upstream file, determinism status
- [LOOSE_ENDS.md](LOOSE_ENDS.md): every place the port is not a literal transliteration
- [PARITY_REPORT.md](PARITY_REPORT.md): public API audit, intentional omissions, API shape
- [PERFORMANCE.md](PERFORMANCE.md): benchmarks

## Install

```bash
go get github.com/gataengine/box3d-go
```

```go
import box3d "github.com/gataengine/box3d-go"
```

Requires Go 1.27. One package, `box3d`, at the module root.

## Usage

A box dropped onto a static ground box (this is `Example` in
[example_test.go](example_test.go), so `go test` checks it):

```go
worldDef := box3d.DefaultWorldDef()
worldDef.Gravity = box3d.Vec3{X: 0, Y: -10, Z: 0}
world := box3d.CreateWorld(&worldDef)
defer world.Destroy()

// Static ground: a 40 x 2 x 40 box whose top face is at y = 0.
groundDef := box3d.DefaultBodyDef()
groundDef.Position = box3d.Pos{X: 0, Y: -1, Z: 0}
ground := world.CreateBody(&groundDef)
groundBox := box3d.MakeBoxHull(20, 1, 20)
shapeDef := box3d.DefaultShapeDef()
ground.CreateHullShape(&shapeDef, &groundBox.Base)

// Dynamic unit cube dropped from 4 m.
bodyDef := box3d.DefaultBodyDef()
bodyDef.Type = box3d.DynamicBody
bodyDef.Position = box3d.Pos{X: 0, Y: 4, Z: 0}
body := world.CreateBody(&bodyDef)
cube := box3d.MakeCubeHull(0.5)
shapeDef.Density = 1
body.CreateHullShape(&shapeDef, &cube.Base)

for i := 0; i < 120; i++ {
	world.Step(1.0/60.0, 4)
}

p := body.GetPosition()
fmt.Printf("y=%.2f\n", p.Y) // y=0.50
```

Upstream's documentation and samples apply directly once names are mapped as below.

## API mapping

The public API is upstream's, under fixed naming rules (full table in
[PARITY_REPORT.md](PARITY_REPORT.md#naming-rules)):

| C | Go |
|---|---|
| `b3MakeBoxHull(…)` | `MakeBoxHull(…)` |
| `b3Body_SetTransform(bodyId, p, q)` | `bodyId.SetTransform(p, q)` (methods on `WorldID`, `BodyID`, `ShapeID`, `JointID`, `ContactID`) |
| `b3RevoluteJoint_SetSpringHertz(jointId, hz)` | `jointId.RevoluteJointSetSpringHertz(hz)` |
| `b3CreateBody(worldId, &def)`, `b3CreateRevoluteJoint(worldId, &def)` | `worldId.CreateBody(&def)`, `worldId.CreateRevoluteJoint(&def)` |
| `b3CreateHullShape(bodyId, &def, hull)` | `bodyId.CreateHullShape(&def, hull)` |
| `b3DestroyBody(bodyId)` | `bodyId.Destroy()` |
| `b3RecPlayer_StepFrame(player)`, `b3DestroyPlayer(player)` | `player.StepFrame()`, `player.Destroy()` |
| `b3DynamicTree_Create(n)`, `b3DynamicTree_Query(&tree, …)` | `CreateDynamicTree(n)`, `tree.Query(…)` |
| `b3BodyId`, `b3_dynamicBody`, `b3_toiStateHit` | `BodyID`, `DynamicBody`, `TOIStateHit` |
| `B3_MAX_MANIFOLD_POINTS` | `MaxManifoldPoints` |
| `B3_LINEAR_SLOP` (scaled by length units) | `LinearSlop()` |

Shape of the API:

- Pointer + count (or capacity) becomes a slice: `bodyId.GetShapes(shapes []ShapeID) int`.
- Out-parameters become extra return values: `GetLengthAndNormalize(v) (Vec3, float32)`.
- `const char*` is `string` (`""` for `NULL`); `void*` user data and callback contexts are `any`.
- Ids are comparable value structs; the zero value is the null id. There are no Go errors:
  failure is a null id or `false`, as upstream.
- Math types (`Vec3`, `Quat`, `Transform`, `Matrix3`, `AABB`) are plain structs and math
  functions are free functions (`Add`, `Cross`, `RotateVector`) so expressions read like the C.
- Geometry (`HullData`, `MeshData`, `HeightFieldData`, `CompoundData`) keeps upstream's byte
  image; accessors such as `GetHullPoints` return slices into it.
- Float32 only. `Pos` and `WorldTransform` are distinct types so a double precision mode can be
  added later.

Not ported: the allocator hooks (`b3SetAllocator`), double precision, Tracy, and the samples
app. The default build runs upstream's scalar 4-lane backend (`GetSIMDWidth()` is 4); with
`box3d_simd` (and `GOEXPERIMENT=simd`) it runs the NEON / SSE 4-lane one, and on amd64 with AVX2
the AVX2 8-lane one (`GetSIMDWidth()` is 8; `box3d_no8` leaves it out, as upstream's
`BOX3D_DISABLE_AVX2`). See
[PARITY_REPORT.md](PARITY_REPORT.md#intentionally-not-ported).

## Determinism

- Upstream's `test_determinism.c` passes with its float golden hashes verbatim: ragdoll at 1–5
  workers, wave pile and query spawn at 1–4, mesh drop at 1 and 4, cross-platform at 1.
- Go-vs-C goldens (`*_golden_test.go`, generators in `testdata/*_gen.c` built against upstream)
  match bit for bit for math, collision, broad phase, solver, every joint type and a full world
  scene, at 1, 2 and 4 workers. Results do not depend on the worker count.
- Verified on darwin/arm64 and darwin/amd64, under the default build, `box3d_noassert`,
  `box3d_validate` and `-race`; and on windows/amd64 and linux/amd64 (default and
  `box3d_validate`; linux also with `GOAMD64=v3` on AVX2/FMA hardware) through the monorepo's
  `tools/xcheck`. The goldens are the same on every platform.
- FMA rule: Go fuses `a + b*c` into a fused multiply-add on arm64 (and amd64 v3); upstream builds
  with `-ffp-contract=off`. Every multiply that feeds an add is wrapped in an explicit `float32()`
  conversion (`mulAdd` / `mulSub`). `fma_guard_test.go` compiles the package for arm64 and
  amd64 v3 and fails on any fused instruction in the assembly listing (skipped under `-short`). Code added to the package must follow the rule.
- Worker count: `WorldDef.WorkerCount` (default 1) runs a goroutine pool with upstream's
  scheduler semantics; custom task systems plug in through `EnqueueTask` / `FinishTask` as
  upstream.

## Build tags

| tag | effect |
|---|---|
| (none) | `B3_ASSERT` checks on (panic via the handler set with `SetAssertFcn`), `B3_VALIDATE` off, as upstream's default build |
| `box3d_noassert` | compiles out `B3_ASSERT` (upstream's `NDEBUG`) |
| `box3d_validate` | enables `B3_VALIDATE` and the `B3_ENABLE_VALIDATION` checks (slow; for debugging) |
| `box3d_simd` | with `GOEXPERIMENT=simd`: the SIMD lane backend on `simd/archsimd` (arm64 NEON, amd64 AVX-encoded SSE; requires AVX on amd64), bit-exact with the default build; other architectures keep the scalar lanes |

```bash
go test ./...
go test -tags box3d_validate ./...
go test -tags box3d_noassert ./...
GOEXPERIMENT=simd go test -tags box3d_simd ./...
```

## Recording and replay

`CreateRecording`, `worldId.StartRecording`, `SaveRecordingToFile`, `CreatePlayer` and the
`(*RecPlayer)` methods are ported. Compatibility with upstream `.b3rec` files comes in three tiers:

| tier | status |
|---|---|
| op stream (API calls and their defs) | byte identical to C; recordings made by upstream (the samples) replay in Go bit for bit |
| geometry registry (hull, mesh, height field, compound images) | byte identical to C |
| world snapshots and player keyframes | Go-native. A layout hash rejects C images before the world is touched; seek and scrub work on recordings Go made |

A C recording that starts from a non-empty world (a C snapshot seed) is rejected; one started on an
empty world replays.

## Updating to a newer upstream

From the spec (`docs/superpowers/specs/2026-10-17-box3d-go-port-design.md`, "Upstream bumps"):

1. `git -C third_party/box3d fetch && git -C third_party/box3d log --oneline <old>..<new>`.
2. For each changed upstream file, `git -C third_party/box3d diff <old> <new> -- src/foo.c test/test_foo.c`;
   apply the hunks to the Go files by hand and update the file's "ported at" column in
   `PROGRESS.md`. Deviations get a `// PORT:` comment and a `LOOSE_ENDS.md` line.
3. Re-pin the goldens from the new `test_determinism.c`; regenerate the `testdata/*_gen.c`
   outputs against the new C build. If they change upstream, they change here.
4. Run `tools/parity_audit.sh` and port any new public symbol.
5. Bump the submodule pointer and the commit hash in `PROGRESS.md` (and this README) in the same
   commit.

## License

MIT, same as upstream. Copyright (c) 2026 Erin Catto; Go port by the gataengine authors.
