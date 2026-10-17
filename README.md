# box3d-go

Pure-Go port of [Box3D](https://github.com/erincatto/box3d), Erin Catto's 3D rigid body physics
engine. A file-by-file transliteration: every upstream source and test file has a Go counterpart
with the same functions in the same order, so upstream changes can be re-applied per file.

Work in progress. See [PROGRESS.md](PROGRESS.md) for what is ported and
[LOOSE_ENDS.md](LOOSE_ENDS.md) for every deviation from upstream.

```bash
go get github.com/gataengine/box3d-go
```

## Conventions

- Upstream names lose the `b3` prefix: `b3MakeBoxHull` → `MakeBoxHull`, `b3Body_SetTransform(id, …)`
  → `id.SetTransform(…)`, `B3_MAX_MANIFOLD_POINTS` → `MaxManifoldPoints`.
- Math types (`Vec3`, `Quat`, `Transform`, `Matrix3`, `AABB`) are plain structs; math functions are
  free functions (`Add`, `Cross`, `RotateVector`) so expressions read like the C.
- Float32 only. `Pos` and `WorldTransform` are distinct types so a double precision mode can be
  added later.
- Determinism: multiply-adds are written with explicit `float32()` rounding so Go never emits
  fused multiply-add; `go test` checks the compiled package for arm64 and amd64 v3.

## Build tags

- `box3d_noassert` compiles out `B3_ASSERT` checks.
- `box3d_validate` enables `B3_VALIDATE` checks (off by default, as upstream).

## License

MIT, same as upstream. Copyright (c) 2026 Erin Catto; Go port by the gataengine authors.
