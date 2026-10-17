// Package box3d is a pure-Go transliteration of Box3D (https://github.com/erincatto/box3d),
// Erin Catto's 3D rigid body physics engine.
//
// Every upstream source file has a Go counterpart with the same name and the same functions in
// the same order; see PROGRESS.md for the file map and LOOSE_ENDS.md for every place the port is
// not literal. Upstream's public API is kept under Go naming: b3Foo becomes Foo, b3Body_Foo(id)
// becomes id.Foo(), and b3CreateBody(worldId, &def) becomes world.CreateBody(&def).
//
// Determinism: all floating-point multiply-adds go through explicit float32 conversions so the
// compiler cannot fuse them into FMA instructions (arm64 does so by default). See mulAdd and
// fma_guard_test.go.
//
// SPDX-License-Identifier: MIT (upstream Box3D, Copyright (c) 2026 Erin Catto)
package box3d
