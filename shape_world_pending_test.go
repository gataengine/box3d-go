//go:build !box3d_m3_integration

// Port of test/test_shape.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// shapeWorldSpecs holds the test_shape.c subtests that need a world. Without the
// box3d_m3_integration build tag they are pending; shape_world_test.go has their bodies.
// PORT: remove this file and the tag once body.c, physics_world.c and name_cache.c are merged.
func shapeWorldSpecs() {
	PIt("ShapeNameTest (needs body.c, physics_world.c, name_cache.c; build tag box3d_m3_integration)")
	PIt("ShapeFlagsTest (needs body.c, physics_world.c; build tag box3d_m3_integration)")
}
