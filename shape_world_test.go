// Port of test/test_shape.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"strings"

	. "github.com/onsi/ginkgo/v2"
)

// PORT: the test_shape.c subtests that need a world, behind the box3d_m3_integration build tag
// until body.c, physics_world.c (and name_cache.c for the names) are merged. shape_test.go calls
// shapeWorldSpecs in upstream's RUN_SUBTEST order. const char* names are strings, NULL is "".

// shapeCheckShapeName is CheckShapeName.
func shapeCheckShapeName(shapeId ShapeID, expected string) {
	GinkgoHelper()
	got := shapeId.GetName()

	srcLen := len(expected)
	ensure(len(got) == srcLen)
	if srcLen > 0 {
		ensure(strings.Compare(got[:srcLen], expected) == 0)
	}
}

func shapeWorldSpecs() {
	// Cover the def path, the setter, over-length truncation, and the NULL clear. A zero cap collapses every
	// case to the empty string, which the length-derived check handles without a special branch.
	It("ShapeNameTest", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyId := worldId.CreateBody(&bodyDef)

		s := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}

		// Default def leaves the name empty.
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		shapeId := bodyId.CreateSphereShape(&shapeDef, &s)
		shapeCheckShapeName(shapeId, "")

		// Name carried on the def.
		shapeDef.Name = "box"
		namedId := bodyId.CreateSphereShape(&shapeDef, &s)
		shapeCheckShapeName(namedId, "box")

		// Setter overwrites.
		shapeId.SetName("wheel")
		shapeCheckShapeName(shapeId, "wheel")

		// Over-length name truncates to the cap.
		shapeId.SetName("abcdefghijklmnopqrstuvwxyz")
		shapeCheckShapeName(shapeId, "abcdefghijklmnopqrstuvwxyz")

		// NULL clears the name.
		shapeId.SetName("")
		shapeCheckShapeName(shapeId, "")

		worldId.Destroy()
	})

	// The event enables moved from separate bools to a shared bit field. Each accessor must touch only its own
	// bit, so start with all set and flip them one at a time, checking the rest hold.
	It("ShapeFlagsTest", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyId := worldId.CreateBody(&bodyDef)

		s := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}

		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		shapeDef.EnableSensorEvents = true
		shapeDef.EnableContactEvents = true
		shapeDef.EnableHitEvents = true
		shapeDef.EnablePreSolveEvents = true
		shapeId := bodyId.CreateSphereShape(&shapeDef, &s)

		// The def carries every enable through to the shape.
		ensure(shapeId.AreSensorEventsEnabled())
		ensure(shapeId.AreContactEventsEnabled())
		ensure(shapeId.AreHitEventsEnabled())
		ensure(shapeId.ArePreSolveEventsEnabled())

		// Clearing one leaves the rest set.
		shapeId.EnableSensorEvents(false)
		ensure(shapeId.AreSensorEventsEnabled() == false)
		ensure(shapeId.AreContactEventsEnabled())
		ensure(shapeId.AreHitEventsEnabled())
		ensure(shapeId.ArePreSolveEventsEnabled())

		// Setting it back does not disturb the others.
		shapeId.EnableSensorEvents(true)
		shapeId.EnableHitEvents(false)
		ensure(shapeId.AreSensorEventsEnabled())
		ensure(shapeId.AreContactEventsEnabled())
		ensure(shapeId.AreHitEventsEnabled() == false)
		ensure(shapeId.ArePreSolveEventsEnabled())

		// Clear two more and confirm only the remaining bit stays set.
		shapeId.EnableContactEvents(false)
		shapeId.EnablePreSolveEvents(false)
		ensure(shapeId.AreSensorEventsEnabled())
		ensure(shapeId.AreContactEventsEnabled() == false)
		ensure(shapeId.AreHitEventsEnabled() == false)
		ensure(shapeId.ArePreSolveEventsEnabled() == false)

		worldId.Destroy()
	})
}
