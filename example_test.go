// Go-only: the README usage example, compiled and checked by go test.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d_test

import (
	"fmt"

	box3d "github.com/gataengine/box3d-go"
)

// A box falls onto a static ground box and comes to rest.
func Example() {
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
	fmt.Printf("y=%.2f\n", p.Y)
	// Output: y=0.50
}
