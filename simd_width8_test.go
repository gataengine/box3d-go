// Go-only test: upstream checks the widths against each other only in SIMDWidthTest
// (test/test_determinism.c) and CrossWidthReplay (test/test_recording.c).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// width8PyramidWorld builds a pyramid of boxCount boxes (rows of 5, 4, 3, … boxes, the last row
// partial) on a ground box at the current SIMD width.
func width8PyramidWorld(boxCount int) (WorldID, []BodyID) {
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = 1
	worldId := CreateWorld(&worldDef)

	groundDef := DefaultBodyDef()
	groundId := worldId.CreateBody(&groundDef)
	groundBox := MakeBoxHull(20.0, 1.0, 20.0)
	groundShape := DefaultShapeDef()
	groundId.CreateHullShape(&groundShape, &groundBox.Base)

	box := MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0

	bodyIds := make([]BodyID, 0, boxCount)
	for row, width := 0, 5; len(bodyIds) < boxCount; row, width = row+1, max(width-1, 1) {
		for column := 0; column < width && len(bodyIds) < boxCount; column++ {
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			x := 1.05*float32(column) - 0.525*float32(width-1)
			bodyDef.Position = Pos{x, 1.5 + 1.02*float32(row), 0.01 * float32(row)}
			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&shapeDef, &box.Base)
			bodyIds = append(bodyIds, bodyId)
		}
	}
	return worldId, bodyIds
}

// width8WorldHash hashes the transforms, velocities and stored contact impulses of bodies.
func width8WorldHash(hash uint32, bodyIds []BodyID) uint32 {
	for _, bodyId := range bodyIds {
		xf := bodyId.GetTransform()
		hash = Hash(hash, scenesBytesOf(&xf))
		v := bodyId.GetLinearVelocity()
		w := bodyId.GetAngularVelocity()
		hash = hashFloats(hash, []float32{v.X, v.Y, v.Z, w.X, w.Y, w.Z})

		var contactData [8]ContactData
		contactCount := bodyId.GetContactData(contactData[:])
		hash = determinismHashManifolds(hash, contactData[:], contactCount)
	}
	return hash
}

// width8PyramidHash steps a pyramid of boxCount boxes 120 times at width and returns the world
// hash and the width the world ran at.
func width8PyramidHash(boxCount, width int) (uint32, int32) {
	SetSIMDWidth(width)
	worldId, bodyIds := width8PyramidWorld(boxCount)
	SetSIMDWidth(suiteSIMDWidth)

	hash := uint32(HashInit)
	for range 120 {
		worldId.Step(1.0/60.0, 4)
		hash = width8WorldHash(hash, bodyIds)
	}
	worldWidth := getWorldFromId(worldId).simdWidth
	worldId.Destroy()
	return hash, worldWidth
}

var _ = Describe("SIMDWidth8", func() {
	BeforeEach(func() {
		if wide8 == nil {
			Skip("width 8 backend not available")
		}
	})

	It("Width8PartialBlocks", func() {
		for _, boxCount := range []int{5, 9, 13} {
			narrow, narrowWidth := width8PyramidHash(boxCount, 4)
			wide, wideWidth := width8PyramidHash(boxCount, 8)
			ensure(narrowWidth == 4 && wideWidth == 8)
			ensure(narrow == wide)
		}
	})

	It("MixedWidthWorlds", func() {
		SetSIMDWidth(4)
		worldA, bodiesA := width8PyramidWorld(13)
		SetSIMDWidth(8)
		worldB, bodiesB := width8PyramidWorld(13)
		SetSIMDWidth(suiteSIMDWidth)

		ensure(getWorldFromId(worldA).simdWidth == 4)
		ensure(getWorldFromId(worldB).simdWidth == 8)

		hashA, hashB := uint32(HashInit), uint32(HashInit)
		for range 120 {
			worldA.Step(1.0/60.0, 4)
			worldB.Step(1.0/60.0, 4)
			hashA = width8WorldHash(hashA, bodiesA)
			hashB = width8WorldHash(hashB, bodiesB)
			ensure(hashA == hashB)
		}

		ensure(getWorldFromId(worldA).simdWidth == 4)
		ensure(getWorldFromId(worldB).simdWidth == 8)
		worldA.Destroy()
		worldB.Destroy()
	})
})
