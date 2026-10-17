// Port of shared/stability.h and shared/stability.c from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: test scene helper (spec: shared/ scenes are test helpers), in a _test.go file of package
// box3d like scenes_determinism_test.go.

// ---- stability.h ----

const meshDropGridCount = 20 // MESH_DROP_GRID_COUNT

// Thin fast boxes dropped on a wave mesh. Stresses continuous collision and mesh contact
// stability, and doubles as a determinism scenario via the sleep hash.
//
// meshDropData is MeshDropData.
type meshDropData struct {
	mesh      *MeshData
	bodies    [meshDropGridCount * meshDropGridCount]BodyID
	stepCount int
	sleepStep int
	hash      uint32
}

// ---- stability.c ----

// createMeshDrop is CreateMeshDrop.
func createMeshDrop(worldId WorldID, origin Pos) meshDropData {
	data := meshDropData{}
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = origin
		groundId := worldId.CreateBody(&bodyDef)

		gridCount := 40
		cellWidth := float32(1.0)
		rowHz := float32(0.1)
		columnHz := float32(0.2)
		groundAmplitude := float32(0.5)

		data.mesh = CreateWaveMesh(gridCount, gridCount, cellWidth, groundAmplitude, rowHz, columnHz)
		shapeDef := DefaultShapeDef()
		shapeDef.Filter.CategoryBits = 1
		groundId.CreateMeshShape(&shapeDef, data.mesh, Vec3One)
	}

	{
		box := MakeBoxHull(0.02, 0.2, 0.04)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody

		shapeDef := DefaultShapeDef()
		shapeDef.BaseMaterial.RollingResistance = 0.1

		// Don't allow shapes to collide with each other.
		shapeDef.Filter.CategoryBits = 2
		shapeDef.Filter.MaskBits = 1

		gRandomSeed = 3963634789

		gridCount := meshDropGridCount

		for i := range gridCount {
			for j := range gridCount {
				linearVelocity := randomVec3Uniform(-1.0, 1.0)
				angularVelocity := randomVec3Uniform(-5.0, 5.0)

				bodyDef.Position = OffsetPos(origin, Vec3{
					float32(0.5 * (float32(i) - float32(0.5*float32(gridCount)))),
					5.0,
					float32(0.5 * (float32(j) - float32(0.5*float32(gridCount)))),
				})
				bodyDef.LinearVelocity = linearVelocity
				bodyDef.AngularVelocity = angularVelocity
				bodyId := worldId.CreateBody(&bodyDef)
				data.bodies[i*gridCount+j] = bodyId

				bodyId.CreateHullShape(&shapeDef, &box.Base)
			}
		}
	}

	return data
}

// updateMeshDrop is UpdateMeshDrop.
func updateMeshDrop(worldId WorldID, data *meshDropData) bool {
	if data.hash == 0 {
		if worldId.GetAwakeBodyCount() == 0 {
			data.hash = HashInit
			bodyCount := meshDropGridCount * meshDropGridCount
			for i := range bodyCount {
				xf := data.bodies[i].GetTransform()
				data.hash = Hash(data.hash, scenesBytesOf(&xf))
			}

			data.sleepStep = data.stepCount
		}
	}

	data.stepCount += 1

	return data.hash != 0
}

// destroyMeshDrop is DestroyMeshDrop.
func destroyMeshDrop(data *meshDropData) {
	DestroyMesh(data.mesh)
}
