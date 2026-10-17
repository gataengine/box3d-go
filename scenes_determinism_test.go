//go:build box3d_m4_joints

// Port of shared/determinism.h and shared/determinism.c from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "unsafe"

// PORT: test scene helper (spec: shared/ scenes are test helpers), in a _test.go file of package
// box3d because the internal tests cannot import a package that imports box3d. Behind the
// box3d_m4_joints build tag until the joints the ragdolls use (human.c, milestone 4) are merged. assert() from <assert.h> is ensure(). Products feeding an add are
// rounded explicitly (float32() / mulAdd) so the scenes match the C build bit for bit when the
// determinism goldens arrive. b3Hash over a struct hashes its bytes (scenesBytesOf).

// ---- determinism.h ----

const (
	ragdollGroupSize = 2 // RAGDOLL_GROUP_SIZE
	ragdollGridCount = 2 // RAGDOLL_GRID_COUNT
)

// ragdollGroup is RagdollGroup.
type ragdollGroup struct {
	humans [ragdollGroupSize]human
}

// fallingRagdollData is FallingRagdollData.
type fallingRagdollData struct {
	groups      [ragdollGridCount * ragdollGridCount]ragdollGroup
	gridMesh    *MeshData
	torusMesh   *MeshData
	columnCount int
	columnIndex int
	stepCount   int
	sleepStep   int
	hash        uint32
}

// Convex pile dropped on a wave height field. Spheres, capsules, boxes, and rocks with
// rolling resistance so the pile sleeps quickly.
const wavePileBodyCount = 100 // WAVE_PILE_BODY_COUNT

// wavePileData is WavePileData.
type wavePileData struct {
	bodies      [wavePileBodyCount]BodyID
	heightField *HeightFieldData
	stepCount   int
	sleepStep   int
	hash        uint32
}

// Query driven spawning in an empty zero gravity world. Each step casts a ray, overlaps an
// AABB, and casts a sphere, then spawns a shape whose position, type, and size depend on the
// query results. Any query divergence cascades into the final state.
const (
	querySpawnCount      = 50  // QUERY_SPAWN_COUNT
	querySpawnCastRadius = 0.5 // QUERY_SPAWN_CAST_RADIUS
)

// querySpawnData is QuerySpawnData.
type querySpawnData struct {
	bodies        [querySpawnCount]BodyID
	spawnCount    int
	queryHitCount int
	queryHash     uint32
	stepCount     int
	sleepStep     int
	hash          uint32

	// Last query cycle, recorded for visualization
	rayOrigin         Pos
	rayTranslation    Vec3
	rayPoint          Pos
	rayNormal         Vec3
	rayDidHit         bool
	overlapBounds     AABB
	overlapCount      int
	castFraction      float32
	lastSpawnPosition Pos
}

// ---- determinism.c ----

const determinismGridSize float32 = 15.0 // GRID_SIZE

// scenesBytesOf views a value as its bytes for b3Hash( hash, (uint8_t*)&x, sizeof( x ) ).
func scenesBytesOf[T any](x *T) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(x)), unsafe.Sizeof(*x))
}

// determinismCreateGroup is CreateGroup.
func determinismCreateGroup(data *fallingRagdollData, worldId WorldID, rowIndex, columnIndex int) {
	ensure(rowIndex < ragdollGridCount && columnIndex < ragdollGridCount)

	groupIndex := rowIndex*ragdollGridCount + columnIndex

	span := float32(ragdollGridCount) * determinismGridSize
	groupDistance := float32(1.0*span) / ragdollGridCount

	var position Pos
	position.X = float32(-0.5*span) + float32(groupDistance*(float32(columnIndex)+0.5))
	position.Y = 15.0
	position.Z = float32(-0.5*span) + float32(groupDistance*(float32(rowIndex)+0.5))

	frictionTorque := float32(5.0)
	hertz := float32(1.0)
	dampingRatio := float32(0.7)
	colorize := false

	for i := range ragdollGroupSize {
		human := &data.groups[groupIndex].humans[i]
		createHuman(human, worldId, position, frictionTorque, hertz, dampingRatio, groupIndex, nil, colorize)
		position.X += 0.75
	}
}

// createFallingRagdolls is CreateFallingRagdolls.
func createFallingRagdolls(worldId WorldID) fallingRagdollData {
	data := fallingRagdollData{}

	halfMeshGridRows := 4
	meshGridCellWidth := determinismGridSize / (2.0 * float32(halfMeshGridRows))
	data.gridMesh = CreateGridMesh(2*halfMeshGridRows, 2*halfMeshGridRows, meshGridCellWidth, 0, true)
	data.torusMesh = CreateTorusMesh(16, 16, float32(0.25*determinismGridSize), 1.0)

	span := determinismGridSize * ragdollGridCount
	bodyDef := DefaultBodyDef()
	shapeDef := DefaultShapeDef()

	bodyDef.Position.X = float32(-0.5*span) + float32(0.5*determinismGridSize)
	for i := range ragdollGridCount {
		bodyDef.Position.Z = float32(-0.5*span) + float32(0.5*determinismGridSize)
		for j := range ragdollGridCount {
			body := worldId.CreateBody(&bodyDef)
			body.CreateMeshShape(&shapeDef, data.gridMesh, Vec3One)
			body.CreateMeshShape(&shapeDef, data.torusMesh, Vec3One)

			determinismCreateGroup(&data, worldId, i, j)

			bodyDef.Position.Z += determinismGridSize
		}

		bodyDef.Position.X += determinismGridSize
	}

	return data
}

// updateFallingRagdolls is UpdateFallingRagdolls.
func updateFallingRagdolls(worldId WorldID, data *fallingRagdollData) bool {
	if data.hash == 0 {
		bodyEvents := worldId.GetBodyEvents()

		if bodyEvents.MoveCount == 0 {
			awakeCount := worldId.GetAwakeBodyCount()
			ensure(awakeCount == 0)

			data.hash = HashInit
			for i := range ragdollGridCount {
				for j := range ragdollGridCount {
					for k := range ragdollGroupSize {
						groupIndex := i*ragdollGridCount + j

						human := &data.groups[groupIndex].humans[k]

						for b := range boneCount {
							bodyId := human.bones[b].bodyId
							xf := bodyId.GetTransform()
							data.hash = Hash(data.hash, scenesBytesOf(&xf))
						}
					}
				}
			}

			data.sleepStep = data.stepCount
		}
	}

	data.stepCount += 1

	return data.hash != 0
}

// destroyFallingRagdolls is DestroyFallingRagdolls.
func destroyFallingRagdolls(data *fallingRagdollData) {
	DestroyMesh(data.gridMesh)
	DestroyMesh(data.torusMesh)

	data.gridMesh = nil
	data.torusMesh = nil
}

const (
	wavePileGrid   = 5 // WAVE_PILE_GRID
	wavePileLayers = 4 // WAVE_PILE_LAYERS
)

// createWavePile is CreateWavePile.
func createWavePile(worldId WorldID) wavePileData {
	data := wavePileData{}

	gRandomSeed = 52977

	// Height fields grow from a corner, offset the body to center the patch.
	fieldCount := 21
	fieldScale := Vec3{1.0, 0.6, 1.0}
	data.heightField = CreateWave(fieldCount, fieldCount, fieldScale, 0.08, 0.06, false)

	{
		extent := fieldScale.X * float32(fieldCount-1)
		bodyDef := DefaultBodyDef()
		bodyDef.Position.X = -0.5 * extent
		bodyDef.Position.Z = -0.5 * extent
		groundId := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		groundId.CreateHeightFieldShape(&shapeDef, data.heightField)
	}

	rock := CreateRock(0.55)
	box := MakeBoxHull(0.45, 0.3, 0.55)
	sphere := Sphere{Vec3Zero, 0.5}
	capsule := Capsule{Vec3{0.0, -0.3, 0.0}, Vec3{0.0, 0.3, 0.0}, 0.35}

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody

	// Rolling resistance so the pile sleeps quickly.
	shapeDef := DefaultShapeDef()
	shapeDef.BaseMaterial.RollingResistance = 0.3

	spacing := float32(1.7)
	index := 0
	for layer := range wavePileLayers {
		for i := range wavePileGrid {
			for j := range wavePileGrid {
				jitter := randomVec3Uniform(-0.3, 0.3)
				// PORT: 0.5f * ( WAVE_PILE_GRID - 1 ) is the constant 2.0f.
				bodyDef.Position.X = float32(spacing*(float32(i)-0.5*(wavePileGrid-1))) + jitter.X
				bodyDef.Position.Y = float32(2.5+float32(1.6*float32(layer))) + float32(0.3*jitter.Y)
				bodyDef.Position.Z = float32(spacing*(float32(j)-0.5*(wavePileGrid-1))) + jitter.Z
				bodyDef.Rotation = randomQuat()

				bodyId := worldId.CreateBody(&bodyDef)
				data.bodies[index] = bodyId

				switch index % 4 {
				case 0:
					bodyId.CreateSphereShape(&shapeDef, &sphere)
				case 1:
					bodyId.CreateCapsuleShape(&shapeDef, &capsule)
				case 2:
					bodyId.CreateHullShape(&shapeDef, &box.Base)
				default:
					bodyId.CreateHullShape(&shapeDef, rock)
				}

				index += 1
			}
		}
	}

	ensure(index == wavePileBodyCount)

	// The world keeps its own copy.
	DestroyHull(rock)

	return data
}

// updateWavePile is UpdateWavePile.
func updateWavePile(worldId WorldID, data *wavePileData) bool {
	if data.hash == 0 {
		if worldId.GetAwakeBodyCount() == 0 {
			data.hash = HashInit
			for i := range wavePileBodyCount {
				xf := data.bodies[i].GetTransform()
				data.hash = Hash(data.hash, scenesBytesOf(&xf))
			}

			data.sleepStep = data.stepCount
		}
	}

	data.stepCount += 1

	return data.hash != 0
}

// destroyWavePile is DestroyWavePile.
func destroyWavePile(data *wavePileData) {
	DestroyHeightField(data.heightField)
	data.heightField = nil
}

// querySpawnOverlapContext is QuerySpawnOverlapContext.
type querySpawnOverlapContext struct {
	data  *querySpawnData
	count int
}

// querySpawnOverlapCallback is QuerySpawnOverlapCallback.
func querySpawnOverlapCallback(shapeId ShapeID, context any) bool {
	overlap := context.(*querySpawnOverlapContext)
	overlap.count += 1
	overlap.data.queryHitCount += 1
	overlap.data.queryHash = Hash(overlap.data.queryHash, scenesBytesOf(&shapeId.Index1))
	return true
}

// querySpawnCastCallback is QuerySpawnCastCallback.
func querySpawnCastCallback(shapeId ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64,
	triangleIndex int, childIndex int, context any) float32 {
	closest := context.(*float32)
	*closest = fraction
	return fraction
}

// querySpawnOnce is QuerySpawnOnce.
func querySpawnOnce(worldId WorldID, data *querySpawnData) {
	filter := DefaultQueryFilter()

	// Ray result decides the spawn position, a miss seeds the cloud near the origin.
	rayOrigin := randomPos(Vec3{-12.0, -12.0, -12.0}, Vec3{12.0, 12.0, 12.0})
	rayTranslation := MulSV(30.0, randomUnitVector())

	var spawnPosition Pos
	ray := worldId.CastRayClosest(rayOrigin, rayTranslation, filter)
	if ray.Hit {
		data.queryHitCount += 1
		data.queryHash = Hash(data.queryHash, scenesBytesOf(&ray.Fraction))
		data.queryHash = Hash(data.queryHash, scenesBytesOf(&ray.Normal))
		spawnPosition = OffsetPos(ray.Point, MulSV(1.2, ray.Normal))
	} else {
		spawnPosition = randomPos(Vec3{-6.0, -6.0, -6.0}, Vec3{6.0, 6.0, 6.0})
	}

	data.rayOrigin = rayOrigin
	data.rayTranslation = rayTranslation
	data.rayDidHit = ray.Hit
	if ray.Hit {
		data.rayPoint = ray.Point
		data.rayNormal = ray.Normal
	} else {
		data.rayPoint = OffsetPos(rayOrigin, rayTranslation)
		data.rayNormal = Vec3Zero
	}

	// Overlap count picks the shape type.
	center := randomVec3Uniform(-10.0, 10.0)
	extent := randomFloatRange(1.0, 4.0)
	var aabb AABB
	aabb.LowerBound = Vec3{center.X - extent, center.Y - extent, center.Z - extent}
	aabb.UpperBound = Vec3{center.X + extent, center.Y + extent, center.Z + extent}

	overlap := querySpawnOverlapContext{data, 0}
	worldId.OverlapAABB(aabb, filter, querySpawnOverlapCallback, &overlap)

	data.overlapBounds = aabb
	data.overlapCount = overlap.count

	// Sphere cast fraction sets the spawn size.
	fraction := float32(1.0)
	proxyPoint := Vec3Zero
	proxy := ShapeProxy{unsafe.Slice(&proxyPoint, 1), 1, querySpawnCastRadius}
	worldId.CastShape(rayOrigin, &proxy, rayTranslation, filter, querySpawnCastCallback, &fraction)
	if fraction < 1.0 {
		data.queryHitCount += 1
		data.queryHash = Hash(data.queryHash, scenesBytesOf(&fraction))
	}

	data.castFraction = fraction

	size := 0.3 + float32(0.2*fraction)

	// Damping guarantees everything comes to rest.
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = spawnPosition
	bodyDef.Rotation = randomQuat()
	bodyDef.LinearVelocity = randomVec3Uniform(-0.2, 0.2)
	bodyDef.AngularVelocity = randomVec3Uniform(-0.5, 0.5)
	bodyDef.LinearDamping = 1.0
	bodyDef.AngularDamping = 1.0

	bodyId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	shapeDef.BaseMaterial.RollingResistance = 0.2

	switch (data.spawnCount + overlap.count) % 3 {
	case 0:
		sphere := Sphere{Vec3Zero, size}
		bodyId.CreateSphereShape(&shapeDef, &sphere)
	case 1:
		capsule := Capsule{Vec3{0.0, -size, 0.0}, Vec3{0.0, size, 0.0}, float32(0.7 * size)}
		bodyId.CreateCapsuleShape(&shapeDef, &capsule)
	default:
		box := MakeBoxHull(size, float32(0.7*size), float32(0.5*size))
		bodyId.CreateHullShape(&shapeDef, &box.Base)
	}

	data.bodies[data.spawnCount] = bodyId
	data.spawnCount += 1
	data.lastSpawnPosition = spawnPosition
}

// createQuerySpawn is CreateQuerySpawn.
func createQuerySpawn(worldId WorldID) querySpawnData {
	data := querySpawnData{}

	gRandomSeed = 71689

	// Empty space, motion comes only from spawn velocities and overlap pushes.
	worldId.SetGravity(Vec3Zero)

	return data
}

// updateQuerySpawn is UpdateQuerySpawn.
func updateQuerySpawn(worldId WorldID, data *querySpawnData) bool {
	if data.spawnCount < querySpawnCount {
		querySpawnOnce(worldId, data)
	} else if data.hash == 0 && worldId.GetAwakeBodyCount() == 0 {
		data.hash = HashInit
		for i := range querySpawnCount {
			xf := data.bodies[i].GetTransform()
			data.hash = Hash(data.hash, scenesBytesOf(&xf))
		}

		data.sleepStep = data.stepCount
	}

	data.stepCount += 1

	return data.hash != 0
}

// destroyQuerySpawn is DestroyQuerySpawn.
func destroyQuerySpawn(data *querySpawnData) {
}
