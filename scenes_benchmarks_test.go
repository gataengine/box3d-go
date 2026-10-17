// Port of shared/benchmarks.h and shared/benchmarks.c, and shared/overflow_color.h and
// shared/overflow_color.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"math"
	"unsafe"
)

// PORT: benchmark and test scene helpers (spec: shared/ scenes are test helpers), in a _test.go
// file of package box3d because the internal tests cannot import a package that imports box3d.
// BENCHMARK_DEBUG is the variable benchmarkDebug, which starts as debugBuild (NDEBUG is
// box3d_noassert) so the tests see upstream's debug sizes; bench_test.go clears it so the
// benchmarks always build upstream's release (NDEBUG) scenes. The compile time sizes that depend
// on it (RAIN_GRID_COUNT, STATIC_FLOOR_GRID, VILLAGE_GRID, ...) are functions. Multiply-adds are
// split with float32() conversions so arm64 does not fuse them. Constant chains that C rounds per
// operation (`-1.15f * B3_PI`) go through a variable so Go does not fold them exactly.

// ---- benchmarks.h / benchmarks.c ----

// benchmarkDebug is BENCHMARK_DEBUG.
var benchmarkDebug = debugBuild

// benchmarkGroundShapeId is g_groundShapeId.
var benchmarkGroundShapeId = ShapeID{}

// getGroundShapeId is GetGroundShapeId.
func getGroundShapeId() ShapeID {
	return benchmarkGroundShapeId
}

// resetGroundShapeId is ResetGroundShapeId.
func resetGroundShapeId() {
	benchmarkGroundShapeId = NullShapeID
}

// createJointGrid is CreateJointGrid.
func createJointGrid(worldId WorldID) {
	worldId.EnableSleeping(false)

	n := 100
	if benchmarkDebug {
		n = 10
	}

	// Allocate to avoid huge stack usage
	bodies := make([]BodyID, n*n)
	index := 0

	shapeDef := DefaultShapeDef()
	shapeDef.Filter.CategoryBits = 2
	// PORT: ~2u is a 32-bit unsigned int, zero extended into the uint64 mask.
	shapeDef.Filter.MaskBits = uint64(^uint32(2))

	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.4}

	jointDef := DefaultSphericalJointDef()
	bodyDef := DefaultBodyDef()
	bodyDef.EnableSleep = false

	for k := range n {
		for i := range n {
			fk := float32(k)
			fi := float32(i)

			if i == 0 {
				bodyDef.Type = StaticBody
			} else {
				bodyDef.Type = DynamicBody
			}

			bodyDef.Position = Pos{fk, -fi, 0.0}

			body := worldId.CreateBody(&bodyDef)

			body.CreateSphereShape(&shapeDef, &sphere)

			if i > 0 {
				jointDef.Base.BodyIdA = bodies[index-1]
				jointDef.Base.BodyIdB = body
				jointDef.Base.LocalFrameA.P = Vec3{0.0, -0.5, 0.0}
				jointDef.Base.LocalFrameB.P = Vec3{0.0, 0.5, 0.0}
				worldId.CreateSphericalJoint(&jointDef)
			}

			if k > 0 {
				jointDef.Base.BodyIdA = bodies[index-n]
				jointDef.Base.BodyIdB = body
				jointDef.Base.LocalFrameA.P = Vec3{0.5, 0.0, 0.0}
				jointDef.Base.LocalFrameB.P = Vec3{-0.5, 0.0, 0.0}
				worldId.CreateSphericalJoint(&jointDef)
			}

			bodies[index] = body
			index++
		}
	}
}

// createLargePyramid is CreateLargePyramid.
// The 80 block version falls over after 1000 steps.
func createLargePyramid(worldId WorldID) {
	worldId.EnableSleeping(false)

	baseCount := 100
	if benchmarkDebug {
		baseCount = 20
	}

	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, -1.0, 0.0}
		groundId := worldId.CreateBody(&bodyDef)

		box := MakeBoxHull(400.0, 1.0, 400.0)
		shapeDef := DefaultShapeDef()
		benchmarkGroundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 100.0

	h := float32(0.5)
	box := MakeBoxHull(h, h, h)

	shift := float32(1.0 * h)

	for i := range baseCount {
		y := float32((float32(2.0*float32(i)) + 1.0) * shift)

		for j := i; j < baseCount; j++ {
			x := float32((float32(i)+1.0)*shift) + float32(float32(2.0*float32(j-i))*shift) - float32(h*float32(baseCount))

			bodyDef.Position = Pos{x, y, 0.0}

			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&shapeDef, &box.Base)
		}
	}
}

// createWidePyramid is CreateWidePyramid.
func createWidePyramid(worldId WorldID) {
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, -1.0, 0.0}
		groundId := worldId.CreateBody(&bodyDef)

		box := MakeBoxHull(100.0, 1.0, 100.0)
		shapeDef := DefaultShapeDef()
		benchmarkGroundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	const boxSize float32 = 2.0
	const boxSeparation float32 = 0.5
	const halfBoxSize float32 = 0.5 * boxSize
	pyramidHeight := 15
	if benchmarkDebug {
		pyramidHeight = 5
	}

	// PORT: `halfBoxSize - 0.025` is a double expression in C.
	h := float32(float64(halfBoxSize) - 0.025)
	box := MakeBoxHull(h, h, h)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody

	shapeDef := DefaultShapeDef()

	for i := range pyramidHeight {
		for j := i / 2; j < pyramidHeight-(i+1)/2; j++ {
			for k := i / 2; k < pyramidHeight-(i+1)/2; k++ {
				odd := float32(0.0)
				if i&1 != 0 {
					odd = halfBoxSize
				}
				x := -float32(pyramidHeight) + float32(boxSize*float32(j)) + odd
				y := 1.0 + float32((boxSize+boxSeparation)*float32(i))
				z := -float32(pyramidHeight) + float32(boxSize*float32(k)) + odd
				bodyDef.Position = Pos{x, y, z}

				bodyId := worldId.CreateBody(&bodyDef)
				bodyId.CreateHullShape(&shapeDef, &box.Base)
			}
		}
	}
}

// createSmallPyramid is CreateSmallPyramid.
func createSmallPyramid(worldId WorldID, baseCount int, extent, centerX, baseZ float32) {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.EnableSleep = false

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 100.0

	box := MakeBoxHull(extent, extent, extent)

	for i := range baseCount {
		y := float32((float32(2.0*float32(i)) + 1.0) * extent)

		for j := i; j < baseCount; j++ {
			x := float32((float32(i)+1.0)*extent) + float32(float32(2.0*float32(j-i))*extent) + centerX - 0.5
			bodyDef.Position = Pos{x, y, baseZ}

			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&shapeDef, &box.Base)
		}
	}
}

// createManyPyramids is CreateManyPyramids.
func createManyPyramids(worldId WorldID) {
	baseCount := 10
	extent := float32(0.5)
	rowCount := 14
	columnCount := 14
	if benchmarkDebug {
		rowCount = 3
		columnCount = 3
	}
	groundExtent := float32(float32(extent*float32(columnCount)) * (float32(baseCount) + 1.0))

	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, -1.0, 0.0}
		groundId := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		box := MakeBoxHull(groundExtent, 1.0, groundExtent)
		benchmarkGroundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	baseWidth := float32(float32(2.0*extent) * float32(baseCount))
	baseZ := -groundExtent + float32(2.0*extent)
	deltaZ := float32(2.0*(groundExtent-float32(2.0*extent))) / (float32(rowCount) - 1.0)

	for range rowCount {
		for j := range columnCount {
			centerX := -groundExtent + float32(float32(j)*(baseWidth+float32(2.0*extent))) + float32(2.0*extent)
			createSmallPyramid(worldId, baseCount, extent, centerX, baseZ)
		}

		baseZ += deltaZ
	}
}

const (
	rainGridSize   float32 = 15.0 // RAIN_GRID_SIZE
	rainLargeWorld         = 0    // RAIN_LARGE_WORLD
	rainGroupMax           = 3    // largest RAIN_GROUP_SIZE
)

// rainGridCount is RAIN_GRID_COUNT.
func rainGridCount() int {
	if benchmarkDebug {
		return 3
	}
	return 10
}

// rainGroupSize is RAIN_GROUP_SIZE.
func rainGroupSize() int {
	if benchmarkDebug {
		return 2
	}
	return 3
}

// rainGroup is Group.
type rainGroup struct {
	humans [rainGroupMax]human
}

// rainDataT is RainData.
type rainDataT struct {
	groups      []rainGroup
	gridMesh    *MeshData
	torusMesh   *MeshData
	columnCount int
	columnIndex int
}

// rainData is g_rainData.
var rainData rainDataT

// getRainCapacity is GetRainCapacity. RAIN_LARGE_WORLD is 0, so it sets nothing.
func getRainCapacity(capacity *Capacity) {
}

// createRain is CreateRain.
func createRain(worldId WorldID) {
	rainData = rainDataT{}

	gridCount := rainGridCount()
	rainData.groups = make([]rainGroup, gridCount*gridCount)

	halfMeshGridRows := 4
	meshGridCellWidth := rainGridSize / float32(2.0*float32(halfMeshGridRows))
	rainData.gridMesh = CreateGridMesh(2*halfMeshGridRows, 2*halfMeshGridRows, meshGridCellWidth, 1, true)
	rainData.torusMesh = CreateTorusMesh(16, 16, 0.25*rainGridSize, 1.0)

	span := float32(rainGridSize * float32(gridCount))
	bodyDef := DefaultBodyDef()
	shapeDef := DefaultShapeDef()

	bodyDef.Position.X = float32(-0.5*span) + 0.5*rainGridSize
	for range gridCount {
		bodyDef.Position.Z = float32(-0.5*span) + 0.5*rainGridSize
		for range gridCount {
			body := worldId.CreateBody(&bodyDef)
			body.CreateMeshShape(&shapeDef, rainData.gridMesh, Vec3One)
			body.CreateMeshShape(&shapeDef, rainData.torusMesh, Vec3One)

			bodyDef.Position.Z += rainGridSize
		}

		bodyDef.Position.X += rainGridSize
	}
}

// destroyRain is DestroyRain.
func destroyRain() {
	DestroyMesh(rainData.gridMesh)
	DestroyMesh(rainData.torusMesh)

	rainData.groups = nil
}

// createGroup is CreateGroup.
func createGroup(worldId WorldID, rowIndex, columnIndex int) {
	gridCount := rainGridCount()
	ensure(rowIndex < gridCount && columnIndex < gridCount)

	groupIndex := rowIndex*gridCount + columnIndex

	span := float32(float32(gridCount) * rainGridSize)
	groupDistance := float32(1.0*span) / float32(gridCount)

	var position Pos
	position.X = float32(-0.5*span) + float32(groupDistance*(float32(columnIndex)+0.5))
	position.Y = 20.0
	position.Z = float32(-0.5*span) + float32(groupDistance*(float32(rowIndex)+0.5))

	frictionTorque := float32(5.0)
	hertz := float32(1.0)
	dampingRatio := float32(0.7)
	colorize := false

	for i := range rainGroupSize() {
		human := &rainData.groups[groupIndex].humans[i]
		createHuman(human, worldId, position, frictionTorque, hertz, dampingRatio, groupIndex, nil, colorize)
		position.X += 0.75
	}
}

// destroyGroup is DestroyGroup.
func destroyGroup(rowIndex, columnIndex int) {
	gridCount := rainGridCount()
	ensure(rowIndex < gridCount && columnIndex < gridCount)

	groupIndex := rowIndex*gridCount + columnIndex

	for i := range rainGroupSize() {
		destroyHuman(&rainData.groups[groupIndex].humans[i])
	}
}

// stepRain is StepRain.
func stepRain(worldId WorldID, stepCount int) {
	delay := 0x2F
	if benchmarkDebug {
		delay = 0x7F
	}
	increment := 1
	if rainLargeWorld != 0 {
		increment = 100
	}
	gridCount := rainGridCount()

	if (stepCount & delay) == 0 {
		if rainData.columnCount < gridCount {
			for i := 0; i < gridCount; i += increment {
				createGroup(worldId, i, rainData.columnCount)
			}

			rainData.columnCount = MinInt(rainData.columnCount+increment, gridCount)
		} else {
			for i := 0; i < gridCount; i += increment {
				destroyGroup(i, rainData.columnIndex)
				createGroup(worldId, i, rainData.columnIndex)
			}

			rainData.columnIndex = rainData.columnIndex + increment
			if rainData.columnIndex >= gridCount {
				rainData.columnIndex = 0
			}
		}
	}
}

// Static Floor: a huge grid of static box bodies with b3ShapeDef::invokeContactCreation = true,
// plus a small number of dynamic spheres that drop onto it staggered over time. The point of this
// benchmark is to exercise the b3BroadPhase move buffer at steady state when it was once populated
// with ~staticShapeCount entries and now sees only a handful of dynamic moves per step.
const staticFloorCellSize float32 = 10.0 // STATIC_FLOOR_CELL_SIZE

// staticFloorGrid is STATIC_FLOOR_GRID.
func staticFloorGrid() int {
	if benchmarkDebug {
		return 32
	}
	return 1000
}

// staticFloorSpheres is STATIC_FLOOR_SPHERES.
func staticFloorSpheres() int {
	if benchmarkDebug {
		return 16
	}
	return 100
}

// staticFloorDropInterval is STATIC_FLOOR_DROP_INTERVAL.
func staticFloorDropInterval() int {
	if benchmarkDebug {
		return 8
	}
	return 5
}

// staticFloorData is g_staticFloorData (StaticFloorData).
var staticFloorData struct {
	spheresDropped int
}

// getLargeWorldCapacity is GetLargeWorldCapacity.
func getLargeWorldCapacity(capacity *Capacity) {
	floorCount := int32(staticFloorGrid() * staticFloorGrid())
	capacity.StaticShapeCount = floorCount
	capacity.StaticBodyCount = floorCount
	capacity.DynamicShapeCount = int32(staticFloorSpheres())
	capacity.DynamicBodyCount = int32(staticFloorSpheres())
	capacity.ContactCount = int32(MaxInt(1024, 8*staticFloorSpheres()))
}

// createLargeWorld is CreateLargeWorld.
func createLargeWorld(worldId WorldID) {
	staticFloorData.spheresDropped = 0

	cell := staticFloorCellSize
	gridCount := staticFloorGrid()
	halfSpan := float32(float32(0.5*cell) * float32(gridCount))

	box := MakeBoxHull(0.5*cell, 0.25, 0.5*cell)

	bodyDef := DefaultBodyDef()
	shapeDef := DefaultShapeDef()

	// The trigger: every static shape gets buffered into the move set on creation.
	shapeDef.InvokeContactCreation = true

	for i := range gridCount {
		x := -halfSpan + float32((float32(i)+0.5)*cell)
		for j := range gridCount {
			z := -halfSpan + float32((float32(j)+0.5)*cell)
			bodyDef.Position = Pos{x, 0.0, z}
			body := worldId.CreateBody(&bodyDef)
			body.CreateHullShape(&shapeDef, &box.Base)
		}
	}
}

// stepLargeWorld is StepLargeWorld.
func stepLargeWorld(worldId WorldID, stepCount int) {
	if staticFloorData.spheresDropped >= staticFloorSpheres() {
		return
	}

	if stepCount == 0 {
		return
	}

	if (stepCount % staticFloorDropInterval()) != 0 {
		return
	}

	// Spread spheres in a coarse grid across the floor so they don't all pile on one box.
	side := 1
	for side*side < staticFloorSpheres() {
		side += 1
	}

	idx := staticFloorData.spheresDropped
	gi := idx % side
	gj := idx / side

	halfSpan := float32(float32(0.5*staticFloorCellSize) * float32(staticFloorGrid()))
	// Confine drops to the inner 80% of the floor so spheres can't roll off the edge.
	inset := float32(float32(0.1*2.0) * halfSpan)
	usable := float32(2.0*halfSpan) - float32(2.0*inset)
	x := -halfSpan + inset + float32((float32(gi)+0.5)*(usable/float32(side)))
	z := -halfSpan + inset + float32((float32(gj)+0.5)*(usable/float32(side)))

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{x, 1.5, z}

	shapeDef := DefaultShapeDef()
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}

	body := worldId.CreateBody(&bodyDef)
	body.CreateSphereShape(&shapeDef, &sphere)

	staticFloorData.spheresDropped += 1
}

// getWasherCapacity is GetWasherCapacity.
func getWasherCapacity(capacity *Capacity) {
	capacity.StaticShapeCount = 16
	capacity.DynamicShapeCount = 10000
	capacity.StaticBodyCount = 16
	capacity.DynamicBodyCount = 10000
	capacity.ContactCount = 60000
}

// createWasher is CreateWasher.
func createWasher(worldId WorldID) {
	kinematic := true

	var groundId BodyID
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position.Y = -1.0
		groundId = worldId.CreateBody(&bodyDef)

		box := MakeBoxHull(60.0, 1.0, 60.0)
		shapeDef := DefaultShapeDef()
		benchmarkGroundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	{
		motorSpeed := float32(25.0)

		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, 21.0, 0.0}

		if kinematic == true {
			bodyDef.Type = KinematicBody
			bodyDef.AngularVelocity = Vec3{0.0, 0.0, float32(Pi/180.0) * motorSpeed}
			bodyDef.LinearVelocity = Vec3{0.001, -0.002, 0.0}
		} else {
			bodyDef.Type = DynamicBody
		}

		bodyId := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()

		r0 := float32(14.0)
		r1 := float32(16.0)
		r2 := float32(18.0)
		nd := Vec3{0.0, 0.0, -10.0}
		pd := Vec3{0.0, 0.0, 10.0}

		angle := Pi / 18.0
		q := MakeQuatFromAxisAngle(Vec3AxisZ, angle)
		tenth := float32(0.1)
		qo := MakeQuatFromAxisAngle(Vec3AxisZ, tenth*angle)
		u1 := Vec3{1.0, 0.0, 0.0}
		for i := range 36 {
			var u2 Vec3
			if i == 35 {
				u2 = Vec3{1.0, 0.0, 0.0}
			} else {
				u2 = RotateVector(q, u1)
			}

			{
				a1 := InvRotateVector(qo, u1)
				a2 := RotateVector(qo, u2)
				p1 := MulAdd(nd, r1, a1)
				p2 := MulAdd(nd, r2, a1)
				p3 := MulAdd(nd, r1, a2)
				p4 := MulAdd(nd, r2, a2)
				p5 := MulAdd(pd, r1, a1)
				p6 := MulAdd(pd, r2, a1)
				p7 := MulAdd(pd, r1, a2)
				p8 := MulAdd(pd, r2, a2)

				points := []Vec3{p1, p2, p3, p4, p5, p6, p7, p8}
				hull := CreateHull(points, 8, 8)
				bodyId.CreateHullShape(&shapeDef, hull)
				DestroyHull(hull)
			}

			if i%9 == 0 {
				p1 := MulAdd(nd, r0, u1)
				p2 := MulAdd(nd, r1, u1)
				p3 := MulAdd(nd, r0, u2)
				p4 := MulAdd(nd, r1, u2)
				p5 := MulAdd(pd, r0, u1)
				p6 := MulAdd(pd, r1, u1)
				p7 := MulAdd(pd, r0, u2)
				p8 := MulAdd(pd, r1, u2)

				points := []Vec3{p1, p2, p3, p4, p5, p6, p7, p8}
				hull := CreateHull(points, 8, 8)
				bodyId.CreateHullShape(&shapeDef, hull)
				DestroyHull(hull)
			}

			u1 = u2
		}

		if kinematic == false {
			jointDef := DefaultRevoluteJointDef()
			jointDef.Base.BodyIdA = groundId
			jointDef.Base.BodyIdB = bodyId
			jointDef.Base.LocalFrameA.P.Y = 10.0
			jointDef.MotorSpeed = float32(Pi/180.0) * motorSpeed
			jointDef.MaxMotorTorque = 1e8
			jointDef.EnableMotor = true

			worldId.CreateRevoluteJoint(&jointDef)
		}
	}

	gridCount := 20
	if benchmarkDebug {
		gridCount = 8
	}
	a := float32(0.2)

	cube := MakeBoxHull(a, a, a)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	shapeDef := DefaultShapeDef()

	x := float32(float32(-2.0*a) * float32(gridCount))
	for range gridCount {
		y := float32(float32(-2.0*a)*float32(gridCount)) + 21.0
		for range gridCount {
			z := float32(float32(-2.0*a) * float32(gridCount))
			for range gridCount {
				bodyDef.Position = Pos{x, y, z}
				bodyId := worldId.CreateBody(&bodyDef)

				bodyId.CreateHullShape(&shapeDef, &cube.Base)
				z += float32(4.0 * a)
			}

			y += float32(4.0 * a)
		}

		x += float32(4.0 * a)
	}
}

// treeData is g_treeData.
var treeData struct {
	meshData *MeshData
}

// createTrees is CreateTrees.
func createTrees(worldId WorldID, scale int) {
	treeData.meshData = nil

	// float tilt = 0.15f * B3_PI;
	tilt := float32(0.0 * Pi)
	bodyDef := DefaultBodyDef()
	bodyDef.Position = Pos{0.0, 0.0, 0.0}
	bodyDef.Rotation = MakeQuatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, tilt)
	groundId := worldId.CreateBody(&bodyDef)

	xCount := scale * 150
	// int zCount = BENCHMARK_DEBUG ? 100 : 400;
	zCount := scale * 200

	cellWidth := 1.0 / float32(scale)
	amplitude := float32(0.4)
	rowHz := float32(0.05)
	columnHz := float32(0.1)

	treeData.meshData = CreateWaveMesh(xCount, zCount, cellWidth, amplitude, rowHz, columnHz)
	shapeDef := DefaultShapeDef()
	groundId.CreateMeshShape(&shapeDef, treeData.meshData, Vec3One)

	bodyDef.Type = DynamicBody
	bodyDef.SleepThreshold = 0.2
	bodyDef.Rotation = QuatIdentity

	bodyCount := 50
	if benchmarkDebug {
		bodyCount = 10
	}

	shapeDef.BaseMaterial.Friction = 0.9
	shapeDef.BaseMaterial.RollingResistance = 0.05
	shapeDef.UpdateBodyMass = false
	shapeDef.Density = 1.0

	const hullCount = 22
	var hulls [22]*HullData

	y := float32(1.0)
	r := float32(0.75)
	l := float32(1.5)
	for i := range hullCount {
		hulls[i] = CreateCylinder(l+float32(2.0*r), r, y-r, 6)
		y += l + float32(2.0*r)
		r = float32(0.95 * r)
	}

	angularVelocity := float32(-0.5)
	z := float32(-70.0)
	if benchmarkDebug {
		z = -15.0
	}
	cs := ComputeCosSin(tilt)
	yTilt := cs.Sine / cs.Cosine
	for bodyIndex := range bodyCount {
		bodyDef.Position = Pos{0.0, 1.0 - float32(z*yTilt), z}
		bodyId := worldId.CreateBody(&bodyDef)

		for shapeIndex := range 22 {
			// Random rotation provided no benefit to behavior.
			bodyId.CreateHullShape(&shapeDef, hulls[shapeIndex])
		}

		velocityScale := 0.5 + float32(0.5*float32(bodyIndex))/float32(bodyCount)
		bodyId.ApplyMassFromShapes()
		center := bodyId.GetWorldCenter()
		omega := Vec3{0.0, 0.0, float32(velocityScale * angularVelocity)}
		v := Cross(omega, SubPos(center, bodyDef.Position))
		bodyId.SetAngularVelocity(omega)
		bodyId.SetLinearVelocity(v)

		z += 3.0
		angularVelocity = -angularVelocity
	}

	for i := range hullCount {
		DestroyHull(hulls[i])
	}
}

// createTrees25 is CreateTrees25.
func createTrees25(worldId WorldID) {
	createTrees(worldId, 4)
}

// createTrees50 is CreateTrees50.
func createTrees50(worldId WorldID) {
	createTrees(worldId, 2)
}

// createTrees100 is CreateTrees100.
func createTrees100(worldId WorldID) {
	createTrees(worldId, 1)
}

// destroyTrees is DestroyTrees.
func destroyTrees() {
	DestroyMesh(treeData.meshData)
	treeData.meshData = nil
}

// meshDropBenchData is g_meshDropData.
var meshDropBenchData struct {
	meshData *MeshData
}

// createMeshDropBenchmark is CreateMeshDropBenchmark.
func createMeshDropBenchmark(worldId WorldID) {
	meshDropBenchData.meshData = nil

	meshGridCount := 120
	borderCount := 10
	cellWidth := float32(1.0)

	{
		bodyDef := DefaultBodyDef()
		groundId := worldId.CreateBody(&bodyDef)

		rowHz := float32(0.1)
		columnHz := float32(0.2)
		groundAmplitude := float32(0.5)

		meshDropBenchData.meshData = CreateWaveMesh(meshGridCount, meshGridCount, cellWidth, groundAmplitude, rowHz, columnHz)
		shapeDef := DefaultShapeDef()
		shapeDef.Filter.CategoryBits = 1
		groundId.CreateMeshShape(&shapeDef, meshDropBenchData.meshData, Vec3One)
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

		gridCount := 100
		if benchmarkDebug {
			gridCount = 20
		}
		spacing := float32(cellWidth*float32(meshGridCount-2*borderCount)) / float32(gridCount)
		offset := float32(float32(0.5*spacing) * float32(1-gridCount))

		for i := range gridCount {
			for j := range gridCount {
				linearVelocity := randomVec3Uniform(-1.0, 1.0)
				angularVelocity := randomVec3Uniform(-5.0, 5.0)

				bodyDef.Position = Pos{offset + float32(spacing*float32(i)), 5.0, offset + float32(spacing*float32(j))}
				bodyDef.LinearVelocity = linearVelocity
				bodyDef.AngularVelocity = angularVelocity
				bodyId := worldId.CreateBody(&bodyDef)

				bodyId.CreateHullShape(&shapeDef, &box.Base)
			}
		}
	}
}

// destroyMeshDropBenchmark is DestroyMeshDropBenchmark.
func destroyMeshDropBenchmark() {
	DestroyMesh(meshDropBenchData.meshData)
	meshDropBenchData.meshData = nil
}

// junkyardData is struct JunkyardData g_junkyardData.
var junkyardData struct {
	pusherId BodyID
	degrees  float32
	radius   float32
}

// createJunkyard is CreateJunkyard.
func createJunkyard(worldId WorldID) {
	var groundId BodyID
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position.Y = -1.0
		groundId = worldId.CreateBody(&bodyDef)
	}

	{
		shapeDef := DefaultShapeDef()
		{
			box := MakeBoxHull(120.0, 1.0, 120.0)
			benchmarkGroundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
		}
		{
			offset := Vec3{-50.0, 8.0, 0.0}
			box := MakeOffsetBoxHull(1.0, 8.0, 50.0, offset)
			groundId.CreateHullShape(&shapeDef, &box.Base)
		}
		{
			offset := Vec3{50.0, 8.0, 0.0}
			box := MakeOffsetBoxHull(1.0, 8.0, 50.0, offset)
			groundId.CreateHullShape(&shapeDef, &box.Base)
		}
		{
			offset := Vec3{0.0, 8.0, -50.0}
			box := MakeOffsetBoxHull(50.0, 8.0, 1.0, offset)
			groundId.CreateHullShape(&shapeDef, &box.Base)
		}
		{
			offset := Vec3{0.0, 8.0, 50.0}
			box := MakeOffsetBoxHull(50.0, 8.0, 1.0, offset)
			groundId.CreateHullShape(&shapeDef, &box.Base)
		}
	}
	{
		rockHull := CreateRock(1.5)

		count := 24
		if benchmarkDebug {
			count = 2
		}
		height := float32(24.0)
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		shapeDef := DefaultShapeDef()
		for Y := range count {
			for X := 0; X <= 20; X++ {
				for Z := 0; Z <= 20; Z++ {
					bodyDef.Position.X = -40.0 + float32(4.0*float32(X))
					bodyDef.Position.Y = float32(4.0*float32(Y)) + height + 1.0
					bodyDef.Position.Z = -40.0 + float32(4.0*float32(Z))
					bodyId := worldId.CreateBody(&bodyDef)
					bodyId.CreateHullShape(&shapeDef, rockHull)
				}
			}
		}

		DestroyHull(rockHull)
	}

	junkyardData.radius = 35.0
	mHeight := float32(24.0)

	hull := CreateCylinder(mHeight, 4.0, 0.0, 16)
	bodyDef := DefaultBodyDef()
	bodyDef.Type = KinematicBody
	bodyDef.Position = Pos{junkyardData.radius, 0.0, 0.0}
	junkyardData.pusherId = worldId.CreateBody(&bodyDef)
	junkyardData.degrees = 0.0
	shapeDef := DefaultShapeDef()
	junkyardData.pusherId.CreateHullShape(&shapeDef, hull)
	DestroyHull(hull)
}

// getJunkyardCapacity is GetJunkyardCapacity.
func getJunkyardCapacity(capacity *Capacity) {
	if !benchmarkDebug {
		capacity.StaticShapeCount = 16
		capacity.DynamicShapeCount = 20*20*24 + 1
		capacity.StaticBodyCount = 16
		capacity.DynamicBodyCount = 20*20*24 + 1
		capacity.ContactCount = 250 * 1024
	} else {
		capacity.StaticShapeCount = 16
		capacity.DynamicShapeCount = 20*20*2 + 1
		capacity.StaticBodyCount = 16
		capacity.DynamicBodyCount = 20*20*2 + 1
	}
}

// stepJunkyard is StepJunkyard.
func stepJunkyard(worldId WorldID, stepCount int) {
	timeStep := float32(1.0 / 60.0)
	const omega float32 = -6.0
	junkyardData.degrees += float32(omega * timeStep)
	cs := ComputeCosSin(junkyardData.degrees * Pi / 180.0)
	r := junkyardData.radius
	targetPos := Pos{r * cs.Cosine, 0.0, r * cs.Sine}
	target := WorldTransform{P: targetPos, Q: QuatIdentity}
	junkyardData.pusherId.SetTargetTransform(target, timeStep, false)
}

// Huge pile of large convexes, ported from PEEL. Each convex is the hull of 32 random points on a
// sphere. A fixed LCG seed makes the hull identical across runs so results compare directly.

// getConvexPileCapacity is GetConvexPileCapacity.
func getConvexPileCapacity(capacity *Capacity) {
	if !benchmarkDebug {
		capacity.DynamicShapeCount = 5120
		capacity.DynamicBodyCount = 5120
		capacity.ContactCount = 64 * 1024
	}
}

// createConvexPile is CreateConvexPile.
func createConvexPile(worldId WorldID) {
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, -1.0, 0.0}
		groundId := worldId.CreateBody(&bodyDef)

		box := MakeBoxHull(250.0, 1.0, 250.0)
		shapeDef := DefaultShapeDef()
		benchmarkGroundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	countX := 8
	countZ := 8
	layers := 80
	if benchmarkDebug {
		layers = 10
	}
	amplitude := float32(2.0)
	scatter := float32(2.0 * amplitude)

	convex := CreateComplexHull(amplitude)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	shapeDef := DefaultShapeDef()

	// Grid tall enough to collapse into a pile
	for layer := range layers {
		for z := range countZ {
			for x := range countX {
				posX := float32((float32(x) - float32(0.5*float32(countX))) * scatter)
				posZ := float32((float32(z) - float32(0.5*float32(countZ))) * scatter)
				posY := amplitude + float32(float32(2.0*amplitude)*float32(layer))

				bodyDef.Position = Pos{posX, posY, posZ}
				bodyId := worldId.CreateBody(&bodyDef)
				bodyId.CreateHullShape(&shapeDef, convex)
			}
		}
	}

	DestroyHull(convex)
}

const spinnerSegmentCount = 90 // SPINNER_SEGMENT_COUNT

// spinnerDataT is SpinnerData.
type spinnerDataT struct {
	cylinderMesh *MeshData
	spinnerId    JointID
}

// spinnerData is g_spinnerData.
var spinnerData spinnerDataT

// createInwardCylinderMesh is CreateInwardCylinderMesh.
// An inward facing cylinder capped on both ends, axis along z. Every triangle is wound so its
// normal points at the axis, which is what holds the contents in. The caps do the same job along
// the axis that the 2D loop had for free.
func createInwardCylinderMesh(radius, halfDepth, yCenter float32, segmentCount int) *MeshData {
	vertexCount := 2*segmentCount + 2
	triangleCount := 4 * segmentCount

	vertices := make([]Vec3, vertexCount)
	indices := make([]int32, 3*triangleCount)

	twoPi := float32(2.0 * Pi)
	for i := range segmentCount {
		angle := float32(twoPi*float32(i)) / float32(segmentCount)
		cs := ComputeCosSin(angle)
		x := float32(radius * cs.Cosine)
		y := float32(radius*cs.Sine) + yCenter
		vertices[2*i+0] = Vec3{x, y, -halfDepth}
		vertices[2*i+1] = Vec3{x, y, halfDepth}
	}

	centerNegative := int32(2 * segmentCount)
	centerPositive := int32(2*segmentCount + 1)
	vertices[centerNegative] = Vec3{0.0, yCenter, -halfDepth}
	vertices[centerPositive] = Vec3{0.0, yCenter, halfDepth}

	index := 0
	push := func(v int32) {
		indices[index] = v
		index++
	}
	for i := range segmentCount {
		j := (i + 1) % segmentCount
		i2 := int32(2 * i)
		j2 := int32(2 * j)

		// Wall, normal towards the axis
		push(i2 + 0)
		push(i2 + 1)
		push(j2 + 1)

		push(i2 + 0)
		push(j2 + 1)
		push(j2 + 0)

		// Near cap, normal towards positive z
		push(centerNegative)
		push(i2 + 0)
		push(j2 + 0)

		// Far cap, normal towards negative z
		push(centerPositive)
		push(j2 + 1)
		push(i2 + 1)
	}

	ensure(index == 3*triangleCount)

	meshDef := MeshDef{}
	meshDef.Vertices = vertices
	meshDef.VertexCount = int32(vertexCount)
	meshDef.Indices = indices
	meshDef.TriangleCount = int32(triangleCount)
	meshDef.IdentifyEdges = true

	return CreateMesh(&meshDef, nil)
}

// getSpinnerCapacity is GetSpinnerCapacity.
func getSpinnerCapacity(capacity *Capacity) {
	capacity.StaticShapeCount = 4
	capacity.StaticBodyCount = 4
	capacity.DynamicShapeCount = 2000
	capacity.DynamicBodyCount = 2000
	capacity.ContactCount = 20000
}

// createSpinner is CreateSpinner.
// A paddle on a motor churning a pile of small shapes inside a closed cylinder. The cylinder axis
// is z so the cross section matches the 2D benchmark and the hinge, which turns about the local
// frame z axis, lines up with it. The paddle circle is internally tangent to the cylinder at the
// bottom, so the tip scrapes the floor.
func createSpinner(worldId WorldID) {
	spinnerData = spinnerDataT{}

	radius := float32(12.0)
	halfDepth := float32(1.0)
	yCenter := radius

	var groundId BodyID
	{
		bodyDef := DefaultBodyDef()
		groundId = worldId.CreateBody(&bodyDef)

		spinnerData.cylinderMesh = createInwardCylinderMesh(radius, halfDepth, yCenter, spinnerSegmentCount)

		shapeDef := DefaultShapeDef()
		shapeDef.BaseMaterial.Friction = 0.1
		groundId.CreateMeshShape(&shapeDef, spinnerData.cylinderMesh, Vec3One)
	}

	{
		halfLength := float32(0.5 * radius)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, yCenter - halfLength, 0.0}
		bodyDef.EnableSleep = false

		spinnerId := worldId.CreateBody(&bodyDef)

		paddle := MakeBoxHull(0.3, halfLength+0.1, halfDepth+0.1)
		shapeDef := DefaultShapeDef()
		shapeDef.BaseMaterial.Friction = 0.0
		spinnerId.CreateHullShape(&shapeDef, &paddle.Base)

		jointDef := DefaultRevoluteJointDef()
		jointDef.Base.BodyIdA = groundId
		jointDef.Base.BodyIdB = spinnerId
		jointDef.Base.LocalFrameA.P = Vec3{0.0, yCenter - halfLength, 0.0}
		jointDef.EnableMotor = true
		jointDef.MotorSpeed = 8.0
		jointDef.MaxMotorTorque = math.MaxFloat32

		spinnerData.spinnerId = worldId.CreateRevoluteJoint(&jointDef)
	}

	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.2}

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody

	shapeDef := DefaultShapeDef()
	shapeDef.BaseMaterial.Friction = 0.1
	shapeDef.BaseMaterial.Restitution = 0.1
	shapeDef.Density = 0.25

	bodyCount := 1500
	if benchmarkDebug {
		bodyCount = 500
	}

	// The grid starts low where the cylinder is narrow, so the span has to fit the chord there
	spacing := float32(0.55)
	yStart := float32(2.0)
	dy := yCenter - yStart
	chordHalfWidth := sqrtf(float32(radius*radius) - float32(dy*dy))
	xLimit := chordHalfWidth - float32(2.0*spacing)
	zLimit := halfDepth - float32(2.0*sphere.Radius)

	x := -xLimit
	y := yStart
	z := -zLimit

	for range bodyCount {
		bodyDef.Position = Pos{x, y, z}
		bodyId := worldId.CreateBody(&bodyDef)

		bodyId.CreateSphereShape(&shapeDef, &sphere)

		z += spacing

		if z > zLimit {
			z = -zLimit
			x += spacing

			if x > xLimit {
				x = -xLimit
				y += spacing
			}
		}
	}
}

// destroySpinner is DestroySpinner.
func destroySpinner() {
	if spinnerData.cylinderMesh != nil {
		DestroyMesh(spinnerData.cylinderMesh)
		spinnerData.cylinderMesh = nil
	}
}

// getSpinnerAngle is GetSpinnerAngle.
// The motor is unbounded, so a healthy run holds the commanded speed and the angle advances
// steadily. A stalling solver shows up here before it shows up in the step time.
func getSpinnerAngle() float32 {
	if spinnerData.spinnerId.IsValid() == false {
		return 0.0
	}

	return spinnerData.spinnerId.RevoluteJointGetAngle()
}

const sleepPyramidCount = 10 // SLEEP_PYRAMID_COUNT

// sleepDataT is SleepData.
type sleepDataT struct {
	bodyIdA [sleepPyramidCount]BodyID
	bodyIdB [sleepPyramidCount]BodyID
}

// sleepData is g_sleepData.
var sleepData sleepDataT

// createSleepPyramid is CreateSleepPyramid.
// Reports two bodies that are certain to share a touching contact, so the filter joint below
// always lands inside one island.
func createSleepPyramid(worldId WorldID, baseCount int, extent, centerX float32, bodyIdA, bodyIdB *BodyID) {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.SleepThreshold = 1.0

	shapeDef := DefaultShapeDef()
	box := MakeBoxHull(extent, extent, extent)

	for i := range baseCount {
		y := float32((float32(2.0*float32(i)) + 1.0) * extent)

		for j := i; j < baseCount; j++ {
			x := float32((float32(i)+1.0)*extent) + float32(float32(2.0*float32(j-i))*extent) + centerX
			bodyDef.Position = Pos{x, y, 0.0}

			bodyId := worldId.CreateBody(&bodyDef)
			bodyId.CreateHullShape(&shapeDef, &box.Base)

			if i == 0 && j == 0 {
				*bodyIdA = bodyId
			} else if i == 1 && j == 1 {
				// Rests on the base body above, so the pair always has a touching contact
				*bodyIdB = bodyId
			}
		}
	}
}

// getSleepCapacity is GetSleepCapacity.
func getSleepCapacity(capacity *Capacity) {
	capacity.StaticShapeCount = 4
	capacity.StaticBodyCount = 4
	capacity.DynamicShapeCount = 2200
	capacity.DynamicBodyCount = 2200
	capacity.ContactCount = 12000
}

// createSleep is CreateSleep.
// Stress tests waking and sleeping. Each pyramid settles into its own island and the step below
// wakes one of them again, so the island splitter never gets to rest.
func createSleep(worldId WorldID) {
	sleepData = sleepDataT{}

	baseCount := 20
	if benchmarkDebug {
		baseCount = 8
	}
	extent := float32(0.5)

	baseWidth := float32(float32(2.0*extent) * float32(baseCount))
	pitch := baseWidth + float32(8.0*extent)
	span := float32(pitch * float32(sleepPyramidCount-1))

	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, -1.0, 0.0}
		groundId := worldId.CreateBody(&bodyDef)

		box := MakeBoxHull(float32(0.5*span)+baseWidth, 1.0, 4.0)
		shapeDef := DefaultShapeDef()
		benchmarkGroundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	for i := range sleepPyramidCount {
		centerX := float32(-0.5*span) + float32(float32(i)*pitch) - float32(0.5*baseWidth)
		createSleepPyramid(worldId, baseCount, extent, centerX, &sleepData.bodyIdA[i], &sleepData.bodyIdB[i])
	}
}

// stepSleep is StepSleep.
func stepSleep(worldId WorldID, stepCount int) {
	for i := range sleepPyramidCount {
		if sleepData.bodyIdA[i].IsAwake() == false {
			// Creating and destroying a joint engages the island splitter
			jointDef := DefaultFilterJointDef()
			jointDef.Base.BodyIdA = sleepData.bodyIdA[i]
			jointDef.Base.BodyIdB = sleepData.bodyIdB[i]
			jointId := worldId.CreateFilterJoint(&jointDef)

			// This wakes the island
			jointId.Destroy(true)

			// Only one per step
			break
		}
	}
}

const (
	villageTileWidth  = 4    // VILLAGE_TILE_WIDTH
	villageCastRadius = 0.05 // VILLAGE_CAST_RADIUS
	villageCastHeight = 10.0 // VILLAGE_CAST_HEIGHT
	villageCastLength = 12.0 // VILLAGE_CAST_LENGTH
	villageStepCount  = 600  // VILLAGE_STEP_COUNT
	villageMaxGrid    = 100  // largest VILLAGE_GRID
)

// villageGrid is VILLAGE_GRID.
func villageGrid() int {
	if benchmarkDebug {
		return 10
	}
	return 100
}

// villageBodyGrid is VILLAGE_BODY_GRID.
func villageBodyGrid() int {
	if benchmarkDebug {
		return 10
	}
	return 70
}

// villageMaxCasts is VILLAGE_MAX_CASTS: one cast per meter across the full span.
const villageMaxCasts = villageMaxGrid*villageTileWidth + 1

// villageCast is VillageCast.
type villageCast struct {
	origin   Pos
	point    Pos
	normal   Vec3
	fraction float32
	hit      bool
}

// villageCurtain is VillageCurtain: the sphere casts from the latest village step, for drawing.
type villageCurtain struct {
	casts       []villageCast
	castCount   int
	translation Vec3
	radius      float32
}

// villageDataT is VillageData.
type villageDataT struct {
	compound        *CompoundData
	rotation        Quat
	halfSpan        float32
	castTranslation Vec3
	castCount       int
	casts           [villageMaxCasts]villageCast
}

// villageData is g_villageData.
var villageData villageDataT

// getVillageCapacity is GetVillageCapacity.
func getVillageCapacity(capacity *Capacity) {
	bodyCount := int32(villageBodyGrid() * villageBodyGrid())
	capacity.StaticShapeCount = 1
	capacity.StaticBodyCount = 1
	capacity.DynamicShapeCount = bodyCount
	capacity.DynamicBodyCount = bodyCount
	capacity.ContactCount = 4 * bodyCount
}

// createVillage is CreateVillage.
func createVillage(worldId WorldID) {
	villageData = villageDataT{}
	gRandomSeed = 2718281828

	gridCount := villageGrid()
	tileWidth := float32(villageTileWidth)
	halfSpan := float32(float32(0.5*tileWidth) * float32(gridCount))
	villageData.halfSpan = halfSpan

	{
		tileCount := gridCount * gridCount
		propCapacity := tileCount/8 + 1

		hulls := make([]CompoundHullDef, tileCount)
		meshes := make([]CompoundMeshDef, tileCount)
		capsules := make([]CompoundCapsuleDef, propCapacity)
		spheres := make([]CompoundSphereDef, propCapacity)

		material := DefaultSurfaceMaterial()
		tile := MakeBoxHull(0.5*tileWidth, 0.5, 0.5*tileWidth)

		// A wave patch replaces a sparse set of tiles so the compound carries mesh children
		wave := CreateWaveMesh(8, 8, tileWidth/8.0, 0.25, 0.25, 0.25)

		hullCount := 0
		meshCount := 0
		capsuleCount := 0
		sphereCount := 0

		for i := range gridCount {
			x := -halfSpan + float32((float32(i)+0.5)*tileWidth)

			for j := range gridCount {
				z := -halfSpan + float32((float32(j)+0.5)*tileWidth)

				if i%16 == 8 && j%16 == 8 {
					meshes[meshCount] = CompoundMeshDef{
						MeshData:      wave,
						Transform:     Transform{Vec3{x, 0.0, z}, QuatIdentity},
						Scale:         Vec3One,
						Materials:     unsafe.Slice(&material, 1),
						MaterialCount: 1,
					}
					meshCount += 1
					continue
				}

				hulls[hullCount] = CompoundHullDef{
					Hull:      &tile.Base,
					Transform: Transform{Vec3{x, -0.5, z}, QuatIdentity},
					Material:  material,
				}
				hullCount += 1

				tileIndex := i*gridCount + j
				if tileIndex%8 != 0 {
					continue
				}

				if (tileIndex/8)&1 != 0 {
					radius := float32(0.25)
					capsules[capsuleCount] = CompoundCapsuleDef{
						Capsule:  Capsule{Vec3{x - 1.0, radius, z}, Vec3{x + 1.0, radius, z}, radius},
						Material: material,
					}
					capsuleCount += 1
				} else {
					radius := float32(0.5)
					spheres[sphereCount] = CompoundSphereDef{
						Sphere:   Sphere{Vec3{x, radius, z}, radius},
						Material: material,
					}
					sphereCount += 1
				}
			}
		}

		def := CompoundDef{
			Capsules:     capsules,
			CapsuleCount: int32(capsuleCount),
			Hulls:        hulls,
			HullCount:    int32(hullCount),
			Meshes:       meshes,
			MeshCount:    int32(meshCount),
			Spheres:      spheres,
			SphereCount:  int32(sphereCount),
		}

		villageData.compound = CreateCompound(&def)

		// The compound clones its inputs
		DestroyMesh(wave)

		// Rotated so broad-phase lookups transform into the compound frame
		turn := float32(-1.15)
		villageData.rotation = MakeQuatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, turn*Pi)

		bodyDef := DefaultBodyDef()
		bodyDef.Rotation = villageData.rotation
		groundId := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		groundId.CreateBakedCompoundShape(&shapeDef, villageData.compound)
	}

	{
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.3}
		capsule := Capsule{Vec3{-0.3, 0.0, 0.0}, Vec3{0.3, 0.0, 0.0}, 0.2}
		box := MakeCubeHull(0.3)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		shapeDef := DefaultShapeDef()

		bodyGrid := villageBodyGrid()
		spacing := float32(1.6)
		offset := float32(float32(0.5*spacing) * float32(1-bodyGrid))

		for i := range bodyGrid {
			for j := range bodyGrid {
				index := i*bodyGrid + j

				bodyDef.Position = Pos{offset + float32(spacing*float32(i)), randomFloatRange(2.0, 6.0), offset + float32(spacing*float32(j))}
				bodyDef.LinearVelocity = Vec3Zero
				bodyDef.AngularVelocity = randomVec3Uniform(-2.0, 2.0)

				// Fast enough to need continuous collision against the compound
				if index%20 == 0 {
					bodyDef.Position.Y = 20.0
					bodyDef.LinearVelocity = Vec3{0.0, -80.0, 0.0}
				}

				bodyId := worldId.CreateBody(&bodyDef)

				switch index % 3 {
				case 0:
					bodyId.CreateSphereShape(&shapeDef, &sphere)
				case 1:
					bodyId.CreateCapsuleShape(&shapeDef, &capsule)
				default:
					bodyId.CreateHullShape(&shapeDef, &box.Base)
				}
			}
		}
	}
}

// villageCastCallback is VillageCastCallback.
func villageCastCallback(shapeId ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64,
	triangleIndex int, childIndex int, context any) float32 {
	// Clipping makes each report closer than the last, so the final one is the closest
	cast := context.(*villageCast)
	cast.point = point
	cast.normal = normal
	cast.fraction = fraction
	cast.hit = true
	return fraction
}

// stepVillage is StepVillage.
// A curtain of tilted sphere casts, one per meter, sweeps across the compound once per run.
func stepVillage(worldId WorldID, stepIndex int) {
	halfSpan := villageData.halfSpan
	t := float32(stepIndex%villageStepCount) / float32(villageStepCount-1)
	x := -halfSpan + float32(float32(2.0*halfSpan)*t)

	tiltFraction := float32(20.0 / 180.0)
	tilt := ComputeCosSin(tiltFraction * Pi)
	length := float32(villageCastLength)
	localTranslation := Vec3{float32(length * tilt.Sine), float32(-length * tilt.Cosine), 0.0}

	// The curtain is laid out in the compound frame
	q := villageData.rotation
	translation := RotateVector(q, localTranslation)

	point := Vec3Zero
	proxy := ShapeProxy{unsafe.Slice(&point, 1), 1, villageCastRadius}
	filter := DefaultQueryFilter()

	castCount := int(float32(2.0*halfSpan)) + 1
	ensure(castCount <= villageMaxCasts)

	for i := range castCount {
		localOrigin := Vec3{x, villageCastHeight, -halfSpan + float32(i)}

		cast := &villageData.casts[i]
		*cast = villageCast{}
		cast.origin = ToPos(RotateVector(q, localOrigin))
		cast.fraction = 1.0

		worldId.CastShape(cast.origin, &proxy, translation, filter, villageCastCallback, cast)
	}

	villageData.castTranslation = translation
	villageData.castCount = castCount
}

// getVillageCurtain is GetVillageCurtain.
func getVillageCurtain() villageCurtain {
	return villageCurtain{
		casts:       villageData.casts[:],
		castCount:   villageData.castCount,
		translation: villageData.castTranslation,
		radius:      villageCastRadius,
	}
}

// destroyVillage is DestroyVillage.
func destroyVillage() {
	DestroyCompound(villageData.compound)
	villageData = villageDataT{}
}

// ---- overflow_color.h ----

// overflowColorPileData is OverflowColorPileData.
// One heavy dynamic hub surrounded by enough dynamic neighbors that the hub's
// degree in the dyn-dyn contact graph exceeds B3_DYNAMIC_COLOR_COUNT (= 20).
// The excess contacts land in the overflow color (B3_GRAPH_COLOR_COUNT - 1),
// exercising the b3*_Overflow solver path that no other scene reaches.
type overflowColorPileData struct {
	groundShapeId ShapeID
	hubId         BodyID
	neighborCount int
}

// ---- overflow_color.c ----

const (
	overflowPileRingCount = 5 // OVERFLOW_PILE_RING_COUNT
	overflowPilePerRing   = 5 // OVERFLOW_PILE_PER_RING
)

// createOverflowColorPile is CreateOverflowColorPile.
// PORT: cosf / sinf are float64 math.Cos / math.Sin rounded to float32.
func createOverflowColorPile(worldId WorldID) overflowColorPileData {
	data := overflowColorPileData{}
	data.neighborCount = overflowPileRingCount * overflowPilePerRing

	// Static ground (top surface at y = 0)
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{0.0, -1.0, 0.0}
		groundId := worldId.CreateBody(&bodyDef)

		box := MakeBoxHull(20.0, 1.0, 20.0)
		shapeDef := DefaultShapeDef()
		data.groundShapeId = groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	// Tall, heavy hub. Tall so neighbors can ring around it in multiple
	// vertical layers; heavy so it stays roughly still under uneven inward
	// pressure from the ring.
	hubHalfX := float32(0.5)
	hubHalfY := float32(2.5)
	hubHalfZ := float32(0.5)
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, hubHalfY, 0.0}
		data.hubId = worldId.CreateBody(&bodyDef)

		box := MakeBoxHull(hubHalfX, hubHalfY, hubHalfZ)
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 50.0
		data.hubId.CreateHullShape(&shapeDef, &box.Base)
	}

	// Neighbors: vertical rings around the hub, each box slightly overlapping
	// the hub so a contact exists on the very first step.
	neighborHalf := float32(0.2)
	ringRadius := hubHalfX + neighborHalf - 0.03

	neighborBox := MakeBoxHull(neighborHalf, neighborHalf, neighborHalf)
	neighborShape := DefaultShapeDef()

	ringSpacing := float32(0.5)
	baseY := neighborHalf + 0.05

	for ring := range overflowPileRingCount {
		y := baseY + float32(ringSpacing*float32(ring))

		// Offset alternate rings by half a slot so neighbors don't sit
		// directly above each other.
		thetaOffset := float32(0.0)
		if ring&1 != 0 {
			pi := Pi
			thetaOffset = pi / overflowPilePerRing
		}

		for slot := range overflowPilePerRing {
			twoPi := float32(2.0 * Pi)
			theta := thetaOffset + float32(twoPi*float32(slot))/overflowPilePerRing

			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			cosTheta := float32(math.Cos(float64(theta)))
			sinTheta := float32(math.Sin(float64(theta)))
			bodyDef.Position = Pos{float32(ringRadius * cosTheta), y, float32(ringRadius * sinTheta)}
			bodyId := worldId.CreateBody(&bodyDef)

			bodyId.CreateHullShape(&neighborShape, &neighborBox.Base)
		}
	}

	return data
}
