// Port of src/sensor.h and src/sensor.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- sensor.h ----

// sensorHit is b3SensorHit: used to track shapes that hit sensors using time of impact
type sensorHit struct {
	sensorId  int32
	visitorId int32
}

// visitor is b3Visitor.
type visitor struct {
	shapeId    int32
	generation uint16
}

// sensor is b3Sensor.
type sensor struct {
	hits      []visitor
	overlaps1 []visitor
	overlaps2 []visitor
	shapeId   int32
}

// sensorTaskContext is b3SensorTaskContext.
type sensorTaskContext struct {
	eventBits bitSet

	// PORT: storage for the b3SensorQueryContext local of b3SensorTask.
	queryContext sensorQueryContext
}

// ---- sensor.c ----

// sensorQueryContext is b3SensorQueryContext.
type sensorQueryContext struct {
	world       *world
	taskContext *sensorTaskContext
	sensor      *sensor
	sensorShape *shape
	transform   Transform
}

// overlapSensor is b3OverlapSensor.
func overlapSensor(sensorShape *shape, sensorTransform Transform, visitorShape *shape,
	visitorTransform Transform) bool {
	typ := sensorShape.typ

	proxy := makeShapeProxy(visitorShape)

	// Get the visitor shape in the frame of the sensor
	relativeTransform := InvMulTransforms(sensorTransform, visitorTransform)

	var localPoints [MaxShapeCastPoints]Vec3
	var localProxy ShapeProxy

	localProxy.Count = int32(MinInt(int(proxy.Count), MaxShapeCastPoints))
	for i := 0; i < int(localProxy.Count); i++ {
		localPoints[i] = TransformPoint(relativeTransform, proxy.Points[i])
	}

	localProxy.Points = localPoints[:localProxy.Count]
	localProxy.Radius = proxy.Radius

	switch typ {
	case CapsuleShape:
		return OverlapCapsule(&sensorShape.capsule, TransformIdentity, &localProxy)

	case CompoundShape:
		return OverlapCompound(sensorShape.compound, TransformIdentity, &localProxy)

	case HeightShape:
		return OverlapHeightField(sensorShape.heightField, TransformIdentity, &localProxy)

	case HullShape:
		return OverlapHull(sensorShape.hull, TransformIdentity, &localProxy)

	case MeshShape:
		return OverlapMesh(&sensorShape.mesh, TransformIdentity, &localProxy)

	case SphereShape:
		return OverlapSphere(&sensorShape.sphere, TransformIdentity, &localProxy)

	default:
		assert(false, "false")
		return false
	}
}

// Sensor shapes need to
// - detect begin and end overlap events
// - events must be reported in deterministic order
// - maintain an active list of overlaps for query

// Assumption
// - sensors don't detect shapes on the same body

// Algorithm
// Query all sensors for overlaps
// Check against previous overlaps

// Data structures
// Each sensor has an double buffered array of overlaps
// These overlaps use a shape reference with index and generation

// sensorQueryCallback is b3SensorQueryCallback.
func sensorQueryCallback(proxyId int, userData uint64, context any) bool {
	shapeId := int32(userData)

	queryContext := context.(*sensorQueryContext)
	sensorShape := queryContext.sensorShape
	sensorShapeId := sensorShape.id

	if shapeId == sensorShapeId {
		return true
	}

	world := queryContext.world
	otherShape := arrayGet(world.shapes, int(shapeId))

	// Visitors must be convex.
	if isConvex(otherShape.typ) == false {
		return true
	}

	// Are sensor events enabled on the other shape?
	if (otherShape.flags & uint8(enableSensorEvents)) == 0 {
		return true
	}

	// Skip shapes on the same body
	if otherShape.bodyId == sensorShape.bodyId {
		return true
	}

	// Check filter
	if shouldShapesCollide(sensorShape.filter, otherShape.filter) == false {
		return true
	}

	// Custom user filter
	if (sensorShape.flags&uint8(enableCustomFiltering)) != 0 || (otherShape.flags&uint8(enableCustomFiltering)) != 0 {
		customFilterFcn := queryContext.world.customFilterFcn
		if customFilterFcn != nil {
			idA := ShapeID{sensorShapeId + 1, world.worldId, sensorShape.generation}
			idB := ShapeID{shapeId + 1, world.worldId, otherShape.generation}
			shouldCollide := customFilterFcn(idA, idB, queryContext.world.customFilterContext)
			if shouldCollide == false {
				return true
			}
		}
	}

	otherTransform := ToRelativeTransform(getBodyTransform(world, int(otherShape.bodyId)), PosZero)

	overlap := overlapSensor(sensorShape, queryContext.transform, otherShape, otherTransform)
	if overlap == false {
		return true
	}

	// Record the overlap
	sensor := queryContext.sensor
	shapeRef := arrayEmplace(&sensor.overlaps2)
	shapeRef.shapeId = shapeId
	shapeRef.generation = otherShape.generation

	return true
}

// compareVisitors is b3CompareVisitors.
// PORT: the qsort comparator becomes the LESS primitive of QSORT (see sensorTask).
func compareVisitors(sa, sb *visitor) bool {
	if sa.shapeId < sb.shapeId {
		return true
	}

	return false
}

// sensorTask is b3SensorTask.
func sensorTask(startIndex, endIndex, workerIndex int, context any) {
	world := context.(*world)
	assert(int32(workerIndex) < world.workerCount, "(int)workerIndex < world->workerCount")
	taskContext := &world.sensorTaskContexts[workerIndex]

	assert(startIndex < endIndex, "startIndex < endIndex")

	trees := &world.broadPhase.trees
	for sensorIndex := startIndex; sensorIndex < endIndex; sensorIndex++ {
		sensor := arrayGet(world.sensors, sensorIndex)
		sensorShape := arrayGet(world.shapes, int(sensor.shapeId))

		// Swap overlap arrays
		temp := sensor.overlaps1
		sensor.overlaps1 = sensor.overlaps2
		sensor.overlaps2 = temp
		arrayClear(&sensor.overlaps2)

		// Append sensor hits
		arrayAppend(&sensor.overlaps2, sensor.hits)

		// Clear the hits
		arrayClear(&sensor.hits)

		body := arrayGet(world.bodies, int(sensorShape.bodyId))
		if body.setIndex == disabledSet || (sensorShape.flags&uint8(enableSensorEvents)) == 0 {
			if len(sensor.overlaps1) != 0 {
				// This sensor is dropping all overlaps because it has been disabled.
				setBit(&taskContext.eventBits, uint32(sensorIndex))
			}
			continue
		}

		transform := ToRelativeTransform(getBodyTransformQuick(world, body), PosZero)

		// PORT: the trees receive the query context as `any`, so a local would escape to the heap;
		// it lives in the worker's sensor task context instead.
		taskContext.queryContext = sensorQueryContext{
			world:       world,
			taskContext: taskContext,
			sensor:      sensor,
			sensorShape: sensorShape,
			transform:   transform,
		}
		queryContext := &taskContext.queryContext

		assert(int(sensorShape.sensorIndex) == sensorIndex, "sensorShape->sensorIndex == sensorIndex")
		queryBounds := sensorShape.aabb

		// Query all trees
		trees[0].Query(queryBounds, sensorShape.filter.MaskBits, false, sensorQueryCallback, queryContext)
		trees[1].Query(queryBounds, sensorShape.filter.MaskBits, false, sensorQueryCallback, queryContext)
		trees[2].Query(queryBounds, sensorShape.filter.MaskBits, false, sensorQueryCallback, queryContext)

		// Sort the overlaps to enable finding begin and end events.
		// PORT: libc qsort becomes the ported QSORT (qsort.go). Neither is stable, but entries with
		// equal shapeId are identical visitors (a hit and a query result for the same live shape
		// carry the same generation), so the deduplicated result does not depend on the sort.
		overlaps := sensor.overlaps2
		qsort(len(overlaps),
			func(i, j int) bool { return compareVisitors(&overlaps[i], &overlaps[j]) },
			func(i, j int) { overlaps[i], overlaps[j] = overlaps[j], overlaps[i] })

		// Remove duplicates from overlaps2 (sorted). Duplicates are possible due to the hit events appended earlier.
		uniqueCount := 0
		overlapCount := len(sensor.overlaps2)
		overlapData := sensor.overlaps2
		for i := 0; i < overlapCount; i++ {
			if uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount-1].shapeId {
				overlapData[uniqueCount] = overlapData[i]
				uniqueCount += 1
			}
		}
		sensor.overlaps2 = sensor.overlaps2[:uniqueCount]

		count1 := len(sensor.overlaps1)
		count2 := len(sensor.overlaps2)
		if count1 != count2 {
			// something changed
			setBit(&taskContext.eventBits, uint32(sensorIndex))
		} else {
			for i := 0; i < count1; i++ {
				s1 := &sensor.overlaps1[i]
				s2 := &sensor.overlaps2[i]

				if s1.shapeId != s2.shapeId || s1.generation != s2.generation {
					// something changed
					setBit(&taskContext.eventBits, uint32(sensorIndex))
					break
				}
			}
		}
	}
}

// overlapSensors is b3OverlapSensors.
func overlapSensors(world *world) {
	sensorCount := len(world.sensors)
	if sensorCount == 0 {
		return
	}

	assert(world.workerCount > 0, "world->workerCount > 0")

	for i := 0; i < int(world.workerCount); i++ {
		setBitCountAndClear(&world.sensorTaskContexts[i].eventBits, uint32(sensorCount))
	}

	// Parallel-for sensors overlaps
	minRange := 16
	parallelFor(world, sensorTask, sensorCount, minRange, world, "sensors")

	bitSet := &world.sensorTaskContexts[0].eventBits
	for i := 1; i < int(world.workerCount); i++ {
		inPlaceUnion(bitSet, &world.sensorTaskContexts[i].eventBits)
	}

	// Iterate sensors bits and publish events
	// Process sensor state changes. Iterate over set bits
	bits := bitSet.bits
	blockCount := bitSet.blockCount

	for k := uint32(0); k < blockCount; k++ {
		word := bits[k]
		for word != 0 {
			ctz := ctz64(word)
			sensorIndex := int(64*k + ctz)

			sensor := arrayGet(world.sensors, sensorIndex)
			sensorShape := arrayGet(world.shapes, int(sensor.shapeId))
			sensorId := ShapeID{sensor.shapeId + 1, world.worldId, sensorShape.generation}

			count1 := len(sensor.overlaps1)
			count2 := len(sensor.overlaps2)
			refs1 := sensor.overlaps1
			refs2 := sensor.overlaps2

			// overlaps1 can have overlaps that end
			// overlaps2 can have overlaps that begin
			index1, index2 := 0, 0
			for index1 < count1 && index2 < count2 {
				r1 := &refs1[index1]
				r2 := &refs2[index2]
				if r1.shapeId == r2.shapeId {
					if r1.generation < r2.generation {
						// end
						visitorId := ShapeID{r1.shapeId + 1, world.worldId, r1.generation}
						event := SensorEndTouchEvent{
							SensorShapeId:  sensorId,
							VisitorShapeId: visitorId,
						}
						arrayPush(&world.sensorEndEvents[world.endEventArrayIndex], event)
						index1 += 1
					} else if r1.generation > r2.generation {
						// begin
						visitorId := ShapeID{r2.shapeId + 1, world.worldId, r2.generation}
						event := SensorBeginTouchEvent{sensorId, visitorId}
						arrayPush(&world.sensorBeginEvents, event)
						index2 += 1
					} else {
						// persisted
						index1 += 1
						index2 += 1
					}
				} else if r1.shapeId < r2.shapeId {
					// end
					visitorId := ShapeID{r1.shapeId + 1, world.worldId, r1.generation}
					event := SensorEndTouchEvent{sensorId, visitorId}
					arrayPush(&world.sensorEndEvents[world.endEventArrayIndex], event)
					index1 += 1
				} else {
					// begin
					visitorId := ShapeID{r2.shapeId + 1, world.worldId, r2.generation}
					event := SensorBeginTouchEvent{sensorId, visitorId}
					arrayPush(&world.sensorBeginEvents, event)
					index2 += 1
				}
			}

			for index1 < count1 {
				// end
				r1 := &refs1[index1]
				visitorId := ShapeID{r1.shapeId + 1, world.worldId, r1.generation}
				event := SensorEndTouchEvent{sensorId, visitorId}
				arrayPush(&world.sensorEndEvents[world.endEventArrayIndex], event)
				index1 += 1
			}

			for index2 < count2 {
				// begin
				r2 := &refs2[index2]
				visitorId := ShapeID{r2.shapeId + 1, world.worldId, r2.generation}
				event := SensorBeginTouchEvent{sensorId, visitorId}
				arrayPush(&world.sensorBeginEvents, event)
				index2 += 1
			}

			// Clear the smallest set bit
			word = word & (word - 1)
		}
	}
}

// destroySensor is b3DestroySensor.
func destroySensor(world *world, sensorShape *shape) {
	sensor := arrayGet(world.sensors, int(sensorShape.sensorIndex))
	for i := 0; i < len(sensor.overlaps2); i++ {
		ref := &sensor.overlaps2[i]
		event := SensorEndTouchEvent{
			SensorShapeId: ShapeID{
				Index1:     sensorShape.id + 1,
				World0:     world.worldId,
				Generation: sensorShape.generation,
			},
			VisitorShapeId: ShapeID{
				Index1:     ref.shapeId + 1,
				World0:     world.worldId,
				Generation: ref.generation,
			},
		}

		arrayPush(&world.sensorEndEvents[world.endEventArrayIndex], event)
	}

	// Destroy sensor
	arrayDestroy(&sensor.hits)
	arrayDestroy(&sensor.overlaps1)
	arrayDestroy(&sensor.overlaps2)

	movedIndex := arrayRemoveSwap(&world.sensors, int(sensorShape.sensorIndex))
	if movedIndex != NullIndex {
		// Fixup moved sensor
		movedSensor := arrayGet(world.sensors, int(sensorShape.sensorIndex))
		otherSensorShape := arrayGet(world.shapes, int(movedSensor.shapeId))
		otherSensorShape.sensorIndex = sensorShape.sensorIndex
	}
}
