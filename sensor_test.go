// Go-only test: sensor.c has no unit test upstream (sensors are exercised by world-level tests
// after milestone 3 integration). This drives b3OverlapSensors and b3DestroySensor on a hand-built
// world with empty broad-phase trees, so overlaps come only from sensor hits (the time of impact
// path): sorting, deduplication, begin/end/persist events with generations, the disabled path,
// event order across sensors with 1 and 4 workers, and the moved-sensor fixup on destroy.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

const sensorTestCount = 40

func sensorTestWorld(workerCount int, enqueue EnqueueTaskCallback, finish FinishTaskCallback) *world {
	w := &world{}
	capacity := Capacity{}
	createBroadPhase(&w.broadPhase, &capacity)
	w.workerCount = int32(workerCount)
	w.enqueueTaskFcn = enqueue
	w.finishTaskFcn = finish
	w.worldId = 3
	w.sensorTaskContexts = make([]sensorTaskContext, workerCount)
	for i := range workerCount {
		w.sensorTaskContexts[i].eventBits = createBitSet(sensorTestCount)
	}

	// static, disabled, awake
	w.solverSets = make([]solverSet, 3)
	w.solverSets[awakeSet].bodySims = make([]bodySim, sensorTestCount)
	w.bodies = make([]body, sensorTestCount)
	w.shapes = make([]shape, sensorTestCount)
	w.sensors = make([]sensor, sensorTestCount)
	for i := range sensorTestCount {
		w.solverSets[awakeSet].bodySims[i].transform = WorldTransformIdentity
		w.bodies[i].id = int32(i)
		w.bodies[i].setIndex = awakeSet
		w.bodies[i].localIndex = int32(i)

		s := &w.shapes[i]
		s.id = int32(i)
		s.bodyId = int32(i)
		s.typ = SphereShape
		s.sphere = Sphere{Center: Vec3{}, Radius: 1.0}
		s.aabb = AABB{LowerBound: Vec3{-1, -1, -1}, UpperBound: Vec3{1, 1, 1}}
		s.filter = DefaultFilter()
		s.flags = uint8(enableSensorEvents)
		s.generation = uint16(10 + i)
		s.sensorIndex = int32(i)

		w.sensors[i].shapeId = int32(i)
	}
	return w
}

func sensorTestShapeID(w *world, shapeId int32, generation uint16) ShapeID {
	return ShapeID{shapeId + 1, w.worldId, generation}
}

func sensorTestStep(w *world) ([]SensorBeginTouchEvent, []SensorEndTouchEvent) {
	w.sensorBeginEvents = w.sensorBeginEvents[:0]
	w.sensorEndEvents[w.endEventArrayIndex] = w.sensorEndEvents[w.endEventArrayIndex][:0]
	overlapSensors(w)
	return w.sensorBeginEvents, w.sensorEndEvents[w.endEventArrayIndex]
}

func sensorTestRun(workerCount int, enqueue EnqueueTaskCallback, finish FinishTaskCallback) {
	w := sensorTestWorld(workerCount, enqueue, finish)
	sid := func(k int) ShapeID { return sensorTestShapeID(w, int32(k), uint16(10+k)) }
	vid := func(id int32, gen uint16) ShapeID { return sensorTestShapeID(w, id, gen) }

	// Step 1: even sensors get unsorted hits with duplicates.
	for k := 0; k < sensorTestCount; k += 2 {
		w.sensors[k].hits = append(w.sensors[k].hits, visitor{7, 1}, visitor{3, 2}, visitor{7, 1}, visitor{5, 0})
	}
	begin, end := sensorTestStep(w)
	ensure(len(end) == 0)
	ensure(len(begin) == 3*sensorTestCount/2)
	for k := 0; k < sensorTestCount; k += 2 {
		b := begin[3*(k/2):]
		ensure(b[0] == SensorBeginTouchEvent{sid(k), vid(3, 2)})
		ensure(b[1] == SensorBeginTouchEvent{sid(k), vid(5, 0)})
		ensure(b[2] == SensorBeginTouchEvent{sid(k), vid(7, 1)})
		ensure(len(w.sensors[k].overlaps2) == 3)
		ensure(len(w.sensors[k].hits) == 0)
	}
	for k := 1; k < sensorTestCount; k += 2 {
		ensure(len(w.sensors[k].overlaps2) == 0)
	}

	// Step 2: visitor 5 persists; on sensor 2 visitor 3 comes back with a new generation.
	for k := 0; k < sensorTestCount; k += 2 {
		w.sensors[k].hits = append(w.sensors[k].hits, visitor{5, 0})
	}
	w.sensors[2].hits = append(w.sensors[2].hits, visitor{3, 3})
	begin, end = sensorTestStep(w)
	ensure(len(begin) == 1)
	ensure(begin[0] == SensorBeginTouchEvent{sid(2), vid(3, 3)})
	ensure(len(end) == 2*sensorTestCount/2)
	// sensor 0: end 3, end 7
	ensure(end[0] == SensorEndTouchEvent{sid(0), vid(3, 2)})
	ensure(end[1] == SensorEndTouchEvent{sid(0), vid(7, 1)})
	// sensor 2: end 3 (old generation), then begin 3 (above), then end 7
	ensure(end[2] == SensorEndTouchEvent{sid(2), vid(3, 2)})
	ensure(end[3] == SensorEndTouchEvent{sid(2), vid(7, 1)})
	for k := 4; k < sensorTestCount; k += 2 {
		e := end[k:]
		ensure(e[0] == SensorEndTouchEvent{sid(k), vid(3, 2)})
		ensure(e[1] == SensorEndTouchEvent{sid(k), vid(7, 1)})
	}

	// Step 3: nothing changes, so no events.
	for k := 0; k < sensorTestCount; k += 2 {
		w.sensors[k].hits = append(w.sensors[k].hits, visitor{5, 0})
	}
	w.sensors[2].hits = append(w.sensors[2].hits, visitor{3, 3})
	begin, end = sensorTestStep(w)
	ensure(len(begin) == 0 && len(end) == 0)

	// Step 4: sensor 4 is disabled and drops its overlaps; sensor 6 turns sensor events off.
	// Everything else keeps its hits.
	w.bodies[4].setIndex = disabledSet
	w.shapes[6].flags = 0
	for k := 0; k < sensorTestCount; k += 2 {
		if k != 4 && k != 6 {
			w.sensors[k].hits = append(w.sensors[k].hits, visitor{5, 0})
		}
	}
	w.sensors[2].hits = append(w.sensors[2].hits, visitor{3, 3})
	begin, end = sensorTestStep(w)
	ensure(len(begin) == 0)
	ensure(len(end) == 2)
	ensure(end[0] == SensorEndTouchEvent{sid(4), vid(5, 0)})
	ensure(end[1] == SensorEndTouchEvent{sid(6), vid(5, 0)})

	// Destroy sensor 2: its overlaps end, the last sensor moves into its slot.
	w.sensorEndEvents[0] = w.sensorEndEvents[0][:0]
	destroySensor(w, &w.shapes[2])
	end = w.sensorEndEvents[0]
	ensure(len(end) == 2)
	ensure(end[0] == SensorEndTouchEvent{sid(2), vid(3, 3)})
	ensure(end[1] == SensorEndTouchEvent{sid(2), vid(5, 0)})
	ensure(len(w.sensors) == sensorTestCount-1)
	ensure(w.sensors[2].shapeId == sensorTestCount-1)
	ensure(w.shapes[sensorTestCount-1].sensorIndex == 2)

	// Destroying the last sensor moves nothing.
	last := &w.shapes[w.sensors[len(w.sensors)-1].shapeId]
	destroySensor(w, last)
	ensure(len(w.sensors) == sensorTestCount-2)
}

var _ = Describe("SensorTest", func() {
	It("serial", func() {
		sensorTestRun(1, testSerialEnqueue, testSerialFinish)
	})

	It("4 goroutine workers", func() {
		sensorTestRun(4, testGoroutineEnqueue, testGoroutineFinish)
	})
})
