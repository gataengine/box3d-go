// Port of src/sensor.h (and later src/sensor.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- sensor.h ----

// PORT: header types only; sensor.c lands in milestone 3.

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
}

// ---- sensor.c ----

// PORT: stubs until sensor.c lands in milestone 3. Other milestone 3 files (body.c and
// physics_world.c) call these; the owner of sensor.c replaces each stub with the real function,
// keeping the signature, in upstream order.

// overlapSensors is b3OverlapSensors.
func overlapSensors(world *world) {
	panic("box3d: b3OverlapSensors not ported yet (sensor.c, milestone 3)")
}

// destroySensor is b3DestroySensor.
func destroySensor(world *world, sensorShape *shape) {
	panic("box3d: b3DestroySensor not ported yet (sensor.c, milestone 3)")
}
