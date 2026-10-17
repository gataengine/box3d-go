// Go-only test: no upstream counterpart.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// The upstream body test (body_test.go) needs a full world and runs after the milestone 3
// integration. These specs cover the body.c internals that only touch the arrays of a world
// assembled by hand: id and sim lookups, flag syncing, contact index refresh and the isotropic
// inertia gate.

// bodyUnitWorld builds a world with one static body (id 0) and two awake bodies (ids 1 and 2).
func bodyUnitWorld() *world {
	w := &world{}
	w.worldId = 3
	w.isotropicInertiaTolerance = 0.2
	w.solverSets = make([]solverSet, firstSleepingSet)
	for i := range w.solverSets {
		w.solverSets[i].setIndex = int32(i)
	}

	static := &w.solverSets[staticSet]
	arrayPush(&static.bodySims, bodySim{bodyId: 0})
	awake := &w.solverSets[awakeSet]
	arrayPush(&awake.bodySims, bodySim{bodyId: 1})
	arrayPush(&awake.bodySims, bodySim{bodyId: 2})
	arrayPush(&awake.bodyStates, identityBodyState)
	arrayPush(&awake.bodyStates, identityBodyState)

	arrayPush(&w.bodies, body{id: 0, setIndex: staticSet, localIndex: 0, generation: 5, headContactKey: NullIndex})
	arrayPush(&w.bodies, body{id: 1, setIndex: awakeSet, localIndex: 0, generation: 7, headContactKey: NullIndex})
	arrayPush(&w.bodies, body{id: 2, setIndex: awakeSet, localIndex: 1, generation: 9, headContactKey: NullIndex})
	return w
}

var _ = Describe("BodyUnit", func() {
	It("lookups", func() {
		w := bodyUnitWorld()

		id := makeBodyId(w, 2)
		ensure(id == BodyID{3, 3, 9})

		static := &w.bodies[0]
		awake := &w.bodies[2]
		ensure(isBodyAwake(w, static) == false)
		ensure(isBodyAwake(w, awake))

		ensure(getBodyState(w, static) == nil)
		ensure(getBodyState(w, awake) == &w.solverSets[awakeSet].bodyStates[1])
		ensure(getBodySim(w, awake) == &w.solverSets[awakeSet].bodySims[1])
		ensure(getBodySim(w, static).bodyId == 0)

		w.solverSets[awakeSet].bodySims[1].transform.P = Pos{1.0, 2.0, 3.0}
		ensure(getBodyTransform(w, 2).P == Pos{1.0, 2.0, 3.0})
	})

	It("syncBodyFlags", func() {
		w := bodyUnitWorld()
		b := &w.bodies[1]
		sim := getBodySim(w, b)
		state := getBodyState(w, b)

		sim.flags = isFast | hadTimeOfImpact | isBullet
		b.flags = dynamicFlag | enableSleep | isSpeedCapped | hadTimeOfImpact | lockAngularX
		syncBodyFlags(w, b)

		// The sim keeps its own fast and time of impact flags and drops the stale bullet flag.
		ensure(sim.flags == isFast|hadTimeOfImpact|dynamicFlag|enableSleep|lockAngularX)
		// The state never sees transient flags.
		ensure(state.flags == dynamicFlag|enableSleep|lockAngularX)

		// A static body has no state; only the sim is written.
		s := &w.bodies[0]
		s.flags = isBullet
		syncBodyFlags(w, s)
		ensure(getBodySim(w, s).flags == isBullet)
	})

	It("refreshBodyContactIndices", func() {
		w := bodyUnitWorld()

		// Contact 0: body 2 is shape A (edge 0). Contact 1: body 2 is shape B (edge 1).
		w.contacts = make([]contact, 2)
		w.contacts[0].edges[0] = contactEdge{bodyId: 2, prevKey: NullIndex, nextKey: (1 << 1) | 1}
		w.contacts[1].edges[1] = contactEdge{bodyId: 2, prevKey: 0, nextKey: NullIndex}
		w.contacts[0].encodedBodySimB = 77
		w.contacts[1].encodedBodySimA = 88

		b := &w.bodies[2]
		b.headContactKey = 0 << 1
		refreshBodyContactIndices(w, b)
		ensure(w.contacts[0].encodedBodySimA == 1)
		ensure(w.contacts[1].encodedBodySimB == 1)
		ensure(w.contacts[0].encodedBodySimB == 77)
		ensure(w.contacts[1].encodedBodySimA == 88)

		// Static bodies encode as -(localIndex + 2).
		s := &w.bodies[0]
		s.headContactKey = 0 << 1
		w.contacts[0].edges[0].nextKey = NullIndex
		refreshBodyContactIndices(w, s)
		ensure(w.contacts[0].encodedBodySimA == -2)
	})

	It("updateIsotropicInertia", func() {
		w := bodyUnitWorld()
		b := &w.bodies[1]
		sim := getBodySim(w, b)
		state := getBodyState(w, b)

		// A cube is isotropic.
		sim.inertia = Matrix3{Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}}
		b.flags = enableGyroscopicTorque
		updateIsotropicInertia(w, b, sim)
		ensure(b.flags&enableGyroscopicTorque == 0)
		ensure(state.flags&enableGyroscopicTorque == 0)

		// A plank is not.
		sim.inertia = Matrix3{Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 2.0, 0.0}, Vec3{0.0, 0.0, 3.0}}
		updateIsotropicInertia(w, b, sim)
		ensure(b.flags&enableGyroscopicTorque != 0)
		ensure(sim.flags&enableGyroscopicTorque != 0)
		ensure(state.flags&enableGyroscopicTorque != 0)

		// Off diagonal terms count: 2 * deviation = 4 * 3 * c^2 against 3 * 0.2^2 = 0.12, so c = 0.05
		// stays isotropic (0.03) and c = 0.2 does not (0.48).
		sim.inertia = Matrix3{Vec3{1.0, 0.05, 0.05}, Vec3{0.05, 1.0, 0.05}, Vec3{0.05, 0.05, 1.0}}
		updateIsotropicInertia(w, b, sim)
		ensure(b.flags&enableGyroscopicTorque == 0)
		sim.inertia = Matrix3{Vec3{1.0, 0.2, 0.2}, Vec3{0.2, 1.0, 0.2}, Vec3{0.2, 0.2, 1.0}}
		updateIsotropicInertia(w, b, sim)
		ensure(b.flags&enableGyroscopicTorque != 0)

		// Zero tolerance: any anisotropy enables the gyroscopic solve, a perfect cube does not.
		w.isotropicInertiaTolerance = 0.0
		sim.inertia = Matrix3{Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0001}}
		updateIsotropicInertia(w, b, sim)
		ensure(b.flags&enableGyroscopicTorque != 0)
		sim.inertia = Matrix3{Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}}
		updateIsotropicInertia(w, b, sim)
		ensure(b.flags&enableGyroscopicTorque == 0)
	})

	It("updateBodyMassData without shapes", func() {
		w := bodyUnitWorld()
		b := &w.bodies[1]
		b.typ = DynamicBody
		b.headShapeId = NullIndex
		b.mass = 4.0
		b.flags = dirtyMass | enableGyroscopicTorque | dynamicFlag

		sim := getBodySim(w, b)
		sim.invMass = 0.25
		sim.inertia = Matrix3{Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 2.0, 0.0}, Vec3{0.0, 0.0, 3.0}}
		sim.invInertiaLocal = sim.inertia
		sim.invInertiaWorld = sim.inertia
		sim.localCenter = Vec3{1.0, 1.0, 1.0}
		sim.minExtent = 0.5
		sim.maxExtent = Vec3{1.0, 1.0, 1.0}

		updateBodyMassData(w, b)

		ensure(b.flags == dynamicFlag)
		ensure(getBodyState(w, b).flags == dynamicFlag)
		ensure(b.mass == 0.0)
		ensure(sim.invMass == 0.0)
		ensure(sim.inertia == Mat3Zero && sim.invInertiaLocal == Mat3Zero && sim.invInertiaWorld == Mat3Zero)
		ensure(sim.localCenter == Vec3Zero)
		ensure(sim.minExtent == Huge())
		ensure(sim.maxExtent == Vec3Zero)
	})
})
