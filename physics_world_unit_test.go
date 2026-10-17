// Go-only test: no upstream counterpart. Focused checks of physics_world.c functions that run
// without the rest of milestone 3 (test_world.c, in world_test.go, needs the whole world).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// physicsWorldMinimal builds the part of b3CreateWorld that the functions under test touch: the
// hull database and the three empty base solver sets with their id pool.
func physicsWorldMinimal() *world {
	w := &world{}
	w.hullDatabase = new(hullMap)
	hullMapInit(w.hullDatabase)

	w.bodyIdPool = createIdPool()
	w.contactIdPool = createIdPool()
	w.jointIdPool = createIdPool()
	w.islandIdPool = createIdPool()
	w.solverSetIdPool = createIdPool()
	for range 3 {
		var set solverSet
		set.setIndex = int32(allocId(&w.solverSetIdPool))
		arrayPush(&w.solverSets, set)
	}
	return w
}

var _ = Describe("PhysicsWorldUnitTest", func() {
	It("HullDatabaseReferenceCounts", func() {
		w := physicsWorldMinimal()

		box := MakeBoxHull(0.5, 0.5, 0.5)
		box2 := MakeBoxHull(0.5, 0.5, 0.5)
		other := MakeBoxHull(0.3, 0.3, 0.3)

		// The database owns a clone, identical content shares it
		a := addHullToDatabase(w, &box.Base)
		ensure(a != nil && a != &box.Base)
		ensure(compareHullData(a, &box.Base))
		ensure(addHullToDatabase(w, &box2.Base) == a)
		ensure(w.hullDatabase.size() == 1)

		// An owned hull with the same content is dropped in favor of the shared copy
		owned := CloneHull(&box.Base)
		ensure(addOwnedHullToDatabase(w, owned) == a)
		ensure(w.hullDatabase.size() == 1)

		// An owned hull with new content is taken as is
		ownedOther := CloneHull(&other.Base)
		ensure(addOwnedHullToDatabase(w, ownedOther) == ownedOther)
		ensure(w.hullDatabase.size() == 2)

		// Three references to a, one to ownedOther
		removeHullFromDatabase(w, a)
		removeHullFromDatabase(w, a)
		ensure(w.hullDatabase.size() == 2)
		ensure(w.hullDatabase.isEnd(w.hullDatabase.get(&box.Base)) == false)
		removeHullFromDatabase(w, a)
		ensure(w.hullDatabase.size() == 1)
		ensure(w.hullDatabase.isEnd(w.hullDatabase.get(&box.Base)) == true)

		removeHullFromDatabase(w, ownedOther)
		ensure(w.hullDatabase.size() == 0)
	})

	It("NullIdsAreInvalid", func() {
		ensure(NullWorldID.IsValid() == false)
		ensure(WorldID{MaxWorlds + 1, 0}.IsValid() == false)
		ensure(NullBodyID.IsValid() == false)
		ensure(BodyID{1, MaxWorlds, 0}.IsValid() == false)
		ensure(NullShapeID.IsValid() == false)
		ensure(ShapeID{1, MaxWorlds, 0}.IsValid() == false)
		ensure(NullJointID.IsValid() == false)
		ensure(JointID{1, MaxWorlds, 0}.IsValid() == false)
		ensure(NullContactID.IsValid() == false)
		ensure(ContactID{1, MaxWorlds, 0, 0}.IsValid() == false)
	})

	It("ValidateEmptyWorld", func() {
		// Only asserts under box3d_validate; an empty world satisfies every invariant.
		w := physicsWorldMinimal()
		validateSolverSets(w)
		validateContacts(w)
		validateConnectivity(w)
	})

	It("DefaultMixingCallbacks", func() {
		ensure(defaultFrictionCallback(0.25, 1, 0.64, 2) == sqrtf(0.25*0.64))
		ensure(defaultRestitutionCallback(0.25, 1, 0.64, 2) == 0.64)
	})
})
