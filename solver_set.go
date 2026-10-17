// Port of src/solver_set.h (and later src/solver_set.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- solver_set.h ----

// PORT: header types only; solver_set.c lands in milestone 3.

// solverSet is b3SolverSet.
// This holds solver set data. The following sets are used:
// - static set for all static bodies and joints between static bodies
// - active set for all active bodies with body states (no
// contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
// The purpose of solver sets is to achieve high memory locality.
// https://www.youtube.com/watch?v=nZNd5FjSquk
type solverSet struct {
	// Body array. Empty for unused set.
	bodySims []bodySim

	// Body state only exists for active set
	bodyStates []bodyState

	// This holds sleeping/disabled joints. Empty for static/active set.
	jointSims []jointSim

	// This holds all contacts for sleeping sets.
	// This holds non-touching contacts for the awake set.
	// This should be empty for the static and disabled sets.
	contactIndices []int32

	// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
	// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
	// islands will be naturally merged with the set is woken.
	// The static and disabled sets have no islands.
	// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
	islandSims []islandSim

	// Aligns with b3World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
	setIndex int32
}

// ---- solver_set.c ----

// PORT: stubs until solver_set.c lands in milestone 3. Other milestone 3 files (body.c, island.c,
// solver.c and physics_world.c) call these; the owner of solver_set.c replaces each stub with the
// real function, keeping the signature, in upstream order.

// destroySolverSet is b3DestroySolverSet.
func destroySolverSet(world *world, setIndex int) {
	panic("box3d: b3DestroySolverSet not ported yet (solver_set.c, milestone 3)")
}

// wakeSolverSet is b3WakeSolverSet.
func wakeSolverSet(world *world, setIndex int) {
	panic("box3d: b3WakeSolverSet not ported yet (solver_set.c, milestone 3)")
}

// trySleepIsland is b3TrySleepIsland.
func trySleepIsland(world *world, islandId int) {
	panic("box3d: b3TrySleepIsland not ported yet (solver_set.c, milestone 3)")
}

// transferBody is b3TransferBody.
func transferBody(world *world, targetSet, sourceSet *solverSet, body *body) {
	panic("box3d: b3TransferBody not ported yet (solver_set.c, milestone 3)")
}

// transferJoint is b3TransferJoint.
func transferJoint(world *world, targetSet, sourceSet *solverSet, joint *joint) {
	panic("box3d: b3TransferJoint not ported yet (solver_set.c, milestone 3)")
}
