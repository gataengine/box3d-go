// Port of test/test_broad_phase.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// PORT: BroadPhaseStressTest creates weld joints and toggles b3Joint_SetCollideConnected, so it is
// pending (PIt) until joint.c and weld_joint.c land in milestone 4. Port its body, CheckPairSet,
// the xorshift RandomState helpers and the stress context from test/test_broad_phase.c then.
// BroadPhaseTest's b3SetAssertFcn( FlushingAssert ) exists to flush stdout before a crash in the C
// runner and is not needed under Ginkgo. The static helpers are prefixed with broadPhase so they
// cannot clash in the one test package.

// broadPhaseRunStaticRebuildKeepsMarks is RunStaticRebuildKeepsMarks.
// The broad-phase keeps its pending pair marks inside the static tree, so a full rebuild between
// a static mutation and the next step used to wipe them and the contact never appeared. Covers
// both ways a static proxy gets marked: a teleport and a shape created with invokeContactCreation.
func broadPhaseRunStaticRebuildKeepsMarks(useInvoke bool) int {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec3Zero
	worldId := CreateWorld(&worldDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Pos{0.0, 1.0, 0.0}
	sphereId := worldId.CreateBody(&bodyDef)
	shapeDef := DefaultShapeDef()
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
	sphereId.CreateSphereShape(&shapeDef, &sphere)

	bodyDef.Type = StaticBody
	bodyDef.Position = Pos{100.0, 0.0, 0.0}
	groundId := worldId.CreateBody(&bodyDef)
	box := MakeBoxHull(2.0, 0.5, 2.0)
	if useInvoke == false {
		groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	// Settle so the resting sphere carries no mark of its own
	for i := 0; i < 3; i++ {
		worldId.Step(1.0/60.0, 4)
	}

	groundId.SetTransform(PosZero, QuatIdentity)
	if useInvoke {
		invokeDef := DefaultShapeDef()
		invokeDef.InvokeContactCreation = true
		groundId.CreateHullShape(&invokeDef, &box.Base)
	}

	worldId.RebuildStaticTree()
	worldId.Step(1.0/60.0, 4)

	ensure(sphereId.GetContactCapacity() == 1)

	worldId.Destroy()
	return 0
}

var _ = Describe("BroadPhaseTest", func() {
	It("StaticRebuildKeepsMarks", func() {
		ensure(broadPhaseRunStaticRebuildKeepsMarks(false) == 0)
		ensure(broadPhaseRunStaticRebuildKeepsMarks(true) == 0)
	})

	PIt("BroadPhaseStressTest (needs weld joints: joint.c and weld_joint.c, milestone 4)")
})
