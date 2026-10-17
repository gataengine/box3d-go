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

// PORT: both upstream tests drive a full world (b3CreateWorld, b3World_Step, b3Body_*,
// b3Create*Shape, b3World_RebuildStaticTree, and for the stress test b3CreateWeldJoint and
// b3Joint_*), so they are pending (PIt) until those files land; each names the missing upstream
// files. Port their bodies, CheckPairSet, the xorshift RandomState helpers and the stress context
// from test/test_broad_phase.c then. BroadPhaseTest's b3SetAssertFcn( FlushingAssert ) exists to
// flush stdout before a crash in the C runner and is not needed under Ginkgo.
// Until then broad_phase_golden_test.go checks the broad-phase against the C build directly.

var _ = Describe("BroadPhaseTest", func() {
	PIt("StaticRebuildKeepsMarks (needs physics_world.c, body.c, shape.c, contact.c, solver.c and the rest of milestone 3)")
	PIt("BroadPhaseStressTest (needs milestone 3 world files plus joint.c and weld_joint.c from milestone 4)")
})
