// Port of test/test_id.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

var _ = Describe("IdTest", func() {
	It("IdTest", func() {
		x := uint64(0x0123456789ABCDEF)

		{
			id := LoadBodyID(x)
			y := StoreBodyID(id)
			ensure(x == y)
		}

		{
			id := LoadShapeID(x)
			y := StoreShapeID(id)
			ensure(x == y)
		}

		{
			id := LoadJointID(x)
			y := StoreJointID(id)
			ensure(x == y)
		}
	})
})
