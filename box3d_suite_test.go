// Port of test/main.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"testing"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

// TestBox3D runs every upstream unit test file as one Ginkgo suite. Upstream's main.c filter
// argument maps to `go test -ginkgo.focus`.
func TestBox3D(t *testing.T) {
	RegisterFailHandler(Fail)
	RunSpecs(t, "Box3D Suite")
}
