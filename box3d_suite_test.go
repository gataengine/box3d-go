// Port of test/main.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"os"
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

// suiteSIMDWidth is the SetSIMDWidth value specs restore after changing the width: 0 (auto-detect)
// or 4 when BOX3D_TEST_FORCE_WIDTH4=1.
// PORT: Go-only. BOX3D_TEST_FORCE_WIDTH4=1 runs the suite at width 4 on an AVX2 host, so the
// width 4 kernels of the box3d_simd build are covered on amd64 as well (upstream covers them on
// x86 with BOX3D_DISABLE_AVX2 builds). Specs that compare widths still set both explicitly.
var suiteSIMDWidth = 0

var _ = BeforeSuite(func() {
	if os.Getenv("BOX3D_TEST_FORCE_WIDTH4") == "1" {
		suiteSIMDWidth = 4
	}
	SetSIMDWidth(suiteSIMDWidth)
	GinkgoWriter.Printf("SIMD width %d\n", GetSIMDWidth())
})
