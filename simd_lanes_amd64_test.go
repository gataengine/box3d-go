// Go-only test: upstream has no test file for the lane layer (src/simd_sse2.h).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build box3d_simd && goexperiment.simd && amd64

package box3d

import (
	"simd/archsimd"

	. "github.com/onsi/ginkgo/v2"
)

var _ = Describe("SIMDLanes", func() {
	It("RequiresAVX", func() {
		// The x4 backend is AVX encoded; simd_x4_amd64.go's init panics without AVX, so a
		// running test binary implies it, and the message names the fix.
		ensure(avxRequiredMessage != "")
		ensure(archsimd.X86.AVX())
	})
})
