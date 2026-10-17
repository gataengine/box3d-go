// Port of test/test_macros.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

// PORT: RUN_TEST / RUN_SUBTEST are replaced by Ginkgo Describe / It. MAYBE_UNUSED is dropped.

// ensure is ENSURE( C ). Gomega is only invoked on failure so hot test loops stay fast.
func ensure(c bool) {
	if !c {
		GinkgoHelper()
		Expect(c).To(BeTrue(), "condition false")
	}
}

// ensureSmall is ENSURE_SMALL( C, tol ): fails if C < -tol || tol < C.
// PORT: like the macro, a NaN value passes (both comparisons are false).
func ensureSmall(c, tol float32) {
	if c < -tol || tol < c {
		GinkgoHelper()
		Expect(c).To(BeNumerically("~", 0, tol))
	}
}

// arrayCount is ARRAY_COUNT( A ).
func arrayCount[T any](a []T) int {
	return len(a)
}
