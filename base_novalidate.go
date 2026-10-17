// Go-only file: build tag switch for B3_VALIDATE (include/box3d/base.h).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build !box3d_validate || box3d_noassert

package box3d

// validateEnabled mirrors B3_ENABLE_VALIDATION: BOX3D_VALIDATE in a debug build.
const validateEnabled = false
