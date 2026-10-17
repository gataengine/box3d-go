// Go-only file: build tag switch for B3_ASSERT (include/box3d/base.h).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

//go:build !box3d_noassert

package box3d

// assertEnabled mirrors `!defined( NDEBUG ) || defined( B3_ENABLE_ASSERT )`.
const assertEnabled = true
