// Port of include/box3d/base.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "runtime"

// PORT: compile-time options (config.h, BOX3D_USER_CONFIG), the shared library export macros,
// B3_INLINE, B3_FORCE_INLINE, B3_ALIGN_AS, B3_LITERAL, B3_ZERO_INIT and B3_PRINTF_FORMAT have
// no Go counterpart. Assertions are controlled by the box3d_noassert build tag and validation
// by the box3d_validate build tag (see base_assert.go and base_validate.go).

// Base functionality

// NullIndex is used to indicate null for interfaces that work with indices instead of pointers.
const NullIndex = -1

// PORT: b3AllocFcn, b3FreeFcn and b3SetAllocator are not ported. The Go garbage collector owns
// all memory; see LOOSE_ENDS.md.

// AssertFcn is the prototype for the user assert callback. Return 0 to skip the debugger break.
type AssertFcn func(condition string, fileName string, lineNumber int) int

// LogFcn is the prototype for user log callback. Used to log warnings.
type LogFcn func(message string)

// GetByteCount: total bytes allocated by Box3D. Implemented in core.go.

// SetAssertFcn overrides the default assert callback. Implemented in core.go.

// breakpoint is B3_BREAKPOINT: break to the debugger.
// PORT: Go has no portable debug trap, so this panics. A debugger stops on the panic.
func breakpoint(condition, fileName string, lineNumber int) {
	panic("box3d assertion failed: " + condition + " (" + fileName + ":" + itoa(lineNumber) + ")")
}

// assert is B3_ASSERT. msg is the condition text as written upstream.
// PORT: compiled out with the box3d_noassert build tag. Unlike the C macro the condition
// expression is still evaluated when assertions are off; it must not have side effects.
func assert(condition bool, msg string) {
	if assertEnabled && !condition {
		_, file, line, _ := runtime.Caller(1)
		internalAssert(msg, file, line)
	}
}

// validate is B3_VALIDATE. Validation is typically only enabled in debug builds.
// Floating point tolerance checks should use this instead of the regular assertion.
// PORT: enabled by the box3d_validate build tag (and not box3d_noassert).
func validate(condition bool, msg string) {
	if validateEnabled && !condition {
		_, file, line, _ := runtime.Caller(1)
		internalAssert(msg, file, line)
	}
}

// SetLogFcn overrides the default logging callback. Implemented in core.go.

// Version numbering scheme.
// See https://semver.org/
type Version struct {
	// Significant changes
	Major int32

	// Incremental changes
	Minor int32

	// Bug fixes
	Revision int32
}

// GetVersion, IsDoublePrecision and GetMaxManifoldPoints are implemented in core.go,
// IsAVX2Available in simd.go.

// GetTicks, GetMilliseconds, GetMillisecondsAndReset, Yield, Sleep and Hash are implemented in
// timer.go.

// HashInit is B3_HASH_INIT: simple djb2 hash function seed for determinism testing.
const HashInit = 5381

// itoa formats a small integer without importing strconv into the hot assert path.
func itoa(x int) string {
	if x == 0 {
		return "0"
	}
	var buf [20]byte
	i := len(buf)
	negative := x < 0
	if negative {
		x = -x
	}
	for x > 0 {
		i--
		buf[i] = byte('0' + x%10)
		x /= 10
	}
	if negative {
		i--
		buf[i] = '-'
	}
	return string(buf[i:])
}
