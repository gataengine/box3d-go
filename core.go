// Port of src/core.h and src/core.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"sync/atomic"
)

// ---- core.h ----

// PORT: the platform, CPU, SIMD and compiler detection macros, B3_AVX2_BEGIN/END and the Tracy
// zone macros are not ported. The SIMD lane width lives in simd_scalar.go (milestone 2).

// debugBuild is B3_DEBUG: 1 unless NDEBUG. Tied to assertions (box3d_noassert is the NDEBUG build).
const debugBuild = assertEnabled

// atomicInt is b3AtomicInt.
// PORT: the atomic structs become sync/atomic types; platform.go keeps upstream's helpers.
type atomicInt = atomic.Int32

// atomicU32 is b3AtomicU32.
type atomicU32 = atomic.Uint32

// atomicI64 is b3AtomicI64. 64-bit atomic wants 8-byte alignment, which atomic.Int64 guarantees.
type atomicI64 = atomic.Int64

// alignment is B3_ALIGNMENT: minimum memory alignment used for all allocations.
const alignment = 32

// PORT: B3_RESTRICT, B3_ARRAY_COUNT (len) and B3_UNUSED are dropped.

// secretCookie is B3_SECRET_COOKIE. Use to validate definitions. Do not take my cookie.
const secretCookie = 1152023

// checkDef is B3_CHECK_DEF( DEF ) and B3_CHECK_JOINT_DEF( DEF ): pass DEF->internalValue
// (or DEF->base.internalValue).
func checkDef(internalValue int32) {
	assert(internalValue == secretCookie, "DEF->internalValue == B3_SECRET_COOKIE")
}

// PORT: B3_ALLOC, B3_FREE, b3Alloc, b3AllocZero, b3Free, b3GrowAlloc, b3GrowAllocZeroed,
// B3_GROW and B3_GROW_ZERO are not ported: memory comes from make/append. See LOOSE_ENDS.md.

// log is b3Log, declared below with core.c.

// mutex, semaphore and thread (b3Mutex, b3Semaphore, b3Thread) are implemented in timer.go.

// PORT: b3StrCpy is not ported (unused upstream, Go strings are values).

// hash64NonZero is declared here and implemented below.

// ---- core.c ----

// PORT: _Static_assert( B3_MAX_MANIFOLD_POINTS >= 4 ) is the compile-time check below.
var _ = [MaxManifoldPoints - 4]struct{}{}

// This allows the user to change the length units at runtime
var lengthUnitsPerMeter float32 = 1.0

// SetLengthUnitsPerMeter sets the length units. Box3D bases all length units on meters, but you
// may need different units for your game. This should be done at application startup and only
// modified once. Default value is 1.
// Warning: This must be modified before any calls to Box3D
func SetLengthUnitsPerMeter(lengthUnits float32) {
	assert(IsValidFloat(lengthUnits) && lengthUnits > 0.0, "b3IsValidFloat( lengthUnits ) && lengthUnits > 0.0f")
	lengthUnitsPerMeter = lengthUnits
}

// GetLengthUnitsPerMeter gets the current length units per meter.
func GetLengthUnitsPerMeter() float32 {
	return lengthUnitsPerMeter
}

func defaultAssertFcn(condition string, fileName string, lineNumber int) int {
	fmt.Printf("BOX3D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber)

	// return non-zero to break to debugger
	return 1
}

var assertHandler AssertFcn = defaultAssertFcn

// SetAssertFcn overrides the default assert callback.
//
//	assertFcn a non-null assert callback
func SetAssertFcn(assertFcn AssertFcn) {
	assert(assertFcn != nil, "assertFcn != NULL")
	assertHandler = assertFcn
}

// internalAssert is the internal assertion handler. Allows for host intervention.
func internalAssert(condition string, fileName string, lineNumber int) int {
	result := assertHandler(condition, fileName, lineNumber)
	if result != 0 {
		breakpoint(condition, fileName, lineNumber)
	}
	return result
}

func defaultLogFcn(message string) {
	fmt.Printf("Box3D: %s\n", message)
}

var logHandler LogFcn = defaultLogFcn

// SetLogFcn overrides the default logging callback.
func SetLogFcn(logFcn LogFcn) {
	assert(logFcn != nil, "logFcn != NULL")
	logHandler = logFcn
}

// log is b3Log.
// PORT: upstream formats into a 512 byte buffer with vsnprintf; Go does not truncate.
func log(format string, args ...any) {
	buffer := fmt.Sprintf(format, args...)
	logHandler(buffer)
}

// GetVersion gets the current version of Box3D.
func GetVersion() Version {
	return Version{0, 2, 0}
}

// IsDoublePrecision returns true if the library was built with BOX3D_DOUBLE_PRECISION (large world mode).
// PORT: always false; double precision is not ported.
func IsDoublePrecision() bool {
	return false
}

// GetMaxManifoldPoints returns the value of B3_MAX_MANIFOLD_POINTS the library was compiled with.
// Useful for an ABI test.
func GetMaxManifoldPoints() int {
	return MaxManifoldPoints
}

// PORT: b3SetAllocator, b3Alloc, b3Free, b3GrowAlloc, b3GrowAllocZeroed and b3AllocZero are not
// ported. byteCount is kept so GetByteCount has upstream's signature; nothing adds to it yet.
var byteCount atomicI64

// GetByteCount returns the total bytes allocated by Box3D.
// PORT: always 0 (no custom allocator); see LOOSE_ENDS.md.
func GetByteCount() int64 {
	return atomicLoadI64(&byteCount)
}

// hash64NonZero is b3Hash64NonZero.
func hash64NonZero(bytes []byte) uint64 {
	n := len(bytes)
	if n <= 0 {
		return 1
	}

	h := rapidhash(bytes[:n])
	if h == 0 {
		return 1
	}
	return h
}
