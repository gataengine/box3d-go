// Port of src/platform.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "sync/atomic"

// PORT: b3Prefetch is dropped (Go has no prefetch intrinsic). The atomics map onto sync/atomic,
// which is sequentially consistent like the __ATOMIC_SEQ_CST builtins upstream uses.

func atomicStoreInt(a *atomicInt, value int32) {
	a.Store(value)
}

func atomicLoadInt(a *atomicInt) int32 {
	return a.Load()
}

func atomicFetchAddInt(a *atomicInt, increment int32) int32 {
	return a.Add(increment) - increment
}

func atomicCompareExchangeInt(a *atomicInt, expected, desired int32) bool {
	return a.CompareAndSwap(expected, desired)
}

func atomicStoreU32(a *atomicU32, value uint32) {
	a.Store(value)
}

func atomicLoadU32(a *atomicU32) uint32 {
	return a.Load()
}

// atomicLoadU32Raw is a relaxed load on a plain word. Lets a racing reader peek before paying for
// a read modify write.
// PORT: Go has no relaxed load; this is a sequentially consistent load.
func atomicLoadU32Raw(a *uint32) uint32 {
	return atomic.LoadUint32(a)
}

func atomicFetchOrU32(a *uint32, mask uint32) uint32 {
	return atomic.OrUint32(a, mask)
}

func atomicFetchAddI64(a *atomicI64, increment int64) int64 {
	return a.Add(increment) - increment
}

func atomicLoadI64(a *atomicI64) int64 {
	return a.Load()
}
