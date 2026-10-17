// Go-only test: no upstream counterpart. Checks the Go replacements for the platform layer in
// src/timer.c and the reference value of b3Hash computed from the pinned upstream C source.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

var _ = Describe("Timer", func() {
	It("measures elapsed time", func() {
		ticks := GetTicks()
		Sleep(2)
		ms := GetMillisecondsAndReset(&ticks)
		ensure(ms >= 1.0)
		ensure(GetMilliseconds(ticks) < ms+1000.0)
	})

	It("runs threads, semaphores and mutexes", func() {
		s := createSemaphore(0)
		m := createMutex()
		counter := 0
		var threads [4]*thread
		for i := range threads {
			threads[i] = createThread(func(context any) {
				lockMutex(m)
				counter += context.(int)
				unlockMutex(m)
				signalSemaphore(s)
			}, i+1, "worker")
		}
		for range threads {
			waitSemaphore(s)
		}
		for _, t := range threads {
			joinThread(t)
		}
		ensure(counter == 10)
		destroySemaphore(s)
		destroyMutex(m)
	})

	It("matches the C hash", func() {
		var data [37]byte
		for i := range data {
			data[i] = byte(i*29 + 3)
		}
		ensure(Hash(HashInit, data[:]) == hashReference)
	})
})

const hashReference uint32 = 0xbd177bbe
