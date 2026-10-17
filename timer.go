// Port of src/timer.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"runtime"
	"sync"
	"time"
)

// PORT: upstream has one implementation per platform (Windows, Linux/Emscripten, Apple, other).
// Go's runtime is the platform layer: ticks are monotonic nanoseconds since package
// initialization, threads are goroutines, and the mutex and semaphore wrap sync primitives.

const nameLength = 16

// ticksOrigin anchors GetTicks. time.Since uses the monotonic clock.
var ticksOrigin = time.Now()

// GetTicks gets the absolute number of system ticks. The value is platform specific.
// PORT: nanoseconds (the Linux implementation's unit).
func GetTicks() uint64 {
	return uint64(time.Since(ticksOrigin).Nanoseconds())
}

// GetMilliseconds gets the milliseconds passed from an initial tick value.
func GetMilliseconds(ticks uint64) float32 {
	ticksNow := GetTicks()
	return float32(float64(ticksNow-ticks) / 1000000.0)
}

// GetMillisecondsAndReset gets the milliseconds passed from an initial tick value and resets it.
func GetMillisecondsAndReset(ticks *uint64) float32 {
	ticksNow := GetTicks()
	ms := float32(float64(ticksNow-*ticks) / 1000000.0)
	*ticks = ticksNow
	return ms
}

// Yield to be used in a busy loop.
func Yield() {
	runtime.Gosched()
}

// Sleep the current thread for a number of milliseconds.
func Sleep(milliseconds int) {
	time.Sleep(time.Duration(milliseconds) * time.Millisecond)
}

// mutex is b3Mutex.
type mutex struct {
	mtx sync.Mutex
}

func createMutex() *mutex {
	m := &mutex{}
	return m
}

func destroyMutex(m *mutex) {
	// PORT: nothing to release; the GC owns the mutex.
}

func lockMutex(m *mutex) {
	m.mtx.Lock()
}

func unlockMutex(m *mutex) {
	m.mtx.Unlock()
}

// semaphore is b3Semaphore: a counting semaphore.
type semaphore struct {
	mtx   sync.Mutex
	cond  sync.Cond
	count int
}

func createSemaphore(initCount int) *semaphore {
	s := &semaphore{count: initCount}
	s.cond.L = &s.mtx
	return s
}

func destroySemaphore(s *semaphore) {
	// PORT: nothing to release; the GC owns the semaphore.
}

func waitSemaphore(s *semaphore) {
	s.mtx.Lock()
	for s.count == 0 {
		s.cond.Wait()
	}
	s.count--
	s.mtx.Unlock()
}

func signalSemaphore(s *semaphore) {
	s.mtx.Lock()
	s.count++
	s.mtx.Unlock()
	s.cond.Signal()
}

// threadFunction is b3ThreadFunction.
type threadFunction func(context any)

// thread is b3Thread.
type thread struct {
	done     chan struct{}
	function threadFunction
	context  any
	name     string
}

// threadStart is b3ThreadStart.
// PORT: goroutines cannot be named, so the name is only stored.
func threadStart(t *thread) {
	defer close(t.done)
	t.function(t.context)
}

// createThread is b3CreateThread. Name may be empty, otherwise it is copied.
func createThread(function threadFunction, context any, name string) *thread {
	t := &thread{done: make(chan struct{}), function: function, context: context}
	if len(name) > nameLength-1 {
		name = name[:nameLength-1]
	}
	t.name = name
	go threadStart(t)
	return t
}

// joinThread is b3JoinThread.
func joinThread(t *thread) {
	<-t.done
	*t = thread{}
}

// Hash is a djb2 hash, folded 8 bytes per iteration to shorten the dependency chain.
// Little endian loads keep the hash value identical across endianness (preserving cross-platform determinism).
// Equivalent to byte-wise djb2 only in spirit; values differ from the original recurrence.
// Simple djb2 hash function for determinism testing. Start with HashInit.
func Hash(hash uint32, data []byte) uint32 {
	count := len(data)
	result := hash
	i := 0

	for i+8 <= count {
		word := binary.LittleEndian.Uint64(data[i:])
		result = (result << 5) + result + uint32(word)
		result = (result << 5) + result + uint32(word>>32)
		i += 8
	}

	for i < count {
		result = (result << 5) + result + uint32(data[i])
		i++
	}

	return result
}
