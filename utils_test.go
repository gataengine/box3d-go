// Port of shared/utils.h and shared/utils.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "runtime"

// PORT: shared/ is a test helper library upstream. These helpers live in a _test.go file of
// package box3d because the upstream tests that use them are internal tests.

const (
	randLimit = 32767 // RAND_LIMIT
	randSeed  = 12345 // RAND_SEED
)

// Global seed for simple random number generator.
var gRandomSeed uint32 = randSeed

func getNumberOfCores() int {
	return runtime.NumCPU()
}

// randomInt is a simple random number generator. Using this instead of rand() for cross platform determinism.
func randomInt() int {
	// XorShift32 algorithm
	x := gRandomSeed
	x ^= x << 13
	x ^= x >> 17
	x ^= x << 5
	gRandomSeed = x

	// Map the 32-bit value to the range 0 to RAND_LIMIT
	return int(x % (randLimit + 1))
}

// randomIntRange returns a random integer in range [lo, hi]
// PORT: upstream returns float (a quirk of the C signature); the value is the same integer.
func randomIntRange(lo, hi int) float32 {
	return float32(lo + randomInt()%(hi-lo+1))
}

// randomFloat returns a random number in range [-1,1]
func randomFloat() float32 {
	r := float32(randomInt() & randLimit)
	r /= randLimit
	r = float32(2.0*r) - 1.0
	return r
}

// randomFloatRange returns a random floating point number in range [lo, hi]
func randomFloatRange(lo, hi float32) float32 {
	r := float32(randomInt() & randLimit)
	r /= randLimit
	r = float32((hi-lo)*r) + lo
	return r
}

// randomVec3 returns a random vector with coordinates in range [lo, hi]
func randomVec3(lo, hi Vec3) Vec3 {
	var v Vec3
	v.X = randomFloatRange(lo.X, hi.X)
	v.Y = randomFloatRange(lo.Y, hi.Y)
	v.Z = randomFloatRange(lo.Z, hi.Z)
	return v
}

// randomPos returns a random world position with coordinates in range [lo, hi]
func randomPos(lo, hi Vec3) Pos {
	var v Pos
	v.X = randomFloatRange(lo.X, hi.X)
	v.Y = randomFloatRange(lo.Y, hi.Y)
	v.Z = randomFloatRange(lo.Z, hi.Z)
	return v
}

func randomVec3Uniform(lo, hi float32) Vec3 {
	var v Vec3
	v.X = randomFloatRange(lo, hi)
	v.Y = randomFloatRange(lo, hi)
	v.Z = randomFloatRange(lo, hi)
	return v
}

func randomUnitVector() Vec3 {
	// Generate uniformly distributed random quaternion using Shoemake's method
	// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992

	u1 := randomFloatRange(0.0, 1.0)
	u2 := randomFloatRange(0.0, 2.0*Pi)
	u3 := randomFloatRange(0.0, 2.0*Pi)

	sqrt1MinusU1 := sqrtf(1.0 - u1)
	sqrtU1 := sqrtf(u1)

	cs2 := ComputeCosSin(u2)
	cs3 := ComputeCosSin(u3)

	var v Vec3
	v.X = sqrt1MinusU1 * cs2.Sine
	v.Y = sqrt1MinusU1 * cs2.Cosine
	v.Z = sqrtU1 * cs3.Sine

	return v
}

func randomQuat() Quat {
	// Generate uniformly distributed random quaternion using Shoemake's method
	// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992

	u1 := randomFloatRange(0.0, 1.0)
	u2 := randomFloatRange(0.0, 2.0*Pi)
	u3 := randomFloatRange(0.0, 2.0*Pi)

	sqrt1MinusU1 := sqrtf(1.0 - u1)
	sqrtU1 := sqrtf(u1)

	cs2 := ComputeCosSin(u2)
	cs3 := ComputeCosSin(u3)

	var q Quat
	q.V.X = sqrt1MinusU1 * cs2.Sine
	q.V.Y = sqrt1MinusU1 * cs2.Cosine
	q.V.Z = sqrtU1 * cs3.Sine
	q.S = sqrtU1 * cs3.Cosine

	return q
}
