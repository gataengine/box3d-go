// Port of test/test_container.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

type containerFoo struct {
	a int32
	b float32
}

type containerBar struct {
	a []int32
}

var _ = Describe("ContainerTest", func() {
	It("TestCreateDestroy", func() {
		var a []int32
		arrayCreate(&a)
		arrayDestroy(&a)
	})

	It("TestAccess", func() {
		var a []int32
		arrayCreate(&a)
		arrayPush(&a, 42)
		element := arrayGet(a, 0)
		ensure(*element == 42)
		arrayDestroy(&a)
	})

	It("TestIteration", func() {
		var a []int32
		arrayPush(&a, 1)
		arrayPush(&a, 2)
		arrayPush(&a, 3)

		var sum int32
		for i := 0; i < len(a); i++ {
			sum += a[i]
		}

		ensure(sum == 6)
		arrayDestroy(&a)
	})

	It("TestArrayOfStruct", func() {
		var a []containerFoo
		arrayCreate(&a)
		arrayPush(&a, containerFoo{a: 1, b: 5.0})
		arrayPush(&a, containerFoo{a: 2, b: 6.0})
		arrayPush(&a, containerFoo{a: 3, b: 7.0})

		var sum1 int32
		var sum2 float32
		for i := 0; i < len(a); i++ {
			sum1 += a[i].a
			sum2 += a[i].b
		}

		ensure(sum1 == 6)
		ensure(sum2 == 18.0)

		arrayDestroy(&a)
	})

	It("TestStructWithArray", func() {
		var a containerBar
		arrayCreate(&a.a)
		arrayPush(&a.a, 1)
		arrayPush(&a.a, 2)
		arrayPush(&a.a, 3)

		var sum1 int32
		for i := 0; i < len(a.a); i++ {
			sum1 += a.a[i]
		}

		ensure(sum1 == 6)

		arrayDestroy(&a.a)
	})

	It("TestArrayEmplace", func() {
		var a []int32

		n := 100
		for i := 0; i < n; i++ {
			j := arrayEmplace(&a)
			*j = int32(i)
		}

		sum := 0
		for i := 0; i < len(a); i++ {
			sum += int(a[i])
		}

		ensure(sum == n*(n-1)/2)

		arrayDestroy(&a)
	})

	It("TestArrayRemove", func() {
		var a []int16

		n := 100
		arrayReserve(&a, n)
		ensure(cap(a) == n && len(a) == 0)

		for i := 0; i < n; i++ {
			arrayPush(&a, int16(i))
		}

		sum := 0
		for i := 0; i < n; i++ {
			value := arrayGet(a, 0)
			sum += int(*value)
			arrayRemoveSwap(&a, 0)
		}

		ensure(sum == n*(n-1)/2)

		arrayDestroy(&a)
	})

	It("TestArrayPop", func() {
		var a []int32

		n := 10
		arrayResize(&a, n)
		ensure(cap(a) == n && len(a) == n)

		for i := 0; i < n; i++ {
			a[i] = int32(i)
		}

		sum := 0
		for len(a) > 0 {
			sum += int(arrayPop(&a))
		}

		ensure(sum == n*(n-1)/2)

		arrayDestroy(&a)
	})
})
