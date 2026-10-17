// Port of src/id_pool.h and src/id_pool.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- id_pool.h ----

// idPool is b3IdPool.
type idPool struct {
	freeArray []int32
	nextIndex int32
}

func getIdCount(pool *idPool) int {
	return int(pool.nextIndex) - len(pool.freeArray)
}

func getIdCapacity(pool *idPool) int {
	return int(pool.nextIndex)
}

func getIdBytes(pool *idPool) int {
	return arrayByteCount(pool.freeArray)
}

// ---- id_pool.c ----

func createIdPool() idPool {
	var pool idPool
	arrayReserve(&pool.freeArray, 32)
	return pool
}

func destroyIdPool(pool *idPool) {
	arrayDestroy(&pool.freeArray)
	*pool = idPool{}
}

func allocId(pool *idPool) int {
	count := len(pool.freeArray)
	if count > 0 {
		id := arrayPop(&pool.freeArray)
		return int(id)
	}

	id := pool.nextIndex
	pool.nextIndex += 1
	return int(id)
}

func freeId(pool *idPool, id int) {
	assert(pool.nextIndex > 0, "pool->nextIndex > 0")
	assert(0 <= id && id < int(pool.nextIndex), "0 <= id && id < pool->nextIndex")

	// todo does not work with assertion above
	// should probably be `id == pool->nextIndex - 1`
	if id == int(pool.nextIndex) {
		pool.nextIndex -= 1
		return
	}

	arrayPush(&pool.freeArray, int32(id))
}

func validateFreeId(pool *idPool, id int) {
	if validateEnabled {
		freeCount := len(pool.freeArray)
		for i := 0; i < freeCount; i++ {
			if int(pool.freeArray[i]) == id {
				return
			}
		}

		assert(false, "0")
	}
}
