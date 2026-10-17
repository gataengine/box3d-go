// Port of test/test_table.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

const (
	setSpan        = 317                             // SET_SPAN
	tableItemCount = (setSpan*setSpan - setSpan) / 2 // ITEM_COUNT
)

var _ = Describe("TableTest", func() {
	It("TableTest", func() {
		power := boundingPowerOf2(3008)
		ensure(power == 12)

		nextPowerOf2 := roundUpPowerOf2(3008)
		ensure(nextPowerOf2 == (1 << power))

		const N int32 = setSpan
		const itemCount uint32 = tableItemCount
		var removed [tableItemCount]bool

		for iter := int32(0); iter < 1; iter++ {
			set := createSet(16)

			// Fill set
			for i := int32(0); i < N; i++ {
				for j := i + 1; j < N; j++ {
					key := shapePairKey(int(i), int(j), 0)
					addKey(&set, key)
				}
			}

			ensure(set.count == itemCount)

			// Remove a portion of the set
			var k int32
			var removeCount uint32
			for i := int32(0); i < N; i++ {
				for j := i + 1; j < N; j++ {
					if j == i+1 {
						key := shapePairKey(int(i), int(j), 0)
						removeKey(&set, key)
						removed[k] = true
						k++
						removeCount += 1
					} else {
						removed[k] = false
						k++
					}
				}
			}

			ensure(set.count == (itemCount - removeCount))

			if debugBuild {
				atomicStoreInt(&probeCount, 0)
			}

			// Test key search
			// ~5ns per search on an AMD 7950x
			ticks := GetTicks()

			k = 0
			for i := int32(0); i < N; i++ {
				for j := i + 1; j < N; j++ {
					key := shapePairKey(int(j), int(i), 0)
					ensure(containsKey(&set, key) || removed[k])
					k += 1
				}
			}

			ms := GetMilliseconds(ticks)
			GinkgoWriter.Printf("set: count = %d, b3ContainsKey = %.5f ms, ave = %.5f us\n", itemCount, ms, 1000.0*ms/float32(itemCount))

			if debugBuild {
				probeCount := atomicLoadInt(&probeCount)
				aveProbeCount := float32(probeCount) / float32(itemCount)
				GinkgoWriter.Printf("item count = %d, probe count = %d, ave probe count %.2f\n", itemCount, probeCount, aveProbeCount)
			}

			// Remove all keys from set
			for i := int32(0); i < N; i++ {
				for j := i + 1; j < N; j++ {
					key := shapePairKey(int(i), int(j), 0)
					removeKey(&set, key)
				}
			}

			ensure(set.count == 0)

			destroySet(&set)
		}
	})
})
