// Go-only test: upstream has no test for verstable.h.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

var _ = Describe("VerstableTest", func() {
	It("MatchesMap", func() {
		var t verstable[int, int]
		// A poor hash forces long probe runs and wrap-around.
		t.init(func(k int) uint64 { return uint64(k % 7) }, func(a, b int) bool { return a == b })
		t.reserve(10)

		ref := map[int]int{}
		seed := uint32(12345)
		for range 5000 {
			seed ^= seed << 13
			seed ^= seed >> 17
			seed ^= seed << 5
			key := int(seed % 97)
			switch seed % 3 {
			case 0:
				itr := t.getOrInsert(key, key*10)
				if v, ok := ref[key]; ok {
					ensure(itr.data.val == v)
				} else {
					ref[key] = key * 10
				}
			case 1:
				t.insert(key, key*20)
				ref[key] = key * 20
			case 2:
				itr := t.get(key)
				_, ok := ref[key]
				ensure(t.isEnd(itr) == !ok)
				if ok {
					t.eraseItr(itr)
					delete(ref, key)
				}
			}
			ensure(t.size() == len(ref))
		}

		count := 0
		for itr := t.first(); !t.isEnd(itr); itr = t.next(itr) {
			ensure(ref[itr.data.key] == itr.data.val)
			count += 1
		}
		ensure(count == len(ref))
		ensure(t.bucketCount()&(t.bucketCount()-1) == 0)

		t.cleanup()
		ensure(t.size() == 0 && t.bucketCount() == 0)
	})
})
