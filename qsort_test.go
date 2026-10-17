// Go-only test: no upstream counterpart. QSORT is not stable, so the final order of equal keys
// depends on the exact sequence of LESS and SWAP calls (mesh_contact.c sorts repeated keys).
// This compares the call counts and the payload order against the pinned upstream qsort.h
// (efca0c184d40fd6a82016983f1601c4d23965867). See testdata/qsort_golden_gen.c; this file mirrors
// it line by line.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"os"
	"slices"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const qsortGoldenMaxCount = 5000

type qsortGoldenGen struct {
	seed      uint32
	keys      [qsortGoldenMaxCount]uint64
	payloads  [qsortGoldenMaxCount]uint32
	lessCount int
	swapCount int
	sb        strings.Builder
}

func (g *qsortGoldenGen) nextRandom() uint32 {
	// xorshift32
	x := g.seed
	x ^= x << 13
	x ^= x >> 17
	x ^= x << 5
	g.seed = x
	return x
}

func (g *qsortGoldenGen) fill(count, pattern int) {
	for i := range count {
		var key uint64
		switch pattern {
		case 0:
			key = uint64(g.nextRandom())
		case 1:
			key = uint64(g.nextRandom() & 7)
		case 2:
			key = uint64(i)
		case 3:
			key = uint64(count - i)
		case 4:
			key = 42
		case 5:
			if i < count/2 {
				key = uint64(i)
			} else {
				key = uint64(count - i)
			}
		default:
			hi := uint64(g.nextRandom() & 0x3FFFFF)
			lo := uint64(g.nextRandom() & 0x3FFFFF)
			key = (hi << 42) | (lo << 20) | uint64(g.nextRandom()&3)
		}
		g.keys[i] = key
		g.payloads[i] = uint32(i)
	}
}

func (g *qsortGoldenGen) runCase(count, pattern int) {
	g.fill(count, pattern)
	g.lessCount = 0
	g.swapCount = 0

	qsort(count, func(i, j int) bool {
		g.lessCount += 1
		return g.keys[i] < g.keys[j]
	}, func(i, j int) {
		g.swapCount += 1
		g.keys[i], g.keys[j] = g.keys[j], g.keys[i]
		g.payloads[i], g.payloads[j] = g.payloads[j], g.payloads[i]
	})

	ensure(slices.IsSorted(g.keys[:count]))

	hash := uint32(2166136261)
	for i := range count {
		hash ^= g.payloads[i]
		hash *= 16777619
	}

	fmt.Fprintf(&g.sb, "pattern %d count %d | %d %d %d %08x\n", pattern, count, count, g.lessCount, g.swapCount, hash)
}

func (g *qsortGoldenGen) run() string {
	counts := []int{0, 1, 2, 3, 5, 15, 16, 17, 31, 64, 100, 257, 1000, 5000}
	for pattern := range 7 {
		for _, count := range counts {
			g.runCase(count, pattern)
		}
	}
	return g.sb.String()
}

var _ = Describe("QSortGoldenTest", func() {
	It("MatchesC", func() {
		want, err := os.ReadFile("testdata/qsort_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		g := qsortGoldenGen{seed: 0x9E3779B9}
		got := g.run()

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		Expect(gotLines).To(HaveLen(len(wantLines)))
		for i := range wantLines {
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
	})
})
