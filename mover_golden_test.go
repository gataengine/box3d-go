// Go-only test: no upstream counterpart. Compares SolvePlanes (delta, iteration count and every
// plane push, including soft push limits) and ClipVector against the pinned upstream C sources
// (efca0c184d40fd6a82016983f1601c4d23965867) compiled with -ffp-contract=off, bit for bit. See
// testdata/mover_golden_gen.c; this file mirrors it line by line.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"os"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

type moverGoldenGen struct {
	seed uint32
	sb   strings.Builder
}

func (g *moverGoldenGen) nextRandom() uint32 {
	g.seed = 1664525*g.seed + 1013904223
	return g.seed
}

func (g *moverGoldenGen) randomFloat(lo, hi float32) float32 {
	t := float32(g.nextRandom()>>8) / float32(1<<24)
	return mulAdd(lo, t, hi-lo)
}

func (g *moverGoldenGen) randomVec(lo, hi float32) Vec3 {
	var v Vec3
	v.X = g.randomFloat(lo, hi)
	v.Y = g.randomFloat(lo, hi)
	v.Z = g.randomFloat(lo, hi)
	return v
}

func (g *moverGoldenGen) printf(format string, args ...any) { fmt.Fprintf(&g.sb, format, args...) }
func (g *moverGoldenGen) pf(f float32)                      { g.printf(" %08x", math.Float32bits(f)) }
func (g *moverGoldenGen) pv(v Vec3)                         { g.pf(v.X); g.pf(v.Y); g.pf(v.Z) }

func (g *moverGoldenGen) run() string {
	g.seed = 4321

	var planes [8]CollisionPlane

	for i := 0; i < 200; i++ {
		count := int(g.nextRandom() % 9)
		for j := 0; j < count; j++ {
			n := g.randomVec(-1.0, 1.0)
			// Mostly upward facing, like ground and walls around a character
			if j == 0 {
				n.Y += 1.5
			} else {
				n.Y += 0.25
			}
			planes[j].Plane.Normal = Normalize(n)
			planes[j].Plane.Offset = g.randomFloat(-0.5, 0.5)
			r := g.nextRandom()
			if (r & 3) == 0 {
				planes[j].PushLimit = g.randomFloat(0.0, 0.2)
			} else {
				planes[j].PushLimit = fltMax
			}
			planes[j].Push = 123.0
			planes[j].ClipVelocity = (r & 4) != 0
		}

		target := g.randomVec(-2.0, 2.0)
		if i%17 == 0 {
			target = MulSV(40.0, target)
		}

		result := SolvePlanes(target, planes[:count])

		g.printf("%d |", i)
		g.pv(result.Delta)
		g.printf(" %d %d", result.IterationCount, count)
		for j := 0; j < count; j++ {
			g.pf(planes[j].Push)
		}

		velocity := g.randomVec(-5.0, 5.0)
		g.pv(ClipVector(velocity, planes[:count]))
		g.printf("\n")
	}

	return g.sb.String()
}

var _ = Describe("MoverGoldenTest", func() {
	It("MatchesC", func() {
		want, err := os.ReadFile("testdata/mover_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		var g moverGoldenGen
		got := g.run()

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		Expect(gotLines).To(HaveLen(len(wantLines)))
		for i := range wantLines {
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
	})
})
