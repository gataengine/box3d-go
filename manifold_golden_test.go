// Go-only test: no upstream counterpart. Compares the collide functions of convex_manifold.c and
// triangle_manifold.c (sphere, capsule, hull and triangle pairs, SAT cache reuse, the manual
// feature modes) and b3ComputeSeparatingAxis against the pinned upstream C sources
// (efca0c184d40fd6a82016983f1601c4d23965867) compiled with -ffp-contract=off, bit for bit. The
// C output is identical for the NEON, SSE2 and BOX3D_DISABLE_SIMD builds; the Go port must match it
// with both variants of the wide SAT edge loop (simdNone true and false). See
// testdata/manifold_golden_gen.c; this file mirrors it line by line.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"os"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

type manifoldGoldenGen struct {
	hullGoldenGen
	points [8]LocalManifoldPoint
}

func (g *manifoldGoldenGen) fresh() LocalManifold {
	g.points = [8]LocalManifoldPoint{}
	var m LocalManifold
	m.Points = g.points[:]
	return m
}

func (g *manifoldGoldenGen) pm(m *LocalManifold) {
	g.printf(" %d", m.PointCount)
	g.pv(m.Normal)
	for i := 0; i < int(m.PointCount); i++ {
		g.pv(m.Points[i].Point)
		g.pf(m.Points[i].Separation)
		g.printf(" %08x", makeFeatureId(m.Points[i].Pair))
	}
	g.printf(" %d", int(m.Feature))
	g.pf(m.SquaredDistance)
}

func (g *manifoldGoldenGen) psat(c *SATCache) {
	g.pf(c.Separation)
	g.printf(" %d %d %d %d", c.Type, c.IndexA, c.IndexB, c.Hit)
}

func (g *manifoldGoldenGen) psc(c *SimplexCache) {
	g.printf(" %d", c.Count)
	for i := 0; i < int(c.Count); i++ {
		g.printf(" %d %d", c.IndexA[i], c.IndexB[i])
	}
}

func (g *manifoldGoldenGen) pa(a separatingAxis) {
	g.pv(a.normal)
	g.pf(a.separation)
	g.printf(" %d %d %d", a.indexA, a.indexB, int(a.typ))
}

func (g *manifoldGoldenGen) paq(q axisQuery) {
	g.pa(q.faceA)
	g.pa(q.faceB)
	g.pa(q.edge)
	g.printf(" %d", int(q.separatedFeature))
}

func (g *manifoldGoldenGen) nudge(xf Transform) Transform {
	r := xf
	r.P = Add(xf.P, g.rv3(0.02, 0.02, 0.02, 0.0, 0.0, 0.0))
	return r
}

const manifoldGoldenPoolCount = 8

func (g *manifoldGoldenGen) run() string {
	g.seed = 4242

	var boxes [3]BoxHull
	var pool [manifoldGoldenPoolCount]*HullData

	boxes[0] = MakeBoxHull(0.5, 0.5, 0.5)
	boxes[1] = MakeBoxHull(1.5, 0.1, 0.1)
	{
		var place Transform
		place.P = g.rv3(0.3, 0.3, 0.3, 0.0, 0.0, 0.0)
		place.Q = g.rq()
		boxes[2] = MakeTransformedBoxHull(0.4, 0.6, 0.5, place)
	}
	pool[0] = &boxes[0].Base
	pool[1] = &boxes[1].Base
	pool[2] = &boxes[2].Base
	pool[3] = CreateCylinder(1.0, 0.5, 0.25, 12)
	pool[4] = CreateCone(1.0, 0.6, 0.2, 9)
	pool[5] = CreateRock(0.7)
	for h := 6; h < manifoldGoldenPoolCount; h++ {
		var points [40]Vec3
		for i := range 40 {
			points[i] = g.rv3(0.8, 0.6, 0.5, 0.0, 0.0, 0.0)
		}
		maxCount := 64
		if h == 6 {
			maxCount = 16
		}
		pool[h] = CreateHull(points[:], 40, maxCount)
	}

	g.printf("pool |")
	for h := range manifoldGoldenPoolCount {
		g.printf(" %d %d %d %016x", pool[h].VertexCount, pool[h].EdgeCount, pool[h].FaceCount, pool[h].Hash)
	}
	g.printf("\n")

	scales := []float32{0.4, 0.8, 1.3}

	for k := range 120 {
		s := scales[k%3]

		var sphereA Sphere
		sphereA.Center = g.rv3(0.3, 0.3, 0.3, 0.0, 0.0, 0.0)
		sphereA.Radius = mulAdd(0.35, 0.25, g.rnd())
		var sphereB Sphere
		sphereB.Center = g.rv3(0.3, 0.3, 0.3, 0.0, 0.0, 0.0)
		sphereB.Radius = mulAdd(0.35, 0.25, g.rnd())

		var capsuleA Capsule
		capsuleA.Center1 = g.rv3(0.6, 0.6, 0.6, 0.0, 0.0, 0.0)
		capsuleA.Center2 = g.rv3(0.6, 0.6, 0.6, 0.0, 0.0, 0.0)
		capsuleA.Radius = mulAdd(0.2, 0.15, g.rnd())
		var capsuleB Capsule
		capsuleB.Center1 = g.rv3(0.6, 0.6, 0.6, 0.0, 0.0, 0.0)
		if k%4 == 1 {
			// Nearly parallel to capsule A.
			jitter := g.rv3(0.01, 0.01, 0.01, 0.0, 0.0, 0.0)
			capsuleB.Center2 = Add(capsuleB.Center1, Add(Sub(capsuleA.Center2, capsuleA.Center1), jitter))
		} else {
			capsuleB.Center2 = g.rv3(0.6, 0.6, 0.6, 0.0, 0.0, 0.0)
		}
		capsuleB.Radius = mulAdd(0.2, 0.15, g.rnd())

		hullA := pool[k%manifoldGoldenPoolCount]
		hullB := pool[(3*k+1)%manifoldGoldenPoolCount]

		var xf Transform
		xf.P = g.rv3(s, s, s, 0.0, 0.0, 0.0)
		xf.Q = g.rq()
		if k%5 == 2 {
			// Aligned stacks drive face contacts with many clip points.
			xf.Q = QuatIdentity
			xf.P.X = float32(0.1 * xf.P.X)
			xf.P.Z = float32(0.1 * xf.P.Z)
		}

		g.printf("pair %d | ", k)

		m := g.fresh()
		CollideSpheres(&m, 8, &sphereA, &sphereB, xf)
		g.pm(&m)

		m = g.fresh()
		CollideCapsuleAndSphere(&m, 8, &capsuleA, &sphereB, xf)
		g.pm(&m)

		{
			cache := SimplexCache{}
			m = g.fresh()
			CollideHullAndSphere(&m, 8, hullA, &sphereB, xf, &cache)
			g.pm(&m)
			g.psc(&cache)
			xf2 := g.nudge(xf)
			m = g.fresh()
			CollideHullAndSphere(&m, 8, hullA, &sphereB, xf2, &cache)
			g.pm(&m)
			g.psc(&cache)
		}

		m = g.fresh()
		CollideCapsules(&m, 8, &capsuleA, &capsuleB, xf)
		g.pm(&m)

		{
			cache := SimplexCache{}
			m = g.fresh()
			CollideHullAndCapsule(&m, 8, hullA, &capsuleB, xf, &cache)
			g.pm(&m)
			g.psc(&cache)
			xf2 := g.nudge(xf)
			m = g.fresh()
			CollideHullAndCapsule(&m, 8, hullA, &capsuleB, xf2, &cache)
			g.pm(&m)
			g.psc(&cache)
		}
		g.printf("\n")

		g.printf("hulls %d |", k)
		{
			cache := SATCache{}
			m = g.fresh()
			CollideHulls(&m, 8, hullA, hullB, xf, &cache)
			g.pm(&m)
			g.psat(&cache)

			xf2 := g.nudge(xf)
			m = g.fresh()
			CollideHulls(&m, 8, hullA, hullB, xf2, &cache)
			g.pm(&m)
			g.psat(&cache)

			for manual := ManualFaceAxisA; manual <= ManualEdgePairAxis; manual++ {
				manualCache := SATCache{}
				manualCache.Type = uint8(manual)
				m = g.fresh()
				CollideHulls(&m, 8, hullA, hullB, xf, &manualCache)
				g.pm(&m)
				g.psat(&manualCache)
			}

			g.paq(computeSeparatingAxis(hullA, hullB, xf, true))
			g.paq(computeSeparatingAxis(hullA, hullB, xf, false))
		}
		g.printf("\n")

		g.printf("triangle %d |", k)
		{
			var triangle [3]Vec3
			triangle[0] = g.rv3(1.5, 0.2, 1.5, 0.0, float32(-0.5*s), 0.0)
			triangle[1] = g.rv3(1.5, 0.2, 1.5, 0.0, float32(-0.5*s), 0.0)
			triangle[2] = g.rv3(1.5, 0.2, 1.5, 0.0, float32(-0.5*s), 0.0)
			if k%4 != 3 {
				// Face up (toward the shapes) except for every fourth triangle, which tests the back side cull.
				n := Cross(Sub(triangle[1], triangle[0]), Sub(triangle[2], triangle[0]))
				if n.Y < 0.0 {
					triangle[1], triangle[2] = triangle[2], triangle[1]
				}
			}
			if k%3 == 0 {
				q := g.rq()
				triangle[0] = RotateVector(q, triangle[0])
				triangle[1] = RotateVector(q, triangle[1])
				triangle[2] = RotateVector(q, triangle[2])
			}
			g.pv(triangle[0])
			g.pv(triangle[1])
			g.pv(triangle[2])

			m = g.fresh()
			CollideTriangleAndSphere(&m, 8, triangle[:], &sphereB)
			g.pm(&m)

			simplexCache := SimplexCache{}
			m = g.fresh()
			CollideTriangleAndCapsule(&m, 8, triangle[:], &capsuleB, &simplexCache)
			g.pm(&m)
			g.psc(&simplexCache)

			enableSpeculative := (k & 1) == 0
			flags := k % 8
			cache := SATCache{}
			m = g.fresh()
			CollideTriangleAndHull(&m, 8, triangle[0], triangle[1], triangle[2], flags, hullB, &cache, enableSpeculative)
			g.pm(&m)
			g.psat(&cache)
			touching := m.PointCount

			jitter := g.rv3(0.01, 0.01, 0.01, 0.0, 0.0, 0.0)
			m = g.fresh()
			CollideTriangleAndHull(&m, 8, Add(triangle[0], jitter), Add(triangle[1], jitter),
				Add(triangle[2], jitter), flags, hullB, &cache, enableSpeculative)
			g.pm(&m)
			g.psat(&cache)

			// The manual modes skip the separation checks, and the edge builder validates that the pair
			// is within the speculative band, so only touching pairs take them.
			for manual := ManualFaceAxisA; touching > 0 && manual <= ManualEdgePairAxis; manual++ {
				manualCache := SATCache{}
				manualCache.Type = uint8(manual)
				m = g.fresh()
				CollideTriangleAndHull(&m, 8, triangle[0], triangle[1], triangle[2], flags, hullB, &manualCache,
					enableSpeculative)
				g.pm(&m)
				g.psat(&manualCache)
			}
		}
		g.printf("\n")
	}

	return g.sb.String()
}

var _ = Describe("ManifoldGoldenTest", func() {
	check := func() {
		// Hull limit messages are silenced like the generator does.
		SetLogFcn(func(string) {})
		defer SetLogFcn(defaultLogFcn)

		want, err := os.ReadFile("testdata/manifold_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		var g manifoldGoldenGen
		got := g.run()

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		Expect(gotLines).To(HaveLen(len(wantLines)))
		for i := range wantLines {
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
	}

	It("MatchesC", func() {
		check()
	})

	// The lane variant of the SAT edge loop (the path upstream's NEON/SSE2/AVX2 builds take) must
	// give the same bits as the B3_SIMD_NONE variant the scalar backend uses.
	It("MatchesCWithLaneEdgeLoop", func() {
		saved := simdNone
		simdNone = false
		defer func() { simdNone = saved }()
		check()
	})
})
