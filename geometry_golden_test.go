// Go-only test: no upstream counterpart. Compares mesh creation (byte images through their byte
// count and content hash), mesh queries, ray and shape casts and mover collision against the
// pinned upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867) compiled with
// -ffp-contract=off, bit for bit. The C output is identical for the NEON, SSE2,
// BOX3D_DISABLE_SIMD and BOX3D_VALIDATE builds. See testdata/geometry_golden_gen.c; this file
// mirrors it line by line.
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

const soupN = 6 // SOUP_N

type geometryGoldenGen struct {
	hullGoldenGen

	queryHash  uint64
	queryCount int
	queryStop  int

	proxyPoints [4]Vec3

	soupVertices  [soupN*soupN*6 + 9]Vec3
	soupIndices   [soupN*soupN*6 + 9]int32
	soupMaterials [soupN*soupN*2 + 3]uint8
	heights       [(soupN + 1) * (soupN + 1)]float32
}

func (g *geometryGoldenGen) rndInt(n uint32) uint32 {
	g.seed ^= g.seed << 13
	g.seed ^= g.seed >> 17
	g.seed ^= g.seed << 5
	return g.seed % n
}

func (g *geometryGoldenGen) pbox(a AABB) {
	g.pv(a.LowerBound)
	g.pv(a.UpperBound)
}

func (g *geometryGoldenGen) pbool(b bool) {
	if b {
		g.printf(" 1")
	} else {
		g.printf(" 0")
	}
}

func (g *geometryGoldenGen) pco(o CastOutput) {
	g.pbool(o.Hit)
	g.pv(o.Normal)
	g.pv(o.Point)
	g.pf(o.Fraction)
	g.printf(" %d %d %d %d", o.Iterations, o.TriangleIndex, o.ChildIndex, o.MaterialIndex)
}

func (g *geometryGoldenGen) pplanes(planes []PlaneResult, count int) {
	g.printf(" %d", count)
	for i := range count {
		g.pv(planes[i].Plane.Normal)
		g.pf(planes[i].Plane.Offset)
		g.pv(planes[i].Point)
		g.printf(" %d %d %d", planes[i].TriangleIndex, planes[i].ChildIndex, planes[i].MaterialIndex)
	}
}

func (g *geometryGoldenGen) ptri(t triangle) {
	g.pv(t.vertices[0])
	g.pv(t.vertices[1])
	g.pv(t.vertices[2])
	g.printf(" %d %d %d %d", t.i1, t.i2, t.i3, t.flags)
}

func (g *geometryGoldenGen) hashWord(w uint32) {
	for i := range 4 {
		g.queryHash ^= uint64((w >> (8 * i)) & 0xFF)
		g.queryHash *= 0x100000001b3
	}
}

func (g *geometryGoldenGen) meshQueryFcn(a, b, c Vec3, triangleIndex int, context any) bool {
	g.hashWord(uint32(triangleIndex))
	for _, f := range [...]float32{a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z} {
		g.hashWord(math.Float32bits(f))
	}
	g.queryCount += 1
	return g.queryCount != g.queryStop
}

func (g *geometryGoldenGen) resetQuery(stop int) {
	g.queryHash = 0xcbf29ce484222325
	g.queryCount = 0
	g.queryStop = stop
}

func (g *geometryGoldenGen) pquery() {
	g.printf(" %d %016x", g.queryCount, g.queryHash)
}

func (g *geometryGoldenGen) pmesh(m *MeshData) {
	if m == nil {
		g.printf(" null")
		return
	}
	g.printf(" %d %016x %d %d %d %d %d %d %d", m.ByteCount, m.Hash, m.TreeHeight, m.NodeCount,
		m.VertexCount, m.TriangleCount, m.DegenerateCount, m.MaterialCount, GetHeight(m))
	g.pf(m.SurfaceArea)
	g.pbox(m.Bounds)
}

// rproxy is a random proxy: 1 to 4 points around center with a radius.
func (g *geometryGoldenGen) rproxy(center Vec3, spread float32) ShapeProxy {
	count := 1 + int(g.rndInt(4))
	for i := range count {
		offset := g.rv3(spread, spread, spread, 0.0, 0.0, 0.0)
		g.proxyPoints[i] = Add(center, offset)
	}
	var radius float32 = 0.0
	if g.rndInt(3) != 0 {
		radius = mulAdd(0.3, 0.25, g.rnd())
	}
	return ShapeProxy{g.proxyPoints[:], int32(count), radius}
}

func (g *geometryGoldenGen) soupDef() MeshDef {
	for i := range (soupN + 1) * (soupN + 1) {
		g.heights[i] = 0.5 * g.rnd()
	}

	v := 0
	t := 0
	for ix := range soupN {
		for iz := range soupN {
			p11 := Vec3{float32(ix), g.heights[ix*(soupN+1)+iz], float32(iz)}
			p12 := Vec3{float32(ix), g.heights[ix*(soupN+1)+iz+1], float32(iz + 1)}
			p21 := Vec3{float32(ix + 1), g.heights[(ix+1)*(soupN+1)+iz], float32(iz)}
			p22 := Vec3{float32(ix + 1), g.heights[(ix+1)*(soupN+1)+iz+1], float32(iz + 1)}

			g.soupVertices[v+0] = p11
			g.soupVertices[v+1] = p12
			g.soupVertices[v+2] = p22
			g.soupVertices[v+3] = p22
			g.soupVertices[v+4] = p21
			g.soupVertices[v+5] = p11
			for k := range 6 {
				g.soupIndices[v+k] = int32(v + k)
			}
			g.soupMaterials[t+0] = uint8(g.rndInt(5))
			g.soupMaterials[t+1] = uint8(g.rndInt(5))
			v += 6
			t += 2
		}
	}

	// Degenerates: a sliver (collinear), a repeated index and a tiny triangle.
	g.soupVertices[v+0] = Vec3{0.0, 1.0, 0.0}
	g.soupVertices[v+1] = Vec3{1.0, 1.0, 0.0}
	g.soupVertices[v+2] = Vec3{2.0, 1.0, 0.0}
	g.soupIndices[v+0] = int32(v + 0)
	g.soupIndices[v+1] = int32(v + 1)
	g.soupIndices[v+2] = int32(v + 2)
	g.soupVertices[v+3] = Vec3{0.0, 2.0, 0.0}
	g.soupVertices[v+4] = Vec3{1.0, 2.0, 0.0}
	g.soupVertices[v+5] = Vec3{0.0, 2.0, 1.0}
	g.soupIndices[v+3] = int32(v + 3)
	g.soupIndices[v+4] = int32(v + 3)
	g.soupIndices[v+5] = int32(v + 5)
	g.soupVertices[v+6] = Vec3{3.0, 2.0, 0.0}
	g.soupVertices[v+7] = Vec3{3.0001, 2.0, 0.0}
	g.soupVertices[v+8] = Vec3{3.0, 2.0, 0.0001}
	g.soupIndices[v+6] = int32(v + 6)
	g.soupIndices[v+7] = int32(v + 7)
	g.soupIndices[v+8] = int32(v + 8)
	g.soupMaterials[t+0] = 1
	g.soupMaterials[t+1] = 2
	g.soupMaterials[t+2] = 3
	v += 9
	t += 3

	def := MeshDef{}
	def.Vertices = g.soupVertices[:]
	def.Indices = g.soupIndices[:]
	def.MaterialIndices = g.soupMaterials[:]
	def.VertexCount = int32(v)
	def.TriangleCount = int32(t)
	return def
}

func (g *geometryGoldenGen) meshQueries(name string, data *MeshData, scale Vec3) {
	mesh := Mesh{data, scale}
	bounds := ComputeMeshAABB(data, TransformIdentity, scale)
	c := bounds.Center()
	e := bounds.Extents()

	g.printf("%s aabb |", name)
	{
		var xf Transform
		xf.P = g.rv3(2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
		xf.Q = g.rq()
		g.pbox(ComputeMeshAABB(data, xf, scale))
	}
	g.printf("\n")

	g.printf("%s triangles |", name)
	for i := 0; i < int(data.TriangleCount); i += 1 + int(data.TriangleCount)/7 {
		g.ptri(getMeshTriangle(&mesh, i))
	}
	g.printf("\n")

	for k := range 12 {
		g.printf("%s ray %d |", name, k)
		var ray RayCastInput
		ray.Origin = g.rv3(1.5*e.X, mulAdd(1.0, 1.5, e.Y), 1.5*e.Z, c.X, c.Y, c.Z)
		target := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		ray.Translation = MulSV(2.0, Sub(target, ray.Origin))
		ray.MaxFraction = 1.0
		if k%4 == 3 {
			ray.MaxFraction = 0.4
		}
		g.pco(RayCastMesh(&mesh, &ray))
		g.printf("\n")
	}

	for k := range 10 {
		g.printf("%s cast %d |", name, k)
		start := g.rv3(1.5*e.X, mulAdd(1.0, 1.5, e.Y), 1.5*e.Z, c.X, c.Y, c.Z)
		var input ShapeCastInput
		input.Proxy = g.rproxy(start, 0.3)
		target := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		input.Translation = MulSV(2.0, Sub(target, start))
		input.MaxFraction = 1.0
		input.CanEncroach = k%3 == 1
		g.pco(ShapeCastMesh(&mesh, &input))
		g.printf("\n")
	}

	g.printf("%s overlap |", name)
	for range 16 {
		var xf Transform
		xf.P = g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0)
		xf.Q = g.rq()
		local := g.rv3(e.X, e.Y+0.5, e.Z, c.X, c.Y, c.Z)
		world := TransformPoint(xf, local)
		proxy := g.rproxy(world, 0.4)
		g.pbool(OverlapMesh(&mesh, xf, &proxy))
	}
	g.printf("\n")

	for k := range 8 {
		g.printf("%s mover %d |", name, k)
		var mover Capsule
		mover.Center1 = g.rv3(e.X, e.Y+0.3, e.Z, c.X, c.Y, c.Z)
		mover.Center2 = Add(mover.Center1, g.rv3(0.3, 0.3, 0.3, 0.0, 0.4, 0.0))
		mover.Radius = mulAdd(0.4, 0.3, g.rnd())
		var planes [16]PlaneResult
		capacity := 16
		if k == 5 {
			capacity = 2
		}
		count := collideMoverAndMesh(planes[:capacity], &mesh, &mover)
		g.pplanes(planes[:], count)
		g.printf("\n")
	}

	g.printf("%s query |", name)
	for k := range 6 {
		var box AABB
		p := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		h := g.rv3(0.5, 0.5, 0.5, 0.8, 0.8, 0.8)
		box.LowerBound = Sub(p, h)
		box.UpperBound = Add(p, h)
		stop := -1
		if k == 4 {
			stop = 3
		}
		g.resetQuery(stop)
		QueryMesh(&mesh, box, g.meshQueryFcn, nil)
		g.pquery()
	}
	g.printf("\n")
}

func (g *geometryGoldenGen) meshSection() {
	var meshes []*MeshData

	g.printf("mesh creators |")
	meshes = append(meshes, CreateGridMesh(5, 4, 0.75, 3, true))
	meshes = append(meshes, CreateGridMesh(3, 3, 1.0, 0, false))
	meshes = append(meshes, CreateWaveMesh(7, 6, 0.6, 0.4, 0.13, 0.21))
	meshes = append(meshes, CreateTorusMesh(10, 7, 2.0, 0.5))
	meshes = append(meshes, CreateBoxMesh(Vec3{0.1, 0.2, -0.3}, Vec3{0.5, 1.0, 1.5}, true))
	meshes = append(meshes, CreateHollowBoxMesh(Vec3{1.0, 0.0, 0.0}, Vec3{2.0, 1.0, 1.5}))
	meshes = append(meshes, CreatePlatformMesh(Vec3{0.0, 1.0, 0.0}, 2.0, 1.0, 3.0))
	for _, m := range meshes {
		g.pmesh(m)
	}
	g.printf("\n")

	g.printf("mesh soup |")
	{
		saved := g.seed
		def := g.soupDef()
		degenerates := [4]int32{-1, -1, -1, -1}
		meshes = append(meshes, CreateMesh(&def, degenerates[:2]))
		g.pmesh(meshes[len(meshes)-1])
		g.printf(" %d %d %d %d", degenerates[0], degenerates[1], degenerates[2], degenerates[3])

		// Same soup, welded, SAH split
		g.seed = saved
		def = g.soupDef()
		def.WeldVertices = true
		def.WeldTolerance = 0.01
		def.IdentifyEdges = true
		meshes = append(meshes, CreateMesh(&def, nil))
		g.pmesh(meshes[len(meshes)-1])

		// Welded, median split, clockwise
		g.seed = saved
		def = g.soupDef()
		def.WeldVertices = true
		def.WeldTolerance = 0.01
		def.IdentifyEdges = true
		def.UseMedianSplit = true
		def.ClockWiseWinding = true
		meshes = append(meshes, CreateMesh(&def, nil))
		g.pmesh(meshes[len(meshes)-1])

		// Rejected defs
		def.Stride = 6
		g.pmesh(CreateMesh(&def, nil))
		def.Stride = 0
		def.VertexCount = 2
		g.pmesh(CreateMesh(&def, nil))
	}
	g.printf("\n")

	g.printf("mesh flags |")
	for _, m := range meshes {
		flags := GetMeshFlags(m)
		materials := GetMeshMaterialIndices(m)
		var sum uint32 = 0
		for j := range int(m.TriangleCount) {
			sum = 31*sum + uint32(flags[j]) + 7*uint32(materials[j])
		}
		g.printf(" %08x", sum)
	}
	g.printf("\n")

	scales := [4]Vec3{
		{1.0, 1.0, 1.0},
		{1.5, 0.5, 2.0},
		{-1.0, 1.0, 1.0},
		{0.75, -1.25, -0.5},
	}

	queried := [4]int{2, 3, 8, 9}
	for i := range 4 {
		for s := range 4 {
			name := fmt.Sprintf("mesh %d scale %d", queried[i], s)
			g.meshQueries(name, meshes[queried[i]], scales[s])
		}
	}

	for _, m := range meshes {
		DestroyMesh(m)
	}
}

func (g *geometryGoldenGen) run() string {
	g.seed = 31337

	g.meshSection()

	return g.sb.String()
}

var _ = Describe("GeometryGoldenTest", func() {
	It("MatchesC", func() {
		SetLogFcn(func(string) {})
		defer SetLogFcn(defaultLogFcn)

		want, err := os.ReadFile("testdata/geometry_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		var g geometryGoldenGen
		got := g.run()

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		for i := range min(len(wantLines), len(gotLines)) {
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
		Expect(gotLines).To(HaveLen(len(wantLines)))
	})
})
