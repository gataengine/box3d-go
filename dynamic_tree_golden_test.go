// Go-only test: no upstream counterpart. Compares dynamic tree insertion (with rotations and
// pool growth), destroy/move/enlarge churn, moved marking, the sweep and fallback refits, partial
// and full rebuilds, save/load, AABB/closest queries, ray casts, box casts and the proxy-less
// tree view against the pinned upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867)
// compiled with -ffp-contract=off, bit for bit. Tree state is compared through b3Hash of the
// node, parent and proxy arrays; queries through a hash of the callback sequence. See
// testdata/dynamic_tree_golden_gen.c; this file mirrors it line by line.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const treeGoldenBoxCount = 600

type treeGoldenGen struct {
	seed    uint32
	sb      strings.Builder
	boxes   [treeGoldenBoxCount]AABB
	proxies [treeGoldenBoxCount]int
	alive   [treeGoldenBoxCount]bool
	dir     string
}

func (g *treeGoldenGen) nextRandom() uint32 {
	g.seed = 1664525*g.seed + 1013904223
	return g.seed
}

func (g *treeGoldenGen) randomFloat(lo, hi float32) float32 {
	t := float32(g.nextRandom()>>8) / float32(1<<24)
	return mulAdd(lo, t, hi-lo)
}

func (g *treeGoldenGen) randomVec(lo, hi float32) Vec3 {
	var v Vec3
	v.X = g.randomFloat(lo, hi)
	v.Y = g.randomFloat(lo, hi)
	v.Z = g.randomFloat(lo, hi)
	return v
}

func (g *treeGoldenGen) randomBox(extent, size float32) AABB {
	center := g.randomVec(-extent, extent)
	half := g.randomVec(0.05, size)
	return AABB{Sub(center, half), Add(center, half)}
}

func (g *treeGoldenGen) printf(format string, args ...any) { fmt.Fprintf(&g.sb, format, args...) }
func (g *treeGoldenGen) pf(f float32)                      { g.printf(" %08x", math.Float32bits(f)) }
func (g *treeGoldenGen) pv(v Vec3)                         { g.pf(v.X); g.pf(v.Y); g.pf(v.Z) }

func boolInt(b bool) int {
	if b {
		return 1
	}
	return 0
}

func (g *treeGoldenGen) printTree(label string, tree *DynamicTree) {
	g.printf("%s |", label)
	g.printf(" %d %d %d %d %d %d %d", tree.NodeEnd, tree.NodeCapacity, tree.PairFreeList, tree.ProxyCount,
		tree.ProxyCapacity, tree.ProxyFreeList, boolInt(tree.DfsOrdered))
	g.printf(" %d", tree.GetHeight())
	g.pf(tree.GetAreaRatio())
	root := tree.GetRootBounds()
	g.pv(root.LowerBound)
	g.pv(root.UpperBound)
	g.printf(" %d", tree.GetByteCount())
	g.printf(" %08x", Hash(HashInit, treeNodesBytes(tree.Nodes, int(tree.NodeEnd))))
	g.printf(" %08x", Hash(HashInit, treeInt32Bytes(tree.Parents, int(tree.NodeEnd))))
	g.printf(" %08x", Hash(HashInit, treeProxiesBytes(tree.Proxies, int(tree.ProxyCapacity))))
	g.printf("\n")
}

type treeTrace struct {
	hash  uint32
	count int
	boxes []AABB
	point Vec3
}

func (trace *treeTrace) traceInt(proxyId int, userData uint64) {
	var b [12]byte
	binary.LittleEndian.PutUint32(b[0:], uint32(int32(proxyId)))
	binary.LittleEndian.PutUint64(b[4:], userData)
	trace.hash = Hash(trace.hash, b[0:4])
	trace.hash = Hash(trace.hash, b[4:12])
	trace.count += 1
}

func (trace *treeTrace) traceFloat(f float32) {
	var b [4]byte
	binary.LittleEndian.PutUint32(b[:], math.Float32bits(f))
	trace.hash = Hash(trace.hash, b[:])
}

func treeGoldenQueryCallback(proxyId int, userData uint64, context any) bool {
	trace := context.(*treeTrace)
	trace.traceInt(proxyId, userData)

	// Stop early on some proxy so the early out is covered
	return (userData & 0xFFFF) != 77
}

func treeGoldenQueryAllCallback(proxyId int, userData uint64, context any) bool {
	trace := context.(*treeTrace)
	trace.traceInt(proxyId, userData)
	return true
}

func treeGoldenClosestCallback(distanceSqrMin float32, proxyId int, userData uint64, context any) float32 {
	trace := context.(*treeTrace)
	trace.traceInt(proxyId, userData)
	trace.traceFloat(distanceSqrMin)

	box := trace.boxes[userData&0xFFFF]
	r := Sub(trace.point, Clamp(trace.point, box.LowerBound, box.UpperBound))
	d := Dot(r, r)
	if d < distanceSqrMin {
		return d
	}
	return distanceSqrMin
}

func treeGoldenRayCallback(input *RayCastInput, proxyId int, userData uint64, context any) float32 {
	trace := context.(*treeTrace)
	trace.traceInt(proxyId, userData)
	trace.traceFloat(input.MaxFraction)

	box := trace.boxes[userData&0xFFFF]
	p2 := MulAdd(input.Origin, input.MaxFraction, input.Translation)
	hit, minFraction, _ := rayCastAABB(box, input.Origin, p2)
	if hit == false {
		return -1.0
	}

	// Clip to the box entry, scaled back to the full translation
	fraction := float32(minFraction * input.MaxFraction)
	if fraction <= 0.0 {
		return -1.0
	}

	return fraction
}

func treeGoldenBoxCallback(input *BoxCastInput, proxyId int, userData uint64, context any) float32 {
	trace := context.(*treeTrace)
	trace.traceInt(proxyId, userData)
	trace.traceFloat(input.MaxFraction)

	switch userData % 4 {
	case 0:
		return -1.0
	case 1:
		return float32(0.75 * input.MaxFraction)
	case 2:
		return input.MaxFraction
	default:
		if (userData & 0xFFFF) == 403 {
			return 0.0
		}
		return float32(0.5 * input.MaxFraction)
	}
}

func (g *treeGoldenGen) printTrace(label string, round int, trace *treeTrace, stats TreeStats) {
	g.printf("%s %d | %d %08x %d %d\n", label, round, trace.count, trace.hash, stats.NodeVisits, stats.LeafVisits)
}

func treeGoldenUserData(i int) uint64 {
	// High bits make the shapeIndex truncation visible
	return (uint64(i*3+1) << 32) | uint64(i)
}

func treeGoldenCategoryBits(i int) uint64 {
	return 1 << (i % 5)
}

func (g *treeGoldenGen) runQueries(label string, tree *DynamicTree) {
	for round := 0; round < 12; round++ {
		box := g.randomBox(12.0, 4.0)
		mask := DefaultMaskBits
		if round%3 != 0 {
			mask = uint64(round%7 + 1)
		}
		requireAll := round%4 == 3

		trace := treeTrace{HashInit, 0, g.boxes[:], Vec3Zero}
		stats := tree.Query(box, mask, requireAll, treeGoldenQueryCallback, &trace)
		g.printTrace(label+" query", round, &trace, stats)
	}

	for round := 0; round < 12; round++ {
		point := g.randomVec(-14.0, 14.0)
		mask := DefaultMaskBits
		if round%2 != 0 {
			mask = uint64(round%7 + 1)
		}

		trace := treeTrace{HashInit, 0, g.boxes[:], point}
		minDistanceSqr := fltMax
		if round == 5 {
			minDistanceSqr = 4.0
		}
		stats := tree.QueryClosest(point, mask, round == 7, treeGoldenClosestCallback, &trace, &minDistanceSqr)
		trace.traceFloat(minDistanceSqr)
		g.printTrace(label+" closest", round, &trace, stats)
	}

	for round := 0; round < 12; round++ {
		var input RayCastInput
		input.Origin = g.randomVec(-14.0, 14.0)
		input.Translation = g.randomVec(-28.0, 28.0)
		input.MaxFraction = 1.0
		if round == 4 {
			input.MaxFraction = 0.5
		}
		mask := DefaultMaskBits
		if round%3 == 1 {
			mask = uint64(round%7 + 1)
		}

		trace := treeTrace{HashInit, 0, g.boxes[:], Vec3Zero}
		stats := tree.RayCast(&input, mask, false, treeGoldenRayCallback, &trace)
		g.printTrace(label+" ray", round, &trace, stats)
	}

	for round := 0; round < 12; round++ {
		var input BoxCastInput
		input.Box = g.randomBox(14.0, 1.0)
		input.Translation = g.randomVec(-28.0, 28.0)
		input.MaxFraction = 1.0

		trace := treeTrace{HashInit, 0, g.boxes[:], Vec3Zero}
		stats := tree.BoxCast(&input, DefaultMaskBits, round == 2, treeGoldenBoxCallback, &trace)
		g.printTrace(label+" boxcast", round, &trace, stats)
	}

	// A view without proxies reports the truncated shape index and skips the filter
	{
		view := makeTreeView(tree)
		view.proxies = nil
		box := g.randomBox(8.0, 6.0)
		trace := treeTrace{HashInit, 0, g.boxes[:], Vec3Zero}
		stats := view.query(box, 0, true, treeGoldenQueryAllCallback, &trace)
		g.printTrace(label+" view", 0, &trace, stats)
	}
}

func (g *treeGoldenGen) printMoved(label string, tree *DynamicTree) {
	var ids [treeGoldenBoxCount]int32
	count := tree.gatherMovedProxies(ids[:])
	g.printf("%s | %d %08x\n", label, count, Hash(HashInit, treeInt32Bytes(ids[:], count)))
}

func (g *treeGoldenGen) run() string {
	g.seed = 2024
	boxes := &g.boxes
	proxies := &g.proxies
	alive := &g.alive

	tree := CreateDynamicTree(16)
	g.printTree("empty", &tree)

	// Build by insertion, with growth of both pools
	for i := 0; i < treeGoldenBoxCount; i++ {
		boxes[i] = g.randomBox(10.0, 1.2)
		proxies[i] = tree.CreateProxy(boxes[i], treeGoldenCategoryBits(i), treeGoldenUserData(i))
		alive[i] = true
		if i == 0 || i == 1 || i == 2 || i == 15 || i == 16 || i == 100 {
			g.printTree(fmt.Sprintf("insert %d", i), &tree)
		}
	}
	g.printTree("inserted", &tree)
	g.runQueries("inserted", &tree)

	// Full rebuild
	sorted := tree.Rebuild(true)
	g.printf("rebuild full | %d\n", sorted)
	g.printTree("rebuilt", &tree)
	g.runQueries("rebuilt", &tree)

	// Churn: destroy, move, enlarge
	for i := 0; i < treeGoldenBoxCount; i += 5 {
		tree.DestroyProxy(proxies[i])
		alive[i] = false
	}
	g.printTree("destroyed", &tree)

	for i := 1; i < treeGoldenBoxCount; i += 3 {
		if alive[i] == false {
			continue
		}
		boxes[i] = g.randomBox(10.0, 1.2)
		tree.MoveProxy(proxies[i], boxes[i])
	}
	g.printTree("moved", &tree)

	for i := 2; i < treeGoldenBoxCount; i += 4 {
		if alive[i] == false {
			continue
		}
		boxes[i] = boxes[i].Inflate(0.3)
		tree.EnlargeProxy(proxies[i], boxes[i])
	}
	g.printTree("enlarged", &tree)
	g.printMoved("enlarged moved", &tree)

	tree.refit()
	g.printTree("refit unordered", &tree)

	sorted = tree.Rebuild(false)
	g.printf("rebuild partial | %d\n", sorted)
	g.printTree("partial", &tree)
	g.runQueries("partial", &tree)

	// Re-create into the holes with moved marks
	for i := 0; i < treeGoldenBoxCount; i += 5 {
		boxes[i] = g.randomBox(10.0, 1.2)
		proxies[i] = createTreeProxyInternal(&tree, boxes[i], treeGoldenCategoryBits(i), treeGoldenUserData(i), true)
		alive[i] = true
	}
	g.printTree("recreated", &tree)

	// The solver path: mark moved with grown boxes, gather, sweep refit, partial rebuild
	sorted = tree.Rebuild(false)
	g.printf("rebuild recreated | %d\n", sorted)
	g.printTree("recreated rebuilt", &tree)

	for i := 3; i < treeGoldenBoxCount; i += 7 {
		boxes[i] = boxes[i].Inflate(0.25)
		tree.markProxyMoved(proxies[i], boxes[i])
	}
	tree.markProxyMovedSerial(proxies[11])
	g.printTree("marked", &tree)
	g.printMoved("marked moved", &tree)

	tree.refit()
	g.printTree("refit sweep", &tree)

	sorted = tree.Rebuild(false)
	g.printf("rebuild marked | %d\n", sorted)
	g.printTree("marked rebuilt", &tree)
	g.runQueries("marked", &tree)

	for i := 4; i < treeGoldenBoxCount; i += 9 {
		boxes[i] = g.randomBox(10.0, 1.2)
		tree.moveProxyInternal(proxies[i], boxes[i], true)
	}
	g.printMoved("internal moved", &tree)
	tree.clearMoved()
	g.printTree("cleared", &tree)

	// Save and load the byte image
	path := filepath.Join(g.dir, "dynamic_tree_golden.dat")
	tree.Save(path)
	{
		image, err := os.ReadFile(path)
		Expect(err).NotTo(HaveOccurred())
		g.printf("saved | %d %08x\n", len(image), Hash(HashInit, image))
	}

	loaded := LoadDynamicTree(path, 2.0)
	_ = os.Remove(path)
	g.printTree("loaded", &loaded)
	loaded.Rebuild(true)
	g.printTree("loaded rebuilt", &loaded)
	loaded.Destroy()

	// Drain to a single proxy and back
	for i := 0; i < treeGoldenBoxCount-1; i++ {
		tree.DestroyProxy(proxies[i])
		alive[i] = false
	}
	g.printTree("single", &tree)
	sorted = tree.Rebuild(true)
	g.printf("rebuild single | %d\n", sorted)
	g.printTree("single rebuilt", &tree)

	for i := 0; i < 40; i++ {
		boxes[i] = g.randomBox(3.0, 0.5)
		proxies[i] = tree.CreateProxy(boxes[i], treeGoldenCategoryBits(i), treeGoldenUserData(i))
	}
	g.printTree("refilled", &tree)
	sorted = tree.Rebuild(false)
	g.printf("rebuild refilled | %d\n", sorted)
	g.printTree("refilled rebuilt", &tree)

	tree.Destroy()
	return g.sb.String()
}

var _ = Describe("DynamicTreeGoldenTest", func() {
	It("MatchesC", func() {
		want, err := os.ReadFile("testdata/dynamic_tree_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		g := treeGoldenGen{dir: GinkgoT().TempDir()}
		got := g.run()

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		Expect(gotLines).To(HaveLen(len(wantLines)))
		for i := range wantLines {
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
	})
})
