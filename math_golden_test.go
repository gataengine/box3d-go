// Go-only test: no upstream counterpart. Compares the math port bit for bit against values
// produced by the pinned upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867) compiled
// with -ffp-contract=off. See testdata/math_golden_gen.c.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

type goldenWriter struct{ out []float32 }

func (w *goldenWriter) f(x float32) { w.out = append(w.out, x) }
func (w *goldenWriter) v(x Vec3)    { w.f(x.X); w.f(x.Y); w.f(x.Z) }
func (w *goldenWriter) q(x Quat)    { w.v(x.V); w.f(x.S) }
func (w *goldenWriter) m(x Matrix3) { w.v(x.Cx); w.v(x.Cy); w.v(x.Cz) }
func (w *goldenWriter) u(x uint32)  { w.f(math.Float32frombits(x)) }

type goldenReader struct{ in []float32 }

func (r *goldenReader) f() float32 { x := r.in[0]; r.in = r.in[1:]; return x }
func (r *goldenReader) v() Vec3    { return Vec3{r.f(), r.f(), r.f()} }
func (r *goldenReader) q() Quat    { return Quat{r.v(), r.f()} }
func (r *goldenReader) m() Matrix3 { return Matrix3{r.v(), r.v(), r.v()} }

func parseHexFloats(fields []string) []float32 {
	out := make([]float32, len(fields))
	for i, s := range fields {
		bits, err := strconv.ParseUint(s, 16, 32)
		Expect(err).NotTo(HaveOccurred())
		out[i] = math.Float32frombits(uint32(bits))
	}
	return out
}

// evalGolden evaluates one golden line. "mat" lines are evaluated by the caller because they
// reuse q1 of the preceding "quat" line, as the generator does.
func evalGolden(name string, r *goldenReader) []float32 {
	var w goldenWriter
	switch name {
	case "atan2":
		y, x := r.f(), r.f()
		w.f(Atan2(y, x))
	case "cossin":
		cs := ComputeCosSin(r.f())
		w.f(cs.Cosine)
		w.f(cs.Sine)
	case "quat":
		q1, q2, v := r.q(), r.q(), r.v()
		w.q(MulQuat(q1, q2))
		w.q(InvMulQuat(q1, q2))
		w.v(RotateVector(q1, v))
		w.v(InvRotateVector(q1, v))
		w.q(NLerp(q1, q2, 0.3))
		w.m(MakeMatrixFromQuat(q1))
		w.f(GetTwistAngle(q1))
		w.f(GetSwingAngle(q1))
		m := MakeMatrixFromQuat(q2)
		w.q(MakeQuatFromMatrix(&m))
		w.v(deltaQuatToRotation(q1, q2))
		w.q(integrateRotation(q1, v))
	case "seg":
		p1, q1, p2, q2 := r.v(), r.v(), r.v(), r.v()
		s := SegmentDistance(p1, q1, p2, q2)
		w.v(s.Point1)
		w.f(s.Fraction1)
		w.v(s.Point2)
		w.f(s.Fraction2)
		s = LineDistance(p1, q1, p2, q2)
		w.v(s.Point1)
		w.f(s.Fraction1)
		w.v(s.Point2)
		w.f(s.Fraction2)
		w.v(PointToSegmentDistance(p1, q1, p2))
		tp := closestPointOnTriangle(p1, q1, p2, q2)
		w.v(tp.point)
		w.u(uint32(tp.feature))
		w.v(Normalize(p1))
		w.v(Cross(p1, p2))
		w.f(Dot(p1, p2))
		w.v(arbitraryPerp(Normalize(p1)))
		w.q(ComputeQuatBetweenUnitVectors(Normalize(p1), Normalize(p2)))
	default:
		Fail("unknown golden function " + name)
	}
	return w.out
}

var _ = Describe("MathGolden", func() {
	It("matches the C implementation bit for bit", func() {
		file, err := os.Open("testdata/math_golden.txt")
		Expect(err).NotTo(HaveOccurred())
		defer file.Close()

		var lastQ1 Quat
		lines := 0
		scanner := bufio.NewScanner(file)
		scanner.Buffer(make([]byte, 1<<16), 1<<20)
		for scanner.Scan() {
			left, right, ok := strings.Cut(scanner.Text(), " |")
			Expect(ok).To(BeTrue())
			fields := strings.Fields(left)
			name := fields[0]
			r := &goldenReader{in: parseHexFloats(fields[1:])}
			want := parseHexFloats(strings.Fields(right))

			var got []float32
			if name == "mat" {
				mm, v, mass := r.m(), r.v(), r.f()
				var w goldenWriter
				w.m(InvertMatrix(mm))
				w.v(Solve3(mm, v))
				w.m(InvertT(mm))
				w.f(Det(mm))
				w.m(Steiner(mass, v))
				w.m(MulMM(mm, mm))
				w.v(MulMV(mm, v))
				w.m(rotateInertia(lastQ1, mm))
				w.f(scalarTripleProduct(mm.Cx, mm.Cy, mm.Cz))
				w.v(blend3(mass, mm.Cx, v.X, mm.Cy, v.Y, mm.Cz))
				got = w.out
			} else {
				if name == "quat" {
					lastQ1 = Quat{Vec3{r.in[0], r.in[1], r.in[2]}, r.in[3]}
				}
				got = evalGolden(name, r)
			}

			Expect(got).To(HaveLen(len(want)), "line %d (%s)", lines+1, name)
			for i := range want {
				g, wv := math.Float32bits(got[i]), math.Float32bits(want[i])
				if g != wv {
					Fail(fmt.Sprintf("line %d (%s) output %d: got %08x (%g) want %08x (%g)", lines+1, name, i, g, got[i], wv, want[i]))
				}
			}
			lines++
		}
		Expect(scanner.Err()).NotTo(HaveOccurred())
		Expect(lines).To(Equal(1500))
	})
})
