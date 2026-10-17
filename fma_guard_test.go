// Go-only test: no upstream counterpart.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"os"
	"os/exec"
	"regexp"
	"strings"
	"testing"
)

// fusedOp matches fused multiply-add instructions in the compiler's assembly listing:
// arm64 FMADD/FMSUB/FNMADD/FNMSUB (S and D forms) and amd64 v3 VFMADD/VFMSUB/VFNMADD/VFNMSUB.
var fusedOp = regexp.MustCompile(`\t(FN?M(ADD|SUB)[SD]|VFN?M(ADD|SUB)(132|213|231)[SP][SD])\t`)

// TestNoFusedMultiplyAdd guards cross-platform determinism. Go fuses x*y + z into one FMA
// instruction on arm64 (and amd64 with GOAMD64=v3) unless an explicit float32 conversion
// rounds the product first. Upstream builds with -ffp-contract=off, so any fused instruction in
// the package is a port bug: wrap the product (see mulAdd / mulSub in math_functions.go).
func TestNoFusedMultiplyAdd(t *testing.T) {
	if testing.Short() {
		t.Skip("compiles the package for two architectures")
	}

	// The pattern must catch what the compiler emits for a + b*c and a - b*c.
	for _, sample := range []string{"\tFMADDS\tF1, F0, F2, F0", "\tFMSUBS\tF1, F0, F2, F0", "\tVFMADD231SS\tX2, X1, X0"} {
		if !fusedOp.MatchString(sample) {
			t.Fatalf("fusedOp misses %q", sample)
		}
	}
	if fusedOp.MatchString("\tFMULS\tF2, F1, F1") {
		t.Fatal("fusedOp matches a plain multiply")
	}

	targets := []struct{ goarch, goamd64 string }{
		{"arm64", ""},
		{"amd64", "v3"},
	}

	for _, target := range targets {
		cmd := exec.Command("go", "build", "-gcflags=-S", "-o", os.DevNull, ".")
		cmd.Env = append(os.Environ(), "GOOS=linux", "GOARCH="+target.goarch, "CGO_ENABLED=0")
		if target.goamd64 != "" {
			cmd.Env = append(cmd.Env, "GOAMD64="+target.goamd64)
		}
		out, err := cmd.CombinedOutput()
		if err != nil {
			t.Fatalf("%s build failed: %v\n%s", target.goarch, err, out)
		}

		var function string
		for line := range strings.SplitSeq(string(out), "\n") {
			if strings.Contains(line, " STEXT ") {
				function, _, _ = strings.Cut(line, " ")
				continue
			}
			if fusedOp.MatchString(line) {
				t.Errorf("%s: fused multiply-add in %s:\n%s", target.goarch, function, line)
			}
		}
	}
}
