// Go-only: the width 8 kernel generator of the Box3D port (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

// Command widegen writes the width 8 copies of the width generic wide kernels.
//
// PORT: upstream compiles contact_solver_wide.inl and convex_manifold_wide.inl twice, with
// B3_SIMD_WIDTH 4 and 8, and the B3_WIDE macro appends the width to every name. Go has no macros,
// so this tool copies each generic source (simd_wide.go, contact_solver_wide.go,
// convex_manifold_wide.go) to <name>8_gen.go and appends "8" to every package level identifier
// declared in the generic sources or in the width 4 lane contract (simd_scalar.go,
// wide_lanes.go). The width 8 lane contract itself is hand-written (simd_x8_amd64.go,
// wide_lanes8_amd64.go).
//
// Usage, in libs/box3d-go:
//
//	go run ./tools/widegen [-check] [-dir .]
//
// -check generates in memory and fails, listing the stale files, when the files on disk differ.
package main

import (
	"bytes"
	"errors"
	"flag"
	"fmt"
	"go/ast"
	"go/build"
	"go/format"
	"go/parser"
	"go/token"
	"io"
	"os"
	"path/filepath"
	"slices"
	"strings"
)

// buildLine is the build constraint of the generated files: the width 8 backend.
const buildLine = "//go:build box3d_simd && goexperiment.simd && amd64 && !box3d_no8"

// fieldRenames maps the width 4 step context and graph color fields named by the generic
// kernels to their width 8 twins (stepContext, graphColor).
var fieldRenames = map[string]string{
	"wideConstraints":     "wideConstraints8",
	"wideMeshConstraints": "wideMeshConstraints8",
}

// config names the files the generator reads, relative to dir.
type config struct {
	dir string
	// inputs are the width generic sources; each one yields one generated file.
	inputs []string
	// laneFiles are the width 4 lane contract; their names join the rename set, the files are
	// not copied.
	laneFiles []string
	// handWritten are the width 8 lane contract files, which declare renamed names by design.
	handWritten []string
}

func defaultConfig(dir string) config {
	return config{
		dir:         dir,
		inputs:      []string{"simd_wide.go", "contact_solver_wide.go", "convex_manifold_wide.go"},
		laneFiles:   []string{"simd_scalar.go", "wide_lanes.go"},
		handWritten: []string{"simd_x8_amd64.go", "wide_lanes8_amd64.go"},
	}
}

// outFile is one generated file.
type outFile struct {
	input string // generic source name
	name  string // generated file name
	src   []byte
}

// outputName maps contact_solver_wide.go to contact_solver_wide8_gen.go.
func outputName(input string) string {
	return strings.TrimSuffix(input, ".go") + "8_gen.go"
}

func main() {
	check := flag.Bool("check", false, "fail if the generated files are stale instead of writing them")
	dir := flag.String("dir", ".", "package directory (libs/box3d-go)")
	flag.Parse()
	if err := run(defaultConfig(*dir), *check, os.Stdout); err != nil {
		fmt.Fprintln(os.Stderr, "widegen:", err)
		os.Exit(1)
	}
}

// run generates the outputs and writes them, or with check compares them with the files on disk.
func run(cfg config, check bool, w io.Writer) error {
	files, err := generate(cfg)
	if err != nil {
		return err
	}
	var stale []string
	for _, f := range files {
		path := filepath.Join(cfg.dir, f.name)
		old, err := os.ReadFile(path)
		if err == nil && bytes.Equal(old, f.src) {
			continue
		}
		if err != nil && !errors.Is(err, os.ErrNotExist) {
			return err
		}
		if check {
			stale = append(stale, f.name)
			continue
		}
		if err := os.WriteFile(path, f.src, 0o644); err != nil {
			return err
		}
		fmt.Fprintln(w, "widegen: wrote", f.name)
	}
	if len(stale) > 0 {
		return fmt.Errorf("stale generated files (run go generate): %s", strings.Join(stale, ", "))
	}
	return nil
}

// generate builds every output in memory, in input order.
func generate(cfg config) ([]outFile, error) {
	fset := token.NewFileSet()
	parse := func(name string) (*ast.File, error) {
		return parser.ParseFile(fset, filepath.Join(cfg.dir, name), nil, parser.ParseComments)
	}

	renames := map[string]bool{}
	collect := func(name string) (*ast.File, error) {
		file, err := parse(name)
		if err != nil {
			return nil, err
		}
		for _, id := range packageNames(file) {
			renames[id.Name] = true
		}
		return file, nil
	}
	inputs := make([]*ast.File, len(cfg.inputs))
	for i, name := range cfg.inputs {
		file, err := collect(name)
		if err != nil {
			return nil, err
		}
		inputs[i] = file
	}
	for _, name := range cfg.laneFiles {
		if !slices.Contains(cfg.inputs, name) {
			if _, err := collect(name); err != nil {
				return nil, err
			}
		}
	}
	if err := checkCollisions(cfg, renames); err != nil {
		return nil, err
	}

	var out []outFile
	for i, file := range inputs {
		name := cfg.inputs[i]
		if err := renameFile(fset, file, renames); err != nil {
			return nil, fmt.Errorf("%s: %w", name, err)
		}
		src, err := render(fset, file, name)
		if err != nil {
			return nil, fmt.Errorf("%s: %w", name, err)
		}
		out = append(out, outFile{input: name, name: outputName(name), src: src})
	}
	return out, nil
}

// packageNames returns the package level identifiers a file declares: types, variables,
// constants and functions, but not methods, the blank identifier or init.
func packageNames(file *ast.File) []*ast.Ident {
	var ids []*ast.Ident
	add := func(id *ast.Ident) {
		if id.Name != "_" && id.Name != "init" {
			ids = append(ids, id)
		}
	}
	for _, decl := range file.Decls {
		switch d := decl.(type) {
		case *ast.FuncDecl:
			if d.Recv == nil {
				add(d.Name)
			}
		case *ast.GenDecl:
			for _, spec := range d.Specs {
				switch s := spec.(type) {
				case *ast.TypeSpec:
					add(s.Name)
				case *ast.ValueSpec:
					for _, id := range s.Names {
						add(id)
					}
				}
			}
		}
	}
	return ids
}

// checkCollisions fails when a renamed name is already declared by a file that is in the width 8
// build, other than the generated files and the hand-written width 8 lane contract. A file is in
// that build when go/build matches it for amd64 with box3d_simd and goexperiment.simd on any of
// linux, darwin and windows.
func checkCollisions(cfg config, renames map[string]bool) error {
	entries, err := os.ReadDir(cfg.dir)
	if err != nil {
		return err
	}
	skip := map[string]bool{}
	for _, name := range cfg.inputs {
		skip[outputName(name)] = true
	}
	for _, name := range cfg.handWritten {
		skip[name] = true
	}
	var found []string
	fset := token.NewFileSet()
	for _, e := range entries {
		name := e.Name()
		if e.IsDir() || !strings.HasSuffix(name, ".go") || strings.HasSuffix(name, "_test.go") || skip[name] {
			continue
		}
		in, err := inWidth8Build(cfg.dir, name)
		if err != nil {
			return err
		}
		if !in {
			continue
		}
		file, err := parser.ParseFile(fset, filepath.Join(cfg.dir, name), nil, parser.SkipObjectResolution)
		if err != nil {
			return err
		}
		for _, id := range packageNames(file) {
			if base, ok := strings.CutSuffix(id.Name, "8"); ok && renames[base] {
				found = append(found, fmt.Sprintf("%s (from %s) is already declared in %s", id.Name, base, name))
			}
		}
	}
	if len(found) > 0 {
		return fmt.Errorf("name collisions:\n\t%s", strings.Join(found, "\n\t"))
	}
	return nil
}

func inWidth8Build(dir, name string) (bool, error) {
	for _, goos := range []string{"linux", "darwin", "windows"} {
		ctxt := build.Default
		ctxt.GOOS = goos
		ctxt.GOARCH = "amd64"
		ctxt.CgoEnabled = false
		ctxt.BuildTags = []string{"box3d_simd"}
		ctxt.ToolTags = []string{"goexperiment.simd"}
		ok, err := ctxt.MatchFile(dir, name)
		if err != nil || ok {
			return ok, err
		}
	}
	return false, nil
}

// renameFile appends "8" to every reference to a name in renames that resolves to a package
// level declaration, and renames the width 4 step context fields. The parser's identifier
// resolution tells package level names apart from locals: an identifier that resolves inside the
// file resolves to its declaration (a local, a parameter, a field, or a package level declaration
// of this file); one that does not resolve names a package level declaration of another file, a
// universe name or an import.
func renameFile(fset *token.FileSet, file *ast.File, renames map[string]bool) error {
	topLevel := map[any]bool{}
	for _, decl := range file.Decls {
		switch d := decl.(type) {
		case *ast.FuncDecl:
			topLevel[d] = true
		case *ast.GenDecl:
			for _, spec := range d.Specs {
				topLevel[spec] = true
			}
		}
	}

	// Identifiers that never name a package level declaration: selectors, method names, struct
	// field and interface method names, struct literal keys, labels and the package name.
	selector := map[*ast.Ident]bool{}
	skip := map[*ast.Ident]bool{file.Name: true}
	var errs []error
	fail := func(pos token.Pos, format string, args ...any) {
		errs = append(errs, fmt.Errorf("%s: %s", fset.Position(pos), fmt.Sprintf(format, args...)))
	}
	ast.Inspect(file, func(n ast.Node) bool {
		switch n := n.(type) {
		case *ast.SelectorExpr:
			selector[n.Sel] = true
		case *ast.FuncDecl:
			if n.Recv != nil {
				skip[n.Name] = true
			}
		case *ast.StructType:
			for _, f := range n.Fields.List {
				for _, id := range f.Names {
					skip[id] = true
				}
				if len(f.Names) == 0 {
					if id := embeddedName(f.Type); id != nil && renames[id.Name] {
						fail(id.Pos(), "embedded field %s would change its field name when renamed", id.Name)
					}
				}
			}
		case *ast.InterfaceType:
			for _, f := range n.Methods.List {
				for _, id := range f.Names {
					skip[id] = true
				}
			}
		case *ast.CompositeLit:
			if n.Type != nil {
				markKeys(n, n.Type, selector, renames, fail)
			}
		case *ast.LabeledStmt:
			skip[n.Label] = true
		case *ast.BranchStmt:
			if n.Label != nil {
				skip[n.Label] = true
			}
		case *ast.ImportSpec:
			if n.Name != nil {
				skip[n.Name] = true
			}
		}
		return true
	})
	if len(errs) > 0 {
		return errors.Join(errs...)
	}

	ast.Inspect(file, func(n ast.Node) bool {
		id, ok := n.(*ast.Ident)
		if !ok || skip[id] {
			return true
		}
		if selector[id] {
			if to, ok := fieldRenames[id.Name]; ok {
				id.Name = to
			}
			return true
		}
		if renames[id.Name] && (id.Obj == nil || topLevel[id.Obj.Decl]) {
			id.Name += "8"
		}
		return true
	})
	return nil
}

// embeddedName returns the type name of an embedded field (T, *T, T[P]), or nil for a
// qualified name.
func embeddedName(x ast.Expr) *ast.Ident {
	switch t := x.(type) {
	case *ast.Ident:
		return t
	case *ast.StarExpr:
		return embeddedName(t.X)
	case *ast.IndexExpr:
		return embeddedName(t.X)
	case *ast.IndexListExpr:
		return embeddedName(t.X)
	}
	return nil
}

// markKeys classifies the keys of a composite literal of type typ, and of the literals nested in
// it with elided types. Array, slice and map keys are expressions and are renamed as usual. Any
// other literal type is taken as a struct (the generator does not resolve named types): its keys
// are field names, renamed like field selectors. A field name key that is also a renamed package
// level name is ambiguous (it could be an index constant of a named array type) and an error.
func markKeys(lit *ast.CompositeLit, typ ast.Expr, selector map[*ast.Ident]bool, renames map[string]bool,
	fail func(token.Pos, string, ...any)) {
	var keyType, eltType ast.Expr
	isStruct := false
	switch t := typ.(type) {
	case *ast.ArrayType:
		eltType = t.Elt
	case *ast.MapType:
		keyType, eltType = t.Key, t.Value
	default:
		isStruct = true
	}
	nested := func(x, typ ast.Expr) {
		if l, ok := x.(*ast.CompositeLit); ok && l.Type == nil {
			markKeys(l, typ, selector, renames, fail)
		}
		if u, ok := x.(*ast.UnaryExpr); ok && u.Op == token.AND {
			if l, ok := u.X.(*ast.CompositeLit); ok && l.Type == nil {
				if s, ok := typ.(*ast.StarExpr); ok {
					markKeys(l, s.X, selector, renames, fail)
				} else {
					markKeys(l, nil, selector, renames, fail)
				}
			}
		}
	}
	for _, e := range lit.Elts {
		kv, ok := e.(*ast.KeyValueExpr)
		if !ok {
			if !isStruct {
				nested(e, eltType)
			}
			continue
		}
		if isStruct {
			if id, ok := kv.Key.(*ast.Ident); ok {
				if renames[id.Name] {
					fail(id.Pos(), "composite literal key %s is a renamed package level name; "+
						"the generator cannot tell a struct field from an index constant", id.Name)
				}
				selector[id] = true
			}
			continue
		}
		nested(kv.Key, keyType)
		nested(kv.Value, eltType)
	}
}

// render prints the renamed file with the generated header and the width 8 build constraint in
// place of the input's own build constraint and go:generate directives.
func render(fset *token.FileSet, file *ast.File, input string) ([]byte, error) {
	var body bytes.Buffer
	if err := format.Node(&body, fset, file); err != nil {
		return nil, err
	}
	var out bytes.Buffer
	fmt.Fprintf(&out, "// Code generated by tools/widegen from %s. DO NOT EDIT.\n\n%s\n\n", input, buildLine)
	var lines []string
	for line := range strings.Lines(body.String()) {
		if strings.HasPrefix(line, "//go:build") || strings.HasPrefix(line, "// +build") ||
			strings.HasPrefix(line, "//go:generate") {
			// Drop the "//" line that separates a directive from the comment above it.
			if n := len(lines); n > 0 && lines[n-1] == "//\n" {
				lines = lines[:n-1]
			}
			continue
		}
		lines = append(lines, line)
	}
	out.WriteString(strings.Join(lines, ""))
	return format.Source(out.Bytes())
}
