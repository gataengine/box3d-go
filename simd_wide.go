// Port of src/simd_wide.h and the b3Vec3W4 part of src/simd.h from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: B3_WIDE compiles the wide kernels once per width; Go has no macros, so tools/widegen
// writes the width 8 copies (simd_wide8_gen.go, contact_solver_wide8_gen.go,
// convex_manifold_wide8_gen.go) of the width generic sources, appending 8 to their package level
// names and to those of the width 4 lane contract (simd_scalar.go, wide_lanes.go).
//
//go:generate go run ./tools/widegen

// PORT: upstream maps the width-neutral names (b3FloatW, b3AddW, …) onto the W4 or W8 functions
// with the B3_WIDE macro so the wide kernels compile twice. The Go port has one lane width
// (laneCount in simd_scalar.go) and the lane functions already carry the width-neutral names:
//
//	b3FloatW → floatW         b3Vec3W → vec3W
//	b3ZeroW, b3SplatW, b3SetW, b3LoadW, b3StoreW, b3NegW, b3AbsW, b3AddW, b3SubW, b3MulW,
//	b3DivW, b3SqrtW, b3MulAddW, b3MinW, b3MaxW, b3SymClampW, b3AndW, b3OrW, b3AndNotW,
//	b3SoftMaskW, b3GreaterThanW, b3LessThanW, b3EqualsW, b3AllZeroW, b3AnyTrueW, b3BlendW,
//	b3EmbedIndexW, b3MinIndexW, b3TransposeW → zeroW … transposeW (simd_scalar.go)
//	b3LoadVW, b3StoreVW, b3SplatVW, b3NegVW, b3MulSVW, b3MulSubSVW, b3MulAddSVW, b3SubVW,
//	b3AddVW, b3DotW, b3CrossW → loadVW … crossW (below, from simd.h)

// wideAlignment is B3_WIDE_ALIGNMENT.
const wideAlignment = 4 * laneCount

// vec3W is b3Vec3W4: wide vec3.
type vec3W struct {
	X, Y, Z floatW
}

// loadVW is b3LoadVW4.
func loadVW(x, y, z []float32) vec3W {
	return vec3W{loadW(x), loadW(y), loadW(z)}
}

// storeVW is b3StoreVW4.
func storeVW(x, y, z []float32, v vec3W) {
	storeW(x, v.X)
	storeW(y, v.Y)
	storeW(z, v.Z)
}

// splatVW is b3SplatVW4.
func splatVW(v Vec3) vec3W {
	return vec3W{splatW(v.X), splatW(v.Y), splatW(v.Z)}
}

// negVW is b3NegVW4.
func negVW(a vec3W) vec3W {
	return vec3W{negW(a.X), negW(a.Y), negW(a.Z)}
}

// mulSVW is b3MulSVW4: s * a
func mulSVW(s floatW, a vec3W) vec3W {
	return vec3W{mulW(s, a.X), mulW(s, a.Y), mulW(s, a.Z)}
}

// mulSubSVW is b3MulSubSVW4: a - s * b
func mulSubSVW(a vec3W, s floatW, b vec3W) vec3W {
	return vec3W{subW(a.X, mulW(s, b.X)), subW(a.Y, mulW(s, b.Y)), subW(a.Z, mulW(s, b.Z))}
}

// mulAddSVW is b3MulAddSVW4: a + s * b
func mulAddSVW(a vec3W, s floatW, b vec3W) vec3W {
	return vec3W{addW(a.X, mulW(s, b.X)), addW(a.Y, mulW(s, b.Y)), addW(a.Z, mulW(s, b.Z))}
}

// subVW is b3SubVW4: a - b
func subVW(a, b vec3W) vec3W {
	return vec3W{
		subW(a.X, b.X),
		subW(a.Y, b.Y),
		subW(a.Z, b.Z),
	}
}

// addVW is b3AddVW4: a + b
func addVW(a, b vec3W) vec3W {
	return vec3W{
		addW(a.X, b.X),
		addW(a.Y, b.Y),
		addW(a.Z, b.Z),
	}
}

// dotW is b3DotW4.
func dotW(a, b vec3W) floatW {
	return addW(addW(mulW(a.X, b.X), mulW(a.Y, b.Y)), mulW(a.Z, b.Z))
}

// crossW is b3CrossW4.
func crossW(a, b vec3W) vec3W {
	var c vec3W
	c.X = subW(mulW(a.Y, b.Z), mulW(a.Z, b.Y))
	c.Y = subW(mulW(a.Z, b.X), mulW(a.X, b.Z))
	c.Z = subW(mulW(a.X, b.Y), mulW(a.Y, b.X))
	return c
}
