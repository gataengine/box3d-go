// Port of test/test_height_field.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"math"
	"os"
	"path/filepath"
	"unsafe"

	. "github.com/onsi/ginkgo/v2"
)

// heightFieldBruteForceShapeCast is BruteForceShapeCast.
// Brute-force shape cast: cast the proxy against every (non-hole) triangle and
// keep the closest hit. This is the ground truth for b3ShapeCastHeightField.
func heightFieldBruteForceShapeCast(hf *HeightFieldData, input *ShapeCastInput) CastOutput {
	best := CastOutput{}
	bestFraction := input.MaxFraction

	triangleCount := getHeightFieldTriangleCount(hf)
	for t := range triangleCount {
		cellIndex := t >> 1
		if GetHeightFieldMaterialIndices(hf)[cellIndex] == HeightFieldHole {
			continue
		}

		tri := getHeightFieldTriangle(hf, t)

		pair := ShapeCastPairInput{}
		pair.ProxyA = ShapeProxy{tri.vertices[:], 3, 0.0}
		pair.ProxyB = input.Proxy
		pair.Transform = TransformIdentity
		pair.TranslationB = input.Translation
		pair.MaxFraction = bestFraction
		pair.CanEncroach = input.CanEncroach

		out := ShapeCast(&pair)
		if out.Hit && out.Fraction < bestFraction {
			bestFraction = out.Fraction
			best = out
			best.TriangleIndex = int32(t)
		}
	}

	return best
}

// heightFieldBruteForceRayCast is BruteForceRayCast.
// Brute-force ray cast: cast the ray against every (non-hole) triangle and keep
// the closest hit. b3GetHeightFieldTriangle returns vertices in the same winding
// order that b3ShapeCastHeightField feeds to b3IntersectRayTriangle, so this is a
// pure traversal/culling check. The per-triangle math is identical.
func heightFieldBruteForceRayCast(hf *HeightFieldData, input *RayCastInput) CastOutput {
	best := CastOutput{}
	bestFraction := input.MaxFraction

	rayStart := loadV(vec3Floats(&input.Origin))
	rayDelta := loadV(vec3Floats(&input.Translation))

	triangleCount := getHeightFieldTriangleCount(hf)
	for t := range triangleCount {
		cellIndex := t >> 1
		if GetHeightFieldMaterialIndices(hf)[cellIndex] == HeightFieldHole {
			continue
		}

		tri := getHeightFieldTriangle(hf, t)
		v1 := loadV(vec3Floats(&tri.vertices[0]))
		v2 := loadV(vec3Floats(&tri.vertices[1]))
		v3 := loadV(vec3Floats(&tri.vertices[2]))

		// b3IntersectRayTriangle returns 1.0 on a miss.
		alpha := intersectRayTriangle(rayStart, rayDelta, v1, v2, v3)
		if alpha < bestFraction {
			bestFraction = alpha
			best.Hit = true
			best.Fraction = alpha
			best.TriangleIndex = int32(t)
		}
	}

	return best
}

// makeFlatField is MakeFlatField (also defined, identically, in test_mover.c).
// Flat 3x3 vertex field at y = 0, one cell material per entry. Caller destroys.
func makeFlatField(materials []uint8, clockwise bool) *HeightFieldData {
	var heights [9]float32

	def := HeightFieldDef{}
	def.Heights = heights[:]
	def.MaterialIndices = materials
	def.Scale = Vec3{1.0, 1.0, 1.0}
	def.CountX = 3
	def.CountZ = 3
	def.GlobalMinimumHeight = -1.0
	def.GlobalMaximumHeight = 1.0
	def.ClockwiseWinding = clockwise

	return CreateHeightField(&def)
}

// pointProxy is ( b3ShapeProxy ){ &point, 1, radius }.
func pointProxy(point *Vec3, radius float32) ShapeProxy {
	return ShapeProxy{unsafe.Slice(point, 1), 1, radius}
}

var _ = Describe("HeightFieldTest", func() {
	It("HeightFieldCreate", func() {
		scale := Vec3{1.0, 1.0, 1.0}
		hf := CreateGrid(4, 4, scale, false)

		ensure(hf.RowCount == 4)
		ensure(hf.ColumnCount == 4)
		ensure(hf.Clockwise == 0)
		ensure(hf.Version == HeightFieldVersion)

		ensureSmall(hf.AABB.LowerBound.X, fltEpsilon)
		ensureSmall(hf.AABB.LowerBound.Y, fltEpsilon)
		ensureSmall(hf.AABB.LowerBound.Z, fltEpsilon)

		ensureSmall(hf.AABB.UpperBound.X-3.0, fltEpsilon)
		ensureSmall(hf.AABB.UpperBound.Y, fltEpsilon)
		ensureSmall(hf.AABB.UpperBound.Z-3.0, fltEpsilon)

		DestroyHeightField(hf)
	})

	It("HeightFieldTriangleIndex", func() {
		// Asymmetric grid (rows != cols) catches off-by-one errors between
		// vertex stride (columnCount) and cell stride (columnCount - 1).
		rowCount := 4
		columnCount := 5
		scale := Vec3{1.0, 1.0, 1.0}
		hf := CreateGrid(rowCount, columnCount, scale, false)

		triangleCount := 2 * (rowCount - 1) * (columnCount - 1)

		for triangleIndex := range triangleCount {
			quadIndex := triangleIndex >> 1
			sub := triangleIndex & 1
			row := quadIndex / (columnCount - 1)
			column := quadIndex - row*(columnCount-1)

			index11 := int32(row*columnCount + column)
			index12 := index11 + 1
			index21 := int32((row+1)*columnCount + column)
			index22 := index21 + 1

			t := getHeightFieldTriangle(hf, triangleIndex)

			if sub == 0 {
				// Triangle 0 (CCW): {11, 21, 12}
				ensure(t.i1 == index11)
				ensure(t.i2 == index21)
				ensure(t.i3 == index12)
			} else {
				// Triangle 1 (CCW): {22, 12, 21}
				ensure(t.i1 == index22)
				ensure(t.i2 == index12)
				ensure(t.i3 == index21)
			}
		}

		DestroyHeightField(hf)
	})

	It("HeightFieldWinding", func() {
		// Build the same flat 3x3 field with CCW and CW winding. The cross-product
		// normal of triangle 0 must flip sign accordingly.
		var heights [9]float32
		materials := [4]uint8{0, 0, 0, 0}

		def := HeightFieldDef{}
		def.Heights = heights[:]
		def.MaterialIndices = materials[:]
		def.Scale = Vec3{1.0, 1.0, 1.0}
		def.CountX = 3
		def.CountZ = 3
		def.GlobalMinimumHeight = -1.0
		def.GlobalMaximumHeight = 1.0

		def.ClockwiseWinding = false
		ccw := CreateHeightField(&def)

		def.ClockwiseWinding = true
		cw := CreateHeightField(&def)

		ta := getHeightFieldTriangle(ccw, 0)
		tb := getHeightFieldTriangle(cw, 0)

		na :=
			Normalize(Cross(Sub(ta.vertices[1], ta.vertices[0]), Sub(ta.vertices[2], ta.vertices[0])))
		nb :=
			Normalize(Cross(Sub(tb.vertices[1], tb.vertices[0]), Sub(tb.vertices[2], tb.vertices[0])))

		ensureSmall(na.X, fltEpsilon)
		ensureSmall(na.Y-1.0, fltEpsilon)
		ensureSmall(na.Z, fltEpsilon)

		ensureSmall(nb.X, fltEpsilon)
		ensureSmall(nb.Y+1.0, fltEpsilon)
		ensureSmall(nb.Z, fltEpsilon)

		DestroyHeightField(ccw)
		DestroyHeightField(cw)
	})

	It("RayCastFlatField", func() {
		// Build a flat 4x4 field with a tight quantization range so the recovered
		// surface stays within ~1e-5 of y=0 (b3CreateGrid uses -256..256 which
		// blows the 1/UINT16_MAX quantum up to ~4e-3 in y).
		var heights [16]float32
		var materials [9]uint8

		def := HeightFieldDef{}
		def.Heights = heights[:]
		def.MaterialIndices = materials[:]
		def.Scale = Vec3{1.0, 1.0, 1.0}
		def.CountX = 4
		def.CountZ = 4
		def.GlobalMinimumHeight = -1.0
		def.GlobalMaximumHeight = 1.0
		def.ClockwiseWinding = false

		hf := CreateHeightField(&def)

		// Origin sits clearly inside triangle 0 of cell (1, 1). Off the cell
		// diagonal x+z = 3. The translation overshoots the surface so the hit
		// fraction is strictly less than maxFraction.
		input := RayCastInput{}
		input.Origin = Vec3{1.25, 10.0, 1.25}
		input.Translation = Vec3{0.0, -20.0, 0.0}
		input.MaxFraction = 1.0

		out := RayCastHeightField(hf, &input)

		ensure(out.Hit == true)
		ensureSmall(out.Fraction-0.5, 1e-5)
		ensureSmall(out.Normal.X, 1e-5)
		ensureSmall(out.Normal.Y-1.0, 1e-5)
		ensureSmall(out.Normal.Z, 1e-5)

		DestroyHeightField(hf)
	})

	It("OverlapAtSurface", func() {
		scale := Vec3{1.0, 1.0, 1.0}
		hf := CreateGrid(4, 4, scale, false)

		// Sphere center 1.0 above the surface, radius 0.5. Clear gap.
		above := Vec3{1.5, 1.0, 1.5}
		proxyAbove := pointProxy(&above, 0.5)
		hitAbove := OverlapHeightField(hf, TransformIdentity, &proxyAbove)
		ensure(hitAbove == false)

		// Sphere centered on the surface. Radius pokes through.
		through := Vec3{1.5, 0.0, 1.5}
		proxyThrough := pointProxy(&through, 0.5)
		hitThrough := OverlapHeightField(hf, TransformIdentity, &proxyThrough)
		ensure(hitThrough == true)

		DestroyHeightField(hf)
	})

	It("FileRoundtrip", func() {
		heights := [9]float32{0.0, 0.5, -0.3, 0.1, 0.0, 0.0, 0.0, 0.2, 0.0}
		materials := [4]uint8{0, HeightFieldHole, 1, 2}

		def := HeightFieldDef{}
		def.Heights = heights[:]
		def.MaterialIndices = materials[:]
		def.Scale = Vec3{1.5, 2.0, 0.75}
		def.CountX = 3
		def.CountZ = 3
		def.GlobalMinimumHeight = -1.0
		def.GlobalMaximumHeight = 1.0
		def.ClockwiseWinding = true

		// PORT: the file goes into a test temporary directory instead of the working directory.
		path := filepath.Join(GinkgoT().TempDir(), "test_height_field_roundtrip.dat")
		DumpHeightData(&def, path)

		loaded := LoadHeightField(path)
		os.Remove(path)

		ensure(loaded != nil)
		ensure(loaded.RowCount == def.CountZ)
		ensure(loaded.ColumnCount == def.CountX)
		ensure((loaded.Clockwise != 0) == def.ClockwiseWinding)

		ensureSmall(loaded.Scale.X-def.Scale.X, fltEpsilon)
		ensureSmall(loaded.Scale.Y-def.Scale.Y, fltEpsilon)
		ensureSmall(loaded.Scale.Z-def.Scale.Z, fltEpsilon)
		ensureSmall(loaded.MinHeight-def.GlobalMinimumHeight, fltEpsilon)
		ensureSmall(loaded.MaxHeight-def.GlobalMaximumHeight, fltEpsilon)

		cellCount := int((def.CountX - 1) * (def.CountZ - 1))
		for i := range cellCount {
			ensure(GetHeightFieldMaterialIndices(loaded)[i] == materials[i])
		}

		// Recovered heights round-trip within the quantization tolerance.
		quantum := (def.GlobalMaximumHeight - def.GlobalMinimumHeight) / float32(math.MaxUint16)
		for i := range int(def.CountX * def.CountZ) {
			recovered := mulAdd(loaded.MinHeight, loaded.HeightScale, float32(GetHeightFieldCompressedHeights(loaded)[i]))
			ensureSmall(recovered-heights[i], 2.0*quantum)
		}

		DestroyHeightField(loaded)
	})

	It("ShapeCastVerticalStraddle", func() {
		// Regression: a vertical shape cast whose swept volume straddles a cell
		// boundary must test every cell it overlaps. The field is flat at y = 0 with
		// only cell (0,0) solid; the surrounding cells are holes. Each sphere is
		// dropped straight down with its center nudged just past a boundary of the
		// solid cell, so that cell sits on the trailing (-x / -z) side of the sweep.
		// A cull AABB pinned to the leading box corner skips the solid cell entirely
		// and reports a miss.
		var heights [9]float32
		materials := [4]uint8{0, HeightFieldHole, HeightFieldHole, HeightFieldHole}

		def := HeightFieldDef{}
		def.Heights = heights[:]
		def.MaterialIndices = materials[:]
		def.Scale = Vec3{1.0, 1.0, 1.0}
		def.CountX = 3
		def.CountZ = 3
		def.GlobalMinimumHeight = -1.0
		def.GlobalMaximumHeight = 1.0
		def.ClockwiseWinding = false

		hf := CreateHeightField(&def)

		// Solid cell (0,0) spans x,z in [0,1]. Radius 0.3 with the center 0.05 past a
		// boundary still reaches back into the solid cell.
		const radius float32 = 0.3

		// Straddle the x = 1 edge: solid cell is on the -x side. Contact lands on the
		// cell edge, sqrt(0.05^2 + cy^2) = radius -> cy = 0.2958040,
		// fraction = (10 - cy) / 20 = 0.4852098.
		{
			center := Vec3{1.05, 10.0, 0.5}
			input := ShapeCastInput{}
			input.Proxy = pointProxy(&center, radius)
			input.Translation = Vec3{0.0, -20.0, 0.0}
			input.MaxFraction = 1.0

			out := ShapeCastHeightField(hf, &input)
			ensure(out.Hit == true)
			ensureSmall(out.Fraction-0.4852098, 2e-3)
		}

		// Straddle the z = 1 edge: solid cell is on the -z side (same geometry).
		{
			center := Vec3{0.5, 10.0, 1.05}
			input := ShapeCastInput{}
			input.Proxy = pointProxy(&center, radius)
			input.Translation = Vec3{0.0, -20.0, 0.0}
			input.MaxFraction = 1.0

			out := ShapeCastHeightField(hf, &input)
			ensure(out.Hit == true)
			ensureSmall(out.Fraction-0.4852098, 2e-3)
		}

		// Straddle the (1,1) corner: solid cell is diagonally trailing. Contact lands
		// on the corner vertex, sqrt(2*0.05^2 + cy^2) = radius -> cy = 0.2915476,
		// fraction = (10 - cy) / 20 = 0.4854226.
		{
			center := Vec3{1.05, 10.0, 1.05}
			input := ShapeCastInput{}
			input.Proxy = pointProxy(&center, radius)
			input.Translation = Vec3{0.0, -20.0, 0.0}
			input.MaxFraction = 1.0

			out := ShapeCastHeightField(hf, &input)
			ensure(out.Hit == true)
			ensureSmall(out.Fraction-0.4854226, 2e-3)
		}

		DestroyHeightField(hf)
	})

	It("ShapeCastBruteForce", func() {
		// b3ShapeCastHeightField walks the grid and culls cells; the brute-force cast
		// against every triangle is the ground truth. The grid walk must never miss a
		// closer hit, regardless of cast direction, origin or radius.
		scale := Vec3{2.0, 1.5, 2.0}
		hf := CreateWave(10, 10, scale, 0.1, 0.03333, false)

		// Documented repro from sample/sample_mesh.cpp "Height Field": a sphere cast
		// that moves only in z (and y). Body at (-9,0,-9), world origin (5.5,4,2.913)
		// -> local (14.5,4,11.913). The grid walk used to terminate one row early
		// because it compared a clamped-sweep fraction against an input-space one.
		{
			origin := Vec3{14.5, 4.0, 11.913}
			input := ShapeCastInput{}
			input.Proxy = pointProxy(&origin, 0.2)
			input.Translation = Vec3{0.0, -8.0, 6.397}
			input.MaxFraction = 1.0

			grid := ShapeCastHeightField(hf, &input)
			brute := heightFieldBruteForceShapeCast(hf, &input)
			ensure(brute.Hit == true)
			ensure(grid.Hit == brute.Hit)
			ensureSmall(grid.Fraction-brute.Fraction, 2e-3)
		}

		// Sweep origins across the field with assorted directions and radii.
		radii := []float32{0.15, 0.4, 0.9}
		deltas := []Vec3{
			{0.0, -8.0, 0.0},  // vertical
			{0.0, -8.0, 6.4},  // z only (+ y)
			{5.1, -8.0, 0.0},  // x only (+ y)
			{0.0, -8.0, -6.4}, // -z
			{-5.1, -8.0, 0.0}, // -x
			{6.0, -8.0, 5.0},  // diagonal
			{-7.0, -8.0, 4.0}, // diagonal, mixed sign
			{9.0, -3.0, -9.0}, // shallow diagonal
		}

		failures := 0
		for xi := range 5 {
			for zi := range 5 {
				// 0.05 nudge keeps the swept box straddling cell boundaries.
				origin := Vec3{1.0 + 4.0*float32(xi) + 0.05, 4.0, 1.0 + 4.0*float32(zi) + 0.05}

				for di := range arrayCount(deltas) {
					for ri := range arrayCount(radii) {
						input := ShapeCastInput{}
						input.Proxy = pointProxy(&origin, radii[ri])
						input.Translation = deltas[di]
						input.MaxFraction = 1.0

						grid := ShapeCastHeightField(hf, &input)
						brute := heightFieldBruteForceShapeCast(hf, &input)

						diff := grid.Fraction - brute.Fraction
						if diff < 0.0 {
							diff = -diff
						}

						if grid.Hit != brute.Hit || (brute.Hit && diff > 2e-3) {
							GinkgoWriter.Printf("  mismatch: origin=(%.2f,%.2f,%.2f) delta=(%.2f,%.2f,%.2f) r=%.2f"+
								" grid(hit=%v,f=%.5f) brute(hit=%v,f=%.5f tri=%d)\n",
								origin.X, origin.Y, origin.Z, deltas[di].X, deltas[di].Y, deltas[di].Z, radii[ri],
								grid.Hit, grid.Fraction, brute.Hit, brute.Fraction, brute.TriangleIndex)
							failures += 1
						}
					}
				}
			}
		}

		ensure(failures == 0)

		DestroyHeightField(hf)
	})

	It("RayCastBruteForce", func() {
		// b3RayCastHeightField routes through the same grid walk as the shape cast
		// (radius-zero point proxy), so it is subject to the same early-termination
		// bug. The brute-force cast against every triangle is the ground truth.
		scale := Vec3{2.0, 1.5, 2.0}
		hf := CreateWave(10, 10, scale, 0.1, 0.03333, false)

		deltas := []Vec3{
			{0.0, -8.0, 0.0},    // straight down
			{0.0, -8.0, 12.0},   // down + z
			{12.0, -8.0, 0.0},   // down + x
			{0.0, -8.0, -12.0},  // down - z
			{-12.0, -8.0, 0.0},  // down - x
			{14.0, -8.0, 11.0},  // diagonal
			{-13.0, -8.0, 9.0},  // diagonal, mixed sign
			{16.0, -4.0, -15.0}, // shallow diagonal
		}

		failures := 0
		for xi := range 5 {
			for zi := range 5 {
				// 0.05 nudge keeps the ray off cell boundaries.
				origin := Vec3{1.0 + 4.0*float32(xi) + 0.05, 4.0, 1.0 + 4.0*float32(zi) + 0.05}

				for di := range arrayCount(deltas) {
					input := RayCastInput{}
					input.Origin = origin
					input.Translation = deltas[di]
					input.MaxFraction = 1.0

					grid := RayCastHeightField(hf, &input)
					brute := heightFieldBruteForceRayCast(hf, &input)

					diff := grid.Fraction - brute.Fraction
					if diff < 0.0 {
						diff = -diff
					}

					if grid.Hit != brute.Hit || (brute.Hit && diff > 1e-4) {
						GinkgoWriter.Printf("  mismatch: origin=(%.2f,%.2f,%.2f) delta=(%.2f,%.2f,%.2f)"+
							" grid(hit=%v,f=%.6f tri=%d) brute(hit=%v,f=%.6f tri=%d)\n",
							origin.X, origin.Y, origin.Z, deltas[di].X, deltas[di].Y, deltas[di].Z, grid.Hit,
							grid.Fraction, grid.TriangleIndex, brute.Hit, brute.Fraction, brute.TriangleIndex)
						failures += 1
					}
				}
			}
		}

		ensure(failures == 0)

		DestroyHeightField(hf)
	})

	It("ShapeCastBackside", func() {
		materials := [4]uint8{1, 0, 0, 0}
		hf := makeFlatField(materials[:], false)

		// Falling onto the front (upper) face reports the hit, the triangle, and its material.
		// The surface sits at y = 0, so a sphere of radius 0.2 stops with its center at y = 0.2.
		above := Vec3{0.3, 5.0, 0.25}
		down := ShapeCastInput{}
		down.Proxy = pointProxy(&above, 0.2)
		down.Translation = Vec3{0.0, -10.0, 0.0}
		down.MaxFraction = 1.0

		out := ShapeCastHeightField(hf, &down)
		ensure(out.Hit == true)
		ensureSmall(out.Fraction-0.48, 1e-2)
		ensure(out.Normal.Y > 0.99)
		ensure(out.TriangleIndex == 0 || out.TriangleIndex == 1)
		ensure(out.MaterialIndex == 1)

		// Rising from behind the face is culled
		below := Vec3{0.3, -5.0, 0.25}
		up := ShapeCastInput{}
		up.Proxy = pointProxy(&below, 0.2)
		up.Translation = Vec3{0.0, 10.0, 0.0}
		up.MaxFraction = 1.0

		missed := ShapeCastHeightField(hf, &up)
		ensure(missed.Hit == false)

		DestroyHeightField(hf)
	})

	// A clockwise height field faces down, see HeightFieldWinding. Shape cast backside
	// culling must respect the flag: a rising sphere hits from below, not from above.
	It("ShapeCastClockwiseWinding", func() {
		materials := [4]uint8{1, 0, 0, 0}
		hf := makeFlatField(materials[:], true)

		below := Vec3{0.3, -5.0, 0.25}
		up := ShapeCastInput{}
		up.Proxy = pointProxy(&below, 0.2)
		up.Translation = Vec3{0.0, 10.0, 0.0}
		up.MaxFraction = 1.0

		out := ShapeCastHeightField(hf, &up)
		ensure(out.Hit == true)
		ensureSmall(out.Fraction-0.48, 1e-2)
		ensure(out.Normal.Y < -0.99)
		ensure(out.TriangleIndex == 0 || out.TriangleIndex == 1)
		ensure(out.MaterialIndex == 1)

		above := Vec3{0.3, 5.0, 0.25}
		down := ShapeCastInput{}
		down.Proxy = pointProxy(&above, 0.2)
		down.Translation = Vec3{0.0, -10.0, 0.0}
		down.MaxFraction = 1.0

		missed := ShapeCastHeightField(hf, &down)
		ensure(missed.Hit == false)

		DestroyHeightField(hf)
	})

	// Ray casts run through the shape cast. The default winding faces up, so a ray from below is the
	// back side and gets culled.
	It("RayCastBackside", func() {
		materials := [4]uint8{1, 0, 0, 0}
		hf := makeFlatField(materials[:], false)

		down := RayCastInput{}
		down.Origin = Vec3{0.3, 5.0, 0.25}
		down.Translation = Vec3{0.0, -10.0, 0.0}
		down.MaxFraction = 1.0

		out := RayCastHeightField(hf, &down)
		ensure(out.Hit == true)
		ensureSmall(out.Fraction-0.5, 1e-3)
		ensure(out.Normal.Y > 0.99)
		ensure(out.TriangleIndex == 0 || out.TriangleIndex == 1)
		ensure(out.MaterialIndex == 1)

		up := RayCastInput{}
		up.Origin = Vec3{0.3, -5.0, 0.25}
		up.Translation = Vec3{0.0, 10.0, 0.0}
		up.MaxFraction = 1.0

		missed := RayCastHeightField(hf, &up)
		ensure(missed.Hit == false)

		DestroyHeightField(hf)
	})

	// A clockwise field faces down. The winding swap already flips the triangle, so the ray normal
	// must not be flipped a second time.
	It("RayCastClockwiseWinding", func() {
		materials := [4]uint8{1, 0, 0, 0}
		hf := makeFlatField(materials[:], true)

		up := RayCastInput{}
		up.Origin = Vec3{0.3, -5.0, 0.25}
		up.Translation = Vec3{0.0, 10.0, 0.0}
		up.MaxFraction = 1.0

		out := RayCastHeightField(hf, &up)
		ensure(out.Hit == true)
		ensureSmall(out.Fraction-0.5, 1e-3)
		ensure(out.Normal.Y < -0.99)
		ensure(out.TriangleIndex == 0 || out.TriangleIndex == 1)
		ensure(out.MaterialIndex == 1)

		down := RayCastInput{}
		down.Origin = Vec3{0.3, 5.0, 0.25}
		down.Translation = Vec3{0.0, -10.0, 0.0}
		down.MaxFraction = 1.0

		missed := RayCastHeightField(hf, &down)
		ensure(missed.Hit == false)

		DestroyHeightField(hf)
	})
})
