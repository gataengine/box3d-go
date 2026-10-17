// Port of src/simd.h and src/simd.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.
//
// Dirk Gregorius contributed portions of this code

package box3d

// PORT: the b3V32 helpers below are the non-SSE2 branch of simd.h and simd.c in every build (see
// simd_aabb.go for the same choice on b3AABBV); the SSE2 branch computes the same values.

// ---- simd.h ----

// I don't expect the use case of b3V32 to benefit from Neon code.
// In particular the cross product is very complex in Neon.

// v32 is b3V32.
type v32 struct {
	x, y, z float32
}

// PORT: the b3128 union (b3V32 viewed as float[3]) is replaced by getV's switch.

// zeroVConst, halfVConst and oneVConst are b3_zeroV, b3_halfV and b3_oneV.
var (
	zeroVConst = v32{0.0, 0.0, 0.0}
	halfVConst = v32{0.5, 0.5, 0.5}
	oneVConst  = v32{1.0, 1.0, 1.0}
)

func addV(a, b v32) v32 {
	return v32{
		a.x + b.x,
		a.y + b.y,
		a.z + b.z,
	}
}

func subV(a, b v32) v32 {
	return v32{
		a.x - b.x,
		a.y - b.y,
		a.z - b.z,
	}
}

func mulV(a, b v32) v32 {
	return v32{
		float32(a.x * b.x),
		float32(a.y * b.y),
		float32(a.z * b.z),
	}
}

func divV(a, b v32) v32 {
	return v32{
		a.x / b.x,
		a.y / b.y,
		a.z / b.z,
	}
}

func negV(a v32) v32 {
	return v32{
		-a.x,
		-a.y,
		-a.z,
	}
}

// loadV is b3LoadV. Unaligned loads are much faster on recent hardware with little to no penalty
func loadV(src []float32) v32 {
	return v32{src[0], src[1], src[2]}
}

func zeroV() v32 {
	return v32{0.0, 0.0, 0.0}
}

func getXV(a v32) float32 {
	return a.x
}

func getYV(a v32) float32 {
	return a.y
}

func getZV(a v32) float32 {
	return a.z
}

func getV(a v32, index int) float32 {
	switch index {
	case 0:
		return a.x
	case 1:
		return a.y
	default:
		return a.z
	}
}

func splatV(x float32) v32 {
	return v32{x, x, x}
}

// absLane is the ternary a < 0.0f ? -a : a used by b3AbsV (and by b3AbsW4 in
// simd_scalar.go).
func absLane(a float32) float32 {
	if a < 0.0 {
		return -a
	}
	return a
}

func absV(a v32) v32 {
	return v32{
		absLane(a.x),
		absLane(a.y),
		absLane(a.z),
	}
}

func minLaneV(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func maxLaneV(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

func minV(a, b v32) v32 {
	return v32{
		minLaneV(a.x, b.x),
		minLaneV(a.y, b.y),
		minLaneV(a.z, b.z),
	}
}

func maxV(a, b v32) v32 {
	return v32{
		maxLaneV(a.x, b.x),
		maxLaneV(a.y, b.y),
		maxLaneV(a.z, b.z),
	}
}

func crossV(a, b v32) v32 {
	var c v32
	c.x = float32(a.y*b.z) - float32(a.z*b.y)
	c.y = float32(a.z*b.x) - float32(a.x*b.z)
	c.z = float32(a.x*b.y) - float32(a.y*b.x)
	return c
}

func modifiedCrossV(a, b v32) v32 {
	var c v32
	c.x = float32(a.y*b.z) + float32(a.z*b.y)
	c.y = float32(a.z*b.x) + float32(a.x*b.z)
	c.z = float32(a.x*b.y) + float32(a.y*b.x)
	return c
}

func anyLess3V(a, b v32) bool {
	return a.x < b.x || a.y < b.y || a.z < b.z
}

func anyLessEq3V(a, b v32) bool {
	return a.x <= b.x || a.y <= b.y || a.z <= b.z
}

func anyGreater3V(a, b v32) bool {
	return a.x > b.x || a.y > b.y || a.z > b.z
}

func allLessEq3V(a, b v32) bool {
	return a.x <= b.x && a.y <= b.y && a.z <= b.z
}

// GetSIMDWidth and SetSIMDWidth are implemented below with simd.c.

// PORT: the b3Vec3W4 type and functions are in simd_wide.go, next to the width-neutral names; the
// b3Vec3W8 functions (B3_SIMD_AVX2) are not ported.

// testBoundsOverlap is b3TestBoundsOverlap.
func testBoundsOverlap(nodeMin1, nodeMax1, nodeMin2, nodeMax2 v32) bool {
	separation := maxV(subV(nodeMin2, nodeMax1), subV(nodeMin1, nodeMax2))
	return allLessEq3V(separation, zeroVConst)
}

// testBoundsRayOverlap is b3TestBoundsRayOverlap: test a ray for edge separation with an AABB (Gino, p80).
func testBoundsRayOverlap(nodeMin, nodeMax, rayStart, rayDelta v32) bool {
	// Setup node
	nodeCenter := mulV(halfVConst, addV(nodeMin, nodeMax))
	nodeExtent := subV(nodeMax, nodeCenter)

	// Setup ray
	rayStart = subV(rayStart, nodeCenter)

	// SAT: Edge separation
	edgeSeparation := subV(absV(crossV(rayDelta, rayStart)), modifiedCrossV(absV(rayDelta), nodeExtent))
	return allLessEq3V(edgeSeparation, zeroVConst)
}

// testBoundsTriangleOverlap and intersectRayTriangle are implemented below with simd.c.

// unionV is b3UnionV.
func unionV(a, b AABB) AABB {
	var result AABB
	storeAABBV(&result, unionAABBV(loadAABBV(&a), loadAABBV(&b)), true)
	return result
}

// ---- simd.c ----

var simdWidth atomicInt

// PORT: cpuid/xgetbv AVX2 detection (B3_SIMD_HAS_WIDTH_8, b3HasAVX2) is not ported here: the
// width 8 backend registers its kernels in wide8 (wide_kernels.go) from its init only when the CPU
// has AVX2, so wide8 != nil stands for B3_SIMD_HAS_WIDTH_8 && b3HasAVX2(). Without it the width is
// laneCount.

// detectSIMDWidth is b3DetectSIMDWidth.
func detectSIMDWidth() int {
	if wide8 != nil {
		return 8
	}

	return laneCount
}

// GetSIMDWidth gets the SIMD width used by the solver.
func GetSIMDWidth() int {
	width := int(atomicLoadInt(&simdWidth))
	if width == 0 {
		width = detectSIMDWidth()
		atomicStoreInt(&simdWidth, int32(width))
	}

	return width
}

// IsAVX2Available returns true if the CPU supports AVX2.
// PORT: true when the width 8 backend is built in and registered (see detectSIMDWidth).
func IsAVX2Available() bool {
	return detectSIMDWidth() == 8
}

// SetSIMDWidth sets the SIMD width. Use 0 to auto-detect.
// PORT: width 8 falls back to 4 without the width 8 backend, as on a CPU without AVX2.
func SetSIMDWidth(width int) {
	assert(width == 0 || width == 4 || width == 8, "width == 0 || width == 4 || width == 8")
	if width == 8 && detectSIMDWidth() != 8 {
		width = 4
	}

	atomicStoreInt(&simdWidth, int32(width))
}

// transpose3 is B3_TRANSPOSE3.
func transpose3(c1, c2, c3 *v32) {
	temp1 := c1.y
	temp2 := c1.z
	temp3 := c2.z

	c1.y = c2.x
	c1.z = c3.x
	c2.z = c3.y

	c2.x = temp1
	c3.x = temp2
	c3.y = temp3
}

func splatXV(a v32) v32 {
	return v32{a.x, a.x, a.x}
}

func splatYV(a v32) v32 {
	return v32{a.y, a.y, a.y}
}

func splatZV(a v32) v32 {
	return v32{a.z, a.z, a.z}
}

func anyGreaterEq3V(a, b v32) bool {
	return a.x >= b.x || a.y >= b.y || a.z >= b.z
}

func dot3V(a, b v32) v32 {
	d := float32(a.x*b.x) + float32(a.y*b.y) + float32(a.z*b.z)
	return v32{d, d, d}
}

func testBoundsTriangleOverlap(nodeCenter, nodeExtent, vertex1, vertex2, vertex3 v32) bool {
	two := splatV(2.0)

	// Setup triangle
	vertex1 = subV(vertex1, nodeCenter)
	vertex2 = subV(vertex2, nodeCenter)
	vertex3 = subV(vertex3, nodeCenter)

	// Face separation
	triangleMin := minV(vertex1, minV(vertex2, vertex3))
	triangleMax := maxV(vertex1, maxV(vertex2, vertex3))

	separation1 := subV(triangleMin, nodeExtent)
	separation2 := addV(triangleMax, nodeExtent)

	faceSeparation := maxV(separation1, negV(separation2))
	if anyGreater3V(faceSeparation, zeroVConst) {
		return false
	}

	// SAT: Face separation
	edge1 := subV(vertex2, vertex1)
	edge2 := subV(vertex3, vertex2)
	edge3 := subV(vertex1, vertex3)

	normal := crossV(edge1, edge2)

	triangleSeparation := subV(absV(dot3V(normal, vertex1)), dot3V(absV(normal), nodeExtent))
	if anyGreater3V(triangleSeparation, zeroVConst) {
		return false
	}

	// SAT: Edge separation
	edgeSeparation1 := subV(subV(absV(crossV(edge1, addV(vertex1, vertex3))), absV(crossV(edge1, edge3))),
		mulV(two, modifiedCrossV(absV(edge1), nodeExtent)))
	if anyGreater3V(edgeSeparation1, zeroVConst) {
		return false
	}

	edgeSeparation2 := subV(subV(absV(crossV(edge2, addV(vertex1, vertex2))), absV(crossV(edge2, edge1))),
		mulV(two, modifiedCrossV(absV(edge2), nodeExtent)))
	if anyGreater3V(edgeSeparation2, zeroVConst) {
		return false
	}

	edgeSeparation3 := subV(subV(absV(crossV(edge3, addV(vertex2, vertex3))), absV(crossV(edge3, edge2))),
		mulV(two, modifiedCrossV(absV(edge3), nodeExtent)))
	if anyGreater3V(edgeSeparation3, zeroVConst) {
		return false
	}

	return true
}

func intersectRayTriangle(rayStart, rayDelta, vertex1, vertex2, vertex3 v32) float32 {
	// Test if ray intersects this triangle sharing same calculations for each triangle
	{
		edge1 := subV(vertex3, vertex2)
		edge2 := subV(vertex1, vertex3)
		edge3 := subV(vertex2, vertex1)

		midPoint1 := mulV(halfVConst, addV(vertex2, vertex3))
		midPoint2 := mulV(halfVConst, addV(vertex3, vertex1))
		midPoint3 := mulV(halfVConst, addV(vertex1, vertex2))

		normal1 := crossV(edge1, subV(midPoint1, rayStart))
		normal2 := crossV(edge2, subV(midPoint2, rayStart))
		normal3 := crossV(edge3, subV(midPoint3, rayStart))
		transpose3(&normal1, &normal2, &normal3)

		rayDeltaX := splatXV(rayDelta)
		rayDeltaY := splatYV(rayDelta)
		rayDeltaZ := splatZV(rayDelta)

		volumes := addV(addV(mulV(normal1, rayDeltaX), mulV(normal2, rayDeltaY)), mulV(normal3, rayDeltaZ))
		if anyLess3V(volumes, zeroVConst) {
			return 1.0
		}
	}

	// Compute intersection with triangle plane
	edge1 := subV(vertex2, vertex1)
	edge2 := subV(vertex3, vertex1)
	normal := crossV(edge1, edge2)

	denominator := dot3V(normal, rayDelta)
	if anyGreaterEq3V(denominator, zeroVConst) {
		return 1.0
	}

	lambda := divV(dot3V(normal, subV(vertex1, rayStart)), denominator)
	if anyLessEq3V(lambda, zeroVConst) {
		return 1.0
	}

	lambda = minV(lambda, oneVConst)
	return getXV(lambda)
}
