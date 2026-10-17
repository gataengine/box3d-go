// Fixture: the complement of the generated files' build; its names do not collide.

//go:build !(box3d_simd && goexperiment.simd && amd64 && !box3d_no8)

package fixture

type vecW8 struct{}
