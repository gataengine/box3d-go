// Fixture: a width-generic kernel source.

package fixture

//go:generate go run ./tools/widegen

// vecW is a wide vector.
type vecW struct {
	v floatW
}

// lenW is a method; its name is not renamed.
func (x *vecW) lenW() floatW { return x.v }

// solveW uses a lane function and a struct field.
func solveW(x *vecW, a, b floatW) floatW {
	x.v = addW(a, b)
	scale := a // shadows the package-level scale
	y := vecW{v: scale}
	return addW(scale, y.v)
}

func helperW(s *state) floatW {
	s.wideConstraints = s.wideConstraints + 1
	return addW(scale, scale)
}
