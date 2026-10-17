// Go-only test: no upstream counterpart. The expected values were produced by compiling
// src/rapidhash.h from the pinned upstream commit (efca0c184d40fd6a82016983f1601c4d23965867)
// with clang and hashing the same buffer.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

var _ = Describe("RapidHash", func() {
	It("matches the C implementation", func() {
		var buf [600]byte
		for i := range buf {
			buf[i] = byte(i*131 + 7)
		}

		cases := []struct {
			n                 int
			full, micro, nano uint64
		}{
			{0, 0x0338dc4be2cecdae, 0x0338dc4be2cecdae, 0x0338dc4be2cecdae},
			{1, 0x5c2caf7d68f06d3e, 0x5c2caf7d68f06d3e, 0x5c2caf7d68f06d3e},
			{2, 0x3efd241375a7eef6, 0x3efd241375a7eef6, 0x3efd241375a7eef6},
			{3, 0x453968562b8692e9, 0x453968562b8692e9, 0x453968562b8692e9},
			{4, 0xcfa4da2476b6636e, 0xcfa4da2476b6636e, 0xcfa4da2476b6636e},
			{7, 0x891cac4198e0c8cd, 0x891cac4198e0c8cd, 0x891cac4198e0c8cd},
			{8, 0x7b06ccf26409e9bc, 0x7b06ccf26409e9bc, 0x7b06ccf26409e9bc},
			{15, 0x7ed9bfa47676d5a2, 0x7ed9bfa47676d5a2, 0x7ed9bfa47676d5a2},
			{16, 0x2a7a372c383b6b27, 0x2a7a372c383b6b27, 0x2a7a372c383b6b27},
			{17, 0xf5896810dccb51a4, 0xf5896810dccb51a4, 0xf5896810dccb51a4},
			{33, 0x169cdbdd63fdc35d, 0x169cdbdd63fdc35d, 0x169cdbdd63fdc35d},
			{49, 0xae5197cdff50d851, 0xae5197cdff50d851, 0x6272a0ec9be0a6cb},
			{65, 0x2c2352a8d74bbf9d, 0x2c2352a8d74bbf9d, 0xb0637257167cd478},
			{81, 0xf7e29cbdfe7d69ee, 0xeb628be581dc787c, 0x26f2a6c9b8ee79eb},
			{97, 0x409e000b7b6da8fc, 0xe025757667e05f57, 0x3070ccd722aeb637},
			{112, 0x710d7cc63a167a21, 0x053f57c6b46765ad, 0x6dc51911b8c29142},
			{113, 0xff65b963a2eb6fdb, 0x32e7c7665f05a1c1, 0x2771cd22ef998777},
			{200, 0x3f04dc071ef554f0, 0x8b776b0e3d1d6bb5, 0x2b659199caedecbc},
			{225, 0xe19212860084855f, 0x282d242fb5d1fca8, 0x6bd9061c558d8654},
			{300, 0xd6230aa357b15ec9, 0x8c523dd8b884ba4c, 0x54cecbc7e2a6c1e3},
			{599, 0xfe71e2419c4a1846, 0x7af893de383b818e, 0x8b934b3133a7090a},
		}

		for _, c := range cases {
			ensure(rapidhash(buf[:c.n]) == c.full)
			ensure(rapidhashMicro(buf[:c.n]) == c.micro)
			ensure(rapidhashNano(buf[:c.n]) == c.nano)
		}
	})
})
