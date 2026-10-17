// Port of src/rapidhash.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// rapidhash V3 - Very fast, high quality, platform-independent hashing algorithm.
//
// Based on 'wyhash', by Wang Yi <godspeed_china@yeah.net>
//
// Copyright (C) 2025 Nicolas De Carli
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// You can contact the author at:
//   - rapidhash source repository: https://github.com/Nicoshev/rapidhash
//
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"math/bits"
)

// PORT: only the configuration upstream compiles is ported: RAPIDHASH_COMPACT (not UNROLLED)
// and RAPIDHASH_FAST (not PROTECTED). Reads are little endian on every platform, which is what
// the C byte swapping on big endian hosts achieves.

// Default secret parameters.
var rapidSecret = [8]uint64{
	0x2d358dccaa6c78a5,
	0x8bb84b93962eacc9,
	0x4b33a62ed433d4a3,
	0x4d5a2da51de1aa47,
	0xa0761d6478bd642f,
	0xe7037ed1a0b428db,
	0x90ed1765281c388c,
	0xaaaaaaaaaaaaaaaa,
}

// rapidMum is the 64*64 -> 128bit multiply function.
//
// Calculates 128-bit C = *A * *B.
// Overwrites A contents with C's low 64 bits.
// Overwrites B contents with C's high 64 bits.
func rapidMum(a, b *uint64) {
	hi, lo := bits.Mul64(*a, *b)
	*a = lo
	*b = hi
}

// rapidMix is the multiply and xor mix function.
//
// Calculates 128-bit C = A * B.
// Returns 64-bit xor between high and low 64 bits of C.
func rapidMix(a, b uint64) uint64 {
	rapidMum(&a, &b)
	return a ^ b
}

// Read functions.
func rapidRead64(p []byte) uint64 { return binary.LittleEndian.Uint64(p) }
func rapidRead32(p []byte) uint64 { return uint64(binary.LittleEndian.Uint32(p)) }

// rapidhashInternal is the rapidhash main function.
//
//	key     Buffer to be hashed.
//	seed    64-bit seed used to alter the hash result predictably.
//	secret  Triplet of 64-bit secrets used to alter hash result predictably.
//
// Returns a 64-bit hash.
func rapidhashInternal(key []byte, seed uint64, secret *[8]uint64) uint64 {
	// PORT: p is a byte offset into key; the tail reads may step back before p as in C.
	p := uint64(0)
	length := uint64(len(key))
	seed ^= rapidMix(seed^secret[2], secret[1])
	var a, b uint64
	i := length
	if length <= 16 {
		if length >= 4 {
			seed ^= length
			if length >= 8 {
				plast := key[p+length-8:]
				a = rapidRead64(key[p:])
				b = rapidRead64(plast)
			} else {
				plast := key[p+length-4:]
				a = rapidRead32(key[p:])
				b = rapidRead32(plast)
			}
		} else if length > 0 {
			a = (uint64(key[0]) << 45) | uint64(key[length-1])
			b = uint64(key[length>>1])
		} else {
			a, b = 0, 0
		}
	} else {
		if length > 112 {
			see1, see2 := seed, seed
			see3, see4 := seed, seed
			see5, see6 := seed, seed
			for {
				seed = rapidMix(rapidRead64(key[p:])^secret[0], rapidRead64(key[p+8:])^seed)
				see1 = rapidMix(rapidRead64(key[p+16:])^secret[1], rapidRead64(key[p+24:])^see1)
				see2 = rapidMix(rapidRead64(key[p+32:])^secret[2], rapidRead64(key[p+40:])^see2)
				see3 = rapidMix(rapidRead64(key[p+48:])^secret[3], rapidRead64(key[p+56:])^see3)
				see4 = rapidMix(rapidRead64(key[p+64:])^secret[4], rapidRead64(key[p+72:])^see4)
				see5 = rapidMix(rapidRead64(key[p+80:])^secret[5], rapidRead64(key[p+88:])^see5)
				see6 = rapidMix(rapidRead64(key[p+96:])^secret[6], rapidRead64(key[p+104:])^see6)
				p += 112
				i -= 112
				if i <= 112 {
					break
				}
			}
			seed ^= see1
			see2 ^= see3
			see4 ^= see5
			seed ^= see6
			see2 ^= see4
			seed ^= see2
		}
		if i > 16 {
			seed = rapidMix(rapidRead64(key[p:])^secret[2], rapidRead64(key[p+8:])^seed)
			if i > 32 {
				seed = rapidMix(rapidRead64(key[p+16:])^secret[2], rapidRead64(key[p+24:])^seed)
				if i > 48 {
					seed = rapidMix(rapidRead64(key[p+32:])^secret[1], rapidRead64(key[p+40:])^seed)
					if i > 64 {
						seed = rapidMix(rapidRead64(key[p+48:])^secret[1], rapidRead64(key[p+56:])^seed)
						if i > 80 {
							seed = rapidMix(rapidRead64(key[p+64:])^secret[2], rapidRead64(key[p+72:])^seed)
							if i > 96 {
								seed = rapidMix(rapidRead64(key[p+80:])^secret[1], rapidRead64(key[p+88:])^seed)
							}
						}
					}
				}
			}
		}
		a = rapidRead64(key[p+i-16:]) ^ i
		b = rapidRead64(key[p+i-8:])
	}
	a ^= secret[1]
	b ^= seed
	rapidMum(&a, &b)
	return rapidMix(a^secret[7], b^secret[1]^i)
}

// rapidhashMicroInternal is the rapidhashMicro main function.
func rapidhashMicroInternal(key []byte, seed uint64, secret *[8]uint64) uint64 {
	// PORT: p is a byte offset into key; the tail reads may step back before p as in C.
	p := uint64(0)
	length := uint64(len(key))
	seed ^= rapidMix(seed^secret[2], secret[1])
	var a, b uint64
	i := length
	if length <= 16 {
		if length >= 4 {
			seed ^= length
			if length >= 8 {
				plast := key[p+length-8:]
				a = rapidRead64(key[p:])
				b = rapidRead64(plast)
			} else {
				plast := key[p+length-4:]
				a = rapidRead32(key[p:])
				b = rapidRead32(plast)
			}
		} else if length > 0 {
			a = (uint64(key[0]) << 45) | uint64(key[length-1])
			b = uint64(key[length>>1])
		} else {
			a, b = 0, 0
		}
	} else {
		if i > 80 {
			see1, see2 := seed, seed
			see3, see4 := seed, seed
			for {
				seed = rapidMix(rapidRead64(key[p:])^secret[0], rapidRead64(key[p+8:])^seed)
				see1 = rapidMix(rapidRead64(key[p+16:])^secret[1], rapidRead64(key[p+24:])^see1)
				see2 = rapidMix(rapidRead64(key[p+32:])^secret[2], rapidRead64(key[p+40:])^see2)
				see3 = rapidMix(rapidRead64(key[p+48:])^secret[3], rapidRead64(key[p+56:])^see3)
				see4 = rapidMix(rapidRead64(key[p+64:])^secret[4], rapidRead64(key[p+72:])^see4)
				p += 80
				i -= 80
				if i <= 80 {
					break
				}
			}
			seed ^= see1
			see2 ^= see3
			seed ^= see4
			seed ^= see2
		}
		if i > 16 {
			seed = rapidMix(rapidRead64(key[p:])^secret[2], rapidRead64(key[p+8:])^seed)
			if i > 32 {
				seed = rapidMix(rapidRead64(key[p+16:])^secret[2], rapidRead64(key[p+24:])^seed)
				if i > 48 {
					seed = rapidMix(rapidRead64(key[p+32:])^secret[1], rapidRead64(key[p+40:])^seed)
					if i > 64 {
						seed = rapidMix(rapidRead64(key[p+48:])^secret[1], rapidRead64(key[p+56:])^seed)
					}
				}
			}
		}
		a = rapidRead64(key[p+i-16:]) ^ i
		b = rapidRead64(key[p+i-8:])
	}
	a ^= secret[1]
	b ^= seed
	rapidMum(&a, &b)
	return rapidMix(a^secret[7], b^secret[1]^i)
}

// rapidhashNanoInternal is the rapidhashNano main function.
func rapidhashNanoInternal(key []byte, seed uint64, secret *[8]uint64) uint64 {
	// PORT: p is a byte offset into key; the tail reads may step back before p as in C.
	p := uint64(0)
	length := uint64(len(key))
	seed ^= rapidMix(seed^secret[2], secret[1])
	var a, b uint64
	i := length
	if length <= 16 {
		if length >= 4 {
			seed ^= length
			if length >= 8 {
				plast := key[p+length-8:]
				a = rapidRead64(key[p:])
				b = rapidRead64(plast)
			} else {
				plast := key[p+length-4:]
				a = rapidRead32(key[p:])
				b = rapidRead32(plast)
			}
		} else if length > 0 {
			a = (uint64(key[0]) << 45) | uint64(key[length-1])
			b = uint64(key[length>>1])
		} else {
			a, b = 0, 0
		}
	} else {
		if i > 48 {
			see1, see2 := seed, seed
			for {
				seed = rapidMix(rapidRead64(key[p:])^secret[0], rapidRead64(key[p+8:])^seed)
				see1 = rapidMix(rapidRead64(key[p+16:])^secret[1], rapidRead64(key[p+24:])^see1)
				see2 = rapidMix(rapidRead64(key[p+32:])^secret[2], rapidRead64(key[p+40:])^see2)
				p += 48
				i -= 48
				if i <= 48 {
					break
				}
			}
			seed ^= see1
			seed ^= see2
		}
		if i > 16 {
			seed = rapidMix(rapidRead64(key[p:])^secret[2], rapidRead64(key[p+8:])^seed)
			if i > 32 {
				seed = rapidMix(rapidRead64(key[p+16:])^secret[2], rapidRead64(key[p+24:])^seed)
			}
		}
		a = rapidRead64(key[p+i-16:]) ^ i
		b = rapidRead64(key[p+i-8:])
	}
	a ^= secret[1]
	b ^= seed
	rapidMum(&a, &b)
	return rapidMix(a^secret[7], b^secret[1]^i)
}

// rapidhashWithSeed is the rapidhash seeded hash function.
// Calls rapidhashInternal using provided parameters and default secrets.
func rapidhashWithSeed(key []byte, seed uint64) uint64 {
	return rapidhashInternal(key, seed, &rapidSecret)
}

// rapidhash is the rapidhash general purpose hash function.
// Calls rapidhashWithSeed using provided parameters and the default seed.
func rapidhash(key []byte) uint64 {
	return rapidhashWithSeed(key, 0)
}

// rapidhashMicroWithSeed is the rapidhashMicro seeded hash function.
//
// Designed for HPC and server applications, where cache misses make a noticeable performance detriment.
// Faster for sizes up to 512 bytes, just 15%-20% slower for inputs above 1kb.
func rapidhashMicroWithSeed(key []byte, seed uint64) uint64 {
	return rapidhashMicroInternal(key, seed, &rapidSecret)
}

// rapidhashMicro is the rapidhashMicro hash function.
func rapidhashMicro(key []byte) uint64 {
	return rapidhashMicroWithSeed(key, 0)
}

// rapidhashNanoWithSeed is the rapidhashNano seeded hash function.
func rapidhashNanoWithSeed(key []byte, seed uint64) uint64 {
	return rapidhashNanoInternal(key, seed, &rapidSecret)
}

// rapidhashNano is the rapidhashNano hash function.
//
// Designed for Mobile and embedded applications, where keeping a small code size is a top priority.
// The fastest for sizes up to 48 bytes, but may be considerably slower for larger inputs.
func rapidhashNano(key []byte) uint64 {
	return rapidhashNanoWithSeed(key, 0)
}
