// Generates testdata/qsort_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// cc -O2 -std=c17 -I src -o $SCRATCH/qsort_golden ../../libs/box3d-go/testdata/qsort_golden_gen.c
// $SCRATCH/qsort_golden > ../../libs/box3d-go/testdata/qsort_golden.txt
//
// QSORT is not stable, so with repeated keys the final order of the payloads depends on the exact
// sequence of LESS and SWAP calls. Each line: a case label, "|", the element count, the LESS and
// SWAP call counts and an FNV-1a hash of the payload order after sorting by key only.
#include "qsort.h"

#include <stdint.h>
#include <stdio.h>

#define MAX_COUNT 5000

static uint32_t s_seed = 0x9E3779B9u;

static uint32_t NextRandom( void )
{
	// xorshift32
	uint32_t x = s_seed;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	s_seed = x;
	return x;
}

static uint64_t s_keys[MAX_COUNT];
static uint32_t s_payloads[MAX_COUNT];
static int s_lessCount;
static int s_swapCount;

static int Less( int i, int j )
{
	s_lessCount += 1;
	return s_keys[i] < s_keys[j];
}

static void Swap( int i, int j )
{
	s_swapCount += 1;
	uint64_t k = s_keys[i];
	s_keys[i] = s_keys[j];
	s_keys[j] = k;
	uint32_t p = s_payloads[i];
	s_payloads[i] = s_payloads[j];
	s_payloads[j] = p;
}

// pattern: 0 random wide, 1 random with 8 distinct keys, 2 ascending, 3 descending, 4 all equal,
// 5 organ pipe, 6 random 64-bit pair-key like values
static void Fill( int count, int pattern )
{
	for ( int i = 0; i < count; ++i )
	{
		uint64_t key;
		switch ( pattern )
		{
			case 0:
				key = NextRandom();
				break;
			case 1:
				key = NextRandom() & 7u;
				break;
			case 2:
				key = (uint64_t)i;
				break;
			case 3:
				key = (uint64_t)( count - i );
				break;
			case 4:
				key = 42;
				break;
			case 5:
				key = (uint64_t)( i < count / 2 ? i : count - i );
				break;
			default:
			{
				uint64_t hi = NextRandom() & 0x3FFFFFu;
				uint64_t lo = NextRandom() & 0x3FFFFFu;
				key = ( hi << 42 ) | ( lo << 20 ) | ( NextRandom() & 3u );
			}
			break;
		}
		s_keys[i] = key;
		s_payloads[i] = (uint32_t)i;
	}
}

static void RunCase( int count, int pattern )
{
	Fill( count, pattern );
	s_lessCount = 0;
	s_swapCount = 0;

#define LESS( i, j ) Less( (int)( i ), (int)( j ) )
#define SWAP( i, j ) Swap( (int)( i ), (int)( j ) )
	QSORT( count, LESS, SWAP );
#undef LESS
#undef SWAP

	uint32_t hash = 2166136261u;
	for ( int i = 0; i < count; ++i )
	{
		hash ^= s_payloads[i];
		hash *= 16777619u;
	}

	printf( "pattern %d count %d | %d %d %d %08x\n", pattern, count, count, s_lessCount, s_swapCount, hash );
}

int main( void )
{
	int counts[] = { 0, 1, 2, 3, 5, 15, 16, 17, 31, 64, 100, 257, 1000, 5000 };
	int countCount = (int)( sizeof( counts ) / sizeof( counts[0] ) );
	for ( int pattern = 0; pattern < 7; ++pattern )
	{
		for ( int i = 0; i < countCount; ++i )
		{
			RunCase( counts[i], pattern );
		}
	}
	return 0;
}
