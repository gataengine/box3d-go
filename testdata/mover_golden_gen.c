// Generates testdata/mover_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository:
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/mover_golden \
//    ../../libs/box3d-go/testdata/mover_golden_gen.c src/mover.c src/math_functions.c src/core.c
// $SCRATCH/mover_golden > ../../libs/box3d-go/testdata/mover_golden.txt
//
// Each line: case index, "|", b3SolvePlanes delta, iteration count, every plane push, then
// b3ClipVector of a velocity, as hex words. Inputs come from an LCG with no libm calls (the
// normals go through b3Normalize, whose sqrt is correctly rounded), so Go reproduces them exactly.
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_seed = 4321;

static uint32_t NextRandom( void )
{
	s_seed = 1664525u * s_seed + 1013904223u;
	return s_seed;
}

static float RandomFloat( float lo, float hi )
{
	float t = (float)( NextRandom() >> 8 ) / (float)( 1 << 24 );
	return lo + t * ( hi - lo );
}

// Sequenced: C leaves the evaluation order of initializer lists unspecified.
static b3Vec3 RandomVec( float lo, float hi )
{
	b3Vec3 v;
	v.x = RandomFloat( lo, hi );
	v.y = RandomFloat( lo, hi );
	v.z = RandomFloat( lo, hi );
	return v;
}

static uint32_t B( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	return u;
}

static void pf( float f )
{
	printf( " %08x", B( f ) );
}

static void pv( b3Vec3 v )
{
	pf( v.x );
	pf( v.y );
	pf( v.z );
}

int main( void )
{
	b3CollisionPlane planes[8];

	for ( int i = 0; i < 200; ++i )
	{
		int count = (int)( NextRandom() % 9 );
		for ( int j = 0; j < count; ++j )
		{
			b3Vec3 n = RandomVec( -1.0f, 1.0f );
			// Mostly upward facing, like ground and walls around a character
			n.y += j == 0 ? 1.5f : 0.25f;
			planes[j].plane.normal = b3Normalize( n );
			planes[j].plane.offset = RandomFloat( -0.5f, 0.5f );
			uint32_t r = NextRandom();
			planes[j].pushLimit = ( r & 3 ) == 0 ? RandomFloat( 0.0f, 0.2f ) : FLT_MAX;
			planes[j].push = 123.0f;
			planes[j].clipVelocity = ( r & 4 ) != 0;
		}

		b3Vec3 target = RandomVec( -2.0f, 2.0f );
		if ( i % 17 == 0 )
		{
			target = b3MulSV( 40.0f, target );
		}

		b3PlaneSolverResult result = b3SolvePlanes( target, planes, count );

		printf( "%d |", i );
		pv( result.delta );
		printf( " %d %d", result.iterationCount, count );
		for ( int j = 0; j < count; ++j )
		{
			pf( planes[j].push );
		}

		b3Vec3 velocity = RandomVec( -5.0f, 5.0f );
		pv( b3ClipVector( velocity, planes, count ) );
		printf( "\n" );
	}

	return 0;
}
