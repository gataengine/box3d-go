// Generates testdata/manifold_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binaries outside the repository ($SCRATCH):
//
// printf '#define B3_SIMD_WIDTH 4\n#include "convex_manifold_wide.inl"\n' > $SCRATCH/wide_cm.c
// SRC="../../libs/box3d-go/testdata/manifold_golden_gen.c src/convex_manifold.c $SCRATCH/wide_cm.c
//      src/triangle_manifold.c src/manifold.c src/hull.c src/distance.c src/math_functions.c
//      src/core.c src/simd.c src/sphere.c src/capsule.c"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/manifold_golden $SRC
// $SCRATCH/manifold_golden > ../../libs/box3d-go/testdata/manifold_golden.txt
//
// The output must not depend on the SIMD backend. Check it against the no-SIMD build
// (add -DBOX3D_DISABLE_SIMD) and, on a Mac, the SSE2 build (-arch x86_64, runs under Rosetta).
//
// Each line: name, then the outputs as hex words. Local manifolds print the point count, the
// normal, every point (position, separation, feature id), the triangle feature and the squared
// distance; caches print every field. Inputs come from an xorshift generator with no libm calls
// so Go reproduces them exactly.
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "manifold.h"
#include "math_internal.h"
#include <stdio.h>
#include <string.h>

static uint32_t seed = 4242;
static float rnd( void )
{
	seed ^= seed << 13;
	seed ^= seed >> 17;
	seed ^= seed << 5;
	return ( (float)( seed % 20001 ) / 10000.0f - 1.0f );
}
// Sequenced helpers: C leaves the evaluation order of initializer lists unspecified.
static b3Vec3 rv3( float sx, float sy, float sz, float ox, float oy, float oz )
{
	b3Vec3 v;
	v.x = ox + sx * rnd();
	v.y = oy + sy * rnd();
	v.z = oz + sz * rnd();
	return v;
}
static b3Quat rq( void )
{
	b3Quat q;
	q.v = rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f );
	q.s = rnd();
	return b3NormalizeQuat( q );
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

static b3LocalManifoldPoint s_points[8];

static b3LocalManifold fresh( void )
{
	memset( s_points, 0, sizeof( s_points ) );
	b3LocalManifold m;
	memset( &m, 0, sizeof( m ) );
	m.points = s_points;
	return m;
}

static void pm( const b3LocalManifold* m )
{
	printf( " %d", m->pointCount );
	pv( m->normal );
	for ( int i = 0; i < m->pointCount; ++i )
	{
		pv( m->points[i].point );
		pf( m->points[i].separation );
		printf( " %08x", b3MakeFeatureId( m->points[i].pair ) );
	}
	printf( " %d", (int)m->feature );
	pf( m->squaredDistance );
}

static void psat( const b3SATCache* c )
{
	pf( c->separation );
	printf( " %d %d %d %d", c->type, c->indexA, c->indexB, c->hit );
}

static void psc( const b3SimplexCache* c )
{
	printf( " %d", c->count );
	for ( int i = 0; i < c->count; ++i )
	{
		printf( " %d %d", c->indexA[i], c->indexB[i] );
	}
}

static void pa( b3SeparatingAxis a )
{
	pv( a.normal );
	pf( a.separation );
	printf( " %d %d %d", a.indexA, a.indexB, (int)a.type );
}

static void paq( b3AxisQuery q )
{
	pa( q.faceA );
	pa( q.faceB );
	pa( q.edge );
	printf( " %d", (int)q.separatedFeature );
}

static b3Transform nudge( b3Transform xf )
{
	b3Transform r = xf;
	r.p = b3Add( xf.p, rv3( 0.02f, 0.02f, 0.02f, 0.0f, 0.0f, 0.0f ) );
	return r;
}

static void quietLog( const char* message )
{
	(void)message;
}

#define POOL_COUNT 8

int main( void )
{
	// Hull limit messages would interleave with the output.
	b3SetLogFcn( quietLog );

	static b3BoxHull boxes[3];
	const b3HullData* pool[POOL_COUNT];

	boxes[0] = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	boxes[1] = b3MakeBoxHull( 1.5f, 0.1f, 0.1f );
	{
		b3Transform place;
		place.p = rv3( 0.3f, 0.3f, 0.3f, 0.0f, 0.0f, 0.0f );
		place.q = rq();
		boxes[2] = b3MakeTransformedBoxHull( 0.4f, 0.6f, 0.5f, place );
	}
	pool[0] = &boxes[0].base;
	pool[1] = &boxes[1].base;
	pool[2] = &boxes[2].base;
	pool[3] = b3CreateCylinder( 1.0f, 0.5f, 0.25f, 12 );
	pool[4] = b3CreateCone( 1.0f, 0.6f, 0.2f, 9 );
	pool[5] = b3CreateRock( 0.7f );
	for ( int h = 6; h < POOL_COUNT; ++h )
	{
		b3Vec3 points[40];
		for ( int i = 0; i < 40; ++i )
		{
			points[i] = rv3( 0.8f, 0.6f, 0.5f, 0.0f, 0.0f, 0.0f );
		}
		pool[h] = b3CreateHull( points, 40, h == 6 ? 16 : 64 );
	}

	printf( "pool |" );
	for ( int h = 0; h < POOL_COUNT; ++h )
	{
		printf( " %d %d %d %016llx", pool[h]->vertexCount, pool[h]->edgeCount, pool[h]->faceCount,
				(unsigned long long)pool[h]->hash );
	}
	printf( "\n" );

	float scales[] = { 0.4f, 0.8f, 1.3f };

	for ( int k = 0; k < 120; ++k )
	{
		float s = scales[k % 3];

		b3Sphere sphereA;
		sphereA.center = rv3( 0.3f, 0.3f, 0.3f, 0.0f, 0.0f, 0.0f );
		sphereA.radius = 0.35f + 0.25f * rnd();
		b3Sphere sphereB;
		sphereB.center = rv3( 0.3f, 0.3f, 0.3f, 0.0f, 0.0f, 0.0f );
		sphereB.radius = 0.35f + 0.25f * rnd();

		b3Capsule capsuleA;
		capsuleA.center1 = rv3( 0.6f, 0.6f, 0.6f, 0.0f, 0.0f, 0.0f );
		capsuleA.center2 = rv3( 0.6f, 0.6f, 0.6f, 0.0f, 0.0f, 0.0f );
		capsuleA.radius = 0.2f + 0.15f * rnd();
		b3Capsule capsuleB;
		capsuleB.center1 = rv3( 0.6f, 0.6f, 0.6f, 0.0f, 0.0f, 0.0f );
		if ( k % 4 == 1 )
		{
			// Nearly parallel to capsule A.
			b3Vec3 jitter = rv3( 0.01f, 0.01f, 0.01f, 0.0f, 0.0f, 0.0f );
			capsuleB.center2 = b3Add( capsuleB.center1, b3Add( b3Sub( capsuleA.center2, capsuleA.center1 ), jitter ) );
		}
		else
		{
			capsuleB.center2 = rv3( 0.6f, 0.6f, 0.6f, 0.0f, 0.0f, 0.0f );
		}
		capsuleB.radius = 0.2f + 0.15f * rnd();

		const b3HullData* hullA = pool[k % POOL_COUNT];
		const b3HullData* hullB = pool[( 3 * k + 1 ) % POOL_COUNT];

		b3Transform xf;
		xf.p = rv3( s, s, s, 0.0f, 0.0f, 0.0f );
		xf.q = rq();
		if ( k % 5 == 2 )
		{
			// Aligned stacks drive face contacts with many clip points.
			xf.q = b3Quat_identity;
			xf.p.x = 0.1f * xf.p.x;
			xf.p.z = 0.1f * xf.p.z;
		}

		printf( "pair %d | ", k );

		b3LocalManifold m = fresh();
		b3CollideSpheres( &m, 8, &sphereA, &sphereB, xf );
		pm( &m );

		m = fresh();
		b3CollideCapsuleAndSphere( &m, 8, &capsuleA, &sphereB, xf );
		pm( &m );

		{
			b3SimplexCache cache = { 0 };
			m = fresh();
			b3CollideHullAndSphere( &m, 8, hullA, &sphereB, xf, &cache );
			pm( &m );
			psc( &cache );
			b3Transform xf2 = nudge( xf );
			m = fresh();
			b3CollideHullAndSphere( &m, 8, hullA, &sphereB, xf2, &cache );
			pm( &m );
			psc( &cache );
		}

		m = fresh();
		b3CollideCapsules( &m, 8, &capsuleA, &capsuleB, xf );
		pm( &m );

		{
			b3SimplexCache cache = { 0 };
			m = fresh();
			b3CollideHullAndCapsule( &m, 8, hullA, &capsuleB, xf, &cache );
			pm( &m );
			psc( &cache );
			b3Transform xf2 = nudge( xf );
			m = fresh();
			b3CollideHullAndCapsule( &m, 8, hullA, &capsuleB, xf2, &cache );
			pm( &m );
			psc( &cache );
		}
		printf( "\n" );

		printf( "hulls %d |", k );
		{
			b3SATCache cache = { 0 };
			m = fresh();
			b3CollideHulls( &m, 8, hullA, hullB, xf, &cache );
			pm( &m );
			psat( &cache );

			b3Transform xf2 = nudge( xf );
			m = fresh();
			b3CollideHulls( &m, 8, hullA, hullB, xf2, &cache );
			pm( &m );
			psat( &cache );

			for ( int manual = b3_manualFaceAxisA; manual <= b3_manualEdgePairAxis; ++manual )
			{
				b3SATCache manualCache = { 0 };
				manualCache.type = (uint8_t)manual;
				m = fresh();
				b3CollideHulls( &m, 8, hullA, hullB, xf, &manualCache );
				pm( &m );
				psat( &manualCache );
			}

			paq( b3ComputeSeparatingAxis( hullA, hullB, xf, true ) );
			paq( b3ComputeSeparatingAxis( hullA, hullB, xf, false ) );
		}
		printf( "\n" );

		printf( "triangle %d |", k );
		{
			b3Vec3 triangle[3];
			triangle[0] = rv3( 1.5f, 0.2f, 1.5f, 0.0f, -0.5f * s, 0.0f );
			triangle[1] = rv3( 1.5f, 0.2f, 1.5f, 0.0f, -0.5f * s, 0.0f );
			triangle[2] = rv3( 1.5f, 0.2f, 1.5f, 0.0f, -0.5f * s, 0.0f );
			if ( k % 4 != 3 )
			{
				// Face up (toward the shapes) except for every fourth triangle, which tests the back side cull.
				b3Vec3 n = b3Cross( b3Sub( triangle[1], triangle[0] ), b3Sub( triangle[2], triangle[0] ) );
				if ( n.y < 0.0f )
				{
					b3Vec3 t = triangle[1];
					triangle[1] = triangle[2];
					triangle[2] = t;
				}
			}
			if ( k % 3 == 0 )
			{
				b3Quat q = rq();
				triangle[0] = b3RotateVector( q, triangle[0] );
				triangle[1] = b3RotateVector( q, triangle[1] );
				triangle[2] = b3RotateVector( q, triangle[2] );
			}
			pv( triangle[0] );
			pv( triangle[1] );
			pv( triangle[2] );

			m = fresh();
			b3CollideTriangleAndSphere( &m, 8, triangle, &sphereB );
			pm( &m );

			b3SimplexCache simplexCache = { 0 };
			m = fresh();
			b3CollideTriangleAndCapsule( &m, 8, triangle, &capsuleB, &simplexCache );
			pm( &m );
			psc( &simplexCache );

			bool enableSpeculative = ( k & 1 ) == 0;
			int flags = k % 8;
			b3SATCache cache = { 0 };
			m = fresh();
			b3CollideTriangleAndHull( &m, 8, triangle[0], triangle[1], triangle[2], flags, hullB, &cache, enableSpeculative );
			pm( &m );
			psat( &cache );
			int touching = m.pointCount;

			b3Vec3 jitter = rv3( 0.01f, 0.01f, 0.01f, 0.0f, 0.0f, 0.0f );
			m = fresh();
			b3CollideTriangleAndHull( &m, 8, b3Add( triangle[0], jitter ), b3Add( triangle[1], jitter ),
									  b3Add( triangle[2], jitter ), flags, hullB, &cache, enableSpeculative );
			pm( &m );
			psat( &cache );

			// The manual modes skip the separation checks, and the edge builder validates that the pair
			// is within the speculative band, so only touching pairs take them.
			for ( int manual = b3_manualFaceAxisA; touching > 0 && manual <= b3_manualEdgePairAxis; ++manual )
			{
				b3SATCache manualCache = { 0 };
				manualCache.type = (uint8_t)manual;
				m = fresh();
				b3CollideTriangleAndHull( &m, 8, triangle[0], triangle[1], triangle[2], flags, hullB, &manualCache,
										  enableSpeculative );
				pm( &m );
				psat( &manualCache );
			}
		}
		printf( "\n" );
	}

	return 0;
}
