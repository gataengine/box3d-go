// Generates testdata/hull_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository:
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/hull_golden \
//    ../../libs/box3d-go/testdata/hull_golden_gen.c src/hull.c src/distance.c src/sphere.c src/capsule.c src/manifold.c src/math_functions.c src/core.c
// $SCRATCH/hull_golden > ../../libs/box3d-go/testdata/hull_golden.txt
//
// Each line: name, inputs as hex floats, "|", outputs as hex words. Hulls are reported by byte
// count and hash (the hash covers every byte of the image). Point clouds come from an xorshift
// generator with no libm calls so Go reproduces the inputs exactly.
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "math_internal.h"
#include "manifold.h"
#include "shape.h"
#include <stdio.h>
#include <string.h>

static uint32_t seed = 777;
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
static void ph( const b3HullData* h )
{
	if ( h == NULL )
	{
		printf( " null" );
		return;
	}
	printf( " %d %016llx", h->byteCount, (unsigned long long)h->hash );
	pf( h->volume );
	pf( h->surfaceArea );
	pf( h->innerRadius );
	pv( h->center );
}

static void quietLog( const char* message )
{
	(void)message;
}

int main( void )
{
	// Hull limit messages would interleave with the output.
	b3SetLogFcn( quietLog );

	printf( "box |" );
	{
		b3BoxHull b = b3MakeBoxHull( 0.5f, 1.0f, 2.0f );
		ph( &b.base );
		b3Transform xf = { { 1.0f, -2.0f, 3.0f }, b3NormalizeQuat( (b3Quat){ { 0.1f, 0.2f, 0.3f }, 0.9f } ) };
		b = b3MakeTransformedBoxHull( 0.25f, 0.5f, 0.75f, xf );
		ph( &b.base );
		b = b3MakeScaledBoxHull( (b3Vec3){ 0.5f, 0.25f, 1.0f }, xf, (b3Vec3){ -1.5f, 2.0f, 0.5f } );
		ph( &b.base );
	}
	printf( "\n" );

	printf( "shapes |" );
	ph( b3CreateCylinder( 2.0f, 1.0f, 0.5f, 12 ) );
	ph( b3CreateCone( 1.5f, 1.0f, 0.25f, 10 ) );
	ph( b3CreateRock( 0.7f ) );
	ph( b3CreateComplexHull( 1.3f ) );
	printf( "\n" );

	for ( int k = 0; k < 40; ++k )
	{
		b3Sphere sphere;
		sphere.center = rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f );
		sphere.radius = 0.5f + 0.4f * rnd();
		b3Capsule capsule;
		capsule.center1 = rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f );
		capsule.center2 = ( k % 5 == 0 ) ? capsule.center1 : rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f );
		capsule.radius = 0.3f + 0.2f * rnd();
		b3Capsule mover;
		mover.center1 = rv3( 2.0f, 2.0f, 2.0f, 0.0f, 0.0f, 0.0f );
		mover.center2 = rv3( 2.0f, 2.0f, 2.0f, 0.0f, 0.0f, 0.0f );
		mover.radius = 0.5f + 0.4f * rnd();
		b3RayCastInput ray;
		ray.origin = rv3( 3.0f, 3.0f, 3.0f, 0.0f, 0.0f, 0.0f );
		if ( k % 7 == 3 )
		{
			// Parallel to the capsule axis.
			ray.translation = b3MulSV( 4.0f, b3Sub( capsule.center2, capsule.center1 ) );
		}
		else
		{
			ray.translation = b3Sub( rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f ), ray.origin );
		}
		ray.maxFraction = 1.0f;
		b3Transform xf;
		xf.p = rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f );
		xf.q = rq();

		printf( "prim %d |", k );
		b3MassData md = b3ComputeSphereMass( &sphere, 1.5f );
		pf( md.mass );
		pv( md.inertia.cx );
		md = b3ComputeCapsuleMass( &capsule, 1.5f );
		pf( md.mass );
		pv( md.center );
		pv( md.inertia.cx );
		pv( md.inertia.cy );
		pv( md.inertia.cz );
		b3AABB box = b3ComputeCapsuleAABB( &capsule, xf );
		pv( box.lowerBound );
		pv( box.upperBound );
		box = b3ComputeSphereAABB( &sphere, xf );
		pv( box.lowerBound );
		pv( box.upperBound );

		b3CastOutput o = b3RayCastSphere( &sphere, &ray );
		pv( o.point );
		pv( o.normal );
		pf( o.fraction );
		printf( " %d", o.hit );
		o = b3RayCastHollowSphere( &sphere, &ray );
		pv( o.point );
		pv( o.normal );
		pf( o.fraction );
		printf( " %d", o.hit );
		o = b3RayCastCapsule( &capsule, &ray );
		pv( o.point );
		pv( o.normal );
		pf( o.fraction );
		printf( " %d", o.hit );

		b3PlaneResult pr = { 0 };
		int n = b3CollideMoverAndSphere( &pr, &sphere, &mover );
		printf( " %d", n );
		pv( pr.plane.normal );
		pf( pr.plane.offset );
		pv( pr.point );
		n = b3CollideMoverAndCapsule( &pr, &capsule, &mover );
		printf( " %d", n );
		pv( pr.plane.normal );
		pf( pr.plane.offset );
		pv( pr.point );

		b3ShapeProxy proxy = { &mover.center1, 2, mover.radius };
		printf( " %d %d", b3OverlapSphere( &sphere, xf, &proxy ), b3OverlapCapsule( &capsule, xf, &proxy ) );
		printf( "\n" );
	}

	for ( int k = 0; k < 20; ++k )
	{
		// Clip a pentagon against a random plane through its neighborhood.
		b3ClipVertex polygon[5];
		for ( int i = 0; i < 5; ++i )
		{
			float angle = 1.25f * (float)i;
			b3CosSin cs = b3ComputeCosSin( angle );
			polygon[i].position = (b3Vec3){ cs.cosine, 0.1f * (float)i, cs.sine };
			polygon[i].separation = 0.0f;
			polygon[i].pair = b3MakeFeaturePair( b3_featureShapeB, i, b3_featureShapeB, ( i + 1 ) % 5 );
		}
		b3Plane clipPlane;
		clipPlane.normal = b3Normalize( rv3( 1.0f, 0.2f, 1.0f, 0.0f, 0.0f, 0.0f ) );
		clipPlane.offset = 0.5f * rnd();
		b3Plane refPlane = { b3Vec3_axisY, 0.05f };
		b3ClipVertex out[10];
		int outCount = b3ClipPolygon( out, polygon, 5, clipPlane, k, refPlane );
		printf( "clip %d |", k );
		pv( clipPlane.normal );
		pf( clipPlane.offset );
		printf( " %d", outCount );
		for ( int i = 0; i < outCount; ++i )
		{
			b3FeaturePair pair = b3FlipPair( out[i].pair );
			pv( out[i].position );
			pf( out[i].separation );
			printf( " %08x %08x", b3MakeFeatureId( out[i].pair ), b3MakeFeatureId( pair ) );
		}
		printf( "\n" );
	}

	int counts[] = { 8, 20, 64, 300 };
	int maxCounts[] = { 8, 16, 32, 64 };
	for ( int c = 0; c < 4; ++c )
	{
		for ( int k = 0; k < 6; ++k )
		{
			b3Vec3 points[300];
			int n = counts[c];
			printf( "hull %d %d", n, maxCounts[k % 4] );
			for ( int i = 0; i < n; ++i )
			{
				points[i] = rv3( 3.0f, 2.0f, 1.0f, 0.0f, 0.0f, 0.0f );
				if ( k & 1 )
				{
					points[i] = b3Normalize( points[i] );
				}
				pv( points[i] );
			}
			printf( " |" );
			b3HullData* h = b3CreateHull( points, n, maxCounts[k % 4] );
			ph( h );
			if ( h != NULL )
			{
				b3Transform xf;
				xf.p = rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f );
				xf.q = rq();
				b3Vec3 scale = rv3( 1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 0.5f );
				pv( xf.p );
				pf( xf.q.v.x );
				pf( xf.q.v.y );
				pf( xf.q.v.z );
				pf( xf.q.s );
				pv( scale );
				b3HullData* t = b3CloneAndTransformHull( h, xf, scale );
				ph( t );

				b3Vec3 dir = b3Normalize( rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f ) );
				pv( dir );
				int supportVertex = b3FindHullSupportVertex( h, dir );
				printf( " %d %d %d", supportVertex, b3FindHullSupportFace( h, dir ), b3FindIncidentFace( h, dir, supportVertex ) );
				pf( b3ComputeHullProjectedArea( h, dir ) );

				b3RayCastInput ray;
				ray.origin = rv3( 5.0f, 5.0f, 5.0f, 0.0f, 0.0f, 0.0f );
				ray.translation = b3Sub( rv3( 0.1f, 0.1f, 0.1f, 0.0f, 0.0f, 0.0f ), ray.origin );
				ray.maxFraction = 1.0f;
				pv( ray.origin );
				pv( ray.translation );
				b3CastOutput ro = b3RayCastHull( h, &ray );
				pv( ro.point );
				pv( ro.normal );
				pf( ro.fraction );
				printf( " %d", ro.hit );

				// Distance and cast against a capsule-like segment.
				b3Vec3 seg[2];
				seg[0] = rv3( 1.0f, 1.0f, 1.0f, 4.0f, 0.0f, 0.0f );
				seg[1] = rv3( 1.0f, 1.0f, 1.0f, 4.0f, 1.0f, 0.0f );
				pv( seg[0] );
				pv( seg[1] );
				b3DistanceInput di;
				di.proxyA = (b3ShapeProxy){ b3GetHullPoints( h ), h->vertexCount, 0.0f };
				di.proxyB = (b3ShapeProxy){ seg, 2, 0.1f };
				di.transform = xf;
				di.useRadii = true;
				b3SimplexCache cache = { 0 };
				b3DistanceOutput dout = b3ShapeDistance( &di, &cache, NULL, 0 );
				pv( dout.pointA );
				pv( dout.pointB );
				pv( dout.normal );
				pf( dout.distance );
				printf( " %d", dout.iterations );

				b3ShapeCastPairInput ci;
				ci.proxyA = di.proxyA;
				ci.proxyB = di.proxyB;
				ci.transform = xf;
				ci.translationB = b3MulSV( -8.0f, xf.p );
				ci.maxFraction = 1.0f;
				ci.canEncroach = false;
				b3CastOutput co = b3ShapeCast( &ci );
				pv( co.point );
				pv( co.normal );
				pf( co.fraction );
				printf( " %d %d", co.hit, co.iterations );

				b3TOIInput ti;
				ti.proxyA = di.proxyA;
				ti.proxyB = di.proxyB;
				ti.sweepA = (b3Sweep){ b3Vec3_zero, b3Vec3_zero, b3Vec3_zero, b3Quat_identity, xf.q };
				ti.sweepB = (b3Sweep){ b3Vec3_zero, (b3Vec3){ 6.0f, 0.0f, 0.0f }, (b3Vec3){ -2.0f, 0.5f, 0.0f }, b3Quat_identity,
									   b3Quat_identity };
				ti.maxFraction = 1.0f;
				b3TOIOutput to = b3TimeOfImpact( &ti );
				printf( " %d", to.state );
				pv( to.point );
				pv( to.normal );
				pf( to.fraction );
				pf( to.distance );

				b3Capsule mover = { seg[0], seg[1], 2.0f };
				b3PlaneResult pr = { 0 };
				int planeCount = b3CollideMoverAndHull( &pr, h, &mover );
				printf( " %d", planeCount );
				pv( pr.plane.normal );
				pf( pr.plane.offset );
				pv( pr.point );
				b3ShapeProxy proxy = { seg, 2, 0.5f };
				printf( " %d", b3OverlapHull( h, xf, &proxy ) );

				b3MassData md = b3ComputeHullMass( h, 2.5f );
				pf( md.mass );
				pv( md.inertia.cx );
				pv( md.inertia.cy );
				pv( md.inertia.cz );
			}
			printf( "\n" );
		}
	}
	return 0;
}
