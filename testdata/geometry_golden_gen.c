// Generates testdata/geometry_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/geometry_golden_gen.c src/mesh.c src/height_field.c src/compound.c
//      src/dynamic_tree.c src/aabb.c src/hull.c src/distance.c src/sphere.c src/capsule.c src/manifold.c
//      src/math_functions.c src/core.c src/timer.c src/simd.c"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/geometry_golden $SRC
// $SCRATCH/geometry_golden > ../../libs/box3d-go/testdata/geometry_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD) and, on a Mac, the SSE2
// build (-arch x86_64, runs under Rosetta). Both must be identical.
//
// Each line: a label, "|", outputs as hex words. Blobs (meshes, height fields, compounds) are
// reported by byte count and content hash (the hash covers every byte of the image). Inputs come
// from an xorshift generator with no libm calls so Go reproduces them exactly.
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "compound.h"
#include "core.h"
#include "math_internal.h"
#include "shape.h"
#include "simd.h"

#include <stdio.h>
#include <string.h>

// b3MakeLocalProxy and b3ComputeProxyAABB live in shape.c, which drags in the whole world.
// Verbatim copies from shape.c at the pinned commit.
b3ShapeProxy b3MakeLocalProxy( const b3ShapeProxy* proxy, b3Transform transform, b3Vec3* buffer )
{
	b3Transform invTransform = b3InvertTransform( transform );
	b3Matrix3 R = b3MakeMatrixFromQuat( invTransform.q );

	int count = b3MinInt( proxy->count, B3_MAX_SHAPE_CAST_POINTS );
	for ( int i = 0; i < count; ++i )
	{
		buffer[i] = b3Add( b3MulMV( R, proxy->points[i] ), invTransform.p );
	}

	return (b3ShapeProxy){
		.points = buffer,
		.count = count,
		.radius = proxy->radius,
	};
}

b3AABB b3ComputeProxyAABB( const b3ShapeProxy* proxy )
{
	const b3Vec3* points = proxy->points;
	b3AABB aabb = {
		.lowerBound = points[0],
		.upperBound = points[0],
	};

	for ( int i = 1; i < proxy->count; ++i )
	{
		aabb.lowerBound = b3Min( aabb.lowerBound, points[i] );
		aabb.upperBound = b3Max( aabb.upperBound, points[i] );
	}

	b3Vec3 r = { proxy->radius, proxy->radius, proxy->radius };
	aabb.lowerBound = b3Sub( aabb.lowerBound, r );
	aabb.upperBound = b3Add( aabb.upperBound, r );
	return aabb;
}

static uint32_t seed = 31337;
static float rnd( void )
{
	seed ^= seed << 13;
	seed ^= seed >> 17;
	seed ^= seed << 5;
	return ( (float)( seed % 20001 ) / 10000.0f - 1.0f );
}
static uint32_t rndInt( uint32_t n )
{
	seed ^= seed << 13;
	seed ^= seed >> 17;
	seed ^= seed << 5;
	return seed % n;
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
static void pbox( b3AABB a )
{
	pv( a.lowerBound );
	pv( a.upperBound );
}
static void pco( b3CastOutput o )
{
	printf( " %d", o.hit );
	pv( o.normal );
	pv( o.point );
	pf( o.fraction );
	printf( " %d %d %d %d", o.iterations, o.triangleIndex, o.childIndex, o.materialIndex );
}
static void pplanes( const b3PlaneResult* planes, int count )
{
	printf( " %d", count );
	for ( int i = 0; i < count; ++i )
	{
		pv( planes[i].plane.normal );
		pf( planes[i].plane.offset );
		pv( planes[i].point );
		printf( " %d %d %d", planes[i].triangleIndex, planes[i].childIndex, planes[i].materialIndex );
	}
}
static void ptri( b3Triangle t )
{
	pv( t.vertices[0] );
	pv( t.vertices[1] );
	pv( t.vertices[2] );
	printf( " %d %d %d %d", t.i1, t.i2, t.i3, t.flags );
}

// FNV-1a over 32 bit words: hashes query callback sequences.
static uint64_t s_queryHash;
static int s_queryCount;
static int s_queryStop;
static void hashWord( uint32_t w )
{
	for ( int i = 0; i < 4; ++i )
	{
		s_queryHash ^= ( w >> ( 8 * i ) ) & 0xFF;
		s_queryHash *= 0x100000001b3ull;
	}
}
static bool MeshQueryFcn( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	(void)context;
	hashWord( (uint32_t)triangleIndex );
	hashWord( B( a.x ) );
	hashWord( B( a.y ) );
	hashWord( B( a.z ) );
	hashWord( B( b.x ) );
	hashWord( B( b.y ) );
	hashWord( B( b.z ) );
	hashWord( B( c.x ) );
	hashWord( B( c.y ) );
	hashWord( B( c.z ) );
	s_queryCount += 1;
	return s_queryCount != s_queryStop;
}
static void resetQuery( int stop )
{
	s_queryHash = 0xcbf29ce484222325ull;
	s_queryCount = 0;
	s_queryStop = stop;
}
static void pquery( void )
{
	printf( " %d %016llx", s_queryCount, (unsigned long long)s_queryHash );
}

static void pmesh( const b3MeshData* m )
{
	if ( m == NULL )
	{
		printf( " null" );
		return;
	}
	printf( " %d %016llx %d %d %d %d %d %d %d", m->byteCount, (unsigned long long)m->hash, m->treeHeight, m->nodeCount,
			m->vertexCount, m->triangleCount, m->degenerateCount, m->materialCount, b3GetHeight( m ) );
	pf( m->surfaceArea );
	pbox( m->bounds );
}

// A random proxy: 1 to 4 points around center with a radius.
static b3Vec3 s_proxyPoints[4];
static b3ShapeProxy rproxy( b3Vec3 center, float spread )
{
	int count = 1 + (int)rndInt( 4 );
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 offset = rv3( spread, spread, spread, 0.0f, 0.0f, 0.0f );
		s_proxyPoints[i] = b3Add( center, offset );
	}
	float radius = 0.0f;
	if ( rndInt( 3 ) != 0 )
	{
		radius = 0.3f + 0.25f * rnd();
	}
	return (b3ShapeProxy){ s_proxyPoints, count, radius };
}

// Random terrain soup: an n x n grid of quads, each emitting two triangles with their own vertex
// copies (so welding has work to do), random heights and materials, plus a few degenerates.
#define SOUP_N 6
static b3Vec3 s_soupVertices[SOUP_N * SOUP_N * 6 + 9];
static int s_soupIndices[SOUP_N * SOUP_N * 6 + 9];
static uint8_t s_soupMaterials[SOUP_N * SOUP_N * 2 + 3];
static float s_heights[( SOUP_N + 1 ) * ( SOUP_N + 1 )];

static b3MeshDef soupDef( void )
{
	for ( int i = 0; i < ( SOUP_N + 1 ) * ( SOUP_N + 1 ); ++i )
	{
		s_heights[i] = 0.5f * rnd();
	}

	int v = 0;
	int t = 0;
	for ( int ix = 0; ix < SOUP_N; ++ix )
	{
		for ( int iz = 0; iz < SOUP_N; ++iz )
		{
			b3Vec3 p11 = { (float)ix, s_heights[ix * ( SOUP_N + 1 ) + iz], (float)iz };
			b3Vec3 p12 = { (float)ix, s_heights[ix * ( SOUP_N + 1 ) + iz + 1], (float)( iz + 1 ) };
			b3Vec3 p21 = { (float)( ix + 1 ), s_heights[( ix + 1 ) * ( SOUP_N + 1 ) + iz], (float)iz };
			b3Vec3 p22 = { (float)( ix + 1 ), s_heights[( ix + 1 ) * ( SOUP_N + 1 ) + iz + 1], (float)( iz + 1 ) };

			s_soupVertices[v + 0] = p11;
			s_soupVertices[v + 1] = p12;
			s_soupVertices[v + 2] = p22;
			s_soupVertices[v + 3] = p22;
			s_soupVertices[v + 4] = p21;
			s_soupVertices[v + 5] = p11;
			for ( int k = 0; k < 6; ++k )
			{
				s_soupIndices[v + k] = v + k;
			}
			s_soupMaterials[t + 0] = (uint8_t)rndInt( 5 );
			s_soupMaterials[t + 1] = (uint8_t)rndInt( 5 );
			v += 6;
			t += 2;
		}
	}

	// Degenerates: a sliver (collinear), a repeated index and a tiny triangle.
	s_soupVertices[v + 0] = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	s_soupVertices[v + 1] = (b3Vec3){ 1.0f, 1.0f, 0.0f };
	s_soupVertices[v + 2] = (b3Vec3){ 2.0f, 1.0f, 0.0f };
	s_soupIndices[v + 0] = v + 0;
	s_soupIndices[v + 1] = v + 1;
	s_soupIndices[v + 2] = v + 2;
	s_soupVertices[v + 3] = (b3Vec3){ 0.0f, 2.0f, 0.0f };
	s_soupVertices[v + 4] = (b3Vec3){ 1.0f, 2.0f, 0.0f };
	s_soupVertices[v + 5] = (b3Vec3){ 0.0f, 2.0f, 1.0f };
	s_soupIndices[v + 3] = v + 3;
	s_soupIndices[v + 4] = v + 3;
	s_soupIndices[v + 5] = v + 5;
	s_soupVertices[v + 6] = (b3Vec3){ 3.0f, 2.0f, 0.0f };
	s_soupVertices[v + 7] = (b3Vec3){ 3.0001f, 2.0f, 0.0f };
	s_soupVertices[v + 8] = (b3Vec3){ 3.0f, 2.0f, 0.0001f };
	s_soupIndices[v + 6] = v + 6;
	s_soupIndices[v + 7] = v + 7;
	s_soupIndices[v + 8] = v + 8;
	s_soupMaterials[t + 0] = 1;
	s_soupMaterials[t + 1] = 2;
	s_soupMaterials[t + 2] = 3;
	v += 9;
	t += 3;

	b3MeshDef def = { 0 };
	def.vertices = s_soupVertices;
	def.indices = s_soupIndices;
	def.materialIndices = s_soupMaterials;
	def.vertexCount = v;
	def.triangleCount = t;
	return def;
}

static void meshQueries( const char* name, const b3MeshData* data, b3Vec3 scale )
{
	b3Mesh mesh = { data, scale };
	b3AABB bounds = b3ComputeMeshAABB( data, b3Transform_identity, scale );
	b3Vec3 c = b3AABB_Center( bounds );
	b3Vec3 e = b3AABB_Extents( bounds );

	printf( "%s aabb |", name );
	{
		b3Transform xf;
		xf.p = rv3( 2.0f, 2.0f, 2.0f, 0.0f, 0.0f, 0.0f );
		xf.q = rq();
		pbox( b3ComputeMeshAABB( data, xf, scale ) );
	}
	printf( "\n" );

	printf( "%s triangles |", name );
	for ( int i = 0; i < data->triangleCount; i += 1 + data->triangleCount / 7 )
	{
		ptri( b3GetMeshTriangle( &mesh, i ) );
	}
	printf( "\n" );

	for ( int k = 0; k < 12; ++k )
	{
		printf( "%s ray %d |", name, k );
		b3RayCastInput ray;
		ray.origin = rv3( 1.5f * e.x, 1.5f * e.y + 1.0f, 1.5f * e.z, c.x, c.y, c.z );
		b3Vec3 target = rv3( e.x, e.y, e.z, c.x, c.y, c.z );
		ray.translation = b3MulSV( 2.0f, b3Sub( target, ray.origin ) );
		ray.maxFraction = k % 4 == 3 ? 0.4f : 1.0f;
		pco( b3RayCastMesh( &mesh, &ray ) );
		printf( "\n" );
	}

	for ( int k = 0; k < 10; ++k )
	{
		printf( "%s cast %d |", name, k );
		b3Vec3 start = rv3( 1.5f * e.x, 1.5f * e.y + 1.0f, 1.5f * e.z, c.x, c.y, c.z );
		b3ShapeCastInput input;
		input.proxy = rproxy( start, 0.3f );
		b3Vec3 target = rv3( e.x, e.y, e.z, c.x, c.y, c.z );
		input.translation = b3MulSV( 2.0f, b3Sub( target, start ) );
		input.maxFraction = 1.0f;
		input.canEncroach = k % 3 == 1;
		pco( b3ShapeCastMesh( &mesh, &input ) );
		printf( "\n" );
	}

	printf( "%s overlap |", name );
	for ( int k = 0; k < 16; ++k )
	{
		b3Transform xf;
		xf.p = rv3( 0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f );
		xf.q = rq();
		b3Vec3 local = rv3( e.x, e.y + 0.5f, e.z, c.x, c.y, c.z );
		b3Vec3 world = b3TransformPoint( xf, local );
		b3ShapeProxy proxy = rproxy( world, 0.4f );
		printf( " %d", b3OverlapMesh( &mesh, xf, &proxy ) );
	}
	printf( "\n" );

	for ( int k = 0; k < 8; ++k )
	{
		printf( "%s mover %d |", name, k );
		b3Capsule mover;
		mover.center1 = rv3( e.x, e.y + 0.3f, e.z, c.x, c.y, c.z );
		mover.center2 = b3Add( mover.center1, rv3( 0.3f, 0.3f, 0.3f, 0.0f, 0.4f, 0.0f ) );
		mover.radius = 0.4f + 0.3f * rnd();
		b3PlaneResult planes[16];
		int capacity = k == 5 ? 2 : 16;
		int count = b3CollideMoverAndMesh( planes, capacity, &mesh, &mover );
		pplanes( planes, count );
		printf( "\n" );
	}

	printf( "%s query |", name );
	for ( int k = 0; k < 6; ++k )
	{
		b3AABB box;
		b3Vec3 p = rv3( e.x, e.y, e.z, c.x, c.y, c.z );
		b3Vec3 h = rv3( 0.5f, 0.5f, 0.5f, 0.8f, 0.8f, 0.8f );
		box.lowerBound = b3Sub( p, h );
		box.upperBound = b3Add( p, h );
		resetQuery( k == 4 ? 3 : -1 );
		b3QueryMesh( &mesh, box, MeshQueryFcn, NULL );
		pquery();
	}
	printf( "\n" );
}

static void meshSection( void )
{
	b3MeshData* meshes[12];
	int meshCount = 0;

	printf( "mesh creators |" );
	meshes[meshCount++] = b3CreateGridMesh( 5, 4, 0.75f, 3, true );
	meshes[meshCount++] = b3CreateGridMesh( 3, 3, 1.0f, 0, false );
	meshes[meshCount++] = b3CreateWaveMesh( 7, 6, 0.6f, 0.4f, 0.13f, 0.21f );
	meshes[meshCount++] = b3CreateTorusMesh( 10, 7, 2.0f, 0.5f );
	meshes[meshCount++] = b3CreateBoxMesh( (b3Vec3){ 0.1f, 0.2f, -0.3f }, (b3Vec3){ 0.5f, 1.0f, 1.5f }, true );
	meshes[meshCount++] = b3CreateHollowBoxMesh( (b3Vec3){ 1.0f, 0.0f, 0.0f }, (b3Vec3){ 2.0f, 1.0f, 1.5f } );
	meshes[meshCount++] = b3CreatePlatformMesh( (b3Vec3){ 0.0f, 1.0f, 0.0f }, 2.0f, 1.0f, 3.0f );
	for ( int i = 0; i < meshCount; ++i )
	{
		pmesh( meshes[i] );
	}
	printf( "\n" );

	printf( "mesh soup |" );
	{
		uint32_t saved = seed;
		b3MeshDef def = soupDef();
		int degenerates[4] = { -1, -1, -1, -1 };
		meshes[meshCount++] = b3CreateMesh( &def, degenerates, 2 );
		pmesh( meshes[meshCount - 1] );
		printf( " %d %d %d %d", degenerates[0], degenerates[1], degenerates[2], degenerates[3] );

		// Same soup, welded, SAH split
		seed = saved;
		def = soupDef();
		def.weldVertices = true;
		def.weldTolerance = 0.01f;
		def.identifyEdges = true;
		meshes[meshCount++] = b3CreateMesh( &def, NULL, 0 );
		pmesh( meshes[meshCount - 1] );

		// Welded, median split, clockwise
		seed = saved;
		def = soupDef();
		def.weldVertices = true;
		def.weldTolerance = 0.01f;
		def.identifyEdges = true;
		def.useMedianSplit = true;
		def.clockWiseWinding = true;
		meshes[meshCount++] = b3CreateMesh( &def, NULL, 0 );
		pmesh( meshes[meshCount - 1] );

		// Rejected defs
		def.stride = 6;
		pmesh( b3CreateMesh( &def, NULL, 0 ) );
		def.stride = 0;
		def.vertexCount = 2;
		pmesh( b3CreateMesh( &def, NULL, 0 ) );
	}
	printf( "\n" );

	printf( "mesh flags |" );
	for ( int i = 0; i < meshCount; ++i )
	{
		const uint8_t* flags = b3GetMeshFlags( meshes[i] );
		const uint8_t* materials = b3GetMeshMaterialIndices( meshes[i] );
		uint32_t sum = 0;
		for ( int j = 0; j < meshes[i]->triangleCount; ++j )
		{
			sum = 31 * sum + flags[j] + 7 * materials[j];
		}
		printf( " %08x", sum );
	}
	printf( "\n" );

	const b3Vec3 scales[4] = {
		{ 1.0f, 1.0f, 1.0f },
		{ 1.5f, 0.5f, 2.0f },
		{ -1.0f, 1.0f, 1.0f },
		{ 0.75f, -1.25f, -0.5f },
	};

	const int queried[4] = { 2, 3, 8, 9 };
	for ( int i = 0; i < 4; ++i )
	{
		for ( int s = 0; s < 4; ++s )
		{
			char name[32];
			snprintf( name, sizeof( name ), "mesh %d scale %d", queried[i], s );
			meshQueries( name, meshes[queried[i]], scales[s] );
		}
	}

	for ( int i = 0; i < meshCount; ++i )
	{
		b3DestroyMesh( meshes[i] );
	}
}

static void quietLog( const char* message )
{
	(void)message;
}

int main( void )
{
	b3SetLogFcn( quietLog );

	meshSection();

	return 0;
}
