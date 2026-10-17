// Generates testdata/dynamic_tree_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository and running it from a
// scratch directory (it saves and removes a tree file in the working directory):
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/dynamic_tree_golden \
//    ../../libs/box3d-go/testdata/dynamic_tree_golden_gen.c src/dynamic_tree.c src/aabb.c \
//    src/math_functions.c src/core.c src/timer.c
// (cd $SCRATCH && ./dynamic_tree_golden) > ../../libs/box3d-go/testdata/dynamic_tree_golden.txt
//
// Each line: a label, "|", outputs as hex words. Tree state is reported as counters plus b3Hash
// of the live node range, the parents and the proxy pool, so every node index, flag, height and
// box bit is covered. Query lines hash the callback sequence (proxy id, user data and the
// fraction or distance handed to the callback), so traversal order is covered too. Inputs come
// from the LCG of test_dynamic_tree.c with no libm calls, so Go reproduces them exactly.
#include "aabb.h"
#include "dynamic_tree.h"

#include "box3d/base.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_seed = 2024;

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

static b3AABB RandomBox( float extent, float size )
{
	b3Vec3 center = RandomVec( -extent, extent );
	b3Vec3 half = RandomVec( 0.05f, size );
	return (b3AABB){ b3Sub( center, half ), b3Add( center, half ) };
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

static uint32_t HashBytes( uint32_t hash, const void* data, int count )
{
	return b3Hash( hash, (const uint8_t*)data, count );
}

static void PrintTree( const char* label, const b3DynamicTree* tree )
{
	printf( "%s |", label );
	printf( " %d %d %d %d %d %d %d", tree->nodeEnd, tree->nodeCapacity, tree->pairFreeList, tree->proxyCount,
			tree->proxyCapacity, tree->proxyFreeList, tree->dfsOrdered ? 1 : 0 );
	printf( " %d", b3DynamicTree_GetHeight( tree ) );
	pf( b3DynamicTree_GetAreaRatio( tree ) );
	b3AABB root = b3DynamicTree_GetRootBounds( tree );
	pv( root.lowerBound );
	pv( root.upperBound );
	printf( " %d", b3DynamicTree_GetByteCount( tree ) );
	printf( " %08x", HashBytes( B3_HASH_INIT, tree->nodes, tree->nodeEnd * (int)sizeof( b3TreeNode ) ) );
	printf( " %08x", HashBytes( B3_HASH_INIT, tree->parents, tree->nodeEnd * (int)sizeof( int32_t ) ) );
	printf( " %08x", HashBytes( B3_HASH_INIT, tree->proxies, tree->proxyCapacity * (int)sizeof( b3TreeProxy ) ) );
	printf( "\n" );
}

typedef struct Trace
{
	uint32_t hash;
	int count;
	const b3AABB* boxes;
	b3Vec3 point;
} Trace;

static void TraceInt( Trace* trace, int proxyId, uint64_t userData )
{
	trace->hash = HashBytes( trace->hash, &proxyId, 4 );
	trace->hash = HashBytes( trace->hash, &userData, 8 );
	trace->count += 1;
}

static bool QueryCallback( int proxyId, uint64_t userData, void* context )
{
	Trace* trace = context;
	TraceInt( trace, proxyId, userData );

	// Stop early on some proxy so the early out is covered
	return ( userData & 0xFFFFu ) != 77;
}

static bool QueryAllCallback( int proxyId, uint64_t userData, void* context )
{
	Trace* trace = context;
	TraceInt( trace, proxyId, userData );
	return true;
}

static float ClosestCallback( float distanceSqrMin, int proxyId, uint64_t userData, void* context )
{
	Trace* trace = context;
	TraceInt( trace, proxyId, userData );
	trace->hash = HashBytes( trace->hash, &distanceSqrMin, 4 );

	b3AABB box = trace->boxes[userData & 0xFFFFu];
	b3Vec3 r = b3Sub( trace->point, b3Clamp( trace->point, box.lowerBound, box.upperBound ) );
	float d = b3Dot( r, r );
	return d < distanceSqrMin ? d : distanceSqrMin;
}

static float RayCallback( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	Trace* trace = context;
	TraceInt( trace, proxyId, userData );
	trace->hash = HashBytes( trace->hash, &input->maxFraction, 4 );

	b3AABB box = trace->boxes[userData & 0xFFFFu];
	b3Vec3 p2 = b3MulAdd( input->origin, input->maxFraction, input->translation );
	float minFraction = 0.0f;
	float maxFraction = 0.0f;
	if ( b3RayCastAABB( box, input->origin, p2, &minFraction, &maxFraction ) == false )
	{
		return -1.0f;
	}

	// Clip to the box entry, scaled back to the full translation
	float fraction = minFraction * input->maxFraction;
	if ( fraction <= 0.0f )
	{
		return -1.0f;
	}

	return fraction;
}

static float BoxCallback( const b3BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	Trace* trace = context;
	TraceInt( trace, proxyId, userData );
	trace->hash = HashBytes( trace->hash, &input->maxFraction, 4 );

	switch ( userData % 4 )
	{
		case 0:
			return -1.0f;
		case 1:
			return 0.75f * input->maxFraction;
		case 2:
			return input->maxFraction;
		default:
			return ( userData & 0xFFFFu ) == 403 ? 0.0f : 0.5f * input->maxFraction;
	}
}

static void PrintTrace( const char* label, int round, Trace* trace, b3TreeStats stats )
{
	printf( "%s %d | %d %08x %d %d\n", label, round, trace->count, trace->hash, stats.nodeVisits, stats.leafVisits );
}

#define BOX_COUNT 600

static b3AABB boxes[BOX_COUNT];
static int proxies[BOX_COUNT];
static bool alive[BOX_COUNT];

static uint64_t UserData( int i )
{
	// High bits make the shapeIndex truncation visible
	return ( (uint64_t)( i * 3 + 1 ) << 32 ) | (uint64_t)i;
}

static uint64_t CategoryBits( int i )
{
	return 1ull << ( i % 5 );
}

static void RunQueries( const char* label, b3DynamicTree* tree )
{
	char name[64];

	for ( int round = 0; round < 12; ++round )
	{
		b3AABB box = RandomBox( 12.0f, 4.0f );
		uint64_t mask = round % 3 == 0 ? B3_DEFAULT_MASK_BITS : (uint64_t)( round % 7 + 1 );
		bool requireAll = round % 4 == 3;

		Trace trace = { B3_HASH_INIT, 0, boxes, b3Vec3_zero };
		b3TreeStats stats = b3DynamicTree_Query( tree, box, mask, requireAll, QueryCallback, &trace );
		snprintf( name, sizeof( name ), "%s query", label );
		PrintTrace( name, round, &trace, stats );
	}

	for ( int round = 0; round < 12; ++round )
	{
		b3Vec3 point = RandomVec( -14.0f, 14.0f );
		uint64_t mask = round % 2 == 0 ? B3_DEFAULT_MASK_BITS : (uint64_t)( round % 7 + 1 );

		Trace trace = { B3_HASH_INIT, 0, boxes, point };
		float minDistanceSqr = round == 5 ? 4.0f : FLT_MAX;
		b3TreeStats stats =
			b3DynamicTree_QueryClosest( tree, point, mask, round == 7, ClosestCallback, &trace, &minDistanceSqr );
		trace.hash = HashBytes( trace.hash, &minDistanceSqr, 4 );
		snprintf( name, sizeof( name ), "%s closest", label );
		PrintTrace( name, round, &trace, stats );
	}

	for ( int round = 0; round < 12; ++round )
	{
		b3RayCastInput input;
		input.origin = RandomVec( -14.0f, 14.0f );
		input.translation = RandomVec( -28.0f, 28.0f );
		input.maxFraction = round == 4 ? 0.5f : 1.0f;
		uint64_t mask = round % 3 == 1 ? (uint64_t)( round % 7 + 1 ) : B3_DEFAULT_MASK_BITS;

		Trace trace = { B3_HASH_INIT, 0, boxes, b3Vec3_zero };
		b3TreeStats stats = b3DynamicTree_RayCast( tree, &input, mask, false, RayCallback, &trace );
		snprintf( name, sizeof( name ), "%s ray", label );
		PrintTrace( name, round, &trace, stats );
	}

	for ( int round = 0; round < 12; ++round )
	{
		b3BoxCastInput input;
		input.box = RandomBox( 14.0f, 1.0f );
		input.translation = RandomVec( -28.0f, 28.0f );
		input.maxFraction = 1.0f;

		Trace trace = { B3_HASH_INIT, 0, boxes, b3Vec3_zero };
		b3TreeStats stats = b3DynamicTree_BoxCast( tree, &input, B3_DEFAULT_MASK_BITS, round == 2, BoxCallback, &trace );
		snprintf( name, sizeof( name ), "%s boxcast", label );
		PrintTrace( name, round, &trace, stats );
	}

	// A view without proxies reports the truncated shape index and skips the filter
	{
		b3TreeView view = b3MakeTreeView( tree );
		view.proxies = NULL;
		b3AABB box = RandomBox( 8.0f, 6.0f );
		Trace trace = { B3_HASH_INIT, 0, boxes, b3Vec3_zero };
		b3TreeStats stats = b3TreeView_Query( &view, box, 0, true, QueryAllCallback, &trace );
		snprintf( name, sizeof( name ), "%s view", label );
		PrintTrace( name, 0, &trace, stats );
	}
}

static void PrintMoved( const char* label, b3DynamicTree* tree )
{
	static int ids[BOX_COUNT];
	int count = b3DynamicTree_GatherMovedProxies( tree, ids );
	printf( "%s | %d %08x\n", label, count, HashBytes( B3_HASH_INIT, ids, count * (int)sizeof( int ) ) );
}

int main( void )
{
	b3DynamicTree tree = b3DynamicTree_Create( 16 );
	PrintTree( "empty", &tree );

	// Build by insertion, with growth of both pools
	for ( int i = 0; i < BOX_COUNT; ++i )
	{
		boxes[i] = RandomBox( 10.0f, 1.2f );
		proxies[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], CategoryBits( i ), UserData( i ) );
		alive[i] = true;
		if ( i == 0 || i == 1 || i == 2 || i == 15 || i == 16 || i == 100 )
		{
			char name[32];
			snprintf( name, sizeof( name ), "insert %d", i );
			PrintTree( name, &tree );
		}
	}
	PrintTree( "inserted", &tree );
	RunQueries( "inserted", &tree );

	// Full rebuild
	int sorted = b3DynamicTree_Rebuild( &tree, true );
	printf( "rebuild full | %d\n", sorted );
	PrintTree( "rebuilt", &tree );
	RunQueries( "rebuilt", &tree );

	// Churn: destroy, move, enlarge
	for ( int i = 0; i < BOX_COUNT; i += 5 )
	{
		b3DynamicTree_DestroyProxy( &tree, proxies[i] );
		alive[i] = false;
	}
	PrintTree( "destroyed", &tree );

	for ( int i = 1; i < BOX_COUNT; i += 3 )
	{
		if ( alive[i] == false )
		{
			continue;
		}
		boxes[i] = RandomBox( 10.0f, 1.2f );
		b3DynamicTree_MoveProxy( &tree, proxies[i], boxes[i] );
	}
	PrintTree( "moved", &tree );

	for ( int i = 2; i < BOX_COUNT; i += 4 )
	{
		if ( alive[i] == false )
		{
			continue;
		}
		boxes[i] = b3AABB_Inflate( boxes[i], 0.3f );
		b3DynamicTree_EnlargeProxy( &tree, proxies[i], boxes[i] );
	}
	PrintTree( "enlarged", &tree );
	PrintMoved( "enlarged moved", &tree );

	b3DynamicTree_Refit( &tree );
	PrintTree( "refit unordered", &tree );

	sorted = b3DynamicTree_Rebuild( &tree, false );
	printf( "rebuild partial | %d\n", sorted );
	PrintTree( "partial", &tree );
	RunQueries( "partial", &tree );

	// Re-create into the holes with moved marks
	for ( int i = 0; i < BOX_COUNT; i += 5 )
	{
		boxes[i] = RandomBox( 10.0f, 1.2f );
		proxies[i] = b3CreateTreeProxyInternal( &tree, boxes[i], CategoryBits( i ), UserData( i ), true );
		alive[i] = true;
	}
	PrintTree( "recreated", &tree );

	// The solver path: mark moved with grown boxes, gather, sweep refit, partial rebuild
	sorted = b3DynamicTree_Rebuild( &tree, false );
	printf( "rebuild recreated | %d\n", sorted );
	PrintTree( "recreated rebuilt", &tree );

	for ( int i = 3; i < BOX_COUNT; i += 7 )
	{
		boxes[i] = b3AABB_Inflate( boxes[i], 0.25f );
		b3DynamicTree_MarkProxyMoved( &tree, proxies[i], boxes[i] );
	}
	b3DynamicTree_MarkProxyMovedSerial( &tree, proxies[11] );
	PrintTree( "marked", &tree );
	PrintMoved( "marked moved", &tree );

	b3DynamicTree_Refit( &tree );
	PrintTree( "refit sweep", &tree );

	sorted = b3DynamicTree_Rebuild( &tree, false );
	printf( "rebuild marked | %d\n", sorted );
	PrintTree( "marked rebuilt", &tree );
	RunQueries( "marked", &tree );

	for ( int i = 4; i < BOX_COUNT; i += 9 )
	{
		boxes[i] = RandomBox( 10.0f, 1.2f );
		b3DynamicTree_MoveProxyInternal( &tree, proxies[i], boxes[i], true );
	}
	PrintMoved( "internal moved", &tree );
	b3DynamicTree_ClearMoved( &tree );
	PrintTree( "cleared", &tree );

	// Save and load the byte image
	b3DynamicTree_Save( &tree, "dynamic_tree_golden.dat" );
	{
		FILE* file = fopen( "dynamic_tree_golden.dat", "rb" );
		static uint8_t image[1 << 20];
		int size = (int)fread( image, 1, sizeof( image ), file );
		fclose( file );
		printf( "saved | %d %08x\n", size, HashBytes( B3_HASH_INIT, image, size ) );
	}

	b3DynamicTree loaded = b3DynamicTree_Load( "dynamic_tree_golden.dat", 2.0f );
	remove( "dynamic_tree_golden.dat" );
	PrintTree( "loaded", &loaded );
	b3DynamicTree_Rebuild( &loaded, true );
	PrintTree( "loaded rebuilt", &loaded );
	b3DynamicTree_Destroy( &loaded );

	// Drain to a single proxy and back
	for ( int i = 0; i < BOX_COUNT - 1; ++i )
	{
		b3DynamicTree_DestroyProxy( &tree, proxies[i] );
		alive[i] = false;
	}
	PrintTree( "single", &tree );
	sorted = b3DynamicTree_Rebuild( &tree, true );
	printf( "rebuild single | %d\n", sorted );
	PrintTree( "single rebuilt", &tree );

	for ( int i = 0; i < 40; ++i )
	{
		boxes[i] = RandomBox( 3.0f, 0.5f );
		proxies[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], CategoryBits( i ), UserData( i ) );
	}
	PrintTree( "refilled", &tree );
	sorted = b3DynamicTree_Rebuild( &tree, false );
	printf( "rebuild refilled | %d\n", sorted );
	PrintTree( "refilled rebuilt", &tree );

	b3DynamicTree_Destroy( &tree );
	return 0;
}
