// Generates testdata/solver_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/solver_golden_gen.c $(ls src/*.c | grep -v -e src/solver.c -e src/wide8.c)"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/solver_golden $SRC
// $SCRATCH/solver_golden > ../../libs/box3d-go/testdata/solver_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD) and the validation build
// (-DBOX3D_VALIDATE). They must be identical.
//
// solver.c is included below (not linked) so its static functions can be called: the block
// partition (b3ComputeBlockCount, b3InitBlocks, GetWorkerStartIndex) and the body kernels
// (b3IntegrateVelocitiesTask with damping, gravity, forces and gyroscopic torque;
// b3IntegratePositionsTask with motion locks and speed clamps). A b3World is assembled by hand
// with only the fields those kernels read (gravity and the awake solver set's body state count).
//
// Each line: a label, "|", outputs as hex words. Inputs come from an LCG with no libm calls (the
// quaternions go through b3NormalizeQuat, whose sqrt is correctly rounded), so Go reproduces them
// exactly.
#include "solver.c"

#include <stdio.h>
#include <string.h>

static uint32_t s_seed = 2468;

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

static b3Quat RandomQuat( void )
{
	b3Quat q;
	q.v = RandomVec( -1.0f, 1.0f );
	q.s = RandomFloat( 0.1f, 1.0f );
	return b3NormalizeQuat( q );
}

// Symmetric, diagonally dominant
static b3Matrix3 RandomInertia( float lo, float hi )
{
	float d0 = RandomFloat( lo, hi );
	float d1 = RandomFloat( lo, hi );
	float d2 = RandomFloat( lo, hi );
	float o01 = RandomFloat( -0.2f * lo, 0.2f * lo );
	float o02 = RandomFloat( -0.2f * lo, 0.2f * lo );
	float o12 = RandomFloat( -0.2f * lo, 0.2f * lo );
	b3Matrix3 m;
	m.cx = ( b3Vec3 ){ d0, o01, o02 };
	m.cy = ( b3Vec3 ){ o01, d1, o12 };
	m.cz = ( b3Vec3 ){ o02, o12, d2 };
	return m;
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

static void pq( b3Quat q )
{
	pv( q.v );
	pf( q.s );
}

#define BODY_COUNT 150

int main( void )
{
	// Block partition
	{
		int itemCounts[] = { 0, 1, 3, 4, 5, 31, 32, 33, 100, 127, 128, 129, 1000, 4095, 70000 };
		int minSizes[] = { 1, 4, 32 };
		int workerCounts[] = { 1, 2, 3, 4, 8, 32 };
		for ( int a = 0; a < (int)( sizeof( itemCounts ) / sizeof( int ) ); ++a )
		{
			for ( int b = 0; b < (int)( sizeof( minSizes ) / sizeof( int ) ); ++b )
			{
				for ( int c = 0; c < (int)( sizeof( workerCounts ) / sizeof( int ) ); ++c )
				{
					int itemCount = itemCounts[a];
					int workerCount = workerCounts[c];
					b3BlockDim dim = b3ComputeBlockCount( itemCount, minSizes[b], 4 * workerCount );
					printf( "dim %d %d %d | %d %d |", itemCount, minSizes[b], workerCount, dim.size, dim.count );

					b3SyncBlock blocks[256];
					b3InitBlocks( blocks, dim, itemCount, b3_graphContactBlock, (uint8_t)( a + b ) );
					for ( int i = 0; i < dim.count; ++i )
					{
						printf( " %d:%d:%d:%d", blocks[i].block.startIndex, blocks[i].block.count, blocks[i].block.blockType,
								blocks[i].block.colorIndex );
					}

					printf( " |" );
					for ( int w = 0; w < workerCount; ++w )
					{
						printf( " %d", GetWorkerStartIndex( w, dim.count, workerCount ) );
					}
					printf( "\n" );
				}
			}
		}
	}

	// Body kernels
	static b3BodySim sims[BODY_COUNT];
	static b3BodyState states[BODY_COUNT];
	b3SolverSet sets[3];
	memset( sets, 0, sizeof( sets ) );
	sets[b3_awakeSet].bodyStates.data = states;
	sets[b3_awakeSet].bodyStates.count = BODY_COUNT;
	sets[b3_awakeSet].bodyStates.capacity = BODY_COUNT;

	b3World world;
	memset( &world, 0, sizeof( world ) );
	world.solverSets.data = sets;
	world.solverSets.count = 3;
	world.solverSets.capacity = 3;
	world.gravity = ( b3Vec3 ){ 0.0f, -10.0f, 0.0f };

	for ( int i = 0; i < BODY_COUNT; ++i )
	{
		b3BodySim* sim = sims + i;
		b3BodyState* state = states + i;
		memset( sim, 0, sizeof( *sim ) );
		*state = b3_identityBodyState;

		uint32_t r = NextRandom();
		sim->transform.q = RandomQuat();
		sim->invMass = ( r & 7 ) == 0 ? 0.0f : RandomFloat( 0.1f, 2.0f );
		sim->gravityScale = RandomFloat( -1.0f, 2.0f );
		sim->force = RandomVec( -20.0f, 20.0f );
		sim->torque = RandomVec( -5.0f, 5.0f );
		sim->linearDamping = ( r & 8 ) ? RandomFloat( 0.0f, 2.0f ) : 0.0f;
		sim->angularDamping = ( r & 16 ) ? RandomFloat( 0.0f, 2.0f ) : 0.0f;
		sim->inertia = RandomInertia( 0.5f, 4.0f );
		sim->invInertiaWorld = RandomInertia( 0.25f, 2.0f );
		sim->flags = ( r & 32 ) ? b3_enableGyroscopicTorque : 0;

		state->linearVelocity = RandomVec( -25.0f, 25.0f );
		state->angularVelocity = RandomVec( -60.0f, 60.0f );
		state->deltaRotation = RandomQuat();
		state->deltaPosition = RandomVec( -0.1f, 0.1f );
		state->flags = ( r >> 8 ) & ( b3_allLocks | b3_allowFastRotation );
	}

	b3StepContext context;
	memset( &context, 0, sizeof( context ) );
	context.world = &world;
	context.states = states;
	context.sims = sims;
	context.dt = 1.0f / 60.0f;
	context.inv_dt = 60.0f;
	context.h = context.dt / 4.0f;
	context.inv_h = 4.0f * context.inv_dt;
	context.maxLinearVelocity = 30.0f;

	b3BlockDim dim = b3ComputeBlockCount( BODY_COUNT, 32, 4 * 2 );
	b3SyncBlock blocks[16];
	b3InitBlocks( blocks, dim, BODY_COUNT, b3_bodyBlock, UINT8_MAX );

	for ( int subStep = 0; subStep < 4; ++subStep )
	{
		for ( int k = 0; k < dim.count; ++k )
		{
			b3IntegrateVelocitiesTask( blocks[k].block, &context );
		}

		for ( int i = 0; i < BODY_COUNT; ++i )
		{
			printf( "vel %d %d |", subStep, i );
			pv( states[i].linearVelocity );
			pv( states[i].angularVelocity );
			printf( "\n" );
		}

		for ( int k = dim.count - 1; k >= 0; --k )
		{
			b3IntegratePositionsTask( blocks[k].block, &context );
		}

		for ( int i = 0; i < BODY_COUNT; ++i )
		{
			printf( "pos %d %d |", subStep, i );
			pv( states[i].linearVelocity );
			pv( states[i].angularVelocity );
			pv( states[i].deltaPosition );
			pq( states[i].deltaRotation );
			printf( " %08x\n", states[i].flags );
		}
	}

	return 0;
}
