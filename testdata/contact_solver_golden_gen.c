// Generates testdata/contact_solver_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/contact_solver_golden_gen.c src/bitset.c src/math_functions.c src/core.c"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/contact_solver_golden $SRC
// $SCRATCH/contact_solver_golden > ../../libs/box3d-go/testdata/contact_solver_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD, which selects the
// B3_SIMD_NONE gather/scatter), the validation build (-DBOX3D_VALIDATE) and the x86_64 SSE2 build
// (-arch x86_64, add src/wide8.c to SRC because B3_SIMD_HAS_WIDTH_8 references the W8 kernels; the
// world uses width 4). They must be identical.
//
// contact_solver.c and contact_solver_wide.inl (at B3_SIMD_WIDTH 4, as wide4.c compiles it) are
// included below. A b3World and b3StepContext are assembled by hand with only the fields the
// contact solver reads: awake body states and sims, contacts with manifolds, two active graph
// colors holding convex contacts (wide constraints) and mesh contacts (wide mesh constraints), the
// overflow color, the prepare spans, the wide buffers sliced per color as solver.c slices them, and
// two worker task contexts. The test runs prepare, then per sub-step warm start, solve, a simple
// position integration and push (relax), then restitution and store impulses, and prints the body
// velocities after every stage and the stored manifold impulses, normal velocities and hit event bits
// at the end. Inputs come from an xorshift generator; the only libm call is sqrtf.
#include "contact_solver.c"

#define B3_SIMD_WIDTH 4
#include "contact_solver_wide.inl"

#include "bitset.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BODY_COUNT 24
#define STATIC_BODY_ID BODY_COUNT
#define MAX_CONTACTS 64
#define MAX_MANIFOLDS 256
#define COLOR_COUNT 2

static uint32_t s_seed;

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

static float RandomFloat( float lo, float hi )
{
	float t = (float)( NextRandom() >> 8 ) / (float)( 1 << 24 );
	return lo + t * ( hi - lo );
}

static int RandomInt( int count )
{
	return (int)( NextRandom() % (uint32_t)count );
}

static b3Vec3 RandomVec( float lo, float hi )
{
	b3Vec3 v;
	v.x = RandomFloat( lo, hi );
	v.y = RandomFloat( lo, hi );
	v.z = RandomFloat( lo, hi );
	return v;
}

static uint32_t Bits( float f )
{
	uint32_t u;
	memcpy( &u, &f, sizeof( u ) );
	return u;
}

static b3World s_world;
static b3BodyState s_states[BODY_COUNT];
static b3BodySim s_sims[BODY_COUNT];
static b3Body s_bodies[BODY_COUNT + 1];
static b3Contact s_contacts[MAX_CONTACTS];
static b3Manifold s_manifolds[MAX_MANIFOLDS];
static b3SolverSet s_sets[3];
static b3TaskContext s_taskContexts[2];
static int s_contactCount;
static int s_manifoldCount;

static int s_convexIds[COLOR_COUNT][8];
static int s_convexCounts[COLOR_COUNT] = { 7, 4 };
static b3ContactSpec s_meshSpecs[COLOR_COUNT][8];
static int s_meshCounts[COLOR_COUNT] = { 6, 3 };
static b3ContactSpec s_overflowSpecs[8];
static int s_overflowCount = 5;

static int CreateContact( int manifoldCount )
{
	int id = s_contactCount++;
	b3Contact* c = s_contacts + id;
	memset( c, 0, sizeof( *c ) );
	c->contactId = id;

	int r = RandomInt( 6 );
	int a = RandomInt( BODY_COUNT );
	int b = RandomInt( BODY_COUNT - 1 );
	if ( b >= a )
	{
		b += 1;
	}

	c->edges[0].bodyId = a;
	c->edges[1].bodyId = b;
	c->encodedBodySimA = a;
	c->encodedBodySimB = b;
	if ( r == 0 )
	{
		c->edges[0].bodyId = STATIC_BODY_ID;
		c->encodedBodySimA = -2;
		c->flags |= b3_contactStaticFlag;
	}
	else if ( r == 1 )
	{
		c->edges[1].bodyId = STATIC_BODY_ID;
		c->encodedBodySimB = -2;
		c->flags |= b3_contactStaticFlag;
	}

	if ( RandomInt( 3 ) == 0 )
	{
		c->flags |= b3_simEnableHitEvent;
	}

	c->friction = RandomFloat( 0.0f, 1.0f );
	c->restitution = 0.0f;
	if ( RandomInt( 3 ) == 0 )
	{
		c->restitution = RandomFloat( 0.1f, 0.9f );
	}
	c->rollingResistance = 0.0f;
	if ( RandomInt( 3 ) == 0 )
	{
		c->rollingResistance = RandomFloat( 0.01f, 0.3f );
	}
	c->tangentVelocity = b3Vec3_zero;
	if ( RandomInt( 4 ) == 0 )
	{
		c->tangentVelocity = RandomVec( -1.0f, 1.0f );
	}

	c->manifolds = s_manifolds + s_manifoldCount;
	c->manifoldCount = manifoldCount;
	s_manifoldCount += manifoldCount;

	for ( int i = 0; i < manifoldCount; ++i )
	{
		b3Manifold* m = c->manifolds + i;
		memset( m, 0, sizeof( *m ) );
		b3Vec3 n = RandomVec( -1.0f, 1.0f );
		m->normal = b3Normalize( n );
		m->twistImpulse = RandomFloat( -0.1f, 0.1f );
		m->frictionImpulse = RandomVec( -0.2f, 0.2f );
		m->rollingImpulse = RandomVec( -0.05f, 0.05f );
		m->pointCount = 1 + RandomInt( 4 );
		for ( int j = 0; j < m->pointCount; ++j )
		{
			b3ManifoldPoint* mp = m->points + j;
			mp->anchorA = RandomVec( -0.5f, 0.5f );
			mp->anchorB = RandomVec( -0.5f, 0.5f );
			mp->separation = RandomFloat( -0.03f, 0.06f );
			mp->normalImpulse = RandomFloat( 0.0f, 0.5f );
		}
	}

	return id;
}

// Wide buffers and spans, laid out as solver.c lays them out.
static void* s_wideConstraints;
static void* s_wideMeshConstraints;
static int s_meshLaneOrder[16];
static int s_meshManifoldStarts[8];
static b3WidePrepareSpan s_widePrepareSpans[COLOR_COUNT + 1];
static b3MeshPrepareSpan s_meshPrepareSpans[COLOR_COUNT + 1];
static b3ContactPrepareSpan s_overflowSpans[2];
static b3ContactConstraint s_overflowConstraints[8];
static b3ManifoldConstraint s_overflowManifoldConstraints[32];
static int s_wideContactCount;
static int s_meshGroupCount;

static void CreateScene( uint32_t seed, bool warmStarting, bool propagate, b3StepContext* context )
{
	s_seed = seed;
	s_contactCount = 0;
	s_manifoldCount = 0;

	memset( &s_world, 0, sizeof( s_world ) );

	for ( int i = 0; i < BODY_COUNT; ++i )
	{
		b3BodyState* state = s_states + i;
		memset( state, 0, sizeof( *state ) );
		state->linearVelocity = RandomVec( -2.0f, 2.0f );
		state->angularVelocity = RandomVec( -3.0f, 3.0f );
		state->deltaPosition = RandomVec( -0.05f, 0.05f );
		b3Quat q;
		q.v = RandomVec( -0.05f, 0.05f );
		q.s = 1.0f;
		state->deltaRotation = b3NormalizeQuat( q );
		bool dynamic = ( i % 5 ) != 4;
		state->flags = dynamic ? b3_dynamicFlag : 0;

		b3BodySim* sim = s_sims + i;
		memset( sim, 0, sizeof( *sim ) );
		float invMass = RandomFloat( 0.2f, 2.0f );
		b3Vec3 d = RandomVec( 0.5f, 3.0f );
		b3Vec3 o = RandomVec( -0.2f, 0.2f );
		if ( dynamic )
		{
			sim->invMass = invMass;
			sim->invInertiaWorld.cx = (b3Vec3){ d.x, o.x, o.y };
			sim->invInertiaWorld.cy = (b3Vec3){ o.x, d.y, o.z };
			sim->invInertiaWorld.cz = (b3Vec3){ o.y, o.z, d.z };
		}

		memset( s_bodies + i, 0, sizeof( b3Body ) );
		s_bodies[i].setIndex = b3_awakeSet;
		s_bodies[i].localIndex = i;
	}

	memset( s_bodies + STATIC_BODY_ID, 0, sizeof( b3Body ) );
	s_bodies[STATIC_BODY_ID].setIndex = b3_staticSet;
	s_bodies[STATIC_BODY_ID].localIndex = 0;

	for ( int c = 0; c < COLOR_COUNT; ++c )
	{
		for ( int i = 0; i < s_convexCounts[c]; ++i )
		{
			s_convexIds[c][i] = CreateContact( 1 );
		}
	}

	for ( int c = 0; c < COLOR_COUNT; ++c )
	{
		for ( int i = 0; i < s_meshCounts[c]; ++i )
		{
			int manifoldCount = 1 + RandomInt( 3 );
			b3ContactSpec* spec = s_meshSpecs[c] + i;
			spec->contactId = CreateContact( manifoldCount );
			spec->manifoldStart = 0;
			spec->manifoldCount = (uint16_t)manifoldCount;
		}
	}

	int overflowManifoldCount = 0;
	for ( int i = 0; i < s_overflowCount; ++i )
	{
		int manifoldCount = 1 + RandomInt( 3 );
		b3ContactSpec* spec = s_overflowSpecs + i;
		spec->contactId = CreateContact( manifoldCount );
		spec->manifoldStart = overflowManifoldCount;
		spec->manifoldCount = (uint16_t)manifoldCount;
		overflowManifoldCount += manifoldCount;
	}

	// World
	b3World* world = &s_world;
	world->contacts.data = s_contacts;
	world->contacts.count = s_contactCount;
	world->contacts.capacity = MAX_CONTACTS;
	world->bodies.data = s_bodies;
	world->bodies.count = BODY_COUNT + 1;
	world->bodies.capacity = BODY_COUNT + 1;
	memset( s_sets, 0, sizeof( s_sets ) );
	s_sets[b3_awakeSet].bodyStates.data = s_states;
	s_sets[b3_awakeSet].bodyStates.count = BODY_COUNT;
	s_sets[b3_awakeSet].bodyStates.capacity = BODY_COUNT;
	s_sets[b3_awakeSet].bodySims.data = s_sims;
	s_sets[b3_awakeSet].bodySims.count = BODY_COUNT;
	s_sets[b3_awakeSet].bodySims.capacity = BODY_COUNT;
	world->solverSets.data = s_sets;
	world->solverSets.count = 3;
	world->solverSets.capacity = 3;
	for ( int i = 0; i < 2; ++i )
	{
		b3TaskContext* tc = s_taskContexts + i;
		if ( tc->hitEventBitSet.bits == NULL )
		{
			tc->hitEventBitSet = b3CreateBitSet( MAX_CONTACTS );
		}
		b3SetBitCountAndClear( &tc->hitEventBitSet, MAX_CONTACTS );
		tc->hasHitEvents = false;
	}
	world->taskContexts.data = s_taskContexts;
	world->taskContexts.count = 2;
	world->taskContexts.capacity = 2;
	world->enableWarmStarting = warmStarting;
	world->enableRestitutionPropagation = propagate;
	world->contactSpeed = 3.0f;
	world->restitutionThreshold = 1.0f;
	world->hitEventThreshold = 1.0f;
	world->simdWidth = 4;

	b3GraphColor* colors = world->constraintGraph.colors;

	// Convex contacts: wide constraints
	int wideContactByteCount = b3GetWideContactConstraintByteCount( 4 );
	s_wideContactCount = 0;
	for ( int c = 0; c < COLOR_COUNT; ++c )
	{
		s_wideContactCount += ( ( s_convexCounts[c] - 1 ) >> 2 ) + 1;
	}
	free( s_wideConstraints );
	s_wideConstraints = calloc( (size_t)s_wideContactCount, (size_t)wideContactByteCount );

	// Mesh contacts: lane order and manifold starts
	s_meshGroupCount = 0;
	int meshSlotCount = 0;
	{
		int maxManifoldCount = 3;
		int bucketStarts[4];
		int orderBase = 0;
		for ( int c = 0; c < COLOR_COUNT; ++c )
		{
			int colorContactCount = s_meshCounts[c];
			const b3ContactSpec* specs = s_meshSpecs[c];
			int* order = s_meshLaneOrder + orderBase;

			for ( int k = 0; k <= maxManifoldCount; ++k )
			{
				bucketStarts[k] = 0;
			}

			for ( int j = 0; j < colorContactCount; ++j )
			{
				bucketStarts[specs[j].manifoldCount] += 1;
			}

			int start = 0;
			for ( int k = maxManifoldCount; k >= 0; --k )
			{
				int count = bucketStarts[k];
				bucketStarts[k] = start;
				start += count;
			}

			for ( int j = 0; j < colorContactCount; ++j )
			{
				order[bucketStarts[specs[j].manifoldCount]++] = j;
			}

			int colorMeshGroupCount = ( ( colorContactCount - 1 ) >> 2 ) + 1;
			for ( int g = 0; g < colorMeshGroupCount; ++g )
			{
				int slotCount = specs[order[g << 2]].manifoldCount;
				s_meshManifoldStarts[s_meshGroupCount + g] = meshSlotCount;
				meshSlotCount += slotCount;
			}

			orderBase += colorContactCount;
			s_meshGroupCount += colorMeshGroupCount;
		}
		s_meshManifoldStarts[s_meshGroupCount] = meshSlotCount;
	}

	int wideMeshConstraintByteCount = b3GetWideMeshConstraintByteCount( 4 );
	int wideMeshManifoldByteCount = b3GetWideMeshManifoldByteCount( 4 );
	free( s_wideMeshConstraints );
	s_wideMeshConstraints =
		calloc( 1, (size_t)( s_meshGroupCount * wideMeshConstraintByteCount + meshSlotCount * wideMeshManifoldByteCount ) );

	// Distribute to colors and build spans
	{
		int wideBase = 0;
		int contactBase = 0;
		int meshGroupBase = 0;
		for ( int c = 0; c < COLOR_COUNT; ++c )
		{
			b3GraphColor* color = colors + c;
			color->convexContacts.data = s_convexIds[c];
			color->convexContacts.count = s_convexCounts[c];
			color->convexContacts.capacity = 8;
			color->contacts.data = s_meshSpecs[c];
			color->contacts.count = s_meshCounts[c];
			color->contacts.capacity = 8;

			s_widePrepareSpans[c].start = wideBase;
			s_widePrepareSpans[c].count = s_convexCounts[c];
			s_widePrepareSpans[c].contacts = s_convexIds[c];
			color->wideConstraints = (uint8_t*)s_wideConstraints + wideBase * wideContactByteCount;
			int colorContactCountW = ( ( s_convexCounts[c] - 1 ) >> 2 ) + 1;
			color->wideConstraintCount = colorContactCountW;
			wideBase += colorContactCountW;

			s_meshPrepareSpans[c].start = meshGroupBase;
			s_meshPrepareSpans[c].count = s_meshCounts[c];
			s_meshPrepareSpans[c].contacts = s_meshSpecs[c];
			s_meshPrepareSpans[c].order = s_meshLaneOrder + contactBase;
			int colorMeshGroupCount = ( ( s_meshCounts[c] - 1 ) >> 2 ) + 1;
			color->wideMeshConstraints = (uint8_t*)s_wideMeshConstraints + meshGroupBase * wideMeshConstraintByteCount +
										 s_meshManifoldStarts[meshGroupBase] * wideMeshManifoldByteCount;
			color->wideMeshManifoldStarts = s_meshManifoldStarts + meshGroupBase;
			color->wideMeshConstraintCount = colorMeshGroupCount;
			meshGroupBase += colorMeshGroupCount;
			contactBase += s_meshCounts[c];
		}

		s_widePrepareSpans[COLOR_COUNT].start = s_wideContactCount;
		s_widePrepareSpans[COLOR_COUNT].count = 0;
		s_widePrepareSpans[COLOR_COUNT].contacts = NULL;
		s_meshPrepareSpans[COLOR_COUNT].start = s_meshGroupCount;
		s_meshPrepareSpans[COLOR_COUNT].count = 0;
		s_meshPrepareSpans[COLOR_COUNT].contacts = NULL;
		s_meshPrepareSpans[COLOR_COUNT].order = NULL;
	}

	b3GraphColor* overflow = colors + B3_OVERFLOW_INDEX;
	overflow->contacts.data = s_overflowSpecs;
	overflow->contacts.count = s_overflowCount;
	overflow->contacts.capacity = 8;
	overflow->contactConstraints = s_overflowConstraints;
	overflow->manifoldConstraints = s_overflowManifoldConstraints;
	s_overflowSpans[0].start = 0;
	s_overflowSpans[0].count = s_overflowCount;
	s_overflowSpans[0].contacts = s_overflowSpecs;
	s_overflowSpans[1].start = s_overflowCount;
	s_overflowSpans[1].count = 0;
	s_overflowSpans[1].contacts = NULL;

	memset( context, 0, sizeof( *context ) );
	context->world = world;
	context->graph = &world->constraintGraph;
	context->states = s_states;
	context->sims = s_sims;
	context->h = 1.0f / 120.0f;
	context->inv_h = 120.0f;
	context->contactSoftness = (b3Softness){ .biasRate = 30.5f, .massScale = 0.8f, .impulseScale = 0.2f };
	context->staticSoftness = (b3Softness){ .biasRate = 61.0f, .massScale = 0.9f, .impulseScale = 0.1f };
	context->wideConstraints = s_wideConstraints;
	context->widePrepareSpans = s_widePrepareSpans;
	context->wideContactCount = s_wideContactCount;
	context->overflowSpans = s_overflowSpans;
	context->wideMeshConstraints = s_wideMeshConstraints;
	context->wideMeshManifoldStarts = s_meshManifoldStarts;
	context->meshPrepareSpans = s_meshPrepareSpans;
	context->activeColorCount = COLOR_COUNT;
}

static b3SolverBlock Block( int startIndex, int count, int blockType, int colorIndex )
{
	b3SolverBlock block = {
		.startIndex = startIndex,
		.count = (uint16_t)count,
		.blockType = (uint8_t)blockType,
		.colorIndex = (uint8_t)colorIndex,
	};
	return block;
}

static void PrintStates( int run, const char* stage )
{
	for ( int i = 0; i < BODY_COUNT; ++i )
	{
		const b3BodyState* s = s_states + i;
		printf( "run %d %s body %d | %08x %08x %08x %08x %08x %08x\n", run, stage, i, Bits( s->linearVelocity.x ),
				Bits( s->linearVelocity.y ), Bits( s->linearVelocity.z ), Bits( s->angularVelocity.x ),
				Bits( s->angularVelocity.y ), Bits( s->angularVelocity.z ) );
	}
}

// Graph color blocks: color 0 is split in two blocks to exercise a non-zero start index.
typedef void SolveFcn( b3SolverBlock block, b3StepContext* context );

static void ColorStage( b3StepContext* context, SolveFcn* convexFcn, SolveFcn* meshFcn, void ( *overflowFcn )( b3StepContext* ) )
{
	for ( int c = 0; c < COLOR_COUNT; ++c )
	{
		b3GraphColor* color = context->graph->colors + c;
		int count = color->wideConstraintCount;
		if ( count > 1 )
		{
			convexFcn( Block( 0, 1, b3_graphWideContactBlock, c ), context );
			convexFcn( Block( 1, count - 1, b3_graphWideContactBlock, c ), context );
		}
		else
		{
			convexFcn( Block( 0, count, b3_graphWideContactBlock, c ), context );
		}

		meshFcn( Block( 0, color->wideMeshConstraintCount, b3_graphContactBlock, c ), context );
	}

	overflowFcn( context );
}

static void IntegratePositions( b3StepContext* context )
{
	float h = context->h;
	for ( int i = 0; i < BODY_COUNT; ++i )
	{
		b3BodyState* s = s_states + i;
		if ( ( s->flags & b3_dynamicFlag ) == 0 )
		{
			continue;
		}

		s->deltaPosition = b3MulAdd( s->deltaPosition, h, s->linearVelocity );
		b3Quat q = s->deltaRotation;
		q.v = b3MulAdd( q.v, 0.5f * h, s->angularVelocity );
		s->deltaRotation = b3NormalizeQuat( q );
	}
}

static void Run( int run, uint32_t seed, bool warmStarting, bool propagate )
{
	b3StepContext context;
	CreateScene( seed, warmStarting, propagate, &context );

	PrintStates( run, "initial" );

	// Prepare: flat blocks across colors, the first convex block ends inside color 0
	b3PrepareContacts_Convex( Block( 0, 1, b3_wideContactBlock, 0 ), &context );
	b3PrepareContacts_Convex( Block( 1, s_wideContactCount - 1, b3_wideContactBlock, 0 ), &context );
	b3PrepareContacts_MeshWide( Block( 0, 2, b3_contactBlock, 0 ), &context );
	b3PrepareContacts_MeshWide( Block( 2, s_meshGroupCount - 2, b3_contactBlock, 0 ), &context );
	b3PrepareContacts_Overflow( &context );
	printf( "run %d prepare | anyRestitution %d\n", run, b3AtomicLoadInt( &context.anyRestitution ) );
	PrintStates( run, "prepare" );

	for ( int subStep = 0; subStep < 2; ++subStep )
	{
		char label[32];

		ColorStage( &context, b3WarmStartContacts_Convex, b3WarmStartContacts_MeshWide, b3WarmStartContacts_Overflow );
		snprintf( label, sizeof( label ), "sub%d-warm", subStep );
		PrintStates( run, label );

		ColorStage( &context, b3SolveContacts_Convex, b3SolveContacts_MeshWide, b3SolveContacts_Overflow );
		snprintf( label, sizeof( label ), "sub%d-solve", subStep );
		PrintStates( run, label );

		IntegratePositions( &context );

		ColorStage( &context, b3PushContacts_Convex, b3PushContacts_MeshWide, b3PushContacts_Overflow );
		snprintf( label, sizeof( label ), "sub%d-push", subStep );
		PrintStates( run, label );
	}

	ColorStage( &context, b3ApplyRestitution_Convex, b3ApplyRestitution_MeshWide, b3ApplyRestitution_Overflow );
	PrintStates( run, "restitution" );

	b3StoreImpulses_Convex( Block( 0, s_wideContactCount, b3_wideContactBlock, 0 ), &context, 0 );
	b3StoreImpulses_MeshWide( Block( 0, 1, b3_contactBlock, 0 ), &context, 1 );
	b3StoreImpulses_MeshWide( Block( 1, s_meshGroupCount - 1, b3_contactBlock, 0 ), &context, 0 );
	b3StoreImpulses_Overflow( &context );

	for ( int i = 0; i < s_contactCount; ++i )
	{
		const b3Contact* c = s_contacts + i;
		for ( int j = 0; j < c->manifoldCount; ++j )
		{
			const b3Manifold* m = c->manifolds + j;
			printf( "run %d store contact %d manifold %d | %08x %08x %08x %08x %08x %08x %08x |", run, i, j,
					Bits( m->twistImpulse ), Bits( m->frictionImpulse.x ), Bits( m->frictionImpulse.y ),
					Bits( m->frictionImpulse.z ), Bits( m->rollingImpulse.x ), Bits( m->rollingImpulse.y ),
					Bits( m->rollingImpulse.z ) );
			for ( int k = 0; k < m->pointCount; ++k )
			{
				const b3ManifoldPoint* mp = m->points + k;
				printf( " %08x %08x %08x", Bits( mp->normalImpulse ), Bits( mp->totalNormalImpulse ), Bits( mp->normalVelocity ) );
			}
			printf( "\n" );
		}
	}

	for ( int w = 0; w < 2; ++w )
	{
		const b3TaskContext* tc = s_taskContexts + w;
		printf( "run %d hits worker %d | %d |", run, w, tc->hasHitEvents ? 1 : 0 );
		for ( int i = 0; i < s_contactCount; ++i )
		{
			if ( b3GetBit( &tc->hitEventBitSet, i ) )
			{
				printf( " %d", i );
			}
		}
		printf( "\n" );
	}
}

int main( void )
{
	Run( 0, 0x2545F491u, true, false );
	Run( 1, 0x9E3779B9u, false, true );
	Run( 2, 0x1B873593u, true, true );
	return 0;
}
