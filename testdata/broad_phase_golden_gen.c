// Generates testdata/broad_phase_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/broad_phase_golden_gen.c src/parallel_for.c src/table.c
//      src/dynamic_tree.c src/arena_allocator.c src/bitset.c src/compound.c src/mesh.c src/height_field.c
//      src/hull.c src/distance.c src/sphere.c src/capsule.c src/manifold.c src/aabb.c
//      src/math_functions.c src/core.c src/timer.c src/simd.c src/types.c"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/broad_phase_golden $SRC
// $SCRATCH/broad_phase_golden > ../../libs/box3d-go/testdata/broad_phase_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD) and the validation build
// (-DBOX3D_VALIDATE). They must be identical.
//
// broad_phase.c is included below (not linked) so the static pair helpers can be called for the
// diagnostics. A b3World is assembled by hand with only the fields the broad-phase reads: shapes,
// bodies, joints, the static solver set (compound body transforms), worker task contexts, the stack
// allocator and the serial default task system. b3CreateContact (contact.c) is replaced by a stub
// that records the key and adds it to the pair set, which is the only effect the broad-phase sees.
// body.c and shape.c functions the build needs are verbatim copies from the pinned commit.
//
// Each line: a label, "|", outputs. Rounds mutate the proxies (move, mark, destroy/recreate,
// static moves, pair set removals, joint toggles) and run b3UpdateBroadPhasePairs. Lines labelled
// "arrival" depend on the worker distribution of the parallel-for; every other line is
// independent of it. Inputs come from an xorshift generator with no libm calls.
#include "broad_phase.c"

#include "joint.h"

#include "box3d/collision.h"

#include <stdio.h>
#include <stdlib.h>

// ---- verbatim copies (body.c, shape.c) ----

b3WorldTransform b3GetBodyTransformQuick( b3World* world, b3Body* body )
{
	b3SolverSet* set = b3Array_Get( world->solverSets, body->setIndex );
	b3BodySim* bodySim = b3Array_Get( set->bodySims, body->localIndex );
	return bodySim->transform;
}

b3WorldTransform b3GetBodyTransform( b3World* world, int bodyId )
{
	b3Body* body = b3Array_Get( world->bodies, bodyId );
	return b3GetBodyTransformQuick( world, body );
}

bool b3ShouldBodiesCollide( b3World* world, b3Body* bodyA, b3Body* bodyB )
{
	if ( bodyA->type != b3_dynamicBody && bodyB->type != b3_dynamicBody )
	{
		return false;
	}

	int jointKey;
	int otherBodyId;
	if ( bodyA->jointCount < bodyB->jointCount )
	{
		jointKey = bodyA->headJointKey;
		otherBodyId = bodyB->id;
	}
	else
	{
		jointKey = bodyB->headJointKey;
		otherBodyId = bodyA->id;
	}

	while ( jointKey != B3_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;
		int otherEdgeIndex = edgeIndex ^ 1;

		b3Joint* joint = b3Array_Get( world->joints, jointId );
		if ( joint->collideConnected == false && joint->edges[otherEdgeIndex].bodyId == otherBodyId )
		{
			return false;
		}

		jointKey = joint->edges[edgeIndex].nextKey;
	}

	return true;
}

b3ShapeProxy b3MakeLocalProxy( const b3ShapeProxy* proxy, b3Transform transform, b3Vec3* buffer )
{
	b3Transform invTransform = b3InvertTransform( transform );
	b3Matrix3 R = b3MakeMatrixFromQuat( invTransform.q );

	int count = b3MinInt( proxy->count, B3_MAX_SHAPE_CAST_POINTS );
	for ( int i = 0; i < count; ++i )
	{
		buffer[i] = b3Add( b3MulMV( R, proxy->points[i] ), invTransform.p );
	}

	return ( b3ShapeProxy ){
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

// ---- stubs ----

#define MAX_CREATED 4096

static uint64_t s_created[MAX_CREATED];
static int s_createdCount;

// Stands in for contact.c: record the pair and add its key to the pair set.
void b3CreateContact( b3World* world, b3Shape* shapeA, b3Shape* shapeB, int childIndex )
{
	uint64_t pairKey = b3ShapePairKey( shapeA->id, shapeB->id, childIndex );
	if ( s_createdCount < MAX_CREATED )
	{
		s_created[s_createdCount] = pairKey;
	}
	s_createdCount += 1;
	b3AddKey( &world->broadPhase.pairSet, pairKey );
}

void b3ValidateSolverSets( b3World* world )
{
	B3_UNUSED( world );
}

static void* SerialEnqueue( b3TaskCallback* task, void* taskContext, void* userContext, const char* name )
{
	B3_UNUSED( userContext, name );
	task( taskContext );
	return NULL;
}

static void SerialFinish( void* userTask, void* userContext )
{
	B3_UNUSED( userTask, userContext );
}

static bool CustomFilter( b3ShapeId idA, b3ShapeId idB, void* context )
{
	B3_UNUSED( context );
	return ( ( idA.index1 * 7 + idB.index1 + idA.generation ) % 3 ) != 0;
}

// ---- generator ----

static uint32_t s_seed = 0x2545F491u;

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

// Sequenced: C leaves the evaluation order of initializer lists unspecified.
static b3Vec3 RandomVec( float lo, float hi )
{
	b3Vec3 v;
	v.x = RandomFloat( lo, hi );
	v.y = RandomFloat( lo, hi );
	v.z = RandomFloat( lo, hi );
	return v;
}

static b3AABB RandomBox( b3Vec3 center, float minHalf, float maxHalf )
{
	b3Vec3 half = RandomVec( minHalf, maxHalf );
	return ( b3AABB ){ b3Sub( center, half ), b3Add( center, half ) };
}

#define DYNAMIC_BODIES 120
#define KINEMATIC_BODIES 12
#define STATIC_BODIES 50
#define COMPOUND_BODIES 2
#define BODY_COUNT ( DYNAMIC_BODIES + KINEMATIC_BODIES + STATIC_BODIES + COMPOUND_BODIES )
#define MAX_SHAPES ( 2 * BODY_COUNT )
#define JOINT_COUNT 10
#define WORKER_COUNT 2
#define ROUND_COUNT 8
#define EXTENT 8.0f

static b3World s_world;
static b3Body s_bodies[BODY_COUNT];
static b3Shape s_shapes[MAX_SHAPES];
static b3Joint s_joints[JOINT_COUNT];
static b3BodySim s_bodySims[COMPOUND_BODIES];
static b3SolverSet s_solverSets[1];
static b3TaskContext s_taskContexts[WORKER_COUNT];

static void PrintHash( const void* data, int byteCount )
{
	printf( " %08x", b3Hash( B3_HASH_INIT, (const uint8_t*)data, byteCount ) );
}

static b3BodyType BodyTypeOf( int bodyIndex )
{
	if ( bodyIndex < DYNAMIC_BODIES )
	{
		return b3_dynamicBody;
	}
	if ( bodyIndex < DYNAMIC_BODIES + KINEMATIC_BODIES )
	{
		return b3_kinematicBody;
	}
	return b3_staticBody;
}

static void PrintTree( const char* label, const b3DynamicTree* tree )
{
	printf( " %s %d %d %d", label, tree->nodeEnd, tree->proxyCount, tree->dfsOrdered ? 1 : 0 );
	PrintHash( tree->nodes, tree->nodeEnd * (int)sizeof( b3TreeNode ) );
}

int main( void )
{
	b3World* world = &s_world;
	b3BroadPhase* bp = &world->broadPhase;

	b3Capacity capacity = { 0 };
	b3CreateBroadPhase( bp, &capacity );
	world->stack = b3CreateStack( 1 << 20 );
	world->workerCount = WORKER_COUNT;
	world->taskContexts.data = s_taskContexts;
	world->taskContexts.count = WORKER_COUNT;
	world->taskContexts.capacity = WORKER_COUNT;
	world->enqueueTaskFcn = SerialEnqueue;
	world->finishTaskFcn = SerialFinish;
	world->userTaskContext = NULL;
	world->customFilterFcn = CustomFilter;
	world->customFilterContext = NULL;
	world->worldId = 0;

	// Compounds: spheres only, on static bodies with exactly representable rotations.
	b3CompoundData* compounds[COMPOUND_BODIES];
	b3Quat compoundRotations[COMPOUND_BODIES] = { { { 0.0f, 0.6f, 0.0f }, 0.8f }, { { 0.8f, 0.0f, 0.0f }, 0.6f } };
	for ( int i = 0; i < COMPOUND_BODIES; ++i )
	{
		b3CompoundSphereDef spheres[12];
		for ( int j = 0; j < 12; ++j )
		{
			spheres[j] = ( b3CompoundSphereDef ){ 0 };
			spheres[j].sphere.center = RandomVec( -6.0f, 6.0f );
			spheres[j].sphere.radius = RandomFloat( 0.5f, 1.5f );
		}
		b3CompoundDef def = { 0 };
		def.spheres = spheres;
		def.sphereCount = 12;
		compounds[i] = b3CreateCompound( &def );

		s_bodySims[i].transform.p = RandomVec( -10.0f, 10.0f );
		s_bodySims[i].transform.q = compoundRotations[i];
	}
	s_solverSets[0].bodySims.data = s_bodySims;
	s_solverSets[0].bodySims.count = COMPOUND_BODIES;
	s_solverSets[0].bodySims.capacity = COMPOUND_BODIES;
	world->solverSets.data = s_solverSets;
	world->solverSets.count = 1;
	world->solverSets.capacity = 1;
	world->compoundShapeCount = COMPOUND_BODIES;

	// Bodies
	for ( int i = 0; i < BODY_COUNT; ++i )
	{
		b3Body* body = s_bodies + i;
		body->id = i;
		body->type = BodyTypeOf( i );
		body->setIndex = body->type == b3_staticBody ? b3_staticSet : b3_awakeSet;
		body->localIndex = i >= BODY_COUNT - COMPOUND_BODIES ? i - ( BODY_COUNT - COMPOUND_BODIES ) : B3_NULL_INDEX;
		body->headJointKey = B3_NULL_INDEX;
		body->jointCount = 0;
		body->headShapeId = B3_NULL_INDEX;
	}
	world->bodies.data = s_bodies;
	world->bodies.count = BODY_COUNT;
	world->bodies.capacity = BODY_COUNT;

	// Joints between dynamic bodies, linked as b3CreateJoint links them
	for ( int j = 0; j < JOINT_COUNT; ++j )
	{
		int bodyIdA = RandomInt( DYNAMIC_BODIES );
		int bodyIdB = ( bodyIdA + 1 + RandomInt( DYNAMIC_BODIES - 1 ) ) % DYNAMIC_BODIES;
		b3Joint* joint = s_joints + j;
		joint->jointId = j;
		joint->collideConnected = RandomInt( 2 ) == 0;

		b3Body* bodyA = s_bodies + bodyIdA;
		b3Body* bodyB = s_bodies + bodyIdB;

		joint->edges[0].bodyId = bodyIdA;
		joint->edges[0].prevKey = B3_NULL_INDEX;
		joint->edges[0].nextKey = bodyA->headJointKey;
		int keyA = ( j << 1 ) | 0;
		if ( bodyA->headJointKey != B3_NULL_INDEX )
		{
			b3Joint* head = s_joints + ( bodyA->headJointKey >> 1 );
			head->edges[bodyA->headJointKey & 1].prevKey = keyA;
		}
		bodyA->headJointKey = keyA;
		bodyA->jointCount += 1;

		joint->edges[1].bodyId = bodyIdB;
		joint->edges[1].prevKey = B3_NULL_INDEX;
		joint->edges[1].nextKey = bodyB->headJointKey;
		int keyB = ( j << 1 ) | 1;
		if ( bodyB->headJointKey != B3_NULL_INDEX )
		{
			b3Joint* head = s_joints + ( bodyB->headJointKey >> 1 );
			head->edges[bodyB->headJointKey & 1].prevKey = keyB;
		}
		bodyB->headJointKey = keyB;
		bodyB->jointCount += 1;
	}
	world->joints.data = s_joints;
	world->joints.count = JOINT_COUNT;
	world->joints.capacity = JOINT_COUNT;

	// Shapes and proxies
	int shapeCount = 0;
	for ( int i = 0; i < BODY_COUNT; ++i )
	{
		b3BodyType type = BodyTypeOf( i );
		bool isCompound = i >= BODY_COUNT - COMPOUND_BODIES;
		int count = type == b3_dynamicBody ? 1 + RandomInt( 2 ) : 1;
		for ( int k = 0; k < count; ++k )
		{
			int shapeId = shapeCount++;
			b3Shape* shape = s_shapes + shapeId;
			shape->id = shapeId;
			shape->bodyId = i;
			shape->type = isCompound ? b3_compoundShape : b3_sphereShape;
			shape->filter.categoryBits = 1ull << RandomInt( 3 );
			shape->filter.maskBits = RandomInt( 4 ) == 0 ? 0x5ull : B3_DEFAULT_MASK_BITS;
			shape->filter.groupIndex = RandomInt( 8 ) == 0 ? -1 : 0;
			shape->flags = RandomInt( 5 ) == 0 ? b3_enableCustomFiltering : 0;
			shape->sensorIndex = ( isCompound == false && RandomInt( 15 ) == 0 ) ? 0 : B3_NULL_INDEX;
			shape->generation = (uint16_t)( 1 + RandomInt( 100 ) );

			b3AABB aabb;
			bool force = false;
			if ( isCompound )
			{
				int c = i - ( BODY_COUNT - COMPOUND_BODIES );
				shape->compound = compounds[c];
				aabb = b3ComputeCompoundAABB( compounds[c], s_bodySims[c].transform );
			}
			else if ( type == b3_staticBody )
			{
				aabb = RandomBox( RandomVec( -EXTENT, EXTENT ), 0.5f, 4.0f );
				force = RandomInt( 3 ) == 0;
			}
			else
			{
				aabb = RandomBox( RandomVec( -EXTENT, EXTENT ), 0.2f, 2.0f );
			}

			shape->proxyKey = b3BroadPhase_CreateProxy( bp, type, aabb, shape->filter.categoryBits, shapeId, force );
		}
	}
	world->shapes.data = s_shapes;
	world->shapes.count = shapeCount;
	world->shapes.capacity = MAX_SHAPES;

	printf( "setup | %d", shapeCount );
	PrintTree( "static", bp->trees + b3_staticBody );
	PrintTree( "kinematic", bp->trees + b3_kinematicBody );
	PrintTree( "dynamic", bp->trees + b3_dynamicBody );
	printf( "\n" );

	int32_t* siblings = malloc( sizeof( int32_t ) * MAX_SHAPES * 4 );
	b3NodePair seeds[2 * B3_CROSS_SEED_COUNT];

	for ( int round = 0; round < ROUND_COUNT; ++round )
	{
		if ( round > 0 && round != 4 )
		{
			// Remove every third pair created last round, as destroyed contacts would.
			for ( int i = 0; i < s_createdCount && i < MAX_CREATED; i += 3 )
			{
				b3RemoveKey( &bp->pairSet, s_created[i] );
			}

			int moveCount = 10 + RandomInt( 20 );
			for ( int m = 0; m < moveCount; ++m )
			{
				int shapeId = RandomInt( shapeCount );
				b3Shape* shape = s_shapes + shapeId;
				b3Vec3 shift = RandomVec( -2.0f, 2.0f );
				if ( shape->proxyKey == B3_NULL_INDEX || shape->type == b3_compoundShape )
				{
					continue;
				}
				b3AABB old =
					b3DynamicTree_GetAABB( bp->trees + B3_PROXY_TYPE( shape->proxyKey ), B3_PROXY_ID( shape->proxyKey ) );
				b3Vec3 center = b3MulSV( 0.5f, b3Add( old.lowerBound, old.upperBound ) );
				b3AABB aabb = RandomBox( b3Add( center, shift ), 0.2f, 2.0f );
				b3BroadPhase_MoveProxy( bp, shape->proxyKey, aabb );
			}

			int markCount = RandomInt( 5 );
			for ( int m = 0; m < markCount; ++m )
			{
				int shapeId = RandomInt( shapeCount );
				b3Shape* shape = s_shapes + shapeId;
				if ( shape->proxyKey != B3_NULL_INDEX && B3_PROXY_TYPE( shape->proxyKey ) != b3_staticBody )
				{
					b3BroadPhase_MarkProxyMovedSerial( bp, shape->proxyKey );
				}
			}

			int toggleCount = RandomInt( 4 );
			for ( int m = 0; m < toggleCount; ++m )
			{
				int shapeId = RandomInt( shapeCount );
				b3Shape* shape = s_shapes + shapeId;
				b3AABB aabb = RandomBox( RandomVec( -EXTENT, EXTENT ), 0.2f, 2.0f );
				bool force = RandomInt( 2 ) == 0;
				if ( shape->type == b3_compoundShape )
				{
					continue;
				}
				if ( shape->proxyKey != B3_NULL_INDEX )
				{
					b3BroadPhase_DestroyProxy( bp, shape->proxyKey );
					shape->proxyKey = B3_NULL_INDEX;
				}
				else
				{
					b3BodyType type = s_bodies[shape->bodyId].type;
					shape->proxyKey = b3BroadPhase_CreateProxy( bp, type, aabb, shape->filter.categoryBits, shapeId, force );
				}
			}

			int jointIndex = RandomInt( JOINT_COUNT );
			s_joints[jointIndex].collideConnected = !s_joints[jointIndex].collideConnected;
		}

		// Diagnostics from the static helpers before the update (pure reads)
		{
			const b3DynamicTree* dynamicTree = bp->trees + b3_dynamicBody;
			int siblingCount = b3GatherMovedSiblings( dynamicTree, siblings );
			int staticSeedCount = b3GatherCrossSeeds( dynamicTree, bp->trees + b3_staticBody, seeds );
			int kinematicSeedCount =
				b3GatherCrossSeeds( dynamicTree, bp->trees + b3_kinematicBody, seeds + staticSeedCount );
			printf( "round %d pre | %d", round, siblingCount );
			PrintHash( siblings, siblingCount * (int)sizeof( int32_t ) );
			printf( " %d %d", staticSeedCount, kinematicSeedCount );
			PrintHash( seeds, ( staticSeedCount + kinematicSeedCount ) * (int)sizeof( b3NodePair ) );
			printf( "\n" );
		}

		for ( int i = 0; i < WORKER_COUNT; ++i )
		{
			b3Array_Clear( s_taskContexts[i].pairKeys );
		}
		world->taskCount = 0;
		world->activeTaskCount = 0;
		s_createdCount = 0;

		b3UpdateBroadPhasePairs( world );

		printf( "round %d arrival |", round );
		for ( int i = 0; i < WORKER_COUNT; ++i )
		{
			printf( " %d", s_taskContexts[i].pairKeys.count );
			PrintHash( s_taskContexts[i].pairKeys.data, s_taskContexts[i].pairKeys.count * (int)sizeof( uint64_t ) );
		}
		printf( "\n" );

		printf( "round %d contacts | %d", round, s_createdCount );
		PrintHash( s_created, b3MinInt( s_createdCount, MAX_CREATED ) * (int)sizeof( uint64_t ) );
		for ( int i = 0; i < s_createdCount && i < 6; ++i )
		{
			printf( " %016llx", (unsigned long long)s_created[i] );
		}
		int compoundPairs = 0;
		int childPairs = 0;
		for ( int i = 0; i < s_createdCount && i < MAX_CREATED; ++i )
		{
			uint64_t key = s_created[i];
			int shapeIdA = (int)( ( key >> ( 64 - B3_SHAPE_POWER ) ) & B3_SHAPE_MASK );
			int shapeIdB = (int)( ( key >> ( 64 - 2 * B3_SHAPE_POWER ) ) & B3_SHAPE_MASK );
			if ( s_shapes[shapeIdA].type == b3_compoundShape || s_shapes[shapeIdB].type == b3_compoundShape )
			{
				compoundPairs += 1;
			}
			if ( ( key & B3_CHILD_MASK ) != 0 )
			{
				childPairs += 1;
			}
		}
		printf( " | compound %d %d | set %u\n", compoundPairs, childPairs, bp->pairSet.count );

		printf( "round %d trees |", round );
		PrintTree( "static", bp->trees + b3_staticBody );
		PrintTree( "kinematic", bp->trees + b3_kinematicBody );
		PrintTree( "dynamic", bp->trees + b3_dynamicBody );
		printf( "\n" );

		printf( "round %d probe |", round );
		for ( int p = 0; p < 6; ++p )
		{
			int shapeIdA = RandomInt( shapeCount );
			int shapeIdB = RandomInt( shapeCount );
			int keyA = s_shapes[shapeIdA].proxyKey;
			int keyB = s_shapes[shapeIdB].proxyKey;
			if ( keyA == B3_NULL_INDEX || keyB == B3_NULL_INDEX )
			{
				printf( " -" );
				continue;
			}
			printf( " %d:%d:%d", b3BroadPhase_TestOverlap( bp, keyA, keyB ) ? 1 : 0,
					b3BroadPhase_GetShapeIndex( bp, keyA ), b3BroadPhase_GetShapeIndex( bp, keyB ) );
		}
		printf( "\n" );

		b3ValidateBroadPhase( bp );
		b3ValidateNoMoved( bp );
	}

	free( siblings );
	b3DestroyBroadPhase( bp );
	b3DestroyStack( &world->stack );
	for ( int i = 0; i < WORKER_COUNT; ++i )
	{
		b3Array_Destroy( s_taskContexts[i].pairKeys );
	}
	for ( int i = 0; i < COMPOUND_BODIES; ++i )
	{
		b3DestroyCompound( compounds[i] );
	}
	return 0;
}
