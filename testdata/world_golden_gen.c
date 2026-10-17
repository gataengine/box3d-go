// Generates testdata/world_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/world_golden_gen.c $(ls src/*.c | grep -v -e src/wide8.c)"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/world_golden $SRC
// $SCRATCH/world_golden > ../../libs/box3d-go/testdata/world_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD) and the validation build
// (-DBOX3D_VALIDATE). They must be identical.
//
// A small world without joints: a grid mesh ground, a tilted static box, a stack of boxes, a
// sphere, a capsule, a rock hull, a cylinder hull, a bullet sphere and a compound-free pile of
// small boxes, stepped WORLD_GOLDEN_STEPS times at 1/60 s with 4 sub-steps. Every
// WORLD_GOLDEN_STRIDE steps (and after the last step) it prints every body's transform and
// velocities as hex words, the awake body count and b3HashWorldState. The scene runs at 1 and at
// 4 workers (internal scheduler; pass the worker count as the argument); both print the same.
//
// Inputs are exactly representable or computed without libm, so Go reproduces them exactly.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "physics_world.h"
#include "recording.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WORLD_GOLDEN_STEPS 240
#define WORLD_GOLDEN_STRIDE 20

static uint32_t Bits( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	return u;
}

static void PrintVec( b3Vec3 v )
{
	printf( " %08x %08x %08x", Bits( v.x ), Bits( v.y ), Bits( v.z ) );
}

static b3BodyId CreateDynamic( b3WorldId worldId, b3Vec3 p, b3Quat q )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = p;
	bodyDef.rotation = q;
	return b3CreateBody( worldId, &bodyDef );
}

static void RunScene( int workerCount, int bodyCapacity )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyId bodies[64];
	int bodyCount = 0;

	// Ground: grid mesh
	b3MeshData* mesh = b3CreateGridMesh( 16, 16, 1.0f, 1, true );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( groundId, &shapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
		bodies[bodyCount++] = groundId;
	}

	// Tilted static box (exact rotation: 3-4-5 about z)
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 5.0f, 0.5f, 0.0f };
		bodyDef.rotation = (b3Quat){ { 0.0f, 0.0f, 0.6f }, 0.8f };
		b3BodyId staticId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 2.0f, 0.25f, 1.5f );
		b3CreateHullShape( staticId, &shapeDef, &box.base );
		bodies[bodyCount++] = staticId;
	}

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;

	// Box stack
	b3BoxHull cube = b3MakeCubeHull( 0.5f );
	for ( int i = 0; i < 5; ++i )
	{
		b3Vec3 p = { 0.0f, 0.5f + 1.0f * i, 0.0f };
		if ( i & 1 )
		{
			p.x = 0.125f;
		}
		b3BodyId bodyId = CreateDynamic( worldId, p, b3Quat_identity );
		b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		bodies[bodyCount++] = bodyId;
	}

	// Sphere rolling in from the side with spin
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -4.0f, 2.0f, 0.5f }, b3Quat_identity );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.05f;
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 3.0f, 0.0f, -0.5f } );
		b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 0.0f, 2.0f, -6.0f } );
		bodies[bodyCount++] = bodyId;
		shapeDef.baseMaterial = b3DefaultSurfaceMaterial();
	}

	// Capsule falling tilted onto the mesh
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -2.0f, 3.0f, 2.0f }, (b3Quat){ { 0.6f, 0.0f, 0.0f }, 0.8f } );
		b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.25f };
		b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		bodies[bodyCount++] = bodyId;
	}

	// Rock hull falling onto the tilted static box
	b3HullData* rock = b3CreateRock( 0.6f );
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ 5.0f, 4.0f, 0.25f }, (b3Quat){ { 0.0f, 0.8f, 0.0f }, 0.6f } );
		b3CreateHullShape( bodyId, &shapeDef, rock );
		bodies[bodyCount++] = bodyId;
	}

	// Cylinder hull
	b3HullData* cylinder = b3CreateCylinder( 1.0f, 0.4f, 0.0f, 12 );
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ 2.0f, 2.5f, -2.0f }, (b3Quat){ { 0.0f, 0.0f, 0.6f }, 0.8f } );
		b3CreateHullShape( bodyId, &shapeDef, cylinder );
		bodies[bodyCount++] = bodyId;
	}

	// Bullet sphere fired at the stack
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Vec3){ 0.0f, 2.5f, -6.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.0f, 40.0f };
		bodyDef.isBullet = true;
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.125f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		bodies[bodyCount++] = bodyId;
	}

	// Pile of small boxes and spheres
	b3BoxHull small = b3MakeBoxHull( 0.25f, 0.125f, 0.25f );
	for ( int i = 0; i < 4; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			b3Vec3 p = { -3.0f + 0.75f * i, 0.5f + 0.5f * j, -3.0f + 0.25f * j };
			b3BodyId bodyId = CreateDynamic( worldId, p, b3Quat_identity );
			if ( ( i + j ) & 1 )
			{
				b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.25f };
				b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			}
			else
			{
				b3CreateHullShape( bodyId, &shapeDef, &small.base );
			}
			bodies[bodyCount++] = bodyId;
		}
	}

	B3_ASSERT( bodyCount <= bodyCapacity );
	(void)bodyCapacity;

	b3World* world = b3GetWorldFromId( worldId );

	for ( int step = 1; step <= WORLD_GOLDEN_STEPS; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		if ( step % WORLD_GOLDEN_STRIDE != 0 && step != WORLD_GOLDEN_STEPS )
		{
			continue;
		}

		for ( int i = 0; i < bodyCount; ++i )
		{
			b3WorldTransform xf = b3Body_GetTransform( bodies[i] );
			b3Vec3 v = b3Body_GetLinearVelocity( bodies[i] );
			b3Vec3 w = b3Body_GetAngularVelocity( bodies[i] );
			printf( "body %d %d |", step, i );
			PrintVec( xf.p );
			PrintVec( xf.q.v );
			printf( " %08x", Bits( xf.q.s ) );
			PrintVec( v );
			PrintVec( w );
			printf( " %d\n", b3Body_IsAwake( bodies[i] ) ? 1 : 0 );
		}

		b3Counters counters = b3World_GetCounters( worldId );
		printf( "world %d | %d %d %d %016llx\n", step, b3World_GetAwakeBodyCount( worldId ), counters.contactCount,
				counters.islandCount, (unsigned long long)b3HashWorldState( world ) );
	}

	b3DestroyWorld( worldId );
	b3DestroyHull( rock );
	b3DestroyHull( cylinder );
	b3DestroyMesh( mesh );
}

int main( int argc, char** argv )
{
	// The golden is the 1 worker run; pass a worker count to check another one prints the same.
	int workerCount = argc > 1 ? atoi( argv[1] ) : 1;
	RunScene( workerCount, 64 );
	return 0;
}
