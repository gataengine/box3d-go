// Generates testdata/replay_golden.b3rec and testdata/replay_golden.txt from the pinned upstream C
// sources (Go-only tooling). Run from third_party/box3d, writing the binary outside the repository
// ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/replay_golden_gen.c $(ls src/*.c | grep -v -e src/wide8.c)"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/replay_golden $SRC
// $SCRATCH/replay_golden ../../libs/box3d-go/testdata/replay_golden.b3rec > ../../libs/box3d-go/testdata/replay_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD); it must be identical.
//
// Records a joint-free scene with the C library: the recording starts on the empty world, then a
// grid mesh ground, a height field, a tilted static box, a box stack, a sphere, a capsule, rock and
// cylinder hulls, a static compound of spheres and a capsule, a bullet, a pile of small boxes and spheres are
// created and stepped REPLAY_GOLDEN_STEPS times at 1/60 s with 4 sub-steps. Between steps it applies
// mutators (forces, impulses, velocities, gravity, transform, type, damping, friction, filter, an
// explosion, a late body create and a body destroy) and issues every recorded spatial query kind,
// some tagged with an id and name. The recording is saved to argv[1].
//
// The output lists every live body slot of the original world after the last step (transform,
// velocities, awake) and its b3HashWorldState, then whether b3ValidateReplay accepts the recording,
// then the same lines for the world of a b3RecPlayer stepped to the end, which must equal the
// original run. Every number is a hex word, so the Go replay compares bit for bit.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "body.h"
#include "physics_world.h"
#include "recording.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REPLAY_GOLDEN_STEPS 180

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

static void PrintWorld( const char* tag, b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	int count = world->bodies.count;
	for ( int i = 0; i < count; ++i )
	{
		if ( world->bodies.data[i].id != i )
		{
			continue;
		}
		b3BodyId bodyId = b3MakeBodyId( world, i );
		b3WorldTransform xf = b3Body_GetTransform( bodyId );
		b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
		b3Vec3 w = b3Body_GetAngularVelocity( bodyId );
		printf( "%s body %d %d |", tag, i, bodyId.generation );
		PrintVec( xf.p );
		PrintVec( xf.q.v );
		printf( " %08x", Bits( xf.q.s ) );
		PrintVec( v );
		PrintVec( w );
		printf( " %d %d\n", b3Body_IsAwake( bodyId ) ? 1 : 0, (int)b3Body_GetType( bodyId ) );
	}
	printf( "%s world | %d %016llx\n", tag, b3World_GetAwakeBodyCount( worldId ), (unsigned long long)b3HashWorldState( world ) );
}

static b3BodyId CreateDynamic( b3WorldId worldId, b3Vec3 p, b3Quat q )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = p;
	bodyDef.rotation = q;
	return b3CreateBody( worldId, &bodyDef );
}

// Query callbacks. Their return values are part of the recording.
static bool OverlapAll( b3ShapeId shapeId, void* context )
{
	int* count = context;
	*count += 1;
	return true;
}

static bool OverlapFirstThree( b3ShapeId shapeId, void* context )
{
	int* count = context;
	*count += 1;
	return *count < 3;
}

static float CastClip( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId, int triangleIndex,
					   int childIndex, void* context )
{
	int* count = context;
	*count += 1;
	return fraction;
}

static float CastAll( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId, int triangleIndex,
					  int childIndex, void* context )
{
	int* count = context;
	*count += 1;
	return 1.0f;
}

static bool MoverFilter( b3ShapeId shapeId, void* context )
{
	int* count = context;
	*count += 1;
	return ( shapeId.index1 & 1 ) == 0 || *count > 2;
}

static bool CollectPlanes( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context )
{
	int* count = context;
	*count += planeCount;
	return true;
}

static void IssueQueries( b3WorldId worldId, int step )
{
	int count = 0;
	b3QueryFilter filter = b3DefaultQueryFilter();

	b3AABB box = { { -4.0f, -1.0f, -4.0f }, { 4.0f, 3.0f, 4.0f } };
	b3World_OverlapAABB( worldId, box, filter, OverlapAll, &count );

	b3QueryFilter tagged = b3DefaultQueryFilter();
	tagged.id = 42;
	tagged.name = "probe";
	b3World_OverlapAABB( worldId, box, tagged, OverlapFirstThree, &count );

	b3Vec3 points[2] = { { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.25f, 0.0f } };
	b3ShapeProxy proxy = { points, 2, 0.25f };
	b3World_OverlapShape( worldId, (b3Vec3){ -1.0f, 1.0f, 0.0f }, &proxy, filter, OverlapAll, &count );

	b3QueryFilter rayTag = b3DefaultQueryFilter();
	rayTag.id = (uint64_t)step;
	b3World_CastRay( worldId, (b3Vec3){ -6.0f, 0.75f, 0.1f }, (b3Vec3){ 12.0f, 0.0f, 0.0f }, rayTag, CastClip, &count );
	b3World_CastRay( worldId, (b3Vec3){ 0.25f, 8.0f, 0.25f }, (b3Vec3){ 0.0f, -10.0f, 0.0f }, filter, CastAll, &count );

	b3RayResult closest =
		b3World_CastRayClosest( worldId, (b3Vec3){ -3.0f, 6.0f, -3.0f }, (b3Vec3){ 1.0f, -9.0f, 0.5f }, filter );
	(void)closest;

	b3QueryFilter castTag = b3DefaultQueryFilter();
	castTag.name = "sweep";
	b3ShapeProxy sphereProxy = { points, 1, 0.3f };
	b3World_CastShape( worldId, (b3Vec3){ -5.0f, 1.0f, -1.0f }, &sphereProxy, (b3Vec3){ 10.0f, 0.0f, 0.5f }, castTag, CastClip,
					   &count );

	b3Capsule mover = { { 0.0f, 0.3f, 0.0f }, { 0.0f, 1.2f, 0.0f }, 0.3f };
	float fraction =
		b3World_CastMover( worldId, (b3Vec3){ -2.0f, 0.1f, 1.0f }, &mover, (b3Vec3){ 4.0f, -0.5f, -1.0f }, filter, MoverFilter, &count );
	(void)fraction;

	b3World_CollideMover( worldId, (b3Vec3){ 0.0f, 0.05f, 0.0f }, &mover, filter, CollectPlanes, &count );
}

int main( int argc, char** argv )
{
	if ( argc < 2 )
	{
		fprintf( stderr, "usage: replay_golden <out.b3rec>\n" );
		return 1;
	}

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Start on the empty world, so the seed snapshot carries no struct images.
	b3Recording* recording = b3CreateRecording( 0 );
	b3World_StartRecording( worldId, recording );

	b3BodyId bodies[64];
	int bodyCount = 0;

	// Ground: grid mesh
	b3MeshData* mesh = b3CreateGridMesh( 16, 16, 1.0f, 1, true );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "ground";
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( groundId, &shapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
		bodies[bodyCount++] = groundId;
	}

	// Height field off to the side
	b3HeightFieldData* heightField = b3CreateGrid( 8, 8, (b3Vec3){ 1.0f, 0.5f, 1.0f }, false );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 10.0f, -0.25f, -4.0f };
		b3BodyId hfId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.4f;
		b3CreateHeightFieldShape( hfId, &shapeDef, heightField );
		bodies[bodyCount++] = hfId;
	}

	// Tilted static box (3-4-5 about z)
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

	// Sphere rolling in from the side with spin, with per-shape events and a name
	b3BodyId sphereId;
	{
		sphereId = CreateDynamic( worldId, (b3Vec3){ -4.0f, 2.0f, 0.5f }, b3Quat_identity );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3ShapeDef sphereDef = shapeDef;
		sphereDef.name = "ball";
		sphereDef.baseMaterial.friction = 0.6f;
		sphereDef.baseMaterial.rollingResistance = 0.05f;
		sphereDef.enableHitEvents = true;
		b3CreateSphereShape( sphereId, &sphereDef, &sphere );
		b3Body_SetLinearVelocity( sphereId, (b3Vec3){ 3.0f, 0.0f, -0.5f } );
		b3Body_SetAngularVelocity( sphereId, (b3Vec3){ 0.0f, 2.0f, -6.0f } );
		bodies[bodyCount++] = sphereId;
	}

	// Capsule falling tilted onto the mesh
	b3ShapeId capsuleShapeId;
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -2.0f, 3.0f, 2.0f }, (b3Quat){ { 0.6f, 0.0f, 0.0f }, 0.8f } );
		b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.25f };
		capsuleShapeId = b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		bodies[bodyCount++] = bodyId;
	}

	// Rock hull falling onto the tilted static box
	b3HullData* rock = b3CreateRock( 0.6f );
	b3BodyId rockId;
	{
		rockId = CreateDynamic( worldId, (b3Vec3){ 5.0f, 4.0f, 0.25f }, (b3Quat){ { 0.0f, 0.8f, 0.0f }, 0.6f } );
		b3CreateHullShape( rockId, &shapeDef, rock );
		bodies[bodyCount++] = rockId;
	}

	// Cylinder hull
	b3HullData* cylinder = b3CreateCylinder( 1.0f, 0.4f, 0.0f, 12 );
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ 2.0f, 2.5f, -2.0f }, (b3Quat){ { 0.0f, 0.0f, 0.6f }, 0.8f } );
		b3CreateHullShape( bodyId, &shapeDef, cylinder );
		bodies[bodyCount++] = bodyId;
	}

	// Static compound of two spheres and a capsule (a dumbbell)
	b3CompoundData* compound;
	{
		b3CompoundSphereDef spheres[2] = {
			{ { { -0.75f, 0.0f, 0.0f }, 0.3f }, b3DefaultSurfaceMaterial() },
			{ { { 0.75f, 0.0f, 0.0f }, 0.3f }, b3DefaultSurfaceMaterial() },
		};
		b3CompoundCapsuleDef capsules[1] = {
			{ { { -0.75f, 0.0f, 0.0f }, { 0.75f, 0.0f, 0.0f }, 0.1f }, b3DefaultSurfaceMaterial() },
		};
		b3CompoundDef compoundDef = { 0 };
		compoundDef.spheres = spheres;
		compoundDef.sphereCount = 2;
		compoundDef.capsules = capsules;
		compoundDef.capsuleCount = 1;
		compound = b3CreateCompound( &compoundDef );

		// Compounds go on static bodies; the falling capsule lands on this one.
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ -2.0f, 0.4f, 2.0f };
		bodyDef.rotation = (b3Quat){ { 0.0f, 0.6f, 0.0f }, 0.8f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateBakedCompoundShape( bodyId, &shapeDef, compound );
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
	b3BodyId pile[12];
	int pileCount = 0;
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
			pile[pileCount++] = bodyId;
			bodies[bodyCount++] = bodyId;
		}
	}

	for ( int step = 1; step <= REPLAY_GOLDEN_STEPS; ++step )
	{
		if ( step % 10 == 0 )
		{
			IssueQueries( worldId, step );
		}

		if ( step <= 20 )
		{
			b3Body_ApplyForce( rockId, (b3Vec3){ 0.0f, 0.0f, -4.0f }, (b3Vec3){ 5.0f, 4.0f, 0.5f }, true );
			b3Body_ApplyTorque( pile[0], (b3Vec3){ 0.0f, 0.5f, 0.0f }, true );
		}

		switch ( step )
		{
			case 30:
				b3Body_ApplyLinearImpulseToCenter( bodies[3], (b3Vec3){ 2.0f, 0.0f, 0.0f }, true );
				b3Body_ApplyAngularImpulse( pile[5], (b3Vec3){ 0.0f, 0.25f, 0.0f }, true );
				break;
			case 45:
				b3Body_SetLinearVelocity( sphereId, (b3Vec3){ -2.0f, 1.0f, 0.0f } );
				b3Body_SetAngularDamping( sphereId, 0.25f );
				break;
			case 60:
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.name = "late";
				bodyDef.position = (b3Vec3){ 1.0f, 6.0f, 1.0f };
				bodyDef.angularVelocity = (b3Vec3){ 1.0f, 0.0f, 0.5f };
				b3BodyId lateId = b3CreateBody( worldId, &bodyDef );
				b3Capsule capsule = { { 0.0f, -0.25f, 0.0f }, { 0.0f, 0.25f, 0.0f }, 0.2f };
				b3CreateCapsuleShape( lateId, &shapeDef, &capsule );
				b3CreateHullShape( lateId, &shapeDef, &small.base );
				b3Body_ApplyMassFromShapes( lateId );
				break;
			}
			case 75:
				b3World_SetGravity( worldId, (b3Vec3){ 0.5f, -7.0f, 0.0f } );
				break;
			case 90:
				b3DestroyBody( pile[7] );
				break;
			case 100:
				b3Shape_SetFriction( capsuleShapeId, 0.1f );
				b3Shape_SetRestitution( capsuleShapeId, 0.5f );
				b3Body_SetGravityScale( pile[2], 0.5f );
				break;
			case 110:
				b3Body_SetTransform( pile[3], (b3Vec3){ -1.0f, 3.0f, -2.0f }, (b3Quat){ { 0.0f, 0.6f, 0.0f }, 0.8f } );
				b3Body_SetLinearDamping( pile[4], 0.5f );
				break;
			case 120:
			{
				b3ExplosionDef def = b3DefaultExplosionDef();
				def.position = (b3Vec3){ -2.0f, 0.0f, -2.0f };
				def.radius = 3.0f;
				def.falloff = 1.0f;
				def.impulsePerArea = 2.0f;
				b3World_Explode( worldId, &def );
				break;
			}
			case 130:
				b3Body_SetType( pile[6], b3_kinematicBody );
				b3Body_SetLinearVelocity( pile[6], (b3Vec3){ 0.0f, 0.5f, 0.0f } );
				break;
			case 140:
			{
				b3Filter filter = b3DefaultFilter();
				filter.groupIndex = -1;
				b3Shape_SetFilter( capsuleShapeId, filter, true );
				b3Body_SetAwake( pile[8], false );
				break;
			}
			case 150:
				b3World_SetContactTuning( worldId, 25.0f, 5.0f, 2.0f );
				b3Body_ApplyLinearImpulse( sphereId, (b3Vec3){ 0.0f, 3.0f, 0.0f }, (b3Vec3){ 0.0f, 0.0f, 0.0f }, true );
				break;
			default:
				break;
		}

		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );

	PrintWorld( "original", worldId );

	if ( b3SaveRecordingToFile( recording, argv[1] ) == false )
	{
		fprintf( stderr, "failed to save %s\n", argv[1] );
		return 1;
	}

	const uint8_t* data = b3Recording_GetData( recording );
	int size = b3Recording_GetSize( recording );
	printf( "validate %d\n", b3ValidateReplay( data, size, 1 ) ? 1 : 0 );

	b3RecPlayer* player = b3CreatePlayer( data, size, 1 );
	while ( b3RecPlayer_StepFrame( player ) )
	{
	}
	printf( "player frames %d diverged %d\n", b3RecPlayer_GetFrame( player ), b3RecPlayer_HasDiverged( player ) ? 1 : 0 );
	PrintWorld( "replay", b3RecPlayer_GetWorldId( player ) );
	b3DestroyPlayer( player );

	b3DestroyWorld( worldId );
	b3DestroyRecording( recording );
	b3DestroyHull( rock );
	b3DestroyHull( cylinder );
	b3DestroyMesh( mesh );
	b3DestroyHeightField( heightField );
	b3DestroyCompound( compound );
	return 0;
}
