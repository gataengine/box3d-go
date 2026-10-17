// Generates testdata/recording_golden.b3rec from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/recording_golden_gen.c $(ls src/*.c | grep -v -e src/wide8.c)"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/recording_golden $SRC
// $SCRATCH/recording_golden ../../libs/box3d-go/testdata/recording_golden.b3rec
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD) and a 4 worker run (pass 4
// as the second argument). They must be identical. The validation build differs only in the
// header's validationEnabled byte.
//
// A joint-free scene recorded from the middle of its life: a grid mesh ground, a static box and a
// sphere exist before b3World_StartRecording (so the seed snapshot interns the mesh and the box
// hull), then the recording creates named bodies with sphere, capsule, hull (a duplicate cube hull
// to exercise dedup), multi material mesh, height field and compound shapes, calls world, body and
// shape mutators, steps the world, runs every spatial query kind (tagged and untagged), destroys a
// shape and a body, and stops. recording_test.go mirrors it line by line; the Go recording must
// match this file byte for byte outside the seed snapshot (header fields except snapshotSize and
// registryOffset, the op stream and the registry block).
//
// Inputs are exactly representable or computed without libm, so Go reproduces them exactly.
#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REC_GOLDEN_STEPS 90

static bool OverlapFcn( b3ShapeId shapeId, void* context )
{
	int* count = context;
	*count += 1;
	// Reject every third shape so the recorded accept stream has both values
	return ( b3StoreShapeId( shapeId ) % 3 ) != 0;
}

static float CastFcn( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId, int triangleIndex,
					  int childIndex, void* context )
{
	(void)shapeId;
	(void)point;
	(void)normal;
	(void)userMaterialId;
	(void)triangleIndex;
	(void)childIndex;
	int* count = context;
	*count += 1;
	return fraction;
}

static bool PlaneFcn( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context )
{
	(void)shapeId;
	(void)planes;
	int* count = context;
	*count += planeCount;
	return true;
}

static bool MoverFilterFcn( b3ShapeId shapeId, void* context )
{
	(void)shapeId;
	(void)context;
	return true;
}

static b3BodyId CreateDynamic( b3WorldId worldId, b3Vec3 p, const char* name )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = p;
	bodyDef.name = name;
	return b3CreateBody( worldId, &bodyDef );
}

int main( int argc, char** argv )
{
	if ( argc < 2 )
	{
		fprintf( stderr, "usage: recording_golden out.b3rec [workerCount]\n" );
		return 1;
	}

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = argc > 2 ? atoi( argv[2] ) : 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Before recording: ground mesh, static box, a resting sphere
	b3MeshData* mesh = b3CreateGridMesh( 16, 16, 1.0f, 1, true );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "ground";
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( groundId, &shapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
	}

	b3BoxHull box = b3MakeBoxHull( 2.0f, 0.25f, 1.5f );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 5.0f, 0.5f, 0.0f };
		bodyDef.rotation = (b3Quat){ { 0.0f, 0.0f, 0.6f }, 0.8f };
		b3BodyId staticId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( staticId, &shapeDef, &box.base );
	}

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;

	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -1.0f, 0.5f, -1.0f }, NULL );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Recording* recording = b3CreateRecording( 0 );
	b3World_StartRecording( worldId, recording );

	// World config
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -9.75f, 0.0f } );
	b3World_EnableSleeping( worldId, true );
	b3World_SetContactTuning( worldId, 30.0f, 10.0f, 3.0f );

	// Box stack with a duplicate cube hull
	b3BoxHull cube = b3MakeCubeHull( 0.5f );
	b3BodyId stack[3];
	for ( int i = 0; i < 3; ++i )
	{
		stack[i] = CreateDynamic( worldId, (b3Vec3){ 0.0f, 0.5f + 1.0f * i, 0.0f }, "stack" );
		b3CreateHullShape( stack[i], &shapeDef, &cube.base );
	}

	// Capsule with a name on the shape
	b3BodyId capsuleBody = CreateDynamic( worldId, (b3Vec3){ -2.0f, 3.0f, 2.0f }, "capsule" );
	b3ShapeId capsuleShape;
	{
		b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.25f };
		b3ShapeDef def = shapeDef;
		def.name = "capsule shape";
		capsuleShape = b3CreateCapsuleShape( capsuleBody, &def, &capsule );
	}

	// Rock hull
	b3HullData* rock = b3CreateRock( 0.5f );
	b3BodyId rockBody = CreateDynamic( worldId, (b3Vec3){ 5.0f, 4.0f, 0.25f }, "rock" );
	b3CreateHullShape( rockBody, &shapeDef, rock );

	// Multi material mesh on a static body
	b3MeshData* bumps = b3CreateGridMesh( 4, 4, 0.5f, 2, true );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ -6.0f, 0.25f, -6.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3SurfaceMaterial materials[2] = { b3DefaultSurfaceMaterial(), b3DefaultSurfaceMaterial() };
		materials[1].friction = 0.25f;
		materials[1].userMaterialId = 42;
		b3ShapeDef def = b3DefaultShapeDef();
		def.materials = materials;
		def.materialCount = 2;
		b3CreateMeshShape( bodyId, &def, bumps, (b3Vec3){ 1.0f, 1.0f, -1.0f } );
	}

	// Height field on a static body
	b3HeightFieldData* heightField = b3CreateGrid( 5, 5, (b3Vec3){ 1.0f, 0.5f, 1.0f }, false );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 8.0f, 0.0f, 8.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef def = b3DefaultShapeDef();
		b3CreateHeightFieldShape( bodyId, &def, heightField );
	}

	// Compound with a sphere, a capsule and a hull
	b3CompoundData* compound;
	{
		b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
		b3CompoundSphereDef sphere = { { { -1.0f, 0.0f, 0.0f }, 0.5f }, mat };
		b3CompoundCapsuleDef capsule = { { { 0.0f, 0.0f, -1.0f }, { 0.0f, 0.0f, 1.0f }, 0.25f }, mat };
		b3BoxHull small = b3MakeBoxHull( 0.25f, 0.25f, 0.25f );
		b3CompoundHullDef hull = { &small.base, { { 1.0f, 0.0f, 0.0f }, { { 0.0f, 0.0f, 0.0f }, 1.0f } }, mat };
		b3CompoundDef def = { 0 };
		def.spheres = &sphere;
		def.sphereCount = 1;
		def.capsules = &capsule;
		def.capsuleCount = 1;
		def.hulls = &hull;
		def.hullCount = 1;
		compound = b3CreateCompound( &def );
	}
	b3BodyId compoundBody = CreateDynamic( worldId, (b3Vec3){ 2.0f, 3.0f, -2.0f }, "compound" );
	b3CreateBakedCompoundShape( compoundBody, &shapeDef, compound );

	// A sphere to destroy later
	b3BodyId doomed = CreateDynamic( worldId, (b3Vec3){ 3.0f, 1.0f, 3.0f }, NULL );
	b3ShapeId doomedShape;
	{
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.25f };
		doomedShape = b3CreateSphereShape( doomed, &shapeDef, &sphere );
		b3Sphere second = { { 0.25f, 0.0f, 0.0f }, 0.25f };
		b3CreateSphereShape( doomed, &shapeDef, &second );
	}

	// Body and shape mutators
	b3Body_SetLinearVelocity( capsuleBody, (b3Vec3){ 1.0f, 0.0f, -0.5f } );
	b3Body_SetAngularVelocity( rockBody, (b3Vec3){ 0.0f, 2.0f, 0.0f } );
	b3Body_ApplyForceToCenter( stack[2], (b3Vec3){ 5.0f, 0.0f, 0.0f }, true );
	b3Body_ApplyLinearImpulse( compoundBody, (b3Vec3){ 0.0f, 1.0f, 0.0f }, (b3Vec3){ 2.0f, 3.0f, -1.75f }, true );
	b3Body_SetAngularDamping( rockBody, 0.5f );
	b3Body_SetName( stack[0], "bottom" );
	b3Shape_SetFriction( capsuleShape, 0.75f );
	b3Shape_SetName( capsuleShape, NULL );
	{
		b3Filter filter = b3DefaultFilter();
		filter.groupIndex = -2;
		b3Shape_SetFilter( doomedShape, filter, true );
	}

	b3QueryFilter tagged = b3DefaultQueryFilter();
	tagged.id = 7;
	tagged.name = "probe";
	b3QueryFilter nameOnly = b3DefaultQueryFilter();
	nameOnly.name = "closest";
	b3QueryFilter untagged = b3DefaultQueryFilter();

	b3Vec3 proxyPoints[3] = { { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, { 0.0f, 0.5f, 0.0f } };
	b3ShapeProxy proxy = { proxyPoints, 3, 0.125f };
	b3Capsule mover = { { 0.0f, 0.25f, 0.0f }, { 0.0f, 1.25f, 0.0f }, 0.25f };

	int count = 0;
	for ( int step = 1; step <= REC_GOLDEN_STEPS; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		if ( step % 15 == 0 )
		{
			b3AABB aabb = { { -3.0f, -1.0f, -3.0f }, { 3.0f, 4.0f, 3.0f } };
			b3World_OverlapAABB( worldId, aabb, tagged, OverlapFcn, &count );
			b3World_OverlapShape( worldId, (b3Vec3){ 0.0f, 0.5f, 0.0f }, &proxy, untagged, OverlapFcn, &count );
			b3World_CastRay( worldId, (b3Vec3){ -8.0f, 1.0f, 0.0f }, (b3Vec3){ 16.0f, 0.0f, 0.0f }, untagged, CastFcn, &count );
			b3World_CastRayClosest( worldId, (b3Vec3){ 0.0f, 10.0f, 0.0f }, (b3Vec3){ 0.0f, -20.0f, 0.0f }, nameOnly );
			b3World_CastShape( worldId, (b3Vec3){ 0.0f, 8.0f, 0.5f }, &proxy, (b3Vec3){ 0.0f, -10.0f, 0.0f }, tagged, CastFcn,
							   &count );
			b3World_CastMover( worldId, (b3Vec3){ -4.0f, 0.0f, 0.0f }, &mover, (b3Vec3){ 8.0f, 0.0f, 0.0f }, untagged,
							   MoverFilterFcn, NULL );
			b3World_CastMover( worldId, (b3Vec3){ -4.0f, 0.0f, 1.0f }, &mover, (b3Vec3){ 8.0f, 0.0f, 0.0f }, tagged, NULL, NULL );
			b3World_CollideMover( worldId, (b3Vec3){ 0.0f, 0.0f, 0.0f }, &mover, untagged, PlaneFcn, &count );
		}

		if ( step == 30 )
		{
			b3DestroyShape( doomedShape, true );
			b3Body_SetTransform( stack[1], (b3Vec3){ 0.25f, 2.0f, 0.0f }, (b3Quat){ { 0.0f, 0.6f, 0.0f }, 0.8f } );
		}

		if ( step == 45 )
		{
			b3DestroyBody( doomed );
			b3ExplosionDef explosion = b3DefaultExplosionDef();
			explosion.position = (b3Vec3){ 0.0f, 0.0f, 0.0f };
			explosion.radius = 4.0f;
			explosion.falloff = 1.0f;
			explosion.impulsePerArea = 2.0f;
			b3World_Explode( worldId, &explosion );
		}
	}

	b3World_StopRecording( worldId );

	if ( b3SaveRecordingToFile( recording, argv[1] ) == false )
	{
		fprintf( stderr, "save failed\n" );
		return 1;
	}

	printf( "bytes %d queries %d awake %d\n", b3Recording_GetSize( recording ), count,
			b3World_GetAwakeBodyCount( worldId ) );

	b3DestroyRecording( recording );
	b3DestroyCompound( compound );
	b3DestroyHeightField( heightField );
	b3DestroyMesh( bumps );
	b3DestroyHull( rock );
	b3DestroyMesh( mesh );
	b3DestroyWorld( worldId );
	return 0;
}
