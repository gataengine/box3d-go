// Generates testdata/shape_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH). shape.c reaches
// into the world, so the whole library is linked; only world-free shape.c functions are called, on
// b3Shape values built by hand:
//
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/shape_golden \
//     ../../libs/box3d-go/testdata/shape_golden_gen.c src/*.c
// $SCRATCH/shape_golden > ../../libs/box3d-go/testdata/shape_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD) and, on a Mac, the SSE2
// build (-arch x86_64, runs under Rosetta). Both must be identical.
//
// Each line: a label, "|", outputs as hex words. Inputs come from an xorshift generator with no
// libm calls so Go reproduces them exactly.
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "core.h"
#include "shape.h"

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
static b3Transform rxf( float spread )
{
	b3Transform xf;
	xf.p = rv3( spread, spread, spread, 0.0f, 0.0f, 0.0f );
	xf.q = rq();
	return xf;
}
static b3SurfaceMaterial rmaterial( void )
{
	b3SurfaceMaterial m = b3DefaultSurfaceMaterial();
	m.friction = 0.1f * (float)rndInt( 3 );
	m.userMaterialId = rndInt( 4 );
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
static void ptoi( b3TOIOutput o )
{
	printf( " %d", (int)o.state );
	pv( o.point );
	pv( o.normal );
	pf( o.fraction );
	pf( o.distance );
	printf( " %d %d %d %d", o.distanceIterations, o.pushBackIterations, o.rootIterations, o.usedFallback );
}

static b3Vec3 s_proxyPoints[4];

// A random proxy: 1 to 4 points around center with a radius.
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

static b3Sweep rsweep( b3Vec3 c1, b3Vec3 c2, bool still )
{
	b3Sweep sweep;
	sweep.localCenter = still ? b3Vec3_zero : rv3( 0.1f, 0.1f, 0.1f, 0.0f, 0.0f, 0.0f );
	sweep.c1 = c1;
	sweep.c2 = c2;
	sweep.q1 = rq();
	sweep.q2 = still ? sweep.q1 : rq();
	return sweep;
}

#define HF_COLUMNS 9
#define HF_ROWS 7
static float s_hfHeights[HF_COLUMNS * HF_ROWS];
static uint8_t s_hfMaterials[( HF_COLUMNS - 1 ) * ( HF_ROWS - 1 )];

enum
{
	e_sphere,
	e_capsule,
	e_box,
	e_cylinder,
	e_gridMesh,
	e_mirroredMesh,
	e_waveMesh,
	e_heightField,
	e_compound,
	e_shapeCount
};

static b3Shape s_shapes[e_shapeCount];
static b3SurfaceMaterial s_materials[e_shapeCount][64];
static b3BoxHull s_box;

static void initShape( int index, b3ShapeType type, int materialCount )
{
	b3Shape* s = s_shapes + index;
	s->id = index;
	s->type = type;
	s->density = 0.5f + 0.25f * (float)rndInt( 4 );
	s->sensorIndex = B3_NULL_INDEX;
	s->proxyKey = B3_NULL_INDEX;
	if ( materialCount == 1 )
	{
		s->material = rmaterial();
		s->materialCount = 1;
		s->materials = NULL;
	}
	else
	{
		for ( int j = 0; j < materialCount; ++j )
		{
			s_materials[index][j] = rmaterial();
		}
		s->materials = s_materials[index];
		s->materialCount = materialCount;
	}
}

static void makeShapes( void )
{
	initShape( e_sphere, b3_sphereShape, 1 );
	s_shapes[e_sphere].sphere.center = rv3( 0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f );
	s_shapes[e_sphere].sphere.radius = 0.5f + 0.25f * rnd();

	initShape( e_capsule, b3_capsuleShape, 1 );
	s_shapes[e_capsule].capsule.center1 = rv3( 0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f );
	s_shapes[e_capsule].capsule.center2 = b3Add( s_shapes[e_capsule].capsule.center1, rv3( 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f ) );
	s_shapes[e_capsule].capsule.radius = 0.3f + 0.2f * rnd();

	initShape( e_box, b3_hullShape, 1 );
	s_box = b3MakeBoxHull( 0.5f, 0.75f, 1.0f );
	s_shapes[e_box].hull = &s_box.base;

	initShape( e_cylinder, b3_hullShape, 1 );
	s_shapes[e_cylinder].hull = b3CreateCylinder( 1.0f, 0.5f, 0.0f, 10 );

	b3MeshData* gridMesh = b3CreateGridMesh( 4, 4, 0.5f, 3, true );

	initShape( e_gridMesh, b3_meshShape, 3 );
	s_shapes[e_gridMesh].mesh.data = gridMesh;
	s_shapes[e_gridMesh].mesh.scale = b3Vec3_one;

	// Fewer shape materials than mesh material indices exercises the clamps.
	initShape( e_mirroredMesh, b3_meshShape, 2 );
	s_shapes[e_mirroredMesh].mesh.data = gridMesh;
	s_shapes[e_mirroredMesh].mesh.scale = (b3Vec3){ -1.2f, 0.8f, 1.1f };

	initShape( e_waveMesh, b3_meshShape, 1 );
	s_shapes[e_waveMesh].mesh.data = b3CreateWaveMesh( 6, 6, 0.5f, 0.3f, 0.2f, 0.3f );
	s_shapes[e_waveMesh].mesh.scale = b3Vec3_one;

	for ( int i = 0; i < HF_COLUMNS * HF_ROWS; ++i )
	{
		s_hfHeights[i] = 1.2f * rnd();
	}
	for ( int i = 0; i < ( HF_COLUMNS - 1 ) * ( HF_ROWS - 1 ); ++i )
	{
		uint32_t r = rndInt( 8 );
		s_hfMaterials[i] = r == 0 ? B3_HEIGHT_FIELD_HOLE : (uint8_t)r;
	}
	b3HeightFieldDef hfDef = { 0 };
	hfDef.heights = s_hfHeights;
	hfDef.materialIndices = s_hfMaterials;
	hfDef.scale = (b3Vec3){ 0.75f, 2.0f, 1.25f };
	hfDef.countX = HF_COLUMNS;
	hfDef.countZ = HF_ROWS;
	hfDef.globalMinimumHeight = -1.0f;
	hfDef.globalMaximumHeight = 1.0f;
	initShape( e_heightField, b3_heightShape, 6 );
	s_shapes[e_heightField].heightField = b3CreateHeightField( &hfDef );

	b3CompoundCapsuleDef capsules[2];
	b3CompoundHullDef hulls[2];
	b3CompoundMeshDef meshDefs[1];
	b3SurfaceMaterial meshMaterials[3];
	b3CompoundSphereDef spheres[2];
	for ( int i = 0; i < 2; ++i )
	{
		capsules[i].capsule.center1 = rv3( 2.0f, 2.0f, 2.0f, 0.0f, 0.0f, 0.0f );
		capsules[i].capsule.center2 = b3Add( capsules[i].capsule.center1, rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f ) );
		capsules[i].capsule.radius = 0.3f + 0.2f * rnd();
		capsules[i].material = rmaterial();
	}
	hulls[0].hull = &s_box.base;
	hulls[1].hull = s_shapes[e_cylinder].hull;
	for ( int i = 0; i < 2; ++i )
	{
		hulls[i].transform = rxf( 2.0f );
		hulls[i].material = rmaterial();
	}
	meshDefs[0].meshData = gridMesh;
	meshDefs[0].transform = rxf( 2.0f );
	meshDefs[0].scale = rv3( 0.3f, 0.3f, 0.3f, 1.0f, 1.0f, 1.0f );
	for ( int j = 0; j < 3; ++j )
	{
		meshMaterials[j] = rmaterial();
	}
	meshDefs[0].materials = meshMaterials;
	meshDefs[0].materialCount = 3;
	for ( int i = 0; i < 2; ++i )
	{
		spheres[i].sphere.center = rv3( 2.0f, 2.0f, 2.0f, 0.0f, 0.0f, 0.0f );
		spheres[i].sphere.radius = 0.4f + 0.3f * rnd();
		spheres[i].material = rmaterial();
	}
	b3CompoundDef compoundDef = { 0 };
	compoundDef.capsules = capsules;
	compoundDef.capsuleCount = 2;
	compoundDef.hulls = hulls;
	compoundDef.hullCount = 2;
	compoundDef.meshes = meshDefs;
	compoundDef.meshCount = 1;
	compoundDef.spheres = spheres;
	compoundDef.sphereCount = 2;
	b3CompoundData* compound = b3CreateCompound( &compoundDef );

	// As b3CreateShapeInternal: the shape owns a copy of the compound materials.
	b3Shape* s = s_shapes + e_compound;
	s->id = e_compound;
	s->type = b3_compoundShape;
	s->density = 1.0f;
	s->sensorIndex = B3_NULL_INDEX;
	s->proxyKey = B3_NULL_INDEX;
	s->compound = compound;
	s->materialCount = compound->materialCount;
	memcpy( s_materials[e_compound], b3GetCompoundMaterials( compound ), compound->materialCount * sizeof( b3SurfaceMaterial ) );
	s->materials = s_materials[e_compound];
}

static int triangleCount( const b3Shape* shape )
{
	switch ( shape->type )
	{
		case b3_meshShape:
			return shape->mesh.data->triangleCount;
		case b3_heightShape:
			return b3GetHeightFieldTriangleCount( shape->heightField );
		default:
			return 1;
	}
}

static void shapeSection( int index )
{
	b3Shape* shape = s_shapes + index;
	bool convex = shape->type == b3_sphereShape || shape->type == b3_capsuleShape || shape->type == b3_hullShape;

	printf( "shape %d props |", index );
	{
		b3MassData md = b3ComputeShapeMass( shape );
		pf( md.mass );
		pv( md.center );
		pv( md.inertia.cx );
		pv( md.inertia.cy );
		pv( md.inertia.cz );
		pv( b3GetShapeCentroid( shape ) );
		pf( b3GetShapeArea( shape ) );
		for ( int k = 0; k < 3; ++k )
		{
			b3Vec3 n = b3Normalize( rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f ) );
			pf( b3GetShapeProjectedArea( shape, n ) );
		}
		for ( int k = 0; k < 2; ++k )
		{
			b3Vec3 c = rv3( 0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f );
			b3ShapeExtent e = b3ComputeShapeExtent( shape, c );
			pf( e.minExtent );
			pv( e.maxExtent );
			pf( b3ComputeShapeMinExtent( shape, c ) );
		}
	}
	printf( "\n" );

	printf( "shape %d aabb |", index );
	for ( int k = 0; k < 2; ++k )
	{
		b3Transform xf = rxf( 2.0f );
		pbox( b3ComputeShapeAABB( shape, xf ) );
		b3WorldTransform wxf;
		wxf.p = rv3( 3.0f, 3.0f, 3.0f, 0.0f, 0.0f, 0.0f );
		wxf.q = rq();
		float extra = 0.1f + 0.05f * rnd();
		pbox( b3ComputeFatShapeAABB( shape, wxf, extra ) );
		if ( convex )
		{
			b3Vec3 c1 = rv3( 2.0f, 2.0f, 2.0f, 0.0f, 0.0f, 0.0f );
			b3Vec3 c2 = rv3( 2.0f, 2.0f, 2.0f, 0.0f, 0.0f, 0.0f );
			b3Sweep sweep = rsweep( c1, c2, false );
			float time = 0.5f + 0.5f * rnd();
			pbox( b3ComputeSweptShapeAABB( shape, &sweep, time ) );
		}
	}
	printf( "\n" );

	if ( convex )
	{
		printf( "shape %d proxy |", index );
		b3ShapeProxy proxy = b3MakeShapeProxy( shape );
		printf( " %d", proxy.count );
		pf( proxy.radius );
		for ( int i = 0; i < proxy.count; ++i )
		{
			pv( proxy.points[i] );
		}
		printf( "\n" );
	}

	b3Vec3 centroid = b3GetShapeCentroid( shape );

	printf( "shape %d ray |", index );
	for ( int k = 0; k < 6; ++k )
	{
		b3Transform xf = rxf( 2.0f );
		b3Vec3 target = b3Add( b3TransformPoint( xf, centroid ), rv3( 0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f ) );
		b3RayCastInput input;
		input.origin = b3Add( target, rv3( 4.0f, 4.0f, 4.0f, 0.0f, 0.0f, 0.0f ) );
		input.translation = b3MulSV( 2.0f, b3Sub( target, input.origin ) );
		input.maxFraction = 1.0f;
		pco( b3RayCastShape( shape, xf, &input ) );
	}
	printf( "\n" );

	printf( "shape %d cast |", index );
	for ( int k = 0; k < 6; ++k )
	{
		b3Transform xf = rxf( 2.0f );
		b3Vec3 target = b3Add( b3TransformPoint( xf, centroid ), rv3( 0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f ) );
		b3Vec3 origin = b3Add( target, rv3( 4.0f, 4.0f, 4.0f, 0.0f, 0.0f, 0.0f ) );
		b3ShapeCastInput input;
		input.proxy = rproxy( origin, 0.3f );
		input.translation = b3MulSV( 2.0f, b3Sub( target, origin ) );
		input.maxFraction = 1.0f;
		input.canEncroach = rndInt( 2 ) == 1;
		pco( b3ShapeCastShape( shape, xf, &input ) );
	}
	printf( "\n" );

	printf( "shape %d overlap |", index );
	for ( int k = 0; k < 8; ++k )
	{
		b3Transform xf = rxf( 2.0f );
		b3Vec3 center = b3Add( b3TransformPoint( xf, centroid ), rv3( 1.5f, 1.5f, 1.5f, 0.0f, 0.0f, 0.0f ) );
		b3ShapeProxy proxy = rproxy( center, 0.4f );
		printf( " %d", b3OverlapShape( shape, xf, &proxy ) );
	}
	printf( "\n" );

	printf( "shape %d mover |", index );
	for ( int k = 0; k < 4; ++k )
	{
		b3Transform xf = rxf( 2.0f );
		b3Capsule mover;
		mover.center1 = b3Add( b3TransformPoint( xf, centroid ), rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f ) );
		mover.center2 = b3Add( mover.center1, rv3( 0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f ) );
		mover.radius = 0.3f + 0.2f * rnd();
		b3PlaneResult planes[8] = { 0 };
		int count = b3CollideMover( planes, 8, shape, xf, &mover );
		pplanes( planes, count );
		printf( " %d", b3CollideMover( planes, 0, shape, xf, &mover ) );
	}
	printf( "\n" );

	printf( "shape %d material |", index );
	if ( shape->type == b3_compoundShape )
	{
		int childCount = shape->compound->capsuleCount + shape->compound->hullCount + shape->compound->meshCount +
						 shape->compound->sphereCount;
		for ( int c = 0; c < childCount; ++c )
		{
			b3ChildShape child = b3GetCompoundChild( shape->compound, c );
			int count = child.type == b3_meshShape ? child.mesh.data->triangleCount : 1;
			for ( int t = 0; t < count; t += 1 + count / 5 )
			{
				printf( " %llu", (unsigned long long)b3GetShapeUserMaterialId( shape, c, t ) );
			}
		}
	}
	else
	{
		int count = triangleCount( shape );
		for ( int t = 0; t < count; t += 1 + count / 12 )
		{
			printf( " %llu", (unsigned long long)b3GetShapeUserMaterialId( shape, 0, t ) );
		}
	}
	printf( "\n" );

	printf( "shape %d toi |", index );
	for ( int b = e_sphere; b <= e_cylinder; ++b )
	{
		for ( int k = 0; k < 3; ++k )
		{
			b3Vec3 a1 = rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f );
			b3Vec3 a2 = convex ? b3Add( a1, rv3( 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f ) ) : a1;
			b3Sweep sweepA = rsweep( a1, a2, convex == false );

			b3Vec3 target = b3Add( a1, rv3( 0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f ) );
			b3Vec3 b1 = b3Add( target, rv3( 3.0f, 3.0f, 3.0f, 0.0f, 0.0f, 0.0f ) );
			b3Vec3 b2 = b3Sub( target, b3MulSV( 0.5f, b3Sub( b1, target ) ) );
			b3Sweep sweepB = rsweep( b1, b2, false );

			float maxFraction = 0.5f + 0.5f * rnd();
			shape->sensorIndex = k == 2 ? 0 : B3_NULL_INDEX;
			ptoi( b3ShapeTimeOfImpact( shape, s_shapes + b, &sweepA, &sweepB, maxFraction ) );
			shape->sensorIndex = B3_NULL_INDEX;
		}
	}
	printf( "\n" );
}

int main( void )
{
	makeShapes();

	for ( int i = 0; i < e_shapeCount; ++i )
	{
		shapeSection( i );
	}

	return 0;
}
