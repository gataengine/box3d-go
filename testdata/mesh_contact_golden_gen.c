// Generates testdata/mesh_contact_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// printf '#define B3_SIMD_WIDTH 4\n#include "convex_manifold_wide.inl"\n' > $SCRATCH/wide_cm.c
// SRC="../../libs/box3d-go/testdata/mesh_contact_golden_gen.c src/triangle_manifold.c src/manifold.c
//      src/convex_manifold.c $SCRATCH/wide_cm.c src/mesh.c src/height_field.c src/hull.c src/distance.c src/sphere.c
//      src/capsule.c src/aabb.c src/dynamic_tree.c src/block_allocator.c src/arena_allocator.c
//      src/math_functions.c src/core.c src/timer.c src/simd.c src/types.c"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/mesh_contact_golden $SRC
// $SCRATCH/mesh_contact_golden > ../../libs/box3d-go/testdata/mesh_contact_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD), the validation build
// (-DBOX3D_VALIDATE) and, on a Mac, the SSE2 build (-arch x86_64). They must be identical.
//
// mesh_contact.c is included below (not linked) so the static helpers (b3CullPoints,
// b3ReduceCluster) can be called directly. b3MakeLocalProxy and b3ComputeProxyAABB (shape.c, which
// drags in the whole world) are verbatim copies from the pinned commit. A b3World is assembled by hand with only the fields
// b3ComputeMeshManifolds reads: one worker task context, the manifold block allocators and their
// mutex, and the friction / restitution callbacks. Shapes are b3Shape structs filled by hand.
//
// Each scenario drives one persistent b3Contact through frames: shape B moves and turns over a
// mesh or height field, b3ComputeMeshManifolds runs, the manifolds, the mixed material values and
// the triangle cache (SAT and simplex caches) are printed, then the impulses are overwritten with
// known values so the next frame shows which ones the old-manifold matching preserved. Each line:
// a label, "|", outputs as hex words. Inputs come from an xorshift generator with no libm calls.
#include "mesh_contact.c"

#include "block_allocator.h"

#include "box3d/collision.h"

#include <stdio.h>
#include <string.h>

// ---- verbatim copies (shape.c) ----

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

// ---- generator ----

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

static void silentLog( const char* message )
{
	(void)message;
}

static float mixFriction( float frictionA, uint64_t userMaterialIdA, float frictionB, uint64_t userMaterialIdB )
{
	float f = frictionA < frictionB ? frictionA : frictionB;
	return f + (float)( userMaterialIdA + 2 * userMaterialIdB );
}

static float mixRestitution( float restitutionA, uint64_t userMaterialIdA, float restitutionB, uint64_t userMaterialIdB )
{
	(void)userMaterialIdA;
	(void)userMaterialIdB;
	return restitutionA > restitutionB ? restitutionA : restitutionB;
}

#define MATERIAL_COUNT 5
static b3SurfaceMaterial s_materials[MATERIAL_COUNT];
static const uint16_t s_materialMap[8] = { 4, 3, 2, 1, 0, 2, 4, 1 };

static void setupMaterials( void )
{
	const float friction[MATERIAL_COUNT] = { 0.2f, 0.5f, 0.8f, 0.3f, 0.6f };
	const float restitution[MATERIAL_COUNT] = { 0.0f, 0.1f, 0.2f, 0.3f, 0.4f };
	const float rolling[MATERIAL_COUNT] = { 0.01f, 0.02f, 0.0f, 0.05f, 0.03f };
	for ( int i = 0; i < MATERIAL_COUNT; ++i )
	{
		s_materials[i] = (b3SurfaceMaterial){ 0 };
		s_materials[i].friction = friction[i];
		s_materials[i].restitution = restitution[i];
		s_materials[i].rollingResistance = rolling[i];
		s_materials[i].tangentVelocity = (b3Vec3){ 0.25f * (float)i, 0.0f, -0.5f };
		s_materials[i].userMaterialId = (uint64_t)( 10 + i );
	}
}

static b3SurfaceMaterial materialB( void )
{
	b3SurfaceMaterial m = { 0 };
	m.friction = 0.4f;
	m.restitution = 0.05f;
	m.rollingResistance = 0.02f;
	m.tangentVelocity = (b3Vec3){ 0.1f, 0.0f, 0.2f };
	m.userMaterialId = 7;
	return m;
}

static b3World s_world;

static void setupWorld( void )
{
	memset( &s_world, 0, sizeof( s_world ) );
	b3TaskContext context = { 0 };
	b3Array_Push( s_world.taskContexts, context );
	s_world.manifoldAllocatorMutex = b3CreateMutex();
	s_world.frictionCallback = mixFriction;
	s_world.restitutionCallback = mixRestitution;
}

// Indexed grid mesh with welded vertices and identified edges (flat and concave flags).
#define GRID_MAX 40
static b3Vec3 s_gridVertices[( GRID_MAX + 1 ) * ( GRID_MAX + 1 )];
static int s_gridIndices[GRID_MAX * GRID_MAX * 6];
static uint8_t s_gridMaterials[GRID_MAX * GRID_MAX * 2];

static b3MeshData* makeGrid( int n, float cell, float amplitude, int flatEvery )
{
	int v = 0;
	for ( int ix = 0; ix <= n; ++ix )
	{
		for ( int iz = 0; iz <= n; ++iz )
		{
			float h = amplitude * rnd();
			if ( flatEvery > 0 && ( ix % flatEvery ) == 0 )
			{
				h = 0.0f;
			}
			b3Vec3 p;
			p.x = cell * (float)ix;
			p.y = h;
			p.z = cell * (float)iz;
			s_gridVertices[v++] = p;
		}
	}

	int k = 0;
	int t = 0;
	for ( int ix = 0; ix < n; ++ix )
	{
		for ( int iz = 0; iz < n; ++iz )
		{
			int i11 = ix * ( n + 1 ) + iz;
			int i12 = i11 + 1;
			int i21 = i11 + ( n + 1 );
			int i22 = i21 + 1;
			s_gridIndices[k++] = i11;
			s_gridIndices[k++] = i12;
			s_gridIndices[k++] = i22;
			s_gridIndices[k++] = i22;
			s_gridIndices[k++] = i21;
			s_gridIndices[k++] = i11;
			s_gridMaterials[t++] = (uint8_t)rndInt( 8 );
			s_gridMaterials[t++] = (uint8_t)rndInt( 8 );
		}
	}

	b3MeshDef def = { 0 };
	def.vertices = s_gridVertices;
	def.indices = s_gridIndices;
	def.materialIndices = s_gridMaterials;
	def.vertexCount = v;
	def.triangleCount = t;
	def.weldVertices = true;
	def.weldTolerance = 0.001f;
	def.identifyEdges = true;
	return b3CreateMesh( &def, NULL, 0 );
}

#define HF_COLUMNS 9
#define HF_ROWS 7
static float s_hfHeights[HF_COLUMNS * HF_ROWS];
static uint8_t s_hfMaterials[( HF_COLUMNS - 1 ) * ( HF_ROWS - 1 )];

static b3HeightFieldData* makeField( bool clockwise )
{
	for ( int i = 0; i < HF_COLUMNS * HF_ROWS; ++i )
	{
		s_hfHeights[i] = 0.3f * rnd();
	}
	for ( int i = 0; i < ( HF_COLUMNS - 1 ) * ( HF_ROWS - 1 ); ++i )
	{
		uint32_t r = rndInt( 10 );
		s_hfMaterials[i] = r == 0 ? B3_HEIGHT_FIELD_HOLE : (uint8_t)r;
	}

	b3HeightFieldDef def = { 0 };
	def.heights = s_hfHeights;
	def.materialIndices = s_hfMaterials;
	def.scale = (b3Vec3){ 0.75f, 2.0f, 1.25f };
	def.countX = HF_COLUMNS;
	def.countZ = HF_ROWS;
	def.globalMinimumHeight = -1.0f;
	def.globalMaximumHeight = 1.0f;
	def.clockwiseWinding = clockwise;
	return b3CreateHeightField( &def );
}

static void computeShapeAABB( b3Shape* shape, b3WorldTransform xf )
{
	b3Transform t = b3ToRelativeTransform( xf, b3Pos_zero );
	switch ( shape->type )
	{
		case b3_sphereShape:
			shape->aabb = b3ComputeSphereAABB( &shape->sphere, t );
			break;
		case b3_capsuleShape:
			shape->aabb = b3ComputeCapsuleAABB( &shape->capsule, t );
			break;
		case b3_hullShape:
			shape->aabb = b3ComputeHullAABB( shape->hull, t );
			break;
		default:
			break;
	}
}

// b3ContactCache is a union: hulls use the SAT cache, capsules the simplex cache, spheres neither.
static void printContact( const char* name, int frame, bool touching, const b3Contact* contact, b3ShapeType typeB )
{
	printf( "%s f%d | %d %d", name, frame, touching ? 1 : 0, contact->manifoldCount );
	for ( int i = 0; i < contact->manifoldCount; ++i )
	{
		const b3Manifold* m = contact->manifolds + i;
		printf( " m" );
		pv( m->normal );
		pf( m->twistImpulse );
		pv( m->frictionImpulse );
		pv( m->rollingImpulse );
		printf( " %d", m->pointCount );
		for ( int j = 0; j < m->pointCount; ++j )
		{
			const b3ManifoldPoint* p = m->points + j;
			printf( " p" );
			pv( p->anchorA );
			pv( p->anchorB );
			pf( p->separation );
			pf( p->normalImpulse );
			printf( " %08x %d %d", p->featureId, p->triangleIndex, p->persisted ? 1 : 0 );
		}
	}
	printf( " | c" );
	pf( contact->friction );
	pf( contact->restitution );
	pf( contact->rollingResistance );
	pv( contact->tangentVelocity );
	printf( " | q" );
	pv( contact->meshContact.queryBounds.lowerBound );
	pv( contact->meshContact.queryBounds.upperBound );
	printf( " | t %d", contact->meshContact.triangleCache.count );
	for ( int i = 0; i < contact->meshContact.triangleCache.count; ++i )
	{
		const b3TriangleCache* tc = contact->meshContact.triangleCache.data + i;
		const b3SATCache* sat = &tc->cache.satCache;
		const b3SimplexCache* sc = &tc->cache.simplexCache;
		printf( " %d", tc->triangleIndex );
		if ( typeB == b3_hullShape )
		{
			printf( ":s%08x,%d,%d,%d,%d", B( sat->separation ), sat->type, sat->indexA, sat->indexB, sat->hit );
		}
		if ( typeB == b3_capsuleShape )
		{
			printf( ":g%08x,%d", B( sc->metric ), sc->count );
			for ( int k = 0; k < sc->count; ++k )
			{
				printf( ",%d,%d", sc->indexA[k], sc->indexB[k] );
			}
		}
	}
	printf( "\n" );
}

// Overwrite the impulses with known values so the next frame shows what matching preserved.
static void stampImpulses( b3Contact* contact, int frame )
{
	for ( int i = 0; i < contact->manifoldCount; ++i )
	{
		b3Manifold* m = contact->manifolds + i;
		float base = (float)( 100 * frame + 10 * i );
		m->twistImpulse = base + 1.0f;
		m->frictionImpulse = (b3Vec3){ base + 2.0f, base + 3.0f, base + 4.0f };
		m->rollingImpulse = (b3Vec3){ base + 5.0f, base + 6.0f, base + 7.0f };
		for ( int j = 0; j < m->pointCount; ++j )
		{
			m->points[j].normalImpulse = base + (float)( j + 1 ) * 0.25f;
		}
	}
}

typedef struct Scenario
{
	const char* name;
	b3Shape* shapeA;
	const uint16_t* materialMap;
	b3Shape* shapeB;
	bool speculative;
	int frameCount;
	b3Vec3 start;
	b3Vec3 velocity;
	b3Quat rotation;
	b3Vec3 spin;
} Scenario;

static void runScenario( const Scenario* s, b3WorldTransform xfA )
{
	b3Contact contact = { 0 };
	b3Array_Create( contact.meshContact.triangleCache );
	contact.flags = s->speculative ? b3_enableSpeculativePoints : 0;

	b3Arena arena = b3CreateArena( 1 << 20 );

	for ( int frame = 0; frame < s->frameCount; ++frame )
	{
		b3Vec3 local = b3MulAdd( s->start, (float)frame, s->velocity );
		b3Quat qLocal;
		qLocal.v = b3MulAdd( s->rotation.v, (float)frame, s->spin );
		qLocal.s = s->rotation.s;
		qLocal = b3NormalizeQuat( qLocal );

		b3WorldTransform xfB;
		xfB.p = b3Add( xfA.p, b3RotateVector( xfA.q, local ) );
		xfB.q = b3MulQuat( xfA.q, qLocal );
		computeShapeAABB( s->shapeB, xfB );

		bool isFast = ( frame % 3 ) == 2;
		bool touching = b3ComputeMeshManifolds( &s_world, 0, &contact, s->shapeA, s->materialMap, xfA, s->shapeB, xfB, isFast, arena );
		printContact( s->name, frame, touching, &contact, s->shapeB->type );
		stampImpulses( &contact, frame );
	}

	b3TaskContext* context = s_world.taskContexts.data;
	printf( "%s sat | %d %d\n", s->name, context->satCallCount, context->satCacheHitCount );
	context->satCallCount = 0;
	context->satCacheHitCount = 0;

	b3FreeManifolds( &s_world, contact.manifolds, contact.manifoldCount );
	b3Array_Destroy( contact.meshContact.triangleCache );
	b3DestroyArena( &arena );
}

static void cullSection( void )
{
	b3Point2D points[16];
	for ( int trial = 0; trial < 60; ++trial )
	{
		int count = 1 + (int)rndInt( 12 );
		int mode = trial % 4;
		for ( int i = 0; i < count; ++i )
		{
			float x = rnd();
			float y = rnd();
			if ( mode == 1 )
			{
				// collinear
				y = x;
			}
			else if ( mode == 2 )
			{
				// within the tolerance
				x = 0.0001f * x;
				y = 0.0001f * y;
			}
			else if ( mode == 3 && i > 0 )
			{
				// duplicates
				x = points[i - 1].p.x;
				y = points[i - 1].p.y;
			}
			points[i].p = (b3Vec2){ x, y };
			points[i].separation = 0.01f * rnd();
			points[i].originalIndex = i;
		}

		int result = b3CullPoints( points, count );
		printf( "cull %d | %d %d", trial, count, result );
		for ( int i = 0; i < result; ++i )
		{
			pf( points[i].p.x );
			pf( points[i].p.y );
			pf( points[i].separation );
			printf( " %d", points[i].originalIndex );
		}
		printf( "\n" );
	}
}

static void reduceSection( void )
{
	b3Arena arena = b3CreateArena( 1 << 16 );
	b3LocalManifoldPoint points[24];
	for ( int trial = 0; trial < 30; ++trial )
	{
		int count = 1 + (int)rndInt( 20 );
		b3Vec3 normal;
		normal.x = rnd();
		normal.y = rnd() + 2.0f;
		normal.z = rnd();
		normal = b3Normalize( normal );
		for ( int i = 0; i < count; ++i )
		{
			points[i] = (b3LocalManifoldPoint){ 0 };
			points[i].point.x = rnd();
			points[i].point.y = 0.1f * rnd();
			points[i].point.z = rnd();
			points[i].separation = 0.02f * rnd();
			points[i].pair.index1 = (uint8_t)i;
			points[i].triangleIndex = (int)rndInt( 100 );
		}

		int result = b3ReduceCluster( points, count, normal, arena );
		printf( "reduce %d | %d %d", trial, count, result );
		for ( int i = 0; i < result; ++i )
		{
			pv( points[i].point );
			pf( points[i].separation );
			printf( " %d %d", points[i].pair.index1, points[i].triangleIndex );
		}
		printf( "\n" );
	}
	b3DestroyArena( &arena );
}

int main( void )
{
	b3SetLogFcn( silentLog );
	setupMaterials();
	setupWorld();

	cullSection();
	reduceSection();

	b3MeshData* grid = makeGrid( 8, 1.0f, 0.4f, 0 );
	b3MeshData* flatGrid = makeGrid( 8, 1.0f, 0.3f, 2 );
	b3MeshData* denseGrid = makeGrid( GRID_MAX, 0.25f, 0.05f, 0 );
	b3HeightFieldData* field = makeField( false );
	b3HeightFieldData* fieldCW = makeField( true );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.3f, 0.4f );
	b3BoxHull offsetBox = b3MakeOffsetBoxHull( 0.3f, 0.2f, 0.6f, (b3Vec3){ 0.1f, 0.05f, -0.1f } );

	b3SurfaceMaterial mB = materialB();

	b3Shape sphere = { 0 };
	sphere.type = b3_sphereShape;
	sphere.sphere = (b3Sphere){ { 0.0f, 0.0f, 0.0f }, 0.45f };
	sphere.material = mB;

	b3Shape bigSphere = sphere;
	bigSphere.sphere.radius = 3.5f;

	b3Shape capsule = { 0 };
	capsule.type = b3_capsuleShape;
	capsule.capsule = (b3Capsule){ { -0.4f, 0.0f, 0.1f }, { 0.4f, 0.05f, -0.1f }, 0.25f };
	capsule.material = mB;

	b3Shape hull = { 0 };
	hull.type = b3_hullShape;
	hull.hull = &box.base;
	hull.material = mB;

	b3Shape offsetHull = hull;
	offsetHull.hull = &offsetBox.base;

	// Mesh shapes. Multi material shapes use the shared material array.
	b3Shape meshMulti = { 0 };
	meshMulti.type = b3_meshShape;
	meshMulti.mesh = (b3Mesh){ grid, { 1.0f, 1.0f, 1.0f } };
	meshMulti.materials = s_materials;
	meshMulti.materialCount = MATERIAL_COUNT;

	b3Shape meshFlipped = { 0 };
	meshFlipped.type = b3_meshShape;
	meshFlipped.mesh = (b3Mesh){ grid, { 1.5f, 0.8f, -1.2f } };
	meshFlipped.material = s_materials[2];

	b3Shape meshScaled = meshMulti;
	meshScaled.mesh.scale = (b3Vec3){ 1.25f, 1.0f, 1.25f };

	b3Shape meshFlat = { 0 };
	meshFlat.type = b3_meshShape;
	meshFlat.mesh = (b3Mesh){ flatGrid, { -1.0f, 1.0f, 1.0f } };
	meshFlat.material = s_materials[1];

	b3Shape meshDense = meshMulti;
	meshDense.mesh = (b3Mesh){ denseGrid, { 1.0f, 1.0f, 1.0f } };

	b3Shape hf = { 0 };
	hf.type = b3_heightShape;
	hf.heightField = field;
	hf.materials = s_materials;
	hf.materialCount = MATERIAL_COUNT;

	b3Shape hfCW = { 0 };
	hfCW.type = b3_heightShape;
	hfCW.heightField = fieldCW;
	hfCW.material = s_materials[3];

	b3WorldTransform xfA;
	xfA.p = (b3Pos){ 0.25f, -0.5f, 0.125f };
	xfA.q = b3NormalizeQuat( (b3Quat){ { 0.05f, 0.1f, -0.03f }, 1.0f } );

	b3Quat q0 = b3NormalizeQuat( (b3Quat){ { 0.1f, 0.2f, 0.05f }, 1.0f } );
	b3Quat qTilt = b3NormalizeQuat( (b3Quat){ { 0.3f, -0.1f, 0.2f }, 1.0f } );
	b3Vec3 spin = { 0.02f, 0.05f, -0.03f };
	b3Vec3 noSpin = { 0.0f, 0.0f, 0.0f };

	Scenario scenarios[] = {
		{ "mesh-sphere", &meshMulti, NULL, &sphere, true, 16, { 0.5f, 0.55f, 0.7f }, { 0.41f, -0.03f, 0.37f }, q0, noSpin },
		{ "mesh-capsule", &meshFlipped, NULL, &capsule, true, 16, { 1.0f, 0.5f, -1.0f }, { 0.5f, -0.02f, -0.45f }, q0, spin },
		{ "mesh-hull", &meshScaled, s_materialMap, &hull, true, 16, { 1.2f, 0.6f, 1.1f }, { 0.43f, -0.025f, 0.38f }, qTilt, spin },
		{ "mesh-hull-nospec", &meshFlipped, NULL, &offsetHull, false, 16, { 1.0f, 0.45f, -1.5f }, { 0.55f, -0.01f, -0.4f }, q0, spin },
		{ "flat-sphere", &meshFlat, NULL, &sphere, true, 16, { -0.5f, 0.4f, 0.3f }, { -0.45f, -0.005f, 0.42f }, q0, noSpin },
		{ "flat-hull", &meshFlat, NULL, &hull, true, 16, { -0.6f, 0.32f, 0.5f }, { -0.4f, -0.002f, 0.4f }, b3Quat_identity, noSpin },
		{ "hf-sphere", &hf, NULL, &sphere, true, 16, { 0.4f, 0.5f, 0.6f }, { 0.35f, -0.03f, 0.45f }, q0, noSpin },
		{ "hf-capsule", &hf, NULL, &capsule, true, 16, { 0.6f, 0.4f, 0.5f }, { 0.33f, -0.015f, 0.42f }, q0, spin },
		{ "hf-hull", &hf, NULL, &hull, true, 16, { 0.5f, 0.55f, 0.4f }, { 0.36f, -0.03f, 0.47f }, qTilt, spin },
		{ "hfcw-hull", &hfCW, NULL, &offsetHull, false, 16, { 0.7f, 0.45f, 0.6f }, { 0.3f, -0.02f, 0.4f }, q0, spin },
		{ "hfcw-sphere", &hfCW, NULL, &sphere, true, 16, { 0.8f, 0.4f, 0.7f }, { 0.31f, -0.02f, 0.41f }, q0, noSpin },
		{ "dense-sphere", &meshDense, NULL, &bigSphere, true, 4, { 5.0f, 0.5f, 5.0f }, { 0.1f, -0.02f, 0.1f }, q0, noSpin },
		{ "dense-hull", &meshDense, s_materialMap, &hull, true, 8, { 5.0f, 0.3f, 5.0f }, { 0.13f, -0.01f, 0.11f }, qTilt, spin },
	};

	for ( int i = 0; i < (int)( sizeof( scenarios ) / sizeof( scenarios[0] ) ); ++i )
	{
		runScenario( scenarios + i, xfA );
	}

	b3DestroyMesh( grid );
	b3DestroyMesh( flatGrid );
	b3DestroyMesh( denseGrid );
	b3DestroyHeightField( field );
	b3DestroyHeightField( fieldCW );
	return 0;
}
