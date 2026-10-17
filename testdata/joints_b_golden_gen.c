// Generates testdata/joints_b_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/joints_b_golden_gen.c $(ls src/*.c | grep -v -e src/wide8.c)"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/joints_b_golden $SRC
// $SCRATCH/joints_b_golden > ../../libs/box3d-go/testdata/joints_b_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD), the validation build
// (-DBOX3D_VALIDATE) and a 4 worker run (pass 4 as the argument). They must be identical.
//
// A world exercising the spherical, weld, wheel and parallel joints (spherical_joint.c,
// weld_joint.c, wheel_joint.c, parallel_joint.c) on a static box ground:
// - two spherical pendulums: one with cone limit, twist limit and motor, one with a spring;
// - a cantilever of three boxes welded to a static pillar: one rigid weld, one soft weld (linear
//   and angular springs) and one with a soft angular spring only;
// - a car: chassis and four sphere wheels on wheel joints with suspension springs and limits, a
//   spin motor on the rear wheels and steering (spring, target angle, limits) on the front wheels;
// - two parallel joints (collideConnected): a spinning box held parallel to the ground and two
//   falling boxes held parallel to each other.
// At steps 60 and 120 the setters of the four files change the joints (limits, motors, springs,
// steering on/off). Every JOINTS_GOLDEN_STRIDE steps (and after the last step) it prints every
// body's transform and velocities, every joint's constraint force and torque, linear and angular
// separation (angular is skipped for the wheel joints: upstream asserts there), the per-type
// getters, the joint events and b3HashWorldState, all as hex words. At steps 60 and 240 it prints
// the joint debug draw (b3World_Draw with drawJoints and drawGraphColors).
//
// Inputs are exactly representable or computed without libm (b3MakeQuatFromAxisAngle uses
// b3ComputeCosSin), so Go reproduces them exactly.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "physics_world.h"
#include "recording.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define JOINTS_GOLDEN_STEPS 240
#define JOINTS_GOLDEN_STRIDE 20

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

static void PrintFloat( float f )
{
	printf( " %08x", Bits( f ) );
}

static void DrawSegment( b3Pos p1, b3Pos p2, b3HexColor color, void* context )
{
	(void)context;
	printf( "seg |" );
	PrintVec( p1 );
	PrintVec( p2 );
	printf( " %06x\n", (unsigned)color );
}

static void DrawPoint( b3Pos p, float size, b3HexColor color, void* context )
{
	(void)context;
	printf( "point |" );
	PrintVec( p );
	PrintFloat( size );
	printf( " %06x\n", (unsigned)color );
}

static void DrawBox( b3Vec3 extents, b3WorldTransform transform, b3HexColor color, void* context )
{
	(void)context;
	printf( "box |" );
	PrintVec( extents );
	PrintVec( transform.p );
	PrintVec( transform.q.v );
	PrintFloat( transform.q.s );
	printf( " %06x\n", (unsigned)color );
}

static b3BodyId CreateDynamic( b3WorldId worldId, b3Vec3 p, b3Quat q )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = p;
	bodyDef.rotation = q;
	return b3CreateBody( worldId, &bodyDef );
}

static void RunScene( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyId bodies[32];
	int bodyCount = 0;
	b3JointId joints[16];
	int jointCount = 0;

	// Ground: static box, top face at y = 0
	b3BodyId groundId;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ 0.0f, -0.5f, 0.0f };
		groundId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 20.0f, 0.5f, 20.0f );
		b3CreateHullShape( groundId, &shapeDef, &box.base );
		bodies[bodyCount++] = groundId;
	}

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BoxHull cube = b3MakeCubeHull( 0.25f );

	// Cone axis (frame z) pointing down
	b3Quat down = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );

	// Spherical pendulum with cone limit, twist limit and motor
	b3JointId sphericalLimitId;
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -6.0f, 4.0f, 0.0f }, b3Quat_identity );
		b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 3.0f, 0.0f, 1.0f } );
		b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 0.0f, 4.0f, 0.0f } );
		bodies[bodyCount++] = bodyId;

		b3SphericalJointDef def = b3DefaultSphericalJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA = (b3Transform){ { -6.0f, 5.5f, 0.0f }, down };
		def.base.localFrameB = (b3Transform){ { 0.0f, 1.0f, 0.0f }, down };
		def.enableConeLimit = true;
		def.coneAngle = 0.5f;
		def.enableTwistLimit = true;
		def.lowerTwistAngle = -0.25f;
		def.upperTwistAngle = 0.25f;
		def.enableMotor = true;
		def.maxMotorTorque = 0.5f;
		def.motorVelocity = (b3Vec3){ 0.0f, 1.0f, 0.0f };
		sphericalLimitId = b3CreateSphericalJoint( worldId, &def );
		joints[jointCount++] = sphericalLimitId;
	}

	// Spherical pendulum with a spring towards a target rotation
	b3JointId sphericalSpringId;
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -8.0f, 4.0f, 0.0f }, b3Quat_identity );
		b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 1.0f, 0.0f, -2.0f } );
		bodies[bodyCount++] = bodyId;

		b3SphericalJointDef def = b3DefaultSphericalJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA = (b3Transform){ { -8.0f, 5.0f, 0.0f }, b3Quat_identity };
		def.base.localFrameB = (b3Transform){ { 0.0f, 0.5f, 0.0f }, b3Quat_identity };
		def.enableSpring = true;
		def.hertz = 2.0f;
		def.dampingRatio = 0.25f;
		def.targetRotation = (b3Quat){ { 0.0f, 0.6f, 0.0f }, 0.8f };
		sphericalSpringId = b3CreateSphericalJoint( worldId, &def );
		joints[jointCount++] = sphericalSpringId;
	}

	// Cantilever: three boxes welded to a static pillar
	b3JointId weldIds[3];
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ -3.0f, 1.5f, 4.0f };
		b3BodyId pillarId = b3CreateBody( worldId, &bodyDef );
		b3BoxHull pillar = b3MakeBoxHull( 0.25f, 1.5f, 0.25f );
		b3CreateHullShape( pillarId, &shapeDef, &pillar.base );
		bodies[bodyCount++] = pillarId;

		b3BodyId prevId = pillarId;
		b3Vec3 prevAnchor = { 0.25f, 1.25f, 0.0f };
		for ( int i = 0; i < 3; ++i )
		{
			b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -2.5f + 0.5f * i, 2.75f, 4.0f }, b3Quat_identity );
			b3CreateHullShape( bodyId, &shapeDef, &cube.base );
			bodies[bodyCount++] = bodyId;

			b3WeldJointDef def = b3DefaultWeldJointDef();
			def.base.bodyIdA = prevId;
			def.base.bodyIdB = bodyId;
			def.base.localFrameA = (b3Transform){ prevAnchor, b3Quat_identity };
			def.base.localFrameB = (b3Transform){ { -0.25f, 0.0f, 0.0f }, b3Quat_identity };
			if ( i == 1 )
			{
				def.linearHertz = 5.0f;
				def.linearDampingRatio = 0.75f;
				def.angularHertz = 3.0f;
				def.angularDampingRatio = 0.5f;
			}
			else if ( i == 2 )
			{
				def.angularHertz = 2.0f;
				def.angularDampingRatio = 0.25f;
			}
			weldIds[i] = b3CreateWeldJoint( worldId, &def );
			joints[jointCount++] = weldIds[i];

			prevId = bodyId;
			prevAnchor = (b3Vec3){ 0.25f, 0.0f, 0.0f };
		}

		b3Joint_SetForceThreshold( weldIds[0], 20.0f );
	}

	// Car: chassis and four sphere wheels on wheel joints
	b3JointId wheelIds[4];
	b3BodyId chassisId;
	{
		chassisId = CreateDynamic( worldId, (b3Vec3){ 4.0f, 1.0f, 0.0f }, b3Quat_identity );
		b3BoxHull chassis = b3MakeBoxHull( 1.0f, 0.25f, 0.5f );
		b3CreateHullShape( chassisId, &shapeDef, &chassis.base );
		bodies[bodyCount++] = chassisId;

		// Suspension axis (frame x) pointing up
		b3Quat up = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );

		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.375f };
		shapeDef.baseMaterial.friction = 0.875f;

		for ( int i = 0; i < 4; ++i )
		{
			float x = ( i & 1 ) ? -0.75f : 0.75f;
			float z = ( i & 2 ) ? -0.75f : 0.75f;
			bool front = x > 0.0f;

			b3BodyId wheelId = CreateDynamic( worldId, (b3Vec3){ 4.0f + x, 0.5f, z }, b3Quat_identity );
			b3CreateSphereShape( wheelId, &shapeDef, &sphere );
			bodies[bodyCount++] = wheelId;

			b3WheelJointDef def = b3DefaultWheelJointDef();
			def.base.bodyIdA = chassisId;
			def.base.bodyIdB = wheelId;
			def.base.localFrameA = (b3Transform){ { x, -0.5f, z }, up };
			def.base.localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
			def.enableSuspensionSpring = true;
			def.suspensionHertz = 4.0f;
			def.suspensionDampingRatio = 0.625f;
			def.enableSuspensionLimit = true;
			def.lowerSuspensionLimit = -0.25f;
			def.upperSuspensionLimit = 0.125f;
			if ( front )
			{
				def.enableSteering = true;
				def.steeringHertz = 5.0f;
				def.steeringDampingRatio = 0.75f;
				def.maxSteeringTorque = 50.0f;
				def.targetSteeringAngle = 0.25f;
				def.enableSteeringLimit = true;
				def.lowerSteeringLimit = -0.375f;
				def.upperSteeringLimit = 0.375f;
			}
			else
			{
				def.enableSpinMotor = true;
				def.maxSpinTorque = 20.0f;
				def.spinSpeed = -8.0f;
			}
			wheelIds[i] = b3CreateWheelJoint( worldId, &def );
			joints[jointCount++] = wheelIds[i];
		}

		shapeDef.baseMaterial = b3DefaultSurfaceMaterial();
		b3Joint_SetTorqueThreshold( wheelIds[1], 5.0f );
	}

	// Parallel joints
	b3JointId parallelIds[2];
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ 8.0f, 2.0f, -3.0f }, b3Quat_identity );
		b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 3.0f, 1.0f, 2.0f } );
		bodies[bodyCount++] = bodyId;

		b3ParallelJointDef def = b3DefaultParallelJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA = (b3Transform){ { 8.0f, 2.5f, -3.0f }, b3Quat_identity };
		def.base.localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		def.base.collideConnected = true;
		def.hertz = 2.0f;
		def.dampingRatio = 0.5f;
		def.maxTorque = 5.0f;
		parallelIds[0] = b3CreateParallelJoint( worldId, &def );
		joints[jointCount++] = parallelIds[0];

		b3BodyId bodyIdA = CreateDynamic( worldId, (b3Vec3){ 8.0f, 1.0f, 3.0f }, b3Quat_identity );
		b3CreateHullShape( bodyIdA, &shapeDef, &cube.base );
		b3Body_SetAngularVelocity( bodyIdA, (b3Vec3){ 0.0f, 0.0f, 2.0f } );
		bodies[bodyCount++] = bodyIdA;

		b3BodyId bodyIdB = CreateDynamic( worldId, (b3Vec3){ 8.0f, 2.5f, 3.0f }, (b3Quat){ { 0.0f, 0.0f, 0.6f }, 0.8f } );
		b3CreateHullShape( bodyIdB, &shapeDef, &cube.base );
		b3Body_SetAngularVelocity( bodyIdB, (b3Vec3){ -2.0f, 0.0f, 0.0f } );
		bodies[bodyCount++] = bodyIdB;

		def.base.bodyIdA = bodyIdA;
		def.base.bodyIdB = bodyIdB;
		def.base.localFrameA = (b3Transform){ { 0.0f, 0.5f, 0.0f }, b3Quat_identity };
		def.base.localFrameB = (b3Transform){ { 0.0f, -0.5f, 0.0f }, b3Quat_identity };
		def.hertz = 10.0f;
		def.dampingRatio = 1.0f;
		def.maxTorque = 1000.0f;
		parallelIds[1] = b3CreateParallelJoint( worldId, &def );
		joints[jointCount++] = parallelIds[1];
	}

	b3World* world = b3GetWorldFromId( worldId );

	b3DebugDraw draw = b3DefaultDebugDraw();
	draw.DrawSegmentFcn = DrawSegment;
	draw.DrawPointFcn = DrawPoint;
	draw.DrawBoxFcn = DrawBox;
	draw.drawJoints = true;
	draw.drawGraphColors = true;

	for ( int step = 1; step <= JOINTS_GOLDEN_STEPS; ++step )
	{
		if ( step == 60 )
		{
			b3SphericalJoint_SetConeLimit( sphericalLimitId, 0.75f );
			b3SphericalJoint_SetTwistLimits( sphericalLimitId, 0.5f, -0.5f );
			b3SphericalJoint_EnableMotor( sphericalLimitId, false );
			b3SphericalJoint_SetMaxMotorTorque( sphericalLimitId, 1.0f );
			b3SphericalJoint_SetMotorVelocity( sphericalLimitId, (b3Vec3){ 0.5f, 0.0f, 0.0f } );
			b3SphericalJoint_SetTargetRotation( sphericalSpringId, b3Quat_identity );
			b3SphericalJoint_SetSpringHertz( sphericalSpringId, 3.0f );
			b3SphericalJoint_SetSpringDampingRatio( sphericalSpringId, 0.5f );

			b3WeldJoint_SetAngularHertz( weldIds[1], 0.0f );
			b3WeldJoint_SetLinearHertz( weldIds[2], 4.0f );
			b3WeldJoint_SetLinearDampingRatio( weldIds[2], 0.5f );
			b3WeldJoint_SetAngularDampingRatio( weldIds[2], 1.0f );

			b3WheelJoint_SetSpinMotorSpeed( wheelIds[1], 4.0f );
			b3WheelJoint_SetMaxSpinTorque( wheelIds[3], 10.0f );
			b3WheelJoint_SetTargetSteeringAngle( wheelIds[0], -0.25f );
			b3WheelJoint_SetTargetSteeringAngle( wheelIds[2], -0.25f );
			b3WheelJoint_SetSteeringLimits( wheelIds[2], -0.125f, 0.125f );
			b3WheelJoint_SetSuspensionHertz( wheelIds[0], 6.0f );
			b3WheelJoint_SetSuspensionDampingRatio( wheelIds[0], 0.5f );
			b3WheelJoint_SetSuspensionLimits( wheelIds[3], -0.125f, 0.25f );
			b3WheelJoint_SetSteeringHertz( wheelIds[0], 3.0f );
			b3WheelJoint_SetSteeringDampingRatio( wheelIds[0], 0.5f );
			b3WheelJoint_SetMaxSteeringTorque( wheelIds[0], 30.0f );

			b3ParallelJoint_SetMaxTorque( parallelIds[0], 0.5f );
			b3ParallelJoint_SetSpringHertz( parallelIds[1], 4.0f );
			b3ParallelJoint_SetSpringDampingRatio( parallelIds[1], 0.25f );
		}

		if ( step == 120 )
		{
			b3SphericalJoint_EnableConeLimit( sphericalLimitId, false );
			b3SphericalJoint_EnableTwistLimit( sphericalLimitId, false );
			b3SphericalJoint_EnableMotor( sphericalLimitId, true );
			b3SphericalJoint_EnableSpring( sphericalSpringId, false );
			b3SphericalJoint_EnableTwistLimit( sphericalSpringId, true );

			b3WeldJoint_SetLinearHertz( weldIds[1], 0.0f );

			b3WheelJoint_EnableSteering( wheelIds[2], false );
			b3WheelJoint_EnableSteeringLimit( wheelIds[0], false );
			b3WheelJoint_EnableSuspension( wheelIds[1], false );
			b3WheelJoint_EnableSuspensionLimit( wheelIds[3], false );
			b3WheelJoint_EnableSpinMotor( wheelIds[3], false );
			b3WheelJoint_EnableSpinMotor( wheelIds[0], true );
			b3WheelJoint_SetSpinMotorSpeed( wheelIds[0], 2.0f );
			b3WheelJoint_SetMaxSpinTorque( wheelIds[0], 5.0f );
		}

		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3JointEvents events = b3World_GetJointEvents( worldId );
		for ( int i = 0; i < events.count; ++i )
		{
			printf( "event %d | %d\n", step, events.jointEvents[i].jointId.index1 );
		}

		if ( step % JOINTS_GOLDEN_STRIDE != 0 && step != JOINTS_GOLDEN_STEPS )
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
			PrintFloat( xf.q.s );
			PrintVec( v );
			PrintVec( w );
			printf( " %d\n", b3Body_IsAwake( bodies[i] ) ? 1 : 0 );
		}

		for ( int i = 0; i < jointCount; ++i )
		{
			b3JointId jointId = joints[i];
			b3JointType type = b3Joint_GetType( jointId );
			printf( "joint %d %d %d |", step, i, (int)type );
			PrintVec( b3Joint_GetConstraintForce( jointId ) );
			PrintVec( b3Joint_GetConstraintTorque( jointId ) );
			PrintFloat( b3Joint_GetLinearSeparation( jointId ) );
			if ( type != b3_wheelJoint )
			{
				PrintFloat( b3Joint_GetAngularSeparation( jointId ) );
			}

			switch ( type )
			{
				case b3_sphericalJoint:
					PrintFloat( b3SphericalJoint_GetConeAngle( jointId ) );
					PrintFloat( b3SphericalJoint_GetTwistAngle( jointId ) );
					PrintVec( b3SphericalJoint_GetMotorTorque( jointId ) );
					PrintFloat( b3SphericalJoint_GetConeLimit( jointId ) );
					PrintFloat( b3SphericalJoint_GetLowerTwistLimit( jointId ) );
					PrintFloat( b3SphericalJoint_GetUpperTwistLimit( jointId ) );
					printf( " %d%d%d%d", b3SphericalJoint_IsConeLimitEnabled( jointId ), b3SphericalJoint_IsTwistLimitEnabled( jointId ),
							b3SphericalJoint_IsSpringEnabled( jointId ), b3SphericalJoint_IsMotorEnabled( jointId ) );
					break;

				case b3_weldJoint:
					PrintFloat( b3WeldJoint_GetLinearHertz( jointId ) );
					PrintFloat( b3WeldJoint_GetAngularHertz( jointId ) );
					break;

				case b3_wheelJoint:
					PrintFloat( b3WheelJoint_GetSpinSpeed( jointId ) );
					PrintFloat( b3WheelJoint_GetSpinTorque( jointId ) );
					PrintFloat( b3WheelJoint_GetSteeringAngle( jointId ) );
					PrintFloat( b3WheelJoint_GetSteeringTorque( jointId ) );
					printf( " %d%d%d%d%d", b3WheelJoint_IsSuspensionEnabled( jointId ), b3WheelJoint_IsSuspensionLimitEnabled( jointId ),
							b3WheelJoint_IsSpinMotorEnabled( jointId ), b3WheelJoint_IsSteeringEnabled( jointId ),
							b3WheelJoint_IsSteeringLimitEnabled( jointId ) );
					break;

				case b3_parallelJoint:
					PrintFloat( b3ParallelJoint_GetSpringHertz( jointId ) );
					PrintFloat( b3ParallelJoint_GetMaxTorque( jointId ) );
					break;

				default:
					break;
			}

			printf( "\n" );
		}

		b3Counters counters = b3World_GetCounters( worldId );
		printf( "world %d | %d %d %d %016llx\n", step, b3World_GetAwakeBodyCount( worldId ), counters.contactCount,
				counters.islandCount, (unsigned long long)b3HashWorldState( world ) );

		if ( step == 60 || step == JOINTS_GOLDEN_STEPS )
		{
			printf( "draw %d\n", step );
			b3World_Draw( worldId, &draw, B3_DEFAULT_MASK_BITS );
		}
	}

	b3DestroyWorld( worldId );
}

int main( int argc, char** argv )
{
	// The golden is the 1 worker run; pass a worker count to check another one prints the same.
	int workerCount = argc > 1 ? atoi( argv[1] ) : 1;
	RunScene( workerCount );
	return 0;
}
