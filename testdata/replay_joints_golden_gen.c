// Generates testdata/replay_joints_golden.b3rec and testdata/replay_joints_golden.txt from the
// pinned upstream C sources (Go-only tooling). Run from third_party/box3d, writing the binary
// outside the repository ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/replay_joints_golden_gen.c $(ls src/*.c | grep -v -e src/wide8.c)"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/replay_joints_golden $SRC
// $SCRATCH/replay_joints_golden ../../libs/box3d-go/testdata/replay_joints_golden.b3rec > ../../libs/box3d-go/testdata/replay_joints_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD), the validation build
// (-DBOX3D_VALIDATE; the .b3rec then differs only in the header's validationEnabled byte) and a 4
// worker run (pass 4 as the second argument). They must be identical.
//
// Records a scene with every joint type, started on the empty world: a static ground box, a
// distance joint (spring, limit, motor), a motor joint, a prismatic slider on a tilted axis
// (spring, limit, motor), a revolute hinge (spring, limit, motor), two spherical pendulums (cone,
// twist, motor; spring), a static pillar with a rigid and a soft weld, a car on four wheel joints
// (suspension, spin motor, steering), a parallel joint and a filter joint between two overlapping
// boxes. While stepping REPLAY_JOINTS_STEPS times at 1/60 s with 4 sub-steps it calls every
// recorded generic b3Joint_* mutator and every b3<Kind>Joint_* setter, creates joints late (one
// with a new body), destroys joints (b3DestroyJoint with and without wakeAttached, and implicitly
// through b3DestroyBody), and puts an island to sleep and wakes it through b3Joint_WakeBodies.
// replay_joints_golden_test.go mirrors the scene line by line (its Go recording must match the
// op stream and registry of this file byte for byte).
//
// The output lists, after every step, the awake body count and b3HashWorldState ("step"), then
// every live body slot (transform, velocities, awake, type) and every live joint slot (type,
// constraint force, linear separation) of the original world after the last step, whether
// b3ValidateReplay accepts the recording, and the same final lines for the world of a b3RecPlayer
// stepped to the end ("replay") and after b3RecPlayer_SeekFrame back to frame 37 and stepped to
// the end again ("cseek"). Every number is a hex word, so the Go replay compares bit for bit.
//
// Inputs are exactly representable or computed without libm (b3MakeQuatFromAxisAngle uses
// b3ComputeCosSin), so Go reproduces them exactly.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "body.h"
#include "joint.h"
#include "physics_world.h"
#include "recording.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REPLAY_JOINTS_STEPS 180

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

	int jointCount = world->joints.count;
	for ( int i = 0; i < jointCount; ++i )
	{
		b3Joint* joint = world->joints.data + i;
		if ( joint->jointId != i )
		{
			continue;
		}
		b3JointId jointId = { i + 1, world->worldId, joint->generation };
		printf( "%s joint %d %d | %d %d %d", tag, i, joint->generation, (int)b3Joint_GetType( jointId ),
				b3Joint_GetBodyA( jointId ).index1, b3Joint_GetBodyB( jointId ).index1 );
		PrintVec( b3Joint_GetConstraintForce( jointId ) );
		printf( " %08x\n", Bits( b3Joint_GetLinearSeparation( jointId ) ) );
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

int main( int argc, char** argv )
{
	if ( argc < 2 )
	{
		fprintf( stderr, "usage: replay_joints_golden <out.b3rec> [workerCount]\n" );
		return 1;
	}

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = argc > 2 ? atoi( argv[2] ) : 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Start on the empty world, so the seed snapshot carries no struct images.
	b3Recording* recording = b3CreateRecording( 0 );
	b3World_StartRecording( worldId, recording );

	// Ground: static box, top face at y = 0
	b3BodyId groundId;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "ground";
		bodyDef.position = (b3Vec3){ 0.0f, -0.5f, 0.0f };
		groundId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 20.0f, 0.5f, 20.0f );
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BoxHull cube = b3MakeCubeHull( 0.25f );
	b3BoxHull bar = b3MakeBoxHull( 0.5f, 0.125f, 0.125f );

	// Cone axis (frame z) pointing down
	b3Quat down = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
	// Suspension axis (frame x) pointing up
	b3Quat up = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
	// 3-4-5 rotation about z
	b3Quat tilt = { { 0.0f, 0.0f, 0.6f }, 0.8f };

	// Distance joint: spring, limit and motor
	b3JointId distanceId;
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -6.0f, 3.0f, -4.0f }, b3Quat_identity );
		b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 2.0f, 0.0f, 1.0f } );

		b3DistanceJointDef def = b3DefaultDistanceJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA = (b3Transform){ { -6.0f, 5.5f, -4.0f }, b3Quat_identity };
		def.base.localFrameB = (b3Transform){ { 0.0f, 0.25f, 0.0f }, b3Quat_identity };
		def.length = 2.0f;
		def.enableSpring = true;
		def.lowerSpringForce = -50.0f;
		def.upperSpringForce = 50.0f;
		def.hertz = 2.0f;
		def.dampingRatio = 0.5f;
		def.enableLimit = true;
		def.minLength = 1.0f;
		def.maxLength = 3.0f;
		def.enableMotor = true;
		def.maxMotorForce = 10.0f;
		def.motorSpeed = 0.5f;
		distanceId = b3CreateDistanceJoint( worldId, &def );
	}

	// Motor joint chasing a velocity with springs
	b3BodyId motorBodyId;
	b3JointId motorId;
	{
		motorBodyId = CreateDynamic( worldId, (b3Vec3){ -3.0f, 1.0f, -4.0f }, b3Quat_identity );
		b3CreateHullShape( motorBodyId, &shapeDef, &cube.base );

		b3MotorJointDef def = b3DefaultMotorJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = motorBodyId;
		def.base.localFrameA = (b3Transform){ { -3.0f, 1.0f, -4.0f }, b3Quat_identity };
		def.base.localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		def.linearVelocity = (b3Vec3){ 0.5f, 0.0f, 0.0f };
		def.maxVelocityForce = 20.0f;
		def.angularVelocity = (b3Vec3){ 0.0f, 1.0f, 0.0f };
		def.maxVelocityTorque = 5.0f;
		def.linearHertz = 2.0f;
		def.linearDampingRatio = 0.75f;
		def.maxSpringForce = 50.0f;
		def.angularHertz = 1.0f;
		def.angularDampingRatio = 0.5f;
		def.maxSpringTorque = 10.0f;
		motorId = b3CreateMotorJoint( worldId, &def );
	}

	// Prismatic slider on a tilted axis: spring, limit and motor
	b3JointId prismaticId;
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ 0.0f, 1.5f, -4.0f }, b3Quat_identity );
		b3CreateHullShape( bodyId, &shapeDef, &cube.base );

		b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA = (b3Transform){ { 0.0f, 2.0f, -4.0f }, tilt };
		def.base.localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, tilt };
		def.enableSpring = true;
		def.hertz = 1.0f;
		def.dampingRatio = 0.5f;
		def.targetTranslation = 0.5f;
		def.enableLimit = true;
		def.lowerTranslation = -1.0f;
		def.upperTranslation = 1.0f;
		def.enableMotor = true;
		def.maxMotorForce = 20.0f;
		def.motorSpeed = 1.0f;
		prismaticId = b3CreatePrismaticJoint( worldId, &def );
	}

	// Revolute hinge: spring, limit and motor
	b3JointId revoluteId;
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ 3.0f, 2.0f, -4.0f }, b3Quat_identity );
		b3CreateHullShape( bodyId, &shapeDef, &bar.base );

		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA = (b3Transform){ { 2.5f, 2.5f, -4.0f }, b3Quat_identity };
		def.base.localFrameB = (b3Transform){ { -0.5f, 0.0f, 0.0f }, b3Quat_identity };
		def.targetAngle = 0.25f;
		def.enableSpring = true;
		def.hertz = 1.0f;
		def.dampingRatio = 0.25f;
		def.enableLimit = true;
		def.lowerAngle = -0.5f;
		def.upperAngle = 0.75f;
		def.enableMotor = true;
		def.maxMotorTorque = 2.0f;
		def.motorSpeed = 1.0f;
		revoluteId = b3CreateRevoluteJoint( worldId, &def );
	}

	// Spherical pendulum with cone limit, twist limit and motor
	b3JointId sphericalLimitId;
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -6.0f, 4.0f, 0.0f }, b3Quat_identity );
		b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 3.0f, 0.0f, 1.0f } );
		b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 0.0f, 4.0f, 0.0f } );

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
	}

	// Spherical pendulum with a spring towards a target rotation
	b3JointId sphericalSpringId;
	{
		b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -8.0f, 4.0f, 0.0f }, b3Quat_identity );
		b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 1.0f, 0.0f, -2.0f } );

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
	}

	// Static pillar with a rigid and a soft weld
	b3JointId weldIds[2];
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Vec3){ -3.0f, 1.5f, 4.0f };
		b3BodyId pillarId = b3CreateBody( worldId, &bodyDef );
		b3BoxHull pillar = b3MakeBoxHull( 0.25f, 1.5f, 0.25f );
		b3CreateHullShape( pillarId, &shapeDef, &pillar.base );

		b3BodyId prevId = pillarId;
		b3Vec3 prevAnchor = { 0.25f, 1.25f, 0.0f };
		for ( int i = 0; i < 2; ++i )
		{
			b3BodyId bodyId = CreateDynamic( worldId, (b3Vec3){ -2.5f + 0.5f * i, 2.75f, 4.0f }, b3Quat_identity );
			b3CreateHullShape( bodyId, &shapeDef, &cube.base );

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
			weldIds[i] = b3CreateWeldJoint( worldId, &def );

			prevId = bodyId;
			prevAnchor = (b3Vec3){ 0.25f, 0.0f, 0.0f };
		}
	}

	// Car: chassis and four sphere wheels on wheel joints
	b3JointId wheelIds[4];
	{
		b3BodyId chassisId = CreateDynamic( worldId, (b3Vec3){ 4.0f, 1.0f, 4.0f }, b3Quat_identity );
		b3BoxHull chassis = b3MakeBoxHull( 1.0f, 0.25f, 0.5f );
		b3CreateHullShape( chassisId, &shapeDef, &chassis.base );

		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.375f };
		b3ShapeDef wheelShapeDef = shapeDef;
		wheelShapeDef.baseMaterial.friction = 0.875f;

		for ( int i = 0; i < 4; ++i )
		{
			float x = ( i & 1 ) ? -0.75f : 0.75f;
			float z = ( i & 2 ) ? -0.75f : 0.75f;
			bool front = x > 0.0f;

			b3BodyId wheelId = CreateDynamic( worldId, (b3Vec3){ 4.0f + x, 0.5f, 4.0f + z }, b3Quat_identity );
			b3CreateSphereShape( wheelId, &wheelShapeDef, &sphere );

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
		}
	}

	// Parallel joint holding a spinning box parallel to the ground
	b3BodyId parallelBodyId;
	b3JointId parallelId;
	{
		parallelBodyId = CreateDynamic( worldId, (b3Vec3){ 8.0f, 2.0f, -3.0f }, b3Quat_identity );
		b3CreateHullShape( parallelBodyId, &shapeDef, &cube.base );
		b3Body_SetAngularVelocity( parallelBodyId, (b3Vec3){ 3.0f, 1.0f, 2.0f } );

		b3ParallelJointDef def = b3DefaultParallelJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = parallelBodyId;
		def.base.localFrameA = (b3Transform){ { 8.0f, 2.5f, -3.0f }, b3Quat_identity };
		def.base.localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		def.base.collideConnected = true;
		def.hertz = 2.0f;
		def.dampingRatio = 0.5f;
		def.maxTorque = 5.0f;
		parallelId = b3CreateParallelJoint( worldId, &def );
	}

	// Filter joint between two overlapping boxes
	b3BodyId filterBodyId;
	b3JointId filterId;
	{
		filterBodyId = CreateDynamic( worldId, (b3Vec3){ -8.0f, 0.25f, 4.0f }, b3Quat_identity );
		b3CreateHullShape( filterBodyId, &shapeDef, &cube.base );
		b3BodyId upperId = CreateDynamic( worldId, (b3Vec3){ -8.0f, 0.625f, 4.125f }, b3Quat_identity );
		b3CreateHullShape( upperId, &shapeDef, &cube.base );

		b3FilterJointDef def = b3DefaultFilterJointDef();
		def.base.bodyIdA = filterBodyId;
		def.base.bodyIdB = upperId;
		filterId = b3CreateFilterJoint( worldId, &def );
	}

	b3BodyId lateBodyId = b3_nullBodyId;
	b3JointId lateJointId = b3_nullJointId;

	for ( int step = 1; step <= REPLAY_JOINTS_STEPS; ++step )
	{
		switch ( step )
		{
			case 20:
				b3Joint_SetLocalFrameA( revoluteId, (b3Transform){ { 2.5f, 2.25f, -4.0f }, b3Quat_identity } );
				b3Joint_SetLocalFrameB( distanceId, (b3Transform){ { 0.0f, 0.25f, 0.125f }, b3Quat_identity } );
				b3Joint_SetCollideConnected( weldIds[1], true );
				b3Joint_SetConstraintTuning( prismaticId, 30.0f, 5.0f );
				b3Joint_SetForceThreshold( distanceId, 5.0f );
				b3Joint_SetTorqueThreshold( revoluteId, 1.0f );
				break;
			case 30:
				b3DistanceJoint_SetLength( distanceId, 1.5f );
				b3DistanceJoint_EnableSpring( distanceId, true );
				b3DistanceJoint_SetSpringForceRange( distanceId, -20.0f, 40.0f );
				b3DistanceJoint_SetSpringHertz( distanceId, 3.0f );
				b3DistanceJoint_SetSpringDampingRatio( distanceId, 0.25f );
				b3DistanceJoint_EnableLimit( distanceId, true );
				b3DistanceJoint_SetLengthRange( distanceId, 0.75f, 2.5f );
				b3DistanceJoint_EnableMotor( distanceId, true );
				b3DistanceJoint_SetMotorSpeed( distanceId, -1.0f );
				b3DistanceJoint_SetMaxMotorForce( distanceId, 5.0f );
				break;
			case 40:
				b3MotorJoint_SetLinearVelocity( motorId, (b3Vec3){ 0.0f, 0.0f, 0.5f } );
				b3MotorJoint_SetAngularVelocity( motorId, (b3Vec3){ 0.5f, 0.0f, 0.0f } );
				b3MotorJoint_SetMaxVelocityForce( motorId, 10.0f );
				b3MotorJoint_SetMaxVelocityTorque( motorId, 2.0f );
				b3MotorJoint_SetLinearHertz( motorId, 4.0f );
				b3MotorJoint_SetLinearDampingRatio( motorId, 0.5f );
				b3MotorJoint_SetAngularHertz( motorId, 2.0f );
				b3MotorJoint_SetAngularDampingRatio( motorId, 0.25f );
				b3MotorJoint_SetMaxSpringForce( motorId, 25.0f );
				b3MotorJoint_SetMaxSpringTorque( motorId, 5.0f );
				break;
			case 50:
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.name = "late";
				bodyDef.position = (b3Vec3){ 0.0f, 4.0f, 2.0f };
				lateBodyId = b3CreateBody( worldId, &bodyDef );
				b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.125f };
				b3CreateCapsuleShape( lateBodyId, &shapeDef, &capsule );

				b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
				def.base.bodyIdA = groundId;
				def.base.bodyIdB = lateBodyId;
				def.base.localFrameA = (b3Transform){ { -0.5f, 4.5f, 2.0f }, b3Quat_identity };
				def.base.localFrameB = (b3Transform){ { -0.5f, 0.0f, 0.0f }, b3Quat_identity };
				lateJointId = b3CreateRevoluteJoint( worldId, &def );
				break;
			}
			case 60:
				b3PrismaticJoint_EnableSpring( prismaticId, true );
				b3PrismaticJoint_SetSpringHertz( prismaticId, 2.0f );
				b3PrismaticJoint_SetSpringDampingRatio( prismaticId, 0.75f );
				b3PrismaticJoint_SetTargetTranslation( prismaticId, -0.5f );
				b3PrismaticJoint_EnableLimit( prismaticId, true );
				b3PrismaticJoint_SetLimits( prismaticId, -0.75f, 0.5f );
				b3PrismaticJoint_EnableMotor( prismaticId, true );
				b3PrismaticJoint_SetMotorSpeed( prismaticId, -2.0f );
				b3PrismaticJoint_SetMaxMotorForce( prismaticId, 10.0f );

				b3RevoluteJoint_EnableSpring( revoluteId, true );
				b3RevoluteJoint_SetSpringHertz( revoluteId, 2.0f );
				b3RevoluteJoint_SetSpringDampingRatio( revoluteId, 0.5f );
				b3RevoluteJoint_SetTargetAngle( revoluteId, -0.25f );
				b3RevoluteJoint_EnableLimit( revoluteId, true );
				b3RevoluteJoint_SetLimits( revoluteId, -0.75f, 0.5f );
				b3RevoluteJoint_EnableMotor( revoluteId, true );
				b3RevoluteJoint_SetMotorSpeed( revoluteId, -1.5f );
				b3RevoluteJoint_SetMaxMotorTorque( revoluteId, 1.0f );
				break;
			case 70:
				b3Body_SetAwake( filterBodyId, false );
				break;
			case 75:
				b3Joint_WakeBodies( filterId );
				break;
			case 80:
				b3SphericalJoint_EnableConeLimit( sphericalLimitId, true );
				b3SphericalJoint_SetConeLimit( sphericalLimitId, 0.75f );
				b3SphericalJoint_EnableTwistLimit( sphericalLimitId, true );
				b3SphericalJoint_SetTwistLimits( sphericalLimitId, -0.5f, 0.5f );
				b3SphericalJoint_EnableMotor( sphericalLimitId, true );
				b3SphericalJoint_SetMotorVelocity( sphericalLimitId, (b3Vec3){ 0.5f, 0.0f, 0.0f } );
				b3SphericalJoint_SetMaxMotorTorque( sphericalLimitId, 1.0f );
				b3SphericalJoint_EnableSpring( sphericalSpringId, true );
				b3SphericalJoint_SetSpringHertz( sphericalSpringId, 3.0f );
				b3SphericalJoint_SetSpringDampingRatio( sphericalSpringId, 0.5f );
				b3SphericalJoint_SetTargetRotation( sphericalSpringId, b3Quat_identity );
				break;
			case 90:
				b3WeldJoint_SetLinearHertz( weldIds[0], 8.0f );
				b3WeldJoint_SetLinearDampingRatio( weldIds[0], 0.5f );
				b3WeldJoint_SetAngularHertz( weldIds[1], 0.0f );
				b3WeldJoint_SetAngularDampingRatio( weldIds[1], 1.0f );
				b3ParallelJoint_SetSpringHertz( parallelId, 4.0f );
				b3ParallelJoint_SetSpringDampingRatio( parallelId, 0.25f );
				b3ParallelJoint_SetMaxTorque( parallelId, 0.5f );
				break;
			case 100:
				b3DestroyJoint( lateJointId, true );
				break;
			case 110:
				b3WheelJoint_EnableSuspension( wheelIds[1], true );
				b3WheelJoint_SetSuspensionHertz( wheelIds[0], 6.0f );
				b3WheelJoint_SetSuspensionDampingRatio( wheelIds[0], 0.5f );
				b3WheelJoint_EnableSuspensionLimit( wheelIds[3], true );
				b3WheelJoint_SetSuspensionLimits( wheelIds[3], -0.125f, 0.25f );
				b3WheelJoint_EnableSpinMotor( wheelIds[0], true );
				b3WheelJoint_SetSpinMotorSpeed( wheelIds[1], 4.0f );
				b3WheelJoint_SetMaxSpinTorque( wheelIds[3], 10.0f );
				b3WheelJoint_EnableSteering( wheelIds[2], true );
				b3WheelJoint_SetSteeringHertz( wheelIds[0], 3.0f );
				b3WheelJoint_SetSteeringDampingRatio( wheelIds[0], 0.5f );
				b3WheelJoint_SetMaxSteeringTorque( wheelIds[0], 30.0f );
				b3WheelJoint_EnableSteeringLimit( wheelIds[2], true );
				b3WheelJoint_SetSteeringLimits( wheelIds[2], -0.125f, 0.125f );
				b3WheelJoint_SetTargetSteeringAngle( wheelIds[0], -0.25f );
				break;
			case 120:
			{
				// Joins the late body's island to the parallel box's
				b3DistanceJointDef def = b3DefaultDistanceJointDef();
				def.base.bodyIdA = lateBodyId;
				def.base.bodyIdB = parallelBodyId;
				def.base.localFrameA = (b3Transform){ { 0.5f, 0.0f, 0.0f }, b3Quat_identity };
				def.base.localFrameB = (b3Transform){ { 0.0f, 0.25f, 0.0f }, b3Quat_identity };
				def.length = 3.0f;
				lateJointId = b3CreateDistanceJoint( worldId, &def );
				break;
			}
			case 130:
				b3DestroyJoint( filterId, false );
				b3DestroyBody( motorBodyId );
				break;
			case 140:
				b3DistanceJoint_EnableSpring( distanceId, false );
				b3RevoluteJoint_EnableLimit( revoluteId, false );
				b3PrismaticJoint_EnableMotor( prismaticId, false );
				b3WheelJoint_EnableSteering( wheelIds[0], false );
				b3SphericalJoint_EnableSpring( sphericalSpringId, false );
				b3Joint_SetCollideConnected( lateJointId, true );
				break;
			case 150:
				b3Joint_WakeBodies( sphericalSpringId );
				b3Joint_SetLocalFrameB( wheelIds[3], (b3Transform){ { 0.0f, 0.0f, 0.125f }, b3Quat_identity } );
				b3Joint_SetConstraintTuning( lateJointId, 60.0f, 2.0f );
				break;
			case 160:
				b3DestroyBody( lateBodyId );
				break;
			default:
				break;
		}

		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		printf( "step %d | %d %016llx\n", step, b3World_GetAwakeBodyCount( worldId ),
				(unsigned long long)b3HashWorldState( b3GetWorldFromId( worldId ) ) );
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

	b3RecPlayer_SeekFrame( player, 37 );
	printf( "seek frame %d\n", b3RecPlayer_GetFrame( player ) );
	while ( b3RecPlayer_StepFrame( player ) )
	{
	}
	printf( "seek frames %d diverged %d\n", b3RecPlayer_GetFrame( player ), b3RecPlayer_HasDiverged( player ) ? 1 : 0 );
	PrintWorld( "cseek", b3RecPlayer_GetWorldId( player ) );
	b3DestroyPlayer( player );

	b3DestroyWorld( worldId );
	b3DestroyRecording( recording );
	return 0;
}
