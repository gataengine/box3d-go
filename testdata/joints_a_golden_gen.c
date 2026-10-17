// Generates testdata/joints_a_golden.txt from the pinned upstream C sources (Go-only tooling).
// Run from third_party/box3d, writing the binary outside the repository ($SCRATCH):
//
// SRC="../../libs/box3d-go/testdata/joints_a_golden_gen.c $(ls src/*.c | grep -v -e src/wide8.c)"
// cc -O2 -ffp-contract=off -std=c17 -I include -I src -o $SCRATCH/joints_a_golden $SRC
// $SCRATCH/joints_a_golden > ../../libs/box3d-go/testdata/joints_a_golden.txt
//
// Check the output against the no-SIMD build (add -DBOX3D_DISABLE_SIMD), the validation build
// (-DBOX3D_VALIDATE) and a 4 worker run (pass 4 as the argument). They must be identical.
//
// A world with distance, motor, prismatic and revolute joints (motors, limits and springs
// enabled) on a static ground box: a rigid distance pendulum, a hinged box carrying a soft distance
// joint with limit, motor and spring force range, a motor joint chasing a velocity with springs, a
// prismatic slider on a rotated axis with spring, limit and motor, a prismatic pair resting on the
// ground, a revolute hinge with spring, limit and motor, a revolute driven by a kinematic body on a
// tilted axis and a free falling revolute pair with a motor. It is stepped JOINTS_GOLDEN_STEPS times
// at 1/60 s with 4 sub-steps; the joint parameters are changed through the b3<Kind>Joint_* setters
// at steps 60, 120 and 180. Every JOINTS_GOLDEN_STRIDE steps (and after the last step) it prints
// every body's transform and velocities, every joint's constraint force and torque (from
// b3Joint_GetConstraintForce/Torque, which for the revolute joint also rewrites its perpendicular
// axes, as upstream), its linear and angular separation and the per-type getters, as hex words,
// then the awake body count and b3HashWorldState.
//
// Inputs are exactly representable or computed without libm, so Go reproduces them exactly.
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

static b3BodyId CreateBox( b3WorldId worldId, b3BodyType type, b3Vec3 p, b3Quat q, const b3BoxHull* box )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = type;
	bodyDef.position = p;
	bodyDef.rotation = q;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3CreateHullShape( bodyId, &shapeDef, &box->base );
	return bodyId;
}

static b3Transform MakeFrame( b3Vec3 p, b3Quat q )
{
	b3Transform frame = { p, q };
	return frame;
}

static void ChangeJoints( int step, b3JointId* joints )
{
	if ( step == 60 )
	{
		b3DistanceJoint_SetLength( joints[0], 2.5f );

		b3DistanceJoint_SetLengthRange( joints[1], 0.8f, 1.2f );
		b3DistanceJoint_EnableMotor( joints[1], false );
		b3DistanceJoint_SetSpringHertz( joints[1], 4.0f );
		b3DistanceJoint_SetSpringDampingRatio( joints[1], 0.5f );
		b3DistanceJoint_SetSpringForceRange( joints[1], -10.0f, 10.0f );

		b3MotorJoint_SetLinearVelocity( joints[3], (b3Vec3){ -0.5f, 0.0f, 0.25f } );
		b3MotorJoint_SetAngularVelocity( joints[3], (b3Vec3){ 0.5f, -1.0f, 0.0f } );
		b3MotorJoint_SetMaxVelocityForce( joints[3], 10.0f );
		b3MotorJoint_SetMaxVelocityTorque( joints[3], 2.0f );

		b3PrismaticJoint_SetLimits( joints[4], 1.0f, -0.5f );
		b3PrismaticJoint_SetTargetTranslation( joints[4], -0.25f );
		b3PrismaticJoint_SetMotorSpeed( joints[4], -1.0f );
		b3PrismaticJoint_SetMaxMotorForce( joints[4], 10.0f );

		b3RevoluteJoint_SetLimits( joints[6], 0.3f, -0.3f );
		b3RevoluteJoint_SetTargetAngle( joints[6], -0.2f );
		b3RevoluteJoint_SetMotorSpeed( joints[6], -2.0f );
		b3RevoluteJoint_SetMaxMotorTorque( joints[6], 5.0f );
		b3RevoluteJoint_SetSpringHertz( joints[6], 3.0f );
		b3RevoluteJoint_SetSpringDampingRatio( joints[6], 0.25f );
	}
	else if ( step == 120 )
	{
		b3DistanceJoint_EnableLimit( joints[1], false );
		b3DistanceJoint_EnableMotor( joints[1], true );
		b3DistanceJoint_SetMotorSpeed( joints[1], -0.75f );
		b3DistanceJoint_SetMaxMotorForce( joints[1], 3.0f );

		b3MotorJoint_SetLinearHertz( joints[3], 4.0f );
		b3MotorJoint_SetLinearDampingRatio( joints[3], 1.0f );
		b3MotorJoint_SetAngularHertz( joints[3], 3.0f );
		b3MotorJoint_SetAngularDampingRatio( joints[3], 0.25f );
		b3MotorJoint_SetMaxSpringForce( joints[3], 25.0f );
		b3MotorJoint_SetMaxSpringTorque( joints[3], -1.0f );

		b3PrismaticJoint_EnableSpring( joints[4], false );
		b3PrismaticJoint_SetSpringHertz( joints[4], 2.0f );
		b3PrismaticJoint_SetSpringDampingRatio( joints[4], 0.75f );
		b3PrismaticJoint_EnableLimit( joints[5], false );
		b3PrismaticJoint_EnableMotor( joints[5], false );

		b3RevoluteJoint_EnableLimit( joints[6], false );
		b3RevoluteJoint_EnableSpring( joints[6], false );
		b3RevoluteJoint_EnableMotor( joints[8], false );
	}
	else if ( step == 180 )
	{
		b3DistanceJoint_EnableSpring( joints[1], false );
		b3DistanceJoint_EnableLimit( joints[1], true );

		b3PrismaticJoint_EnableSpring( joints[4], true );
		b3PrismaticJoint_EnableMotor( joints[4], false );
		b3PrismaticJoint_EnableLimit( joints[5], true );

		b3RevoluteJoint_EnableLimit( joints[6], true );
		b3RevoluteJoint_EnableMotor( joints[6], false );
		b3RevoluteJoint_EnableSpring( joints[7], true );
		b3RevoluteJoint_EnableMotor( joints[8], true );
	}
}

static void PrintJoint( int step, int index, b3JointId jointId )
{
	printf( "joint %d %d |", step, index );
	PrintVec( b3Joint_GetConstraintForce( jointId ) );
	PrintVec( b3Joint_GetConstraintTorque( jointId ) );
	printf( " %08x %08x", Bits( b3Joint_GetLinearSeparation( jointId ) ), Bits( b3Joint_GetAngularSeparation( jointId ) ) );

	switch ( b3Joint_GetType( jointId ) )
	{
		case b3_distanceJoint:
		{
			float lower, upper;
			b3DistanceJoint_GetSpringForceRange( jointId, &lower, &upper );
			printf( " | %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %d%d%d", Bits( b3DistanceJoint_GetCurrentLength( jointId ) ),
					Bits( b3DistanceJoint_GetMotorForce( jointId ) ), Bits( b3DistanceJoint_GetLength( jointId ) ),
					Bits( b3DistanceJoint_GetMinLength( jointId ) ), Bits( b3DistanceJoint_GetMaxLength( jointId ) ),
					Bits( b3DistanceJoint_GetSpringHertz( jointId ) ), Bits( b3DistanceJoint_GetSpringDampingRatio( jointId ) ),
					Bits( b3DistanceJoint_GetMotorSpeed( jointId ) ), Bits( b3DistanceJoint_GetMaxMotorForce( jointId ) ),
					Bits( upper - lower ), b3DistanceJoint_IsSpringEnabled( jointId ), b3DistanceJoint_IsLimitEnabled( jointId ),
					b3DistanceJoint_IsMotorEnabled( jointId ) );
			break;
		}

		case b3_motorJoint:
			printf( " |" );
			PrintVec( b3MotorJoint_GetLinearVelocity( jointId ) );
			PrintVec( b3MotorJoint_GetAngularVelocity( jointId ) );
			printf( " %08x %08x %08x %08x %08x %08x %08x %08x", Bits( b3MotorJoint_GetMaxVelocityForce( jointId ) ),
					Bits( b3MotorJoint_GetMaxVelocityTorque( jointId ) ), Bits( b3MotorJoint_GetLinearHertz( jointId ) ),
					Bits( b3MotorJoint_GetLinearDampingRatio( jointId ) ), Bits( b3MotorJoint_GetAngularHertz( jointId ) ),
					Bits( b3MotorJoint_GetAngularDampingRatio( jointId ) ), Bits( b3MotorJoint_GetMaxSpringForce( jointId ) ),
					Bits( b3MotorJoint_GetMaxSpringTorque( jointId ) ) );
			break;

		case b3_prismaticJoint:
			printf( " | %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %d%d%d", Bits( b3PrismaticJoint_GetTranslation( jointId ) ),
					Bits( b3PrismaticJoint_GetSpeed( jointId ) ), Bits( b3PrismaticJoint_GetMotorForce( jointId ) ),
					Bits( b3PrismaticJoint_GetLowerLimit( jointId ) ), Bits( b3PrismaticJoint_GetUpperLimit( jointId ) ),
					Bits( b3PrismaticJoint_GetTargetTranslation( jointId ) ), Bits( b3PrismaticJoint_GetSpringHertz( jointId ) ),
					Bits( b3PrismaticJoint_GetSpringDampingRatio( jointId ) ), Bits( b3PrismaticJoint_GetMotorSpeed( jointId ) ),
					Bits( b3PrismaticJoint_GetMaxMotorForce( jointId ) ), b3PrismaticJoint_IsSpringEnabled( jointId ),
					b3PrismaticJoint_IsLimitEnabled( jointId ), b3PrismaticJoint_IsMotorEnabled( jointId ) );
			break;

		case b3_revoluteJoint:
			printf( " | %08x %08x %08x %08x %08x %08x %08x %08x %08x %d%d%d", Bits( b3RevoluteJoint_GetAngle( jointId ) ),
					Bits( b3RevoluteJoint_GetMotorTorque( jointId ) ), Bits( b3RevoluteJoint_GetLowerLimit( jointId ) ),
					Bits( b3RevoluteJoint_GetUpperLimit( jointId ) ), Bits( b3RevoluteJoint_GetTargetAngle( jointId ) ),
					Bits( b3RevoluteJoint_GetSpringHertz( jointId ) ), Bits( b3RevoluteJoint_GetSpringDampingRatio( jointId ) ),
					Bits( b3RevoluteJoint_GetMotorSpeed( jointId ) ), Bits( b3RevoluteJoint_GetMaxMotorTorque( jointId ) ),
					b3RevoluteJoint_IsSpringEnabled( jointId ), b3RevoluteJoint_IsLimitEnabled( jointId ),
					b3RevoluteJoint_IsMotorEnabled( jointId ) );
			break;

		default:
			break;
	}

	printf( "\n" );
}

static void RunScene( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyId bodies[16];
	int bodyCount = 0;
	b3JointId joints[16];
	int jointCount = 0;

	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 0.5f, 20.0f );
	b3BoxHull box = b3MakeBoxHull( 0.25f, 0.25f, 0.25f );
	b3BoxHull plank = b3MakeBoxHull( 0.5f, 0.125f, 0.25f );

	b3Quat qz = { { 0.0f, 0.0f, 0.6f }, 0.8f };
	b3Quat qx = { { 0.6f, 0.0f, 0.0f }, 0.8f };
	b3Quat qy = { { 0.0f, 0.8f, 0.0f }, 0.6f };

	b3BodyId groundId = CreateBox( worldId, b3_staticBody, (b3Vec3){ 0.0f, -0.5f, 0.0f }, b3Quat_identity, &groundBox );
	bodies[bodyCount++] = groundId;

	// 0: rigid distance pendulum
	{
		b3BodyId bodyId = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ -6.0f, 4.0f, 0.0f }, b3Quat_identity, &box );
		b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 0.0f, 0.0f, 3.0f } );
		bodies[bodyCount++] = bodyId;

		b3DistanceJointDef def = b3DefaultDistanceJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA.p = (b3Vec3){ -6.0f, 6.5f, 0.0f };
		def.base.localFrameB.p = (b3Vec3){ 0.25f, 0.0f, 0.0f };
		def.length = 2.0f;
		joints[jointCount++] = b3CreateDistanceJoint( worldId, &def );
	}

	// 1, 2: hinged box carrying a soft distance joint (spring, limit, motor, spring force range)
	{
		b3BodyId hingeId = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ -3.0f, 4.0f, 0.0f }, b3Quat_identity, &plank );
		bodies[bodyCount++] = hingeId;
		b3BodyId bobId = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ -2.5f, 2.75f, 0.5f }, qy, &box );
		bodies[bodyCount++] = bobId;

		b3DistanceJointDef def = b3DefaultDistanceJointDef();
		def.base.bodyIdA = hingeId;
		def.base.bodyIdB = bobId;
		def.base.localFrameA.p = (b3Vec3){ 0.5f, 0.0f, 0.0f };
		def.base.localFrameB.p = (b3Vec3){ 0.0f, 0.25f, 0.0f };
		def.length = 1.0f;
		def.enableSpring = true;
		def.hertz = 3.0f;
		def.dampingRatio = 0.2f;
		def.lowerSpringForce = -20.0f;
		def.upperSpringForce = 20.0f;
		def.enableLimit = true;
		def.minLength = 0.6f;
		def.maxLength = 1.6f;
		def.enableMotor = true;
		def.motorSpeed = 0.5f;
		def.maxMotorForce = 2.0f;
		joints[jointCount++] = b3CreateDistanceJoint( worldId, &def );

		b3RevoluteJointDef hinge = b3DefaultRevoluteJointDef();
		hinge.base.bodyIdA = groundId;
		hinge.base.bodyIdB = hingeId;
		hinge.base.localFrameA = MakeFrame( (b3Vec3){ -3.5f, 4.5f, 0.0f }, qx );
		hinge.base.localFrameB = MakeFrame( (b3Vec3){ -0.5f, 0.0f, 0.0f }, qx );
		joints[jointCount++] = b3CreateRevoluteJoint( worldId, &hinge );
	}

	// 3: motor joint chasing a velocity with linear and angular springs
	{
		b3BodyId bodyId = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ 0.0f, 2.0f, 0.0f }, qz, &plank );
		bodies[bodyCount++] = bodyId;

		b3MotorJointDef def = b3DefaultMotorJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA.p = (b3Vec3){ 0.0f, 2.5f, 0.0f };
		def.linearVelocity = (b3Vec3){ 0.5f, 0.0f, 0.0f };
		def.maxVelocityForce = 20.0f;
		def.angularVelocity = (b3Vec3){ 0.0f, 1.0f, 0.0f };
		def.maxVelocityTorque = 5.0f;
		def.linearHertz = 2.0f;
		def.linearDampingRatio = 0.7f;
		def.maxSpringForce = 50.0f;
		def.angularHertz = 1.5f;
		def.angularDampingRatio = 0.5f;
		def.maxSpringTorque = 10.0f;
		joints[jointCount++] = b3CreateMotorJoint( worldId, &def );
	}

	// 4: prismatic slider on a rotated axis with spring, limit and motor
	{
		b3BodyId bodyId = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ 3.0f, 1.5f, 0.0f }, b3Quat_identity, &box );
		bodies[bodyCount++] = bodyId;

		b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA = MakeFrame( (b3Vec3){ 3.0f, 2.0f, 0.0f }, qz );
		def.base.localFrameB = MakeFrame( (b3Vec3){ 0.0f, 0.0f, 0.0f }, qz );
		def.enableSpring = true;
		def.hertz = 1.0f;
		def.dampingRatio = 0.5f;
		def.targetTranslation = 0.5f;
		def.enableLimit = true;
		def.lowerTranslation = -1.0f;
		def.upperTranslation = 1.5f;
		def.enableMotor = true;
		def.motorSpeed = 2.0f;
		def.maxMotorForce = 30.0f;
		joints[jointCount++] = b3CreatePrismaticJoint( worldId, &def );
	}

	// 5: prismatic pair resting on the ground with limit and motor
	{
		b3BodyId bodyIdA = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ 6.0f, 0.25f, 0.0f }, b3Quat_identity, &box );
		bodies[bodyCount++] = bodyIdA;
		b3BodyId bodyIdB = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ 7.0f, 0.25f, 0.0f }, b3Quat_identity, &box );
		bodies[bodyCount++] = bodyIdB;

		b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
		def.base.bodyIdA = bodyIdA;
		def.base.bodyIdB = bodyIdB;
		def.base.localFrameA.p = (b3Vec3){ 0.25f, 0.0f, 0.0f };
		def.base.localFrameB.p = (b3Vec3){ -0.25f, 0.0f, 0.0f };
		def.enableLimit = true;
		def.lowerTranslation = 0.25f;
		def.upperTranslation = 1.0f;
		def.enableMotor = true;
		def.motorSpeed = -1.0f;
		def.maxMotorForce = 5.0f;
		joints[jointCount++] = b3CreatePrismaticJoint( worldId, &def );
	}

	// 6: revolute hinge with spring, limit and motor
	{
		b3BodyId bodyId = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ 9.5f, 3.0f, 0.0f }, b3Quat_identity, &plank );
		bodies[bodyCount++] = bodyId;

		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA.p = (b3Vec3){ 9.0f, 3.5f, 0.0f };
		def.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
		def.enableSpring = true;
		def.hertz = 2.0f;
		def.dampingRatio = 0.1f;
		def.targetAngle = 0.3f;
		def.enableLimit = true;
		def.lowerAngle = -0.5f;
		def.upperAngle = 0.8f;
		def.enableMotor = true;
		def.motorSpeed = 1.5f;
		def.maxMotorTorque = 10.0f;
		joints[jointCount++] = b3CreateRevoluteJoint( worldId, &def );
	}

	// 7: revolute driven by a kinematic body on a tilted axis
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = (b3Vec3){ 12.0f, 2.0f, 0.0f };
		bodyDef.angularVelocity = (b3Vec3){ 0.0f, 0.0f, 1.0f };
		b3BodyId kinematicId = b3CreateBody( worldId, &bodyDef );
		bodies[bodyCount++] = kinematicId;

		b3BodyId bodyId = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ 13.0f, 2.0f, 0.0f }, b3Quat_identity, &plank );
		bodies[bodyCount++] = bodyId;

		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		def.base.bodyIdA = kinematicId;
		def.base.bodyIdB = bodyId;
		def.base.localFrameA = MakeFrame( (b3Vec3){ 0.5f, 0.0f, 0.0f }, qx );
		def.base.localFrameB = MakeFrame( (b3Vec3){ -0.5f, 0.0f, 0.0f }, qx );
		def.hertz = 1.0f;
		def.dampingRatio = 0.5f;
		def.enableLimit = true;
		def.lowerAngle = -1.0f;
		def.upperAngle = 1.0f;
		joints[jointCount++] = b3CreateRevoluteJoint( worldId, &def );
	}

	// 8: free falling revolute pair with a motor
	{
		b3BodyId bodyIdA = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ 15.0f, 5.0f, 0.0f }, qy, &plank );
		bodies[bodyCount++] = bodyIdA;
		b3BodyId bodyIdB = CreateBox( worldId, b3_dynamicBody, (b3Vec3){ 15.0f, 5.0f, 1.0f }, qy, &plank );
		bodies[bodyCount++] = bodyIdB;

		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		def.base.bodyIdA = bodyIdA;
		def.base.bodyIdB = bodyIdB;
		def.base.localFrameA = MakeFrame( (b3Vec3){ 0.0f, 0.0f, 0.5f }, qz );
		def.base.localFrameB = MakeFrame( (b3Vec3){ 0.0f, 0.0f, -0.5f }, qz );
		def.enableMotor = true;
		def.motorSpeed = 4.0f;
		def.maxMotorTorque = 1.0f;
		joints[jointCount++] = b3CreateRevoluteJoint( worldId, &def );
	}

	b3World* world = b3GetWorldFromId( worldId );

	for ( int step = 1; step <= JOINTS_GOLDEN_STEPS; ++step )
	{
		ChangeJoints( step, joints );

		b3World_Step( worldId, 1.0f / 60.0f, 4 );

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
			printf( " %08x", Bits( xf.q.s ) );
			PrintVec( v );
			PrintVec( w );
			printf( " %d\n", b3Body_IsAwake( bodies[i] ) ? 1 : 0 );
		}

		for ( int i = 0; i < jointCount; ++i )
		{
			PrintJoint( step, i, joints[i] );
		}

		printf( "world %d | %d %016llx\n", step, b3World_GetAwakeBodyCount( worldId ),
				(unsigned long long)b3HashWorldState( world ) );
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
