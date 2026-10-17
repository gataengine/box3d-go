// Port of src/joint.h (and later src/joint.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- joint.h ----

// PORT: header types only; joint.c lands in milestone 4. b3World (joints), b3GraphColor and
// b3SolverSet (joint sims) reach these structs. The inline b3SolvePointConstraint lands with
// joint.c. The joint.c functions that milestone 3 files call are placeholders in the joint.c
// section at the end.

// jointEdge is b3JointEdge.
// A joint edge is used to connect bodies and joints together
// in a joint graph where each body is a node and each joint
// is an edge. A joint edge belongs to a doubly linked list
// maintained in each attached body. Each joint has two joint
// nodes, one for each attached body.
type jointEdge struct {
	bodyId  int32
	prevKey int32
	nextKey int32
}

// joint is b3Joint: map from b3JointId to b3Joint in the solver sets
type joint struct {
	userData any

	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	setIndex int32

	// index into the constraint graph color array, may be B3_NULL_INDEX for sleeping/disabled joints
	// B3_NULL_INDEX when slot is free
	colorIndex int32

	// joint index within set or graph color
	// B3_NULL_INDEX when slot is free
	localIndex int32

	edges [2]jointEdge

	jointId  int32
	islandId int32

	// Index into the island's joints array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex int32

	drawScale float32

	typ JointType // PORT: type is a Go keyword

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b3JointId
	generation uint16

	collideConnected bool
}

// distanceJoint is b3DistanceJoint.
type distanceJoint struct {
	length           float32
	hertz            float32
	dampingRatio     float32
	lowerSpringForce float32
	upperSpringForce float32
	minLength        float32
	maxLength        float32

	maxMotorForce float32
	motorSpeed    float32

	impulse      float32
	lowerImpulse float32
	upperImpulse float32
	motorImpulse float32

	indexA           int32
	indexB           int32
	anchorA          Vec3
	anchorB          Vec3
	deltaCenter      Vec3
	distanceSoftness softness
	axialMass        float32

	enableSpring bool
	enableLimit  bool
	enableMotor  bool
}

// motorJoint is b3MotorJoint.
type motorJoint struct {
	linearVelocity      Vec3
	angularVelocity     Vec3
	maxVelocityForce    float32
	maxVelocityTorque   float32
	linearHertz         float32
	linearDampingRatio  float32
	maxSpringForce      float32
	angularHertz        float32
	angularDampingRatio float32
	maxSpringTorque     float32

	linearVelocityImpulse  Vec3
	angularVelocityImpulse Vec3
	linearSpringImpulse    Vec3
	angularSpringImpulse   Vec3

	linearSpring  softness
	angularSpring softness

	indexA      int32
	indexB      int32
	frameA      Transform
	frameB      Transform
	deltaCenter Vec3
	angularMass Matrix3
}

// parallelJoint is b3ParallelJoint.
type parallelJoint struct {
	hertz        float32
	dampingRatio float32
	maxTorque    float32

	perpImpulse Vec2
	perpAxisX   Vec3
	perpAxisY   Vec3

	quatA    Quat
	quatB    Quat
	indexA   int32
	indexB   int32
	softness softness
}

// prismaticJoint is b3PrismaticJoint.
type prismaticJoint struct {
	perpImpulse       Vec2
	angularImpulse    Vec3
	springImpulse     float32
	motorImpulse      float32
	lowerImpulse      float32
	upperImpulse      float32
	hertz             float32
	dampingRatio      float32
	maxMotorForce     float32
	motorSpeed        float32
	targetTranslation float32
	lowerTranslation  float32
	upperTranslation  float32

	indexA         int32
	indexB         int32
	frameA         Transform
	frameB         Transform
	jointAxis      Vec3
	perpAxisY      Vec3
	perpAxisZ      Vec3
	deltaCenter    Vec3
	deltaAngle     float32
	rotationMass   Matrix3
	springSoftness softness

	enableSpring bool
	enableLimit  bool
	enableMotor  bool
}

// revoluteJoint is b3RevoluteJoint.
type revoluteJoint struct {
	linearImpulse  Vec3
	perpImpulse    Vec2
	springImpulse  float32
	motorImpulse   float32
	lowerImpulse   float32
	upperImpulse   float32
	hertz          float32
	dampingRatio   float32
	maxMotorTorque float32
	motorSpeed     float32
	targetAngle    float32
	lowerAngle     float32
	upperAngle     float32

	indexA         int32
	indexB         int32
	frameA         Transform
	frameB         Transform
	rotationAxisZ  Vec3
	perpAxisX      Vec3
	perpAxisY      Vec3
	deltaCenter    Vec3
	deltaAngle     float32
	axialMass      float32
	springSoftness softness

	enableSpring bool
	enableMotor  bool
	enableLimit  bool
}

// sphericalJoint is b3SphericalJoint.
type sphericalJoint struct {
	linearImpulse     Vec3
	springImpulse     Vec3
	motorImpulse      Vec3
	lowerTwistImpulse float32
	upperTwistImpulse float32
	swingImpulse      float32
	hertz             float32
	dampingRatio      float32
	maxMotorTorque    float32
	motorVelocity     Vec3
	lowerTwistAngle   float32
	upperTwistAngle   float32
	coneAngle         float32
	targetRotation    Quat

	indexA        int32
	indexB        int32
	frameA        Transform
	frameB        Transform
	deltaCenter   Vec3
	swingAxis     Vec3
	twistJacobian Vec3

	rotationMass   Matrix3
	swingMass      float32
	twistMass      float32
	springSoftness softness

	enableSpring     bool
	enableMotor      bool
	enableConeLimit  bool
	enableTwistLimit bool
}

// weldJoint is b3WeldJoint.
type weldJoint struct {
	linearHertz         float32
	linearDampingRatio  float32
	angularHertz        float32
	angularDampingRatio float32

	linearSpring   softness
	angularSpring  softness
	linearImpulse  Vec3
	angularImpulse Vec3

	indexA      int32
	indexB      int32
	frameA      Transform
	frameB      Transform
	deltaCenter Vec3

	angularMass Matrix3
}

// wheelJoint is b3WheelJoint.
type wheelJoint struct {
	linearImpulse           Vec2
	angularImpulse          Vec2
	spinImpulse             float32
	maxSpinTorque           float32
	spinSpeed               float32
	suspensionSpringImpulse float32
	lowerSuspensionImpulse  float32
	upperSuspensionImpulse  float32
	lowerSuspensionLimit    float32
	upperSuspensionLimit    float32
	suspensionHertz         float32
	suspensionDampingRatio  float32
	steeringSpringImpulse   float32
	lowerSteeringImpulse    float32
	upperSteeringImpulse    float32
	lowerSteeringLimit      float32
	upperSteeringLimit      float32
	targetSteeringAngle     float32
	maxSteeringTorque       float32
	steeringHertz           float32
	steeringDampingRatio    float32

	indexA             int32
	indexB             int32
	frameA             Transform
	frameB             Transform
	deltaCenter        Vec3
	spinMass           float32
	suspensionMass     float32
	steeringMass       float32
	suspensionSoftness softness
	steeringSoftness   softness

	enableSpinMotor        bool
	enableSuspensionSpring bool
	enableSuspensionLimit  bool
	enableSteering         bool
	enableSteeringLimit    bool
	enableSteeringMotor    bool
}

// jointSim is b3JointSim.
// The base joint class. Joints are used to constraint two bodies together in
// various fashions. Some joints also feature limits and motors.
type jointSim struct {
	jointId int32

	bodyIdA int32
	bodyIdB int32

	typ JointType // PORT: type is a Go keyword

	// Joint frames local to body origin
	localFrameA Transform
	localFrameB Transform

	invMassA, invMassB float32
	invIA, invIB       Matrix3

	constraintHertz        float32
	constraintDampingRatio float32

	constraintSoftness softness

	forceThreshold  float32
	torqueThreshold float32

	fixedRotation bool

	// PORT: anonymous union of the joint types. All members are present; typ says which one is
	// valid.
	distanceJoint  distanceJoint
	motorJoint     motorJoint
	parallelJoint  parallelJoint
	revoluteJoint  revoluteJoint
	sphericalJoint sphericalJoint
	prismaticJoint prismaticJoint
	weldJoint      weldJoint
	wheelJoint     wheelJoint
}

// ---- joint.c ----

// PORT: placeholders until milestone 4 (joint.c). body.c, solver.c and physics_world.c call these
// in milestone 3. No joint can exist before joint.c lands (the b3Create*Joint functions are
// there), so every placeholder that acts on a joint panics; the three overflow functions loop over
// the overflow color's joints exactly as upstream, which is a no-op while there are none. The
// signatures are final; the owner of joint.c replaces the bodies in upstream order.

// DefaultExplosionDef is b3DefaultExplosionDef.
// PORT: pulled forward from joint.c (milestone 4) because b3World_Explode tests need it; it is
// final, and it sits where joint.c defines it relative to the other placeholders.
func DefaultExplosionDef() ExplosionDef {
	def := ExplosionDef{}
	def.MaskBits = DefaultMaskBits
	return def
}

// getJointSim is b3GetJointSim.
func getJointSim(world *world, joint *joint) *jointSim {
	// PORT: placeholder until milestone 4 (joint.c)
	panic("box3d: b3GetJointSim not ported yet (joint.c, milestone 4)")
}

// destroyJointInternal is b3DestroyJointInternal.
func destroyJointInternal(world *world, joint *joint, wakeBodies bool) {
	// PORT: placeholder until milestone 4 (joint.c)
	panic("box3d: b3DestroyJointInternal not ported yet (joint.c, milestone 4)")
}

// getJointReaction is b3GetJointReaction.
// PORT: the force and torque out-parameters are the return values.
func getJointReaction(world *world, sim *jointSim, invTimeStep float32) (force float32, torque float32) {
	// PORT: placeholder until milestone 4 (joint.c)
	panic("box3d: b3GetJointReaction not ported yet (joint.c, milestone 4)")
}

// prepareJoint is b3PrepareJoint.
func prepareJoint(joint *jointSim, context *stepContext) {
	// PORT: placeholder until milestone 4 (joint.c)
	panic("box3d: b3PrepareJoint not ported yet (joint.c, milestone 4)")
}

// warmStartJoint is b3WarmStartJoint.
func warmStartJoint(joint *jointSim, context *stepContext) {
	// PORT: placeholder until milestone 4 (joint.c)
	panic("box3d: b3WarmStartJoint not ported yet (joint.c, milestone 4)")
}

// solveJoint is b3SolveJoint.
func solveJoint(joint *jointSim, context *stepContext, useBias bool) {
	// PORT: placeholder until milestone 4 (joint.c)
	panic("box3d: b3SolveJoint not ported yet (joint.c, milestone 4)")
}

// prepareJointsOverflow is b3PrepareJoints_Overflow.
func prepareJointsOverflow(context *stepContext) {
	// PORT: placeholder until milestone 4 (joint.c); the loop is upstream's.
	graph := context.graph
	joints := graph.colors[overflowIndex].jointSims
	jointCount := len(joints)

	for i := 0; i < jointCount; i++ {
		joint := &joints[i]
		prepareJoint(joint, context)
	}
}

// warmStartJointsOverflow is b3WarmStartJoints_Overflow.
func warmStartJointsOverflow(context *stepContext) {
	// PORT: placeholder until milestone 4 (joint.c); the loop is upstream's.
	graph := context.graph
	joints := graph.colors[overflowIndex].jointSims
	jointCount := len(joints)

	for i := 0; i < jointCount; i++ {
		joint := &joints[i]
		warmStartJoint(joint, context)
	}
}

// solveJointsOverflow is b3SolveJoints_Overflow.
func solveJointsOverflow(context *stepContext, useBias bool) {
	// PORT: placeholder until milestone 4 (joint.c); the loop is upstream's.
	graph := context.graph
	joints := graph.colors[overflowIndex].jointSims
	jointCount := len(joints)

	for i := 0; i < jointCount; i++ {
		joint := &joints[i]
		solveJoint(joint, context, useBias)
	}
}

// drawJoint is b3DrawJoint.
func drawJoint(draw *DebugDraw, world *world, joint *joint) {
	// PORT: placeholder until milestone 4 (joint.c)
	panic("box3d: b3DrawJoint not ported yet (joint.c, milestone 4)")
}
