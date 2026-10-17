// Port of src/recording.h (and later src/recording.c) from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"math"
	"unsafe"
)

// ---- recording.h ----

// PORT: recording.h is ported in full (types, constants, inline). The arg structs and opcode
// constants it generates from the X-macro manifest are in recording_ops.go. Every function it
// declares is defined in recording.c, which lands in milestone 5; until then each one is a stub
// below that does nothing and returns zero values. world.recording stays nil in milestones 3 and 4
// (startRecordingIntoBuffer is a stub), so the B3_REC hooks in body.c, shape.c, joint.c and
// physics_world.c never reach the stubs.
//
// B3_REC( world, Name, args... ) expands at the call site to
//
//	if world.recording != nil {
//		recArgs := recArgsName{args...}
//		recWriteName(world.recording, &recArgs)
//	}
//
// and B3_REC_CREATE( world, Name, id, args... ) to
//
//	if world.recording != nil {
//		recCreateArgs := recArgsName{args...}
//		recWriteRetName(world.recording, &recCreateArgs, id)
//	}

// FNV-1a 64-bit constants
const (
	snapFnvInit  uint64 = 14695981039346656037 // B3_SNAP_FNV_INIT
	snapFnvPrime uint64 = 1099511628211        // B3_SNAP_FNV_PRIME
)

// fnvMixPosition is b3FnvMixPosition.
// Mix a world position at full width so the determinism gate validates past float precision
// when the body is far from the origin.
// PORT: float build only (BOX3D_DOUBLE_PRECISION is not ported).
func fnvMixPosition(hash uint64, p Pos) uint64 {
	fx := math.Float32bits(p.X)
	fy := math.Float32bits(p.Y)
	fz := math.Float32bits(p.Z)
	bx, by, bz := uint64(fx), uint64(fy), uint64(fz)
	hash = (hash ^ bx) * snapFnvPrime
	hash = (hash ^ by) * snapFnvPrime
	hash = (hash ^ bz) * snapFnvPrime
	return hash
}

// Magic 'B3RC' in little-endian: bytes B(0x42) 3(0x33) R(0x52) C(0x43)
const recMagic uint32 = 0x43523342 // B3_REC_MAGIC

// Major recording version is bumped when writers change.
// Major version 6 widened the body def with the safety factor.
const recVersionMajor = 6 // B3_REC_VERSION_MAJOR

// Minor tracks op-stream additions that keep the 48 byte header shape.
// Minor version 7 made the contact body sim locators persistent
const recVersionMinor = 9 // B3_REC_VERSION_MINOR

// recHeader is b3RecHeader.
// File header, fixed 48 bytes. Contains the registry locator so the player
// can load geometry before replaying any ops.
type recHeader struct {
	magic             uint32 // 'B3RC' = 0x43523342
	versionMajor      uint16 // B3_REC_VERSION_MAJOR
	versionMinor      uint16 // B3_REC_VERSION_MINOR
	pointerWidth      uint8  // sizeof(void*), gates POD-def struct layout
	bigEndian         uint8  // 0 on all supported targets
	validationEnabled uint8  // 1 if built with BOX3D_VALIDATE, diagnostic only
	reserved          uint8
	lengthScale       float32 // b3GetLengthUnitsPerMeter()
	reserved2         uint32
	reserved3         uint32 // explicit pad so the 64-bit fields align with no implicit gap
	snapshotSize      uint64 // bytes of snapshot blob after the header (0 in Phase 1)
	registryOffset    uint64 // absolute offset to trailing registry block, backpatched at stop
	registryByteCount uint64 // size of the registry block
}

// PORT: _Static_assert( sizeof( b3RecHeader ) == 48 ) as constant expressions.
var _ = [1]struct{}{}[unsafe.Sizeof(recHeader{})-48] // recording header must be 48 bytes
var _ = [1]struct{}{}[48-unsafe.Sizeof(recHeader{})] // recording header must be 48 bytes

// recBuffer is b3RecBuffer.
// Growable append-only byte buffer. Doubles on demand. countOnly mode tallies size without
// allocating, used to size a buffer cheaply before a second filling pass.
// PORT: uint8_t* data is a slice; capacity and size are kept.
type recBuffer struct {
	data      []byte
	capacity  int32
	size      int32
	countOnly bool
}

// geometryKind is b3GeometryKind: geometry kinds for the trailing registry section
type geometryKind int32

const (
	geometryHull geometryKind = iota
	geometryMesh
	geometryHeightField
	geometryCompound
)

// geometryEntry is b3GeometryEntry.
// One entry per unique geometry blob. id == index in the entries array.
// hashNext chains entries that share a content hash so dedup stays exact under a hash collision,
// which the keyframe registry depends on to never grow during capture. B3_NULL_INDEX ends the chain.
// PORT: uint8_t* bytes is a slice; byteCount is kept.
type geometryEntry struct {
	contentHash uint64
	id          uint32
	kind        geometryKind
	byteCount   int32
	bytes       []byte
	hashNext    int32
}

// geometryRegistry is b3GeometryRegistry.
// Growable array of geometry entries. Ids are array indices, so the array is serialized in order.
// dedupMap maps content hash to entry id for O(1) dedup; it is opaque here and owned by recording.c.
// PORT: void* dedupMap is `any` until recording.c types it.
type geometryRegistry struct {
	entries  []geometryEntry
	dedupMap any
}

// Limit the maximum query name length to make recording simpler. Query names longer than this
// probably indicate a bug in user code.
const maxQueryNameLength = 64 // B3_MAX_QUERY_NAME_LENGTH

// recTag is b3RecTag.
// Query tag from b3QueryFilter.
// Stored once per key in the trailing block so a tagged query carries only the 8 byte key on the wire.
// Shared by the recorder (accumulate) and the player (load).
type recTag struct {
	// hash of (id, queryName)
	key       uint64
	id        uint64
	queryName [maxQueryNameLength + 1]byte
}

// Recording is b3Recording.
// User-owned recording buffer. The world appends into it while active; the host saves and
// destroys it. Opaque across the public API.
// PORT: exported because the public API (box3d.h) hands it out (b3CreateRecording,
// b3World_StartRecording); all fields are unexported. b3RecTag* tags is a slice with tagCount and
// tagCapacity kept; void* tagMap is `any` until recording.c types it.
type Recording struct {
	buffer      recBuffer
	recordStart int32  // offset of the 3-byte size field for u24 backpatch
	lock        *mutex // serializes record writes from concurrent threads
	registry    geometryRegistry

	// Interned query tags accumulated during capture, written to the tail of the registry block at stop.
	// tagMap maps a tag key to its index for O(1) dedup.
	tags        []recTag
	tagCount    int32
	tagCapacity int32
	tagMap      any

	// Union of world bounds over every recorded step, written at stop.
	accumulatedBounds AABB
	haveBounds        bool
}

// recQueryWriter is b3RecQueryWriter.
// Per-query writer context: holds the user fcn+ctx, the local payload buffer, and the hit counter
// PORT: union userFcn { overlapFcn; castFcn; planeFcn; moverFilterFcn }; all members are present.
// const char* tagName is a string (empty = NULL).
type recQueryWriter struct {
	overlapFcn     OverlapResultFcn
	castFcn        CastResultFcn
	planeFcn       PlaneResultFcn
	moverFilterFcn MoverFilterFcn
	userContext    any
	buf            recBuffer // per-call local payload, heap-backed
	countOffset    int32     // offset of the reserved u32 hit-count slot
	hitCount       uint32
	tagId          uint64 // caller query id, 0 = untagged. Emitted as a QueryTag before the record.
	tagName        string // caller query name, interned by id. NULL = none.
}

// ---- recording.c ----

// PORT: every function below is a stub until milestone 5 (recording.c): it does nothing and returns
// zero values. The signatures are final; recording.c fills in the bodies in upstream order.
// Pointer + size parameters are slices; const char* is a string; void* context is any.

// Low-level buffer helpers

// recBufAppend is b3RecBufAppend.
func recBufAppend(buf *recBuffer, data []byte) {
	// PORT: stub until milestone 5 (recording.c)
}

// recBufFree is b3RecBufFree.
func recBufFree(buf *recBuffer) {
	// PORT: stub until milestone 5 (recording.c)
}

// Write primitives
// recWU8 is b3RecW_U8.
func recWU8(buf *recBuffer, v uint8) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWU16 is b3RecW_U16.
func recWU16(buf *recBuffer, v uint16) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWU32 is b3RecW_U32.
func recWU32(buf *recBuffer, v uint32) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWU64 is b3RecW_U64.
func recWU64(buf *recBuffer, v uint64) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWI32 is b3RecW_I32.
func recWI32(buf *recBuffer, v int32) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWF32 is b3RecW_F32.
func recWF32(buf *recBuffer, v float32) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWF64 is b3RecW_F64.
func recWF64(buf *recBuffer, v float64) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWBOOL is b3RecW_BOOL.
func recWBOOL(buf *recBuffer, v bool) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWVEC3 is b3RecW_VEC3.
func recWVEC3(buf *recBuffer, v Vec3) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWQUAT is b3RecW_QUAT.
func recWQUAT(buf *recBuffer, v Quat) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWTRANSFORM is b3RecW_TRANSFORM.
func recWTRANSFORM(buf *recBuffer, v Transform) {
	// PORT: stub until milestone 5 (recording.c)
}

// World position: doubles in large-world mode, floats otherwise (wire-identical to VEC3 in float build)
// recWPOSITION is b3RecW_POSITION.
func recWPOSITION(buf *recBuffer, v Pos) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWWORLDXF is b3RecW_WORLDXF.
func recWWORLDXF(buf *recBuffer, v WorldTransform) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWMATRIX3 is b3RecW_MATRIX3.
func recWMATRIX3(buf *recBuffer, v Matrix3) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWAABB is b3RecW_AABB.
func recWAABB(buf *recBuffer, v AABB) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWQUERYFILTER is b3RecW_QUERYFILTER.
func recWQUERYFILTER(buf *recBuffer, v QueryFilter) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWSHAPEPROXY is b3RecW_SHAPEPROXY.
func recWSHAPEPROXY(buf *recBuffer, v ShapeProxy) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWTREESTATS is b3RecW_TREESTATS.
func recWTREESTATS(buf *recBuffer, v TreeStats) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWRAYRESULT is b3RecW_RAYRESULT.
func recWRAYRESULT(buf *recBuffer, v RayResult) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWPLANERESULT is b3RecW_PLANERESULT.
func recWPLANERESULT(buf *recBuffer, v PlaneResult) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWWORLDID is b3RecW_WORLDID.
func recWWORLDID(buf *recBuffer, v WorldID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWBODYID is b3RecW_BODYID.
func recWBODYID(buf *recBuffer, v BodyID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWSHAPEID is b3RecW_SHAPEID.
func recWSHAPEID(buf *recBuffer, v ShapeID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWJOINTID is b3RecW_JOINTID.
func recWJOINTID(buf *recBuffer, v JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWSPHERE is b3RecW_SPHERE.
func recWSPHERE(buf *recBuffer, v Sphere) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWCAPSULE is b3RecW_CAPSULE.
func recWCAPSULE(buf *recBuffer, v Capsule) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWGEOMID is b3RecW_GEOMID.
func recWGEOMID(buf *recBuffer, v uint32) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWFILTER is b3RecW_FILTER.
func recWFILTER(buf *recBuffer, v Filter) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWMATERIAL is b3RecW_MATERIAL.
func recWMATERIAL(buf *recBuffer, v SurfaceMaterial) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWMASSDATA is b3RecW_MASSDATA.
func recWMASSDATA(buf *recBuffer, v MassData) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWLOCKS is b3RecW_LOCKS.
func recWLOCKS(buf *recBuffer, v MotionLocks) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWEXPLOSIONDEF is b3RecW_EXPLOSIONDEF.
func recWEXPLOSIONDEF(buf *recBuffer, v ExplosionDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWBODYDEF is b3RecW_BODYDEF.
func recWBODYDEF(buf *recBuffer, v BodyDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWSHAPEDEF is b3RecW_SHAPEDEF.
func recWSHAPEDEF(buf *recBuffer, v ShapeDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWPARALLELJOINTDEF is b3RecW_PARALLELJOINTDEF.
func recWPARALLELJOINTDEF(buf *recBuffer, v ParallelJointDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWDISTANCEJOINTDEF is b3RecW_DISTANCEJOINTDEF.
func recWDISTANCEJOINTDEF(buf *recBuffer, v DistanceJointDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWFILTERJOINTDEF is b3RecW_FILTERJOINTDEF.
func recWFILTERJOINTDEF(buf *recBuffer, v FilterJointDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWMOTORJOINTDEF is b3RecW_MOTORJOINTDEF.
func recWMOTORJOINTDEF(buf *recBuffer, v MotorJointDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWPRISMATICJOINTDEF is b3RecW_PRISMATICJOINTDEF.
func recWPRISMATICJOINTDEF(buf *recBuffer, v PrismaticJointDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWREVOLUTEJOINTDEF is b3RecW_REVOLUTEJOINTDEF.
func recWREVOLUTEJOINTDEF(buf *recBuffer, v RevoluteJointDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWSPHERICALJOINTDEF is b3RecW_SPHERICALJOINTDEF.
func recWSPHERICALJOINTDEF(buf *recBuffer, v SphericalJointDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWWELDJOINTDEF is b3RecW_WELDJOINTDEF.
func recWWELDJOINTDEF(buf *recBuffer, v WeldJointDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWWHEELJOINTDEF is b3RecW_WHEELJOINTDEF.
func recWWHEELJOINTDEF(buf *recBuffer, v WheelJointDef) {
	// PORT: stub until milestone 5 (recording.c)
}

// Record framing

// recBeginRecord is b3RecBeginRecord.
func recBeginRecord(rec *Recording, opcode uint8) {
	// PORT: stub until milestone 5 (recording.c)
}

// recEndRecord is b3RecEndRecord.
func recEndRecord(rec *Recording) {
	// PORT: stub until milestone 5 (recording.c)
}

// Per-op full writers (framing + args), generated from the manifest (recording_ops.go).
// PORT: b3RecWriteArgs_Name (args without framing) is only called inside recording.c and lands
// with it.

// recWriteDestroyWorld is b3RecWrite_DestroyWorld.
func recWriteDestroyWorld(rec *Recording, a *recArgsDestroyWorld) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteStep is b3RecWrite_Step.
func recWriteStep(rec *Recording, a *recArgsStep) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldEnableSleeping is b3RecWrite_WorldEnableSleeping.
func recWriteWorldEnableSleeping(rec *Recording, a *recArgsWorldEnableSleeping) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldEnableContinuous is b3RecWrite_WorldEnableContinuous.
func recWriteWorldEnableContinuous(rec *Recording, a *recArgsWorldEnableContinuous) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldSetRestitutionThreshold is b3RecWrite_WorldSetRestitutionThreshold.
func recWriteWorldSetRestitutionThreshold(rec *Recording, a *recArgsWorldSetRestitutionThreshold) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldSetHitEventThreshold is b3RecWrite_WorldSetHitEventThreshold.
func recWriteWorldSetHitEventThreshold(rec *Recording, a *recArgsWorldSetHitEventThreshold) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldSetGravity is b3RecWrite_WorldSetGravity.
func recWriteWorldSetGravity(rec *Recording, a *recArgsWorldSetGravity) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldExplode is b3RecWrite_WorldExplode.
func recWriteWorldExplode(rec *Recording, a *recArgsWorldExplode) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldSetContactTuning is b3RecWrite_WorldSetContactTuning.
func recWriteWorldSetContactTuning(rec *Recording, a *recArgsWorldSetContactTuning) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldSetContactRecycleDistance is b3RecWrite_WorldSetContactRecycleDistance.
func recWriteWorldSetContactRecycleDistance(rec *Recording, a *recArgsWorldSetContactRecycleDistance) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldSetMaximumLinearSpeed is b3RecWrite_WorldSetMaximumLinearSpeed.
func recWriteWorldSetMaximumLinearSpeed(rec *Recording, a *recArgsWorldSetMaximumLinearSpeed) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldEnableWarmStarting is b3RecWrite_WorldEnableWarmStarting.
func recWriteWorldEnableWarmStarting(rec *Recording, a *recArgsWorldEnableWarmStarting) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldRebuildStaticTree is b3RecWrite_WorldRebuildStaticTree.
func recWriteWorldRebuildStaticTree(rec *Recording, a *recArgsWorldRebuildStaticTree) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldEnableSpeculative is b3RecWrite_WorldEnableSpeculative.
func recWriteWorldEnableSpeculative(rec *Recording, a *recArgsWorldEnableSpeculative) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldSetRestitutionIterations is b3RecWrite_WorldSetRestitutionIterations.
func recWriteWorldSetRestitutionIterations(rec *Recording, a *recArgsWorldSetRestitutionIterations) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldEnableRestitutionPropagation is b3RecWrite_WorldEnableRestitutionPropagation.
func recWriteWorldEnableRestitutionPropagation(rec *Recording, a *recArgsWorldEnableRestitutionPropagation) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWorldSetIsotropicInertiaTolerance is b3RecWrite_WorldSetIsotropicInertiaTolerance.
func recWriteWorldSetIsotropicInertiaTolerance(rec *Recording, a *recArgsWorldSetIsotropicInertiaTolerance) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateBody is b3RecWrite_CreateBody.
func recWriteCreateBody(rec *Recording, a *recArgsCreateBody) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDestroyBody is b3RecWrite_DestroyBody.
func recWriteDestroyBody(rec *Recording, a *recArgsDestroyBody) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetTransform is b3RecWrite_BodySetTransform.
func recWriteBodySetTransform(rec *Recording, a *recArgsBodySetTransform) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetLinearVelocity is b3RecWrite_BodySetLinearVelocity.
func recWriteBodySetLinearVelocity(rec *Recording, a *recArgsBodySetLinearVelocity) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetType is b3RecWrite_BodySetType.
func recWriteBodySetType(rec *Recording, a *recArgsBodySetType) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetName is b3RecWrite_BodySetName.
func recWriteBodySetName(rec *Recording, a *recArgsBodySetName) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetAngularVelocity is b3RecWrite_BodySetAngularVelocity.
func recWriteBodySetAngularVelocity(rec *Recording, a *recArgsBodySetAngularVelocity) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetTargetTransform is b3RecWrite_BodySetTargetTransform.
func recWriteBodySetTargetTransform(rec *Recording, a *recArgsBodySetTargetTransform) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyApplyForce is b3RecWrite_BodyApplyForce.
func recWriteBodyApplyForce(rec *Recording, a *recArgsBodyApplyForce) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyApplyForceToCenter is b3RecWrite_BodyApplyForceToCenter.
func recWriteBodyApplyForceToCenter(rec *Recording, a *recArgsBodyApplyForceToCenter) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyApplyTorque is b3RecWrite_BodyApplyTorque.
func recWriteBodyApplyTorque(rec *Recording, a *recArgsBodyApplyTorque) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyApplyLinearImpulse is b3RecWrite_BodyApplyLinearImpulse.
func recWriteBodyApplyLinearImpulse(rec *Recording, a *recArgsBodyApplyLinearImpulse) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyApplyLinearImpulseToCenter is b3RecWrite_BodyApplyLinearImpulseToCenter.
func recWriteBodyApplyLinearImpulseToCenter(rec *Recording, a *recArgsBodyApplyLinearImpulseToCenter) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyApplyAngularImpulse is b3RecWrite_BodyApplyAngularImpulse.
func recWriteBodyApplyAngularImpulse(rec *Recording, a *recArgsBodyApplyAngularImpulse) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetMassData is b3RecWrite_BodySetMassData.
func recWriteBodySetMassData(rec *Recording, a *recArgsBodySetMassData) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyApplyMassFromShapes is b3RecWrite_BodyApplyMassFromShapes.
func recWriteBodyApplyMassFromShapes(rec *Recording, a *recArgsBodyApplyMassFromShapes) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetLinearDamping is b3RecWrite_BodySetLinearDamping.
func recWriteBodySetLinearDamping(rec *Recording, a *recArgsBodySetLinearDamping) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetAngularDamping is b3RecWrite_BodySetAngularDamping.
func recWriteBodySetAngularDamping(rec *Recording, a *recArgsBodySetAngularDamping) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetGravityScale is b3RecWrite_BodySetGravityScale.
func recWriteBodySetGravityScale(rec *Recording, a *recArgsBodySetGravityScale) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetAwake is b3RecWrite_BodySetAwake.
func recWriteBodySetAwake(rec *Recording, a *recArgsBodySetAwake) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyEnableSleep is b3RecWrite_BodyEnableSleep.
func recWriteBodyEnableSleep(rec *Recording, a *recArgsBodyEnableSleep) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetSleepThreshold is b3RecWrite_BodySetSleepThreshold.
func recWriteBodySetSleepThreshold(rec *Recording, a *recArgsBodySetSleepThreshold) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyDisable is b3RecWrite_BodyDisable.
func recWriteBodyDisable(rec *Recording, a *recArgsBodyDisable) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyEnable is b3RecWrite_BodyEnable.
func recWriteBodyEnable(rec *Recording, a *recArgsBodyEnable) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetMotionLocks is b3RecWrite_BodySetMotionLocks.
func recWriteBodySetMotionLocks(rec *Recording, a *recArgsBodySetMotionLocks) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetBullet is b3RecWrite_BodySetBullet.
func recWriteBodySetBullet(rec *Recording, a *recArgsBodySetBullet) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyEnableContactRecycling is b3RecWrite_BodyEnableContactRecycling.
func recWriteBodyEnableContactRecycling(rec *Recording, a *recArgsBodyEnableContactRecycling) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyEnableHitEvents is b3RecWrite_BodyEnableHitEvents.
func recWriteBodyEnableHitEvents(rec *Recording, a *recArgsBodyEnableHitEvents) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodyAllowFastRotation is b3RecWrite_BodyAllowFastRotation.
func recWriteBodyAllowFastRotation(rec *Recording, a *recArgsBodyAllowFastRotation) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteBodySetSafetyFactor is b3RecWrite_BodySetSafetyFactor.
func recWriteBodySetSafetyFactor(rec *Recording, a *recArgsBodySetSafetyFactor) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateSphereShape is b3RecWrite_CreateSphereShape.
func recWriteCreateSphereShape(rec *Recording, a *recArgsCreateSphereShape) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateCapsuleShape is b3RecWrite_CreateCapsuleShape.
func recWriteCreateCapsuleShape(rec *Recording, a *recArgsCreateCapsuleShape) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateHullShape is b3RecWrite_CreateHullShape.
func recWriteCreateHullShape(rec *Recording, a *recArgsCreateHullShape) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateMeshShape is b3RecWrite_CreateMeshShape.
func recWriteCreateMeshShape(rec *Recording, a *recArgsCreateMeshShape) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateHeightFieldShape is b3RecWrite_CreateHeightFieldShape.
func recWriteCreateHeightFieldShape(rec *Recording, a *recArgsCreateHeightFieldShape) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateCompoundShape is b3RecWrite_CreateCompoundShape.
func recWriteCreateCompoundShape(rec *Recording, a *recArgsCreateCompoundShape) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDestroyShape is b3RecWrite_DestroyShape.
func recWriteDestroyShape(rec *Recording, a *recArgsDestroyShape) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetDensity is b3RecWrite_ShapeSetDensity.
func recWriteShapeSetDensity(rec *Recording, a *recArgsShapeSetDensity) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetFriction is b3RecWrite_ShapeSetFriction.
func recWriteShapeSetFriction(rec *Recording, a *recArgsShapeSetFriction) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetRestitution is b3RecWrite_ShapeSetRestitution.
func recWriteShapeSetRestitution(rec *Recording, a *recArgsShapeSetRestitution) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetSurfaceMaterial is b3RecWrite_ShapeSetSurfaceMaterial.
func recWriteShapeSetSurfaceMaterial(rec *Recording, a *recArgsShapeSetSurfaceMaterial) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetFilter is b3RecWrite_ShapeSetFilter.
func recWriteShapeSetFilter(rec *Recording, a *recArgsShapeSetFilter) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeEnableSensorEvents is b3RecWrite_ShapeEnableSensorEvents.
func recWriteShapeEnableSensorEvents(rec *Recording, a *recArgsShapeEnableSensorEvents) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeEnableContactEvents is b3RecWrite_ShapeEnableContactEvents.
func recWriteShapeEnableContactEvents(rec *Recording, a *recArgsShapeEnableContactEvents) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeEnablePreSolveEvents is b3RecWrite_ShapeEnablePreSolveEvents.
func recWriteShapeEnablePreSolveEvents(rec *Recording, a *recArgsShapeEnablePreSolveEvents) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeEnableHitEvents is b3RecWrite_ShapeEnableHitEvents.
func recWriteShapeEnableHitEvents(rec *Recording, a *recArgsShapeEnableHitEvents) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetSphere is b3RecWrite_ShapeSetSphere.
func recWriteShapeSetSphere(rec *Recording, a *recArgsShapeSetSphere) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetCapsule is b3RecWrite_ShapeSetCapsule.
func recWriteShapeSetCapsule(rec *Recording, a *recArgsShapeSetCapsule) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeApplyWind is b3RecWrite_ShapeApplyWind.
func recWriteShapeApplyWind(rec *Recording, a *recArgsShapeApplyWind) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetName is b3RecWrite_ShapeSetName.
func recWriteShapeSetName(rec *Recording, a *recArgsShapeSetName) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetMeshMaterial is b3RecWrite_ShapeSetMeshMaterial.
func recWriteShapeSetMeshMaterial(rec *Recording, a *recArgsShapeSetMeshMaterial) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetHull is b3RecWrite_ShapeSetHull.
func recWriteShapeSetHull(rec *Recording, a *recArgsShapeSetHull) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteShapeSetMesh is b3RecWrite_ShapeSetMesh.
func recWriteShapeSetMesh(rec *Recording, a *recArgsShapeSetMesh) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateParallelJoint is b3RecWrite_CreateParallelJoint.
func recWriteCreateParallelJoint(rec *Recording, a *recArgsCreateParallelJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateDistanceJoint is b3RecWrite_CreateDistanceJoint.
func recWriteCreateDistanceJoint(rec *Recording, a *recArgsCreateDistanceJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateFilterJoint is b3RecWrite_CreateFilterJoint.
func recWriteCreateFilterJoint(rec *Recording, a *recArgsCreateFilterJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateMotorJoint is b3RecWrite_CreateMotorJoint.
func recWriteCreateMotorJoint(rec *Recording, a *recArgsCreateMotorJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreatePrismaticJoint is b3RecWrite_CreatePrismaticJoint.
func recWriteCreatePrismaticJoint(rec *Recording, a *recArgsCreatePrismaticJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateRevoluteJoint is b3RecWrite_CreateRevoluteJoint.
func recWriteCreateRevoluteJoint(rec *Recording, a *recArgsCreateRevoluteJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateSphericalJoint is b3RecWrite_CreateSphericalJoint.
func recWriteCreateSphericalJoint(rec *Recording, a *recArgsCreateSphericalJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateWeldJoint is b3RecWrite_CreateWeldJoint.
func recWriteCreateWeldJoint(rec *Recording, a *recArgsCreateWeldJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteCreateWheelJoint is b3RecWrite_CreateWheelJoint.
func recWriteCreateWheelJoint(rec *Recording, a *recArgsCreateWheelJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDestroyJoint is b3RecWrite_DestroyJoint.
func recWriteDestroyJoint(rec *Recording, a *recArgsDestroyJoint) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteJointSetLocalFrameA is b3RecWrite_JointSetLocalFrameA.
func recWriteJointSetLocalFrameA(rec *Recording, a *recArgsJointSetLocalFrameA) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteJointSetLocalFrameB is b3RecWrite_JointSetLocalFrameB.
func recWriteJointSetLocalFrameB(rec *Recording, a *recArgsJointSetLocalFrameB) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteJointSetCollideConnected is b3RecWrite_JointSetCollideConnected.
func recWriteJointSetCollideConnected(rec *Recording, a *recArgsJointSetCollideConnected) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteJointWakeBodies is b3RecWrite_JointWakeBodies.
func recWriteJointWakeBodies(rec *Recording, a *recArgsJointWakeBodies) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteJointSetConstraintTuning is b3RecWrite_JointSetConstraintTuning.
func recWriteJointSetConstraintTuning(rec *Recording, a *recArgsJointSetConstraintTuning) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteJointSetForceThreshold is b3RecWrite_JointSetForceThreshold.
func recWriteJointSetForceThreshold(rec *Recording, a *recArgsJointSetForceThreshold) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteJointSetTorqueThreshold is b3RecWrite_JointSetTorqueThreshold.
func recWriteJointSetTorqueThreshold(rec *Recording, a *recArgsJointSetTorqueThreshold) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteParallelJointSetSpringHertz is b3RecWrite_ParallelJointSetSpringHertz.
func recWriteParallelJointSetSpringHertz(rec *Recording, a *recArgsParallelJointSetSpringHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteParallelJointSetSpringDampingRatio is b3RecWrite_ParallelJointSetSpringDampingRatio.
func recWriteParallelJointSetSpringDampingRatio(rec *Recording, a *recArgsParallelJointSetSpringDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteParallelJointSetMaxTorque is b3RecWrite_ParallelJointSetMaxTorque.
func recWriteParallelJointSetMaxTorque(rec *Recording, a *recArgsParallelJointSetMaxTorque) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointSetLength is b3RecWrite_DistanceJointSetLength.
func recWriteDistanceJointSetLength(rec *Recording, a *recArgsDistanceJointSetLength) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointEnableSpring is b3RecWrite_DistanceJointEnableSpring.
func recWriteDistanceJointEnableSpring(rec *Recording, a *recArgsDistanceJointEnableSpring) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointSetSpringForceRange is b3RecWrite_DistanceJointSetSpringForceRange.
func recWriteDistanceJointSetSpringForceRange(rec *Recording, a *recArgsDistanceJointSetSpringForceRange) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointSetSpringHertz is b3RecWrite_DistanceJointSetSpringHertz.
func recWriteDistanceJointSetSpringHertz(rec *Recording, a *recArgsDistanceJointSetSpringHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointSetSpringDampingRatio is b3RecWrite_DistanceJointSetSpringDampingRatio.
func recWriteDistanceJointSetSpringDampingRatio(rec *Recording, a *recArgsDistanceJointSetSpringDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointEnableLimit is b3RecWrite_DistanceJointEnableLimit.
func recWriteDistanceJointEnableLimit(rec *Recording, a *recArgsDistanceJointEnableLimit) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointSetLengthRange is b3RecWrite_DistanceJointSetLengthRange.
func recWriteDistanceJointSetLengthRange(rec *Recording, a *recArgsDistanceJointSetLengthRange) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointEnableMotor is b3RecWrite_DistanceJointEnableMotor.
func recWriteDistanceJointEnableMotor(rec *Recording, a *recArgsDistanceJointEnableMotor) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointSetMotorSpeed is b3RecWrite_DistanceJointSetMotorSpeed.
func recWriteDistanceJointSetMotorSpeed(rec *Recording, a *recArgsDistanceJointSetMotorSpeed) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteDistanceJointSetMaxMotorForce is b3RecWrite_DistanceJointSetMaxMotorForce.
func recWriteDistanceJointSetMaxMotorForce(rec *Recording, a *recArgsDistanceJointSetMaxMotorForce) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetLinearVelocity is b3RecWrite_MotorJointSetLinearVelocity.
func recWriteMotorJointSetLinearVelocity(rec *Recording, a *recArgsMotorJointSetLinearVelocity) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetAngularVelocity is b3RecWrite_MotorJointSetAngularVelocity.
func recWriteMotorJointSetAngularVelocity(rec *Recording, a *recArgsMotorJointSetAngularVelocity) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetMaxVelocityForce is b3RecWrite_MotorJointSetMaxVelocityForce.
func recWriteMotorJointSetMaxVelocityForce(rec *Recording, a *recArgsMotorJointSetMaxVelocityForce) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetMaxVelocityTorque is b3RecWrite_MotorJointSetMaxVelocityTorque.
func recWriteMotorJointSetMaxVelocityTorque(rec *Recording, a *recArgsMotorJointSetMaxVelocityTorque) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetLinearHertz is b3RecWrite_MotorJointSetLinearHertz.
func recWriteMotorJointSetLinearHertz(rec *Recording, a *recArgsMotorJointSetLinearHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetLinearDampingRatio is b3RecWrite_MotorJointSetLinearDampingRatio.
func recWriteMotorJointSetLinearDampingRatio(rec *Recording, a *recArgsMotorJointSetLinearDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetAngularHertz is b3RecWrite_MotorJointSetAngularHertz.
func recWriteMotorJointSetAngularHertz(rec *Recording, a *recArgsMotorJointSetAngularHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetAngularDampingRatio is b3RecWrite_MotorJointSetAngularDampingRatio.
func recWriteMotorJointSetAngularDampingRatio(rec *Recording, a *recArgsMotorJointSetAngularDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetMaxSpringForce is b3RecWrite_MotorJointSetMaxSpringForce.
func recWriteMotorJointSetMaxSpringForce(rec *Recording, a *recArgsMotorJointSetMaxSpringForce) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteMotorJointSetMaxSpringTorque is b3RecWrite_MotorJointSetMaxSpringTorque.
func recWriteMotorJointSetMaxSpringTorque(rec *Recording, a *recArgsMotorJointSetMaxSpringTorque) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWritePrismaticJointEnableSpring is b3RecWrite_PrismaticJointEnableSpring.
func recWritePrismaticJointEnableSpring(rec *Recording, a *recArgsPrismaticJointEnableSpring) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWritePrismaticJointSetSpringHertz is b3RecWrite_PrismaticJointSetSpringHertz.
func recWritePrismaticJointSetSpringHertz(rec *Recording, a *recArgsPrismaticJointSetSpringHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWritePrismaticJointSetSpringDampingRatio is b3RecWrite_PrismaticJointSetSpringDampingRatio.
func recWritePrismaticJointSetSpringDampingRatio(rec *Recording, a *recArgsPrismaticJointSetSpringDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWritePrismaticJointSetTargetTranslation is b3RecWrite_PrismaticJointSetTargetTranslation.
func recWritePrismaticJointSetTargetTranslation(rec *Recording, a *recArgsPrismaticJointSetTargetTranslation) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWritePrismaticJointEnableLimit is b3RecWrite_PrismaticJointEnableLimit.
func recWritePrismaticJointEnableLimit(rec *Recording, a *recArgsPrismaticJointEnableLimit) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWritePrismaticJointSetLimits is b3RecWrite_PrismaticJointSetLimits.
func recWritePrismaticJointSetLimits(rec *Recording, a *recArgsPrismaticJointSetLimits) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWritePrismaticJointEnableMotor is b3RecWrite_PrismaticJointEnableMotor.
func recWritePrismaticJointEnableMotor(rec *Recording, a *recArgsPrismaticJointEnableMotor) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWritePrismaticJointSetMotorSpeed is b3RecWrite_PrismaticJointSetMotorSpeed.
func recWritePrismaticJointSetMotorSpeed(rec *Recording, a *recArgsPrismaticJointSetMotorSpeed) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWritePrismaticJointSetMaxMotorForce is b3RecWrite_PrismaticJointSetMaxMotorForce.
func recWritePrismaticJointSetMaxMotorForce(rec *Recording, a *recArgsPrismaticJointSetMaxMotorForce) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRevoluteJointEnableSpring is b3RecWrite_RevoluteJointEnableSpring.
func recWriteRevoluteJointEnableSpring(rec *Recording, a *recArgsRevoluteJointEnableSpring) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRevoluteJointSetSpringHertz is b3RecWrite_RevoluteJointSetSpringHertz.
func recWriteRevoluteJointSetSpringHertz(rec *Recording, a *recArgsRevoluteJointSetSpringHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRevoluteJointSetSpringDampingRatio is b3RecWrite_RevoluteJointSetSpringDampingRatio.
func recWriteRevoluteJointSetSpringDampingRatio(rec *Recording, a *recArgsRevoluteJointSetSpringDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRevoluteJointSetTargetAngle is b3RecWrite_RevoluteJointSetTargetAngle.
func recWriteRevoluteJointSetTargetAngle(rec *Recording, a *recArgsRevoluteJointSetTargetAngle) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRevoluteJointEnableLimit is b3RecWrite_RevoluteJointEnableLimit.
func recWriteRevoluteJointEnableLimit(rec *Recording, a *recArgsRevoluteJointEnableLimit) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRevoluteJointSetLimits is b3RecWrite_RevoluteJointSetLimits.
func recWriteRevoluteJointSetLimits(rec *Recording, a *recArgsRevoluteJointSetLimits) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRevoluteJointEnableMotor is b3RecWrite_RevoluteJointEnableMotor.
func recWriteRevoluteJointEnableMotor(rec *Recording, a *recArgsRevoluteJointEnableMotor) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRevoluteJointSetMotorSpeed is b3RecWrite_RevoluteJointSetMotorSpeed.
func recWriteRevoluteJointSetMotorSpeed(rec *Recording, a *recArgsRevoluteJointSetMotorSpeed) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRevoluteJointSetMaxMotorTorque is b3RecWrite_RevoluteJointSetMaxMotorTorque.
func recWriteRevoluteJointSetMaxMotorTorque(rec *Recording, a *recArgsRevoluteJointSetMaxMotorTorque) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointEnableConeLimit is b3RecWrite_SphericalJointEnableConeLimit.
func recWriteSphericalJointEnableConeLimit(rec *Recording, a *recArgsSphericalJointEnableConeLimit) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointSetConeLimit is b3RecWrite_SphericalJointSetConeLimit.
func recWriteSphericalJointSetConeLimit(rec *Recording, a *recArgsSphericalJointSetConeLimit) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointEnableTwistLimit is b3RecWrite_SphericalJointEnableTwistLimit.
func recWriteSphericalJointEnableTwistLimit(rec *Recording, a *recArgsSphericalJointEnableTwistLimit) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointSetTwistLimits is b3RecWrite_SphericalJointSetTwistLimits.
func recWriteSphericalJointSetTwistLimits(rec *Recording, a *recArgsSphericalJointSetTwistLimits) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointEnableSpring is b3RecWrite_SphericalJointEnableSpring.
func recWriteSphericalJointEnableSpring(rec *Recording, a *recArgsSphericalJointEnableSpring) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointSetSpringHertz is b3RecWrite_SphericalJointSetSpringHertz.
func recWriteSphericalJointSetSpringHertz(rec *Recording, a *recArgsSphericalJointSetSpringHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointSetSpringDampingRatio is b3RecWrite_SphericalJointSetSpringDampingRatio.
func recWriteSphericalJointSetSpringDampingRatio(rec *Recording, a *recArgsSphericalJointSetSpringDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointSetTargetRotation is b3RecWrite_SphericalJointSetTargetRotation.
func recWriteSphericalJointSetTargetRotation(rec *Recording, a *recArgsSphericalJointSetTargetRotation) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointEnableMotor is b3RecWrite_SphericalJointEnableMotor.
func recWriteSphericalJointEnableMotor(rec *Recording, a *recArgsSphericalJointEnableMotor) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointSetMotorVelocity is b3RecWrite_SphericalJointSetMotorVelocity.
func recWriteSphericalJointSetMotorVelocity(rec *Recording, a *recArgsSphericalJointSetMotorVelocity) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteSphericalJointSetMaxMotorTorque is b3RecWrite_SphericalJointSetMaxMotorTorque.
func recWriteSphericalJointSetMaxMotorTorque(rec *Recording, a *recArgsSphericalJointSetMaxMotorTorque) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWeldJointSetLinearHertz is b3RecWrite_WeldJointSetLinearHertz.
func recWriteWeldJointSetLinearHertz(rec *Recording, a *recArgsWeldJointSetLinearHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWeldJointSetLinearDampingRatio is b3RecWrite_WeldJointSetLinearDampingRatio.
func recWriteWeldJointSetLinearDampingRatio(rec *Recording, a *recArgsWeldJointSetLinearDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWeldJointSetAngularHertz is b3RecWrite_WeldJointSetAngularHertz.
func recWriteWeldJointSetAngularHertz(rec *Recording, a *recArgsWeldJointSetAngularHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWeldJointSetAngularDampingRatio is b3RecWrite_WeldJointSetAngularDampingRatio.
func recWriteWeldJointSetAngularDampingRatio(rec *Recording, a *recArgsWeldJointSetAngularDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointEnableSuspension is b3RecWrite_WheelJointEnableSuspension.
func recWriteWheelJointEnableSuspension(rec *Recording, a *recArgsWheelJointEnableSuspension) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetSuspensionHertz is b3RecWrite_WheelJointSetSuspensionHertz.
func recWriteWheelJointSetSuspensionHertz(rec *Recording, a *recArgsWheelJointSetSuspensionHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetSuspensionDampingRatio is b3RecWrite_WheelJointSetSuspensionDampingRatio.
func recWriteWheelJointSetSuspensionDampingRatio(rec *Recording, a *recArgsWheelJointSetSuspensionDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointEnableSuspensionLimit is b3RecWrite_WheelJointEnableSuspensionLimit.
func recWriteWheelJointEnableSuspensionLimit(rec *Recording, a *recArgsWheelJointEnableSuspensionLimit) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetSuspensionLimits is b3RecWrite_WheelJointSetSuspensionLimits.
func recWriteWheelJointSetSuspensionLimits(rec *Recording, a *recArgsWheelJointSetSuspensionLimits) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointEnableSpinMotor is b3RecWrite_WheelJointEnableSpinMotor.
func recWriteWheelJointEnableSpinMotor(rec *Recording, a *recArgsWheelJointEnableSpinMotor) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetSpinMotorSpeed is b3RecWrite_WheelJointSetSpinMotorSpeed.
func recWriteWheelJointSetSpinMotorSpeed(rec *Recording, a *recArgsWheelJointSetSpinMotorSpeed) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetMaxSpinTorque is b3RecWrite_WheelJointSetMaxSpinTorque.
func recWriteWheelJointSetMaxSpinTorque(rec *Recording, a *recArgsWheelJointSetMaxSpinTorque) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointEnableSteering is b3RecWrite_WheelJointEnableSteering.
func recWriteWheelJointEnableSteering(rec *Recording, a *recArgsWheelJointEnableSteering) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetSteeringHertz is b3RecWrite_WheelJointSetSteeringHertz.
func recWriteWheelJointSetSteeringHertz(rec *Recording, a *recArgsWheelJointSetSteeringHertz) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetSteeringDampingRatio is b3RecWrite_WheelJointSetSteeringDampingRatio.
func recWriteWheelJointSetSteeringDampingRatio(rec *Recording, a *recArgsWheelJointSetSteeringDampingRatio) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetMaxSteeringTorque is b3RecWrite_WheelJointSetMaxSteeringTorque.
func recWriteWheelJointSetMaxSteeringTorque(rec *Recording, a *recArgsWheelJointSetMaxSteeringTorque) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointEnableSteeringLimit is b3RecWrite_WheelJointEnableSteeringLimit.
func recWriteWheelJointEnableSteeringLimit(rec *Recording, a *recArgsWheelJointEnableSteeringLimit) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetSteeringLimits is b3RecWrite_WheelJointSetSteeringLimits.
func recWriteWheelJointSetSteeringLimits(rec *Recording, a *recArgsWheelJointSetSteeringLimits) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteWheelJointSetTargetSteeringAngle is b3RecWrite_WheelJointSetTargetSteeringAngle.
func recWriteWheelJointSetTargetSteeringAngle(rec *Recording, a *recArgsWheelJointSetTargetSteeringAngle) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteQueryOverlapAABB is b3RecWrite_QueryOverlapAABB.
func recWriteQueryOverlapAABB(rec *Recording, a *recArgsQueryOverlapAABB) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteQueryOverlapShape is b3RecWrite_QueryOverlapShape.
func recWriteQueryOverlapShape(rec *Recording, a *recArgsQueryOverlapShape) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteQueryCastRay is b3RecWrite_QueryCastRay.
func recWriteQueryCastRay(rec *Recording, a *recArgsQueryCastRay) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteQueryCastShape is b3RecWrite_QueryCastShape.
func recWriteQueryCastShape(rec *Recording, a *recArgsQueryCastShape) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteQueryCastRayClosest is b3RecWrite_QueryCastRayClosest.
func recWriteQueryCastRayClosest(rec *Recording, a *recArgsQueryCastRayClosest) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteQueryCastMover is b3RecWrite_QueryCastMover.
func recWriteQueryCastMover(rec *Recording, a *recArgsQueryCastMover) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteQueryCollideMover is b3RecWrite_QueryCollideMover.
func recWriteQueryCollideMover(rec *Recording, a *recArgsQueryCollideMover) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteQueryTag is b3RecWrite_QueryTag.
func recWriteQueryTag(rec *Recording, a *recArgsQueryTag) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteStateHash is b3RecWrite_StateHash.
func recWriteStateHash(rec *Recording, a *recArgsStateHash) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRecordingBounds is b3RecWrite_RecordingBounds.
func recWriteRecordingBounds(rec *Recording, a *recArgsRecordingBounds) {
	// PORT: stub until milestone 5 (recording.c)
}

// Create ops: writers that also append the returned id inside the record.

// recWriteRetCreateBody is b3RecWriteRet_CreateBody.
func recWriteRetCreateBody(rec *Recording, a *recArgsCreateBody, id BodyID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateSphereShape is b3RecWriteRet_CreateSphereShape.
func recWriteRetCreateSphereShape(rec *Recording, a *recArgsCreateSphereShape, id ShapeID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateCapsuleShape is b3RecWriteRet_CreateCapsuleShape.
func recWriteRetCreateCapsuleShape(rec *Recording, a *recArgsCreateCapsuleShape, id ShapeID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateHullShape is b3RecWriteRet_CreateHullShape.
func recWriteRetCreateHullShape(rec *Recording, a *recArgsCreateHullShape, id ShapeID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateMeshShape is b3RecWriteRet_CreateMeshShape.
func recWriteRetCreateMeshShape(rec *Recording, a *recArgsCreateMeshShape, id ShapeID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateHeightFieldShape is b3RecWriteRet_CreateHeightFieldShape.
func recWriteRetCreateHeightFieldShape(rec *Recording, a *recArgsCreateHeightFieldShape, id ShapeID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateCompoundShape is b3RecWriteRet_CreateCompoundShape.
func recWriteRetCreateCompoundShape(rec *Recording, a *recArgsCreateCompoundShape, id ShapeID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateParallelJoint is b3RecWriteRet_CreateParallelJoint.
func recWriteRetCreateParallelJoint(rec *Recording, a *recArgsCreateParallelJoint, id JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateDistanceJoint is b3RecWriteRet_CreateDistanceJoint.
func recWriteRetCreateDistanceJoint(rec *Recording, a *recArgsCreateDistanceJoint, id JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateFilterJoint is b3RecWriteRet_CreateFilterJoint.
func recWriteRetCreateFilterJoint(rec *Recording, a *recArgsCreateFilterJoint, id JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateMotorJoint is b3RecWriteRet_CreateMotorJoint.
func recWriteRetCreateMotorJoint(rec *Recording, a *recArgsCreateMotorJoint, id JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreatePrismaticJoint is b3RecWriteRet_CreatePrismaticJoint.
func recWriteRetCreatePrismaticJoint(rec *Recording, a *recArgsCreatePrismaticJoint, id JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateRevoluteJoint is b3RecWriteRet_CreateRevoluteJoint.
func recWriteRetCreateRevoluteJoint(rec *Recording, a *recArgsCreateRevoluteJoint, id JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateSphericalJoint is b3RecWriteRet_CreateSphericalJoint.
func recWriteRetCreateSphericalJoint(rec *Recording, a *recArgsCreateSphericalJoint, id JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateWeldJoint is b3RecWriteRet_CreateWeldJoint.
func recWriteRetCreateWeldJoint(rec *Recording, a *recArgsCreateWeldJoint, id JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRetCreateWheelJoint is b3RecWriteRet_CreateWheelJoint.
func recWriteRetCreateWheelJoint(rec *Recording, a *recArgsCreateWheelJoint, id JointID) {
	// PORT: stub until milestone 5 (recording.c)
}

// Patch helpers for the query hit-count backfill

// recReserveU32 is b3RecReserveU32.
func recReserveU32(buf *recBuffer) int {
	// PORT: stub until milestone 5 (recording.c)
	return 0
}

// recPatchU32 is b3RecPatchU32.
func recPatchU32(buf *recBuffer, offset int, v uint32) {
	// PORT: stub until milestone 5 (recording.c)
}

// recCommitRecord is b3RecCommitRecord.
// Commit a finished query record under the lock. The payload buffer stays owned by the caller.
func recCommitRecord(rec *Recording, opcode uint8, payload []byte) {
	// PORT: stub until milestone 5 (recording.c)
}

// recQueryBegin is b3RecQueryBegin.
func recQueryBegin(w *recQueryWriter, context any, tagId uint64, tagName string) {
	// PORT: stub until milestone 5 (recording.c)
}

// recQueryCommit is b3RecQueryCommit.
func recQueryCommit(rec *Recording, opcode uint8, w *recQueryWriter) {
	// PORT: stub until milestone 5 (recording.c)
}

// Recording trampolines: replace the user fcn so hits are captured before dispatch. The overlap
// trampoline doubles for the mover filter, which has the same bool(shapeId, ctx) shape.

// recOverlapTrampoline is b3RecOverlapTrampoline.
func recOverlapTrampoline(id ShapeID, ctx any) bool {
	// PORT: stub until milestone 5 (recording.c)
	return false
}

// recCastTrampoline is b3RecCastTrampoline.
func recCastTrampoline(id ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64, triangleIndex int,
	childIndex int, ctx any) float32 {
	// PORT: stub until milestone 5 (recording.c)
	return 0.0
}

// recPlaneTrampoline is b3RecPlaneTrampoline.
// PORT: const b3PlaneResult* planes + planeCount is a slice (as PlaneResultFcn).
func recPlaneTrampoline(id ShapeID, planes []PlaneResult, ctx any) bool {
	// PORT: stub until milestone 5 (recording.c)
	return false
}

// Geometry registry

// internGeometry is b3InternGeometry.
func internGeometry(reg *geometryRegistry, kind geometryKind, contentHash uint64, bytes []byte) uint32 {
	// PORT: stub until milestone 5 (recording.c)
	return 0
}

// appendGeometry is b3AppendGeometry.
// Append an entry unconditionally and return its id, which equals its array index. Unlike
// b3InternGeometry it never deduplicates, so the keyframe seed can mirror slots 1:1 even when an
// already-recorded file carries byte-identical duplicate slots (a hash collision wrote them apart).
func appendGeometry(reg *geometryRegistry, kind geometryKind, contentHash uint64, bytes []byte) uint32 {
	// PORT: stub until milestone 5 (recording.c)
	return 0
}

// freeRegistry is b3FreeRegistry.
func freeRegistry(reg *geometryRegistry) {
	// PORT: stub until milestone 5 (recording.c)
}

// recWriteRegistry is b3RecWriteRegistry.
func recWriteRegistry(rec *Recording) {
	// PORT: stub until milestone 5 (recording.c)
}

// hashQueryTag is b3HashQueryTag.
// Hash a query (id, name) pair into the stable key the viewer tracks the query by. Never returns 0,
// so the key doubles as a tagged/untagged flag.
func hashQueryTag(id uint64, name string) uint64 {
	// PORT: stub until milestone 5 (recording.c)
	return 0
}

// recInternTag is b3RecInternTag.
// Record a key->(id, name) mapping once, deduped by key (a repeated key keeps its first id/name).
func recInternTag(rec *Recording, key uint64, id uint64, name string) {
	// PORT: stub until milestone 5 (recording.c)
}

// Intern each large geometry kind and return a stable u32 id for use in create ops.

// recInternHull is b3RecInternHull.
func recInternHull(rec *Recording, hull *HullData) uint32 {
	// PORT: stub until milestone 5 (recording.c)
	return 0
}

// recInternMesh is b3RecInternMesh.
func recInternMesh(rec *Recording, mesh *MeshData) uint32 {
	// PORT: stub until milestone 5 (recording.c)
	return 0
}

// recInternHeightField is b3RecInternHeightField.
func recInternHeightField(rec *Recording, hf *HeightFieldData) uint32 {
	// PORT: stub until milestone 5 (recording.c)
	return 0
}

// recInternCompound is b3RecInternCompound.
func recInternCompound(rec *Recording, compound *CompoundData) uint32 {
	// PORT: stub until milestone 5 (recording.c)
	return 0
}

// Lifecycle engine-side hooks

// startRecordingIntoBuffer is b3StartRecordingIntoBuffer.
// PORT: the stub leaves world.recording nil, so recording stays off until milestone 5.
func startRecordingIntoBuffer(world *world, recording *Recording) {
	// PORT: stub until milestone 5 (recording.c)
}

// stopRecordingInternal is b3StopRecordingInternal.
func stopRecordingInternal(world *world) {
	// PORT: stub until milestone 5 (recording.c)
}

// recAccumulateBounds is b3RecAccumulateBounds.
// Fold one step's world bounds into the running union.
func recAccumulateBounds(rec *Recording, bounds AABB) {
	// PORT: stub until milestone 5 (recording.c)
}

// hashWorldState is b3HashWorldState.
// Deterministic hash over all body transforms and velocities.
// Called by both recorder and replayer to verify simulation reproduces exactly.
// PORT: pulled forward from recording.c (milestone 5) at the milestone 3 integration; the world
// tests compare it across worker counts. B3_HASH_FLOAT is written out with math.Float32bits.
func hashWorldState(world *world) uint64 {
	hash := snapFnvInit
	prime := snapFnvPrime

	bodyCount := len(world.bodies)
	for i := 0; i < bodyCount; i++ {
		body := &world.bodies[i]
		if int(body.id) != i {
			// Free or never-used slot
			continue
		}

		sim := getBodySim(world, body)

		hash = fnvMixPosition(hash, sim.transform.P)
		hash = (hash ^ uint64(math.Float32bits(sim.transform.Q.V.X))) * prime
		hash = (hash ^ uint64(math.Float32bits(sim.transform.Q.V.Y))) * prime
		hash = (hash ^ uint64(math.Float32bits(sim.transform.Q.V.Z))) * prime
		hash = (hash ^ uint64(math.Float32bits(sim.transform.Q.S))) * prime

		state := getBodyState(world, body)
		if state != nil {
			hash = (hash ^ uint64(math.Float32bits(state.linearVelocity.X))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.linearVelocity.Y))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.linearVelocity.Z))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.angularVelocity.X))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.angularVelocity.Y))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.angularVelocity.Z))) * prime
		}
	}

	return hash
}
