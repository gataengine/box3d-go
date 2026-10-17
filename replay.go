// Port of src/replay.h and src/replay.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"unsafe"
)

// PORT: replay.c is split in three files in upstream order: replay.go holds replay.h and the
// read primitives, def readers and id retargeting; replay_ops.go the per-op dispatch functions,
// the spatial query replay and the dispatch switch; replay_player.go the public entry point and
// the b3RecPlayer implementation. printf diagnostics are fmt.Printf with the same text.

// ---- replay.h ----

// recRecordedHit is b3RecRecordedHit.
// A single recorded callback hit, used both as reader scratch during a query replay and as the
// per-frame draw store. For collide-mover, one hit is one plane, with planeCount and userReturnB
// replicated across a shape's planes so the replay walker can re-group and compare per shape.
type recRecordedHit struct {
	id             ShapeID
	point          Pos
	normal         Vec3
	fraction       float32
	userMaterialId uint64
	triangleIndex  int32
	childIndex     int32
	plane          PlaneResult // collide-mover: this plane
	planeCount     int32       // collide-mover: planes in this hit's shape group (replicated)
	userReturnF    float32     // cast queries
	userReturnB    bool        // overlap / collide-mover (per shape, replicated)
}

// Recorded query kind, matching the public b3RecQueryType order (b3RecQueryKind).
const (
	recqOverlapAABB    = iota // B3_RECQ_OVERLAP_AABB
	recqOverlapShape          // B3_RECQ_OVERLAP_SHAPE
	recqCastRay               // B3_RECQ_CAST_RAY
	recqCastShape             // B3_RECQ_CAST_SHAPE
	recqCastRayClosest        // B3_RECQ_CAST_RAY_CLOSEST
	recqCastMover             // B3_RECQ_CAST_MOVER
	recqCollideMover          // B3_RECQ_COLLIDE_MOVER
)

// recDrawQuery is b3RecDrawQuery.
// Per-frame draw record for one query call. Self-contained (no aliased pointers) so the player's
// frameQueries array can grow with a plain memcpy. Geometry is origin relative except the overlap
// AABB, which is world space in 3D.
type recDrawQuery struct {
	kind         int32
	key          uint64 // identity key (hash of caller id+name), 0 = untagged
	filter       QueryFilter
	aabb         AABB                     // world-space bounds of the query, swept for casts
	proxyPoints  [MaxShapeCastPoints]Vec3 // overlap/cast shape proxy, origin relative
	proxyCount   int32
	proxyRadius  float32
	mover        Capsule // cast/collide mover, origin relative
	origin       Pos
	translation  Vec3
	castFraction float32   // cast-mover result fraction
	rayResult    RayResult // cast-ray-closest result
	shape        ShapeID
	hitStart     int32 // first hit in the player's frameHits store
	hitCount     int32
}

// registrySlot is b3RegistrySlot.
// One slot in the preloaded geometry registry. Loaded from the trailing block before any
// ops run.
// PORT: uint8_t* bytes is a slice viewing an 8-byte aligned blob (blobAlloc) so the geometry
// headers can be read in place; byteCount is kept.
type registrySlot struct {
	kind      geometryKind
	byteCount int32
	bytes     []byte // raw bytes from the file (always freed at teardown)
}

// This is used to simplify the scratch buffer lifetime. Names longer than this probably
// indicate a bug.
const maxNameLength = 256 // B3_MAX_NAME_LENGTH

// recReader is b3RecReader.
// Reader state threaded through the replay loop and all dispatch functions
// PORT: const uint8_t* data is a slice with size kept. The scratch arrays are slices whose length
// is the capacity (matScratchCap, hitCap and proxyScratchCap are dropped). The rotating
// stringBuffers are dropped because Go strings are immutable values. b3RecTag* tags is a slice
// with tagCount and tagCapacity kept; void* tagMap is a map[uint64]uint32 (lookup only, never
// iterated, so the order of verstable does not matter).
type recReader struct {
	data          []byte
	size          int
	cursor        int
	replayWorldId WorldID
	ok            bool // false on read overrun or id mismatch, fatal stop
	diverged      bool // a StateHash failed, non-fatal

	// Player that owns this reader, or NULL during a headless b3ValidateReplay. Body
	// create/destroy and the bounds record fold back into it for the outliner and camera framing.
	owner *RecPlayer

	// Scratch for per-triangle materials in shape defs (grown on demand, freed at teardown)
	matScratch []SurfaceMaterial

	// Preloaded geometry registry
	slots     []registrySlot
	slotCount int

	// Preloaded query-tag table (key -> id, name), loaded with the registry. Resolves the caller id and
	// label for the viewer. tagMap maps a key to its tag index for O(1) lookup; opaque, owned by the player.
	tags        []recTag
	tagCount    int // tags that loaded; a truncated tail loads fewer
	tagCapacity int // tags allocated, used to free the array
	tagMap      map[uint64]uint32

	// Key from the QueryTag op preceding the next query, consumed by the next stash. 0 = untagged.
	pendingQueryKey uint64

	// Scratch for recorded query hits; grown on demand, freed with the player.
	hits []recRecordedHit

	// Scratch for a shape-proxy point cloud read from the stream. b3ShapeProxy holds the points
	// behind a pointer, so a decoded proxy borrows this until the next proxy read or teardown.
	proxyScratch []Vec3
}

// recKeyframe is b3RecKeyframe.
// Stored snapshot for fast backward seek.
// PORT: uint8_t* image is a slice with imageSize and imageCapacity kept; b3BodyId* bodyIds is a
// slice with bodyIdCount kept.
type recKeyframe struct {
	image         []byte // serialized world image at the end of this frame
	imageSize     int
	imageCapacity int  // allocation size (may exceed imageSize)
	frame         int  // frame index this restores to
	cursor        int  // op-stream cursor for the frame AFTER this one
	divergeFrame  int  // divergeFrame state at capture
	diverged      bool // rdr.diverged state at capture

	// Outliner body list as it stood at this frame, restored verbatim so ordinals are stable.
	bodyIds     []BodyID
	bodyIdCount int
}

// RecPlayer is b3RecPlayer.
// PORT: exported because the public API (box3d.h) hands it out (b3CreatePlayer); all fields are
// unexported. Pointer + count + capacity arrays are slices: the capacity fields are dropped where
// the slice length serves (bodyIdCap, frameQueryCap, frameHitCap, keyframeCapacity) and the
// counts are kept. size_t is uint64.
type RecPlayer struct {
	data                 []byte // owned copy of recording bytes
	size                 int
	headerEnd            int // first byte of op stream (past header + snapshot blob)
	registryEnd          int // end of op stream = start of registry block (or size)
	lengthScale          float32
	previousLengthScale  float32
	frame                int
	frameCount           int
	recordedDt           float32
	recordedSubStepCount int
	recordedWorkerCount  int  // worker count requested for the replay world
	bounds               AABB // accumulated world bounds, decoded from the trailing record
	atEnd                bool
	// Indicates all ops for the step have been consumed up to the world step op. The next sub-step
	// will clear this and perform the world step.
	atPreStep    bool
	divergeFrame int // first frame that diverged, -1 until then

	// Outliner body list, indexed by creation ordinal. Holes (null ids) mark destroyed bodies so
	// later ordinals never shift. Snapshotted into each keyframe and the frame-0 copy, not rebuilt
	// from the world, so a stored selection survives backward seeks.
	bodyIds           []BodyID
	bodyIdCount       int
	frame0BodyIds     []BodyID
	frame0BodyIdCount int

	// Per-frame query store, reset at the top of each StepFrame and filled by the query dispatchers.
	// Drawn by b3RecPlayer_DrawFrameQueries and inspected via the public GetFrameQuery API.
	frameQueries    []recDrawQuery
	frameQueryCount int
	frameHits       []recRecordedHit
	frameHitCount   int

	// Host debug-shape callbacks applied to every world the player creates. The 3D
	// sample renderer builds GPU meshes here, so a replay world without them draws
	// nothing. Set once via b3RecPlayer_SetDebugShapeCallbacks; persisted so a world
	// rebuilt under new callbacks keeps drawing.
	createDebugShape  CreateDebugShapeCallback
	destroyDebugShape DestroyDebugShapeCallback
	debugShapeContext any

	rdr recReader

	// Frame-0 restore image, points into the owned data copy. Restart and backward seek
	// deserialize this in place so the replay world id stays stable.
	frame0Image []byte
	frame0Size  int

	// Keyframe ring
	keyframes           []recKeyframe
	keyframeCount       int
	keyframeBudget      uint64
	keyframeBytes       uint64
	keyframeMinInterval int
	keyframeInterval    int
	lastKeyframeFrame   int

	// Pre-populated recording used by b3SerializeWorld during keyframe capture.
	// Its registry mirrors rdr.slots so geometry ids stay stable.
	keyframeRec *Recording
}

// ---- replay.c ----

// Read primitives

// recRdrCheck is b3RecRdrCheck.
func recRdrCheck(rdr *recReader, size int) {
	if size < 0 || int64(rdr.cursor)+int64(size) > int64(rdr.size) {
		rdr.ok = false
	}
}

// recRdrBlob is b3RecRdrBlob.
// PORT: returns the size bytes read (zeros on overrun) instead of copying into an out pointer;
// the callers decode the C struct image field by field.
func recRdrBlob(rdr *recReader, size int) []byte {
	recRdrCheck(rdr, size)
	if !rdr.ok {
		return make([]byte, size)
	}
	out := rdr.data[rdr.cursor : rdr.cursor+size]
	rdr.cursor += size
	return out
}

// recRU8 is b3RecR_U8.
func recRU8(rdr *recReader) uint8 {
	recRdrCheck(rdr, 1)
	if !rdr.ok {
		return 0
	}
	v := rdr.data[rdr.cursor]
	rdr.cursor++
	return v
}

// recRU16 is b3RecR_U16.
func recRU16(rdr *recReader) uint16 {
	recRdrCheck(rdr, 2)
	if !rdr.ok {
		return 0
	}
	v := uint16(rdr.data[rdr.cursor]) | (uint16(rdr.data[rdr.cursor+1]) << 8)
	rdr.cursor += 2
	return v
}

// recRU24 is b3RecR_U24.
func recRU24(rdr *recReader) uint32 {
	recRdrCheck(rdr, 3)
	if !rdr.ok {
		return 0
	}
	v := uint32(rdr.data[rdr.cursor]) | (uint32(rdr.data[rdr.cursor+1]) << 8) |
		(uint32(rdr.data[rdr.cursor+2]) << 16)
	rdr.cursor += 3
	return v
}

// recRU32 is b3RecR_U32.
func recRU32(rdr *recReader) uint32 {
	recRdrCheck(rdr, 4)
	if !rdr.ok {
		return 0
	}
	v := uint32(rdr.data[rdr.cursor]) | (uint32(rdr.data[rdr.cursor+1]) << 8) |
		(uint32(rdr.data[rdr.cursor+2]) << 16) | (uint32(rdr.data[rdr.cursor+3]) << 24)
	rdr.cursor += 4
	return v
}

// recRU64 is b3RecR_U64.
func recRU64(rdr *recReader) uint64 {
	recRdrCheck(rdr, 8)
	if !rdr.ok {
		return 0
	}
	v := uint64(rdr.data[rdr.cursor]) | (uint64(rdr.data[rdr.cursor+1]) << 8) |
		(uint64(rdr.data[rdr.cursor+2]) << 16) | (uint64(rdr.data[rdr.cursor+3]) << 24) |
		(uint64(rdr.data[rdr.cursor+4]) << 32) | (uint64(rdr.data[rdr.cursor+5]) << 40) |
		(uint64(rdr.data[rdr.cursor+6]) << 48) | (uint64(rdr.data[rdr.cursor+7]) << 56)
	rdr.cursor += 8
	return v
}

// recRI32 is b3RecR_I32.
func recRI32(rdr *recReader) int32 {
	return int32(recRU32(rdr))
}

// recRF32 is b3RecR_F32.
func recRF32(rdr *recReader) float32 {
	bits := recRU32(rdr)
	return math.Float32frombits(bits)
}

// recRF64 is b3RecR_F64.
func recRF64(rdr *recReader) float64 {
	bits := recRU64(rdr)
	return math.Float64frombits(bits)
}

// recRBOOL is b3RecR_BOOL.
func recRBOOL(rdr *recReader) bool {
	return recRU8(rdr) != 0
}

// recRVEC3 is b3RecR_VEC3.
func recRVEC3(rdr *recReader) Vec3 {
	var v Vec3
	v.X = recRF32(rdr)
	v.Y = recRF32(rdr)
	v.Z = recRF32(rdr)
	return v
}

// recRQUAT is b3RecR_QUAT.
func recRQUAT(rdr *recReader) Quat {
	var q Quat
	q.V.X = recRF32(rdr)
	q.V.Y = recRF32(rdr)
	q.V.Z = recRF32(rdr)
	q.S = recRF32(rdr)
	return q
}

// recRTRANSFORM is b3RecR_TRANSFORM.
func recRTRANSFORM(rdr *recReader) Transform {
	var t Transform
	t.P = recRVEC3(rdr)
	t.Q = recRQUAT(rdr)
	return t
}

// recRPOSITION is b3RecR_POSITION.
// PORT: float build only (BOX3D_DOUBLE_PRECISION is not ported).
func recRPOSITION(rdr *recReader) Pos {
	var p Pos
	p.X = recRF32(rdr)
	p.Y = recRF32(rdr)
	p.Z = recRF32(rdr)
	return p
}

// recRWORLDXF is b3RecR_WORLDXF.
func recRWORLDXF(rdr *recReader) WorldTransform {
	var t WorldTransform
	t.P = recRPOSITION(rdr)
	t.Q = recRQUAT(rdr)
	return t
}

// recRMATRIX3 is b3RecR_MATRIX3.
func recRMATRIX3(rdr *recReader) Matrix3 {
	var m Matrix3
	m.Cx = recRVEC3(rdr)
	m.Cy = recRVEC3(rdr)
	m.Cz = recRVEC3(rdr)
	return m
}

// recRAABB is b3RecR_AABB.
func recRAABB(rdr *recReader) AABB {
	var v AABB
	v.LowerBound = recRVEC3(rdr)
	v.UpperBound = recRVEC3(rdr)
	return v
}

// recRQUERYFILTER is b3RecR_QUERYFILTER.
func recRQUERYFILTER(rdr *recReader) QueryFilter {
	// id and name are not on the wire here, they ride the separate QueryTag op. Start from the default
	// so they keep the untagged sentinel instead of garbage.
	f := DefaultQueryFilter()
	f.CategoryBits = recRU64(rdr)
	f.MaskBits = recRU64(rdr)
	return f
}

// recReserveScratch is b3RecReserveScratch.
// Reserve reader scratch for a count taken from an untrusted file. Every recorded element
// consumes at least one byte, so a valid count can never exceed the bytes left in the file.
// Reject anything larger (or negative, or that would overflow the byte size) by failing the read
// rather than allocating wildly. A grow keeps the old contents so callers can accumulate across
// reserves, as the collide-mover dispatcher does one shape group at a time.
// PORT: generic over the scratch slice; its length is the capacity and elemSize is the Go size.
func recReserveScratch[T any](rdr *recReader, data *[]T, need int) bool {
	var zero T
	elemSize := int(unsafe.Sizeof(zero))
	remaining := rdr.size - rdr.cursor
	if need < 0 || remaining < 0 || need > remaining || need > math.MaxInt32/elemSize {
		rdr.ok = false
		return false
	}
	if need <= len(*data) {
		return true
	}
	newCap := need
	if need <= math.MaxInt32/elemSize-8 {
		newCap = need + 8
	}
	grown := make([]T, newCap)
	copy(grown, *data)
	*data = grown
	return true
}

// recRSHAPEPROXY is b3RecR_SHAPEPROXY.
// Variable length, mirrors b3RecW_SHAPEPROXY: count, count points, radius. The decoded proxy borrows
// the reader's scratch for its point cloud, valid until the next proxy read.
func recRSHAPEPROXY(rdr *recReader) ShapeProxy {
	var p ShapeProxy
	count := int(recRI32(rdr))
	if count < 0 {
		count = 0
	}
	if count > MaxShapeCastPoints {
		count = MaxShapeCastPoints
	}
	if count > 0 && recReserveScratch(rdr, &rdr.proxyScratch, count) {
		for i := 0; i < count; i++ {
			rdr.proxyScratch[i] = recRVEC3(rdr)
		}
		p.Points = rdr.proxyScratch[:count]
		p.Count = int32(count)
	}
	p.Radius = recRF32(rdr)
	return p
}

// recRTREESTATS is b3RecR_TREESTATS.
func recRTREESTATS(rdr *recReader) TreeStats {
	var v TreeStats
	v.NodeVisits = recRI32(rdr)
	v.LeafVisits = recRI32(rdr)
	return v
}

// recRRAYRESULT is b3RecR_RAYRESULT.
func recRRAYRESULT(rdr *recReader) RayResult {
	var v RayResult
	// shapeId keeps the recorded world0; b3RecMakeShapeId is applied at compare time
	v.ShapeId = recRSHAPEID(rdr)
	v.Point = recRPOSITION(rdr)
	v.Normal = recRVEC3(rdr)
	v.UserMaterialId = recRU64(rdr)
	v.Fraction = recRF32(rdr)
	v.TriangleIndex = recRI32(rdr)
	v.ChildIndex = recRI32(rdr)
	v.Hit = recRBOOL(rdr)
	return v
}

// recRPLANERESULT is b3RecR_PLANERESULT.
func recRPLANERESULT(rdr *recReader) PlaneResult {
	var v PlaneResult
	v.Plane.Normal = recRVEC3(rdr)
	v.Plane.Offset = recRF32(rdr)
	v.Point = recRVEC3(rdr)
	v.TriangleIndex = recRI32(rdr)
	v.ChildIndex = recRI32(rdr)
	v.MaterialIndex = recRI32(rdr)
	return v
}

// recRWORLDID is b3RecR_WORLDID.
func recRWORLDID(rdr *recReader) WorldID {
	return LoadWorldID(recRU32(rdr))
}

// recRBODYID is b3RecR_BODYID.
func recRBODYID(rdr *recReader) BodyID {
	return LoadBodyID(recRU64(rdr))
}

// recRSHAPEID is b3RecR_SHAPEID.
func recRSHAPEID(rdr *recReader) ShapeID {
	return LoadShapeID(recRU64(rdr))
}

// recRJOINTID is b3RecR_JOINTID.
func recRJOINTID(rdr *recReader) JointID {
	return LoadJointID(recRU64(rdr))
}

// recBlobF32 decodes the little-endian float at byte offset i of a struct image.
// PORT: helper for the b3RecRdrBlob struct reads (memcpy of b3Sphere / b3Capsule).
func recBlobF32(b []byte, i int) float32 {
	return math.Float32frombits(uint32(b[i]) | uint32(b[i+1])<<8 | uint32(b[i+2])<<16 | uint32(b[i+3])<<24)
}

// recRSPHERE is b3RecR_SPHERE.
// PORT: the memcpy of the 16 byte b3Sphere image (center xyz, radius; no padding) is decoded field
// by field.
func recRSPHERE(rdr *recReader) Sphere {
	var s Sphere
	b := recRdrBlob(rdr, 16)
	s.Center = Vec3{recBlobF32(b, 0), recBlobF32(b, 4), recBlobF32(b, 8)}
	s.Radius = recBlobF32(b, 12)
	return s
}

// recRCAPSULE is b3RecR_CAPSULE.
// PORT: the memcpy of the 28 byte b3Capsule image (center1 xyz, center2 xyz, radius; no padding)
// is decoded field by field.
func recRCAPSULE(rdr *recReader) Capsule {
	var c Capsule
	b := recRdrBlob(rdr, 28)
	c.Center1 = Vec3{recBlobF32(b, 0), recBlobF32(b, 4), recBlobF32(b, 8)}
	c.Center2 = Vec3{recBlobF32(b, 12), recBlobF32(b, 16), recBlobF32(b, 20)}
	c.Radius = recBlobF32(b, 24)
	return c
}

// recRGEOMID is b3RecR_GEOMID.
func recRGEOMID(rdr *recReader) uint32 {
	return recRU32(rdr)
}

// recRFILTER is b3RecR_FILTER.
func recRFILTER(rdr *recReader) Filter {
	var f Filter
	f.CategoryBits = recRU64(rdr)
	f.MaskBits = recRU64(rdr)
	f.GroupIndex = recRI32(rdr)
	return f
}

// recRMATERIAL is b3RecR_MATERIAL.
func recRMATERIAL(rdr *recReader) SurfaceMaterial {
	m := DefaultSurfaceMaterial()
	m.Friction = recRF32(rdr)
	m.Restitution = recRF32(rdr)
	m.RollingResistance = recRF32(rdr)
	m.TangentVelocity = recRVEC3(rdr)
	m.UserMaterialId = recRU64(rdr)
	m.CustomColor = recRU32(rdr)
	return m
}

// recRMASSDATA is b3RecR_MASSDATA.
func recRMASSDATA(rdr *recReader) MassData {
	var md MassData
	md.Mass = recRF32(rdr)
	md.Center = recRVEC3(rdr)
	md.Inertia = recRMATRIX3(rdr)
	return md
}

// recRLOCKS is b3RecR_LOCKS.
func recRLOCKS(rdr *recReader) MotionLocks {
	var locks MotionLocks
	locks.LinearX = recRBOOL(rdr)
	locks.LinearY = recRBOOL(rdr)
	locks.LinearZ = recRBOOL(rdr)
	locks.AngularX = recRBOOL(rdr)
	locks.AngularY = recRBOOL(rdr)
	locks.AngularZ = recRBOOL(rdr)
	return locks
}

// recRSTR is b3RecR_STR.
// Rotating set of static string buffers, valid until the next 4 STR reads.
// PORT: returns a Go string, so the rotating buffers are not needed; NULL (length 0xFFFF) is the
// empty string. On a read overrun the cursor still advances by len as upstream, and the result is
// empty (upstream leaves the buffer's previous bytes; the op is not dispatched either way).
func recRSTR(rdr *recReader) string {
	length := recRU16(rdr)
	if length == 0xFFFF {
		return ""
	}

	n := int(length)
	if n > maxNameLength {
		n = maxNameLength
	}
	recRdrCheck(rdr, int(length))
	s := ""
	if rdr.ok && n > 0 {
		s = string(rdr.data[rdr.cursor : rdr.cursor+n])
	}
	rdr.cursor += int(length)
	// PORT: the C string ends at the first zero byte.
	for i := 0; i < len(s); i++ {
		if s[i] == 0 {
			s = s[:i]
			break
		}
	}
	return s
}

// Def readers: start from b3Default*Def() then overlay each serialized field
// in the exact order the writer produced them.

// recREXPLOSIONDEF is b3RecR_EXPLOSIONDEF.
func recREXPLOSIONDEF(rdr *recReader) ExplosionDef {
	def := DefaultExplosionDef()
	def.MaskBits = recRU64(rdr)
	def.Position = recRPOSITION(rdr)
	def.Radius = recRF32(rdr)
	def.Falloff = recRF32(rdr)
	def.ImpulsePerArea = recRF32(rdr)
	return def
}

// recRBODYDEF is b3RecR_BODYDEF.
func recRBODYDEF(rdr *recReader) BodyDef {
	def := DefaultBodyDef()
	def.Type = BodyType(recRI32(rdr))
	def.Position = recRPOSITION(rdr)
	def.Rotation = recRQUAT(rdr)
	def.LinearVelocity = recRVEC3(rdr)
	def.AngularVelocity = recRVEC3(rdr)
	def.LinearDamping = recRF32(rdr)
	def.AngularDamping = recRF32(rdr)
	def.GravityScale = recRF32(rdr)
	def.SleepThreshold = recRF32(rdr)
	def.SafetyFactor = recRF32(rdr)
	def.Name = recRSTR(rdr)
	_ = recRU64(rdr) // userData placeholder
	def.MotionLocks = recRLOCKS(rdr)
	def.EnableSleep = recRBOOL(rdr)
	def.IsAwake = recRBOOL(rdr)
	def.IsBullet = recRBOOL(rdr)
	def.IsEnabled = recRBOOL(rdr)
	def.AllowFastRotation = recRBOOL(rdr)
	def.EnableContactRecycling = recRBOOL(rdr)
	def.UserData = nil
	return def
}

// recRSHAPEDEF is b3RecR_SHAPEDEF.
func recRSHAPEDEF(rdr *recReader) ShapeDef {
	def := DefaultShapeDef()

	def.Name = recRSTR(rdr)
	_ = recRU64(rdr) // userData placeholder

	matCount := int(recRI32(rdr))
	if matCount < 0 {
		matCount = 0
	}
	if matCount > 0 && recReserveScratch(rdr, &rdr.matScratch, matCount) {
		for i := 0; i < matCount; i++ {
			rdr.matScratch[i] = recRMATERIAL(rdr)
		}
		def.Materials = rdr.matScratch[:matCount]
		def.MaterialCount = int32(matCount)
	} else {
		for i := 0; i < matCount; i++ {
			_ = recRMATERIAL(rdr)
		}
		def.Materials = nil
		def.MaterialCount = 0
	}

	def.BaseMaterial = recRMATERIAL(rdr)
	def.Density = recRF32(rdr)
	def.ExplosionScale = recRF32(rdr)
	def.Filter = recRFILTER(rdr)
	def.EnableCustomFiltering = recRBOOL(rdr)
	def.IsSensor = recRBOOL(rdr)
	def.EnableSensorEvents = recRBOOL(rdr)
	def.EnableContactEvents = recRBOOL(rdr)
	def.EnableHitEvents = recRBOOL(rdr)
	def.EnablePreSolveEvents = recRBOOL(rdr)
	def.InvokeContactCreation = recRBOOL(rdr)
	def.UpdateBodyMass = recRBOOL(rdr)
	def.EnableSpeculativeContact = recRBOOL(rdr)
	def.UserData = nil
	return def
}

// recRJointBase is b3RecR_JointBase.
// Shared base for all joint defs. Body ids come in with recorded world0; callers remap them.
func recRJointBase(rdr *recReader, base *JointDef) {
	_ = recRU64(rdr) // userData
	base.BodyIdA = recRBODYID(rdr)
	base.BodyIdB = recRBODYID(rdr)
	base.LocalFrameA = recRTRANSFORM(rdr)
	base.LocalFrameB = recRTRANSFORM(rdr)
	base.ForceThreshold = recRF32(rdr)
	base.TorqueThreshold = recRF32(rdr)
	base.ConstraintHertz = recRF32(rdr)
	base.ConstraintDampingRatio = recRF32(rdr)
	base.DrawScale = recRF32(rdr)
	base.CollideConnected = recRBOOL(rdr)
	base.UserData = nil
}

// recRPARALLELJOINTDEF is b3RecR_PARALLELJOINTDEF.
func recRPARALLELJOINTDEF(rdr *recReader) ParallelJointDef {
	def := DefaultParallelJointDef()
	recRJointBase(rdr, &def.Base)
	def.Hertz = recRF32(rdr)
	def.DampingRatio = recRF32(rdr)
	def.MaxTorque = recRF32(rdr)
	return def
}

// recRDISTANCEJOINTDEF is b3RecR_DISTANCEJOINTDEF.
func recRDISTANCEJOINTDEF(rdr *recReader) DistanceJointDef {
	def := DefaultDistanceJointDef()
	recRJointBase(rdr, &def.Base)
	def.Length = recRF32(rdr)
	def.EnableSpring = recRBOOL(rdr)
	def.LowerSpringForce = recRF32(rdr)
	def.UpperSpringForce = recRF32(rdr)
	def.Hertz = recRF32(rdr)
	def.DampingRatio = recRF32(rdr)
	def.EnableLimit = recRBOOL(rdr)
	def.MinLength = recRF32(rdr)
	def.MaxLength = recRF32(rdr)
	def.EnableMotor = recRBOOL(rdr)
	def.MaxMotorForce = recRF32(rdr)
	def.MotorSpeed = recRF32(rdr)
	return def
}

// recRFILTERJOINTDEF is b3RecR_FILTERJOINTDEF.
func recRFILTERJOINTDEF(rdr *recReader) FilterJointDef {
	def := DefaultFilterJointDef()
	recRJointBase(rdr, &def.Base)
	return def
}

// recRMOTORJOINTDEF is b3RecR_MOTORJOINTDEF.
func recRMOTORJOINTDEF(rdr *recReader) MotorJointDef {
	def := DefaultMotorJointDef()
	recRJointBase(rdr, &def.Base)
	def.LinearVelocity = recRVEC3(rdr)
	def.MaxVelocityForce = recRF32(rdr)
	def.AngularVelocity = recRVEC3(rdr)
	def.MaxVelocityTorque = recRF32(rdr)
	def.LinearHertz = recRF32(rdr)
	def.LinearDampingRatio = recRF32(rdr)
	def.MaxSpringForce = recRF32(rdr)
	def.AngularHertz = recRF32(rdr)
	def.AngularDampingRatio = recRF32(rdr)
	def.MaxSpringTorque = recRF32(rdr)
	return def
}

// recRPRISMATICJOINTDEF is b3RecR_PRISMATICJOINTDEF.
func recRPRISMATICJOINTDEF(rdr *recReader) PrismaticJointDef {
	def := DefaultPrismaticJointDef()
	recRJointBase(rdr, &def.Base)
	def.EnableSpring = recRBOOL(rdr)
	def.Hertz = recRF32(rdr)
	def.DampingRatio = recRF32(rdr)
	def.TargetTranslation = recRF32(rdr)
	def.EnableLimit = recRBOOL(rdr)
	def.LowerTranslation = recRF32(rdr)
	def.UpperTranslation = recRF32(rdr)
	def.EnableMotor = recRBOOL(rdr)
	def.MaxMotorForce = recRF32(rdr)
	def.MotorSpeed = recRF32(rdr)
	return def
}

// recRREVOLUTEJOINTDEF is b3RecR_REVOLUTEJOINTDEF.
func recRREVOLUTEJOINTDEF(rdr *recReader) RevoluteJointDef {
	def := DefaultRevoluteJointDef()
	recRJointBase(rdr, &def.Base)
	def.TargetAngle = recRF32(rdr)
	def.EnableSpring = recRBOOL(rdr)
	def.Hertz = recRF32(rdr)
	def.DampingRatio = recRF32(rdr)
	def.EnableLimit = recRBOOL(rdr)
	def.LowerAngle = recRF32(rdr)
	def.UpperAngle = recRF32(rdr)
	def.EnableMotor = recRBOOL(rdr)
	def.MaxMotorTorque = recRF32(rdr)
	def.MotorSpeed = recRF32(rdr)
	return def
}

// recRSPHERICALJOINTDEF is b3RecR_SPHERICALJOINTDEF.
func recRSPHERICALJOINTDEF(rdr *recReader) SphericalJointDef {
	def := DefaultSphericalJointDef()
	recRJointBase(rdr, &def.Base)
	def.EnableSpring = recRBOOL(rdr)
	def.Hertz = recRF32(rdr)
	def.DampingRatio = recRF32(rdr)
	def.TargetRotation = recRQUAT(rdr)
	def.EnableConeLimit = recRBOOL(rdr)
	def.ConeAngle = recRF32(rdr)
	def.EnableTwistLimit = recRBOOL(rdr)
	def.LowerTwistAngle = recRF32(rdr)
	def.UpperTwistAngle = recRF32(rdr)
	def.EnableMotor = recRBOOL(rdr)
	def.MaxMotorTorque = recRF32(rdr)
	def.MotorVelocity = recRVEC3(rdr)
	return def
}

// recRWELDJOINTDEF is b3RecR_WELDJOINTDEF.
func recRWELDJOINTDEF(rdr *recReader) WeldJointDef {
	def := DefaultWeldJointDef()
	recRJointBase(rdr, &def.Base)
	def.LinearHertz = recRF32(rdr)
	def.AngularHertz = recRF32(rdr)
	def.LinearDampingRatio = recRF32(rdr)
	def.AngularDampingRatio = recRF32(rdr)
	return def
}

// recRWHEELJOINTDEF is b3RecR_WHEELJOINTDEF.
func recRWHEELJOINTDEF(rdr *recReader) WheelJointDef {
	def := DefaultWheelJointDef()
	recRJointBase(rdr, &def.Base)
	def.EnableSuspensionSpring = recRBOOL(rdr)
	def.SuspensionHertz = recRF32(rdr)
	def.SuspensionDampingRatio = recRF32(rdr)
	def.EnableSuspensionLimit = recRBOOL(rdr)
	def.LowerSuspensionLimit = recRF32(rdr)
	def.UpperSuspensionLimit = recRF32(rdr)
	def.EnableSpinMotor = recRBOOL(rdr)
	def.MaxSpinTorque = recRF32(rdr)
	def.SpinSpeed = recRF32(rdr)
	def.EnableSteering = recRBOOL(rdr)
	def.SteeringHertz = recRF32(rdr)
	def.SteeringDampingRatio = recRF32(rdr)
	def.TargetSteeringAngle = recRF32(rdr)
	def.MaxSteeringTorque = recRF32(rdr)
	def.EnableSteeringLimit = recRBOOL(rdr)
	def.LowerSteeringLimit = recRF32(rdr)
	def.UpperSteeringLimit = recRF32(rdr)
	return def
}

// Outliner body tracking. Defined after the player struct; forward declared here because the
// create/destroy dispatch sits above that struct.
// PORT: Go needs no forward declarations; recTrackBodyCreate and recTrackBodyDestroy are in
// replay_player.go.

// Id retargeting: replace world0 with the replay world's slot index.

// recMakeBodyId is b3RecMakeBodyId.
func recMakeBodyId(rdr *recReader, recorded BodyID) BodyID {
	var id BodyID
	id.Index1 = recorded.Index1
	id.World0 = rdr.replayWorldId.Index1 - 1
	id.Generation = recorded.Generation
	return id
}

// recMakeShapeId is b3RecMakeShapeId.
func recMakeShapeId(rdr *recReader, recorded ShapeID) ShapeID {
	var id ShapeID
	id.Index1 = recorded.Index1
	id.World0 = rdr.replayWorldId.Index1 - 1
	id.Generation = recorded.Generation
	return id
}

// recMakeJointId is b3RecMakeJointId.
func recMakeJointId(rdr *recReader, recorded JointID) JointID {
	var id JointID
	id.Index1 = recorded.Index1
	id.World0 = rdr.replayWorldId.Index1 - 1
	id.Generation = recorded.Generation
	return id
}

// recCheckId is b3RecCheckId.
// A create op appends the returned id after args. index1 and generation must match;
// world0 always differs so we ignore it.
func recCheckId(rdr *recReader, kind string, gotIndex int32, gotGen uint16, recIndex int32, recGen uint16) {
	if gotIndex != recIndex || gotGen != recGen {
		fmt.Printf("b3ReplayFile: %s id mismatch (rec index1=%d gen=%d, got index1=%d gen=%d)\n", kind, recIndex, recGen, gotIndex,
			gotGen)
		rdr.ok = false
	}
}

// recCheckBodyId is b3RecCheckBodyId.
func recCheckBodyId(rdr *recReader, got BodyID, rec BodyID) {
	recCheckId(rdr, "body", got.Index1, got.Generation, rec.Index1, rec.Generation)
}

// recCheckShapeId is b3RecCheckShapeId.
func recCheckShapeId(rdr *recReader, got ShapeID, rec ShapeID) {
	recCheckId(rdr, "shape", got.Index1, got.Generation, rec.Index1, rec.Generation)
}

// recCheckJointId is b3RecCheckJointId.
func recCheckJointId(rdr *recReader, got JointID, rec JointID) {
	recCheckId(rdr, "joint", got.Index1, got.Generation, rec.Index1, rec.Generation)
}

// recGetSlot is b3RecGetSlot.
// Get a geometry slot with kind validation.
func recGetSlot(rdr *recReader, id uint32, kind geometryKind) *registrySlot {
	if rdr == nil || id >= uint32(rdr.slotCount) || rdr.slots[id].kind != kind {
		return nil
	}
	return &rdr.slots[id]
}

// recSlotHeader views a registry slot's blob as its geometry header (the (const b3HullData*) casts
// of slot->bytes upstream).
// PORT: helper for the casts; slot bytes are an 8-byte aligned blob (see recLoadSlots).
func recSlotHeader[T any](slot *registrySlot) *T {
	return (*T)(unsafe.Pointer(unsafe.SliceData(slot.bytes)))
}
