// Port of include/box3d/id.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// Note: this file should be stand-alone

// These ids serve as handles to internal Box3D objects.
// These should be considered opaque data and passed by value.
// All ids are considered null if initialized to zero.
//
//	var worldID box3d.WorldID // null
//
// Warning: Do not use the internals of these ids. They are subject to change. Ids should be treated as opaque objects.
// Warning: You should use ids to access objects in Box3D. Do not access files within the src folder. Such usage is unsupported.
//
// PORT: ids are comparable with ==, which replaces B3_ID_EQUALS.

// WorldID references a world instance. This should be treated as an opaque handle.
type WorldID struct {
	Index1     uint16
	Generation uint16
}

// BodyID references a body instance. This should be treated as an opaque handle.
type BodyID struct {
	Index1     int32
	World0     uint16
	Generation uint16
}

// ShapeID references a shape instance. This should be treated as an opaque handle.
type ShapeID struct {
	Index1     int32
	World0     uint16
	Generation uint16
}

// JointID references a joint instance. This should be treated as an opaque handle.
type JointID struct {
	Index1     int32
	World0     uint16
	Generation uint16
}

// ContactID references a contact instance. This should be treated as an opaque handle.
type ContactID struct {
	Index1     int32
	World0     uint16
	Padding    int16
	Generation uint32
}

// Use these to make your identifiers null.
// You may also use zero initialization to get null.
var (
	NullWorldID   = WorldID{}
	NullBodyID    = BodyID{}
	NullShapeID   = ShapeID{}
	NullJointID   = JointID{}
	NullContactID = ContactID{}
)

// IsNull is B3_IS_NULL: determine if the id is null.
func (id WorldID) IsNull() bool { return id.Index1 == 0 }

// IsNull is B3_IS_NULL: determine if the id is null.
func (id BodyID) IsNull() bool { return id.Index1 == 0 }

// IsNull is B3_IS_NULL: determine if the id is null.
func (id ShapeID) IsNull() bool { return id.Index1 == 0 }

// IsNull is B3_IS_NULL: determine if the id is null.
func (id JointID) IsNull() bool { return id.Index1 == 0 }

// IsNull is B3_IS_NULL: determine if the id is null.
func (id ContactID) IsNull() bool { return id.Index1 == 0 }

// IsNonNull is B3_IS_NON_NULL: determine if the id is non-null.
func (id WorldID) IsNonNull() bool { return id.Index1 != 0 }

// IsNonNull is B3_IS_NON_NULL: determine if the id is non-null.
func (id BodyID) IsNonNull() bool { return id.Index1 != 0 }

// IsNonNull is B3_IS_NON_NULL: determine if the id is non-null.
func (id ShapeID) IsNonNull() bool { return id.Index1 != 0 }

// IsNonNull is B3_IS_NON_NULL: determine if the id is non-null.
func (id JointID) IsNonNull() bool { return id.Index1 != 0 }

// IsNonNull is B3_IS_NON_NULL: determine if the id is non-null.
func (id ContactID) IsNonNull() bool { return id.Index1 != 0 }

// StoreWorldID stores a world id into a uint32.
func StoreWorldID(id WorldID) uint32 {
	return (uint32(id.Index1) << 16) | uint32(id.Generation)
}

// LoadWorldID loads a uint32 into a world id.
func LoadWorldID(x uint32) WorldID {
	id := WorldID{uint16(x >> 16), uint16(x)}
	return id
}

// StoreBodyID stores a body id into a uint64.
func StoreBodyID(id BodyID) uint64 {
	return (uint64(uint32(id.Index1)) << 32) | (uint64(id.World0))<<16 | uint64(id.Generation)
}

// LoadBodyID loads a uint64 into a body id.
func LoadBodyID(x uint64) BodyID {
	id := BodyID{int32(x >> 32), uint16(x >> 16), uint16(x)}
	return id
}

// StoreShapeID stores a shape id into a uint64.
func StoreShapeID(id ShapeID) uint64 {
	return (uint64(uint32(id.Index1)) << 32) | (uint64(id.World0))<<16 | uint64(id.Generation)
}

// LoadShapeID loads a uint64 into a shape id.
func LoadShapeID(x uint64) ShapeID {
	id := ShapeID{int32(x >> 32), uint16(x >> 16), uint16(x)}
	return id
}

// StoreJointID stores a joint id into a uint64.
func StoreJointID(id JointID) uint64 {
	return (uint64(uint32(id.Index1)) << 32) | (uint64(id.World0))<<16 | uint64(id.Generation)
}

// LoadJointID loads a uint64 into a joint id.
func LoadJointID(x uint64) JointID {
	id := JointID{int32(x >> 32), uint16(x >> 16), uint16(x)}
	return id
}

// StoreContactID stores a contact id into three uint32 values.
func StoreContactID(id ContactID, values *[3]uint32) {
	values[0] = uint32(id.Index1)
	values[1] = uint32(id.World0)
	values[2] = id.Generation
}

// LoadContactID loads a contact id from three uint32 values.
func LoadContactID(values *[3]uint32) ContactID {
	var id ContactID
	id.Index1 = int32(values[0])
	id.World0 = uint16(values[1])
	id.Padding = 0
	id.Generation = values[2]
	return id
}
