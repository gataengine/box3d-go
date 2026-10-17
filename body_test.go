// Port of test/test_body.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

// PORT: test_body.c drives a full world (b3CreateWorld, b3CreateSphereShape, b3World_Step, ...),
// which needs shape.c, physics_world.c and the rest of milestone 3. The build tag keeps this file
// out of the build until the milestone 3 integration enables it (remove the tag then).

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
)

// b3UpdateBodyMassData shifts each shape's inertia to the body center of mass with the parallel
// axis theorem. When shapes sit far from the body origin the shift term dwarfs the central inertia,
// so any error in the per shape framing blows up the tensor. Spheres make a clean oracle: the
// central inertia is isotropic and independent of placement, so the shift is the only thing tested.

func bodyTestSphereBodyMass(centers []Vec3, count int, radius, density float32) MassData {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	shapeDef.Density = density

	for i := 0; i < count; i++ {
		sphere := Sphere{centers[i], radius}
		bodyId.CreateSphereShape(&shapeDef, &sphere)
	}

	bodyId.ApplyMassFromShapes()
	massData := bodyId.GetMassData()

	worldId.Destroy()
	return massData
}

// Diagonal inertia with inverses that are exact in float, so tolerances stay tight.
var bodyTestDiagInertia = Matrix3{Vec3{2.0, 0.0, 0.0}, Vec3{0.0, 4.0, 0.0}, Vec3{0.0, 0.0, 8.0}}

func bodyTestCreateSpinningHullBody(worldId WorldID, x float32, hull *HullData) BodyID {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position.X = x
	bodyDef.Rotation = MakeQuatFromAxisAngle(Normalize(Vec3{0.3, -0.8, 0.5}), 1.3)
	bodyDef.AngularVelocity = Vec3{1.0, -2.0, 3.0}
	bodyId := worldId.CreateBody(&bodyDef)

	shapeDef := DefaultShapeDef()
	bodyId.CreateHullShape(&shapeDef, hull)
	return bodyId
}

func bodyTestHasGyroscopicTorque(bodyId BodyID) bool {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return (body.flags & enableGyroscopicTorque) != 0
}

var _ = Describe("BodyTest", func() {
	// One sphere far from the body origin. The center of mass lands on the sphere and the inertia about
	// it must be the bare central inertia, with no trace of the offset.
	It("FarSingleSphereMass", func() {
		var radius float32 = 0.5
		var density float32 = 1.0
		center := Vec3{100.0, -50.0, 75.0}
		md := bodyTestSphereBodyMass([]Vec3{center}, 1, radius, density)

		mass := density * (4.0 / 3.0) * Pi * radius * radius * radius
		central := 0.4 * mass * radius * radius

		ensureSmall(md.Mass-mass, 1e-4)

		ensureSmall(md.Center.X-center.X, 1e-3)
		ensureSmall(md.Center.Y-center.Y, 1e-3)
		ensureSmall(md.Center.Z-center.Z, 1e-3)

		ensureSmall(md.Inertia.Cx.X-central, 1e-3)
		ensureSmall(md.Inertia.Cy.Y-central, 1e-3)
		ensureSmall(md.Inertia.Cz.Z-central, 1e-3)

		ensureSmall(md.Inertia.Cy.X, 1e-3)
		ensureSmall(md.Inertia.Cz.X, 1e-3)
		ensureSmall(md.Inertia.Cz.Y, 1e-3)
	})

	// Eight equal spheres on the corners of a cube, the whole cube parked far from the body origin.
	// The center of mass is the cube center and the products of inertia cancel by symmetry, so the
	// tensor stays diagonal no matter how far out the cube sits.
	It("FarCubeSphereMass", func() {
		var radius float32 = 0.5
		var density float32 = 1.0
		var h float32 = 1.0
		p := Vec3{100.0, 100.0, 100.0}

		var centers [8]Vec3
		k := 0
		for sx := -1; sx <= 1; sx += 2 {
			for sy := -1; sy <= 1; sy += 2 {
				for sz := -1; sz <= 1; sz += 2 {
					centers[k] = Vec3{p.X + float32(float32(sx)*h), p.Y + float32(float32(sy)*h), p.Z + float32(float32(sz)*h)}
					k++
				}
			}
		}

		md := bodyTestSphereBodyMass(centers[:], 8, radius, density)

		mass := density * (4.0 / 3.0) * Pi * radius * radius * radius
		totalMass := 8.0 * mass

		// Per sphere central inertia summed, plus the parallel axis term for each corner offset
		// (dy^2 + dz^2) = (h^2 + h^2) about every axis.
		diag := float32(8.0*0.4*mass*radius*radius) + float32(16.0*mass*h*h)

		ensureSmall(md.Mass-totalMass, 1e-3)

		ensureSmall(md.Center.X-p.X, 1e-2)
		ensureSmall(md.Center.Y-p.Y, 1e-2)
		ensureSmall(md.Center.Z-p.Z, 1e-2)

		ensureSmall(md.Inertia.Cx.X-diag, 1e-2)
		ensureSmall(md.Inertia.Cy.Y-diag, 1e-2)
		ensureSmall(md.Inertia.Cz.Z-diag, 1e-2)

		ensureSmall(md.Inertia.Cy.X, 1e-2)
		ensureSmall(md.Inertia.Cz.X, 1e-2)
		ensureSmall(md.Inertia.Cz.Y, 1e-2)
	})

	// Shapes added with updateBodyMass = false defer the mass update, which is also the only place
	// body extents are computed. A body that reaches the solver with minExtent == B3_HUGE never passes
	// the continuous collision gate. The dirty mass flag must track the deferral, and both
	// ApplyMassFromShapes and SetMassData must leave finite extents behind.
	It("DeferredMassExtents", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody

		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		shapeDef.UpdateBodyMass = false

		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}

		// Deferred create leaves mass and extents untouched but marks the body dirty.
		applyId := worldId.CreateBody(&bodyDef)
		applyId.CreateSphereShape(&shapeDef, &sphere)

		world := getWorld(int(applyId.World0))
		applyBody := getBodyFullId(world, applyId)
		applySim := getBodySim(world, applyBody)

		ensure((applyBody.flags & dirtyMass) != 0)
		ensure(applySim.minExtent == Huge())

		// ApplyMassFromShapes computes extents and clears the flag.
		applyId.ApplyMassFromShapes()
		ensure((applyBody.flags & dirtyMass) == 0)
		ensure(applySim.minExtent < Huge())

		// SetMassData alone must also produce finite extents and clear the flag (the issue #35 repro).
		massId := worldId.CreateBody(&bodyDef)
		massId.CreateSphereShape(&shapeDef, &sphere)

		massBody := getBodyFullId(world, massId)
		massSim := getBodySim(world, massBody)
		ensure((massBody.flags & dirtyMass) != 0)

		inertia := Matrix3{Vec3{0.2, 0.0, 0.0}, Vec3{0.0, 0.2, 0.0}, Vec3{0.0, 0.0, 0.2}}
		massData := MassData{2.0, Vec3{0.0, 0.0, 0.0}, inertia}
		massId.SetMassData(massData)

		ensure((massBody.flags & dirtyMass) == 0)
		ensure(massSim.minExtent < Huge())

		worldId.Destroy()
	})

	// b3Body_SetMassData overrides the mass properties directly, bypassing the shapes. It must derive
	// everything the solver reads from the supplied tensor: the inverse mass, the local inverse inertia,
	// and the world inverse inertia rotated by the body orientation. Fixed rotation zeros the angular part.
	// These tests drive it through the public getters, no shapes required.

	It("SetMassDataRoundTrip", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{5.0, -3.0, 2.0}
		bodyId := worldId.CreateBody(&bodyDef)

		center := Vec3{0.1, 0.2, 0.3}
		massData := MassData{3.0, center, bodyTestDiagInertia}
		bodyId.SetMassData(massData)

		ensureSmall(bodyId.GetMass()-3.0, 1e-6)
		ensureSmall(bodyId.GetInverseMass()-1.0/3.0, 1e-6)

		md := bodyId.GetMassData()
		ensureSmall(md.Mass-3.0, 1e-6)
		ensureSmall(md.Center.X-center.X, 1e-6)
		ensureSmall(md.Center.Y-center.Y, 1e-6)
		ensureSmall(md.Center.Z-center.Z, 1e-6)
		ensureSmall(md.Inertia.Cx.X-2.0, 1e-6)
		ensureSmall(md.Inertia.Cy.Y-4.0, 1e-6)
		ensureSmall(md.Inertia.Cz.Z-8.0, 1e-6)

		localCenter := bodyId.GetLocalCenter()
		ensureSmall(localCenter.X-center.X, 1e-6)
		ensureSmall(localCenter.Y-center.Y, 1e-6)
		ensureSmall(localCenter.Z-center.Z, 1e-6)

		localInertia := bodyId.GetLocalRotationalInertia()
		ensureSmall(localInertia.Cx.X-2.0, 1e-6)
		ensureSmall(localInertia.Cy.Y-4.0, 1e-6)
		ensureSmall(localInertia.Cz.Z-8.0, 1e-6)

		// Identity rotation, so the world inverse inertia is just the local inverse: diag(1/2, 1/4, 1/8).
		invWorld := bodyId.GetWorldInverseRotationalInertia()
		ensureSmall(invWorld.Cx.X-0.5, 1e-5)
		ensureSmall(invWorld.Cy.Y-0.25, 1e-5)
		ensureSmall(invWorld.Cz.Z-0.125, 1e-5)
		ensureSmall(invWorld.Cy.X, 1e-5)
		ensureSmall(invWorld.Cz.X, 1e-5)
		ensureSmall(invWorld.Cx.Y, 1e-5)
		ensureSmall(invWorld.Cz.Y, 1e-5)
		ensureSmall(invWorld.Cx.Z, 1e-5)
		ensureSmall(invWorld.Cy.Z, 1e-5)

		// World center of mass is the body origin plus the local center under identity rotation.
		worldCenter := bodyId.GetWorldCenter()
		ensureSmall(worldCenter.X-(5.0+center.X), 1e-5)
		ensureSmall(worldCenter.Y-(-3.0+center.Y), 1e-5)
		ensureSmall(worldCenter.Z-(2.0+center.Z), 1e-5)

		worldId.Destroy()
	})

	// The world inverse inertia must be the local inverse rotated into world space. A 90 degree turn about
	// z swaps the x and y principal moments, so diag(1/2, 1/4, 1/8) becomes diag(1/4, 1/2, 1/8). This is the
	// regression guard: before SetMassData rotated the tensor it left the world inverse inertia stale.
	It("SetMassDataWorldInertiaRotated", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Rotation = MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)
		bodyId := worldId.CreateBody(&bodyDef)

		massData := MassData{1.0, Vec3{0.0, 0.0, 0.0}, bodyTestDiagInertia}
		bodyId.SetMassData(massData)

		// The local inertia is stored untouched by the world transform.
		localInertia := bodyId.GetLocalRotationalInertia()
		ensureSmall(localInertia.Cx.X-2.0, 1e-6)
		ensureSmall(localInertia.Cy.Y-4.0, 1e-6)
		ensureSmall(localInertia.Cz.Z-8.0, 1e-6)

		invWorld := bodyId.GetWorldInverseRotationalInertia()
		ensureSmall(invWorld.Cx.X-0.25, 1e-4)
		ensureSmall(invWorld.Cy.Y-0.5, 1e-4)
		ensureSmall(invWorld.Cz.Z-0.125, 1e-4)
		ensureSmall(invWorld.Cy.X, 1e-4)
		ensureSmall(invWorld.Cz.X, 1e-4)
		ensureSmall(invWorld.Cx.Y, 1e-4)
		ensureSmall(invWorld.Cz.Y, 1e-4)
		ensureSmall(invWorld.Cx.Z, 1e-4)
		ensureSmall(invWorld.Cy.Z, 1e-4)

		worldId.Destroy()
	})

	// Fixed rotation must leave the mass intact but drive the whole angular inertia to zero, even when the
	// caller hands in a real tensor.
	It("SetMassDataFixedRotation", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.MotionLocks.AngularX = true
		bodyDef.MotionLocks.AngularY = true
		bodyDef.MotionLocks.AngularZ = true
		bodyId := worldId.CreateBody(&bodyDef)

		massData := MassData{5.0, Vec3{0.0, 0.0, 0.0}, bodyTestDiagInertia}
		bodyId.SetMassData(massData)

		ensureSmall(bodyId.GetMass()-5.0, 1e-6)
		ensureSmall(bodyId.GetInverseMass()-0.2, 1e-6)

		localInertia := bodyId.GetLocalRotationalInertia()
		ensureSmall(localInertia.Cx.X, 1e-6)
		ensureSmall(localInertia.Cy.Y, 1e-6)
		ensureSmall(localInertia.Cz.Z, 1e-6)

		invWorld := bodyId.GetWorldInverseRotationalInertia()
		ensureSmall(invWorld.Cx.X, 1e-6)
		ensureSmall(invWorld.Cy.Y, 1e-6)
		ensureSmall(invWorld.Cz.Z, 1e-6)

		md := bodyId.GetMassData()
		ensureSmall(md.Inertia.Cx.X, 1e-6)
		ensureSmall(md.Inertia.Cy.Y, 1e-6)
		ensureSmall(md.Inertia.Cz.Z, 1e-6)

		worldId.Destroy()
	})

	// Zero mass and a zero tensor have zero determinant, so the inverses must collapse to zero rather than
	// divide by it.
	It("SetMassDataZeroMass", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyId := worldId.CreateBody(&bodyDef)

		massData := MassData{0.0, Vec3{0.0, 0.0, 0.0}, Mat3Zero}
		bodyId.SetMassData(massData)

		ensureSmall(bodyId.GetInverseMass(), 1e-6)

		localInertia := bodyId.GetLocalRotationalInertia()
		ensureSmall(localInertia.Cx.X, 1e-6)
		ensureSmall(localInertia.Cy.Y, 1e-6)
		ensureSmall(localInertia.Cz.Z, 1e-6)

		invWorld := bodyId.GetWorldInverseRotationalInertia()
		ensureSmall(invWorld.Cx.X, 1e-6)
		ensureSmall(invWorld.Cy.Y, 1e-6)
		ensureSmall(invWorld.Cz.Z, 1e-6)

		worldId.Destroy()
	})

	// The stored linear velocity tracks the center of mass. Moving the center picks a different material
	// point, so a spinning body must have its velocity re-referenced by omega x (newCenter - oldCenter),
	// otherwise the mass edit silently injects or drains kinetic energy. Under identity rotation the world
	// shift equals the supplied local center, keeping the expected values exact in float.
	It("SetMassDataConsistentVelocity", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{7.0, 1.0, -4.0}
		bodyId := worldId.CreateBody(&bodyDef)

		// Spin about the origin center, then shift the center of mass off the origin.
		omega := Vec3{1.0, 2.0, 4.0}
		bodyId.SetLinearVelocity(Vec3{1.0, -2.0, 3.0})
		bodyId.SetAngularVelocity(omega)

		center := Vec3{0.5, 0.25, 0.125}
		massData := MassData{3.0, center, bodyTestDiagInertia}
		bodyId.SetMassData(massData)

		// omega x center = ( 2*0.125 - 4*0.25, 4*0.5 - 1*0.125, 1*0.25 - 2*0.5 ) = ( -0.75, 1.875, -0.75 )
		v := bodyId.GetLinearVelocity()
		ensureSmall(v.X-(1.0-0.75), 1e-6)
		ensureSmall(v.Y-(-2.0+1.875), 1e-6)
		ensureSmall(v.Z-(3.0-0.75), 1e-6)

		// Only the reference point moved, the angular velocity is untouched.
		w := bodyId.GetAngularVelocity()
		ensureSmall(w.X-omega.X, 1e-6)
		ensureSmall(w.Y-omega.Y, 1e-6)
		ensureSmall(w.Z-omega.Z, 1e-6)

		worldId.Destroy()
	})

	// Extents bound the shapes about the center of mass, per axis. An offset shape must count its
	// offset, not just its own size.
	It("ShapeExtents", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0

		// Kinematic bodies measure from the body origin
		bodyDef := DefaultBodyDef()
		bodyDef.Type = KinematicBody

		capsuleId := worldId.CreateBody(&bodyDef)
		capsule := Capsule{Vec3{-2.0, 0.0, 0.0}, Vec3{-1.0, 0.0, 0.0}, 0.2}
		capsuleId.CreateCapsuleShape(&shapeDef, &capsule)

		maxExtent := capsuleId.GetMaxExtent()
		ensureSmall(maxExtent.X-2.2, 1e-5)
		ensureSmall(maxExtent.Y-0.2, 1e-5)
		ensureSmall(maxExtent.Z-0.2, 1e-5)
		ensureSmall(capsuleId.GetMinExtent()-0.2, 1e-5)

		sphereId := worldId.CreateBody(&bodyDef)
		sphere := Sphere{Vec3{1.0, 2.0, 3.0}, 0.5}
		sphereId.CreateSphereShape(&shapeDef, &sphere)

		maxExtent = sphereId.GetMaxExtent()
		ensureSmall(maxExtent.X-1.5, 1e-5)
		ensureSmall(maxExtent.Y-2.5, 1e-5)
		ensureSmall(maxExtent.Z-3.5, 1e-5)
		ensureSmall(sphereId.GetMinExtent()-0.5, 1e-5)

		// Dynamic bodies measure from the center of mass. A light sphere hung off a cube pulls the
		// center toward it, so the far side of the sphere is the widest point.
		bodyDef.Type = DynamicBody
		cubeId := worldId.CreateBody(&bodyDef)
		cube := MakeCubeHull(0.5)
		cubeId.CreateHullShape(&shapeDef, &cube.Base)
		offsetSphere := Sphere{Vec3{1.0, 0.0, 0.0}, 0.2}
		cubeId.CreateSphereShape(&shapeDef, &offsetSphere)

		localCenter := cubeId.GetLocalCenter()
		ensure(0.0 < localCenter.X && localCenter.X < 0.5)

		maxExtent = cubeId.GetMaxExtent()
		ensureSmall(maxExtent.X-(1.2-localCenter.X), 1e-5)
		ensureSmall(maxExtent.Y-0.5, 1e-5)
		ensureSmall(maxExtent.Z-0.5, 1e-5)
		ensureSmall(cubeId.GetMinExtent()-0.2, 1e-5)

		originExtent := cubeId.GetMaxExtentOrigin()
		ensureSmall(originExtent.X-1.2, 1e-5)
		ensureSmall(originExtent.Y-0.5, 1e-5)
		ensureSmall(originExtent.Z-0.5, 1e-5)

		// A mesh has no mass, so the sphere alone places the center at x = 1 and the far edge of the
		// mesh at x = -3 is four units out.
		vertices := [6]Vec3{
			{-3.0, 0.0, -1.0}, {-2.0, 0.0, 1.0}, {-1.0, 0.0, -1.0},
			{1.0, 0.0, -1.0}, {2.0, 0.0, 1.0}, {3.0, 0.0, -1.0},
		}
		indices := [6]int32{0, 1, 2, 3, 4, 5}
		var meshDef MeshDef
		meshDef.Vertices = vertices[:]
		meshDef.Stride = 12 // sizeof( b3Vec3 )
		meshDef.Indices = indices[:]
		meshDef.VertexCount = 6
		meshDef.TriangleCount = 2
		mesh := CreateMesh(&meshDef, nil)

		meshId := worldId.CreateBody(&bodyDef)
		meshId.CreateSphereShape(&shapeDef, &offsetSphere)
		meshId.CreateMeshShape(&shapeDef, mesh, Vec3{1.0, 1.0, 1.0})

		localCenter = meshId.GetLocalCenter()
		ensureSmall(localCenter.X-1.0, 1e-5)

		maxExtent = meshId.GetMaxExtent()
		ensureSmall(maxExtent.X-4.0, 1e-4)
		ensureSmall(maxExtent.Y-0.2, 1e-4)
		ensureSmall(maxExtent.Z-1.0, 1e-4)
		ensureSmall(meshId.GetMinExtent()-0.2, 1e-5)

		worldId.Destroy()
		DestroyMesh(mesh)
	})

	It("CompoundInertiaConsistency", func() {
		worldDef := DefaultWorldDef()
		worldId := CreateWorld(&worldDef)

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Rotation = MakeQuatFromAxisAngle(Normalize(Vec3{0.3, -0.8, 0.5}), 1.3)
		bodyId := worldId.CreateBody(&bodyDef)

		shapeDef := DefaultShapeDef()
		boxA := MakeOffsetBoxHull(0.5, 0.25, 0.5, Vec3{1.0, 1.0, 0.0})
		boxB := MakeOffsetBoxHull(0.3, 0.6, 0.2, Vec3{-0.8, -0.4, 0.3})
		bodyId.CreateHullShape(&shapeDef, &boxA.Base)
		bodyId.CreateHullShape(&shapeDef, &boxB.Base)
		bodyId.ApplyMassFromShapes()

		local := bodyId.GetLocalRotationalInertia()
		ensure(AbsFloat(local.Cy.X) > 0.1)

		r := MakeMatrixFromQuat(bodyDef.Rotation)
		world := MulMM(MulMM(r, local), Transpose(r))
		invWorld := bodyId.GetWorldInverseRotationalInertia()
		product := MulMM(invWorld, world)

		ensureSmall(product.Cx.X-1.0, 1.0e-4)
		ensureSmall(product.Cy.Y-1.0, 1.0e-4)
		ensureSmall(product.Cz.Z-1.0, 1.0e-4)
		ensureSmall(product.Cx.Y, 1.0e-4)
		ensureSmall(product.Cx.Z, 1.0e-4)
		ensureSmall(product.Cy.X, 1.0e-4)
		ensureSmall(product.Cy.Z, 1.0e-4)
		ensureSmall(product.Cz.X, 1.0e-4)
		ensureSmall(product.Cz.Y, 1.0e-4)

		q2 := MakeQuatFromAxisAngle(Normalize(Vec3{-1.0, 0.2, 0.4}), -2.1)
		bodyId.SetTransform(bodyId.GetPosition(), q2)
		r = MakeMatrixFromQuat(q2)
		world = MulMM(MulMM(r, local), Transpose(r))
		invWorld = bodyId.GetWorldInverseRotationalInertia()
		product = MulMM(invWorld, world)
		ensureSmall(product.Cx.X-1.0, 1.0e-4)
		ensureSmall(product.Cy.Y-1.0, 1.0e-4)
		ensureSmall(product.Cz.Z-1.0, 1.0e-4)
		ensureSmall(product.Cy.X, 1.0e-4)
		ensureSmall(product.Cz.Y, 1.0e-4)

		worldId.Destroy()
	})

	// Bodies within the isotropic tolerance skip the gyroscopic solve, so in free flight the world
	// angular velocity is untouched. Anisotropic bodies precess.
	It("IsotropicInertiaTolerance", func() {
		w0 := Vec3{1.0, -2.0, 3.0}

		rock := CreateRock(0.5)
		complex := CreateComplexHull(0.5)
		cube := MakeCubeHull(0.5)
		cubeTransform := Transform{Vec3Zero, MakeQuatFromAxisAngle(Normalize(Vec3{1.0, 2.0, 3.0}), 0.7)}
		rotatedCube := MakeTransformedBoxHull(0.5, 0.5, 0.5, cubeTransform)
		plank := MakeBoxHull(0.5, 1.0, 0.25)

		{
			worldDef := DefaultWorldDef()
			worldDef.Gravity = Vec3Zero
			worldId := CreateWorld(&worldDef)

			rockId := bodyTestCreateSpinningHullBody(worldId, 0.0, rock)
			complexId := bodyTestCreateSpinningHullBody(worldId, 5.0, complex)
			cubeId := bodyTestCreateSpinningHullBody(worldId, 10.0, &cube.Base)
			rotatedCubeId := bodyTestCreateSpinningHullBody(worldId, 15.0, &rotatedCube.Base)
			plankId := bodyTestCreateSpinningHullBody(worldId, 20.0, &plank.Base)

			ensure(bodyTestHasGyroscopicTorque(rockId) == false)
			ensure(bodyTestHasGyroscopicTorque(complexId) == false)
			ensure(bodyTestHasGyroscopicTorque(cubeId) == false)
			ensure(bodyTestHasGyroscopicTorque(rotatedCubeId) == false)
			ensure(bodyTestHasGyroscopicTorque(plankId))

			for i := 0; i < 10; i++ {
				worldId.Step(1.0/60.0, 4)
			}

			w := rockId.GetAngularVelocity()
			ensure(w.X == w0.X && w.Y == w0.Y && w.Z == w0.Z)
			w = complexId.GetAngularVelocity()
			ensure(w.X == w0.X && w.Y == w0.Y && w.Z == w0.Z)
			w = rotatedCubeId.GetAngularVelocity()
			ensure(w.X == w0.X && w.Y == w0.Y && w.Z == w0.Z)
			w = plankId.GetAngularVelocity()
			ensure(Length(Sub(w, w0)) > 0.01)

			plankId.SetMotionLocks(MotionLocks{AngularX: true, AngularY: true, AngularZ: true})
			ensure(bodyTestHasGyroscopicTorque(plankId) == false)

			// Gyroscopic torque stays off until a mass update proves the body anisotropic
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Position.X = 25.0
			bodyDef.AngularVelocity = w0
			emptyId := worldId.CreateBody(&bodyDef)
			ensure(bodyTestHasGyroscopicTorque(emptyId) == false)

			bodyDef.Position.X = 30.0
			deferredId := worldId.CreateBody(&bodyDef)
			shapeDef := DefaultShapeDef()
			shapeDef.UpdateBodyMass = false
			deferredId.CreateHullShape(&shapeDef, &plank.Base)
			ensure(bodyTestHasGyroscopicTorque(deferredId) == false)

			deferredId.ApplyMassFromShapes()
			ensure(bodyTestHasGyroscopicTorque(deferredId))

			worldId.Destroy()
		}

		{
			worldDef := DefaultWorldDef()
			worldDef.Gravity = Vec3Zero
			worldDef.IsotropicInertiaTolerance = 0.0
			worldId := CreateWorld(&worldDef)

			rockId := bodyTestCreateSpinningHullBody(worldId, 0.0, rock)
			cubeId := bodyTestCreateSpinningHullBody(worldId, 10.0, &cube.Base)

			ensure(bodyTestHasGyroscopicTorque(rockId))
			ensure(bodyTestHasGyroscopicTorque(cubeId) == false)

			for i := 0; i < 10; i++ {
				worldId.Step(1.0/60.0, 4)
			}

			w := rockId.GetAngularVelocity()
			ensure(Length(Sub(w, w0)) > 0.001)

			worldId.Destroy()
		}

		DestroyHull(rock)
		DestroyHull(complex)
	})
})
