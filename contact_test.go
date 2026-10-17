// Go-only test: no upstream counterpart (upstream has no test_contact.c; contacts are covered by
// the world tests). Exercises contact.c on a hand assembled world: the contact register table, the
// body contact lists, solver set and pair set bookkeeping of b3CreateContact / b3DestroyContact,
// and the convex paths of b3UpdateContact (including the compound child flips).
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

// contactTestWorld is a world with just what contact.c reaches: bodies, shapes, four solver sets
// (static, disabled, awake, one sleeping), the contact id pool, the pair set, the manifold
// allocators and the material callbacks.
func contactTestWorld() *world {
	w := &world{}
	capacity := Capacity{}
	createBroadPhase(&w.broadPhase, &capacity)
	w.contactIdPool = createIdPool()
	w.solverSets = make([]solverSet, 4)
	for i := range w.solverSets {
		w.solverSets[i].setIndex = int32(i)
	}
	w.manifoldAllocatorMutex = createMutex()
	w.taskContexts = make([]taskContext, 1)
	w.workerCount = 1
	w.simdWidth = int32(GetSIMDWidth())
	w.worldId = 3
	w.frictionCallback = func(frictionA float32, _ uint64, frictionB float32, _ uint64) float32 {
		return MaxFloat(frictionA, frictionB)
	}
	w.restitutionCallback = func(restitutionA float32, _ uint64, restitutionB float32, _ uint64) float32 {
		return MinFloat(restitutionA, restitutionB)
	}
	return w
}

func contactTestBody(w *world, typ BodyType, setIndex, localIndex int) int {
	id := len(w.bodies)
	b := arrayEmplace(&w.bodies)
	b.id = int32(id)
	b.typ = typ
	b.setIndex = int32(setIndex)
	b.localIndex = int32(localIndex)
	b.headContactKey = NullIndex
	b.headShapeId = NullIndex
	b.headJointKey = NullIndex
	b.islandId = NullIndex
	return id
}

func contactTestShape(w *world, bodyId int, typ ShapeType) *shape {
	id := len(w.shapes)
	s := arrayEmplace(&w.shapes)
	s.id = int32(id)
	s.bodyId = int32(bodyId)
	s.typ = typ
	s.sensorIndex = NullIndex
	s.proxyKey = NullIndex
	s.generation = uint16(10 + id)
	s.material = DefaultSurfaceMaterial()
	s.materialCount = 1
	s.flags = uint8(enableSpeculative)
	return s
}

var _ = Describe("ContactTest", func() {
	BeforeEach(func() {
		// The register table is process global. Restore it so specs that rely on it being empty
		// (broad_phase_golden_test.go emulates b3CreateContact) see the same state in any order.
		savedRegisters := contactRegisters
		savedInitialized := contactRegistersInitialized
		DeferCleanup(func() {
			contactRegisters = savedRegisters
			contactRegistersInitialized = savedInitialized
		})
		initializeContactRegisters()
	})

	It("RegisterTable", func() {
		primary := [][2]ShapeType{
			{SphereShape, SphereShape},
			{CapsuleShape, SphereShape},
			{CapsuleShape, CapsuleShape},
			{CompoundShape, SphereShape},
			{CompoundShape, CapsuleShape},
			{CompoundShape, HullShape},
			{HullShape, SphereShape},
			{HullShape, CapsuleShape},
			{HullShape, HullShape},
			{MeshShape, SphereShape},
			{MeshShape, CapsuleShape},
			{MeshShape, HullShape},
			{HeightShape, SphereShape},
			{HeightShape, CapsuleShape},
			{HeightShape, HullShape},
		}
		isPrimary := func(a, b ShapeType) bool {
			for _, p := range primary {
				if p[0] == a && p[1] == b {
					return true
				}
			}
			return false
		}

		supportedCount := 0
		for a := ShapeType(0); a < ShapeTypeCount; a++ {
			for b := ShapeType(0); b < ShapeTypeCount; b++ {
				r := contactRegisters[a][b]
				Expect(r.supported).To(Equal(isPrimary(a, b) || isPrimary(b, a)), "supported %d %d", a, b)
				Expect(r.primary).To(Equal(isPrimary(a, b)), "primary %d %d", a, b)
				if r.supported {
					supportedCount += 1
				}
			}
		}
		// 3 symmetric pairs plus 12 asymmetric ones registered both ways.
		Expect(supportedCount).To(Equal(3 + 2*12))

		ensure(contactRegisters[MeshShape][MeshShape].supported == false)
		ensure(contactRegisters[CompoundShape][CompoundShape].supported == false)
		ensure(contactRegisters[CompoundShape][MeshShape].supported == false)
		ensure(contactRegisters[HeightShape][MeshShape].supported == false)
		ensure(contactRegisters[SphereShape][HullShape].primary == false)

		// Idempotent
		before := contactRegisters
		initializeContactRegisters()
		ensure(contactRegisters == before)
	})

	It("CreateDestroy", func() {
		w := contactTestWorld()
		body0 := contactTestBody(w, DynamicBody, awakeSet, 0)
		body1 := contactTestBody(w, StaticBody, staticSet, 0)
		body2 := contactTestBody(w, DynamicBody, awakeSet, 1)
		body3 := contactTestBody(w, DynamicBody, 3, 0)
		body4 := contactTestBody(w, DynamicBody, 3, 1)
		w.bodies[body0].flags = enableContactRecycling
		w.bodies[body2].flags = enableContactRecycling

		s0 := contactTestShape(w, body0, SphereShape)
		s0.sphere = Sphere{Radius: 0.5}
		s0.material.RollingResistance = 0.1
		s0.flags |= uint8(enableContactEvents)
		s1 := contactTestShape(w, body1, HullShape)
		box := MakeCubeHull(1.0)
		s1.hull = &box.Base
		s2 := contactTestShape(w, body2, CapsuleShape)
		s2.capsule = Capsule{Center1: Vec3{0, -0.5, 0}, Center2: Vec3{0, 0.5, 0}, Radius: 0.3}
		s2.material.RollingResistance = 0.2
		s2.flags |= uint8(enablePreSolveEvents)
		s3 := contactTestShape(w, body3, MeshShape)
		s4 := contactTestShape(w, body4, MeshShape)
		s5 := contactTestShape(w, body3, SphereShape)
		s5.sphere = Sphere{Radius: 0.25}
		s6 := contactTestShape(w, body4, HullShape)
		s6.hull = &box.Base
		s6.flags = 0

		// Unsupported pair: nothing happens
		createContact(w, s3, s4, 0)
		ensure(len(w.contacts) == 0)
		ensure(w.broadPhase.pairSet.count == 0)

		// sphere (dynamic) vs hull (static): flipped so the hull is shape A
		createContact(w, &w.shapes[0], &w.shapes[1], 0)
		ensure(len(w.contacts) == 1)
		c0 := &w.contacts[0]
		ensure(c0.contactId == 0 && c0.generation == 1)
		ensure(c0.shapeIdA == 1 && c0.shapeIdB == 0)
		ensure(c0.setIndex == awakeSet && c0.localIndex == 0)
		ensure(c0.colorIndex == NullIndex && c0.islandId == NullIndex && c0.islandIndex == NullIndex)
		ensure(c0.encodedBodySimA == -2 && c0.encodedBodySimB == 0)
		ensure(c0.flags == contactStaticFlag|contactEnableContactEvents|enableSpeculativePoints)
		ensure(c0.edges[0].bodyId == int32(body1) && c0.edges[1].bodyId == int32(body0))
		ensure(w.bodies[body1].headContactKey == 0 && w.bodies[body0].headContactKey == 1)
		ensure(c0.rollingResistance == MaxFloat(0.0, 0.1)*0.5)
		ensure(containsKey(&w.broadPhase.pairSet, shapePairKey(0, 1, 0)))

		// sphere vs capsule: capsule is primary
		createContact(w, &w.shapes[0], &w.shapes[2], 0)
		c0 = &w.contacts[0]
		c1 := &w.contacts[1]
		ensure(c1.shapeIdA == 2 && c1.shapeIdB == 0)
		ensure(c1.flags == contactEnableContactEvents|enableSpeculativePoints|contactRecycleFlag|simEnablePreSolveEvents)
		ensure(c1.rollingResistance == MaxFloat(0.2, 0.1)*MaxFloat(0.3, 0.5))
		ensure(c1.localIndex == 1)
		// body0 is edge B of contact 1 (key 3), then contact 0 (key 1)
		ensure(w.bodies[body0].headContactKey == 3 && w.bodies[body0].contactCount == 2)
		ensure(c1.edges[1].nextKey == 1 && c0.edges[1].prevKey == 3)
		ensure(w.bodies[body2].headContactKey == 2)

		// Sleeping pair lives in the disabled set
		createContact(w, &w.shapes[5], &w.shapes[6], 0)
		c2 := &w.contacts[2]
		ensure(c2.setIndex == disabledSet && c2.localIndex == 0)
		ensure(c2.shapeIdA == 6 && c2.shapeIdB == 5)
		ensure(c2.encodedBodySimA == NullIndex && c2.encodedBodySimB == NullIndex)
		ensure(c2.flags == 0)
		ensure(len(w.solverSets[disabledSet].contactIndices) == 1)

		awake := &w.solverSets[awakeSet]
		ensure(len(awake.contactIndices) == 2 && awake.contactIndices[0] == 0 && awake.contactIndices[1] == 1)
		ensure(w.broadPhase.pairSet.count == 3)

		// Destroy non-touching contact 0: no event, unlink, swap remove
		destroyContact(w, &w.contacts[0], false)
		ensure(containsKey(&w.broadPhase.pairSet, shapePairKey(0, 1, 0)) == false)
		ensure(w.broadPhase.pairSet.count == 2)
		ensure(len(w.contactEndEvents[0]) == 0)
		ensure(w.bodies[body1].headContactKey == NullIndex && w.bodies[body1].contactCount == 0)
		ensure(w.bodies[body0].headContactKey == 3 && w.bodies[body0].contactCount == 1)
		ensure(w.contacts[1].edges[1].nextKey == NullIndex)
		ensure(len(awake.contactIndices) == 1 && awake.contactIndices[0] == 1)
		ensure(w.contacts[1].localIndex == 0)
		ensure(w.contacts[0].contactId == NullIndex && w.contacts[0].setIndex == NullIndex)
		ensure(w.contacts[0].generation == 1)

		// The freed id is reused with the next generation
		createContact(w, &w.shapes[1], &w.shapes[0], 0)
		ensure(len(w.contacts) == 3)
		ensure(w.contacts[0].contactId == 0 && w.contacts[0].generation == 2)
		ensure(w.contacts[0].localIndex == 1)
		ensure(w.bodies[body0].headContactKey == 1 && w.contacts[1].edges[1].prevKey == 1)

		// Destroy the sleeping contact as if touching with events: end event
		w.endEventArrayIndex = 1
		w.contacts[2].flags |= contactTouchingFlag | contactEnableContactEvents
		destroyContact(w, &w.contacts[2], false)
		ensure(len(w.contactEndEvents[0]) == 0 && len(w.contactEndEvents[1]) == 1)
		ev := w.contactEndEvents[1][0]
		ensure(ev.ShapeIdA == ShapeID{7, 3, w.shapes[6].generation})
		ensure(ev.ShapeIdB == ShapeID{6, 3, w.shapes[5].generation})
		ensure(ev.ContactId == ContactID{Index1: 3, World0: 3, Generation: 1})

		// Destroy the remaining contacts
		destroyContact(w, &w.contacts[1], false)
		destroyContact(w, &w.contacts[0], false)
		ensure(w.broadPhase.pairSet.count == 0)
		ensure(len(awake.contactIndices) == 0 && len(w.solverSets[disabledSet].contactIndices) == 0)
		for i := range w.bodies {
			ensure(w.bodies[i].headContactKey == NullIndex && w.bodies[i].contactCount == 0)
		}
		ensure(len(w.contactEndEvents[1]) == 1)
	})

	It("UpdateConvex", func() {
		w := contactTestWorld()
		bodyA := contactTestBody(w, DynamicBody, awakeSet, 0)
		bodyB := contactTestBody(w, DynamicBody, awakeSet, 1)
		sA := contactTestShape(w, bodyA, SphereShape)
		sA.sphere = Sphere{Radius: 0.5}
		sA.material.Friction = 0.3
		sA.material.Restitution = 0.4
		sA.material.TangentVelocity = Vec3{1, 0, 0}
		sB := contactTestShape(w, bodyB, SphereShape)
		sB.sphere = Sphere{Radius: 0.5}
		sB.material.Friction = 0.7
		sB.material.Restitution = 0.2
		sB.flags |= uint8(enableHitEvents)

		createContact(w, sA, sB, 0)
		c := &w.contacts[0]
		a := createArena(1 << 12)

		xfA := WorldTransform{P: Pos{1, 2, 3}, Q: QuatIdentity}
		xfB := WorldTransform{P: Pos{1.9, 2, 3}, Q: QuatIdentity}
		localCenterA := Vec3{0.1, 0, 0}
		localCenterB := Vec3{0, 0.1, 0}

		touching := updateContact(w, 0, c, sA, localCenterA, xfA, sB, localCenterB, xfB, false, a)
		ensure(touching)
		ensure(c.manifoldCount == 1 && len(c.manifolds) >= 1)
		m := &c.manifolds[0]
		ensure(m.PointCount == 1)
		Expect(m.Normal.X).To(BeNumerically("~", 1.0, 1e-6))
		Expect(m.Points[0].Separation).To(BeNumerically("~", -0.1, 1e-5))
		ensure(m.Points[0].Persisted == false && m.Points[0].NormalImpulse == 0.0)
		ensure(m.Points[0].TriangleIndex == NullIndex)
		// Same world point from both bodies (anchors relative to the centers of mass)
		pa := Add(Add(ToVec3(xfA.P), localCenterA), m.Points[0].AnchorA)
		pb := Add(Add(ToVec3(xfB.P), localCenterB), m.Points[0].AnchorB)
		Expect(Length(Sub(pa, pb))).To(BeNumerically("<", 1e-5))
		ensure(c.friction == 0.7 && c.restitution == 0.2)
		ensure(c.rollingResistance == 0.0)
		ensure(c.tangentVelocity == Vec3{1, 0, 0})
		ensure(c.flags&simTouchingFlag != 0)
		ensure(c.flags&simEnableHitEvent != 0)

		// Warm starting carries the impulse of a matching feature
		m.Points[0].NormalImpulse = 2.5
		touching = updateContact(w, 0, c, sA, localCenterA, xfA, sB, localCenterB, xfB, false, a)
		ensure(touching)
		m = &c.manifolds[0]
		ensure(m.Points[0].Persisted && m.Points[0].NormalImpulse == 2.5)

		// Pre-solve can disable the contact
		c.flags |= simEnablePreSolveEvents | relativeTransformValid
		w.preSolveFcn = func(shapeIdA, shapeIdB ShapeID, point Pos, normal Vec3, context any) bool {
			ensure(shapeIdA == ShapeID{1, 3, sA.generation} && shapeIdB == ShapeID{2, 3, sB.generation})
			ensure(context == "ctx")
			return false
		}
		w.preSolveContext = "ctx"
		touching = updateContact(w, 0, c, sA, localCenterA, xfA, sB, localCenterB, xfB, false, a)
		ensure(touching == false)
		ensure(c.manifolds == nil && c.manifoldCount == 0)
		ensure(c.flags&(simTouchingFlag|relativeTransformValid) == 0)
		w.preSolveFcn = nil

		// Far apart: not touching
		xfB.P = Pos{5, 2, 3}
		touching = updateContact(w, 0, c, sA, localCenterA, xfA, sB, localCenterB, xfB, false, a)
		ensure(touching == false)
		ensure(c.manifolds == nil && c.manifoldCount == 0)
		ensure(c.flags&simTouchingFlag == 0)
	})

	It("UpdateHullsAndCompound", func() {
		w := contactTestWorld()
		box := MakeCubeHull(0.5)
		mat := DefaultSurfaceMaterial()
		mat.RollingResistance = 0.5

		// Compound on a static body: a sphere child and a hull child
		def := CompoundDef{}
		def.Spheres = []CompoundSphereDef{{Sphere: Sphere{Center: Vec3{-3, 0, 0}, Radius: 0.5}, Material: mat}}
		def.SphereCount = 1
		def.Hulls = []CompoundHullDef{{Hull: &box.Base, Transform: Transform{P: Vec3{3, 0, 0}, Q: QuatIdentity}, Material: mat}}
		def.HullCount = 1
		compound := CreateCompound(&def)

		bodyC := contactTestBody(w, StaticBody, staticSet, 0)
		bodyH := contactTestBody(w, DynamicBody, awakeSet, 0)
		bodyS := contactTestBody(w, DynamicBody, awakeSet, 1)
		sC := contactTestShape(w, bodyC, CompoundShape)
		sC.compound = compound
		sC.materials = []SurfaceMaterial{mat}
		sH := contactTestShape(w, bodyH, HullShape)
		sH.hull = &box.Base
		sS := contactTestShape(w, bodyS, SphereShape)
		sS.sphere = Sphere{Radius: 0.25}

		a := createArena(1 << 12)
		xfC := WorldTransform{P: Pos{0, 1, 0}, Q: QuatIdentity}

		sphereChild := -1
		hullChild := -1
		for i := 0; i < 2; i++ {
			if GetCompoundChild(compound, i).Type == SphereShape {
				sphereChild = i
			} else {
				hullChild = i
			}
		}
		ensure(sphereChild >= 0 && hullChild >= 0)

		check := func(c *contact, xfA, xfB WorldTransform, wantNormal Vec3) {
			ensure(c.manifoldCount == 1)
			m := &c.manifolds[0]
			ensure(m.PointCount >= 1)
			Expect(Dot(m.Normal, wantNormal)).To(BeNumerically(">", 0.999))
			for j := 0; j < int(m.PointCount); j++ {
				pa := Add(ToVec3(xfA.P), m.Points[j].AnchorA)
				pb := Add(ToVec3(xfB.P), m.Points[j].AnchorB)
				Expect(Length(Sub(pa, pb))).To(BeNumerically("<", 1e-5))
			}
		}

		// Compound sphere child vs hull: the child is flipped behind the hull, then flipped back
		createContact(w, sH, sC, sphereChild)
		c := &w.contacts[0]
		ensure(c.shapeIdA == sC.id && c.shapeIdB == sH.id && int(c.childIndex) == sphereChild)
		xfH := WorldTransform{P: Pos{-3, 1.95, 0}, Q: QuatIdentity}
		ensure(updateContact(w, 0, c, sC, Vec3{}, xfC, sH, Vec3{}, xfH, false, a))
		check(c, xfC, xfH, Vec3{0, 1, 0})
		// child radius 0.5, hull inner radius scaled by 0.25
		ensure(c.rollingResistance == 0.5*MaxFloat(0.5, 0.25*box.Base.InnerRadius))

		// Compound hull child vs sphere: child transform and anchor offset
		createContact(w, sC, sS, hullChild)
		c = &w.contacts[1]
		xfS := WorldTransform{P: Pos{3, 1.7, 0}, Q: QuatIdentity}
		ensure(updateContact(w, 0, c, sC, Vec3{}, xfC, sS, Vec3{}, xfS, false, a))
		check(c, xfC, xfS, Vec3{0, 1, 0})

		// Hull vs hull counts SAT calls
		bodyH2 := contactTestBody(w, DynamicBody, awakeSet, 2)
		sH2 := contactTestShape(w, bodyH2, HullShape)
		sH2.hull = &box.Base
		sH = &w.shapes[1]
		createContact(w, sH, sH2, 0)
		c = &w.contacts[len(w.contacts)-1]
		xfH2 := WorldTransform{P: Pos{-3, 2.9, 0}, Q: QuatIdentity}
		calls := w.taskContexts[0].satCallCount
		ensure(updateContact(w, 0, c, sH, Vec3{}, xfH, sH2, Vec3{}, xfH2, false, a))
		ensure(w.taskContexts[0].satCallCount == calls+1)
		check(c, xfH, xfH2, Vec3{0, 1, 0})
		ensure(c.manifolds[0].PointCount == 4)
	})
})
