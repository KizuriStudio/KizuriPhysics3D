// KizuriPhysics - tests/TestShapes.cpp
#include "TestFramework.h"
#include "Kizuri/Geometry/Shape.h"

using namespace kizuri;

KZ_TEST(Sphere_MassProperties) {
    SphereShape s(2.0);
    MassProperties mp = s.GetMassProperties(1.0);
    Real expectedMass = Real(4.0 / 3.0) * math::kPi * 8.0;
    CHECK_NEAR(mp.mass, expectedMass, 1e-3);
    Real expectedI = Real(0.4) * expectedMass * 4.0;
    CHECK_NEAR(mp.inertia[0].x, expectedI, 1e-2);
    CHECK_NEAR(mp.inertia[1].y, expectedI, 1e-2);
    CHECK_NEAR(mp.inertia[2].z, expectedI, 1e-2);
}

KZ_TEST(Box_MassProperties) {
    BoxShape b(1, 2, 3);
    MassProperties mp = b.GetMassProperties(1.0);
    CHECK_NEAR(mp.mass, 48.0, 1e-3);
    // Ixx = m/3*(hy^2+hz^2) = 16*(4+9) = 208
    CHECK_NEAR(mp.inertia[0].x, 208.0, 1e-2);
    CHECK_NEAR(mp.inertia[1].y, 16.0 * (1 + 9), 1e-2);
    CHECK_NEAR(mp.inertia[2].z, 16.0 * (1 + 4), 1e-2);
}

KZ_TEST(Box_RayCast) {
    BoxShape b(1, 1, 1);
    Ray ray(Vec3(-5, 0, 0), Vec3(1, 0, 0));
    Vec3 n;
    Real t = b.RayCastLocal(ray, 100.0, n);
    CHECK_NEAR(t, 4.0, 1e-4);
    CHECK_NEAR(n.x, -1.0, 1e-4);
}

KZ_TEST(Sphere_RayCast) {
    SphereShape s(1.0);
    Ray ray(Vec3(-5, 0, 0), Vec3(1, 0, 0));
    Vec3 n;
    Real t = s.RayCastLocal(ray, 100.0, n);
    CHECK_NEAR(t, 4.0, 1e-4);
    CHECK_NEAR(n.x, -1.0, 1e-4);
}

KZ_TEST(Capsule_VolumeAndMass) {
    CapsuleShape c(1.0, 0.5);
    Real expected = math::kPi * 0.25 * 2.0 + Real(4.0 / 3.0) * math::kPi * 0.125;
    CHECK_NEAR(c.GetVolume(), expected, 1e-3);
    MassProperties mp = c.GetMassProperties(1.0);
    CHECK_NEAR(mp.mass, expected, 1e-3);
    CHECK(mp.inertia[1].y > 0);
}

KZ_TEST(ConvexHull_FromCubePoints) {
    // 8 corners of a cube.
    Vec3 pts[8] = {
        {-1,-1,-1}, {1,-1,-1}, {-1,1,-1}, {1,1,-1},
        {-1,-1, 1}, {1,-1, 1}, {-1,1, 1}, {1,1, 1}
    };
    Ref<ConvexHullShape> hull = ConvexHullShape::Create(ArrayView<const Vec3>(pts, 8));
    CHECK(hull != nullptr);
    if (!hull) return;
    CHECK_EQ(hull->GetNumVertices(), 8u);
    CHECK_EQ(hull->GetNumFaces(), 12u); // 12 triangles
    CHECK_NEAR(hull->GetVolume(), 8.0, 1e-3);
    MassProperties mp = hull->GetMassProperties(1.0);
    CHECK_NEAR(mp.mass, 8.0, 1e-3);
    // Cube of half extent 1: Ixx = m/3*(1+1) = 8/3*2 = 5.333
    CHECK_NEAR(mp.inertia[0].x, 8.0 / 3.0 * 2.0, 1e-2);
}

KZ_TEST(ConvexHull_Support) {
    Vec3 pts[4] = { {0,0,0}, {1,0,0}, {0,1,0}, {0,0,1} };
    Ref<ConvexHullShape> hull = ConvexHullShape::Create(ArrayView<const Vec3>(pts, 4));
    CHECK(hull != nullptr);
    if (!hull) return;
    Vec3 s = hull->GetSupport(Vec3(1, 1, 1));
    CHECK_NEAR(s.Dot(Vec3(1, 1, 1)), 1.0, 1e-5);
}

KZ_TEST(Compound_MassProperties) {
    auto box = MakeRef<BoxShape>(Vec3(1, 1, 1));
    CompoundShape compound;
    compound.AddChild(Transform(Vec3(0, 0, 0)), box);
    compound.AddChild(Transform(Vec3(4, 0, 0)), box);
    compound.Finalize();
    MassProperties mp = compound.GetMassProperties(1.0);
    CHECK_NEAR(mp.mass, 16.0, 1e-2);
    // Center of mass midway between the two boxes.
    CHECK_NEAR(mp.centerOfMass.x, 2.0, 1e-3);
    CHECK_EQ(compound.GetNumChildren(), 2u);
}

KZ_TEST(HeightField_Interpolation) {
    // 3x3 grid, flat at height 1.
    Real heights[9] = { 1,1,1, 1,1,1, 1,1,1 };
    HeightFieldShape hf(3, 3, 2.0, 2.0, ArrayView<const Real>(heights, 9));
    CHECK_NEAR(hf.GetInterpolatedHeight(0, 0), 1.0, 1e-4);
    Vec3 n = hf.GetNormal(1, 1);
    CHECK_NEAR(n.y, 1.0, 1e-4);
}
