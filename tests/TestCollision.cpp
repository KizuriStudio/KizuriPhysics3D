// KizuriPhysics - tests/TestCollision.cpp
#include "TestFramework.h"
#include "Kizuri/Collision/ContactManifold.h"
#include "Kizuri/Collision/EPA.h"

using namespace kizuri;

KZ_TEST(GJK_SphereDistance) {
    SphereShape a(1.0);
    SphereShape b(1.0);
    Transform ta(Quat::Identity(), Vec3(0, 0, 0));
    Transform tb(Quat::Identity(), Vec3(5, 0, 0));
    SupportMap map(a, ta, b, tb);
    GjkResult r = GJK::GetClosestPoints(map);
    CHECK_FALSE(r.intersect);
    CHECK_NEAR(r.distance, 3.0, 1e-3);
    CHECK_NEAR(r.normal.x, 1.0, 1e-2);
}

KZ_TEST(GJK_SphereIntersect) {
    SphereShape a(1.0);
    SphereShape b(1.0);
    Transform ta(Quat::Identity(), Vec3(0, 0, 0));
    Transform tb(Quat::Identity(), Vec3(1.0, 0, 0));
    SupportMap map(a, ta, b, tb);
    GjkResult r = GJK::GetClosestPoints(map);
    CHECK_TRUE(r.intersect);
}

KZ_TEST(EPA_SpherePenetration) {
    SphereShape a(1.0);
    SphereShape b(1.0);
    Transform ta(Quat::Identity(), Vec3(0, 0, 0));
    Transform tb(Quat::Identity(), Vec3(1.5, 0, 0));
    SupportMap map(a, ta, b, tb);
    GjkResult gjk = GJK::GetClosestPoints(map);
    CHECK_TRUE(gjk.intersect);
    EpaResult epa = EPA::Compute(map, gjk);
    CHECK_TRUE(epa.valid);
    // Two unit spheres 1.5 apart overlap by 0.5.
    CHECK_NEAR(epa.depth, 0.5, 1e-2);
    CHECK_NEAR(math::Abs(epa.normal.x), 1.0, 1e-2);
}

KZ_TEST(NarrowPhase_SphereSphereManifold) {
    SphereShape a(1.0);
    SphereShape b(1.0);
    Transform ta(Quat::Identity(), Vec3(0, 0, 0));
    Transform tb(Quat::Identity(), Vec3(1.8, 0, 0));
    CollideSettings settings;
    Manifold m;
    u32 n = NarrowPhase::Collide(a, ta, b, tb, settings, m);
    CHECK_EQ(n, 1u);
    if (n > 0) {
        CHECK_NEAR(m.points[0].separation, -0.2, 1e-2);
        // Normal points from B to A; A is at the origin and B is at +x.
        CHECK_NEAR(m.points[0].normal.x, -1.0, 1e-2);
    }
}

KZ_TEST(NarrowPhase_BoxOnPlane) {
    BoxShape box(Vec3(1, 1, 1));
    PlaneShape plane(Plane(Vec3::UnitY(), 0.0f));
    // Box center at y=0.9 -> penetrates the plane by 0.1.
    Transform tb(Quat::Identity(), Vec3(0, 0.9, 0));
    Transform tp(Quat::Identity(), Vec3(0, 0, 0));
    CollideSettings settings;
    Manifold m;
    u32 n = NarrowPhase::Collide(box, tb, plane, tp, settings, m);
    CHECK_EQ(n, 4u); // four bottom corners
    if (n > 0) {
        CHECK_NEAR(m.points[0].separation, -0.1, 1e-2);
    }
}

KZ_TEST(NarrowPhase_BoxBoxManifold) {
    BoxShape a(Vec3(1, 1, 1));
    BoxShape b(Vec3(1, 1, 1));
    Transform ta(Quat::Identity(), Vec3(0, 0, 0));
    Transform tb(Quat::Identity(), Vec3(0, 1.9, 0)); // overlap 0.1
    CollideSettings settings;
    Manifold m;
    u32 n = NarrowPhase::Collide(a, ta, b, tb, settings, m);
    CHECK(n >= 4u);
    CHECK(m.points[0].separation < 0);
}

KZ_TEST(NarrowPhase_SphereBox) {
    SphereShape s(0.5);
    BoxShape box(Vec3(1, 1, 1));
    Transform ts(Quat::Identity(), Vec3(1.4, 0, 0));
    Transform tb(Quat::Identity(), Vec3(0, 0, 0));
    CollideSettings settings;
    Manifold m;
    u32 n = NarrowPhase::Collide(s, ts, box, tb, settings, m);
    CHECK_EQ(n, 1u);
    if (n > 0) {
        // Sphere surface at x=0.9, box surface at x=1.0 -> overlap 0.1.
        CHECK_NEAR(m.points[0].separation, -0.1, 1e-2);
    }
}

KZ_TEST(NarrowPhase_Separated) {
    BoxShape a(Vec3(1, 1, 1));
    BoxShape b(Vec3(1, 1, 1));
    Transform ta(Quat::Identity(), Vec3(0, 0, 0));
    Transform tb(Quat::Identity(), Vec3(10, 0, 0));
    CollideSettings settings;
    Manifold m;
    u32 n = NarrowPhase::Collide(a, ta, b, tb, settings, m);
    CHECK_EQ(n, 0u);
}
