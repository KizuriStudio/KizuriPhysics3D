// KizuriPhysics - tests/TestWorld.cpp
#include "TestFramework.h"
#include "Kizuri/Kizuri.h"

using namespace kizuri;

namespace {

BodyID AddGround(PhysicsWorld& world, Real y = 0) {
    BodySettings s;
    s.shape = MakeRef<PlaneShape>(Plane(Vec3::UnitY(), y));
    s.position = Vec3(0, y, 0);
    s.motionType = MotionType::Static;
    s.friction = 0.8;
    return world.CreateBody(s);
}

BodyID AddSphere(PhysicsWorld& world, const Vec3& pos, Real radius) {
    BodySettings s;
    s.shape = MakeRef<SphereShape>(radius);
    s.position = pos;
    s.motionType = MotionType::Dynamic;
    s.friction = 0.5;
    s.restitution = 0.0;
    return world.CreateBody(s);
}

BodyID AddBox(PhysicsWorld& world, const Vec3& pos, const Vec3& halfExtent) {
    BodySettings s;
    s.shape = MakeRef<BoxShape>(halfExtent);
    s.position = pos;
    s.motionType = MotionType::Dynamic;
    s.friction = 0.6;
    s.restitution = 0.0;
    return world.CreateBody(s);
}

void StepSeconds(PhysicsWorld& world, Real seconds, Real dt = Real(1.0 / 60.0)) {
    int steps = int(seconds / dt);
    for (int i = 0; i < steps; ++i) world.StepFixed(dt);
}

} // namespace

KZ_TEST(World_SphereFallsOntoPlane) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    AddGround(world);
    BodyID sphere = AddSphere(world, Vec3(0, 5, 0), 0.5);

    StepSeconds(world, 3.0);

    const Body* b = world.GetBody(sphere);
    CHECK(b != nullptr);
    if (b) {
        // The sphere should rest on the plane at y ~= radius.
        CHECK_NEAR(b->GetPosition().y, 0.5, 0.05);
        CHECK_NEAR(b->GetLinearVelocity().Length(), 0.0, 0.1);
    }
}

KZ_TEST(World_BoxFallsOntoPlane) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    AddGround(world);
    BodyID box = AddBox(world, Vec3(0, 3, 0), Vec3(0.5, 0.5, 0.5));

    StepSeconds(world, 3.0);

    const Body* b = world.GetBody(box);
    CHECK(b != nullptr);
    if (b) {
        CHECK_NEAR(b->GetPosition().y, 0.5, 0.1);
    }
}

KZ_TEST(World_BoxStackStable) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    AddGround(world);

    const int count = 5;
    BodyID boxes[count];
    for (int i = 0; i < count; ++i) {
        boxes[i] = AddBox(world, Vec3(0, 0.5 + i * 1.01, 0), Vec3(0.5, 0.5, 0.5));
    }

    StepSeconds(world, 4.0);

    for (int i = 0; i < count; ++i) {
        const Body* b = world.GetBody(boxes[i]);
        CHECK(b != nullptr);
        if (b) {
            Real expectedY = 0.5 + i * 1.01;
            CHECK_NEAR(b->GetPosition().y, expectedY, 0.2);
            CHECK_NEAR(b->GetLinearVelocity().Length(), 0.0, 0.2);
        }
    }
}

KZ_TEST(World_PyramidRest) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    AddGround(world);

    // 4-3-2-1 pyramid of unit boxes.
    int rows = 4;
    BodyID ids[16];
    int idx = 0;
    for (int row = 0; row < rows; ++row) {
        int n = rows - row;
        Real y = 0.5 + row * 1.02;
        for (int i = 0; i < n; ++i) {
            Real x = (i - (n - 1) * 0.5) * 1.02;
            ids[idx++] = AddBox(world, Vec3(x, y, 0), Vec3(0.5, 0.5, 0.5));
        }
    }
    StepSeconds(world, 4.0);

    // No body should have fallen below the ground.
    for (int i = 0; i < idx; ++i) {
        const Body* b = world.GetBody(ids[i]);
        CHECK(b != nullptr);
        if (b) CHECK(b->GetPosition().y > 0.0);
    }
}

KZ_TEST(World_Determinism) {
    auto build = [](PhysicsWorld& world) {
        world.SetGravity(Vec3(0, -9.81, 0));
        AddGround(world);
        for (int i = 0; i < 20; ++i) {
            Real x = (i % 5) * 1.5 - 3.0;
            Real z = (i / 5) * 1.5 - 3.0;
            AddBox(world, Vec3(x, 2.0 + i * 0.5, z), Vec3(0.4, 0.4, 0.4));
        }
    };

    PhysicsWorld a, b;
    build(a);
    build(b);
    StepSeconds(a, 2.0);
    StepSeconds(b, 2.0);
    CHECK_EQ(a.ComputeStateHash(), b.ComputeStateHash());
}

KZ_TEST(World_RayCast) {
    PhysicsWorld world;
    AddGround(world);
    BodyID box = AddBox(world, Vec3(0, 2, 0), Vec3(1, 1, 1));

    Ray ray(Vec3(0, 10, 0), Vec3(0, -1, 0));
    RayCastResult hit = world.RayCast(ray, 100.0);
    CHECK_TRUE(hit.hit);
    if (hit.hit) {
        CHECK_EQ(hit.body, box);
        // Box top surface is at y = 3.
        CHECK_NEAR(hit.position.y, 3.0, 0.1);
    }
}

KZ_TEST(World_CastShape) {
    PhysicsWorld world;
    AddGround(world);
    BodyID box = AddBox(world, Vec3(0, 2, 0), Vec3(1, 1, 1));

    // Sweep a sphere straight down from y = 10 over 10 units. The box top is
    // at y = 3, so a radius-0.5 sphere stops with its centre at y = 3.5,
    // i.e. a fraction of (10 - 3.5) / 10 = 0.65.
    SphereShape probe(Real(0.5));
    Transform start(Quat::Identity(), Vec3(0, 10, 0));
    ShapeCastResult hit = world.CastShape(probe, start, Vec3(0, -1, 0), Real(10.0));
    CHECK_TRUE(hit.hit);
    if (hit.hit) {
        CHECK_EQ(hit.body, box);
        CHECK_NEAR(hit.fraction, 0.65, 0.05);
    }

    // Sweeping upwards, away from everything, must miss.
    ShapeCastResult miss = world.CastShape(probe, start, Vec3(0, 1, 0), Real(5.0));
    CHECK_FALSE(miss.hit);
}

KZ_TEST(World_PointConstraintPendulum) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));

    // Static anchor.
    BodySettings anchorSettings;
    anchorSettings.shape = MakeRef<SphereShape>(0.1);
    anchorSettings.position = Vec3(0, 5, 0);
    anchorSettings.motionType = MotionType::Static;
    BodyID anchor = world.CreateBody(anchorSettings);

    // Dynamic bob.
    BodyID bob = AddSphere(world, Vec3(2, 5, 0), 0.3);

    auto constraint = std::make_unique<PointConstraint>(
        world.GetBody(anchor), world.GetBody(bob), Vec3(0, 5, 0));
    world.AddConstraint(std::move(constraint));

    StepSeconds(world, 2.0);

    const Body* b = world.GetBody(bob);
    CHECK(b != nullptr);
    if (b) {
        // Distance from the anchor must stay at the rod length (2 units).
        Real dist = (b->GetPosition() - Vec3(0, 5, 0)).Length();
        CHECK_NEAR(dist, 2.0, 0.15);
    }
}

KZ_TEST(World_DistanceConstraint) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));

    BodyID a = AddSphere(world, Vec3(0, 5, 0), 0.2);
    BodyID b = AddSphere(world, Vec3(0, 3, 0), 0.2);
    world.GetBody(a)->SetMotionType(MotionType::Static);

    auto constraint = std::make_unique<DistanceConstraint>(
        world.GetBody(a), world.GetBody(b), Vec3(0, 5, 0), Vec3(0, 3, 0), 2.0);
    world.AddConstraint(std::move(constraint));

    StepSeconds(world, 2.0);

    const Body* bb = world.GetBody(b);
    if (bb) {
        Real dist = (bb->GetPosition() - Vec3(0, 5, 0)).Length();
        CHECK_NEAR(dist, 2.0, 0.15);
    }
}

KZ_TEST(World_Sleeping) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    AddGround(world);
    BodyID box = AddBox(world, Vec3(0, 0.5, 0), Vec3(0.5, 0.5, 0.5));

    StepSeconds(world, 3.0);
    const Body* b = world.GetBody(box);
    CHECK(b != nullptr);
    if (b) {
        CHECK_FALSE(b->IsActive()); // should have gone to sleep
    }
}
