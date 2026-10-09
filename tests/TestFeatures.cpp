// KizuriPhysics - tests/TestFeatures.cpp
// Tests for the higher-level systems: character controller, vehicle, soft
// bodies and serialization.
#include "TestFramework.h"
#include "Kizuri/Kizuri.h"

using namespace kizuri;

namespace {

BodyID MakeGround(PhysicsWorld& world) {
    BodySettings s;
    s.shape = MakeRef<PlaneShape>(Plane(Vec3::UnitY(), 0));
    s.motionType = MotionType::Static;
    s.friction = Real(0.9);
    return world.CreateBody(s);
}

BodyID MakeStaticBox(PhysicsWorld& world, const Vec3& pos, const Vec3& half) {
    BodySettings s;
    s.shape = MakeRef<BoxShape>(half);
    s.position = pos;
    s.motionType = MotionType::Static;
    s.friction = Real(0.8);
    return world.CreateBody(s);
}

} // namespace

// ===========================================================================
// Serialization
// ===========================================================================
KZ_TEST(Serialization_RoundTrip) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    MakeGround(world);

    // A mix of shapes and motion types.
    BodySettings box;
    box.shape = MakeRef<BoxShape>(Vec3(0.5, 0.5, 0.5));
    box.position = Vec3(0, 4, 0);
    box.motionType = MotionType::Dynamic;
    world.CreateBody(box);

    BodySettings sphere;
    sphere.shape = MakeRef<SphereShape>(Real(0.4));
    sphere.position = Vec3(1, 6, 0);
    sphere.motionType = MotionType::Dynamic;
    sphere.linearVelocity = Vec3(1, 0, 0);
    world.CreateBody(sphere);

    BodySettings kinematic;
    kinematic.shape = MakeRef<CapsuleShape>(Real(0.5), Real(0.2));
    kinematic.position = Vec3(-2, 2, 0);
    kinematic.motionType = MotionType::Kinematic;
    world.CreateBody(kinematic);

    for (int i = 0; i < 30; ++i) world.Step(Real(1.0 / 60.0));

    Vector<u8> buffer;
    CHECK_TRUE(SerializeWorld(world, buffer));
    CHECK_TRUE(buffer.Size() > 0);

    PhysicsWorld loaded;
    loaded.SetGravity(Vec3(0, -9.81, 0));
    CHECK_TRUE(DeserializeWorld(loaded, buffer.Data(), buffer.Size()));

    // Continue both simulations; they must stay in lockstep.
    for (int i = 0; i < 60; ++i) {
        world.Step(Real(1.0 / 60.0));
        loaded.Step(Real(1.0 / 60.0));
    }

    const BodyManager& a = world.GetBodyManager();
    const BodyManager& b = loaded.GetBodyManager();
    CHECK_EQ(a.GetBodySlotCount(), b.GetBodySlotCount());
    for (u32 i = 0; i < a.GetBodySlotCount(); ++i) {
        const Body* ba = a.GetBodyBySlot(i);
        const Body* bb = b.GetBodyBySlot(i);
        CHECK_TRUE((ba == nullptr) == (bb == nullptr));
        if (ba && bb) {
            CHECK_NEAR((ba->GetPosition() - bb->GetPosition()).Length(), 0.0, 1.0e-3);
            CHECK_NEAR((ba->GetLinearVelocity() - bb->GetLinearVelocity()).Length(), 0.0, 1.0e-3);
        }
    }
}

// ===========================================================================
// Soft body
// ===========================================================================
KZ_TEST(SoftBody_ClothHangsUnderGravity) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));

    SoftBody cloth(world);
    // A 5x5 cloth hanging in the XZ plane; the first row is pinned.
    cloth.CreateCloth(Vec3(0, 3, 0), Real(1.0), Real(1.0), 5, 5);

    const Real pinnedY = cloth.GetParticlePosition(0).y;
    const u32 last = cloth.GetNumParticles() - 1;
    const Real startBottomY = cloth.GetParticlePosition(last).y;

    for (int i = 0; i < 120; ++i) cloth.Update(Real(1.0 / 60.0));

    CHECK_NEAR(cloth.GetParticlePosition(0).y, pinnedY, 0.01); // pinned
    CHECK_TRUE(cloth.GetParticlePosition(last).y < startBottomY - Real(0.2));
}

KZ_TEST(SoftBody_BoxRestsOnGround) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    MakeGround(world);

    SoftBody soft(world);
    soft.GetSettings().particleRadius = Real(0.1);
    soft.CreateBox(Vec3(0, 1.5, 0), Vec3(0.4, 0.4, 0.4), 3);

    for (int i = 0; i < 240; ++i) soft.Update(Real(1.0 / 60.0));

    // No particle should be far below the ground.
    Real lowest = Real(1.0e30);
    for (u32 i = 0; i < soft.GetNumParticles(); ++i) {
        lowest = math::Min(lowest, soft.GetParticlePosition(i).y);
    }
    CHECK_TRUE(lowest > Real(-0.2));
}

// ===========================================================================
// Vehicle
// ===========================================================================
KZ_TEST(Vehicle_DrivesForward) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    MakeGround(world);

    BodySettings chassisSettings;
    chassisSettings.shape = MakeRef<BoxShape>(Vec3(1, 0.3, 2));
    chassisSettings.position = Vec3(0, 1.2, 0);
    chassisSettings.motionType = MotionType::Dynamic;
    chassisSettings.massOverride = Real(800);
    chassisSettings.friction = Real(0.5);
    BodyID chassis = world.CreateBody(chassisSettings);

    VehicleSettings vehicleSettings;
    vehicleSettings.forward = Vec3(0, 0, 1);
    vehicleSettings.up = Vec3(0, 1, 0);
    Vehicle vehicle(world, chassis, vehicleSettings);

    const Real wheelX = Real(0.9);
    const Real wheelZ = Real(1.3);
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            WheelSettings wheel;
            wheel.localPosition = Vec3(Real(sx) * wheelX, Real(-0.3), Real(sz) * wheelZ);
            wheel.radius = Real(0.35);
            wheel.suspensionRestLength = Real(0.4);
            wheel.steerable = (sz > 0);
            wheel.driven = true;
            vehicle.AddWheel(wheel);
        }
    }

    // Let the suspension settle, then drive.
    for (int i = 0; i < 60; ++i) {
        vehicle.Update(Real(1.0 / 60.0));
        world.Step(Real(1.0 / 60.0));
    }
    const Body* before = world.GetBody(chassis);
    const Real startZ = before ? before->GetPosition().z : Real(0);

    vehicle.SetThrottle(Real(1.0));
    for (int i = 0; i < 240; ++i) {
        vehicle.Update(Real(1.0 / 60.0));
        world.Step(Real(1.0 / 60.0));
    }

    const Body* after = world.GetBody(chassis);
    CHECK(after != nullptr);
    if (after) {
        CHECK_TRUE(after->GetPosition().z > startZ + Real(5.0));
    }
    CHECK_TRUE(vehicle.IsWheelGrounded(0));
}

// ===========================================================================
// Character controller
// ===========================================================================
KZ_TEST(Character_StandsAndWalks) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    MakeGround(world);

    CharacterController character(world);
    character.SetPosition(Vec3(0, 1.5, 0));

    for (int i = 0; i < 180; ++i) character.Update(Real(1.0 / 60.0));
    CHECK_TRUE(character.IsGrounded());
    const Real restY = character.GetPosition().y;

    character.SetHorizontalVelocity(Vec3(2, 0, 0));
    for (int i = 0; i < 90; ++i) character.Update(Real(1.0 / 60.0));

    CHECK_TRUE(character.GetPosition().x > Real(1.5));
    CHECK_NEAR(character.GetPosition().y, restY, 0.1);
    CHECK_TRUE(character.IsGrounded());
}

KZ_TEST(Character_BlockedByWall) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    MakeGround(world);
    MakeStaticBox(world, Vec3(2, 2, 0), Vec3(0.5, 2, 2));

    CharacterController character(world);
    character.SetPosition(Vec3(0, 1.5, 0));
    for (int i = 0; i < 120; ++i) character.Update(Real(1.0 / 60.0));

    character.SetHorizontalVelocity(Vec3(6, 0, 0));
    for (int i = 0; i < 180; ++i) character.Update(Real(1.0 / 60.0));

    // The character must stop before reaching the wall centre (x = 2).
    CHECK_TRUE(character.GetPosition().x < Real(2.0));
}

KZ_TEST(Character_StepsOntoLowLedge) {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));
    MakeGround(world);
    // A low ledge (0.25 m tall) spanning x in [1, 5].
    MakeStaticBox(world, Vec3(3, 0.125, 0), Vec3(2, 0.125, 1));

    CharacterController character(world);
    character.SetPosition(Vec3(0, 1.0, 0));
    for (int i = 0; i < 120; ++i) character.Update(Real(1.0 / 60.0));

    character.SetHorizontalVelocity(Vec3(2, 0, 0));
    for (int i = 0; i < 120; ++i) character.Update(Real(1.0 / 60.0));

    // Should have climbed onto the ledge.
    CHECK_TRUE(character.GetPosition().x > Real(2.0));
    CHECK_TRUE(character.GetPosition().y > Real(0.4));
}
