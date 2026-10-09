// KizuriPhysics - examples/Vehicle.cpp
// Drives a four-wheeled raycast vehicle forward and then steers.
#include "Kizuri/Kizuri.h"

#include <cstdio>

using namespace kizuri;

int main() {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));

    BodySettings ground;
    ground.shape = MakeRef<PlaneShape>(Plane(Vec3::UnitY(), 0));
    ground.motionType = MotionType::Static;
    ground.friction = Real(0.9);
    world.CreateBody(ground);

    BodySettings chassisSettings;
    chassisSettings.shape = MakeRef<BoxShape>(Vec3(1, 0.3, 2));
    chassisSettings.position = Vec3(0, 1.2, 0);
    chassisSettings.motionType = MotionType::Dynamic;
    chassisSettings.massOverride = Real(800);
    chassisSettings.friction = Real(0.5);
    const BodyID chassis = world.CreateBody(chassisSettings);

    VehicleSettings vs;
    vs.forward = Vec3(0, 0, 1);
    vs.up = Vec3(0, 1, 0);
    Vehicle vehicle(world, chassis, vs);

    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            WheelSettings wheel;
            wheel.localPosition = Vec3(Real(sx) * Real(0.9), Real(-0.3), Real(sz) * Real(1.3));
            wheel.radius = Real(0.35);
            wheel.suspensionRestLength = Real(0.4);
            wheel.steerable = (sz > 0);
            wheel.driven = true;
            vehicle.AddWheel(wheel);
        }
    }

    // Settle the suspension.
    for (int i = 0; i < 60; ++i) {
        vehicle.Update(Real(1.0 / 60.0));
        world.Step(Real(1.0 / 60.0));
    }

    vehicle.SetThrottle(Real(1.0));
    std::printf("Driving forward...\n");
    for (int i = 0; i < 180; ++i) {
        vehicle.Update(Real(1.0 / 60.0));
        world.Step(Real(1.0 / 60.0));
        if (i % 60 == 0) {
            const Body* b = world.GetBody(chassis);
            std::printf("  t=%.1fs  z=%.2f  speed=%.2f  grounded=%d\n",
                        Real(i) / Real(60), b->GetPosition().z,
                        b->GetLinearVelocity().Length(),
                        vehicle.IsWheelGrounded(0) ? 1 : 0);
        }
    }

    vehicle.SetThrottle(Real(0.5));
    vehicle.SetSteer(Real(1.0));
    std::printf("Steering right...\n");
    for (int i = 0; i < 120; ++i) {
        vehicle.Update(Real(1.0 / 60.0));
        world.Step(Real(1.0 / 60.0));
    }

    const Body* b = world.GetBody(chassis);
    std::printf("Final position: (%.2f, %.2f, %.2f)\n",
                b->GetPosition().x, b->GetPosition().y, b->GetPosition().z);
    return 0;
}
