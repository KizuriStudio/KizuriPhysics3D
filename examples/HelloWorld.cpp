// KizuriPhysics - examples/HelloWorld.cpp
//
// The smallest possible simulation: a sphere falls onto a static ground plane
// and comes to rest. Run it to sanity-check the build.
#include "Kizuri/Kizuri.h"

#include <cstdio>

using namespace kizuri;

int main() {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81f, 0));

    // Static ground plane.
    BodySettings ground;
    ground.shape = MakeRef<PlaneShape>(Plane(Vec3::UnitY(), 0.0f));
    ground.motionType = MotionType::Static;
    ground.friction = 0.8f;
    world.CreateBody(ground);

    // Dynamic sphere.
    BodySettings sphere;
    sphere.shape = MakeRef<SphereShape>(0.5f);
    sphere.position = Vec3(0, 4, 0);
    sphere.motionType = MotionType::Dynamic;
    sphere.restitution = 0.4f;
    BodyID id = world.CreateBody(sphere);

    std::printf("Simulating a bouncing sphere for 5 seconds...\n");
    const Real dt = Real(1.0 / 60.0);
    for (int i = 0; i < 300; ++i) {
        world.StepFixed(dt);
        if (i % 30 == 0) {
            const Body* b = world.GetBody(id);
            std::printf("  t=%4.2fs  y=%7.3f  vy=%7.3f\n",
                        double(i * dt), double(b->GetPosition().y),
                        double(b->GetLinearVelocity().y));
        }
    }

    const Body* b = world.GetBody(id);
    std::printf("Final: y = %.3f (expected ~0.5), speed = %.3f\n",
                double(b->GetPosition().y), double(b->GetLinearVelocity().Length()));
    std::printf("Steps: %u bodies, %u pairs, %.3f ms/step\n",
                world.GetStats().numBodies, world.GetStats().numBroadPhasePairs,
                double(world.GetStats().stepTimeMs));
    return 0;
}
