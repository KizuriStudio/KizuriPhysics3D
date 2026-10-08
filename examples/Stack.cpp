// KizuriPhysics - examples/Stack.cpp
//
// Builds a tower of boxes and a pyramid, then lets them settle. Demonstrates
// stable stacking through warm-started sequential impulses.
#include "Kizuri/Kizuri.h"

#include <cstdio>

using namespace kizuri;

int main() {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81f, 0));

    WorldSettings& s = world.GetSettings();
    s.solver.velocityIterations = 12;
    s.solver.positionIterations = 4;

    // Ground.
    BodySettings ground;
    ground.shape = MakeRef<BoxShape>(Vec3(20, 0.5f, 20));
    ground.position = Vec3(0, -0.5f, 0);
    ground.motionType = MotionType::Static;
    ground.friction = 0.9f;
    world.CreateBody(ground);

    const Real hs = 0.5f; // half extent

    // Tower.
    const int towerHeight = 8;
    for (int i = 0; i < towerHeight; ++i) {
        BodySettings box;
        box.shape = MakeRef<BoxShape>(Vec3(hs, hs, hs));
        box.position = Vec3(-4, hs + i * (2 * hs), 0);
        box.motionType = MotionType::Dynamic;
        box.friction = 0.7f;
        box.restitution = 0.0f;
        world.CreateBody(box);
    }

    // Pyramid.
    const int base = 6;
    for (int layer = 0; layer < base; ++layer) {
        int count = base - layer;
        for (int i = 0; i < count; ++i) {
            BodySettings box;
            box.shape = MakeRef<BoxShape>(Vec3(hs, hs, hs));
            box.position = Vec3(4 + (i - (count - 1) * 0.5f) * (2 * hs + 0.02f),
                                hs + layer * (2 * hs + 0.02f), 0);
            box.motionType = MotionType::Dynamic;
            box.friction = 0.8f;
            box.restitution = 0.0f;
            world.CreateBody(box);
        }
    }

    const Real dt = Real(1.0 / 60.0);
    for (int i = 0; i < 600; ++i) world.StepFixed(dt);

    std::printf("Settled %u bodies in 10 s\n", world.GetStats().numBodies);
    std::printf("  active bodies : %u\n", world.GetStats().numActiveBodies);
    std::printf("  broadphase    : %u pairs\n", world.GetStats().numBroadPhasePairs);
    std::printf("  contacts      : %u\n", world.GetStats().numContactPoints);
    std::printf("  islands       : %u\n", world.GetStats().numIslands);
    std::printf("  step time     : %.3f ms\n", double(world.GetStats().stepTimeMs));
    return 0;
}
