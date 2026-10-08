// KizuriPhysics - examples/DebugDraw.cpp
//
// Shows how to consume the engine's debug output. The CountingRenderer below
// implements DebugRenderer and simply tallies the primitives; a real host
// would forward DrawLine/DrawTriangle to its graphics API.
#include "Kizuri/Kizuri.h"

#include <cstdio>

using namespace kizuri;

class CountingRenderer final : public DebugRenderer {
public:
    u64 lines = 0;
    u64 triangles = 0;

    void DrawLine(const Vec3&, const Vec3&, const DebugColor&) override { ++lines; }
    void DrawTriangle(const Vec3&, const Vec3&, const Vec3&, const DebugColor&) override { ++triangles; }
};

int main() {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81f, 0));

    BodySettings ground;
    ground.shape = MakeRef<BoxShape>(Vec3(10, 0.5f, 10));
    ground.position = Vec3(0, -0.5f, 0);
    ground.motionType = MotionType::Static;
    world.CreateBody(ground);

    // A mixed pile: spheres, boxes and a convex hull.
    for (int i = 0; i < 12; ++i) {
        BodySettings s;
        if (i % 3 == 0) {
            s.shape = MakeRef<SphereShape>(0.4f);
        } else if (i % 3 == 1) {
            s.shape = MakeRef<BoxShape>(Vec3(0.4f, 0.4f, 0.4f));
        } else {
            Vec3 pts[8] = {
                {-0.4f, -0.4f, -0.4f}, {0.4f, -0.4f, -0.4f}, {-0.4f, 0.4f, -0.4f}, {0.4f, 0.4f, -0.4f},
                {-0.4f, -0.4f,  0.4f}, {0.4f, -0.4f,  0.4f}, {-0.4f, 0.4f,  0.4f}, {0.4f, 0.4f,  0.4f}
            };
            s.shape = ConvexHullShape::Create(ArrayView<const Vec3>(pts, 8));
        }
        s.position = Vec3(Real(i % 4) * 1.0f - 1.5f, 1.0f + Real(i / 4) * 1.0f, 0);
        s.motionType = MotionType::Dynamic;
        world.CreateBody(s);
    }

    for (int i = 0; i < 180; ++i) world.StepFixed(Real(1.0 / 60.0));

    CountingRenderer renderer;
    world.DrawDebug(renderer, /*shapes*/ true, /*contacts*/ true, /*aabbs*/ false, /*joints*/ false);
    std::printf("Debug draw produced %llu lines and %llu triangles\n",
                (unsigned long long)renderer.lines, (unsigned long long)renderer.triangles);
    std::printf("  bodies: %u, contacts: %u\n",
                world.GetStats().numBodies, world.GetStats().numContactPoints);
    return 0;
}
