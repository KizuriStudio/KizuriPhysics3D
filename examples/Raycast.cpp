// KizuriPhysics - examples/Raycast.cpp
//
// Demonstrates the query API: ray casts, multi-hit ray casts, shape casts and
// overlap queries against a small scene.
#include "Kizuri/Kizuri.h"

#include <cstdio>

using namespace kizuri;

int main() {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81f, 0));

    // Static ground so the boxes come to rest at y = 0.5.
    {
        BodySettings g;
        g.shape = MakeRef<PlaneShape>(Plane(Vec3::UnitY(), 0));
        g.motionType = MotionType::Static;
        g.friction = 0.8f;
        world.CreateBody(g);
    }

    // A grid of boxes.
    for (int x = -2; x <= 2; ++x) {
        for (int z = -2; z <= 2; ++z) {
            BodySettings s;
            s.shape = MakeRef<BoxShape>(Vec3(0.5f, 0.5f, 0.5f));
            s.position = Vec3(Real(x) * 1.5f, 0.5f, Real(z) * 1.5f);
            s.motionType = MotionType::Dynamic;
            world.CreateBody(s);
        }
    }
    for (int i = 0; i < 120; ++i) world.StepFixed(Real(1.0 / 60.0));

    // 1. Simple ray cast straight down through the centre.
    {
        Ray ray(Vec3(0, 10, 0), Vec3(0, -1, 0));
        RayCastResult hit = world.RayCast(ray, 100.0f);
        if (hit.hit) {
            std::printf("Ray hit body at (%.2f, %.2f, %.2f), normal (%.2f, %.2f, %.2f)\n",
                        double(hit.position.x), double(hit.position.y), double(hit.position.z),
                        double(hit.normal.x), double(hit.normal.y), double(hit.normal.z));
        } else {
            std::printf("Ray missed\n");
        }
    }

    // 2. Multi-hit ray cast.
    {
        Ray ray(Vec3(0, 0.5f, 0), Vec3(1, 0, 0));
        RayCastResult hits[16];
        u32 n = world.RayCastAll(ray, 20.0f, hits, 16);
        std::printf("RayCastAll found %u bodies along +x\n", n);
    }

    // 3. Shape cast: sweep a sphere along a direction.
    {
        SphereShape probe(0.4f);
        Transform start(Quat::Identity(), Vec3(0, 3, 0));
        ShapeCastResult hit = world.CastShape(probe, start, Vec3(0, -1, 0), 10.0f);
        std::printf("Sphere cast %s (fraction %.3f)\n", hit.hit ? "hit" : "missed",
                    double(hit.fraction));
    }

    // 4. AABB overlap query.
    {
        AABB box(Vec3(-4, -1, -4), Vec3(4, 2, 4));
        BodyID ids[64];
        u32 n = world.QueryAABB(box, ids, 64);
        std::printf("QueryAABB returned %u bodies\n", n);
    }

    // 5. Point query.
    {
        BodyID id = world.QueryPoint(Vec3(0, 0.5f, 0));
        std::printf("QueryPoint at (0, 0.5, 0): %s\n", id.IsInvalid() ? "empty" : "hit");
    }
    return 0;
}
