// KizuriPhysics - examples/Character.cpp
// Walks a capsule character across a platform and up a step.
#include "Kizuri/Kizuri.h"

#include <cstdio>

using namespace kizuri;

int main() {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));

    // Ground.
    BodySettings ground;
    ground.shape = MakeRef<PlaneShape>(Plane(Vec3::UnitY(), 0));
    ground.motionType = MotionType::Static;
    ground.friction = Real(0.9);
    world.CreateBody(ground);

    // A low ledge to step onto.
    BodySettings ledge;
    ledge.shape = MakeRef<BoxShape>(Vec3(3, 0.2, 2));
    ledge.position = Vec3(6, 0.2, 0);
    ledge.motionType = MotionType::Static;
    world.CreateBody(ledge);

    CharacterController character(world);
    character.SetPosition(Vec3(0, 1.0, 0));
    character.SetHorizontalVelocity(Vec3(3, 0, 0));

    std::printf("Walking a character forward...\n");
    for (int i = 0; i < 300; ++i) {
        character.Update(Real(1.0 / 60.0));
        if (i % 60 == 0) {
            const Vec3 p = character.GetPosition();
            std::printf("  t=%.1fs  pos=(%.2f, %.2f, %.2f)  grounded=%d\n",
                        Real(i) / Real(60), p.x, p.y, p.z,
                        character.IsGrounded() ? 1 : 0);
        }
    }

    const Vec3 finalPos = character.GetPosition();
    std::printf("Final position: (%.2f, %.2f, %.2f)\n", finalPos.x, finalPos.y, finalPos.z);
    std::printf("Climbed onto the ledge: %s\n", finalPos.y > Real(0.5) ? "yes" : "no");
    return 0;
}
