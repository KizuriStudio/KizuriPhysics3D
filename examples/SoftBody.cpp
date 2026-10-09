// KizuriPhysics - examples/SoftBody.cpp
// Drops a soft lattice box and hangs a cloth.
#include "Kizuri/Kizuri.h"

#include <cstdio>

using namespace kizuri;

int main() {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81, 0));

    BodySettings ground;
    ground.shape = MakeRef<PlaneShape>(Plane(Vec3::UnitY(), 0));
    ground.motionType = MotionType::Static;
    world.CreateBody(ground);

    // Soft box.
    SoftBody box(world);
    box.GetSettings().particleRadius = Real(0.1);
    box.CreateBox(Vec3(-2, 2.0, 0), Vec3(0.5, 0.5, 0.5), 3);

    // Hanging cloth (first row pinned).
    SoftBody cloth(world);
    cloth.CreateCloth(Vec3(2, 3.0, 0), Real(1.5), Real(1.5), 6, 6);

    for (int i = 0; i < 240; ++i) {
        box.Update(Real(1.0 / 60.0));
        cloth.Update(Real(1.0 / 60.0));
    }

    Real lowestBox = Real(1.0e30);
    for (u32 i = 0; i < box.GetNumParticles(); ++i) {
        lowestBox = math::Min(lowestBox, box.GetParticlePosition(i).y);
    }

    Real lowestCloth = Real(1.0e30);
    for (u32 i = 0; i < cloth.GetNumParticles(); ++i) {
        lowestCloth = math::Min(lowestCloth, cloth.GetParticlePosition(i).y);
    }

    std::printf("Soft box : %u particles, %u edges, lowest y = %.3f\n",
                box.GetNumParticles(), box.GetNumEdges(), lowestBox);
    std::printf("Cloth    : %u particles, %u edges, lowest y = %.3f\n",
                cloth.GetNumParticles(), cloth.GetNumEdges(), lowestCloth);
    std::printf("Box settled above ground: %s\n", lowestBox > Real(-0.2) ? "yes" : "no");
    return 0;
}
