// KizuriPhysics - examples/CCD.cpp
// Fires a fast bullet at a thin wall with and without continuous collision
// detection to show tunnelling prevention.
#include "Kizuri/Kizuri.h"

#include <cstdio>

using namespace kizuri;

namespace {

Real FireBullet(MotionQuality quality) {
    PhysicsWorld world;
    world.SetGravity(Vec3::Zero());

    BodySettings wall;
    wall.shape = MakeRef<BoxShape>(Vec3(0.05, 5, 5));
    wall.motionType = MotionType::Static;
    world.CreateBody(wall);

    BodySettings bullet;
    bullet.shape = MakeRef<SphereShape>(Real(0.1));
    bullet.position = Vec3(-5, 0, 0);
    bullet.motionType = MotionType::Dynamic;
    bullet.motionQuality = quality;
    bullet.linearVelocity = Vec3(120, 0, 0);
    bullet.gravityFactor = 0;
    const BodyID id = world.CreateBody(bullet);

    for (int i = 0; i < 10; ++i) world.StepFixed(Real(1.0 / 60.0));
    return world.GetBody(id)->GetPosition().x;
}

} // namespace

int main() {
    const Real discrete = FireBullet(MotionQuality::Discrete);
    const Real continuous = FireBullet(MotionQuality::LinearCast);

    std::printf("Bullet fired at 120 m/s towards a 0.1 m thick wall at x = 0\n");
    std::printf("  Discrete   motion: final x = %7.3f  (%s)\n", discrete,
                discrete > Real(0.5) ? "tunnelled through" : "stopped");
    std::printf("  LinearCast (CCD) : final x = %7.3f  (%s)\n", continuous,
                continuous > Real(0.5) ? "tunnelled through" : "stopped");
    return 0;
}
