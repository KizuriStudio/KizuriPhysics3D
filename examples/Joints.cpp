// KizuriPhysics - examples/Joints.cpp
//
// Demonstrates the joint constraints: a ball-socket chain, a hinge with a
// motor, and a slider. Runs headless and prints the resulting motion.
#include "Kizuri/Kizuri.h"

#include <cstdio>
#include <memory>

using namespace kizuri;

static BodyID AddStaticAnchor(PhysicsWorld& world, const Vec3& pos) {
    BodySettings s;
    s.shape = MakeRef<SphereShape>(0.1f);
    s.position = pos;
    s.motionType = MotionType::Static;
    return world.CreateBody(s);
}

static BodyID AddLink(PhysicsWorld& world, const Vec3& pos) {
    BodySettings s;
    s.shape = MakeRef<BoxShape>(Vec3(0.5f, 0.1f, 0.1f));
    s.position = pos;
    s.motionType = MotionType::Dynamic;
    s.friction = 0.3f;
    return world.CreateBody(s);
}

int main() {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81f, 0));

    // --- Ball-socket chain (pendulum) --------------------------------------
    BodyID anchor = AddStaticAnchor(world, Vec3(-6, 5, 0));
    BodyID prev = anchor;
    for (int i = 0; i < 5; ++i) {
        Vec3 pos(-6 + (i + 1) * 1.0f, 5, 0);
        BodyID link = AddLink(world, pos);
        world.AddConstraint(std::make_unique<PointConstraint>(
            world.GetBody(prev), world.GetBody(link), pos - Vec3(0.5f, 0, 0)));
        prev = link;
    }

    // --- Hinge with a motor -------------------------------------------------
    BodyID hingeAnchor = AddStaticAnchor(world, Vec3(0, 5, 0));
    BodyID blade = AddLink(world, Vec3(1, 5, 0));
    auto hinge = std::make_unique<HingeConstraint>(
        world.GetBody(hingeAnchor), world.GetBody(blade),
        Vec3(0, 5, 0), Vec3(0, 0, 1), Vec3(0, 0, 1));
    hinge->SetMotor(3.0f, 100.0f);
    world.AddConstraint(std::move(hinge));

    // --- Slider -------------------------------------------------------------
    BodyID railAnchor = AddStaticAnchor(world, Vec3(6, 5, 0));
    BodyID slider = AddLink(world, Vec3(7, 5, 0));
    auto slide = std::make_unique<SliderConstraint>(
        world.GetBody(railAnchor), world.GetBody(slider), Vec3(7, 5, 0), Vec3(1, 0, 0));
    slide->SetLimits(-2.0f, 2.0f);
    slide->SetMotor(1.0f, 50.0f);
    world.AddConstraint(std::move(slide));

    const Real dt = Real(1.0 / 60.0);
    for (int i = 0; i < 300; ++i) world.StepFixed(dt);

    std::printf("Ran %u constraints for 5 s\n", world.GetNumConstraints());
    std::printf("  step time: %.3f ms\n", double(world.GetStats().stepTimeMs));

    // Report the blade's angular velocity to show the motor is working.
    if (const Body* b = world.GetBody(blade)) {
        std::printf("  blade angular velocity: %.3f rad/s\n",
                    double(b->GetAngularVelocity().Length()));
    }
    if (const Body* b = world.GetBody(slider)) {
        std::printf("  slider position: %.3f\n", double(b->GetPosition().x - 6.0f));
    }
    return 0;
}
