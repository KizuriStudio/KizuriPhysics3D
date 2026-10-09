// KizuriPhysics - Vehicle/Vehicle.h
// Raycast wheeled vehicle: per-wheel suspension, steering, engine and braking.
#pragma once

#include "Kizuri/World/PhysicsWorld.h"

namespace kizuri {

// ---------------------------------------------------------------------------
// One wheel.
// ---------------------------------------------------------------------------
struct WheelSettings {
    /// Attachment point in chassis-local space (top of the suspension).
    Vec3 localPosition = Vec3(Real(0), Real(-0.3), Real(0));
    Real radius = Real(0.3);
    /// Distance the suspension can extend from the attachment point.
    Real suspensionRestLength = Real(0.3);
    /// Spring stiffness (N/m).
    Real suspensionStiffness = Real(30000);
    /// Damper coefficient (N*s/m).
    Real suspensionDamping = Real(2000);
    /// Clamp on the suspension force (N).
    Real maxSuspensionForce = Real(100000);
    /// Tire friction coefficient.
    Real friction = Real(1.0);
    /// Sideways grip stiffness (N per m/s).
    Real lateralStiffness = Real(8000);
    bool steerable = false;
    bool driven = false;
};

// ---------------------------------------------------------------------------
// Vehicle configuration.
// ---------------------------------------------------------------------------
struct VehicleSettings {
    Vec3 forward = Vec3(Real(0), Real(0), Real(1));
    Vec3 up = Vec3(Real(0), Real(1), Real(0));
    /// Maximum steering angle in radians.
    Real maxSteerAngle = Real(0.5);
    /// Engine force at full throttle (N).
    Real engineForce = Real(6000);
    /// Brake force at full braking (N).
    Real brakeForce = Real(4000);
    /// How fast the steering angle approaches the target (1/s).
    Real steerSpeed = Real(6.0);
};

// ---------------------------------------------------------------------------
// Vehicle
// ---------------------------------------------------------------------------
class Vehicle {
public:
    Vehicle(PhysicsWorld& world, BodyID chassis, const VehicleSettings& settings = VehicleSettings());

    u32 AddWheel(const WheelSettings& settings);
    u32 GetNumWheels() const { return u32(mWheels.Size()); }

    /// Normalized steering input in [-1, 1].
    void SetSteer(Real steer) { mSteerInput = math::Clamp(steer, Real(-1), Real(1)); }
    /// Normalized throttle in [-1, 1].
    void SetThrottle(Real throttle) { mThrottle = math::Clamp(throttle, Real(-1), Real(1)); }
    /// Normalized brake in [0, 1].
    void SetBrake(Real brake) { mBrake = math::Clamp(brake, Real(0), Real(1)); }

    /// Apply suspension, engine and tire forces for this step. Call before
    /// PhysicsWorld::Step().
    void Update(Real dt);

    bool IsWheelGrounded(u32 index) const;
    /// Current suspension compression in metres (0 = fully extended).
    Real GetSuspensionCompression(u32 index) const;
    /// Angular speed of a wheel in rad/s (for rendering).
    Real GetWheelAngularSpeed(u32 index) const;
    /// World-space transform of a wheel (for rendering).
    Transform GetWheelTransform(u32 index) const;

    BodyID GetChassis() const { return mChassis; }

private:
    struct Wheel {
        WheelSettings settings;
        bool grounded = false;
        Real compression = Real(0);
        Real angularSpeed = Real(0);
        Real rotationAngle = Real(0);
        Vec3 contactPoint = Vec3::Zero();
        Vec3 contactNormal = Vec3::UnitY();
    };

    PhysicsWorld& mWorld;
    BodyID mChassis;
    VehicleSettings mSettings;
    Vector<Wheel> mWheels;

    Real mSteerInput = Real(0);
    Real mThrottle = Real(0);
    Real mBrake = Real(0);
    Real mCurrentSteer = Real(0);
};

} // namespace kizuri
