// KizuriPhysics - Vehicle/Vehicle.cpp
#include "Kizuri/Vehicle/Vehicle.h"

namespace kizuri {

Vehicle::Vehicle(PhysicsWorld& world, BodyID chassis, const VehicleSettings& settings)
    : mWorld(world), mChassis(chassis), mSettings(settings) {}

u32 Vehicle::AddWheel(const WheelSettings& settings) {
    Wheel w;
    w.settings = settings;
    mWheels.PushBack(w);
    return u32(mWheels.Size()) - 1;
}

bool Vehicle::IsWheelGrounded(u32 index) const {
    return index < mWheels.Size() && mWheels[index].grounded;
}

Real Vehicle::GetSuspensionCompression(u32 index) const {
    return index < mWheels.Size() ? mWheels[index].compression : Real(0);
}

Real Vehicle::GetWheelAngularSpeed(u32 index) const {
    return index < mWheels.Size() ? mWheels[index].angularSpeed : Real(0);
}

Transform Vehicle::GetWheelTransform(u32 index) const {
    if (index >= mWheels.Size()) return Transform();
    const Wheel& w = mWheels[index];
    const Body* chassis = mWorld.GetBody(mChassis);
    if (!chassis) return Transform();
    const Transform& t = chassis->GetTransform();
    const Quat& rot = chassis->GetRotation();
    Vec3 up = rot.Rotate(mSettings.up).NormalizedOr(Vec3::UnitY());
    Vec3 attach = t * w.settings.localPosition;
    Real suspensionLength = w.settings.suspensionRestLength - w.compression;
    Vec3 center = attach - up * suspensionLength;
    return Transform(rot, center);
}

void Vehicle::Update(Real dt) {
    if (dt <= Real(0)) return;
    Body* chassis = mWorld.GetBody(mChassis);
    if (!chassis) return;

    const Transform& t = chassis->GetTransform();
    const Quat& rot = chassis->GetRotation();

    // Smooth the steering towards the input.
    const Real targetSteer = mSteerInput * mSettings.maxSteerAngle;
    const Real alpha = math::Clamp(mSettings.steerSpeed * dt, Real(0), Real(1));
    mCurrentSteer += (targetSteer - mCurrentSteer) * alpha;

    const Vec3 up = rot.Rotate(mSettings.up).NormalizedOr(Vec3::UnitY());
    const Vec3 forward = rot.Rotate(mSettings.forward).NormalizedOr(Vec3::UnitZ());

    for (u32 i = 0; i < u32(mWheels.Size()); ++i) {
        Wheel& w = mWheels[i];
        const Vec3 attach = t * w.settings.localPosition;
        const Real maxDist = w.settings.suspensionRestLength + w.settings.radius;

        Ray ray(attach, -up);
        RayCastResult hits[8];
        const u32 n = mWorld.RayCastAll(ray, maxDist, hits, 8);
        const RayCastResult* hit = nullptr;
        for (u32 k = 0; k < n; ++k) {
            if (hits[k].body == mChassis) continue;
            hit = &hits[k];
            break;
        }

        w.grounded = (hit != nullptr);
        if (!hit) {
            w.compression = Real(0);
            w.angularSpeed *= Real(0.95);
            w.rotationAngle += w.angularSpeed * dt;
            continue;
        }

        w.contactPoint = hit->position;
        w.contactNormal = hit->normal;
        const Real distance = hit->fraction; // raycast fraction is a distance
        Real compression = w.settings.suspensionRestLength + w.settings.radius - distance;
        compression = math::Clamp(compression, Real(0),
                                  w.settings.suspensionRestLength + w.settings.radius);
        w.compression = compression;

        const Vec3 contactVel = chassis->GetPointVelocity(w.contactPoint);
        const Real vUp = contactVel.Dot(up);
        Real suspForce = w.settings.suspensionStiffness * compression -
                         w.settings.suspensionDamping * vUp;
        suspForce = math::Clamp(suspForce, Real(0), w.settings.maxSuspensionForce);
        Vec3 force = up * suspForce;

        // Steering rotates the wheel about the chassis up axis.
        Vec3 wheelForward = forward;
        if (w.settings.steerable && math::Abs(mCurrentSteer) > math::kEpsilon) {
            wheelForward = Quat::AxisAngle(up, mCurrentSteer).Rotate(forward);
        }
        const Vec3 wheelRight = wheelForward.Cross(up).NormalizedOr(forward.Cross(up));

        const Real vLong = contactVel.Dot(wheelForward);
        const Real vLat = contactVel.Dot(wheelRight);

        Real fLong = Real(0);
        if (w.settings.driven) fLong += mThrottle * mSettings.engineForce;
        if (mBrake > Real(0)) {
            const Real brakeMag = mBrake * mSettings.brakeForce;
            if (vLong > Real(0.1)) fLong -= brakeMag;
            else if (vLong < Real(-0.1)) fLong += brakeMag;
            else fLong -= vLong * brakeMag;
        }
        fLong -= vLong * Real(20.0); // rolling resistance

        const Real maxFriction = w.settings.friction * suspForce;
        fLong = math::Clamp(fLong, -maxFriction, maxFriction);
        Real fLat = -vLat * w.settings.lateralStiffness;
        fLat = math::Clamp(fLat, -maxFriction, maxFriction);

        force += wheelForward * fLong + wheelRight * fLat;
        mWorld.AddForceAtPoint(mChassis, force, w.contactPoint);

        w.angularSpeed = vLong / math::Max(w.settings.radius, Real(0.01));
        w.rotationAngle += w.angularSpeed * dt;
    }
}

} // namespace kizuri
