// KizuriPhysics - Character/CharacterController.cpp
#include "Kizuri/Character/CharacterController.h"

namespace kizuri {

CharacterController::CharacterController(PhysicsWorld& world, const CharacterSettings& settings)
    : mWorld(world), mSettings(settings) {
    if (mSettings.height < Real(2) * mSettings.radius) {
        mSettings.height = Real(2) * mSettings.radius;
    }
    mHalfHeight = Real(0.5) * mSettings.height - mSettings.radius;
    // A slightly shrunken capsule is used for the casts so the character is
    // never in contact with the ground when a sweep starts (which would make
    // conservative advancement report an immediate hit).
    Real castRadius = mSettings.radius - mSettings.skin;
    if (castRadius < mSettings.radius * Real(0.5)) castRadius = mSettings.radius * Real(0.5);
    mShape = MakeRef<CapsuleShape>(mHalfHeight, castRadius);
}

bool CharacterController::IsWalkable(const Vec3& normal) const {
    Real cosLimit = math::Cos(mSettings.maxSlopeAngle * math::kPi / Real(180));
    return normal.Dot(Vec3::UnitY()) >= cosLimit;
}

bool CharacterController::DetectGround(Real probeDistance, Vec3& outNormal) const {
    // Collect every hit so an adjacent wall (non-walkable) cannot hide the
    // walkable floor beneath the character.
    ShapeCastResult hits[8];
    u32 count = mWorld.CastShapeAll(*mShape, Transform(Quat::Identity(), mPosition),
                                    Vec3(0, -1, 0), probeDistance, hits, 8,
                                    mSettings.collisionMask);
    for (u32 i = 0; i < count; ++i) {
        if (IsWalkable(hits[i].normal)) {
            outNormal = hits[i].normal;
            return true;
        }
    }
    return false;
}

void CharacterController::Move(const Vec3& displacement) {
    Vec3 remaining = displacement;
    for (u32 iter = 0; iter < mSettings.maxIterations; ++iter) {
        Real dist = remaining.Length();
        if (dist < math::kEpsilon) break;
        Vec3 dir = remaining / dist;

        ShapeCastResult hit = mWorld.CastShape(*mShape,
            Transform(Quat::Identity(), mPosition), dir, dist, mSettings.collisionMask);
        if (!hit.hit) {
            mPosition += remaining;
            break;
        }

        Real travel = hit.fraction * dist - mSettings.skin;
        if (travel < Real(0)) travel = Real(0);
        mPosition += dir * travel;
        // Slide the remaining motion along the surface.
        remaining -= hit.normal * remaining.Dot(hit.normal);
    }
}

void CharacterController::MoveHorizontalWithSteps(const Vec3& horizontal, Real dt) {
    KZ_UNUSED(dt);
    if (horizontal.LengthSq() < math::kEpsilon) {
        mClimbing = false;
        return;
    }
    const Real desired = horizontal.Length();
    const Vec3 up(0, mSettings.stepHeight, 0);
    const Vec3 down(0, -(mSettings.stepHeight + mSettings.skin), 0);

    auto horizProgress = [](const Vec3& d) { return Vec3(d.x, Real(0), d.z).Length(); };

    // Continue an in-progress climb: raise, move, settle back down.
    if (mClimbing && mSettings.stepHeight > Real(0)) {
        const Vec3 before = mPosition;
        mPosition += up;
        Move(horizontal);
        Move(down);
        if (mPosition.y > before.y + Real(1.0e-3)) return; // still gaining height
        mPosition = before;
        mClimbing = false;
    }

    const Vec3 before = mPosition;
    Move(horizontal);
    if (horizProgress(mPosition - before) >= desired * Real(0.9)) return; // free move
    if (!mGrounded || mSettings.stepHeight <= Real(0)) return;

    // Blocked: try to climb by raising, moving, then settling back down.
    const Vec3 blocked = mPosition;
    mPosition = before + up;
    Move(horizontal);
    Move(down);
    if (mPosition.y > blocked.y + Real(1.0e-3)) {
        mClimbing = true; // started climbing; keep going next frame
    } else {
        mPosition = blocked;
    }
}

void CharacterController::Update(Real dt) {
    if (dt <= Real(0)) return;

    // Gravity.
    if (!mGrounded) {
        mVerticalVelocity += mSettings.gravity.y * mSettings.gravityFactor * dt;
    }

    const Vec3 horizontal = Vec3(mVelocity.x, Real(0), mVelocity.z) * dt;
    MoveHorizontalWithSteps(horizontal, dt);

    const Vec3 vertical = Vec3(0, mVerticalVelocity * dt, 0);
    Move(vertical);

    // Ground probe.
    Vec3 normal = Vec3::UnitY();
    const Real probe = mSettings.skin + Real(0.05);
    mGrounded = DetectGround(probe, normal) && IsWalkable(normal);
    if (mGrounded) {
        mGroundNormal = normal;
        if (mVerticalVelocity < Real(0)) mVerticalVelocity = Real(0);
    } else {
        mGroundNormal = Vec3::UnitY();
    }
}

} // namespace kizuri
