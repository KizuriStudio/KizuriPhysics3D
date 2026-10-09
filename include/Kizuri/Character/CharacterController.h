// KizuriPhysics - Character/CharacterController.h
// Kinematic capsule character with collide-and-slide movement, slopes, steps,
// ground detection, gravity and jumping.
#pragma once

#include "Kizuri/World/PhysicsWorld.h"

namespace kizuri {

// ---------------------------------------------------------------------------
// Character configuration.
// ---------------------------------------------------------------------------
struct CharacterSettings {
    /// Capsule radius.
    Real radius = Real(0.3);
    /// Total character height (must be >= 2 * radius).
    Real height = Real(1.8);
    /// Maximum walkable slope angle in degrees.
    Real maxSlopeAngle = Real(50.0);
    /// Maximum step height the character can climb.
    Real stepHeight = Real(0.4);
    /// Small gap kept between the character and geometry.
    Real skin = Real(0.02);
    /// Gravity applied when airborne.
    Vec3 gravity = Vec3(Real(0), Real(-9.81), Real(0));
    Real gravityFactor = Real(1.0);
    /// Number of collide-and-slide iterations per move.
    u32 maxIterations = 4;
    /// Layers the character collides with.
    u32 collisionMask = 0xFFFFFFFFu;
};

// ---------------------------------------------------------------------------
// CharacterController
// ---------------------------------------------------------------------------
class CharacterController {
public:
    CharacterController(PhysicsWorld& world, const CharacterSettings& settings = CharacterSettings());

    void SetPosition(const Vec3& p) { mPosition = p; }
    const Vec3& GetPosition() const { return mPosition; }

    /// Horizontal velocity the character tries to move with. The vertical
    /// component is managed internally (gravity / jump).
    void SetVelocity(const Vec3& v) { mVelocity = v; }
    const Vec3& GetVelocity() const { return mVelocity; }

    void SetHorizontalVelocity(const Vec3& v) {
        mVelocity.x = v.x;
        mVelocity.z = v.z;
    }

    void Jump(Real speed) {
        if (mGrounded) {
            mVerticalVelocity = speed;
            mGrounded = false;
        }
    }

    bool IsGrounded() const { return mGrounded; }
    const Vec3& GetGroundNormal() const { return mGroundNormal; }

    /// The capsule shape used by the controller (for debug drawing).
    const CapsuleShape& GetShape() const { return *mShape; }
    Real GetCapsuleHalfHeight() const { return mHalfHeight; }

    /// Advance the controller by `dt`: apply gravity, move (with steps and
    /// sliding) and re-evaluate the ground state.
    void Update(Real dt);

    /// Move by an explicit displacement using collide-and-slide only.
    void Move(const Vec3& displacement);

private:
    void MoveHorizontalWithSteps(const Vec3& horizontal, Real dt);
    bool DetectGround(Real probeDistance, Vec3& outNormal) const;
    bool IsWalkable(const Vec3& normal) const;

    PhysicsWorld& mWorld;
    CharacterSettings mSettings;
    Ref<CapsuleShape> mShape;
    Real mHalfHeight = Real(0.7);

    Vec3 mPosition = Vec3::Zero();
    Vec3 mVelocity = Vec3::Zero();     // horizontal intent
    Real mVerticalVelocity = Real(0);
    Vec3 mGroundNormal = Vec3::UnitY();
    bool mGrounded = false;
    /// True while the character is part-way up a step (keeps step attempts
    /// active across frames even though the intermediate pose is not walkable).
    bool mClimbing = false;
};

} // namespace kizuri
