// KizuriPhysics - Body/Body.h
// Rigid body definition and motion state.
#pragma once

#include "Kizuri/Core/Math.h"
#include "Kizuri/Core/Memory.h"
#include "Kizuri/Geometry/Shape.h"

namespace kizuri {

// ---------------------------------------------------------------------------
// BodyID - handle with a generation counter for safe slot reuse.
// ---------------------------------------------------------------------------
struct BodyID {
    u32 index = 0x00FFFFFFu;
    u32 sequence = 0;

    static constexpr u32 kInvalidIndex = 0x00FFFFFFu;

    constexpr BodyID() = default;
    constexpr BodyID(u32 idx, u32 seq) : index(idx), sequence(seq) {}

    constexpr bool IsInvalid() const { return index == kInvalidIndex; }
    constexpr bool operator==(const BodyID& o) const { return index == o.index && sequence == o.sequence; }
    constexpr bool operator!=(const BodyID& o) const { return !(*this == o); }

    /// Pack into a 32-bit value for serialization / hashing.
    u32 Packed() const { return (sequence << 24) | (index & 0x00FFFFFFu); }
    static BodyID Unpack(u32 v) { return BodyID(v & 0x00FFFFFFu, v >> 24); }
};

inline constexpr BodyID kInvalidBodyId{};

// ---------------------------------------------------------------------------
// Motion type
// ---------------------------------------------------------------------------
enum class MotionType : u8 {
    Static = 0,     // never moves, infinite mass
    Kinematic,      // moves by velocity/transform but is not affected by forces
    Dynamic         // fully simulated
};

// ---------------------------------------------------------------------------
// Activation state
// ---------------------------------------------------------------------------
enum class ActivationState : u8 {
    Active = 0,
    Inactive
};

// ---------------------------------------------------------------------------
// Body creation settings
// ---------------------------------------------------------------------------
struct BodySettings {
    Vec3 position = Vec3::Zero();
    Quat rotation = Quat::Identity();
    ShapeRef shape;
    MotionType motionType = MotionType::Dynamic;
    Vec3 linearVelocity = Vec3::Zero();
    Vec3 angularVelocity = Vec3::Zero();
    Real friction = Real(0.5);
    Real restitution = Real(0.0);
    Real linearDamping = Real(0.05);
    Real angularDamping = Real(0.05);
    Real gravityFactor = Real(1.0);
    Real maxLinearVelocity = Real(500.0);
    Real maxAngularVelocity = Real(50.0);
    /// Bitmask of the layers this body belongs to (bit 0 by default).
    u32 objectLayer = 1;
    /// Bitmask of the layers this body collides with.
    u32 collisionMask = 0xFFFFFFFFu;
    bool allowSleeping = true;
    bool isSensor = false;
    void* userData = nullptr;
    /// Optional explicit mass override (0 = derived from shape and density).
    Real massOverride = Real(0);
    Real density = Real(1000.0);
};

// ---------------------------------------------------------------------------
// Motion properties of a dynamic body.
// ---------------------------------------------------------------------------
struct MotionProperties {
    Real inverseMass = 0;
    Mat3 inverseInertiaLocal = Mat3::Zero();
    Mat3 inverseInertiaWorld = Mat3::Zero();

    Vec3 linearVelocity = Vec3::Zero();
    Vec3 angularVelocity = Vec3::Zero();

    /// Split-impulse pseudo velocities. Used only to remove penetration
    /// (position correction); they never carry kinetic energy, which keeps
    /// stacks stable and free of Baumgarte jitter.
    Vec3 pseudoLinearVelocity = Vec3::Zero();
    Vec3 pseudoAngularVelocity = Vec3::Zero();

    /// Accumulated forces and torques for the current step.
    Vec3 force = Vec3::Zero();
    Vec3 torque = Vec3::Zero();

    Real linearDamping = Real(0.05);
    Real angularDamping = Real(0.05);
    Real gravityFactor = Real(1.0);
    Real maxLinearVelocity = Real(500.0);
    Real maxAngularVelocity = Real(50.0);

    /// Time the body has been below the sleep threshold.
    Real sleepTimer = Real(0);

    KZ_FORCEINLINE void UpdateWorldInertia(const Quat& rotation) {
        Mat3 r = rotation.ToMat3();
        inverseInertiaWorld = r * inverseInertiaLocal * r.Transposed();
    }
};

// ---------------------------------------------------------------------------
// Body - a rigid body. Stored in a BodyManager array.
// ---------------------------------------------------------------------------
class Body {
public:
    // --- Identity / configuration -----------------------------------------
    BodyID GetID() const { return mID; }
    MotionType GetMotionType() const { return mMotionType; }
    void SetMotionType(MotionType t) { mMotionType = t; }

    const ShapeRef& GetShape() const { return mShape; }
    void SetShape(const ShapeRef& shape) { mShape = shape; }

    // --- Transform ---------------------------------------------------------
    const Vec3& GetPosition() const { return mTransform.translation; }
    const Quat& GetRotation() const { return mTransform.rotation; }
    const Transform& GetTransform() const { return mTransform; }
    void SetPosition(const Vec3& p) { mTransform.translation = p; }
    void SetRotation(const Quat& q) { mTransform.rotation = q; }
    void SetTransform(const Transform& t) { mTransform = t; }

    // --- Material ----------------------------------------------------------
    Real GetFriction() const { return mFriction; }
    void SetFriction(Real f) { mFriction = f; }
    Real GetRestitution() const { return mRestitution; }
    void SetRestitution(Real r) { mRestitution = r; }

    // --- Motion ------------------------------------------------------------
    MotionProperties& GetMotionProperties() { return mMotion; }
    const MotionProperties& GetMotionProperties() const { return mMotion; }

    Vec3 GetLinearVelocity() const { return mMotion.linearVelocity; }
    Vec3 GetAngularVelocity() const { return mMotion.angularVelocity; }
    void SetLinearVelocity(const Vec3& v) { mMotion.linearVelocity = v; }
    void SetAngularVelocity(const Vec3& w) { mMotion.angularVelocity = w; }

    Real GetMass() const { return mMotion.inverseMass > Real(0) ? Real(1) / mMotion.inverseMass : Real(0); }
    Real GetInverseMass() const { return mMotion.inverseMass; }
    Mat3 GetWorldInverseInertia() const { return mMotion.inverseInertiaWorld; }
    Mat3 GetLocalInverseInertia() const { return mMotion.inverseInertiaLocal; }
    void SetInverseMass(Real invMass) { mMotion.inverseMass = invMass; }
    void SetInverseInertiaLocal(const Mat3& m) { mMotion.inverseInertiaLocal = m; }

    /// World-space velocity of a point rigidly attached to the body.
    KZ_FORCEINLINE Vec3 GetPointVelocity(const Vec3& worldPoint) const {
        return mMotion.linearVelocity + mMotion.angularVelocity.Cross(worldPoint - mTransform.translation);
    }

    // --- Forces ------------------------------------------------------------
    void ApplyForce(const Vec3& force, const Vec3& worldPoint) {
        if (mMotionType != MotionType::Dynamic) return;
        mMotion.force += force;
        mMotion.torque += (worldPoint - mTransform.translation).Cross(force);
    }
    void ApplyForceToCenter(const Vec3& force) {
        if (mMotionType != MotionType::Dynamic) return;
        mMotion.force += force;
    }
    void ApplyTorque(const Vec3& torque) {
        if (mMotionType != MotionType::Dynamic) return;
        mMotion.torque += torque;
    }
    void ApplyImpulse(const Vec3& impulse, const Vec3& worldPoint) {
        if (mMotionType != MotionType::Dynamic) return;
        mMotion.linearVelocity += impulse * mMotion.inverseMass;
        Vec3 r = worldPoint - mTransform.translation;
        mMotion.angularVelocity += mMotion.inverseInertiaWorld * r.Cross(impulse);
    }
    void ApplyImpulseToCenter(const Vec3& impulse) {
        if (mMotionType != MotionType::Dynamic) return;
        mMotion.linearVelocity += impulse * mMotion.inverseMass;
    }
    void ClearForces() { mMotion.force = Vec3::Zero(); mMotion.torque = Vec3::Zero(); }

    // --- State / flags -----------------------------------------------------
    bool IsActive() const { return mActivation == ActivationState::Active; }
    void SetActivation(ActivationState s) { mActivation = s; }
    bool IsDynamic() const { return mMotionType == MotionType::Dynamic; }
    bool IsStatic() const { return mMotionType == MotionType::Static; }
    bool IsKinematic() const { return mMotionType == MotionType::Kinematic; }
    bool IsSensor() const { return mIsSensor; }
    void SetSensor(bool s) { mIsSensor = s; }
    bool IsSleepingAllowed() const { return mAllowSleeping; }
    void SetAllowSleeping(bool v) { mAllowSleeping = v; }

    // --- Broadphase --------------------------------------------------------
    u32 GetBroadPhaseProxy() const { return mBroadPhaseProxy; }
    void SetBroadPhaseProxy(u32 p) { mBroadPhaseProxy = p; }
    u32 GetObjectLayer() const { return mObjectLayer; }
    u32 GetCollisionMask() const { return mCollisionMask; }
    void SetObjectLayer(u32 layer) { mObjectLayer = layer; }
    void SetCollisionMask(u32 mask) { mCollisionMask = mask; }

    // --- Island / solver scratch ------------------------------------------
    u32 GetIslandIndex() const { return mIslandIndex; }
    void SetIslandIndex(u32 i) { mIslandIndex = i; }

    // --- User data ---------------------------------------------------------
    void* GetUserData() const { return mUserData; }
    void SetUserData(void* p) { mUserData = p; }

    // --- Bounds ------------------------------------------------------------
    AABB GetWorldBounds() const { return mShape ? mShape->GetWorldBounds(mTransform) : AABB(); }

private:
    friend class BodyManager;

    BodyID mID;
    Transform mTransform;
    ShapeRef mShape;
    MotionProperties mMotion;
    MotionType mMotionType = MotionType::Dynamic;
    ActivationState mActivation = ActivationState::Active;

    Real mFriction = Real(0.5);
    Real mRestitution = Real(0);
    u32 mObjectLayer = 1;
    u32 mCollisionMask = 0xFFFFFFFFu;
    u32 mBroadPhaseProxy = 0xFFFFFFFFu;
    u32 mIslandIndex = 0xFFFFFFFFu;

    bool mIsSensor = false;
    bool mAllowSleeping = true;
    void* mUserData = nullptr;
};

} // namespace kizuri
