// KizuriPhysics - Body/BodyManager.cpp
#include "Kizuri/Body/BodyManager.h"

namespace kizuri {

BodyManager::BodyManager() {
    mBodies.Reserve(256);
    mSequences.Reserve(256);
}

BodyManager::~BodyManager() {
    DestroyAllBodies();
}

BodyID BodyManager::CreateBody(const BodySettings& settings) {
    if (!settings.shape) return kInvalidBodyId;

    u32 index;
    if (!mFreeList.Empty()) {
        index = mFreeList.Back();
        mFreeList.PopBack();
    } else {
        index = u32(mBodies.Size());
        mBodies.PushBack(nullptr);
        mSequences.PushBack(0);
    }
    if (index >= config::kMaxBodies) return kInvalidBodyId;

    Body* body = new Body();
    body->mID = BodyID(index, mSequences[index]);
    body->mTransform = Transform(settings.rotation, settings.position);
    body->mShape = settings.shape;
    body->mMotionType = settings.motionType;
    body->mFriction = settings.friction;
    body->mRestitution = settings.restitution;
    body->mObjectLayer = settings.objectLayer;
    body->mCollisionMask = settings.collisionMask;
    body->mIsSensor = settings.isSensor;
    body->mAllowSleeping = settings.allowSleeping;
    body->mUserData = settings.userData;
    body->mActivation = ActivationState::Active;

    MotionProperties& mp = body->mMotion;
    mp.linearVelocity = settings.linearVelocity;
    mp.angularVelocity = settings.angularVelocity;
    mp.linearDamping = settings.linearDamping;
    mp.angularDamping = settings.angularDamping;
    mp.gravityFactor = settings.gravityFactor;
    mp.maxLinearVelocity = settings.maxLinearVelocity;
    mp.maxAngularVelocity = settings.maxAngularVelocity;
    mp.sleepTimer = 0;

    RecomputeMassProperties(*body, settings);

    mBodies[index] = body;
    ++mNumBodies;
    mActiveBodies.PushBack(body->mID);
    return body->mID;
}

void BodyManager::RecomputeMassProperties(Body& body, const BodySettings& settings) {
    MotionProperties& mp = body.mMotion;
    if (settings.motionType != MotionType::Dynamic) {
        mp.inverseMass = 0;
        mp.inverseInertiaLocal = Mat3::Zero();
        mp.inverseInertiaWorld = Mat3::Zero();
        return;
    }

    MassProperties props = settings.shape->GetMassProperties(settings.density);
    Real mass = settings.massOverride > Real(0) ? settings.massOverride : props.mass;
    if (mass <= math::kEpsilon) {
        mp.inverseMass = 0;
        mp.inverseInertiaLocal = Mat3::Zero();
        mp.inverseInertiaWorld = Mat3::Zero();
        return;
    }

    mp.inverseMass = Real(1) / mass;

    // Inertia: if the shape has no volume, fall back to a point mass.
    Mat3 inertia = props.inertia;
    Real det = inertia.Determinant();
    if (math::Abs(det) < Real(1.0e-12)) {
        // Approximate with a sphere of equivalent radius.
        Real r = settings.shape->GetBoundingRadius();
        Real i = Real(0.4) * mass * r * r;
        inertia = Mat3::Identity() * i;
    }

    // Shift inertia so it is expressed about the body origin, not the CoM.
    Vec3 com = props.centerOfMass;
    inertia = inertia + (Mat3::Identity() * com.LengthSq() - Mat3::OuterProduct(com, com)) * mass;

    mp.inverseInertiaLocal = inertia.Inversed();
    mp.UpdateWorldInertia(body.mTransform.rotation);
}

void BodyManager::DestroyBody(BodyID id) {
    if (!IsValid(id)) return;
    Body* body = mBodies[id.index];
    delete body;
    mBodies[id.index] = nullptr;
    // Bump the sequence so stale handles are rejected.
    mSequences[id.index] = (mSequences[id.index] + 1) & 0xFF;
    mFreeList.PushBack(id.index);
    --mNumBodies;
    // Remove from the active list (swap-remove).
    for (u32 i = 0; i < mActiveBodies.Size(); ++i) {
        if (mActiveBodies[i] == id) { mActiveBodies.RemoveSwap(i); break; }
    }
}

void BodyManager::DestroyAllBodies() {
    for (u32 i = 0; i < mBodies.Size(); ++i) {
        delete mBodies[i];
        mBodies[i] = nullptr;
    }
    mBodies.Clear();
    mSequences.Clear();
    mFreeList.Clear();
    mActiveBodies.Clear();
    mNumBodies = 0;
}

Body* BodyManager::GetBody(BodyID id) {
    if (!IsValid(id)) return nullptr;
    return mBodies[id.index];
}

const Body* BodyManager::GetBody(BodyID id) const {
    if (!IsValid(id)) return nullptr;
    return mBodies[id.index];
}

void BodyManager::ActivateBody(BodyID id) {
    Body* body = GetBody(id);
    if (!body) return;
    if (body->mActivation == ActivationState::Inactive) {
        body->mActivation = ActivationState::Active;
        body->mMotion.sleepTimer = 0;
        mActiveBodies.PushBack(id);
    }
}

void BodyManager::DeactivateBody(BodyID id) {
    Body* body = GetBody(id);
    if (!body) return;
    if (body->mActivation == ActivationState::Active) {
        body->mActivation = ActivationState::Inactive;
        body->mMotion.linearVelocity = Vec3::Zero();
        body->mMotion.angularVelocity = Vec3::Zero();
        for (u32 i = 0; i < mActiveBodies.Size(); ++i) {
            if (mActiveBodies[i] == id) { mActiveBodies.RemoveSwap(i); break; }
        }
    }
}

} // namespace kizuri
