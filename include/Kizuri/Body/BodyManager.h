// KizuriPhysics - Body/BodyManager.h
// Storage and lifetime management for rigid bodies.
#pragma once

#include "Kizuri/Body/Body.h"

namespace kizuri {

class BodyManager {
public:
    BodyManager();
    ~BodyManager();

    BodyManager(const BodyManager&) = delete;
    BodyManager& operator=(const BodyManager&) = delete;

    /// Create a body from settings. Returns an invalid ID on failure.
    BodyID CreateBody(const BodySettings& settings);
    /// Destroy a body and free its slot.
    void DestroyBody(BodyID id);
    /// Destroy every body.
    void DestroyAllBodies();

    /// Look up a body. Returns nullptr when the ID is stale or invalid.
    Body* GetBody(BodyID id);
    const Body* GetBody(BodyID id) const;

    KZ_FORCEINLINE bool IsValid(BodyID id) const {
        return id.index < mBodies.Size() && mBodies[id.index] != nullptr &&
               mSequences[id.index] == id.sequence;
    }

    u32 NumBodies() const { return mNumBodies; }
    u32 Capacity() const { return u32(mBodies.Size()); }

    /// Activate a body (wakes it and its island).
    void ActivateBody(BodyID id);
    void DeactivateBody(BodyID id);

    /// Iterate every body (including inactive).
    template <typename Fn>
    void ForEachBody(Fn&& fn) {
        for (u32 i = 0; i < mBodies.Size(); ++i) {
            if (mBodies[i]) fn(*mBodies[i]);
        }
    }
    template <typename Fn>
    void ForEachBody(Fn&& fn) const {
        for (u32 i = 0; i < mBodies.Size(); ++i) {
            if (mBodies[i]) fn(*mBodies[i]);
        }
    }

    /// Iterate only active bodies.
    template <typename Fn>
    void ForEachActiveBody(Fn&& fn) {
        for (BodyID id : mActiveBodies) {
            Body* b = GetBody(id);
            if (b) fn(*b);
        }
    }

    ArrayView<const BodyID> GetActiveBodyList() const { return { mActiveBodies.Data(), mActiveBodies.Size() }; }

    /// Number of body slots (including empty/destroyed slots).
    u32 GetBodySlotCount() const { return u32(mBodies.Size()); }
    /// Body stored in a slot, or nullptr for an empty slot.
    Body* GetBodyBySlot(u32 slot) { return slot < u32(mBodies.Size()) ? mBodies[slot] : nullptr; }
    const Body* GetBodyBySlot(u32 slot) const {
        return slot < u32(mBodies.Size()) ? mBodies[slot] : nullptr;
    }

private:
    void RecomputeMassProperties(Body& body, const BodySettings& settings);

    Vector<Body*> mBodies;      // slot -> body or nullptr
    Vector<u32>   mSequences;   // slot -> generation
    Vector<u32>   mFreeList;
    Vector<BodyID> mActiveBodies;
    u32 mNumBodies = 0;
};

} // namespace kizuri
