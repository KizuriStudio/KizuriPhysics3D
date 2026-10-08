// KizuriPhysics - World/PhysicsWorld.cpp
#include "Kizuri/World/PhysicsWorld.h"

#include "Kizuri/Core/JobSystem.h"

#include <algorithm>
#include <chrono>

namespace kizuri {

namespace {

KZ_FORCEINLINE u64 MakePairKey(BodyID a, BodyID b) {
    u32 pa = a.Packed();
    u32 pb = b.Packed();
    if (pa > pb) { u32 t = pa; pa = pb; pb = t; }
    return (u64(pa) << 32) | u64(pb);
}

// Swap the two bodies of a manifold (used to keep a stable A/B order so the
// warm-start cache matches reliably across frames).
KZ_FORCEINLINE void SwapManifold(Manifold& m) {
    for (u32 i = 0; i < m.numPoints; ++i) {
        Vec3 t = m.points[i].pointOnA;
        m.points[i].pointOnA = m.points[i].pointOnB;
        m.points[i].pointOnB = t;
        m.points[i].normal = -m.points[i].normal;
    }
}

KZ_FORCEINLINE Quat IntegrateRotation(const Quat& q, const Vec3& w, Real dt) {
    // q' = q + 0.5 * (0, w) * q * dt
    Quat dq(Real(0.5) * dt * w.x, Real(0.5) * dt * w.y, Real(0.5) * dt * w.z, Real(0));
    return (q + dq * q).Normalized();
}

/// Simple union-find for island building.
struct UnionFind {
    Vector<u32> parent;
    Vector<u32> rank;
    void Init(u32 n) {
        parent.Resize(n);
        rank.Resize(n);
        for (u32 i = 0; i < n; ++i) { parent[i] = i; rank[i] = 0; }
    }
    u32 Find(u32 x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    void Union(u32 a, u32 b) {
        a = Find(a); b = Find(b);
        if (a == b) return;
        if (rank[a] < rank[b]) { u32 t = a; a = b; b = t; }
        parent[b] = a;
        if (rank[a] == rank[b]) ++rank[a];
    }
};

} // namespace

// ===========================================================================
// Lifetime
// ===========================================================================
PhysicsWorld::PhysicsWorld() {
    mCacheCapacity = 1024;
    mCache.Resize(mCacheCapacity);
    mManifolds.Reserve(1024);
    mManifoldKeys.Reserve(1024);
    mContactConstraints.Reserve(1024);
}

PhysicsWorld::~PhysicsWorld() = default;

// ===========================================================================
// Body management
// ===========================================================================
BodyID PhysicsWorld::CreateBody(const BodySettings& settings) {
    BodyID id = mBodies.CreateBody(settings);
    if (id.IsInvalid()) return id;

    Body* body = mBodies.GetBody(id);
    AABB bounds = body->GetWorldBounds();
    BroadPhaseProxyId proxy = mBroadPhase.CreateProxy(bounds, id.Packed(),
                                                      body->GetObjectLayer(), body->GetCollisionMask());
    body->SetBroadPhaseProxy(proxy);
    return id;
}

void PhysicsWorld::DestroyBody(BodyID id) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    u32 proxy = body->GetBroadPhaseProxy();
    if (proxy != kInvalidProxy) mBroadPhase.DestroyProxy(proxy);
    mBodies.DestroyBody(id);
}

void PhysicsWorld::DestroyAllBodies() {
    mBodies.DestroyAllBodies();
    mBroadPhase = BroadPhase();
    mConstraints.clear();
    mManifolds.Clear();
    mManifoldKeys.Clear();
    mContactConstraints.Clear();
    for (auto& entry : mCache) entry = CachedManifold();
}

void PhysicsWorld::ActivateBody(BodyID id) { mBodies.ActivateBody(id); }
void PhysicsWorld::DeactivateBody(BodyID id) { mBodies.DeactivateBody(id); }

void PhysicsWorld::SetBodyPosition(BodyID id, const Vec3& position, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->SetPosition(position);
    if (wake) mBodies.ActivateBody(id);
}

void PhysicsWorld::SetBodyRotation(BodyID id, const Quat& rotation, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->SetRotation(rotation.Normalized());
    body->GetMotionProperties().UpdateWorldInertia(body->GetRotation());
    if (wake) mBodies.ActivateBody(id);
}

void PhysicsWorld::SetBodyTransform(BodyID id, const Transform& transform, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->SetTransform(transform);
    body->GetMotionProperties().UpdateWorldInertia(transform.rotation);
    if (wake) mBodies.ActivateBody(id);
}

void PhysicsWorld::SetLinearVelocity(BodyID id, const Vec3& v, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->SetLinearVelocity(v);
    if (wake) mBodies.ActivateBody(id);
}

void PhysicsWorld::SetAngularVelocity(BodyID id, const Vec3& w, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->SetAngularVelocity(w);
    if (wake) mBodies.ActivateBody(id);
}

void PhysicsWorld::AddForce(BodyID id, const Vec3& force, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->ApplyForceToCenter(force);
    if (wake) mBodies.ActivateBody(id);
}

void PhysicsWorld::AddForceAtPoint(BodyID id, const Vec3& force, const Vec3& point, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->ApplyForce(force, point);
    if (wake) mBodies.ActivateBody(id);
}

void PhysicsWorld::AddTorque(BodyID id, const Vec3& torque, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->ApplyTorque(torque);
    if (wake) mBodies.ActivateBody(id);
}

void PhysicsWorld::AddImpulse(BodyID id, const Vec3& impulse, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->ApplyImpulseToCenter(impulse);
    if (wake) mBodies.ActivateBody(id);
}

void PhysicsWorld::AddImpulseAtPoint(BodyID id, const Vec3& impulse, const Vec3& point, bool wake) {
    Body* body = mBodies.GetBody(id);
    if (!body) return;
    body->ApplyImpulse(impulse, point);
    if (wake) mBodies.ActivateBody(id);
}

// ===========================================================================
// Constraints
// ===========================================================================
void PhysicsWorld::AddConstraint(std::unique_ptr<Constraint> constraint) {
    mConstraints.push_back(std::move(constraint));
}

void PhysicsWorld::RemoveConstraint(Constraint* constraint) {
    for (auto it = mConstraints.begin(); it != mConstraints.end(); ++it) {
        if (it->get() == constraint) { mConstraints.erase(it); return; }
    }
}

void PhysicsWorld::RemoveAllConstraints() { mConstraints.clear(); }

// ===========================================================================
// Stepping
// ===========================================================================
void PhysicsWorld::Step(Real dt) {
    if (dt <= Real(0)) return;

    if (!mSettings.useFixedTimestep) {
        StepFixed(dt);
        return;
    }

    mAccumulator += dt;
    const Real fixed = mSettings.fixedTimestep;
    u32 steps = 0;
    while (mAccumulator >= fixed && steps < mSettings.maxSubSteps) {
        StepFixed(fixed);
        mAccumulator -= fixed;
        ++steps;
    }
    // Avoid spiral of death.
    if (steps == mSettings.maxSubSteps) mAccumulator = Real(0);
}

void PhysicsWorld::StepFixed(Real dt) {
    using Clock = std::chrono::high_resolution_clock;
    auto t0 = Clock::now();

    ++mFrame;
    if (mFrame == 0) mFrame = 1;

    // 1. Broad phase.
    auto tb0 = Clock::now();
    UpdateBroadPhase();
    mBroadPhase.UpdatePairs();
    auto tb1 = Clock::now();

    // 2. Narrow phase.
    Collide();
    auto tn1 = Clock::now();

    // 3. Build contact constraints (with warm starting).
    BuildContactConstraints();

    // 4. Integrate velocities (gravity, forces, damping).
    IntegrateVelocities(dt);

    // 5. Solve contacts + joints.
    auto ts0 = Clock::now();
    SolveContacts(dt);
    auto ts1 = Clock::now();

    // 6. Integrate positions (real velocities) then apply the split-impulse
    //    position correction (pseudo velocities) and clear the pseudo state.
    IntegratePositions(dt);
    if (!mSettings.solver.useVelocityBias) {
        const auto& activeList = mBodies.GetActiveBodyList();
        for (u32 i = 0; i < activeList.Size(); ++i) {
            Body* body = mBodies.GetBody(activeList[i]);
            if (!body || body->IsStatic()) continue;
            MotionProperties& mp = body->GetMotionProperties();
            if (mp.pseudoLinearVelocity.LengthSq() > Real(0))
                body->SetPosition(body->GetPosition() + mp.pseudoLinearVelocity * dt);
            if (mp.pseudoAngularVelocity.LengthSq() > Real(0))
                body->SetRotation(IntegrateRotation(body->GetRotation(), mp.pseudoAngularVelocity, dt));
            mp.pseudoLinearVelocity = Vec3::Zero();
            mp.pseudoAngularVelocity = Vec3::Zero();
        }
    }
    UpdateBodyInertias();

    // 7. Joint position correction.
    for (u32 iter = 0; iter < mSettings.solver.positionIterations; ++iter) {
        for (auto& c : mConstraints) {
            if (c->IsEnabled()) c->SolvePosition(dt);
        }
    }

    // 8. Sleeping.
    UpdateSleeping(dt);

    // 9. Clear per-step accumulators.
    mBodies.ForEachBody([](Body& b) { b.ClearForces(); });

    auto tEnd = Clock::now();
    auto ms = [](auto a, auto b) {
        return std::chrono::duration<Real, std::milli>(b - a).count();
    };
    mStats.stepTimeMs = ms(t0, tEnd);
    mStats.broadPhaseMs = ms(tb0, tb1);
    mStats.narrowPhaseMs = ms(tn1, ts0);
    mStats.solveMs = ms(ts0, ts1);
    mStats.numBodies = mBodies.NumBodies();
    mStats.numActiveBodies = u32(mBodies.GetActiveBodyList().Size());
    mStats.numBroadPhasePairs = u32(mBroadPhase.GetPairs().Size());
    mStats.numContactConstraints = u32(mContactConstraints.Size());
    u32 points = 0;
    for (const ContactConstraint& c : mContactConstraints) points += c.numPoints;
    mStats.numContactPoints = points;
}

// ===========================================================================
// Broad phase update
// ===========================================================================
void PhysicsWorld::UpdateBroadPhase() {
    const auto& activeList = mBodies.GetActiveBodyList();
    for (u32 i = 0; i < activeList.Size(); ++i) {
        Body* body = mBodies.GetBody(activeList[i]);
        if (!body || body->IsStatic()) continue;
        u32 proxy = body->GetBroadPhaseProxy();
        if (proxy == kInvalidProxy) continue;
        AABB bounds = body->GetWorldBounds();
        Vec3 displacement = body->GetLinearVelocity() * mSettings.fixedTimestep;
        mBroadPhase.MoveProxy(proxy, bounds, displacement);
    }
}

// ===========================================================================
// Narrow phase
// ===========================================================================
void PhysicsWorld::Collide() {
    ArrayView<const BroadPhasePair> pairs = mBroadPhase.GetPairs();
    const u32 numPairs = u32(pairs.Size());

    mManifolds.Resize(numPairs);
    mManifoldKeys.Resize(numPairs);

    CollideSettings collideSettings;
    collideSettings.maxSeparation = Real(0.02);

    struct Context {
        PhysicsWorld* world;
        ArrayView<const BroadPhasePair> pairs;
    };
    Context ctx{ this, pairs };

    auto collideOne = [](void* data, u32 index) {
        Context* c = static_cast<Context*>(data);
        PhysicsWorld* self = c->world;
        const BroadPhasePair& pair = c->pairs[index];
        BodyID idA = BodyID::Unpack(self->mBroadPhase.GetUserData(pair.proxyA));
        BodyID idB = BodyID::Unpack(self->mBroadPhase.GetUserData(pair.proxyB));
        Body* bodyA = self->mBodies.GetBody(idA);
        Body* bodyB = self->mBodies.GetBody(idB);

        self->mManifolds[index].Clear();
        self->mManifoldKeys[index] = 0;
        if (!bodyA || !bodyB) return;
        if (!bodyA->IsDynamic() && !bodyB->IsDynamic()) return;

        // Normalize the body order (lower packed id first) so the manifold
        // orientation - and therefore the warm-start match - is stable across
        // frames even if the broad phase flips the pair order.
        const bool swapped = idA.Packed() > idB.Packed();
        Body* first = swapped ? bodyB : bodyA;
        Body* second = swapped ? bodyA : bodyB;
        self->mManifoldKeys[index] = MakePairKey(idA, idB);
        CollideSettings cs;
        cs.maxSeparation = Real(0.02);
        NarrowPhase::Collide(*first->GetShape(), first->GetTransform(),
                             *second->GetShape(), second->GetTransform(),
                             cs, self->mManifolds[index]);
        if (swapped) SwapManifold(self->mManifolds[index]);
    };

    if (mSettings.useMultithreading && JobSystem::IsInitialized() && numPairs > 64) {
        JobSystem::Get().ParallelFor(0, numPairs, 32, collideOne, &ctx);
    } else {
        for (u32 i = 0; i < numPairs; ++i) collideOne(&ctx, i);
    }
}

// ===========================================================================
// Contact constraints + warm starting
// ===========================================================================
PhysicsWorld::CachedManifold* PhysicsWorld::FindCached(u64 key) {
    if (mCacheCapacity == 0) return nullptr;
    u32 mask = mCacheCapacity - 1;
    u32 slot = u32(key ^ (key >> 32)) & mask;
    for (u32 probe = 0; probe < mCacheCapacity; ++probe) {
        CachedManifold& e = mCache[slot];
        if (e.key == key && e.frame != 0) return &e;
        if (e.frame == 0) return nullptr;
        slot = (slot + 1) & mask;
    }
    return nullptr;
}

PhysicsWorld::CachedManifold& PhysicsWorld::GetOrCreateCached(u64 key) {
    // Grow when the load factor exceeds 0.7.
    if (mCacheCount * 10 >= mCacheCapacity * 7) {
        u32 newCapacity = mCacheCapacity * 2;
        Vector<CachedManifold> newCache;
        newCache.Resize(newCapacity);
        u32 mask = newCapacity - 1;
        for (auto& e : mCache) {
            if (e.frame == 0) continue;
            u32 slot = u32(e.key ^ (e.key >> 32)) & mask;
            while (newCache[slot].frame != 0) slot = (slot + 1) & mask;
            newCache[slot] = e;
        }
        mCache = std::move(newCache);
        mCacheCapacity = newCapacity;
    }

    u32 mask = mCacheCapacity - 1;
    u32 slot = u32(key ^ (key >> 32)) & mask;
    for (;;) {
        CachedManifold& e = mCache[slot];
        if (e.frame == 0) {
            e = CachedManifold();
            e.key = key;
            e.frame = mFrame;
            ++mCacheCount;
            return e;
        }
        if (e.key == key) {
            e.frame = mFrame;
            return e;
        }
        slot = (slot + 1) & mask;
    }
}

void PhysicsWorld::BuildContactConstraints() {
    mContactConstraints.Clear();

    for (u32 i = 0; i < mManifolds.Size(); ++i) {
        Manifold& m = mManifolds[i];
        if (m.numPoints == 0) continue;

        u64 key = mManifoldKeys[i];
        if (key == 0) continue;

        const BroadPhasePair& pair = mBroadPhase.GetPairs()[i];
        Body* bodyA = mBodies.GetBody(BodyID::Unpack(mBroadPhase.GetUserData(pair.proxyA)));
        Body* bodyB = mBodies.GetBody(BodyID::Unpack(mBroadPhase.GetUserData(pair.proxyB)));
        if (!bodyA || !bodyB) continue;
        // Match the manifold's normalized orientation (see Collide()).
        if (bodyA->GetID().Packed() > bodyB->GetID().Packed()) {
            Body* t = bodyA; bodyA = bodyB; bodyB = t;
        }

        CachedManifold* cached = FindCached(key);
        if (std::getenv("KZ_NOWS") != nullptr) cached = nullptr;

        ContactConstraint c;
        c.bodyA = bodyA;
        c.bodyB = bodyB;
        c.isSensor = bodyA->IsSensor() || bodyB->IsSensor();
        c.friction = math::Sqrt(bodyA->GetFriction() * bodyB->GetFriction());
        c.restitution = math::Max(bodyA->GetRestitution(), bodyB->GetRestitution());
        c.numPoints = m.numPoints;

        for (u32 p = 0; p < m.numPoints; ++p) {
            ContactPointConstraint& cp = c.points[p];
            const ContactPoint& src = m.points[p];
            cp.pointOnA = src.pointOnA;
            cp.pointOnB = src.pointOnB;
            cp.normal = src.normal;
            cp.separation = src.separation;
            cp.featureId = src.featureId;
            cp.combinedFriction = c.friction;
            cp.combinedRestitution = c.restitution;

            // Warm start: match this point to a cached point by proximity.
            cp.normalImpulse = 0;
            cp.tangentImpulse1 = 0;
            cp.tangentImpulse2 = 0;
            if (cached) {
                Real bestDist = Real(0.10) * Real(0.10);
                u32 bestIdx = 0xFFFFFFFFu;
                for (u32 k = 0; k < cached->numPoints; ++k) {
                    Real d = (cached->positions[k] - src.pointOnA).LengthSq();
                    if (d < bestDist) { bestDist = d; bestIdx = k; }
                }
                if (bestIdx != 0xFFFFFFFFu) {
                    cp.normalImpulse = cached->normalImpulse[bestIdx];
                    cp.tangentImpulse1 = cached->tangentImpulse1[bestIdx];
                    cp.tangentImpulse2 = cached->tangentImpulse2[bestIdx];
                }
            }
        }

        mContactConstraints.PushBack(c);
    }
}

// ===========================================================================
// Solver
// ===========================================================================
void PhysicsWorld::SolveContacts(Real dt) {
    ArrayView<ContactConstraint> constraints(mContactConstraints.Data(), mContactConstraints.Size());

    mSolver.SetSettings(mSettings.solver);
    mSolver.Prepare(constraints, dt);
    mSolver.SolveVelocity(constraints, dt);

    // Split-impulse position correction (pseudo velocities, no energy gain).
    if (!mSettings.solver.useVelocityBias) mSolver.SolveBias(constraints, dt);

    // Joints: velocity iterations.
    for (auto& j : mConstraints) if (j->IsEnabled()) j->Prepare(dt);
    for (auto& j : mConstraints) if (j->IsEnabled()) j->WarmStart();
    for (u32 iter = 0; iter < mSettings.solver.velocityIterations; ++iter) {
        for (auto& j : mConstraints) if (j->IsEnabled()) j->SolveVelocity(dt);
    }

    // Store impulses back into the cache for the next frame.
    for (u32 ci = 0; ci < mContactConstraints.Size(); ++ci) {
        ContactConstraint& c = mContactConstraints[ci];
        // Recompute the pair key from the body IDs.
        u64 key = MakePairKey(c.bodyA->GetID(), c.bodyB->GetID());
        CachedManifold& entry = GetOrCreateCached(key);
        entry.numPoints = c.numPoints;
        for (u32 p = 0; p < c.numPoints; ++p) {
            entry.positions[p] = c.points[p].pointOnA;
            entry.normalImpulse[p] = c.points[p].normalImpulse;
            entry.tangentImpulse1[p] = c.points[p].tangentImpulse1;
            entry.tangentImpulse2[p] = c.points[p].tangentImpulse2;
        }
    }
}

// ===========================================================================
// Integration
// ===========================================================================
void PhysicsWorld::IntegrateVelocities(Real dt) {
    const Vec3 gravity = mSettings.gravity;
    const auto& activeList = mBodies.GetActiveBodyList();

    for (u32 i = 0; i < activeList.Size(); ++i) {
        Body* body = mBodies.GetBody(activeList[i]);
        if (!body || !body->IsDynamic()) continue;

        MotionProperties& mp = body->GetMotionProperties();
        Vec3 acceleration = gravity * mp.gravityFactor + mp.force * mp.inverseMass;
        mp.linearVelocity += acceleration * dt;
        mp.angularVelocity += mp.inverseInertiaWorld * mp.torque * dt;

        // Damping.
        mp.linearVelocity *= Real(1) / (Real(1) + mp.linearDamping * dt);
        mp.angularVelocity *= Real(1) / (Real(1) + mp.angularDamping * dt);

        // Velocity clamps.
        mp.linearVelocity = ClampLength(mp.linearVelocity, mp.maxLinearVelocity);
        mp.angularVelocity = ClampLength(mp.angularVelocity, mp.maxAngularVelocity);

        if (!mp.linearVelocity.IsFinite() || !mp.angularVelocity.IsFinite()) {
            mp.linearVelocity = Vec3::Zero();
            mp.angularVelocity = Vec3::Zero();
        }
    }
}

void PhysicsWorld::IntegratePositions(Real dt) {
    const auto& activeList = mBodies.GetActiveBodyList();
    for (u32 i = 0; i < activeList.Size(); ++i) {
        Body* body = mBodies.GetBody(activeList[i]);
        if (!body || body->IsStatic()) continue;

        MotionProperties& mp = body->GetMotionProperties();
        body->SetPosition(body->GetPosition() + mp.linearVelocity * dt);
        if (mp.angularVelocity.LengthSq() > Real(0)) {
            body->SetRotation(IntegrateRotation(body->GetRotation(), mp.angularVelocity, dt));
        }
    }
}

void PhysicsWorld::UpdateBodyInertias() {
    const auto& activeList = mBodies.GetActiveBodyList();
    for (u32 i = 0; i < activeList.Size(); ++i) {
        Body* body = mBodies.GetBody(activeList[i]);
        if (!body || !body->IsDynamic()) continue;
        body->GetMotionProperties().UpdateWorldInertia(body->GetRotation());
    }
}

// ===========================================================================
// Sleeping (island based)
// ===========================================================================
void PhysicsWorld::UpdateSleeping(Real dt) {
    if (!mSettings.allowSleeping) return;

    const auto& activeList = mBodies.GetActiveBodyList();
    const u32 n = u32(activeList.Size());
    if (n == 0) return;

    // Update per-body sleep timers.
    for (u32 i = 0; i < n; ++i) {
        Body* body = mBodies.GetBody(activeList[i]);
        if (!body) continue;
        if (!body->IsDynamic() || !body->IsSleepingAllowed()) {
            body->GetMotionProperties().sleepTimer = 0;
            continue;
        }
        MotionProperties& mp = body->GetMotionProperties();
        Real linSq = mp.linearVelocity.LengthSq();
        Real angSq = mp.angularVelocity.LengthSq();
        if (linSq < mSettings.sleepLinearThreshold * mSettings.sleepLinearThreshold &&
            angSq < mSettings.sleepAngularThreshold * mSettings.sleepAngularThreshold) {
            mp.sleepTimer += dt;
        } else {
            mp.sleepTimer = 0;
        }
    }

    // Union-find islands over contact constraints.
    UnionFind uf;
    uf.Init(n);

    // Index lookup: use island index scratch on the body (reuse mIslandIndex).
    for (u32 i = 0; i < n; ++i) {
        Body* body = mBodies.GetBody(activeList[i]);
        if (body) body->SetIslandIndex(i);
    }

    for (const ContactConstraint& c : mContactConstraints) {
        if (!c.bodyA || !c.bodyB) continue;
        if (!c.bodyA->IsDynamic() || !c.bodyB->IsDynamic()) continue;
        u32 ia = c.bodyA->GetIslandIndex();
        u32 ib = c.bodyB->GetIslandIndex();
        if (ia < n && ib < n) uf.Union(ia, ib);
    }
    for (const auto& j : mConstraints) {
        if (!j->IsEnabled()) continue;
        Body* a = j->GetBodyA();
        Body* b = j->GetBodyB();
        if (!a || !b || !a->IsDynamic() || !b->IsDynamic()) continue;
        u32 ia = a->GetIslandIndex();
        u32 ib = b->GetIslandIndex();
        if (ia < n && ib < n) uf.Union(ia, ib);
    }

    // For each island, sleep it when every dynamic body has been quiet long enough.
    // Aggregate minimum sleep timer per island root.
    Vector<Real> islandMinTimer;
    islandMinTimer.Resize(n);
    for (u32 i = 0; i < n; ++i) islandMinTimer[i] = math::kBigNumber;

    for (u32 i = 0; i < n; ++i) {
        Body* body = mBodies.GetBody(activeList[i]);
        if (!body || !body->IsDynamic() || !body->IsSleepingAllowed()) continue;
        u32 root = uf.Find(i);
        islandMinTimer[root] = math::Min(islandMinTimer[root], body->GetMotionProperties().sleepTimer);
    }

    for (u32 i = 0; i < n; ++i) {
        Body* body = mBodies.GetBody(activeList[i]);
        if (!body || !body->IsDynamic() || !body->IsSleepingAllowed()) continue;
        u32 root = uf.Find(i);
        if (islandMinTimer[root] >= mSettings.sleepTimeThreshold) {
            body->GetMotionProperties().linearVelocity = Vec3::Zero();
            body->GetMotionProperties().angularVelocity = Vec3::Zero();
            mBodies.DeactivateBody(activeList[i]);
        }
    }

    // Count distinct dynamic islands.
    u32 islandCount = 0;
    for (u32 i = 0; i < n; ++i) {
        Body* body = mBodies.GetBody(activeList[i]);
        if (!body || !body->IsDynamic()) continue;
        if (uf.Find(i) == i) ++islandCount;
    }
    mStats.numIslands = islandCount;
}

// ===========================================================================
// Queries
// ===========================================================================
RayCastResult PhysicsWorld::RayCast(const Ray& ray, Real maxDistance, u32 layerMask) const {
    RayCastResult best;
    best.fraction = maxDistance;
    best.hit = false;

    // Let the broad phase prune; the callback tests the shape and can shrink
    // the traversal's max fraction by returning a smaller value.
    Real finalFraction = mBroadPhase.RayCast(ray, maxDistance,
        [&](u32 proxyId, Real maxFraction) -> Real {
            u32 packed = mBroadPhase.GetUserData(proxyId);
            const Body* body = mBodies.GetBody(BodyID::Unpack(packed));
            if (!body) return maxFraction;
            if ((body->GetObjectLayer() & layerMask) == 0) return maxFraction;

            Transform inv = body->GetTransform().Inverse();
            Ray localRay(inv * ray.origin, inv.TransformDirection(ray.direction));
            Vec3 localNormal;
            Real t = body->GetShape()->RayCastLocal(localRay, maxFraction, localNormal);
            if (t < maxFraction) {
                best.fraction = t;
                best.body = body->GetID();
                best.position = ray.PointAt(t);
                best.normal = body->GetRotation().Rotate(localNormal);
                best.hit = true;
                return t;
            }
            return maxFraction;
        });
    KZ_UNUSED(finalFraction);
    return best;
}

u32 PhysicsWorld::RayCastAll(const Ray& ray, Real maxDistance, RayCastResult* outHits,
                             u32 maxHits, u32 layerMask) const {
    u32 count = 0;
    struct Hit { Real t; u32 proxy; };
    Vector<Hit> hits;

    mBroadPhase.RayCast(ray, maxDistance, [&](u32 proxyId, Real maxFraction) -> Real {
        u32 packed = mBroadPhase.GetUserData(proxyId);
        const Body* body = mBodies.GetBody(BodyID::Unpack(packed));
        if (!body) return maxFraction;
        if ((body->GetObjectLayer() & layerMask) == 0) return maxFraction;
        Transform inv = body->GetTransform().Inverse();
        Ray localRay(inv * ray.origin, inv.TransformDirection(ray.direction));
        Vec3 localNormal;
        Real t = body->GetShape()->RayCastLocal(localRay, maxFraction, localNormal);
        if (t < maxFraction) {
            hits.PushBack({ t, proxyId });
        }
        return maxFraction;
    });

    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.t < b.t; });

    for (const Hit& h : hits) {
        if (count >= maxHits) break;
        u32 packed = mBroadPhase.GetUserData(h.proxy);
        const Body* body = mBodies.GetBody(BodyID::Unpack(packed));
        if (!body) continue;
        Transform inv = body->GetTransform().Inverse();
        Ray localRay(inv * ray.origin, inv.TransformDirection(ray.direction));
        Vec3 localNormal;
        Real t = body->GetShape()->RayCastLocal(localRay, maxDistance, localNormal);
        RayCastResult& r = outHits[count++];
        r.body = body->GetID();
        r.fraction = t;
        r.position = ray.PointAt(t);
        r.normal = body->GetRotation().Rotate(localNormal);
        r.hit = true;
    }
    return count;
}

ShapeCastResult PhysicsWorld::CastShape(const Shape& shape, const Transform& start,
                                        const Vec3& direction, Real maxDistance,
                                        u32 layerMask) const {
    ShapeCastResult result;
    Vec3 sweepDir = direction;
    const Real dirLen = sweepDir.Length();
    if (dirLen > math::kEpsilon) sweepDir = sweepDir / dirLen;
    AABB sweepBounds = shape.GetWorldBounds(start);
    AABB endBounds = shape.GetWorldBounds(
        Transform(start.rotation, start.translation + sweepDir * maxDistance));
    sweepBounds.Encapsulate(endBounds);
    sweepBounds.Expand(Real(0.1));

    Real bestFraction = Real(1);
    Vec3 bestNormal = Vec3::UnitY();
    BodyID bestBody = kInvalidBodyId;
    bool found = false;

    mBroadPhase.Query(sweepBounds, [&](u32 proxyId) {
        u32 packed = mBroadPhase.GetUserData(proxyId);
        const Body* body = mBodies.GetBody(BodyID::Unpack(packed));
        if (!body) return;
        if ((body->GetObjectLayer() & layerMask) == 0) return;
        Real fraction;
        Vec3 normal;
        if (NarrowPhase::CastShape(shape, start, direction, maxDistance,
                                   *body->GetShape(), body->GetTransform(), fraction, normal)) {
            if (fraction < bestFraction) {
                bestFraction = fraction;
                bestNormal = normal;
                bestBody = body->GetID();
                found = true;
            }
        }
    });

    if (found) {
        result.hit = true;
        result.body = bestBody;
        result.fraction = bestFraction;
        result.normal = bestNormal;
    }
    return result;
}

u32 PhysicsWorld::QueryAABB(const AABB& box, BodyID* outBodies, u32 maxBodies, u32 layerMask) const {
    u32 count = 0;
    mBroadPhase.Query(box, [&](u32 proxyId) {
        if (count >= maxBodies) return;
        u32 packed = mBroadPhase.GetUserData(proxyId);
        const Body* body = mBodies.GetBody(BodyID::Unpack(packed));
        if (!body) return;
        if ((body->GetObjectLayer() & layerMask) == 0) return;
        outBodies[count++] = body->GetID();
    });
    return count;
}

BodyID PhysicsWorld::QueryPoint(const Vec3& point, u32 layerMask) const {
    AABB box(point, point);
    BodyID result = kInvalidBodyId;
    mBroadPhase.Query(box, [&](u32 proxyId) {
        if (!result.IsInvalid()) return;
        u32 packed = mBroadPhase.GetUserData(proxyId);
        const Body* body = mBodies.GetBody(BodyID::Unpack(packed));
        if (!body) return;
        if ((body->GetObjectLayer() & layerMask) == 0) return;
        Vec3 local = body->GetRotation().InverseRotate(point - body->GetPosition());
        if (body->GetShape()->ContainsPoint(local)) result = body->GetID();
    });
    return result;
}

// ===========================================================================
// State hash (determinism verification)
// ===========================================================================
u64 PhysicsWorld::ComputeStateHash() const {
    // FNV-1a over quantized body state.
    u64 hash = 1469598103934665603ull;
    auto mix = [&hash](u64 v) {
        hash ^= v;
        hash *= 1099511628211ull;
    };
    auto quantize = [](Real v) -> u64 {
        // Quantize to ~1e-5 resolution for stable hashing.
        i64 q = i64(v * Real(100000.0));
        return u64(q);
    };

    mBodies.ForEachBody([&](const Body& b) {
        mix(u64(b.GetID().Packed()));
        mix(quantize(b.GetPosition().x));
        mix(quantize(b.GetPosition().y));
        mix(quantize(b.GetPosition().z));
        mix(quantize(b.GetRotation().x));
        mix(quantize(b.GetRotation().y));
        mix(quantize(b.GetRotation().z));
        mix(quantize(b.GetRotation().w));
        mix(quantize(b.GetLinearVelocity().x));
        mix(quantize(b.GetLinearVelocity().y));
        mix(quantize(b.GetLinearVelocity().z));
        mix(quantize(b.GetAngularVelocity().x));
        mix(quantize(b.GetAngularVelocity().y));
        mix(quantize(b.GetAngularVelocity().z));
    });
    return hash;
}

// ===========================================================================
// Debug rendering
// ===========================================================================
namespace {

void DrawShapeRecursive(DebugRenderer& r, const Shape& shape, const Transform& t,
                        const DebugColor& color, u32 depth) {
    if (depth > 4) return;
    switch (shape.GetType()) {
        case ShapeType::Sphere: {
            const auto& s = static_cast<const SphereShape&>(shape);
            r.DrawSphere(t.translation, s.GetRadius(), color);
            break;
        }
        case ShapeType::Box: {
            const auto& b = static_cast<const BoxShape&>(shape);
            r.DrawBox(t.translation, b.GetHalfExtent(), t.rotation, color);
            break;
        }
        case ShapeType::Capsule:
        case ShapeType::TaperedCapsule: {
            const auto& c = static_cast<const CapsuleShape&>(shape);
            Vec3 axis = t.rotation.Rotate(Vec3::UnitY());
            Vec3 top = t.translation + axis * c.GetHalfHeight();
            Vec3 bot = t.translation - axis * c.GetHalfHeight();
            r.DrawLine(top, bot, color);
            r.DrawSphere(top, c.GetRadius(), color, 8);
            r.DrawSphere(bot, c.GetRadius(), color, 8);
            break;
        }
        case ShapeType::Cylinder: {
            const auto& c = static_cast<const CylinderShape&>(shape);
            Vec3 axis = t.rotation.Rotate(Vec3::UnitY());
            r.DrawLine(t.translation + axis * c.GetHalfHeight(),
                       t.translation - axis * c.GetHalfHeight(), color);
            r.DrawSphere(t.translation + axis * c.GetHalfHeight(), c.GetRadius(), color, 8);
            r.DrawSphere(t.translation - axis * c.GetHalfHeight(), c.GetRadius(), color, 8);
            break;
        }
        case ShapeType::ConvexHull: {
            const auto& h = static_cast<const ConvexHullShape&>(shape);
            ArrayView<const Vec3> pts = h.GetPoints();
            ArrayView<const u32> idx = h.GetFaceIndices();
            ArrayView<const u8> sizes = h.GetFaceSizes();
            u32 off = 0;
            for (u32 f = 0; f < sizes.Size(); ++f) {
                u32 n = sizes[f];
                for (u32 k = 1; k + 1 < n; ++k) {
                    Vec3 a = t * pts[idx[off]];
                    Vec3 b = t * pts[idx[off + k]];
                    Vec3 c = t * pts[idx[off + k + 1]];
                    r.DrawTriangle(a, b, c, color);
                }
                off += n;
            }
            break;
        }
        case ShapeType::Mesh: {
            const auto& m = static_cast<const MeshShape&>(shape);
            for (u32 tri = 0; tri < m.GetNumTriangles(); ++tri) {
                Vec3 a, b, c;
                m.GetTriangle(tri, a, b, c);
                r.DrawTriangle(t * a, t * b, t * c, color);
            }
            break;
        }
        case ShapeType::Compound: {
            const auto& comp = static_cast<const CompoundShape&>(shape);
            for (u32 i = 0; i < comp.GetNumChildren(); ++i) {
                const auto& child = comp.GetChild(i);
                DrawShapeRecursive(r, *child.shape.Get(), t * child.transform, color, depth + 1);
            }
            break;
        }
        case ShapeType::Plane: {
            const auto& p = static_cast<const PlaneShape&>(shape);
            Vec3 n = t.rotation.Rotate(p.GetPlane().normal);
            Vec3 o = t.translation + n * p.GetPlane().distance;
            Vec3 u, v;
            n.GetBasis(u, v);
            const Real s = Real(10.0);
            for (int i = -5; i <= 5; ++i) {
                Real f = Real(i);
                r.DrawLine(o + v * (f * s) - u * (Real(5) * s), o + v * (f * s) + u * (Real(5) * s), color);
                r.DrawLine(o + u * (f * s) - v * (Real(5) * s), o + u * (f * s) + v * (Real(5) * s), color);
            }
            break;
        }
        default:
            break;
    }
}

} // namespace

void PhysicsWorld::DrawDebug(DebugRenderer& renderer, bool drawShapes, bool drawContacts,
                             bool drawAABBs, bool drawJoints) const {
    if (drawShapes || drawAABBs) {
        mBodies.ForEachBody([&](const Body& b) {
            const Shape* shape = b.GetShape().Get();
            if (!shape) return;
            DebugColor color = b.IsStatic() ? DebugColor::Gray()
                             : b.IsKinematic() ? DebugColor::Cyan()
                             : b.IsActive() ? DebugColor::Green() : DebugColor::Blue();
            if (drawShapes) DrawShapeRecursive(renderer, *shape, b.GetTransform(), color, 0);
            if (drawAABBs) {
                AABB box = b.GetWorldBounds();
                renderer.DrawBox(box.Center(), box.Extent(), Quat::Identity(), DebugColor::Magenta());
            }
        });
    }

    if (drawContacts) {
        for (u32 i = 0; i < mManifolds.Size(); ++i) {
            const Manifold& m = mManifolds[i];
            for (u32 p = 0; p < m.numPoints; ++p) {
                const ContactPoint& cp = m.points[p];
                renderer.DrawLine(cp.pointOnA, cp.pointOnB, DebugColor::Yellow());
                renderer.DrawArrow(cp.pointOnA, cp.pointOnA + cp.normal * Real(0.2), DebugColor::Orange());
            }
        }
    }

    if (drawJoints) {
        for (auto& c : mConstraints) {
            const Body* a = c->GetBodyA();
            const Body* b = c->GetBodyB();
            if (a && b) renderer.DrawLine(a->GetPosition(), b->GetPosition(), DebugColor::White());
        }
    }
}

} // namespace kizuri
