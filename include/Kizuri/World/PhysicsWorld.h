// KizuriPhysics - World/PhysicsWorld.h
// The main physics system: owns bodies, broad phase, narrow phase and solver.
#pragma once

#include "Kizuri/Body/BodyManager.h"
#include "Kizuri/Debug/DebugRenderer.h"
#include "Kizuri/BroadPhase/BroadPhase.h"
#include "Kizuri/Collision/ContactManifold.h"
#include "Kizuri/Constraint/Constraints.h"
#include "Kizuri/Solver/ContactSolver.h"

#include <memory>
#include <vector>

namespace kizuri {

// ---------------------------------------------------------------------------
// World settings
// ---------------------------------------------------------------------------
struct WorldSettings {
    Vec3 gravity = Vec3(Real(0), Real(-9.81), Real(0));
    SolverSettings solver;

    bool allowSleeping = true;
    Real sleepTimeThreshold = Real(0.5);      // seconds below threshold to sleep
    Real sleepLinearThreshold = Real(0.05);   // m/s
    Real sleepAngularThreshold = Real(0.1);   // rad/s

    /// Fixed timestep accumulator. When enabled, Step() subdivides the frame
    /// time into fixed steps for deterministic, stable simulation.
    bool useFixedTimestep = true;
    Real fixedTimestep = Real(1.0 / 60.0);
    u32 maxSubSteps = 4;

    /// Enable multithreaded narrow phase.
    bool useMultithreading = true;

    /// Enable continuous collision detection for LinearCast bodies.
    bool useCCD = true;
    /// The linear cast stops this far before the surface so the next step's
    /// discrete pass can generate a contact.
    Real ccdMaxPenetration = Real(0.01);
};

// ---------------------------------------------------------------------------
// Query result types
// ---------------------------------------------------------------------------
struct RayCastResult {
    BodyID body;
    Real fraction = Real(1);
    Vec3 position = Vec3::Zero();
    Vec3 normal = Vec3::UnitY();
    bool hit = false;
};

struct ShapeCastResult {
    BodyID body;
    Real fraction = Real(1);
    Vec3 normal = Vec3::UnitY();
    bool hit = false;
};

// ---------------------------------------------------------------------------
// Statistics for profiling.
// ---------------------------------------------------------------------------
struct WorldStats {
    u32 numBodies = 0;
    u32 numActiveBodies = 0;
    u32 numBroadPhasePairs = 0;
    u32 numContactConstraints = 0;
    u32 numContactPoints = 0;
    u32 numIslands = 0;
    Real stepTimeMs = 0;
    Real broadPhaseMs = 0;
    Real narrowPhaseMs = 0;
    Real solveMs = 0;
};

// ---------------------------------------------------------------------------
// PhysicsWorld
// ---------------------------------------------------------------------------
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    // --- Settings ----------------------------------------------------------
    const WorldSettings& GetSettings() const { return mSettings; }
    WorldSettings& GetSettings() { return mSettings; }
    WorldSettings& GetMutableSettings() { return mSettings; }
    void SetGravity(const Vec3& g) { mSettings.gravity = g; }
    Vec3 GetGravity() const { return mSettings.gravity; }

    // --- Body management ---------------------------------------------------
    BodyID CreateBody(const BodySettings& settings);
    /// Remove every body, constraint and cached contact from the world.
    void Clear();
    /// Wake or sleep a body (adds/removes it from the active set).
    void SetBodyActive(BodyID id, bool active);
    void DestroyBody(BodyID id);
    void DestroyAllBodies();

    Body* GetBody(BodyID id) { return mBodies.GetBody(id); }
    const Body* GetBody(BodyID id) const { return mBodies.GetBody(id); }
    bool IsValidBody(BodyID id) const { return mBodies.IsValid(id); }

    u32 GetNumBodies() const { return mBodies.NumBodies(); }
    u32 GetNumActiveBodies() const { return u32(mBodies.GetActiveBodyList().Size()); }

    void ActivateBody(BodyID id);
    void DeactivateBody(BodyID id);

    // --- Convenience body operations --------------------------------------
    void SetBodyPosition(BodyID id, const Vec3& position, bool wake = true);
    void SetBodyRotation(BodyID id, const Quat& rotation, bool wake = true);
    void SetBodyTransform(BodyID id, const Transform& transform, bool wake = true);
    void SetLinearVelocity(BodyID id, const Vec3& v, bool wake = true);
    void SetAngularVelocity(BodyID id, const Vec3& w, bool wake = true);
    void AddForce(BodyID id, const Vec3& force, bool wake = true);
    void AddForceAtPoint(BodyID id, const Vec3& force, const Vec3& point, bool wake = true);
    void AddTorque(BodyID id, const Vec3& torque, bool wake = true);
    void AddImpulse(BodyID id, const Vec3& impulse, bool wake = true);
    void AddImpulseAtPoint(BodyID id, const Vec3& impulse, const Vec3& point, bool wake = true);

    // --- Constraints -------------------------------------------------------
    /// Add a constraint. The world takes ownership.
    void AddConstraint(std::unique_ptr<Constraint> constraint);
    void RemoveConstraint(Constraint* constraint);
    void RemoveAllConstraints();
    u32 GetNumConstraints() const { return u32(mConstraints.size()); }

    // --- Simulation --------------------------------------------------------
    /// Advance the simulation by `dt` seconds. Applies sub-stepping according
    /// to the world settings.
    void Step(Real dt);

    /// Perform a single fixed step with no sub-stepping.
    void StepFixed(Real dt);

    // --- Queries -----------------------------------------------------------
    /// Cast a ray; returns the closest hit.
    RayCastResult RayCast(const Ray& ray, Real maxDistance = math::kBigNumber,
                          u32 layerMask = 0xFFFFFFFFu) const;

    /// Cast a ray and collect all hits (up to maxHits).
    u32 RayCastAll(const Ray& ray, Real maxDistance, RayCastResult* outHits, u32 maxHits,
                   u32 layerMask = 0xFFFFFFFFu) const;

    /// Cast a convex shape along a direction; returns the first blocking hit.
    ShapeCastResult CastShape(const Shape& shape, const Transform& start,
                              const Vec3& direction, Real maxDistance,
                              u32 layerMask = 0xFFFFFFFFu) const;

    /// Cast a convex shape and collect every hit, sorted by fraction.
    u32 CastShapeAll(const Shape& shape, const Transform& start, const Vec3& direction,
                     Real maxDistance, ShapeCastResult* outHits, u32 maxHits,
                     u32 layerMask = 0xFFFFFFFFu) const;

    /// Find all bodies whose AABB overlaps a box.
    u32 QueryAABB(const AABB& box, BodyID* outBodies, u32 maxBodies,
                  u32 layerMask = 0xFFFFFFFFu) const;

    /// Test whether a point is inside any body.
    BodyID QueryPoint(const Vec3& point, u32 layerMask = 0xFFFFFFFFu) const;

    // --- Stats / debug -----------------------------------------------------
    const WorldStats& GetStats() const { return mStats; }
    const BroadPhase& GetBroadPhase() const { return mBroadPhase; }
    const BodyManager& GetBodyManager() const { return mBodies; }
    ArrayView<const ContactConstraint> GetContactConstraints() const {
        return { mContactConstraints.Data(), mContactConstraints.Size() };
    }

    /// Deterministic hash of the full simulation state (for lockstep / tests).
    u64 ComputeStateHash() const;

    /// Draw the simulation through a debug renderer. The engine draws shapes,
    /// contact points/normals, joint anchors and broad-phase AABBs.
    void DrawDebug(DebugRenderer& renderer, bool drawShapes = true,
                   bool drawContacts = true, bool drawAABBs = false,
                   bool drawJoints = true) const;

private:
    void UpdateBroadPhase();
    void Collide();
    void BuildContactConstraints();
    void SolveContacts(Real dt);
    void BuildIslands();
    void SolveIsland(u32 islandIndex, Real dt);
    void SolveIslandPositions(u32 islandIndex, Real dt);
    void IntegrateVelocities(Real dt);
    void IntegratePositions(Real dt);
    /// Clamp a body's step displacement so it does not tunnel through geometry.
    void ClampMotionCCD(Body& body, Vec3& displacement) const;
    void UpdateSleeping(Real dt);
    void UpdateBodyInertias();

    // Warm-start manifold cache.
    struct CachedManifold {
        u64 key = 0;
        u32 frame = 0;
        u32 numPoints = 0;
        Vec3 positions[kMaxManifoldPoints];
        Real normalImpulse[kMaxManifoldPoints];
        Real tangentImpulse1[kMaxManifoldPoints];
        Real tangentImpulse2[kMaxManifoldPoints];
    };
    CachedManifold* FindCached(u64 key);
    CachedManifold& GetOrCreateCached(u64 key);

    WorldSettings mSettings;
    BodyManager mBodies;
    BroadPhase mBroadPhase;
    ContactSolver mSolver;

    Vector<Manifold> mManifolds;         // per broadphase pair
    Vector<u64> mManifoldKeys;           // body pair keys
    Vector<ContactConstraint> mContactConstraints;

    // Island partition: contacts and joints that share no dynamic bodies are
    // solved independently (and, when enabled, in parallel).
    struct Island {
        Vector<u32> contacts;   // indices into mContactConstraints
        Vector<u32> joints;     // indices into mConstraints
    };
    Vector<Island> mIslands;
    Vector<CachedManifold> mCache;       // open addressing hash table
    u32 mCacheCapacity = 0;
    u32 mCacheCount = 0;

    std::vector<std::unique_ptr<Constraint>> mConstraints;

    u32 mFrame = 0;
    Real mAccumulator = 0;
    WorldStats mStats;
};

} // namespace kizuri
