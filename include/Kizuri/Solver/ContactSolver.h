// KizuriPhysics - Solver/ContactSolver.h
// Sequential impulse contact solver with warm starting.
#pragma once

#include "Kizuri/Body/Body.h"
#include "Kizuri/Collision/ContactManifold.h"

namespace kizuri {

// ---------------------------------------------------------------------------
// A single contact point constraint (one manifold point).
// ---------------------------------------------------------------------------
struct ContactPointConstraint {
    Vec3 pointOnA = Vec3::Zero();    // contact point on body A (world)
    Vec3 pointOnB = Vec3::Zero();    // contact point on body B (world)
    Vec3 rA = Vec3::Zero();          // contact point relative to body A's origin (world aligned)
    Vec3 rB = Vec3::Zero();          // contact point relative to body B's origin (world aligned)
    Vec3 localAnchorA = Vec3::Zero(); // contact anchor in body A's local frame
    Vec3 localAnchorB = Vec3::Zero(); // contact anchor in body B's local frame

    Vec3 normal = Vec3::UnitY();     // world normal, B -> A
    Vec3 tangent1 = Vec3::UnitX();
    Vec3 tangent2 = Vec3::UnitZ();

    Real normalMass = 0;             // inverse effective mass along the normal
    Real tangentMass1 = 0;
    Real tangentMass2 = 0;

    Real normalImpulse = 0;          // accumulated (warm started)
    Real tangentImpulse1 = 0;
    Real tangentImpulse2 = 0;

    Real separation = 0;             // negative when penetrating
    Real restitutionBias = 0;        // target separating velocity from restitution
    Real positionBias = 0;           // target separating velocity from penetration
    Real biasImpulse = 0;            // accumulated split impulse (pseudo velocity)

    Real combinedFriction = 0;
    Real combinedRestitution = 0;

    Real normalVelocity = 0;         // relative normal velocity before solving
    u32 featureId = 0;
};

// ---------------------------------------------------------------------------
// A contact constraint: one body pair with up to kMaxManifoldPoints points.
// ---------------------------------------------------------------------------
struct ContactConstraint {
    Body* bodyA = nullptr;
    Body* bodyB = nullptr;
    u32 numPoints = 0;
    ContactPointConstraint points[kMaxManifoldPoints];

    // Solver configuration.
    Real friction = Real(0.5);
    Real restitution = 0;
    bool isSensor = false;
    /// Whether each body is dynamic. Static/kinematic bodies must not be
    /// written to, so islands sharing them can be solved in parallel.
    bool dynamicA = true;
    bool dynamicB = true;
};

// ---------------------------------------------------------------------------
// Solver settings.
// ---------------------------------------------------------------------------
struct SolverSettings {
    u32 velocityIterations = 16;
    u32 positionIterations = 6;
    Real baumgarte = Real(0.2);         // position error correction factor
    Real penetrationSlop = Real(0.005); // allowed penetration
    Real restitutionThreshold = Real(1.0); // min approach speed for restitution
    Real maxBiasVelocity = Real(3.0);   // cap on position correction velocity
    Real warmStartFactor = Real(1.0);
    bool useSplitImpulse = true;
    /// Batch contact points four at a time and solve them with SSE.
    bool useSIMDSolver = true;
    /// Add a Baumgarte position bias to the velocity solve. Disabled by
    /// default because the separate position (non-linear Gauss-Seidel) pass
    /// already removes penetration; enabling both double-corrects and jitters.
    bool useVelocityBias = false;
};

// ---------------------------------------------------------------------------
// The contact solver.
// ---------------------------------------------------------------------------
class ContactSolver {
public:
    explicit ContactSolver(const SolverSettings& settings = SolverSettings()) : mSettings(settings) {}

    const SolverSettings& GetSettings() const { return mSettings; }
    void SetSettings(const SolverSettings& s) { mSettings = s; }

    /// Prepare constraints: compute effective masses, relative velocities and
    /// warm-start impulses. `dt` is the simulation timestep.
    void Prepare(ArrayView<ContactConstraint> constraints, Real dt);

    /// Run the velocity iterations (includes warm starting).
    void SolveVelocity(ArrayView<ContactConstraint> constraints, Real dt);

    /// Run the position iterations (non-linear Gauss-Seidel, no velocity change).
    void SolvePosition(ArrayView<ContactConstraint> constraints, Real dt);

    /// Split-impulse bias pass: removes penetration using pseudo velocities that
    /// are integrated into the positions separately from the real motion. This
    /// is the recommended position correction (no energy injection).
    void SolveBias(ArrayView<ContactConstraint> constraints, Real dt);

    // --- Per-constraint API (used by the island solver) --------------------
    /// Prepare a single constraint (effective masses, biases, warm start).
    void PrepareConstraint(ContactConstraint& c, Real dt);
    /// Apply the warm-start impulses of a single constraint.
    void WarmStart(ContactConstraint& c);
    /// One velocity iteration on a single constraint.
    void SolveVelocityConstraint(ContactConstraint& c, Real dt);
    /// One position (non-linear Gauss-Seidel) iteration on a single constraint.
    void SolvePositionConstraint(ContactConstraint& c, Real dt);
    /// One split-impulse bias iteration on a single constraint.
    void SolveBiasConstraint(ContactConstraint& c, Real dt);

    // --- SIMD (4-wide SoA) -------------------------------------------------
    /// Identifies one contact point inside a constraint.
    struct ContactPointRef {
        ContactConstraint* constraint = nullptr;
        u32 point = 0;
    };
    /// Solve up to four contact points in parallel with SSE. Every body that
    /// appears in the batch must be distinct so the impulse scatter is
    /// race-free. Falls back to the scalar path when SSE is unavailable.
    void SolveVelocityBatchSIMD(const ContactPointRef* refs, u32 count, Real dt);

private:
    void SolveContactVelocity(ContactConstraint& c, ContactPointConstraint& p);

    SolverSettings mSettings;
};

} // namespace kizuri
