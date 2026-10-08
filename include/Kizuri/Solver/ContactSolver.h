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

private:
    void WarmStart(ContactConstraint& c);
    void SolveVelocityConstraint(ContactConstraint& c, Real dt);
    void SolvePositionConstraint(ContactConstraint& c, Real dt);
    void SolveBiasConstraint(ContactConstraint& c, Real dt);
    void SolveContactVelocity(ContactConstraint& c, ContactPointConstraint& p);

    SolverSettings mSettings;
};

} // namespace kizuri
