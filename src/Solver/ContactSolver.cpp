// KizuriPhysics - Solver/ContactSolver.cpp
#include "Kizuri/Solver/ContactSolver.h"

namespace kizuri {

namespace {

/// Integrate a small angular displacement into a quaternion.
KZ_FORCEINLINE Quat IntegrateRotation(const Quat& q, const Vec3& angularDisplacement) {
    Real angleSq = angularDisplacement.LengthSq();
    if (angleSq < Real(1.0e-12)) return q;
    Real angle = math::Sqrt(angleSq);
    Quat dq = Quat::AxisAngle(angularDisplacement / angle, angle);
    return (dq * q).Normalized();
}

/// Effective mass along a direction for two bodies at the given offsets.
KZ_FORCEINLINE Real EffectiveMass(const Body* a, const Body* b, const Vec3& rA, const Vec3& rB,
                                  const Vec3& dir) {
    const MotionProperties& ma = a->GetMotionProperties();
    const MotionProperties& mb = b->GetMotionProperties();
    Vec3 ra = rA.Cross(dir);
    Vec3 rb = rB.Cross(dir);
    Real k = ma.inverseMass + mb.inverseMass;
    k += ra.Dot(ma.inverseInertiaWorld * ra);
    k += rb.Dot(mb.inverseInertiaWorld * rb);
    return k > math::kEpsilon ? Real(1) / k : Real(0);
}

/// Apply an impulse to the body pair. Static/kinematic bodies are skipped so
/// that islands sharing them can be solved concurrently without data races.
KZ_FORCEINLINE void ApplyImpulsePair(MotionProperties& ma, MotionProperties& mb,
                                     const Vec3& rA, const Vec3& rB, const Vec3& impulse,
                                     bool dynA, bool dynB) {
    if (dynA) {
        ma.linearVelocity += impulse * ma.inverseMass;
        ma.angularVelocity += ma.inverseInertiaWorld * rA.Cross(impulse);
    }
    if (dynB) {
        mb.linearVelocity -= impulse * mb.inverseMass;
        mb.angularVelocity -= mb.inverseInertiaWorld * rB.Cross(impulse);
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Prepare
// ---------------------------------------------------------------------------
void ContactSolver::Prepare(ArrayView<ContactConstraint> constraints, Real dt) {
    for (u32 ci = 0; ci < constraints.Size(); ++ci) {
        PrepareConstraint(constraints[ci], dt);
    }
}

void ContactSolver::PrepareConstraint(ContactConstraint& c, Real dt) {
        Body* A = c.bodyA;
        Body* B = c.bodyB;
        if (!A || !B) return;
        c.dynamicA = A->IsDynamic();
        c.dynamicB = B->IsDynamic();

        for (u32 pi = 0; pi < c.numPoints; ++pi) {
            ContactPointConstraint& p = c.points[pi];

            // Anchors relative to each body's origin.
            p.rA = p.pointOnA - A->GetPosition();
            p.rB = p.pointOnB - B->GetPosition();
            p.localAnchorA = A->GetRotation().InverseRotate(p.rA);
            p.localAnchorB = B->GetRotation().InverseRotate(p.rB);

            // Orthonormal contact basis.
            Vec3 n = p.normal.NormalizedOr(Vec3::UnitY());
            p.normal = n;
            Vec3 t1 = n.GetNormalizedPerpendicular();
            p.tangent1 = t1;
            p.tangent2 = n.Cross(t1);

            // Effective masses.
            p.normalMass = EffectiveMass(A, B, p.rA, p.rB, n);
            p.tangentMass1 = EffectiveMass(A, B, p.rA, p.rB, p.tangent1);
            p.tangentMass2 = EffectiveMass(A, B, p.rA, p.rB, p.tangent2);

            // Relative normal velocity before solving.
            Vec3 vA = A->GetPointVelocity(p.pointOnA);
            Vec3 vB = B->GetPointVelocity(p.pointOnB);
            Vec3 vRel = vA - vB;
            p.normalVelocity = vRel.Dot(n);

            // Restitution target.
            p.restitutionBias = 0;
            if (p.normalVelocity < -mSettings.restitutionThreshold) {
                p.restitutionBias = -p.combinedRestitution * p.normalVelocity;
            }

            // Position correction target. Consumed either by the split-impulse
            // bias pass (default) or, if requested, added directly to the
            // velocity solve as classic Baumgarte.
            p.positionBias = 0;
            if (p.separation < -mSettings.penetrationSlop) {
                p.positionBias = mSettings.baumgarte / dt * (-p.separation - mSettings.penetrationSlop);
                p.positionBias = math::Min(p.positionBias, mSettings.maxBiasVelocity);
            }
            p.biasImpulse = 0; // split-impulse accumulator restarts each step

            // Clamp warm-start impulses to the friction cone.
            Real maxFriction = p.combinedFriction * p.normalImpulse;
            p.tangentImpulse1 = math::Clamp(p.tangentImpulse1, -maxFriction, maxFriction);
            p.tangentImpulse2 = math::Clamp(p.tangentImpulse2, -maxFriction, maxFriction);
            p.normalImpulse = math::Max(p.normalImpulse, Real(0));
        }
}

// ---------------------------------------------------------------------------
// Warm start
// ---------------------------------------------------------------------------
void ContactSolver::WarmStart(ContactConstraint& c) {
    Body* A = c.bodyA;
    Body* B = c.bodyB;
    MotionProperties& ma = A->GetMotionProperties();
    MotionProperties& mb = B->GetMotionProperties();

    for (u32 pi = 0; pi < c.numPoints; ++pi) {
        ContactPointConstraint& p = c.points[pi];
        Vec3 impulse = p.normal * p.normalImpulse
                     + p.tangent1 * p.tangentImpulse1
                     + p.tangent2 * p.tangentImpulse2;
        impulse *= mSettings.warmStartFactor;

        ApplyImpulsePair(ma, mb, p.rA, p.rB, impulse, c.dynamicA, c.dynamicB);
    }
}

// ---------------------------------------------------------------------------
// Velocity constraint
// ---------------------------------------------------------------------------
void ContactSolver::SolveContactVelocity(ContactConstraint& c, ContactPointConstraint& p) {
    Body* A = c.bodyA;
    Body* B = c.bodyB;
    MotionProperties& ma = A->GetMotionProperties();
    MotionProperties& mb = B->GetMotionProperties();

    // Recompute the offsets from the current orientation (bodies may rotate
    // during the velocity iterations through accumulated angular impulses).
    Vec3 rA = A->GetRotation().Rotate(p.localAnchorA);
    Vec3 rB = B->GetRotation().Rotate(p.localAnchorB);

    // --- Friction (solved first, using the previous normal impulse) --------
    Vec3 vRel = (ma.linearVelocity + ma.angularVelocity.Cross(rA))
              - (mb.linearVelocity + mb.angularVelocity.Cross(rB));

    for (int t = 0; t < 2; ++t) {
        const Vec3& tangent = (t == 0) ? p.tangent1 : p.tangent2;
        Real mass = (t == 0) ? p.tangentMass1 : p.tangentMass2;
        Real& accum = (t == 0) ? p.tangentImpulse1 : p.tangentImpulse2;

        Real lambda = -mass * vRel.Dot(tangent);
        Real maxFriction = p.combinedFriction * p.normalImpulse;
        Real newImpulse = math::Clamp(accum + lambda, -maxFriction, maxFriction);
        lambda = newImpulse - accum;
        accum = newImpulse;

        Vec3 impulse = tangent * lambda;
        ApplyImpulsePair(ma, mb, rA, rB, impulse, c.dynamicA, c.dynamicB);
    }

    // --- Normal ------------------------------------------------------------
    vRel = (ma.linearVelocity + ma.angularVelocity.Cross(rA))
         - (mb.linearVelocity + mb.angularVelocity.Cross(rB));

    Real bias = p.restitutionBias;
    if (mSettings.useVelocityBias) bias += p.positionBias;
    Real lambda = -p.normalMass * (vRel.Dot(p.normal) - bias);

    Real newImpulse = math::Max(p.normalImpulse + lambda, Real(0));
    lambda = newImpulse - p.normalImpulse;
    p.normalImpulse = newImpulse;

    Vec3 impulse = p.normal * lambda;
    ApplyImpulsePair(ma, mb, rA, rB, impulse, c.dynamicA, c.dynamicB);
}

void ContactSolver::SolveVelocityConstraint(ContactConstraint& c, Real dt) {
    KZ_UNUSED(dt);
    for (u32 pi = 0; pi < c.numPoints; ++pi) {
        SolveContactVelocity(c, c.points[pi]);
    }
}

void ContactSolver::SolveVelocity(ArrayView<ContactConstraint> constraints, Real dt) {
    for (u32 ci = 0; ci < constraints.Size(); ++ci) {
        ContactConstraint& c = constraints[ci];
        if (!c.bodyA || !c.bodyB || c.isSensor) continue;
        WarmStart(c);
    }
    for (u32 iter = 0; iter < mSettings.velocityIterations; ++iter) {
        for (u32 ci = 0; ci < constraints.Size(); ++ci) {
            ContactConstraint& c = constraints[ci];
            if (!c.bodyA || !c.bodyB || c.isSensor) continue;
            SolveVelocityConstraint(c, dt);
        }
    }
}

// ---------------------------------------------------------------------------
// Position constraint (non-linear Gauss-Seidel)
// ---------------------------------------------------------------------------
void ContactSolver::SolvePositionConstraint(ContactConstraint& c, Real dt) {
    KZ_UNUSED(dt);
    Body* A = c.bodyA;
    Body* B = c.bodyB;
    MotionProperties& ma = A->GetMotionProperties();
    MotionProperties& mb = B->GetMotionProperties();

    // Skip fully static pairs.
    if (ma.inverseMass <= Real(0) && mb.inverseMass <= Real(0)) return;

    for (u32 pi = 0; pi < c.numPoints; ++pi) {
        ContactPointConstraint& p = c.points[pi];

        Vec3 rA = A->GetRotation().Rotate(p.localAnchorA);
        Vec3 rB = B->GetRotation().Rotate(p.localAnchorB);
        Vec3 pA = A->GetPosition() + rA;
        Vec3 pB = B->GetPosition() + rB;

        Vec3 n = p.normal;
        Real separation = (pA - pB).Dot(n);
        Real C = math::Min(separation + mSettings.penetrationSlop, Real(0));
        if (C >= Real(0)) continue;

        Real k = EffectiveMass(A, B, rA, rB, n);
        Real lambda = -k * C * mSettings.baumgarte;
        if (lambda <= Real(0)) continue;

        Vec3 impulse = n * lambda;
        A->SetPosition(A->GetPosition() + impulse * ma.inverseMass);
        B->SetPosition(B->GetPosition() - impulse * mb.inverseMass);

        A->SetRotation(IntegrateRotation(A->GetRotation(), ma.inverseInertiaWorld * rA.Cross(impulse)));
        B->SetRotation(IntegrateRotation(B->GetRotation(), -(mb.inverseInertiaWorld * rB.Cross(impulse))));
    }
}

void ContactSolver::SolvePosition(ArrayView<ContactConstraint> constraints, Real dt) {
    for (u32 iter = 0; iter < mSettings.positionIterations; ++iter) {
        for (u32 ci = 0; ci < constraints.Size(); ++ci) {
            ContactConstraint& c = constraints[ci];
            if (!c.bodyA || !c.bodyB || c.isSensor) continue;
            SolvePositionConstraint(c, dt);
        }
    }
}

// ---------------------------------------------------------------------------
// Split-impulse bias pass (position correction via pseudo velocities)
// ---------------------------------------------------------------------------
void ContactSolver::SolveBiasConstraint(ContactConstraint& c, Real dt) {
    KZ_UNUSED(dt);
    Body* A = c.bodyA;
    Body* B = c.bodyB;
    MotionProperties& ma = A->GetMotionProperties();
    MotionProperties& mb = B->GetMotionProperties();
    if (ma.inverseMass <= Real(0) && mb.inverseMass <= Real(0)) return;

    for (u32 pi = 0; pi < c.numPoints; ++pi) {
        ContactPointConstraint& p = c.points[pi];
        if (p.positionBias <= Real(0)) continue;

        Vec3 rA = A->GetRotation().Rotate(p.localAnchorA);
        Vec3 rB = B->GetRotation().Rotate(p.localAnchorB);

        Vec3 vRel = (ma.pseudoLinearVelocity + ma.pseudoAngularVelocity.Cross(rA))
                  - (mb.pseudoLinearVelocity + mb.pseudoAngularVelocity.Cross(rB));

        Real lambda = -p.normalMass * (vRel.Dot(p.normal) - p.positionBias);
        Real newImpulse = math::Max(p.biasImpulse + lambda, Real(0));
        lambda = newImpulse - p.biasImpulse;
        p.biasImpulse = newImpulse;

        Vec3 impulse = p.normal * lambda;
        if (c.dynamicA) {
            ma.pseudoLinearVelocity += impulse * ma.inverseMass;
            ma.pseudoAngularVelocity += ma.inverseInertiaWorld * rA.Cross(impulse);
        }
        if (c.dynamicB) {
            mb.pseudoLinearVelocity -= impulse * mb.inverseMass;
            mb.pseudoAngularVelocity -= mb.inverseInertiaWorld * rB.Cross(impulse);
        }
    }
}

void ContactSolver::SolveBias(ArrayView<ContactConstraint> constraints, Real dt) {
    for (u32 iter = 0; iter < mSettings.positionIterations; ++iter) {
        for (u32 ci = 0; ci < constraints.Size(); ++ci) {
            ContactConstraint& c = constraints[ci];
            if (!c.bodyA || !c.bodyB || c.isSensor) continue;
            SolveBiasConstraint(c, dt);
        }
    }
}

} // namespace kizuri
