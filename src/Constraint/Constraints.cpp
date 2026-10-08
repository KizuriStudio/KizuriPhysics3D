// KizuriPhysics - Constraint/Constraints.cpp
#include "Kizuri/Constraint/Constraints.h"

namespace kizuri {

namespace {

KZ_FORCEINLINE void ApplyImpulse(Body* a, Body* b, const Vec3& ra, const Vec3& rb, const Vec3& impulse) {
    MotionProperties& ma = a->GetMotionProperties();
    MotionProperties& mb = b->GetMotionProperties();
    ma.linearVelocity += impulse * ma.inverseMass;
    ma.angularVelocity += ma.inverseInertiaWorld * ra.Cross(impulse);
    mb.linearVelocity -= impulse * mb.inverseMass;
    mb.angularVelocity -= mb.inverseInertiaWorld * rb.Cross(impulse);
}

KZ_FORCEINLINE void ApplyAngularImpulse(Body* a, Body* b, const Vec3& angular) {
    a->GetMotionProperties().angularVelocity += a->GetMotionProperties().inverseInertiaWorld * angular;
    b->GetMotionProperties().angularVelocity -= b->GetMotionProperties().inverseInertiaWorld * angular;
}

/// Effective mass matrix for a 3-DOF point constraint.
Mat3 PointEffectiveMass(const Body* a, const Body* b, const Vec3& ra, const Vec3& rb) {
    const MotionProperties& ma = a->GetMotionProperties();
    const MotionProperties& mb = b->GetMotionProperties();
    Mat3 K = Mat3::Identity() * (ma.inverseMass + mb.inverseMass);
    Mat3 skewA = Mat3::Skew(ra);
    Mat3 skewB = Mat3::Skew(rb);
    K -= skewA * (ma.inverseInertiaWorld * skewA);
    K -= skewB * (mb.inverseInertiaWorld * skewB);
    return K;
}

/// Effective mass for a single angular axis.
Real AngularEffectiveMass(const Body* a, const Body* b, const Vec3& axis) {
    const MotionProperties& ma = a->GetMotionProperties();
    const MotionProperties& mb = b->GetMotionProperties();
    Real k = axis.Dot(ma.inverseInertiaWorld * axis) + axis.Dot(mb.inverseInertiaWorld * axis);
    return k > math::kEpsilon ? Real(1) / k : Real(0);
}

/// Effective mass for a single linear axis at the given offsets.
Real LinearEffectiveMass(const Body* a, const Body* b, const Vec3& ra, const Vec3& rb, const Vec3& axis) {
    const MotionProperties& ma = a->GetMotionProperties();
    const MotionProperties& mb = b->GetMotionProperties();
    Vec3 ca = ra.Cross(axis);
    Vec3 cb = rb.Cross(axis);
    Real k = ma.inverseMass + mb.inverseMass;
    k += ca.Dot(ma.inverseInertiaWorld * ca);
    k += cb.Dot(mb.inverseInertiaWorld * cb);
    return k > math::kEpsilon ? Real(1) / k : Real(0);
}

KZ_FORCEINLINE Quat IntegrateRotation(const Quat& q, const Vec3& dw) {
    Real a2 = dw.LengthSq();
    if (a2 < Real(1.0e-12)) return q;
    Real a = math::Sqrt(a2);
    return (Quat::AxisAngle(dw / a, a) * q).Normalized();
}

/// Rotate a body's position/rotation by a positional impulse.
void ApplyPositionalImpulse(Body* a, Body* b, const Vec3& ra, const Vec3& rb,
                            const Vec3& linearImpulse, const Vec3& angularImpulse) {
    KZ_UNUSED(ra); KZ_UNUSED(rb);
    MotionProperties& ma = a->GetMotionProperties();
    MotionProperties& mb = b->GetMotionProperties();
    a->SetPosition(a->GetPosition() + linearImpulse * ma.inverseMass);
    a->SetRotation(IntegrateRotation(a->GetRotation(), ma.inverseInertiaWorld * angularImpulse));
    b->SetPosition(b->GetPosition() - linearImpulse * mb.inverseMass);
    b->SetRotation(IntegrateRotation(b->GetRotation(), -(mb.inverseInertiaWorld * angularImpulse)));
}

} // namespace

// ===========================================================================
// PointConstraint
// ===========================================================================
PointConstraint::PointConstraint(Body* a, Body* b, const Vec3& worldAnchor)
    : Constraint(a, b) {
    mLocalAnchorA = a->GetRotation().InverseRotate(worldAnchor - a->GetPosition());
    mLocalAnchorB = b->GetRotation().InverseRotate(worldAnchor - b->GetPosition());
}

PointConstraint::PointConstraint(Body* a, Body* b, const Vec3& localAnchorA, const Vec3& localAnchorB)
    : Constraint(a, b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB) {}

void PointConstraint::Prepare(Real dt) {
    KZ_UNUSED(dt);
    mR[0] = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    mR[1] = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    mEffectiveMass = PointEffectiveMass(mBodyA, mBodyB, mR[0], mR[1]).Inversed();
}

void PointConstraint::WarmStart() {
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], mImpulse);
}

void PointConstraint::SolveVelocity(Real /*dt*/) {
    MotionProperties& ma = mBodyA->GetMotionProperties();
    MotionProperties& mb = mBodyB->GetMotionProperties();
    Vec3 vRel = (ma.linearVelocity + ma.angularVelocity.Cross(mR[0]))
              - (mb.linearVelocity + mb.angularVelocity.Cross(mR[1]));
    Vec3 impulse = mEffectiveMass * (-vRel);
    mImpulse += impulse;
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], impulse);
}

void PointConstraint::SolvePosition(Real /*dt*/) {
    Vec3 ra = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    Vec3 rb = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    Vec3 pA = mBodyA->GetPosition() + ra;
    Vec3 pB = mBodyB->GetPosition() + rb;
    Vec3 error = pA - pB;
    const Real slop = Real(0.001);
    if (error.LengthSq() < slop * slop) return;
    Mat3 K = PointEffectiveMass(mBodyA, mBodyB, ra, rb).Inversed();
    Vec3 impulse = K * (-error);
    ApplyPositionalImpulse(mBodyA, mBodyB, ra, rb, impulse, ra.Cross(impulse));
}

// ===========================================================================
// DistanceConstraint
// ===========================================================================
DistanceConstraint::DistanceConstraint(Body* a, Body* b, const Vec3& worldAnchorA,
                                       const Vec3& worldAnchorB, Real distance)
    : Constraint(a, b), mTargetDistance(distance) {
    mLocalAnchorA = a->GetRotation().InverseRotate(worldAnchorA - a->GetPosition());
    mLocalAnchorB = b->GetRotation().InverseRotate(worldAnchorB - b->GetPosition());
}

void DistanceConstraint::Prepare(Real /*dt*/) {
    mR[0] = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    mR[1] = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    Vec3 pA = mBodyA->GetPosition() + mR[0];
    Vec3 pB = mBodyB->GetPosition() + mR[1];
    Vec3 d = pB - pA;
    Real dist = d.Length();
    mAxis = dist > math::kEpsilon ? d / dist : Vec3::UnitY();
    mEffectiveMass = LinearEffectiveMass(mBodyA, mBodyB, mR[0], mR[1], mAxis);
}

void DistanceConstraint::WarmStart() {
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], mAxis * mImpulse);
}

void DistanceConstraint::SolveVelocity(Real /*dt*/) {
    MotionProperties& ma = mBodyA->GetMotionProperties();
    MotionProperties& mb = mBodyB->GetMotionProperties();
    Vec3 vRel = (ma.linearVelocity + ma.angularVelocity.Cross(mR[0]))
              - (mb.linearVelocity + mb.angularVelocity.Cross(mR[1]));
    Real rel = vRel.Dot(mAxis);
    Real lambda = -mEffectiveMass * rel;
    mImpulse += lambda;
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], mAxis * lambda);
}

void DistanceConstraint::SolvePosition(Real /*dt*/) {
    Vec3 ra = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    Vec3 rb = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    Vec3 pA = mBodyA->GetPosition() + ra;
    Vec3 pB = mBodyB->GetPosition() + rb;
    Vec3 d = pB - pA;
    Real dist = d.Length();
    if (dist < math::kEpsilon) return;
    Vec3 axis = d / dist;
    Real error = dist - mTargetDistance;
    Real k = LinearEffectiveMass(mBodyA, mBodyB, ra, rb, axis);
    Real lambda = -k * error * Real(0.2);
    ApplyPositionalImpulse(mBodyA, mBodyB, ra, rb, axis * lambda, Vec3::Zero());
}

// ===========================================================================
// FixedConstraint
// ===========================================================================
FixedConstraint::FixedConstraint(Body* a, Body* b, const Vec3& worldAnchor)
    : Constraint(a, b) {
    mLocalAnchorA = a->GetRotation().InverseRotate(worldAnchor - a->GetPosition());
    mLocalAnchorB = b->GetRotation().InverseRotate(worldAnchor - b->GetPosition());
    mRelativeRotation = a->GetRotation().Conjugated() * b->GetRotation();
}

void FixedConstraint::Prepare(Real /*dt*/) {
    mR[0] = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    mR[1] = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    mLinearMass = PointEffectiveMass(mBodyA, mBodyB, mR[0], mR[1]).Inversed();

    const MotionProperties& ma = mBodyA->GetMotionProperties();
    const MotionProperties& mb = mBodyB->GetMotionProperties();
    Mat3 K = ma.inverseInertiaWorld + mb.inverseInertiaWorld;
    mAngularMass = K.Inversed();
}

void FixedConstraint::WarmStart() {
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], mLinearImpulse);
    ApplyAngularImpulse(mBodyA, mBodyB, mAngularImpulse);
}

void FixedConstraint::SolveVelocity(Real /*dt*/) {
    MotionProperties& ma = mBodyA->GetMotionProperties();
    MotionProperties& mb = mBodyB->GetMotionProperties();

    // Linear.
    Vec3 vRel = (ma.linearVelocity + ma.angularVelocity.Cross(mR[0]))
              - (mb.linearVelocity + mb.angularVelocity.Cross(mR[1]));
    Vec3 linImpulse = mLinearMass * (-vRel);
    mLinearImpulse += linImpulse;
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], linImpulse);

    // Angular.
    Vec3 wRel = ma.angularVelocity - mb.angularVelocity;
    Vec3 angImpulse = mAngularMass * (-wRel);
    mAngularImpulse += angImpulse;
    ApplyAngularImpulse(mBodyA, mBodyB, angImpulse);
}

void FixedConstraint::SolvePosition(Real /*dt*/) {
    // Linear error.
    Vec3 ra = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    Vec3 rb = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    Vec3 error = (mBodyA->GetPosition() + ra) - (mBodyB->GetPosition() + rb);
    if (error.LengthSq() > Real(1.0e-8)) {
        Mat3 K = PointEffectiveMass(mBodyA, mBodyB, ra, rb).Inversed();
        Vec3 impulse = K * (-error) * Real(0.2);
        ApplyPositionalImpulse(mBodyA, mBodyB, ra, rb, impulse, ra.Cross(impulse));
    }

    // Angular error.
    Quat current = mBodyA->GetRotation().Conjugated() * mBodyB->GetRotation();
    Quat diff = current * mRelativeRotation.Conjugated();
    if (diff.w < Real(0)) diff = -diff;
    Vec3 axis = diff.GetAxis();
    Real angle = Real(2) * math::Acos(math::Clamp(diff.w, Real(-1), Real(1)));
    if (math::Abs(angle) > Real(1.0e-4)) {
        Vec3 angError = axis * angle;
        const MotionProperties& ma = mBodyA->GetMotionProperties();
        const MotionProperties& mb = mBodyB->GetMotionProperties();
        Mat3 K = (ma.inverseInertiaWorld + mb.inverseInertiaWorld).Inversed();
        Vec3 impulse = K * (-angError) * Real(0.2);
        ApplyPositionalImpulse(mBodyA, mBodyB, Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), impulse);
    }
}

// ===========================================================================
// HingeConstraint
// ===========================================================================
HingeConstraint::HingeConstraint(Body* a, Body* b, const Vec3& worldAnchor,
                                 const Vec3& worldAxisA, const Vec3& worldAxisB)
    : Constraint(a, b) {
    mLocalAnchorA = a->GetRotation().InverseRotate(worldAnchor - a->GetPosition());
    mLocalAnchorB = b->GetRotation().InverseRotate(worldAnchor - b->GetPosition());
    mLocalAxisA = a->GetRotation().InverseRotate(worldAxisA.Normalized());
    mLocalAxisB = b->GetRotation().InverseRotate(worldAxisB.Normalized());

    // Build two perpendicular axes in each body's local frame.
    Vec3 p1, p2;
    mLocalAxisA.GetBasis(p1, p2);
    mLocalPerpA[0] = p1; mLocalPerpA[1] = p2;
    mLocalAxisB.GetBasis(p1, p2);
    mLocalPerpB[0] = p1; mLocalPerpB[1] = p2;
}

void HingeConstraint::Prepare(Real /*dt*/) {
    mR[0] = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    mR[1] = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    mPointMass = PointEffectiveMass(mBodyA, mBodyB, mR[0], mR[1]).Inversed();

    Vec3 perpA[2] = { mBodyA->GetRotation().Rotate(mLocalPerpA[0]),
                      mBodyA->GetRotation().Rotate(mLocalPerpA[1]) };
    Vec3 perpB[2] = { mBodyB->GetRotation().Rotate(mLocalPerpB[0]),
                      mBodyB->GetRotation().Rotate(mLocalPerpB[1]) };

    // Average the perpendicular directions from both bodies to define the
    // locked angular axes.
    for (int i = 0; i < 2; ++i) {
        Vec3 axis = (perpA[i] + perpB[i]).NormalizedOr(perpA[i]);
        mAngularMass[i] = AngularEffectiveMass(mBodyA, mBodyB, axis);
        mLocalPerpA[i] = mBodyA->GetRotation().InverseRotate(axis);
    }

    // Limit/motor along the hinge axis.
    Vec3 hingeAxis = mBodyA->GetRotation().Rotate(mLocalAxisA);
    mLimitMass = AngularEffectiveMass(mBodyA, mBodyB, hingeAxis);

    mAngle = GetAngle();
}

void HingeConstraint::WarmStart() {
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], mPointImpulse);
    for (int i = 0; i < 2; ++i) {
        ApplyAngularImpulse(mBodyA, mBodyB,
            mBodyA->GetRotation().Rotate(mLocalPerpA[i]) * mAngularImpulse[i]);
    }
    Vec3 hingeAxis = mBodyA->GetRotation().Rotate(mLocalAxisA);
    ApplyAngularImpulse(mBodyA, mBodyB, hingeAxis * (mLimitImpulse + mMotorImpulse));
}

void HingeConstraint::SolveVelocity(Real /*dt*/) {
    MotionProperties& ma = mBodyA->GetMotionProperties();
    MotionProperties& mb = mBodyB->GetMotionProperties();

    // Point.
    Vec3 vRel = (ma.linearVelocity + ma.angularVelocity.Cross(mR[0]))
              - (mb.linearVelocity + mb.angularVelocity.Cross(mR[1]));
    Vec3 linImpulse = mPointMass * (-vRel);
    mPointImpulse += linImpulse;
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], linImpulse);

    // Locked angular axes.
    Vec3 wRel = ma.angularVelocity - mb.angularVelocity;
    for (int i = 0; i < 2; ++i) {
        Vec3 axis = mBodyA->GetRotation().Rotate(mLocalPerpA[i]);
        Real lambda = -mAngularMass[i] * wRel.Dot(axis);
        mAngularImpulse[i] += lambda;
        ApplyAngularImpulse(mBodyA, mBodyB, axis * lambda);
    }

    // Motor.
    if (mHasMotor) {
        Vec3 hingeAxis = mBodyA->GetRotation().Rotate(mLocalAxisA);
        Real relVel = (ma.angularVelocity - mb.angularVelocity).Dot(hingeAxis);
        Real lambda = mLimitMass * (mMotorTargetVelocity - relVel);
        Real maxImpulse = mMotorMaxImpulse;
        Real newImpulse = math::Clamp(mMotorImpulse + lambda, -maxImpulse, maxImpulse);
        lambda = newImpulse - mMotorImpulse;
        mMotorImpulse = newImpulse;
        ApplyAngularImpulse(mBodyA, mBodyB, hingeAxis * lambda);
    }

    // Limits.
    if (mHasLimits) {
        Vec3 hingeAxis = mBodyA->GetRotation().Rotate(mLocalAxisA);
        Real angle = GetAngle();
        Real relVel = (ma.angularVelocity - mb.angularVelocity).Dot(hingeAxis);
        Real lambda = 0;
        if (angle <= mLowerLimit) {
            lambda = mLimitMass * (-relVel);
            mLimitImpulse = math::Max(mLimitImpulse + lambda, Real(0));
            lambda = mLimitImpulse;
        } else if (angle >= mUpperLimit) {
            lambda = mLimitMass * (-relVel);
            mLimitImpulse = math::Min(mLimitImpulse + lambda, Real(0));
            lambda = mLimitImpulse;
        } else {
            mLimitImpulse = 0;
            lambda = 0;
        }
        if (lambda != Real(0)) ApplyAngularImpulse(mBodyA, mBodyB, hingeAxis * lambda);
    }
}

void HingeConstraint::SolvePosition(Real /*dt*/) {
    Vec3 ra = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    Vec3 rb = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    Vec3 error = (mBodyA->GetPosition() + ra) - (mBodyB->GetPosition() + rb);
    if (error.LengthSq() > Real(1.0e-8)) {
        Mat3 K = PointEffectiveMass(mBodyA, mBodyB, ra, rb).Inversed();
        Vec3 impulse = K * (-error) * Real(0.2);
        ApplyPositionalImpulse(mBodyA, mBodyB, ra, rb, impulse, ra.Cross(impulse));
    }
}

Real HingeConstraint::GetAngle() const {
    Vec3 axis = mBodyA->GetRotation().Rotate(mLocalAxisA);
    Vec3 pA = mBodyA->GetRotation().Rotate(mLocalPerpA[0]);
    Vec3 pB = mBodyB->GetRotation().Rotate(mLocalPerpB[0]);
    Real x = pA.Dot(pB);
    Real y = axis.Dot(pA.Cross(pB));
    return math::Atan2(y, x);
}

// ===========================================================================
// SliderConstraint
// ===========================================================================
SliderConstraint::SliderConstraint(Body* a, Body* b, const Vec3& worldAnchor, const Vec3& worldAxis)
    : Constraint(a, b) {
    mLocalAnchorA = a->GetRotation().InverseRotate(worldAnchor - a->GetPosition());
    mLocalAnchorB = b->GetRotation().InverseRotate(worldAnchor - b->GetPosition());
    mLocalAxisA = a->GetRotation().InverseRotate(worldAxis.Normalized());
    mLocalAxisB = b->GetRotation().InverseRotate(worldAxis.Normalized());
}

void SliderConstraint::Prepare(Real /*dt*/) {
    mR[0] = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    mR[1] = mBodyB->GetRotation().Rotate(mLocalAnchorB);

    Vec3 axis = mBodyA->GetRotation().Rotate(mLocalAxisA);
    Vec3 p1, p2;
    axis.GetBasis(p1, p2);
    mAxialMass = LinearEffectiveMass(mBodyA, mBodyB, mR[0], mR[1], axis);

    // Perpendicular linear constraints (2 DOF) via a 3x3 reduced system.
    Mat3 K = PointEffectiveMass(mBodyA, mBodyB, mR[0], mR[1]);
    // Project onto the plane perpendicular to the axis.
    Vec3 perp[2] = { p1, p2 };
    Real k00 = perp[0].Dot(K * perp[0]);
    Real k01 = perp[0].Dot(K * perp[1]);
    Real k11 = perp[1].Dot(K * perp[1]);
    Real det = k00 * k11 - k01 * k01;
    if (math::Abs(det) > Real(1.0e-12)) {
        Real inv = Real(1) / det;
        mPerpMass = Mat3(
            perp[0] * (k11 * inv) + perp[1] * (-k01 * inv),
            perp[0] * (-k01 * inv) + perp[1] * (k00 * inv),
            Vec3::Zero());
    } else {
        mPerpMass = Mat3::Zero();
    }
    mAngularMass = (mBodyA->GetMotionProperties().inverseInertiaWorld +
                    mBodyB->GetMotionProperties().inverseInertiaWorld).Inversed();
}

void SliderConstraint::WarmStart() {
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], mPerpImpulse);
    ApplyAngularImpulse(mBodyA, mBodyB, mAngularImpulse);
    Vec3 axis = mBodyA->GetRotation().Rotate(mLocalAxisA);
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], axis * mAxialImpulse);
}

void SliderConstraint::SolveVelocity(Real /*dt*/) {
    MotionProperties& ma = mBodyA->GetMotionProperties();
    MotionProperties& mb = mBodyB->GetMotionProperties();
    Vec3 axis = mBodyA->GetRotation().Rotate(mLocalAxisA);

    // Perpendicular linear (remove velocity components not along the axis).
    Vec3 vRel = (ma.linearVelocity + ma.angularVelocity.Cross(mR[0]))
              - (mb.linearVelocity + mb.angularVelocity.Cross(mR[1]));
    Vec3 vPerp = vRel - axis * vRel.Dot(axis);
    Vec3 perpImpulse = -(mPerpMass * vPerp);
    mPerpImpulse += perpImpulse;
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], perpImpulse);

    // Locked rotation.
    Vec3 wRel = ma.angularVelocity - mb.angularVelocity;
    Vec3 angImpulse = mAngularMass * (-wRel);
    mAngularImpulse += angImpulse;
    ApplyAngularImpulse(mBodyA, mBodyB, angImpulse);

    // Motor / axial.
    Real relAxial = (ma.linearVelocity - mb.linearVelocity).Dot(axis);
    if (mHasMotor) {
        Real lambda = mAxialMass * (mMotorTargetVelocity - relAxial);
        Real newImpulse = math::Clamp(mMotorImpulse + lambda, -mMotorMaxImpulse, mMotorMaxImpulse);
        lambda = newImpulse - mMotorImpulse;
        mMotorImpulse = newImpulse;
        ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], axis * lambda);
    } else if (mHasLimits) {
        Real pos = GetPosition();
        Real lambda = 0;
        if (pos <= mLowerLimit) {
            lambda = mAxialMass * (-relAxial);
            mAxialImpulse = math::Max(mAxialImpulse + lambda, Real(0));
            lambda = mAxialImpulse;
        } else if (pos >= mUpperLimit) {
            lambda = mAxialMass * (-relAxial);
            mAxialImpulse = math::Min(mAxialImpulse + lambda, Real(0));
            lambda = mAxialImpulse;
        } else {
            mAxialImpulse = 0;
        }
        if (lambda != Real(0)) ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], axis * lambda);
    }
}

void SliderConstraint::SolvePosition(Real /*dt*/) {
    Vec3 ra = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    Vec3 rb = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    Vec3 pA = mBodyA->GetPosition() + ra;
    Vec3 pB = mBodyB->GetPosition() + rb;
    Vec3 axis = mBodyA->GetRotation().Rotate(mLocalAxisA);
    Vec3 error = pA - pB;
    error -= axis * error.Dot(axis);
    if (error.LengthSq() > Real(1.0e-8)) {
        Mat3 K = PointEffectiveMass(mBodyA, mBodyB, ra, rb).Inversed();
        Vec3 impulse = K * (-error) * Real(0.2);
        impulse -= axis * impulse.Dot(axis);
        ApplyPositionalImpulse(mBodyA, mBodyB, ra, rb, impulse, ra.Cross(impulse));
    }
}

Real SliderConstraint::GetPosition() const {
    Vec3 ra = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    Vec3 rb = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    Vec3 axis = mBodyA->GetRotation().Rotate(mLocalAxisA);
    return ((mBodyB->GetPosition() + rb) - (mBodyA->GetPosition() + ra)).Dot(axis);
}

// ===========================================================================
// SixDOFConstraint
// ===========================================================================
SixDOFConstraint::SixDOFConstraint(Body* a, Body* b, const Vec3& worldAnchor, const Quat& worldRotation)
    : Constraint(a, b) {
    mLocalAnchorA = a->GetRotation().InverseRotate(worldAnchor - a->GetPosition());
    mLocalAnchorB = b->GetRotation().InverseRotate(worldAnchor - b->GetPosition());
    mConstraintRotation = worldRotation.ToMat3();
}

void SixDOFConstraint::Prepare(Real /*dt*/) {
    mR[0] = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    mR[1] = mBodyB->GetRotation().Rotate(mLocalAnchorB);

    // Constraint axes in world space.
    Mat3 rotA = mBodyA->GetRotation().ToMat3() * mConstraintRotation;
    for (u32 i = 0; i < 3; ++i) {
        Vec3 axis = rotA.GetColumn(i);
        mAxes[i][0] = axis.x; mAxes[i][1] = axis.y; mAxes[i][2] = axis.z;
        mAngularAxes[i][0] = axis.x; mAngularAxes[i][1] = axis.y; mAngularAxes[i][2] = axis.z;
    }
}

void SixDOFConstraint::WarmStart() {
    for (u32 i = 0; i < 3; ++i) {
        ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], mLinearImpulse[i]);
        ApplyAngularImpulse(mBodyA, mBodyB, mAngularImpulse[i]);
    }
}

void SixDOFConstraint::SolveLinearAxis(u32 axis, Real /*dt*/) {
    if (mLinearMode[axis] == AxisMode::Free && !mLinearMotorEnabled[axis]) return;
    Vec3 dir(mAxes[axis][0], mAxes[axis][1], mAxes[axis][2]);
    MotionProperties& ma = mBodyA->GetMotionProperties();
    MotionProperties& mb = mBodyB->GetMotionProperties();
    Vec3 vRel = (ma.linearVelocity + ma.angularVelocity.Cross(mR[0]))
              - (mb.linearVelocity + mb.angularVelocity.Cross(mR[1]));
    Real rel = vRel.Dot(dir);
    Real mass = LinearEffectiveMass(mBodyA, mBodyB, mR[0], mR[1], dir);
    Real target = 0;
    Real lambda = mass * (target - rel);
    if (mLinearMotorEnabled[axis]) {
        target = mLinearMotorTarget[axis];
        lambda = mass * (target - rel);
        Real maxImpulse = mLinearMotorMaxImpulse[axis];
        Vec3 current = mLinearImpulse[axis];
        Real accum = current.Dot(dir);
        Real newAccum = math::Clamp(accum + lambda, -maxImpulse, maxImpulse);
        lambda = newAccum - accum;
    } else if (mLinearMode[axis] == AxisMode::Fixed) {
        Vec3 current = mLinearImpulse[axis];
        Real accum = current.Dot(dir);
        lambda = mass * (-rel);
        // Accumulate freely for fixed axes.
        Vec3 impulse = dir * lambda;
        mLinearImpulse[axis] += impulse;
        ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], impulse);
        KZ_UNUSED(accum);
        return;
    }
    Vec3 impulse = dir * lambda;
    mLinearImpulse[axis] += impulse;
    ApplyImpulse(mBodyA, mBodyB, mR[0], mR[1], impulse);
}

void SixDOFConstraint::SolveAngularAxis(u32 axis, Real /*dt*/) {
    if (mAngularMode[axis] == AxisMode::Free && !mAngularMotorEnabled[axis]) return;
    Vec3 dir(mAngularAxes[axis][0], mAngularAxes[axis][1], mAngularAxes[axis][2]);
    MotionProperties& ma = mBodyA->GetMotionProperties();
    MotionProperties& mb = mBodyB->GetMotionProperties();
    Vec3 wRel = ma.angularVelocity - mb.angularVelocity;
    Real rel = wRel.Dot(dir);
    Real mass = AngularEffectiveMass(mBodyA, mBodyB, dir);
    Real lambda;
    if (mAngularMotorEnabled[axis]) {
        lambda = mass * (mAngularMotorTarget[axis] - rel);
        Real accum = mAngularImpulse[axis].Dot(dir);
        Real newAccum = math::Clamp(accum + lambda, -mAngularMotorMaxImpulse[axis], mAngularMotorMaxImpulse[axis]);
        lambda = newAccum - accum;
    } else {
        lambda = mass * (-rel);
    }
    Vec3 impulse = dir * lambda;
    mAngularImpulse[axis] += impulse;
    ApplyAngularImpulse(mBodyA, mBodyB, impulse);
}

void SixDOFConstraint::SolveVelocity(Real dt) {
    for (u32 i = 0; i < 3; ++i) SolveLinearAxis(i, dt);
    for (u32 i = 0; i < 3; ++i) SolveAngularAxis(i, dt);
}

void SixDOFConstraint::SolvePosition(Real /*dt*/) {
    Vec3 ra = mBodyA->GetRotation().Rotate(mLocalAnchorA);
    Vec3 rb = mBodyB->GetRotation().Rotate(mLocalAnchorB);
    Vec3 pA = mBodyA->GetPosition() + ra;
    Vec3 pB = mBodyB->GetPosition() + rb;
    Vec3 error = pA - pB;

    for (u32 i = 0; i < 3; ++i) {
        if (mLinearMode[i] != AxisMode::Fixed) continue;
        Vec3 dir(mAxes[i][0], mAxes[i][1], mAxes[i][2]);
        Real e = error.Dot(dir);
        if (math::Abs(e) < Real(1.0e-4)) continue;
        Real mass = LinearEffectiveMass(mBodyA, mBodyB, ra, rb, dir);
        Vec3 impulse = dir * (-mass * e * Real(0.2));
        ApplyPositionalImpulse(mBodyA, mBodyB, ra, rb, impulse, ra.Cross(impulse));
    }
}

} // namespace kizuri
