// KizuriPhysics - Constraint/Constraints.h
// Joint constraints: point, distance, fixed, hinge, slider and six-DOF.
#pragma once

#include "Kizuri/Body/Body.h"

namespace kizuri {

// ---------------------------------------------------------------------------
// Base constraint
// ---------------------------------------------------------------------------
class Constraint {
public:
    virtual ~Constraint() = default;

    Body* GetBodyA() const { return mBodyA; }
    Body* GetBodyB() const { return mBodyB; }
    bool IsEnabled() const { return mEnabled; }
    void SetEnabled(bool e) { mEnabled = e; }

    /// Called once per step before the solver iterations.
    virtual void Prepare(Real dt) = 0;
    /// Apply warm-start impulses.
    virtual void WarmStart() = 0;
    /// One velocity iteration.
    virtual void SolveVelocity(Real dt) = 0;
    /// One position iteration.
    virtual void SolvePosition(Real dt) = 0;

protected:
    Constraint(Body* a, Body* b) : mBodyA(a), mBodyB(b) {}

    Body* mBodyA = nullptr;
    Body* mBodyB = nullptr;
    bool mEnabled = true;
};

// ---------------------------------------------------------------------------
// Point constraint (ball-socket): keeps two local anchors coincident.
// ---------------------------------------------------------------------------
class PointConstraint final : public Constraint {
public:
    PointConstraint(Body* a, Body* b, const Vec3& worldAnchor);
    PointConstraint(Body* a, Body* b, const Vec3& localAnchorA, const Vec3& localAnchorB);

    void Prepare(Real dt) override;
    void WarmStart() override;
    void SolveVelocity(Real dt) override;
    void SolvePosition(Real dt) override;

    void SetAnchorA(const Vec3& local) { mLocalAnchorA = local; }
    void SetAnchorB(const Vec3& local) { mLocalAnchorB = local; }
    const Vec3& GetLocalAnchorA() const { return mLocalAnchorA; }
    const Vec3& GetLocalAnchorB() const { return mLocalAnchorB; }

private:
    Vec3 mLocalAnchorA;
    Vec3 mLocalAnchorB;
    Vec3 mR[2] = { Vec3::Zero(), Vec3::Zero() };
    Mat3 mEffectiveMass = Mat3::Zero();
    Vec3 mImpulse = Vec3::Zero();
    Vec3 mVelocityBias = Vec3::Zero();
};

// ---------------------------------------------------------------------------
// Distance constraint: keeps two anchors at a fixed distance.
// ---------------------------------------------------------------------------
class DistanceConstraint final : public Constraint {
public:
    DistanceConstraint(Body* a, Body* b, const Vec3& worldAnchorA, const Vec3& worldAnchorB,
                       Real distance);

    void Prepare(Real dt) override;
    void WarmStart() override;
    void SolveVelocity(Real dt) override;
    void SolvePosition(Real dt) override;

    Real GetDistance() const { return mTargetDistance; }

private:
    Vec3 mLocalAnchorA;
    Vec3 mLocalAnchorB;
    Real mTargetDistance;
    Real mEffectiveMass = 0;
    Real mImpulse = 0;
    Vec3 mAxis = Vec3::UnitY();
    Vec3 mR[2] = { Vec3::Zero(), Vec3::Zero() };
};

// ---------------------------------------------------------------------------
// Fixed constraint: rigidly welds two bodies.
// ---------------------------------------------------------------------------
class FixedConstraint final : public Constraint {
public:
    FixedConstraint(Body* a, Body* b, const Vec3& worldAnchor);

    void Prepare(Real dt) override;
    void WarmStart() override;
    void SolveVelocity(Real dt) override;
    void SolvePosition(Real dt) override;

private:
    Vec3 mLocalAnchorA;
    Vec3 mLocalAnchorB;
    Quat mRelativeRotation;
    Vec3 mR[2] = { Vec3::Zero(), Vec3::Zero() };
    Mat3 mLinearMass = Mat3::Zero();
    Mat3 mAngularMass = Mat3::Zero();
    Vec3 mLinearImpulse = Vec3::Zero();
    Vec3 mAngularImpulse = Vec3::Zero();
};

// ---------------------------------------------------------------------------
// Hinge constraint (revolute): one rotational degree of freedom about an axis.
// Supports limits and a motor.
// ---------------------------------------------------------------------------
class HingeConstraint final : public Constraint {
public:
    HingeConstraint(Body* a, Body* b, const Vec3& worldAnchor,
                    const Vec3& worldAxisA, const Vec3& worldAxisB);

    void Prepare(Real dt) override;
    void WarmStart() override;
    void SolveVelocity(Real dt) override;
    void SolvePosition(Real dt) override;

    void SetLimits(Real lower, Real upper) { mLowerLimit = lower; mUpperLimit = upper; mHasLimits = true; }
    void SetMotor(Real targetVelocity, Real maxTorque) {
        mMotorTargetVelocity = targetVelocity;
        mMotorMaxImpulse = maxTorque;
        mHasMotor = true;
    }
    void DisableMotor() { mHasMotor = false; }
    void DisableLimits() { mHasLimits = false; }

    /// Current hinge angle in radians.
    Real GetAngle() const;

private:
    Vec3 mLocalAnchorA;
    Vec3 mLocalAnchorB;
    Vec3 mLocalAxisA;
    Vec3 mLocalAxisB;

    // Two axes perpendicular to the hinge axis (locked).
    Vec3 mLocalPerpA[2];
    Vec3 mLocalPerpB[2];

    Vec3 mR[2] = { Vec3::Zero(), Vec3::Zero() };
    Mat3 mPointMass = Mat3::Zero();
    Vec3 mPointImpulse = Vec3::Zero();

    Real mAngularMass[2] = { 0, 0 };
    Real mAngularImpulse[2] = { 0, 0 };

    Real mLimitMass = 0;
    Real mLimitImpulse = 0;
    Real mAngle = 0;

    bool mHasLimits = false;
    Real mLowerLimit = -math::kPi;
    Real mUpperLimit = math::kPi;

    bool mHasMotor = false;
    Real mMotorTargetVelocity = 0;
    Real mMotorMaxImpulse = 0;
    Real mMotorImpulse = 0;
};

// ---------------------------------------------------------------------------
// Slider constraint (prismatic): translation along one axis, other DOFs locked.
// ---------------------------------------------------------------------------
class SliderConstraint final : public Constraint {
public:
    SliderConstraint(Body* a, Body* b, const Vec3& worldAnchor, const Vec3& worldAxis);

    void Prepare(Real dt) override;
    void WarmStart() override;
    void SolveVelocity(Real dt) override;
    void SolvePosition(Real dt) override;

    void SetLimits(Real lower, Real upper) { mLowerLimit = lower; mUpperLimit = upper; mHasLimits = true; }
    void SetMotor(Real targetVelocity, Real maxForce) {
        mMotorTargetVelocity = targetVelocity;
        mMotorMaxImpulse = maxForce;
        mHasMotor = true;
    }

    Real GetPosition() const;

private:
    Vec3 mLocalAnchorA;
    Vec3 mLocalAnchorB;
    Vec3 mLocalAxisA;
    Vec3 mLocalAxisB;

    Vec3 mR[2] = { Vec3::Zero(), Vec3::Zero() };
    Mat3 mPerpMass = Mat3::Zero();   // constrained perpendicular directions
    Mat3 mAngularMass = Mat3::Zero(); // locked rotations
    Vec3 mPerpImpulse = Vec3::Zero();
    Vec3 mAngularImpulse = Vec3::Zero();
    Real mAxialMass = 0;
    Real mAxialImpulse = 0;

    bool mHasLimits = false;
    Real mLowerLimit = 0;
    Real mUpperLimit = 0;
    bool mHasMotor = false;
    Real mMotorTargetVelocity = 0;
    Real mMotorMaxImpulse = 0;
    Real mMotorImpulse = 0;
};

// ---------------------------------------------------------------------------
// Six-DOF constraint: per-axis linear and angular freedom, limits and motors.
// ---------------------------------------------------------------------------
class SixDOFConstraint final : public Constraint {
public:
    enum class AxisMode : u8 { Free, Limited, Fixed };

    SixDOFConstraint(Body* a, Body* b, const Vec3& worldAnchor, const Quat& worldRotation);

    void SetLinearAxisMode(u32 axis, AxisMode mode) { mLinearMode[axis] = mode; }
    void SetAngularAxisMode(u32 axis, AxisMode mode) { mAngularMode[axis] = mode; }
    void SetLinearLimits(u32 axis, Real lower, Real upper) {
        mLinearLower[axis] = lower; mLinearUpper[axis] = upper;
    }
    void SetAngularLimits(u32 axis, Real lower, Real upper) {
        mAngularLower[axis] = lower; mAngularUpper[axis] = upper;
    }
    void SetLinearMotor(u32 axis, Real targetVelocity, Real maxForce) {
        mLinearMotorTarget[axis] = targetVelocity;
        mLinearMotorMaxImpulse[axis] = maxForce;
        mLinearMotorEnabled[axis] = true;
    }
    void SetAngularMotor(u32 axis, Real targetVelocity, Real maxTorque) {
        mAngularMotorTarget[axis] = targetVelocity;
        mAngularMotorMaxImpulse[axis] = maxTorque;
        mAngularMotorEnabled[axis] = true;
    }

    void Prepare(Real dt) override;
    void WarmStart() override;
    void SolveVelocity(Real dt) override;
    void SolvePosition(Real dt) override;

private:
    void SolveLinearAxis(u32 axis, Real dt);
    void SolveAngularAxis(u32 axis, Real dt);

    Vec3 mLocalAnchorA;
    Vec3 mLocalAnchorB;
    Mat3 mConstraintRotation; // rotation from A's frame to B's frame at creation

    AxisMode mLinearMode[3] = { AxisMode::Free, AxisMode::Free, AxisMode::Free };
    AxisMode mAngularMode[3] = { AxisMode::Free, AxisMode::Free, AxisMode::Free };

    Real mLinearLower[3] = { 0, 0, 0 };
    Real mLinearUpper[3] = { 0, 0, 0 };
    Real mAngularLower[3] = { 0, 0, 0 };
    Real mAngularUpper[3] = { 0, 0, 0 };

    bool mLinearMotorEnabled[3] = { false, false, false };
    Real mLinearMotorTarget[3] = { 0, 0, 0 };
    Real mLinearMotorMaxImpulse[3] = { 0, 0, 0 };
    bool mAngularMotorEnabled[3] = { false, false, false };
    Real mAngularMotorTarget[3] = { 0, 0, 0 };
    Real mAngularMotorMaxImpulse[3] = { 0, 0, 0 };

    Vec3 mR[2] = { Vec3::Zero(), Vec3::Zero() };
    Vec3 mLinearImpulse[3] = { Vec3::Zero(), Vec3::Zero(), Vec3::Zero() };
    Vec3 mAngularImpulse[3] = { Vec3::Zero(), Vec3::Zero(), Vec3::Zero() };
    Real mAxes[3][3]; // world constraint axes for linear (row per axis)
    Real mAngularAxes[3][3];
};

} // namespace kizuri
