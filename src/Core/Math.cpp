// KizuriPhysics - Core/Math.cpp
#include "Kizuri/Core/Math.h"

namespace kizuri {

// ---------------------------------------------------------------------------
// Vec3
// ---------------------------------------------------------------------------
void Vec3::GetBasis(Vec3& outB, Vec3& outC) const {
    // Robust orthonormal basis (Frisvad, "Building an Orthonormal Basis from
    // a Normal Vector").  Both returned vectors are unit length and
    // perpendicular to *this and to each other.
    if (z < Real(-0.9999999)) {
        outB = Vec3(Real(0), Real(-1), Real(0));
        outC = Vec3(Real(-1), Real(0), Real(0));
    } else {
        const Real a = Real(1) / (Real(1) + z);
        const Real b = -x * y * a;
        outB = Vec3(Real(1) - x * x * a, b, -x);
        outC = Vec3(b, Real(1) - y * y * a, -y);
    }
}

// ---------------------------------------------------------------------------
// Mat3
// ---------------------------------------------------------------------------
Mat3 Mat3::Inversed() const {
    const Vec3& a = col[0];
    const Vec3& b = col[1];
    const Vec3& c = col[2];

    Vec3 r0 = b.Cross(c);
    Vec3 r1 = c.Cross(a);
    Vec3 r2 = a.Cross(b);

    Real det = a.Dot(r0);
    Real invDet = det != Real(0) ? Real(1) / det : Real(0);

    // Result columns are the rows of the adjugate (cofactor) matrix.
    return Mat3(Vec3(r0.x, r1.x, r2.x) * invDet,
                Vec3(r0.y, r1.y, r2.y) * invDet,
                Vec3(r0.z, r1.z, r2.z) * invDet);
}

Mat3 Mat3::FromToRotation(const Vec3& from, const Vec3& to) {
    Vec3 f = from.Normalized();
    Vec3 t = to.Normalized();
    Real cosAngle = math::Clamp(f.Dot(t), Real(-1), Real(1));

    if (cosAngle >= Real(1) - Real(1.0e-6))
        return Mat3::Identity();

    if (cosAngle <= Real(-1) + Real(1.0e-6)) {
        // Opposite vectors: 180 degree rotation about any perpendicular axis.
        Vec3 axis = f.GetNormalizedPerpendicular();
        return RotationAxisAngle(axis, math::kPi);
    }

    Vec3 axis = f.Cross(t).Normalized();
    Real angle = math::Acos(cosAngle);
    return RotationAxisAngle(axis, angle);
}

Mat3 Mat3::RotationAxisAngle(const Vec3& axis, Real angle) {
    Real c = math::Cos(angle);
    Real s = math::Sin(angle);
    Real t = Real(1) - c;
    Real x = axis.x, y = axis.y, z = axis.z;
    return Mat3(
        Vec3(t*x*x + c,     t*x*y + s*z, t*x*z - s*y),
        Vec3(t*x*y - s*z,   t*y*y + c,   t*y*z + s*x),
        Vec3(t*x*z + s*y,   t*y*z - s*x, t*z*z + c));
}

// ---------------------------------------------------------------------------
// Quat
// ---------------------------------------------------------------------------
Quat Quat::AxisAngle(const Vec3& axis, Real angle) {
    Real half = angle * Real(0.5);
    Real s = math::Sin(half);
    Vec3 a = axis.Normalized();
    return Quat(a.x * s, a.y * s, a.z * s, math::Cos(half));
}

Quat Quat::FromMat3(const Mat3& m) {
    Real m00 = m.col[0].x, m01 = m.col[1].x, m02 = m.col[2].x;
    Real m10 = m.col[0].y, m11 = m.col[1].y, m12 = m.col[2].y;
    Real m20 = m.col[0].z, m21 = m.col[1].z, m22 = m.col[2].z;

    Real trace = m00 + m11 + m22;
    Quat q;
    if (trace > Real(0)) {
        Real s = math::Sqrt(trace + Real(1)) * Real(2);
        q.w = Real(0.25) * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        Real s = math::Sqrt(Real(1) + m00 - m11 - m22) * Real(2);
        q.w = (m21 - m12) / s;
        q.x = Real(0.25) * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        Real s = math::Sqrt(Real(1) + m11 - m00 - m22) * Real(2);
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = Real(0.25) * s;
        q.z = (m12 + m21) / s;
    } else {
        Real s = math::Sqrt(Real(1) + m22 - m00 - m11) * Real(2);
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = Real(0.25) * s;
    }
    return q.Normalized();
}

Quat Quat::FromTo(const Vec3& from, const Vec3& to) {
    return FromMat3(Mat3::FromToRotation(from, to));
}

Quat Quat::IntegratedAngularVelocity(const Vec3& w, Real dt) const {
    // q' = q + 0.5 * (0, w) * q * dt
    Quat dq(Real(0.5) * dt * w.x, Real(0.5) * dt * w.y, Real(0.5) * dt * w.z, Real(0));
    Quat r = (*this) + dq * (*this);
    return r.Normalized();
}

Quat Quat::GetTwist(const Vec3& axis) const {
    // Project the vector part onto the axis.
    Real proj = x * axis.x + y * axis.y + z * axis.z;
    Quat twist(axis.x * proj, axis.y * proj, axis.z * proj, w);
    return twist.Normalized();
}

Quat Quat::GetSwing(const Vec3& axis) const {
    Quat twist = GetTwist(axis);
    return ((*this) * twist.Conjugated()).Normalized();
}

Real Quat::Angle(const Quat& q) const {
    Real d = math::Abs(math::Clamp(Dot(q), Real(-1), Real(1)));
    return Real(2) * math::Acos(d);
}

Quat Slerp(const Quat& a, const Quat& bIn, Real t) {
    Quat b = bIn;
    Real cosTheta = a.Dot(b);

    // Take the shorter path.
    if (cosTheta < Real(0)) {
        b = -b;
        cosTheta = -cosTheta;
    }

    if (cosTheta > Real(0.9995)) {
        // Very close: linear interpolation then normalize.
        Quat r(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
               a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
        return r.Normalized();
    }

    Real theta = math::Acos(math::Clamp(cosTheta, Real(-1), Real(1)));
    Real sinTheta = math::Sin(theta);
    Real w1 = math::Sin((Real(1) - t) * theta) / sinTheta;
    Real w2 = math::Sin(t * theta) / sinTheta;
    return Quat(a.x * w1 + b.x * w2, a.y * w1 + b.y * w2,
                a.z * w1 + b.z * w2, a.w * w1 + b.w * w2);
}

// ---------------------------------------------------------------------------
// AABB
// ---------------------------------------------------------------------------
AABB AABB::Transformed(const Transform& t) const {
    // Transform the 8 corners and fit.
    Vec3 c = Center();
    Vec3 e = Extent();
    Mat3 m = t.rotation.ToMat3();

    // New center and extent (tight for a rotated box).
    Vec3 newCenter = t * c;
    Vec3 newExtent(
        math::Abs(m.col[0].x) * e.x + math::Abs(m.col[1].x) * e.y + math::Abs(m.col[2].x) * e.z,
        math::Abs(m.col[0].y) * e.x + math::Abs(m.col[1].y) * e.y + math::Abs(m.col[2].y) * e.z,
        math::Abs(m.col[0].z) * e.x + math::Abs(m.col[1].z) * e.y + math::Abs(m.col[2].z) * e.z);
    return AABB(newCenter - newExtent, newCenter + newExtent);
}

AABB AABB::TransformedConservative(const Transform& t) const {
    return Transformed(t);
}

AABB AABB::Transformed(const Mat3& m) const {
    Vec3 c = Center();
    Vec3 e = Extent();
    Vec3 newCenter = m * c;
    Vec3 newExtent(
        math::Abs(m.col[0].x) * e.x + math::Abs(m.col[1].x) * e.y + math::Abs(m.col[2].x) * e.z,
        math::Abs(m.col[0].y) * e.x + math::Abs(m.col[1].y) * e.y + math::Abs(m.col[2].y) * e.z,
        math::Abs(m.col[0].z) * e.x + math::Abs(m.col[1].z) * e.y + math::Abs(m.col[2].z) * e.z);
    return AABB(newCenter - newExtent, newCenter + newExtent);
}

} // namespace kizuri
