// KizuriPhysics - Core/Math.h
// Vector / quaternion / matrix / transform / AABB primitives.
#pragma once

#include "Types.h"

#include <cstring>

namespace kizuri {

struct Mat3;
struct Mat4;
struct Quat;
struct Transform;

// ===========================================================================
// Vec3
// ===========================================================================
struct alignas(16) Vec3 {
    Real x, y, z;
    Real _pad = 0; // keeps 16-byte alignment for SSE loads

    constexpr Vec3() : x(0), y(0), z(0) {}
    constexpr Vec3(Real s) : x(s), y(s), z(s) {}
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}

    KZ_FORCEINLINE Real*       Data()       { return &x; }
    KZ_FORCEINLINE const Real* Data() const { return &x; }

    KZ_FORCEINLINE Real operator[](int i) const { return Data()[i]; }
    KZ_FORCEINLINE Real& operator[](int i)      { return Data()[i]; }

    KZ_FORCEINLINE Vec3 operator-() const { return { -x, -y, -z }; }
    KZ_FORCEINLINE Vec3 operator+(const Vec3& o) const { return { x + o.x, y + o.y, z + o.z }; }
    KZ_FORCEINLINE Vec3 operator-(const Vec3& o) const { return { x - o.x, y - o.y, z - o.z }; }
    KZ_FORCEINLINE Vec3 operator*(const Vec3& o) const { return { x * o.x, y * o.y, z * o.z }; }
    KZ_FORCEINLINE Vec3 operator/(const Vec3& o) const { return { x / o.x, y / o.y, z / o.z }; }
    KZ_FORCEINLINE Vec3 operator*(Real s) const { return { x * s, y * s, z * s }; }
    KZ_FORCEINLINE Vec3 operator/(Real s) const { Real inv = Real(1) / s; return { x * inv, y * inv, z * inv }; }

    KZ_FORCEINLINE Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    KZ_FORCEINLINE Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    KZ_FORCEINLINE Vec3& operator*=(const Vec3& o) { x *= o.x; y *= o.y; z *= o.z; return *this; }
    KZ_FORCEINLINE Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
    KZ_FORCEINLINE Vec3& operator/=(Real s) { Real inv = Real(1) / s; x *= inv; y *= inv; z *= inv; return *this; }

    KZ_FORCEINLINE bool operator==(const Vec3& o) const { return x == o.x && y == o.y && z == o.z; }
    KZ_FORCEINLINE bool operator!=(const Vec3& o) const { return !(*this == o); }

    KZ_FORCEINLINE Real LengthSq() const { return x * x + y * y + z * z; }
    KZ_FORCEINLINE Real Length()   const { return math::Sqrt(LengthSq()); }
    KZ_FORCEINLINE Real Dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    KZ_FORCEINLINE Vec3 Cross(const Vec3& o) const {
        return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x };
    }

    /// Normalized copy; returns zero vector when below epsilon.
    KZ_FORCEINLINE Vec3 Normalized() const {
        Real len = Length();
        return len >= math::kEpsilon ? *this / len : Vec3(Real(0));
    }
    /// Normalize in place; returns the original length.
    KZ_FORCEINLINE Real Normalize() {
        Real len = Length();
        if (len >= math::kEpsilon) *this /= len;
        return len;
    }
    /// Normalize or return the fallback when degenerate.
    KZ_FORCEINLINE Vec3 NormalizedOr(const Vec3& fallback) const {
        Real len = Length();
        return len >= math::kEpsilon ? *this / len : fallback;
    }

    KZ_FORCEINLINE Vec3 Abs() const { return { math::Abs(x), math::Abs(y), math::Abs(z) }; }
    KZ_FORCEINLINE Vec3 Min(const Vec3& o) const { return { math::Min(x,o.x), math::Min(y,o.y), math::Min(z,o.z) }; }
    KZ_FORCEINLINE Vec3 Max(const Vec3& o) const { return { math::Max(x,o.x), math::Max(y,o.y), math::Max(z,o.z) }; }
    KZ_FORCEINLINE Real MaxComponent() const { return math::Max(x, math::Max(y, z)); }
    KZ_FORCEINLINE Real MinComponent() const { return math::Min(x, math::Min(y, z)); }
    KZ_FORCEINLINE int  MaxComponentIndex() const { return x >= y ? (x >= z ? 0 : 2) : (y >= z ? 1 : 2); }

    KZ_FORCEINLINE bool IsFinite() const { return math::IsFinite(x) && math::IsFinite(y) && math::IsFinite(z); }
    KZ_FORCEINLINE bool IsNearZero(Real tol = math::kEpsilon) const { return Abs().MaxComponent() <= tol; }

    /// Returns true if *this is (approximately) parallel to o.
    KZ_FORCEINLINE bool IsParallel(const Vec3& o, Real tol = Real(1.0e-6)) const {
        return Cross(o).LengthSq() <= tol * tol;
    }
    /// Returns true if *this is (approximately) perpendicular to o.
    KZ_FORCEINLINE bool IsPerpendicular(const Vec3& o, Real tol = Real(1.0e-6)) const {
        return math::Abs(Dot(o)) <= tol;
    }

    static KZ_FORCEINLINE Vec3 Zero()  { return { Real(0), Real(0), Real(0) }; }
    static KZ_FORCEINLINE Vec3 One()   { return { Real(1), Real(1), Real(1) }; }
    static KZ_FORCEINLINE Vec3 UnitX() { return { Real(1), Real(0), Real(0) }; }
    static KZ_FORCEINLINE Vec3 UnitY() { return { Real(0), Real(1), Real(0) }; }
    static KZ_FORCEINLINE Vec3 UnitZ() { return { Real(0), Real(0), Real(1) }; }
    static KZ_FORCEINLINE Vec3 Infinity() { return { math::kInfinity, math::kInfinity, math::kInfinity }; }
    static KZ_FORCEINLINE Vec3 NegInfinity() { return { -math::kInfinity, -math::kInfinity, -math::kInfinity }; }

    /// Any unit vector perpendicular to *this (assumes *this is normalized).
    KZ_FORCEINLINE Vec3 GetNormalizedPerpendicular() const {
        if (math::Abs(x) > math::Abs(y)) {
            Real len = math::Sqrt(x * x + z * z);
            return { -z / len, Real(0), x / len };
        } else {
            Real len = math::Sqrt(y * y + z * z);
            return { Real(0), z / len, -y / len };
        }
    }

    /// A stable orthonormal basis with *this as column 0.
    void GetBasis(Vec3& outB, Vec3& outC) const;
};

KZ_FORCEINLINE Vec3 operator*(Real s, const Vec3& v) { return v * s; }

/// Component-wise linear interpolation.
KZ_FORCEINLINE Vec3 Lerp(const Vec3& a, const Vec3& b, Real t) { return a + (b - a) * t; }

/// Project a onto b (b need not be normalized).
KZ_FORCEINLINE Vec3 Project(const Vec3& a, const Vec3& b) {
    Real d = b.LengthSq();
    return d >= math::kEpsilon ? b * (a.Dot(b) / d) : Vec3(Real(0));
}
/// Reject (component of a perpendicular to b).
KZ_FORCEINLINE Vec3 Reject(const Vec3& a, const Vec3& b) { return a - Project(a, b); }

/// Clamp the magnitude of v to maxLength.
KZ_FORCEINLINE Vec3 ClampLength(const Vec3& v, Real maxLength) {
    Real len = v.Length();
    return len > maxLength && len >= math::kEpsilon ? v * (maxLength / len) : v;
}

// ===========================================================================
// Vec4 (used by the SoA solver)
// ===========================================================================
struct alignas(16) Vec4 {
    Real x, y, z, w;
    constexpr Vec4() : x(0), y(0), z(0), w(0) {}
    constexpr Vec4(Real s) : x(s), y(s), z(s), w(s) {}
    constexpr Vec4(Real x_, Real y_, Real z_, Real w_) : x(x_), y(y_), z(z_), w(w_) {}
    constexpr Vec4(const Vec3& v, Real w_) : x(v.x), y(v.y), z(v.z), w(w_) {}

    KZ_FORCEINLINE Real*       Data()       { return &x; }
    KZ_FORCEINLINE const Real* Data() const { return &x; }

    KZ_FORCEINLINE Vec4 operator+(const Vec4& o) const { return { x+o.x, y+o.y, z+o.z, w+o.w }; }
    KZ_FORCEINLINE Vec4 operator-(const Vec4& o) const { return { x-o.x, y-o.y, z-o.z, w-o.w }; }
    KZ_FORCEINLINE Vec4 operator*(const Vec4& o) const { return { x*o.x, y*o.y, z*o.z, w*o.w }; }
    KZ_FORCEINLINE Vec4 operator*(Real s) const { return { x*s, y*s, z*s, w*s }; }
    KZ_FORCEINLINE Vec4 operator/(Real s) const { Real inv = Real(1)/s; return { x*inv, y*inv, z*inv, w*inv }; }

    KZ_FORCEINLINE Real Dot(const Vec4& o) const { return x*o.x + y*o.y + z*o.z + w*o.w; }
    KZ_FORCEINLINE Vec3 XYZ() const { return { x, y, z }; }
    KZ_FORCEINLINE Vec4 SplatX() const { return { x, x, x, x }; }
    KZ_FORCEINLINE Vec4 SplatY() const { return { y, y, y, y }; }
    KZ_FORCEINLINE Vec4 SplatZ() const { return { z, z, z, z }; }
    KZ_FORCEINLINE Vec4 SplatW() const { return { w, w, w, w }; }
    static KZ_FORCEINLINE Vec4 Load(const Real* p) { return { p[0], p[1], p[2], p[3] }; }
    KZ_FORCEINLINE void Store(Real* p) const { p[0]=x; p[1]=y; p[2]=z; p[3]=w; }
};

// ===========================================================================
// Mat3 - column-major rotation matrix
// ===========================================================================
struct alignas(16) Mat3 {
    // Columns.
    Vec3 col[3];

    constexpr Mat3()
        : col{ Vec3(Real(1), Real(0), Real(0)), Vec3(Real(0), Real(1), Real(0)), Vec3(Real(0), Real(0), Real(1)) } {}
    explicit constexpr Mat3(Real diagonal)
        : col{ Vec3(diagonal, Real(0), Real(0)), Vec3(Real(0), diagonal, Real(0)), Vec3(Real(0), Real(0), diagonal) } {}
    constexpr Mat3(const Vec3& c0, const Vec3& c1, const Vec3& c2) : col{ c0, c1, c2 } {}

    KZ_FORCEINLINE Vec3& operator[](int i) { return col[i]; }
    KZ_FORCEINLINE const Vec3& operator[](int i) const { return col[i]; }
    /// Column access (matrix * vector).
    KZ_FORCEINLINE Vec3 GetColumn(int i) const { return col[i]; }

    /// Row access (vector * matrix).
    KZ_FORCEINLINE Vec3 GetRow(int i) const { return { col[0][i], col[1][i], col[2][i] }; }

    static Mat3 Zero() { return Mat3(Vec3(Real(0)), Vec3(Real(0)), Vec3(Real(0))); }
    static Mat3 Identity() { return Mat3(); }

    /// Outer product a * b^T.
    static Mat3 OuterProduct(const Vec3& a, const Vec3& b) {
        return Mat3(a * b.x, a * b.y, a * b.z);
    }
    /// Skew-symmetric cross-product matrix such that Skew(v)*u == v.Cross(u).
    static Mat3 Skew(const Vec3& v) {
        return Mat3(Vec3(Real(0), v.z, -v.y),
                    Vec3(-v.z, Real(0), v.x),
                    Vec3(v.y, -v.x, Real(0)));
    }
    /// Symmetric cross-product matrices: -Skew(a)*Skew(b).
    static Mat3 CrossProductMatrix(const Vec3& a, const Vec3& b) {
        return Mat3(
            Vec3(-(a.y * b.y + a.z * b.z), a.y * b.x, a.z * b.x),
            Vec3(a.x * b.y, -(a.x * b.x + a.z * b.z), a.z * b.y),
            Vec3(a.x * b.z, a.y * b.z, -(a.x * b.x + a.y * b.y)));
    }

    KZ_FORCEINLINE Mat3 operator+(const Mat3& o) const { return { col[0]+o.col[0], col[1]+o.col[1], col[2]+o.col[2] }; }
    KZ_FORCEINLINE Mat3 operator-(const Mat3& o) const { return { col[0]-o.col[0], col[1]-o.col[1], col[2]-o.col[2] }; }
    KZ_FORCEINLINE Mat3 operator*(Real s) const { return { col[0]*s, col[1]*s, col[2]*s }; }
    KZ_FORCEINLINE Mat3 operator*(const Mat3& o) const {
        return { (*this) * o.col[0], (*this) * o.col[1], (*this) * o.col[2] };
    }
    KZ_FORCEINLINE Mat3& operator+=(const Mat3& o) { col[0]+=o.col[0]; col[1]+=o.col[1]; col[2]+=o.col[2]; return *this; }
    KZ_FORCEINLINE Mat3& operator-=(const Mat3& o) { col[0]-=o.col[0]; col[1]-=o.col[1]; col[2]-=o.col[2]; return *this; }

    /// Matrix * vector.
    KZ_FORCEINLINE Vec3 operator*(const Vec3& v) const { return col[0]*v.x + col[1]*v.y + col[2]*v.z; }
    /// Vector * matrix (i.e. v^T * M).
    KZ_FORCEINLINE Vec3 MultiplyTransposed(const Vec3& v) const {
        return { col[0].Dot(v), col[1].Dot(v), col[2].Dot(v) };
    }

    KZ_FORCEINLINE Mat3 Transposed() const {
        return Mat3(GetRow(0), GetRow(1), GetRow(2));
    }
    KZ_FORCEINLINE Real Determinant() const { return col[0].Dot(col[1].Cross(col[2])); }

    /// Inverse of a general 3x3 matrix.
    Mat3 Inversed() const;
    /// Inverse assuming an orthonormal rotation matrix (== transpose).
    KZ_FORCEINLINE Mat3 InversedRotation() const { return Transposed(); }

    /// Build a diagonal matrix.
    static Mat3 Scale(const Vec3& s) { return Mat3(Vec3(s.x,0,0), Vec3(0,s.y,0), Vec3(0,0,s.z)); }

    /// Skew-symmetric matrix of a vector; convenience.
    KZ_FORCEINLINE static Mat3 SkewSymmetric(const Vec3& v) { return Skew(v); }

    /// Matrix that transforms a to b (a rotation) via shortest arc.
    static Mat3 FromToRotation(const Vec3& from, const Vec3& to);
    /// Rodrigues rotation about a unit axis.
    static Mat3 RotationAxisAngle(const Vec3& axis, Real angle);
};

/// Matrix-vector product of the transpose: M^T * v.
KZ_FORCEINLINE Vec3 TransposedMultiply(const Mat3& m, const Vec3& v) { return m.MultiplyTransposed(v); }

// ===========================================================================
// Quat - unit quaternion
// ===========================================================================
struct alignas(16) Quat {
    Real x, y, z, w;

    constexpr Quat() : x(0), y(0), z(0), w(1) {}
    constexpr Quat(Real x_, Real y_, Real z_, Real w_) : x(x_), y(y_), z(z_), w(w_) {}
    /// From axis (must be unit) and angle in radians.
    static Quat AxisAngle(const Vec3& axis, Real angle);
    /// From a rotation matrix.
    static Quat FromMat3(const Mat3& m);
    /// From Euler angles (radians), applied in XYZ intrinsic order.
    static Quat FromEuler(const Vec3& radians) {
        Real cx = math::Cos(radians.x * Real(0.5)), sx = math::Sin(radians.x * Real(0.5));
        Real cy = math::Cos(radians.y * Real(0.5)), sy = math::Sin(radians.y * Real(0.5));
        Real cz = math::Cos(radians.z * Real(0.5)), sz = math::Sin(radians.z * Real(0.5));
        return Quat(
            sx * cy * cz + cx * sy * sz,
            cx * sy * cz - sx * cy * sz,
            cx * cy * sz + sx * sy * cz,
            cx * cy * cz - sx * sy * sz);
    }

    KZ_FORCEINLINE Quat operator*(const Quat& q) const {
        return Quat(
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w,
            w * q.w - x * q.x - y * q.y - z * q.z);
    }
    KZ_FORCEINLINE Quat operator*(Real s) const { return { x*s, y*s, z*s, w*s }; }
    KZ_FORCEINLINE Quat operator+(const Quat& q) const { return { x+q.x, y+q.y, z+q.z, w+q.w }; }
    KZ_FORCEINLINE Quat operator-() const { return { -x, -y, -z, -w }; }

    KZ_FORCEINLINE Real Dot(const Quat& q) const { return x*q.x + y*q.y + z*q.z + w*q.w; }
    KZ_FORCEINLINE Real LengthSq() const { return x*x + y*y + z*z + w*w; }
    KZ_FORCEINLINE Real Length() const { return math::Sqrt(LengthSq()); }

    KZ_FORCEINLINE Quat Conjugated() const { return { -x, -y, -z, w }; }
    /// Inverse for a unit quaternion (== conjugate). Use Normalized().Conjugated() otherwise.
    KZ_FORCEINLINE Quat Inversed() const { return Conjugated(); }

    Quat Normalized() const {
        Real len = Length();
        return len >= math::kEpsilon ? *this * (Real(1) / len) : Quat();
    }

    /// Rotate a vector: q * v * q^-1, expanded for speed.
    KZ_FORCEINLINE Vec3 Rotate(const Vec3& v) const {
        // t = 2 * (q_vec x v); v' = v + w*t + q_vec x t
        Vec3 qv(x, y, z);
        Vec3 t = qv.Cross(v) * Real(2);
        return v + t * w + qv.Cross(t);
    }
    /// Inverse rotation.
    KZ_FORCEINLINE Vec3 InverseRotate(const Vec3& v) const {
        Vec3 qv(-x, -y, -z);
        Vec3 t = qv.Cross(v) * Real(2);
        return v + t * w + qv.Cross(t);
    }

    /// Rotate by the shortest arc taking from to to (both need not be normalized).
    static Quat FromTo(const Vec3& from, const Vec3& to);
    /// Integrate an angular velocity over dt: q' = q + 0.5*w*q*dt, renormalized.
    Quat IntegratedAngularVelocity(const Vec3& angularVelocity, Real dt) const;

    KZ_FORCEINLINE Mat3 ToMat3() const {
        Real xx = x*x, yy = y*y, zz = z*z;
        Real xy = x*y, xz = x*z, yz = y*z;
        Real wx = w*x, wy = w*y, wz = w*z;
        return Mat3(
            Vec3(Real(1) - Real(2)*(yy+zz), Real(2)*(xy+wz),       Real(2)*(xz-wy)),
            Vec3(Real(2)*(xy-wz),       Real(1) - Real(2)*(xx+zz), Real(2)*(yz+wx)),
            Vec3(Real(2)*(xz+wy),       Real(2)*(yz-wx),           Real(1) - Real(2)*(xx+yy)));
    }

    /// Get the twist component about a unit axis.
    Quat GetTwist(const Vec3& axis) const;
    /// Get the swing component (twist removed).
    Quat GetSwing(const Vec3& axis) const;

    /// Angular distance between two rotations, in radians.
    Real Angle(const Quat& q) const;

    KZ_FORCEINLINE Vec3 GetAxis() const {
        Real s = math::Sqrt(math::Max(Real(0), Real(1) - w * w));
        return s >= math::kEpsilon ? Vec3(x, y, z) / s : Vec3::UnitX();
    }
    KZ_FORCEINLINE Real GetAngle() const { return Real(2) * math::Acos(math::Clamp(w, Real(-1), Real(1))); }

    KZ_FORCEINLINE static Quat Identity() { return Quat(); }

    /// Build from a 3x3 matrix (alias).
    KZ_FORCEINLINE static Quat FromRotation(const Mat3& m) { return FromMat3(m); }
};

/// Spherical linear interpolation between unit quaternions.
Quat Slerp(const Quat& a, const Quat& b, Real t);

// ===========================================================================
// Transform - rotation + translation
// ===========================================================================
struct alignas(16) Transform {
    Quat rotation;
    Vec3 translation;

    constexpr Transform() : rotation(), translation() {}
    constexpr Transform(const Quat& r, const Vec3& t) : rotation(r), translation(t) {}
    explicit constexpr Transform(const Vec3& t) : rotation(), translation(t) {}

    /// Compose: (*this) then other applied as parent. result = this * other.
    KZ_FORCEINLINE Transform operator*(const Transform& o) const {
        return Transform(rotation * o.rotation, rotation.Rotate(o.translation) + translation);
    }
    /// Transform a point.
    KZ_FORCEINLINE Vec3 operator*(const Vec3& p) const { return rotation.Rotate(p) + translation; }
    /// Transform a direction.
    KZ_FORCEINLINE Vec3 TransformDirection(const Vec3& d) const { return rotation.Rotate(d); }
    /// Inverse transform of a point.
    KZ_FORCEINLINE Vec3 InverseTransformPoint(const Vec3& p) const { return rotation.InverseRotate(p - translation); }

    KZ_FORCEINLINE Transform Inverse() const {
        Quat inv = rotation.Inversed();
        return Transform(inv, -inv.Rotate(translation));
    }
    /// Compose with a child transform; equivalent to (*this) * child.
    KZ_FORCEINLINE Transform MultiplyChild(const Transform& child) const { return (*this) * child; }

    static Transform Identity() { return Transform(); }
    static Transform Translation(const Vec3& t) { return Transform(Quat(), t); }
    static Transform Rotation(const Quat& q) { return Transform(q, Vec3::Zero()); }
    static Transform TranslationRotation(const Vec3& t, const Quat& q) { return Transform(q, t); }
};

/// Compose transforms in the "local to parent" convention used by Jolt.
KZ_FORCEINLINE Transform operator*(const Transform& a, const Quat& q) {
    return Transform(a.rotation * q, a.translation);
}

// ===========================================================================
// AABB - axis aligned bounding box
// ===========================================================================
struct alignas(16) AABB {
    Vec3 min;
    Vec3 max;

    constexpr AABB()
        : min(math::kInfinity), max(-math::kInfinity) {}
    constexpr AABB(const Vec3& mn, const Vec3& mx) : min(mn), max(mx) {}

    KZ_FORCEINLINE void SetEmpty() { min = Vec3(math::kInfinity); max = Vec3(-math::kInfinity); }
    KZ_FORCEINLINE bool IsEmpty() const { return max.x < min.x || max.y < min.y || max.z < min.z; }

    KZ_FORCEINLINE void Encapsulate(const Vec3& p) { min = min.Min(p); max = max.Max(p); }
    KZ_FORCEINLINE void Encapsulate(const AABB& b) { min = min.Min(b.min); max = max.Max(b.max); }
    KZ_FORCEINLINE void Expand(Real amount) { min -= Vec3(amount); max += Vec3(amount); }

    KZ_FORCEINLINE Vec3 Center() const { return (min + max) * Real(0.5); }
    KZ_FORCEINLINE Vec3 Extent() const { return (max - min) * Real(0.5); }
    KZ_FORCEINLINE Vec3 Size() const { return max - min; }
    KZ_FORCEINLINE Real SurfaceArea() const {
        Vec3 d = max - min;
        return Real(2) * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
    KZ_FORCEINLINE Real Volume() const {
        Vec3 d = max - min;
        return d.x * d.y * d.z;
    }
    /// Index of the longest axis.
    KZ_FORCEINLINE int GetLongestAxis() const {
        Vec3 d = max - min;
        return d.MaxComponentIndex();
    }

    KZ_FORCEINLINE bool Contains(const Vec3& p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z;
    }
    KZ_FORCEINLINE bool Overlaps(const AABB& o) const {
        return min.x <= o.max.x && max.x >= o.min.x &&
               min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }
    KZ_FORCEINLINE bool Contains(const AABB& o) const {
        return min.x <= o.min.x && max.x >= o.max.x &&
               min.y <= o.min.y && max.y >= o.max.y &&
               min.z <= o.min.z && max.z >= o.max.z;
    }
    /// Squared distance between two boxes (0 when overlapping).
    Real DistanceSq(const AABB& o) const {
        Vec3 d = (min - o.max).Max(o.min - max).Max(Vec3(Real(0)));
        return d.Dot(d);
    }

    /// Transform this AABB by a transform, producing the tightest enclosing AABB.
    AABB Transformed(const Transform& t) const;
    /// Fast conservative transform: use the rotation's absolute matrix.
    AABB TransformedConservative(const Transform& t) const;
    /// Transform by a matrix (columns are axes).
    AABB Transformed(const Mat3& m) const;
};

KZ_FORCEINLINE AABB Merge(const AABB& a, const AABB& b) { return AABB(a.min.Min(b.min), a.max.Max(b.max)); }
KZ_FORCEINLINE AABB Intersect(const AABB& a, const AABB& b) {
    AABB r(a.min.Max(b.min), a.max.Min(b.max));
    return r;
}

// ===========================================================================
// Ray / Plane
// ===========================================================================
struct Ray {
    Vec3 origin;
    Vec3 direction; // should be normalized

    Ray() = default;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    KZ_FORCEINLINE Vec3 PointAt(Real t) const { return origin + direction * t; }
};

struct RayHit {
    Real fraction = math::kBigNumber;
    Vec3 normal;
    bool hit = false;
};

struct Plane {
    Vec3 normal;   // unit
    Real distance; // plane: dot(normal, p) = distance

    Plane() : normal(Vec3::UnitY()), distance(0) {}
    Plane(const Vec3& n, Real d) : normal(n), distance(d) {}
    Plane(const Vec3& n, const Vec3& p) : normal(n), distance(n.Dot(p)) {}

    KZ_FORCEINLINE Real SignedDistance(const Vec3& p) const { return normal.Dot(p) - distance; }
};

} // namespace kizuri
