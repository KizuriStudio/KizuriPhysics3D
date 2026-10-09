// KizuriPhysics - Geometry/Shape.h
// Collision shape base class and shared geometry types.
#pragma once

#include "Kizuri/Core/Math.h"
#include "Kizuri/Core/Memory.h"

namespace kizuri {

// ---------------------------------------------------------------------------
// Shape types
// ---------------------------------------------------------------------------
enum class ShapeType : u8 {
    Sphere = 0,
    Box,
    Capsule,
    Cylinder,
    ConvexHull,
    Mesh,
    Compound,
    HeightField,
    Plane,
    TaperedCapsule,
    Count
};

const char* ShapeTypeName(ShapeType type);

// ---------------------------------------------------------------------------
// Mass properties of a shape (or a set of shapes) for a given density.
// ---------------------------------------------------------------------------
struct MassProperties {
    Real  mass = 0;
    Mat3  inertia = Mat3::Zero();   // inertia tensor about the center of mass, in local space
    Vec3  centerOfMass = Vec3::Zero();

    /// Inertia tensor translated to a new reference point (parallel axis theorem).
    Mat3 InertiaAtPoint(const Vec3& point) const {
        Vec3 r = centerOfMass - point;
        Real r2 = r.LengthSq();
        return inertia + Mat3::Identity() * (mass * r2) - Mat3::OuterProduct(r, r) * mass;
    }

    MassProperties Scaled(Real scale) const {
        MassProperties result;
        result.mass = mass * scale;
        result.inertia = inertia * scale;
        result.centerOfMass = centerOfMass;
        return result;
    }

    /// Combine another shape's mass properties (already expressed in this frame).
    void Add(const MassProperties& other) {
        Real newMass = mass + other.mass;
        if (newMass <= math::kEpsilon) return;
        Vec3 newCom = (centerOfMass * mass + other.centerOfMass * other.mass) / newMass;
        inertia = InertiaAtPoint(newCom) + other.InertiaAtPoint(newCom);
        mass = newMass;
        centerOfMass = newCom;
    }
};

// ---------------------------------------------------------------------------
// Base shape
// ---------------------------------------------------------------------------
class Shape : public RefCounted {
public:
    ~Shape() override = default;

    virtual ShapeType GetType() const = 0;
    virtual const char* GetName() const = 0;

    /// Local-space axis aligned bounding box.
    virtual AABB GetLocalBounds() const = 0;
    /// World-space AABB for a given transform (rotation + translation).
    virtual AABB GetWorldBounds(const Transform& transform) const {
        return GetLocalBounds().Transformed(transform);
    }
    /// Bounding sphere radius around the shape's local origin.
    virtual Real GetBoundingRadius() const = 0;
    /// Volume in cubic units.
    virtual Real GetVolume() const = 0;

    /// Mass properties for a uniform density.
    virtual MassProperties GetMassProperties(Real density) const = 0;

    /// Support point of the convex hull in a direction (local space).
    /// Only valid when IsConvex() is true.
    virtual Vec3 GetSupport(const Vec3& direction) const { KZ_UNUSED(direction); return Vec3::Zero(); }

    /// Convex shapes support GJK/EPA and have a support function.
    virtual bool IsConvex() const { return true; }

    /// Interior point (a point strictly inside the shape), used by EPA.
    virtual Vec3 GetInteriorPoint() const { return Vec3::Zero(); }

    /// Maximum number of contact points this shape can produce with another.
    virtual u32 GetMaxContactPoints() const { return 4; }

    /// Ray cast in local space against this shape. Returns fraction in [0,1]
    /// relative to `maxFraction`, or maxFraction when there is no hit.
    virtual Real RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const {
        KZ_UNUSED(ray); KZ_UNUSED(outNormal);
        return maxFraction;
    }

    /// True when `point` (local space) is inside the shape.
    virtual bool ContainsPoint(const Vec3& point) const { KZ_UNUSED(point); return false; }

    /// Signed distance from a point to the surface (negative inside).
    /// Only meaningful for primitive shapes; returns a conservative value.
    virtual Real GetSignedDistance(const Vec3& point) const { KZ_UNUSED(point); return math::kInfinity; }

    /// Local scale applied to this shape (informational).
    Vec3 GetScale() const { return mScale; }
    void SetScale(const Vec3& s) { mScale = s; }

    /// Optional user data pointer.
    void* GetUserData() const { return mUserData; }
    void  SetUserData(void* p) { mUserData = p; }

    /// Identifier assigned when the shape is registered in a shape registry.
    u32 GetShapeId() const { return mShapeId; }
    void SetShapeId(u32 id) { mShapeId = id; }

protected:
    Shape() = default;

    Vec3 mScale = Vec3::One();
    void* mUserData = nullptr;
    u32 mShapeId = 0;
};

using ShapeRef = Ref<Shape>;

// ---------------------------------------------------------------------------
// Sphere
// ---------------------------------------------------------------------------
class SphereShape final : public Shape {
public:
    explicit SphereShape(Real radius) : mRadius(radius) { KZ_ASSERT(radius > 0); }

    ShapeType GetType() const override { return ShapeType::Sphere; }
    const char* GetName() const override { return "SphereShape"; }
    AABB GetLocalBounds() const override {
        Vec3 r(mRadius);
        return AABB(-r, r);
    }
    Real GetBoundingRadius() const override { return mRadius; }
    Real GetVolume() const override { return Real(4.0 / 3.0) * math::kPi * mRadius * mRadius * mRadius; }
    MassProperties GetMassProperties(Real density) const override;
    Vec3 GetSupport(const Vec3& direction) const override {
        return direction.NormalizedOr(Vec3::UnitY()) * mRadius;
    }
    Vec3 GetInteriorPoint() const override { return Vec3::Zero(); }
    Real RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const override;
    bool ContainsPoint(const Vec3& point) const override { return point.LengthSq() <= mRadius * mRadius; }
    Real GetSignedDistance(const Vec3& point) const override { return point.Length() - mRadius; }

    Real GetRadius() const { return mRadius; }
    void SetRadius(Real r) { KZ_ASSERT(r > 0); mRadius = r; }

private:
    Real mRadius;
};

// ---------------------------------------------------------------------------
// Box (centered on the origin, half extents)
// ---------------------------------------------------------------------------
class BoxShape final : public Shape {
public:
    explicit BoxShape(const Vec3& halfExtent) : mHalfExtent(halfExtent) {
        KZ_ASSERT(halfExtent.x > 0 && halfExtent.y > 0 && halfExtent.z > 0);
    }
    BoxShape(Real hx, Real hy, Real hz) : BoxShape(Vec3(hx, hy, hz)) {}

    ShapeType GetType() const override { return ShapeType::Box; }
    const char* GetName() const override { return "BoxShape"; }
    AABB GetLocalBounds() const override { return AABB(-mHalfExtent, mHalfExtent); }
    Real GetBoundingRadius() const override { return mHalfExtent.Length(); }
    Real GetVolume() const override { return Real(8) * mHalfExtent.x * mHalfExtent.y * mHalfExtent.z; }
    MassProperties GetMassProperties(Real density) const override;
    Vec3 GetSupport(const Vec3& direction) const override {
        return Vec3(direction.x >= 0 ? mHalfExtent.x : -mHalfExtent.x,
                    direction.y >= 0 ? mHalfExtent.y : -mHalfExtent.y,
                    direction.z >= 0 ? mHalfExtent.z : -mHalfExtent.z);
    }
    Real RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const override;
    bool ContainsPoint(const Vec3& point) const override {
        return math::Abs(point.x) <= mHalfExtent.x && math::Abs(point.y) <= mHalfExtent.y && math::Abs(point.z) <= mHalfExtent.z;
    }
    Real GetSignedDistance(const Vec3& point) const override;

    const Vec3& GetHalfExtent() const { return mHalfExtent; }
    void SetHalfExtent(const Vec3& h) { mHalfExtent = h; }

private:
    Vec3 mHalfExtent;
};

// ---------------------------------------------------------------------------
// Capsule (segment + radius, axis along Y)
// ---------------------------------------------------------------------------
class CapsuleShape final : public Shape {
public:
    CapsuleShape(Real halfHeight, Real radius) : mHalfHeight(halfHeight), mRadius(radius) {
        KZ_ASSERT(halfHeight >= 0 && radius > 0);
    }

    ShapeType GetType() const override { return ShapeType::Capsule; }
    const char* GetName() const override { return "CapsuleShape"; }
    AABB GetLocalBounds() const override {
        Vec3 e(mRadius, mHalfHeight + mRadius, mRadius);
        return AABB(-e, e);
    }
    Real GetBoundingRadius() const override { return mHalfHeight + mRadius; }
    Real GetVolume() const override {
        Real cyl = math::kPi * mRadius * mRadius * (Real(2) * mHalfHeight);
        Real sph = Real(4.0 / 3.0) * math::kPi * mRadius * mRadius * mRadius;
        return cyl + sph;
    }
    MassProperties GetMassProperties(Real density) const override;
    Vec3 GetSupport(const Vec3& direction) const override {
        Real sign = direction.y >= 0 ? mHalfHeight : -mHalfHeight;
        return Vec3(Real(0), sign, Real(0)) + direction.NormalizedOr(Vec3::UnitY()) * mRadius;
    }
    Real RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const override;

    /// Closest point on the capsule's central segment to a point.
    Vec3 GetClosestPointOnSegment(const Vec3& point) const {
        Real y = math::Clamp(point.y, -mHalfHeight, mHalfHeight);
        return Vec3(Real(0), y, Real(0));
    }

    Real GetHalfHeight() const { return mHalfHeight; }
    Real GetRadius() const { return mRadius; }
    Real GetTotalHeight() const { return Real(2) * mHalfHeight + Real(2) * mRadius; }

private:
    Real mHalfHeight;
    Real mRadius;
};

// ---------------------------------------------------------------------------
// Cylinder (axis along Y)
// ---------------------------------------------------------------------------
class CylinderShape final : public Shape {
public:
    CylinderShape(Real halfHeight, Real radius) : mHalfHeight(halfHeight), mRadius(radius) {
        KZ_ASSERT(halfHeight > 0 && radius > 0);
    }

    ShapeType GetType() const override { return ShapeType::Cylinder; }
    const char* GetName() const override { return "CylinderShape"; }
    AABB GetLocalBounds() const override {
        Vec3 e(mRadius, mHalfHeight, mRadius);
        return AABB(-e, e);
    }
    Real GetBoundingRadius() const override { return math::Sqrt(mRadius * mRadius + mHalfHeight * mHalfHeight); }
    Real GetVolume() const override { return math::kPi * mRadius * mRadius * (Real(2) * mHalfHeight); }
    MassProperties GetMassProperties(Real density) const override;
    Vec3 GetSupport(const Vec3& direction) const override {
        Vec3 d = Vec3(direction.x, Real(0), direction.z).NormalizedOr(Vec3::UnitX());
        return Vec3(d.x * mRadius, direction.y >= 0 ? mHalfHeight : -mHalfHeight, d.z * mRadius);
    }
    Real RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const override;

    Real GetHalfHeight() const { return mHalfHeight; }
    Real GetRadius() const { return mRadius; }

private:
    Real mHalfHeight;
    Real mRadius;
};

// ---------------------------------------------------------------------------
// TaperedCapsule (a cone segment with two radii + a sphere cap)
// ---------------------------------------------------------------------------
class TaperedCapsuleShape final : public Shape {
public:
    TaperedCapsuleShape(Real halfHeight, Real topRadius, Real bottomRadius)
        : mHalfHeight(halfHeight), mTopRadius(topRadius), mBottomRadius(bottomRadius) {
        KZ_ASSERT(halfHeight > 0 && topRadius > 0 && bottomRadius > 0);
    }

    ShapeType GetType() const override { return ShapeType::TaperedCapsule; }
    const char* GetName() const override { return "TaperedCapsuleShape"; }
    AABB GetLocalBounds() const override {
        Real r = math::Max(mTopRadius, mBottomRadius);
        Vec3 e(r, mHalfHeight + r, r);
        return AABB(-e, e);
    }
    Real GetBoundingRadius() const override { return mHalfHeight + math::Max(mTopRadius, mBottomRadius); }
    Real GetVolume() const override {
        Real r1 = mBottomRadius, r2 = mTopRadius, h = Real(2) * mHalfHeight;
        Real cone = math::kPi * h / Real(3) * (r1*r1 + r1*r2 + r2*r2);
        Real s1 = Real(4.0 / 3.0) * math::kPi * r1*r1*r1 * Real(0.5);
        Real s2 = Real(4.0 / 3.0) * math::kPi * r2*r2*r2 * Real(0.5);
        return cone + s1 + s2;
    }
    MassProperties GetMassProperties(Real density) const override;
    Vec3 GetSupport(const Vec3& direction) const override;

    Real GetHalfHeight() const { return mHalfHeight; }
    Real GetTopRadius() const { return mTopRadius; }
    Real GetBottomRadius() const { return mBottomRadius; }

private:
    Real mHalfHeight;
    Real mTopRadius;
    Real mBottomRadius;
};

// ---------------------------------------------------------------------------
// ConvexHull (a point cloud; the convex hull is computed at construction)
// ---------------------------------------------------------------------------
class ConvexHullShape final : public Shape {
public:
    /// Build from a point cloud. Returns null when the cloud is degenerate
    /// (fewer than 4 non-coplanar points). `maxPoints` limits the hull size.
    static Ref<ConvexHullShape> Create(ArrayView<const Vec3> points, u32 maxPoints = 256);
    /// Build directly from explicit face data.
    static Ref<ConvexHullShape> CreateFromFaces(ArrayView<const Vec3> points,
                                                 ArrayView<const u32> faceIndices,
                                                 ArrayView<const u8> faceSizes);

    ShapeType GetType() const override { return ShapeType::ConvexHull; }
    const char* GetName() const override { return "ConvexHullShape"; }
    AABB GetLocalBounds() const override { return mLocalBounds; }
    Real GetBoundingRadius() const override { return mBoundingRadius; }
    Real GetVolume() const override { return mVolume; }
    MassProperties GetMassProperties(Real density) const override;
    Vec3 GetSupport(const Vec3& direction) const override;
    Vec3 GetInteriorPoint() const override { return mInteriorPoint; }
    u32 GetMaxContactPoints() const override { return 8; }

    ArrayView<const Vec3> GetPoints() const { return { mPoints.Data(), mPoints.Size() }; }
    ArrayView<const u32>  GetFaceIndices() const { return { mFaceIndices.Data(), mFaceIndices.Size() }; }
    ArrayView<const u8>   GetFaceSizes() const { return { mFaceSizes.Data(), mFaceSizes.Size() }; }
    u32 GetNumFaces() const { return u32(mFaceSizes.Size()); }
    u32 GetNumVertices() const { return u32(mPoints.Size()); }
    const Vec3& GetCentroid() const { return mCentroid; }

private:
    ConvexHullShape() = default;
    void ComputeDerivedData();

    Vector<Vec3> mPoints;
    Vector<u32>  mFaceIndices;
    Vector<u8>   mFaceSizes;
    Vector<Vec3> mFaceNormals;
    AABB mLocalBounds;
    Vec3 mCentroid = Vec3::Zero();
    Vec3 mInteriorPoint = Vec3::Zero();
    Real mBoundingRadius = 0;
    Real mVolume = 0;
};

// ---------------------------------------------------------------------------
// Mesh (triangle soup, possibly non-convex; one-sided or double-sided)
// ---------------------------------------------------------------------------
class MeshShape final : public Shape {
public:
    MeshShape(ArrayView<const Vec3> vertices, ArrayView<const u32> indices, bool doubleSided = false);

    ShapeType GetType() const override { return ShapeType::Mesh; }
    const char* GetName() const override { return "MeshShape"; }
    AABB GetLocalBounds() const override { return mLocalBounds; }
    Real GetBoundingRadius() const override { return mBoundingRadius; }
    Real GetVolume() const override { return 0; } // open meshes have no volume
    MassProperties GetMassProperties(Real density) const override;
    bool IsConvex() const override { return false; }
    Real RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const override;

    u32 GetNumTriangles() const { return u32(mIndices.Size()) / 3; }
    const Vec3& GetVertex(u32 i) const { return mVertices[i]; }
    void GetTriangle(u32 tri, Vec3& a, Vec3& b, Vec3& c) const {
        a = mVertices[mIndices[tri * 3 + 0]];
        b = mVertices[mIndices[tri * 3 + 1]];
        c = mVertices[mIndices[tri * 3 + 2]];
    }
    bool IsDoubleSided() const { return mDoubleSided; }

    /// Accelerated structure (BVH) used for queries and narrowphase.
    struct BVH;

    ~MeshShape() override;

private:
    Vector<Vec3> mVertices;
    Vector<u32>  mIndices;
    Vector<u32>  mTriNormals; // packed face normals (unused placeholder for SIMD)
    AABB mLocalBounds;
    Real mBoundingRadius = 0;
    bool mDoubleSided = false;
    BVH* mBVH = nullptr;
};

// ---------------------------------------------------------------------------
// Compound (a set of child shapes with local transforms)
// ---------------------------------------------------------------------------
class CompoundShape final : public Shape {
public:
    struct Child {
        Transform transform;
        ShapeRef  shape;
        u32       userIndex = 0;
    };

    CompoundShape() = default;
    explicit CompoundShape(ArrayView<const Child> children);

    void AddChild(const Transform& transform, const ShapeRef& shape, u32 userIndex = 0);
    void Finalize();

    ShapeType GetType() const override { return ShapeType::Compound; }
    const char* GetName() const override { return "CompoundShape"; }
    AABB GetLocalBounds() const override { return mLocalBounds; }
    Real GetBoundingRadius() const override { return mBoundingRadius; }
    Real GetVolume() const override;
    MassProperties GetMassProperties(Real density) const override;
    bool IsConvex() const override { return false; }
    Real RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const override;
    bool ContainsPoint(const Vec3& point) const override;

    u32 GetNumChildren() const { return u32(mChildren.Size()); }
    const Child& GetChild(u32 i) const { return mChildren[i]; }

private:
    Vector<Child> mChildren;
    AABB mLocalBounds;
    Real mBoundingRadius = 0;
};

// ---------------------------------------------------------------------------
// HeightField (regular grid of heights on the XZ plane)
// ---------------------------------------------------------------------------
class HeightFieldShape final : public Shape {
public:
    HeightFieldShape(u32 sampleCountX, u32 sampleCountZ, Real scaleX, Real scaleZ,
                     ArrayView<const Real> heights, bool doubleSided = true);

    ShapeType GetType() const override { return ShapeType::HeightField; }
    const char* GetName() const override { return "HeightFieldShape"; }
    AABB GetLocalBounds() const override { return mLocalBounds; }
    Real GetBoundingRadius() const override { return mBoundingRadius; }
    Real GetVolume() const override { return 0; }
    MassProperties GetMassProperties(Real density) const override;
    bool IsConvex() const override { return false; }
    Real RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const override;

    u32 GetSampleCountX() const { return mSampleCountX; }
    u32 GetSampleCountZ() const { return mSampleCountZ; }
    Real GetScaleX() const { return mScaleX; }
    Real GetScaleZ() const { return mScaleZ; }
    Real GetHeight(u32 x, u32 z) const { return mHeights[z * mSampleCountX + x]; }
    /// Bilinearly interpolated height at a grid position in [-1,1]^2 style coords.
    Real GetInterpolatedHeight(Real localX, Real localZ) const;
    Vec3 GetNormal(u32 x, u32 z) const;

    /// Iterate the triangles overlapping an AABB (local space).
    template <typename Fn>
    void ForEachTriangleInBounds(const AABB& bounds, Fn&& fn) const {
        Real cellX = Real(2) * mScaleX / Real(mSampleCountX - 1);
        Real cellZ = Real(2) * mScaleZ / Real(mSampleCountZ - 1);
        i32 x0 = math::Max(0, i32((bounds.min.x + mScaleX) / cellX) - 1);
        i32 z0 = math::Max(0, i32((bounds.min.z + mScaleZ) / cellZ) - 1);
        i32 x1 = math::Min(i32(mSampleCountX) - 2, i32((bounds.max.x + mScaleX) / cellX) + 1);
        i32 z1 = math::Min(i32(mSampleCountZ) - 2, i32((bounds.max.z + mScaleZ) / cellZ) + 1);
        for (i32 z = z0; z <= z1; ++z) {
            for (i32 x = x0; x <= x1; ++x) {
                Real x0w = -mScaleX + Real(x) * cellX;
                Real z0w = -mScaleZ + Real(z) * cellZ;
                Real x1w = x0w + cellX;
                Real z1w = z0w + cellZ;
                Real h00 = GetHeight(u32(x), u32(z));
                Real h10 = GetHeight(u32(x + 1), u32(z));
                Real h01 = GetHeight(u32(x), u32(z + 1));
                Real h11 = GetHeight(u32(x + 1), u32(z + 1));
                Vec3 v00(x0w, h00, z0w), v10(x1w, h10, z0w);
                Vec3 v01(x0w, h01, z1w), v11(x1w, h11, z1w);
                fn(v00, v10, v11);
                fn(v00, v11, v01);
            }
        }
    }

private:
    u32 mSampleCountX;
    u32 mSampleCountZ;
    Real mScaleX;
    Real mScaleZ;
    Vector<Real> mHeights;
    AABB mLocalBounds;
    Real mBoundingRadius = 0;
    bool mDoubleSided;
};

// ---------------------------------------------------------------------------
// Plane (infinite half-space; not convex in the bounded sense, handled specially)
// ---------------------------------------------------------------------------
class PlaneShape final : public Shape {
public:
    explicit PlaneShape(const Plane& plane = Plane()) : mPlane(plane) {}

    ShapeType GetType() const override { return ShapeType::Plane; }
    const char* GetName() const override { return "PlaneShape"; }
    AABB GetLocalBounds() const override {
        // A large finite box used for broadphase culling.
        const Real big = Real(1.0e6);
        Vec3 n = mPlane.normal;
        Vec3 extent = Vec3(big) - n.Abs() * big; // thin along the normal
        Vec3 center = n * mPlane.distance;
        return AABB(center - extent, center + extent);
    }
    Real GetBoundingRadius() const override { return 0; }
    Real GetVolume() const override { return 0; }
    MassProperties GetMassProperties(Real) const override { return MassProperties{}; }
    bool IsConvex() const override { return true; }
    Vec3 GetSupport(const Vec3& direction) const override {
        // Only the normal component is meaningful.
        return mPlane.normal * mPlane.distance + direction * Real(1.0e6);
    }
    Real GetSignedDistance(const Vec3& point) const override { return mPlane.SignedDistance(point); }
    Real RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const override {
        Real denom = mPlane.normal.Dot(ray.direction);
        if (math::Abs(denom) < math::kEpsilon) return maxFraction;
        Real t = (mPlane.distance - mPlane.normal.Dot(ray.origin)) / denom;
        if (t < Real(0) || t > maxFraction) return maxFraction;
        outNormal = mPlane.normal;
        return t;
    }
    bool ContainsPoint(const Vec3& point) const override {
        return mPlane.SignedDistance(point) <= Real(0);
    }

    const Plane& GetPlane() const { return mPlane; }
    void SetPlane(const Plane& p) { mPlane = p; }

private:
    Plane mPlane;
};

} // namespace kizuri
