// KizuriPhysics - Collision/GJK.h
// Gilbert-Johnson-Keerthi distance and intersection algorithm.
#pragma once

#include "Kizuri/Geometry/Shape.h"

namespace kizuri {

// ---------------------------------------------------------------------------
// Support mapping for the Minkowski difference A - B.
// ---------------------------------------------------------------------------
struct SupportPoint {
    Vec3 pointA; // support on shape A (world space)
    Vec3 pointB; // support on shape B (world space)
    Vec3 point;  // pointA - pointB

    KZ_FORCEINLINE void Set(const Vec3& a, const Vec3& b) {
        pointA = a;
        pointB = b;
        point = a - b;
    }
};

/// A support mapping pair with cached world transforms.
class SupportMap {
public:
    SupportMap(const Shape& shapeA, const Transform& transformA,
               const Shape& shapeB, const Transform& transformB)
        : mShapeA(shapeA), mShapeB(shapeB), mTransformA(transformA), mTransformB(transformB) {}

    KZ_FORCEINLINE SupportPoint GetSupport(const Vec3& direction) const {
        SupportPoint sp;
        // Support of A in direction d, support of B in direction -d.
        Vec3 localA = mTransformA.rotation.InverseRotate(direction);
        Vec3 localB = mTransformB.rotation.InverseRotate(-direction);
        Vec3 a = mTransformA * mShapeA.GetSupport(localA);
        Vec3 b = mTransformB * mShapeB.GetSupport(localB);
        sp.Set(a, b);
        return sp;
    }

    const Shape& GetShapeA() const { return mShapeA; }
    const Shape& GetShapeB() const { return mShapeB; }

private:
    const Shape& mShapeA;
    const Shape& mShapeB;
    Transform mTransformA;
    Transform mTransformB;
};

// ---------------------------------------------------------------------------
// GJK result
// ---------------------------------------------------------------------------
struct GjkResult {
    bool intersect = false;    // true when the shapes overlap
    bool valid = true;         // false when the algorithm failed to converge
    Real distance = 0;         // distance between the shapes (0 when intersecting)
    Vec3 pointA = Vec3::Zero(); // closest point on A (world)
    Vec3 pointB = Vec3::Zero(); // closest point on B (world)
    Vec3 normal = Vec3::Zero(); // unit direction from B to A (separated case)
    u32 iterations = 0;
    u32 simplexSize = 0;       // simplex size at termination (used by EPA)
    SupportPoint simplex[4];   // terminal simplex (for EPA warm start)
};

class GJK {
public:
    /// Compute the closest points / intersection between two convex shapes.
    /// `tolerance` controls convergence (defaults to 1e-4 * scale).
    static GjkResult GetClosestPoints(const SupportMap& map, Real tolerance = Real(1.0e-4),
                                      u32 maxIterations = 32);

    /// Fast boolean intersection test (no closest points).
    static bool Intersects(const SupportMap& map, u32 maxIterations = 32);
};

} // namespace kizuri
