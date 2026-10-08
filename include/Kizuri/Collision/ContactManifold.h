// KizuriPhysics - Collision/ContactManifold.h
// Contact points and manifold produced by the narrow phase.
#pragma once

#include "Kizuri/Geometry/Shape.h"
#include "Kizuri/Collision/GJK.h"

namespace kizuri {

/// Maximum contact points stored per body pair.
inline constexpr u32 kMaxManifoldPoints = 8;

// ---------------------------------------------------------------------------
// A single contact point.
// ---------------------------------------------------------------------------
struct ContactPoint {
    /// Contact position on shape A, world space.
    Vec3 pointOnA = Vec3::Zero();
    /// Contact position on shape B, world space.
    Vec3 pointOnB = Vec3::Zero();
    /// Contact normal, unit, pointing from B towards A.
    Vec3 normal = Vec3::UnitY();
    /// Signed separation along the normal (negative when penetrating).
    Real separation = 0;
    /// Feature identifier used to match points across frames for warm starting.
    u32 featureId = 0;
};

// ---------------------------------------------------------------------------
// Contact manifold: up to kMaxManifoldPoints points sharing a normal.
// ---------------------------------------------------------------------------
struct Manifold {
    ContactPoint points[kMaxManifoldPoints];
    u32 numPoints = 0;

    void Clear() { numPoints = 0; }

    void AddPoint(const ContactPoint& p) {
        if (numPoints < kMaxManifoldPoints) points[numPoints++] = p;
    }

    bool Empty() const { return numPoints == 0; }
};

// ---------------------------------------------------------------------------
// Narrow phase settings.
// ---------------------------------------------------------------------------
struct CollideSettings {
    /// Distance below which contacts are generated (usually a small skin).
    Real collisionTolerance = Real(1.0e-4);
    /// Points whose separation exceeds this are discarded.
    Real maxSeparation = Real(0.02);
    /// Relative tolerance for feature matching.
    Real featureTolerance = Real(1.0e-3);
    /// When true, generate contact points even when only nearly touching.
    bool collectTouchPoints = true;
};

// ---------------------------------------------------------------------------
// Result of a shape cast / collide query.
// ---------------------------------------------------------------------------
struct CollideShapeResult {
    Vec3 pointOnA = Vec3::Zero();
    Vec3 pointOnB = Vec3::Zero();
    Vec3 normal = Vec3::UnitY(); // from B to A
    Real penetrationDepth = 0;
    u32 shapeAId = 0;
    u32 shapeBId = 0;
};

// ---------------------------------------------------------------------------
// Narrow phase entry point.
// ---------------------------------------------------------------------------
class NarrowPhase {
public:
    /// Compute the contact manifold between two shapes.
    /// Returns the number of contact points written to `outManifold`.
    static u32 Collide(const Shape& shapeA, const Transform& transformA,
                       const Shape& shapeB, const Transform& transformB,
                       const CollideSettings& settings, Manifold& outManifold);

    /// Boolean overlap test (faster than a full manifold).
    static bool Overlap(const Shape& shapeA, const Transform& transformA,
                        const Shape& shapeB, const Transform& transformB);

    /// Closest points / distance between two convex shapes.
    static GjkResult ClosestPoints(const Shape& shapeA, const Transform& transformA,
                                   const Shape& shapeB, const Transform& transformB);

    /// Cast a convex shape along a direction against another shape.
    static bool CastShape(const Shape& shapeA, const Transform& transformA, const Vec3& direction,
                          Real maxDistance, const Shape& shapeB, const Transform& transformB,
                          Real& outFraction, Vec3& outNormal);
};

} // namespace kizuri
