// KizuriPhysics - Collision/CollideShape.cpp
// Narrow phase: contact manifold generation for all shape pairs.
#include "Kizuri/Collision/ContactManifold.h"
#include "Kizuri/Collision/EPA.h"

namespace kizuri {

namespace {

// ---------------------------------------------------------------------------
// Manifold helpers
// ---------------------------------------------------------------------------
void SwapManifold(Manifold& m) {
    for (u32 i = 0; i < m.numPoints; ++i) {
        ContactPoint& p = m.points[i];
        Vec3 tmp = p.pointOnA;
        p.pointOnA = p.pointOnB;
        p.pointOnB = tmp;
        p.normal = -p.normal;
    }
}

// ---------------------------------------------------------------------------
// Polygon / face extraction for polyhedral shapes
// ---------------------------------------------------------------------------
constexpr u32 kMaxFaceVerts = 16;

/// Extract the face of a polyhedral shape most aligned with a world direction.
/// Returns false for shapes without polygonal faces.
bool GetShapeFace(const Shape& shape, const Transform& transform, const Vec3& worldDir,
                  Vec3 outVerts[kMaxFaceVerts], u32& outCount, Vec3& outNormalWorld) {
    switch (shape.GetType()) {
        case ShapeType::Box: {
            const BoxShape& box = static_cast<const BoxShape&>(shape);
            Vec3 localDir = transform.rotation.InverseRotate(worldDir);
            int axis = localDir.Abs().MaxComponentIndex();
            Real sign = localDir[axis] >= Real(0) ? Real(1) : Real(-1);
            Vec3 he = box.GetHalfExtent();
            Vec3 n = Vec3::Zero();
            n[axis] = sign;
            // Build the 4 corners of that face.
            Vec3 c = Vec3(he.x * n.x, he.y * n.y, he.z * n.z);
            int a0 = (axis + 1) % 3;
            int a1 = (axis + 2) % 3;
            Real s0 = he[a0], s1 = he[a1];
            Vec3 corners[4];
            corners[0] = c;
            corners[0][a0] = s0; corners[0][a1] = s1;
            corners[1] = c;
            corners[1][a0] = -s0; corners[1][a1] = s1;
            corners[2] = c;
            corners[2][a0] = -s0; corners[2][a1] = -s1;
            corners[3] = c;
            corners[3][a0] = s0; corners[3][a1] = -s1;
            for (u32 i = 0; i < 4; ++i) outVerts[i] = transform * corners[i];
            outCount = 4;
            outNormalWorld = transform.rotation.Rotate(n);
            return true;
        }
        case ShapeType::ConvexHull: {
            const ConvexHullShape& hull = static_cast<const ConvexHullShape&>(shape);
            Vec3 localDir = transform.rotation.InverseRotate(worldDir);
            ArrayView<const Vec3> points = hull.GetPoints();
            ArrayView<const u32> indices = hull.GetFaceIndices();
            ArrayView<const u8> sizes = hull.GetFaceSizes();
            Real bestDot = -math::kInfinity;
            u32 bestFace = 0xFFFFFFFFu;
            u32 offset = 0;
            for (u32 fi = 0; fi < sizes.Size(); ++fi) {
                u32 n = sizes[fi];
                Vec3 a = points[indices[offset]];
                Vec3 b = points[indices[offset + 1]];
                Vec3 c = points[indices[offset + 2]];
                Vec3 fn = (b - a).Cross(c - a).NormalizedOr(Vec3::UnitY());
                Real d = fn.Dot(localDir);
                if (d > bestDot) { bestDot = d; bestFace = fi; }
                offset += n;
            }
            if (bestFace == 0xFFFFFFFFu) return false;
            offset = 0;
            for (u32 fi = 0; fi < bestFace; ++fi) offset += sizes[fi];
            u32 n = sizes[bestFace];
            if (n > kMaxFaceVerts) n = kMaxFaceVerts;
            for (u32 i = 0; i < n; ++i) {
                outVerts[i] = transform * points[indices[offset + i]];
            }
            outCount = n;
            Vec3 a = points[indices[offset]];
            Vec3 b = points[indices[offset + 1]];
            Vec3 c = points[indices[offset + 2]];
            Vec3 fn = (b - a).Cross(c - a).NormalizedOr(Vec3::UnitY());
            outNormalWorld = transform.rotation.Rotate(fn);
            return true;
        }
        default:
            return false;
    }
}

// ---------------------------------------------------------------------------
// Sutherland-Hodgman polygon clipping against a plane (keep front side).
// ---------------------------------------------------------------------------
struct ClipPolygon {
    Vec3 verts[kMaxFaceVerts * 2];
    u32 count = 0;

    void Clear() { count = 0; }
    void Add(const Vec3& v) { if (count < kMaxFaceVerts * 2) verts[count++] = v; }
};

void ClipAgainstPlane(const ClipPolygon& in, const Vec3& planeNormal, Real planeDist,
                      ClipPolygon& out) {
    out.Clear();
    if (in.count == 0) return;
    for (u32 i = 0; i < in.count; ++i) {
        const Vec3& cur = in.verts[i];
        const Vec3& next = in.verts[(i + 1) % in.count];
        Real dCur = planeNormal.Dot(cur) - planeDist;
        Real dNext = planeNormal.Dot(next) - planeDist;
        if (dCur <= Real(0)) out.Add(cur);
        if ((dCur <= Real(0)) != (dNext <= Real(0))) {
            Real t = dCur / (dCur - dNext);
            out.Add(Lerp(cur, next, t));
        }
    }
}

// ---------------------------------------------------------------------------
// Face-clipping manifold between two polyhedral shapes.
// ---------------------------------------------------------------------------
u32 CollidePolyhedral(const Shape& shapeA, const Transform& tA,
                      const Shape& shapeB, const Transform& tB,
                      const CollideSettings& settings, Manifold& manifold, const Vec3& normal) {
    // Determine the reference shape: the one whose face is most perpendicular
    // to the contact normal.
    Vec3 refFaceA[kMaxFaceVerts], refFaceB[kMaxFaceVerts];
    Vec3 normA, normB;
    u32 countA = 0, countB = 0;
    // Face of A facing B (outward normal ~ -normal).
    bool haveA = GetShapeFace(shapeA, tA, -normal, refFaceA, countA, normA);
    // Face of B facing A (outward normal ~ +normal).
    bool haveB = GetShapeFace(shapeB, tB, normal, refFaceB, countB, normB);
    if (!haveA || !haveB) return 0;

    Real alignA = normA.Dot(-normal);
    Real alignB = normB.Dot(normal);

    Vec3* refVerts; u32 refCount;
    Vec3* incVerts; u32 incCount;
    bool refIsA;

    if (alignA >= alignB) {
        refVerts = refFaceA; refCount = countA;
        incVerts = refFaceB; incCount = countB;
        refIsA = true;
    } else {
        refVerts = refFaceB; refCount = countB;
        incVerts = refFaceA; incCount = countA;
        refIsA = false;
    }

    // Reference face plane (outward normal = refNormal).
    Vec3 refNormal = (refIsA ? normA : normB);
    Vec3 refPoint = refVerts[0];
    Real refDist = refNormal.Dot(refPoint);

    // Clip the incident polygon against the reference face's side planes.
    ClipPolygon poly;
    poly.Clear();
    for (u32 i = 0; i < incCount; ++i) poly.Add(incVerts[i]);

    // Face centroid, used to orient the side planes outward. This makes the
    // clipping robust to the (axis-dependent) vertex winding of the face.
    Vec3 refCentroid = Vec3::Zero();
    for (u32 i = 0; i < refCount; ++i) refCentroid += refVerts[i];
    refCentroid /= Real(refCount);

    ClipPolygon tmp;
    for (u32 i = 0; i < refCount; ++i) {
        Vec3 v0 = refVerts[i];
        Vec3 v1 = refVerts[(i + 1) % refCount];
        Vec3 edge = v1 - v0;
        Vec3 sideNormal = edge.Cross(refNormal);
        Real len = sideNormal.Length();
        if (len < Real(1.0e-9)) continue;
        sideNormal /= len;
        if (sideNormal.Dot(v0 - refCentroid) < Real(0)) sideNormal = -sideNormal;
        Real sideDist = sideNormal.Dot(v0);
        ClipAgainstPlane(poly, sideNormal, sideDist, tmp);
        poly = tmp;
        if (poly.count == 0) break;
    }

    u32 numContacts = 0;
    for (u32 i = 0; i < poly.count && numContacts < kMaxManifoldPoints; ++i) {
        const Vec3& p = poly.verts[i];
        Real sep = refNormal.Dot(p) - refDist; // negative when penetrating
        if (sep > settings.maxSeparation) continue;

        ContactPoint cp;
        // Project the incident point onto the reference plane.
        Vec3 projected = p - refNormal * sep;
        if (refIsA) {
            cp.pointOnA = projected;
            cp.pointOnB = p;
        } else {
            cp.pointOnA = p;
            cp.pointOnB = projected;
        }
        cp.normal = normal;
        cp.separation = sep;
        cp.featureId = i;
        manifold.points[numContacts++] = cp;
    }

    // Reduce to at most 4 points spread over the contact area.
    if (numContacts > 4) {
        // Keep the deepest and the extremes; simple reduction by keeping every
        // other point after sorting by depth.
        ContactPoint kept[4];
        for (u32 i = 0; i < 4; ++i) kept[i] = manifold.points[i * (numContacts - 1) / 3];
        for (u32 i = 0; i < 4; ++i) manifold.points[i] = kept[i];
        numContacts = 4;
    }

    manifold.numPoints = numContacts;
    return numContacts;
}

// ---------------------------------------------------------------------------
// Generic convex-convex via GJK/EPA, single or few contact points.
// ---------------------------------------------------------------------------
u32 CollideConvexGeneric(const Shape& shapeA, const Transform& tA,
                         const Shape& shapeB, const Transform& tB,
                         const CollideSettings& settings, Manifold& manifold) {
    SupportMap map(shapeA, tA, shapeB, tB);
    GjkResult gjk = GJK::GetClosestPoints(map, settings.collisionTolerance);

    if (!gjk.intersect) {
        if (gjk.distance > settings.maxSeparation) return 0;
        // Near-touching: for polyhedral pairs still generate the full face
        // manifold. A single point here is what makes resting stacks lose
        // their support and start to "breathe".
        const bool pA = (shapeA.GetType() == ShapeType::Box || shapeA.GetType() == ShapeType::ConvexHull);
        const bool pB = (shapeB.GetType() == ShapeType::Box || shapeB.GetType() == ShapeType::ConvexHull);
        if (pA && pB && gjk.normal.LengthSq() > Real(0)) {
            if (CollidePolyhedral(shapeA, tA, shapeB, tB, settings, manifold, gjk.normal) > 0)
                return manifold.numPoints;
        }
        // Otherwise one contact point.
        ContactPoint cp;
        cp.pointOnA = gjk.pointA;
        cp.pointOnB = gjk.pointB;
        cp.normal = gjk.normal;
        cp.separation = gjk.distance;
        manifold.AddPoint(cp);
        return manifold.numPoints;
    }

    // Overlapping: polyhedral shapes get face clipping, others a single point.
    bool polyA = (shapeA.GetType() == ShapeType::Box || shapeA.GetType() == ShapeType::ConvexHull);
    bool polyB = (shapeB.GetType() == ShapeType::Box || shapeB.GetType() == ShapeType::ConvexHull);
    if (polyA && polyB) {
        EpaResult epa = EPA::Compute(map, gjk, settings.collisionTolerance);
        if (epa.valid) {
            if (CollidePolyhedral(shapeA, tA, shapeB, tB, settings, manifold, epa.normal) > 0) {
                return manifold.numPoints;
            }
            // Fall back to the EPA witness point.
            ContactPoint cp;
            cp.pointOnA = epa.pointA;
            cp.pointOnB = epa.pointB;
            cp.normal = epa.normal;
            cp.separation = -epa.depth;
            manifold.AddPoint(cp);
            return manifold.numPoints;
        }
    }

    EpaResult epa = EPA::Compute(map, gjk, settings.collisionTolerance);
    if (epa.valid) {
        ContactPoint cp;
        cp.pointOnA = epa.pointA;
        cp.pointOnB = epa.pointB;
        cp.normal = epa.normal;
        cp.separation = -epa.depth;
        manifold.AddPoint(cp);
        return manifold.numPoints;
    }

    // Last resort: use the GJK witness with zero depth.
    ContactPoint cp;
    cp.pointOnA = gjk.pointA;
    cp.pointOnB = gjk.pointB;
    cp.normal = gjk.normal.LengthSq() > Real(0) ? gjk.normal : Vec3::UnitY();
    cp.separation = Real(0);
    manifold.AddPoint(cp);
    return manifold.numPoints;
}

// ---------------------------------------------------------------------------
// Sphere vs convex
// ---------------------------------------------------------------------------
u32 CollideSphereConvex(const SphereShape& sphere, const Transform& tSphere,
                        const Shape& shapeB, const Transform& tB,
                        const CollideSettings& settings, Manifold& manifold) {
    // Treat the sphere as a point and find the closest point on B.
    SphereShape point(Real(1.0e-4)); // tiny sphere for GJK
    SupportMap map(point, tSphere, shapeB, tB);
    GjkResult gjk = GJK::GetClosestPoints(map, settings.collisionTolerance);

    Real radius = sphere.GetRadius();
    Vec3 sphereCenter = tSphere.translation;

    // Closest point on B (approximately): gjk.pointB when separated.
    Vec3 closestOnB = gjk.pointB;
    if (gjk.intersect) {
        // Sphere center inside B: push out along the surface normal.
        EpaResult epa = EPA::Compute(map, gjk, settings.collisionTolerance);
        if (epa.valid) {
            ContactPoint cp;
            cp.normal = -epa.normal; // from B to sphere center
            cp.pointOnB = epa.pointB;
            cp.pointOnA = sphereCenter + cp.normal * radius;
            cp.separation = -(epa.depth + radius);
            manifold.AddPoint(cp);
            return manifold.numPoints;
        }
        return 0;
    }

    Real dist = gjk.distance;
    Real sep = dist - radius;
    if (sep > settings.maxSeparation) return 0;

    Vec3 normal = (dist > math::kEpsilon) ? (sphereCenter - closestOnB) / dist : Vec3::UnitY();
    ContactPoint cp;
    cp.normal = normal;
    cp.pointOnA = sphereCenter - normal * radius;
    cp.pointOnB = closestOnB;
    cp.separation = sep;
    manifold.AddPoint(cp);
    return manifold.numPoints;
}

// ---------------------------------------------------------------------------
// Sphere vs sphere
// ---------------------------------------------------------------------------
u32 CollideSphereSphere(const SphereShape& a, const Transform& ta,
                        const SphereShape& b, const Transform& tb,
                        const CollideSettings& settings, Manifold& manifold) {
    Vec3 d = ta.translation - tb.translation;
    Real dist = d.Length();
    Real sep = dist - (a.GetRadius() + b.GetRadius());
    if (sep > settings.maxSeparation) return 0;

    Vec3 normal = dist > math::kEpsilon ? d / dist : Vec3::UnitY();
    ContactPoint cp;
    cp.normal = normal;
    cp.pointOnA = ta.translation - normal * a.GetRadius();
    cp.pointOnB = tb.translation + normal * b.GetRadius();
    cp.separation = sep;
    manifold.AddPoint(cp);
    return manifold.numPoints;
}

// ---------------------------------------------------------------------------
// Plane vs convex
// ---------------------------------------------------------------------------
u32 CollidePlaneConvex(const PlaneShape& plane, const Transform& tPlane,
                       const Shape& shapeB, const Transform& tB,
                       const CollideSettings& settings, Manifold& manifold) {
    // Plane in world space.
    Vec3 planeNormal = tPlane.rotation.Rotate(plane.GetPlane().normal);
    Real planeDist = plane.GetPlane().distance + planeNormal.Dot(tPlane.translation);

    // For polyhedral shapes, collect all vertices below the plane.
    if (shapeB.GetType() == ShapeType::Box || shapeB.GetType() == ShapeType::ConvexHull) {
        Vec3 verts[kMaxFaceVerts];
        u32 n = 0;
        if (shapeB.GetType() == ShapeType::Box) {
            const BoxShape& box = static_cast<const BoxShape&>(shapeB);
            Vec3 he = box.GetHalfExtent();
            for (int i = 0; i < 8; ++i) {
                Vec3 c((i & 1) ? he.x : -he.x, (i & 2) ? he.y : -he.y, (i & 4) ? he.z : -he.z);
                verts[n++] = tB * c;
            }
        } else {
            const ConvexHullShape& hull = static_cast<const ConvexHullShape&>(shapeB);
            ArrayView<const Vec3> pts = hull.GetPoints();
            for (usize i = 0; i < pts.Size() && n < kMaxFaceVerts; ++i) verts[n++] = tB * pts[i];
        }

        u32 count = 0;
        for (u32 i = 0; i < n && count < kMaxManifoldPoints; ++i) {
            Real d = planeNormal.Dot(verts[i]) - planeDist;
            if (d > settings.maxSeparation) continue;
            ContactPoint cp;
            cp.normal = planeNormal; // from plane (B) to shape (A)
            cp.pointOnA = verts[i];
            cp.pointOnB = verts[i] - planeNormal * d;
            cp.separation = d;
            manifold.points[count++] = cp;
        }
        // Reduce to 4.
        if (count > 4) {
            ContactPoint kept[4];
            for (u32 i = 0; i < 4; ++i) kept[i] = manifold.points[i * (count - 1) / 3];
            for (u32 i = 0; i < 4; ++i) manifold.points[i] = kept[i];
            count = 4;
        }
        manifold.numPoints = count;
        return count;
    }

    // Non-polyhedral convex: deepest support point.
    Vec3 localDir = tB.rotation.InverseRotate(-planeNormal);
    Vec3 deepestLocal = shapeB.GetSupport(localDir);
    Vec3 deepest = tB * deepestLocal;
    Real d = planeNormal.Dot(deepest) - planeDist;
    if (d > settings.maxSeparation) return 0;

    ContactPoint cp;
    cp.normal = planeNormal;
    cp.pointOnA = deepest;
    cp.pointOnB = deepest - planeNormal * d;
    cp.separation = d;
    manifold.AddPoint(cp);
    return manifold.numPoints;
}

// ---------------------------------------------------------------------------
// Convex vs triangle (used by mesh and heightfield collision).
// ---------------------------------------------------------------------------
u32 CollideConvexTriangle(const Shape& convex, const Transform& tConvex,
                          const Vec3& a, const Vec3& b, const Vec3& c,
                          const CollideSettings& settings, Manifold& manifold) {
    // Triangle plane.
    Vec3 triNormal = (b - a).Cross(c - a);
    Real area2 = triNormal.Length();
    if (area2 < Real(1.0e-12)) return 0;
    triNormal /= area2;
    Real triDist = triNormal.Dot(a);

    // Deepest point of the convex shape below the plane (support in -triNormal).
    Vec3 localDir = tConvex.rotation.InverseRotate(-triNormal);
    Vec3 deepest = tConvex * convex.GetSupport(localDir);
    Real d = triNormal.Dot(deepest) - triDist;
    if (d > settings.maxSeparation) return 0;

    // Contact normal points from the triangle (B) to the convex (A).
    ContactPoint cp;
    cp.normal = -triNormal;
    cp.pointOnA = deepest;
    cp.pointOnB = deepest + triNormal * d;
    cp.separation = d;

    // Verify the projected point lies within the triangle.
    Vec3 projected = deepest - triNormal * d;
    Vec3 v0 = b - a, v1 = c - a, v2 = projected - a;
    Real d00 = v0.Dot(v0), d01 = v0.Dot(v1), d11 = v1.Dot(v1);
    Real d20 = v2.Dot(v0), d21 = v2.Dot(v1);
    Real denom = d00 * d11 - d01 * d01;
    if (math::Abs(denom) < Real(1.0e-18)) return 0;
    Real v = (d11 * d20 - d01 * d21) / denom;
    Real w = (d00 * d21 - d01 * d20) / denom;
    Real u = Real(1) - v - w;
    const Real margin = Real(0.05);
    if (u < -margin || v < -margin || w < -margin) return 0;

    manifold.AddPoint(cp);
    return manifold.numPoints;
}

} // namespace

// ---------------------------------------------------------------------------
// Public dispatch
// ---------------------------------------------------------------------------
u32 NarrowPhase::Collide(const Shape& shapeA, const Transform& transformA,
                         const Shape& shapeB, const Transform& transformB,
                         const CollideSettings& settings, Manifold& outManifold) {
    outManifold.Clear();
    const ShapeType typeA = shapeA.GetType();
    const ShapeType typeB = shapeB.GetType();

    // --- Compound recursion -------------------------------------------------
    if (typeA == ShapeType::Compound) {
        const CompoundShape& compound = static_cast<const CompoundShape&>(shapeA);
        for (u32 i = 0; i < compound.GetNumChildren(); ++i) {
            const CompoundShape::Child& child = compound.GetChild(i);
            Manifold sub;
            Transform childWorld = transformA * child.transform;
            Collide(*child.shape, childWorld, shapeB, transformB, settings, sub);
            for (u32 k = 0; k < sub.numPoints && outManifold.numPoints < kMaxManifoldPoints; ++k) {
                outManifold.points[outManifold.numPoints++] = sub.points[k];
            }
        }
        return outManifold.numPoints;
    }
    if (typeB == ShapeType::Compound) {
        Manifold sub;
        u32 total = Collide(shapeB, transformB, shapeA, transformA, settings, sub);
        if (total > 0) { SwapManifold(sub); outManifold = sub; }
        return outManifold.numPoints;
    }

    // --- Plane --------------------------------------------------------------
    // CollidePlaneConvex always emits a manifold with A = convex, B = plane
    // (normal points from the plane towards the convex shape).
    if (typeA == ShapeType::Plane) {
        u32 n = CollidePlaneConvex(static_cast<const PlaneShape&>(shapeA), transformA,
                                   shapeB, transformB, settings, outManifold);
        if (n > 0) SwapManifold(outManifold); // pair is A=plane, B=convex
        return n;
    }
    if (typeB == ShapeType::Plane) {
        return CollidePlaneConvex(static_cast<const PlaneShape&>(shapeB), transformB,
                                  shapeA, transformA, settings, outManifold);
    }

    // --- Mesh / HeightField vs convex --------------------------------------
    auto collideMesh = [&](const Shape& mesh, const Transform& tMesh,
                           const Shape& convex, const Transform& tConvex,
                           bool meshIsA) -> u32 {
        Transform inv = tMesh.Inverse();
        Transform convexInMesh = inv * tConvex;
        u32 total = 0;
        Manifold sub;
        if (mesh.GetType() == ShapeType::Mesh) {
            const MeshShape& m = static_cast<const MeshShape&>(mesh);
            AABB convexBounds = convex.GetWorldBounds(convexInMesh);
            for (u32 t = 0; t < m.GetNumTriangles(); ++t) {
                Vec3 a, b, c;
                m.GetTriangle(t, a, b, c);
                AABB triBounds;
                triBounds.Encapsulate(a); triBounds.Encapsulate(b); triBounds.Encapsulate(c);
                if (!triBounds.Overlaps(convexBounds)) continue;
                // Transform the triangle into world space for the test.
                Vec3 wa = tMesh * a, wb = tMesh * b, wc = tMesh * c;
                sub.Clear();
                CollideConvexTriangle(convex, tConvex, wa, wb, wc, settings, sub);
                for (u32 k = 0; k < sub.numPoints && total < kMaxManifoldPoints; ++k) {
                    outManifold.points[total++] = sub.points[k];
                }
                if (total >= kMaxManifoldPoints) break;
            }
        } else {
            const HeightFieldShape& hf = static_cast<const HeightFieldShape&>(mesh);
            AABB convexBounds = convex.GetWorldBounds(convexInMesh);
            hf.ForEachTriangleInBounds(convexBounds, [&](const Vec3& a, const Vec3& b, const Vec3& c) {
                if (total >= kMaxManifoldPoints) return;
                Vec3 wa = tMesh * a, wb = tMesh * b, wc = tMesh * c;
                sub.Clear();
                CollideConvexTriangle(convex, tConvex, wa, wb, wc, settings, sub);
                for (u32 k = 0; k < sub.numPoints && total < kMaxManifoldPoints; ++k) {
                    outManifold.points[total++] = sub.points[k];
                }
            });
        }
        outManifold.numPoints = total;
        if (meshIsA) SwapManifold(outManifold);
        return total;
    };

    if (typeA == ShapeType::Mesh || typeA == ShapeType::HeightField) {
        return collideMesh(shapeA, transformA, shapeB, transformB, true);
    }
    if (typeB == ShapeType::Mesh || typeB == ShapeType::HeightField) {
        return collideMesh(shapeB, transformB, shapeA, transformA, false);
    }

    // --- Sphere pairs -------------------------------------------------------
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Sphere) {
        return CollideSphereSphere(static_cast<const SphereShape&>(shapeA), transformA,
                                   static_cast<const SphereShape&>(shapeB), transformB,
                                   settings, outManifold);
    }
    if (typeA == ShapeType::Sphere) {
        return CollideSphereConvex(static_cast<const SphereShape&>(shapeA), transformA,
                                   shapeB, transformB, settings, outManifold);
    }
    if (typeB == ShapeType::Sphere) {
        u32 n = CollideSphereConvex(static_cast<const SphereShape&>(shapeB), transformB,
                                    shapeA, transformA, settings, outManifold);
        if (n > 0) SwapManifold(outManifold);
        return n;
    }

    // --- Generic convex-convex ---------------------------------------------
    return CollideConvexGeneric(shapeA, transformA, shapeB, transformB, settings, outManifold);
}

bool NarrowPhase::Overlap(const Shape& shapeA, const Transform& transformA,
                          const Shape& shapeB, const Transform& transformB) {
    CollideSettings settings;
    Manifold m;
    return Collide(shapeA, transformA, shapeB, transformB, settings, m) > 0;
}

GjkResult NarrowPhase::ClosestPoints(const Shape& shapeA, const Transform& transformA,
                                     const Shape& shapeB, const Transform& transformB) {
    SupportMap map(shapeA, transformA, shapeB, transformB);
    return GJK::GetClosestPoints(map, Real(1.0e-4));
}

bool NarrowPhase::CastShape(const Shape& shapeA, const Transform& transformA, const Vec3& direction,
                            Real maxDistance, const Shape& shapeB, const Transform& transformB,
                            Real& outFraction, Vec3& outNormal) {
    // Conservative advancement using GJK distance.
    Real dirLen = direction.Length();
    if (dirLen < math::kEpsilon) return false;
    Vec3 dir = direction / dirLen;

    // Planes are unbounded and have no meaningful GJK support, so handle them
    // analytically: the deepest support point of shapeA toward the plane
    // reaches it after t = s / (-normal . dir).
    if (shapeB.GetType() == ShapeType::Plane) {
        if (shapeA.GetType() == ShapeType::Plane) return false;
        const PlaneShape& planeShape = static_cast<const PlaneShape&>(shapeB);
        const Plane& plane = planeShape.GetPlane();
        Vec3 nWorld = transformB.rotation.Rotate(plane.normal).Normalized();
        Vec3 pointOnPlane = transformB * (plane.normal * plane.distance);
        Real planeDist = nWorld.Dot(pointOnPlane);

        Vec3 localDirA = transformA.rotation.InverseRotate(-nWorld);
        Vec3 deepest = transformA * shapeA.GetSupport(localDirA);
        Real s = nWorld.Dot(deepest) - planeDist;
        if (s <= Real(1.0e-4)) {
            outFraction = Real(0);
            outNormal = nWorld;
            return true;
        }
        Real rate = nWorld.Dot(dir);
        if (rate >= -math::kEpsilon) return false; // moving parallel/away
        Real tHit = s / (-rate);
        if (tHit > maxDistance) return false;
        outFraction = tHit / maxDistance;
        outNormal = nWorld;
        return true;
    }

    Real t = Real(0);
    const Real tol = Real(1.0e-4);
    for (u32 iter = 0; iter < 64; ++iter) {
        Transform current(transformA.rotation, transformA.translation + dir * t);
        SupportMap map(shapeA, current, shapeB, transformB);
        GjkResult gjk = GJK::GetClosestPoints(map, tol);
        if (gjk.intersect || gjk.distance <= tol) {
            outFraction = t / maxDistance;
            outNormal = gjk.normal;
            return true;
        }
        if (t >= maxDistance) return false;
        Real advance = gjk.distance - tol;
        if (advance <= Real(0)) advance = tol;
        t += advance;
        if (t > maxDistance) return false;
    }
    return false;
}

} // namespace kizuri
