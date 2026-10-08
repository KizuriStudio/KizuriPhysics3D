// KizuriPhysics - Collision/GJK.cpp
#include "Kizuri/Collision/GJK.h"

namespace kizuri {

namespace {

// ---------------------------------------------------------------------------
// Closest point on a triangle to a point, with barycentric coordinates.
// ---------------------------------------------------------------------------
Vec3 ClosestPtPointTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c,
                            Real& u, Real& v, Real& w) {
    Vec3 ab = b - a, ac = c - a, ap = p - a;
    Real d1 = ab.Dot(ap), d2 = ac.Dot(ap);
    if (d1 <= Real(0) && d2 <= Real(0)) { u = 1; v = 0; w = 0; return a; }

    Vec3 bp = p - b;
    Real d3 = ab.Dot(bp), d4 = ac.Dot(bp);
    if (d3 >= Real(0) && d4 <= d3) { u = 0; v = 1; w = 0; return b; }

    Real vc = d1 * d4 - d3 * d2;
    if (vc <= Real(0) && d1 >= Real(0) && d3 <= Real(0)) {
        Real t = d1 / (d1 - d3);
        u = 1 - t; v = t; w = 0;
        return a + ab * t;
    }

    Vec3 cp = p - c;
    Real d5 = ab.Dot(cp), d6 = ac.Dot(cp);
    if (d6 >= Real(0) && d5 <= d6) { u = 0; v = 0; w = 1; return c; }

    Real vb = d5 * d2 - d1 * d6;
    if (vb <= Real(0) && d2 >= Real(0) && d6 <= Real(0)) {
        Real t = d2 / (d2 - d6);
        u = 1 - t; v = 0; w = t;
        return a + ac * t;
    }

    Real va = d3 * d6 - d5 * d4;
    if (va <= Real(0) && (d4 - d3) >= Real(0) && (d5 - d6) >= Real(0)) {
        Real t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        u = 0; v = 1 - t; w = t;
        return b + (c - b) * t;
    }

    Real denom = Real(1) / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    u = Real(1) - v - w;
    return a + ab * v + ac * w;
}

// ---------------------------------------------------------------------------
// Reduce a simplex to the sub-simplex closest to the origin.
// Returns true when the origin is contained (intersection detected).
// ---------------------------------------------------------------------------
bool ReduceSimplex(SupportPoint* simplex, u32& count, Vec3& closest) {
    switch (count) {
        case 1: {
            closest = simplex[0].point;
            break;
        }
        case 2: {
            const Vec3& a = simplex[0].point;
            const Vec3& b = simplex[1].point;
            Vec3 ab = b - a;
            Real t = -a.Dot(ab);
            if (t <= Real(0)) { closest = a; count = 1; }
            else if (t >= ab.LengthSq()) { simplex[0] = simplex[1]; closest = b; count = 1; }
            else { closest = a + ab * (t / ab.LengthSq()); }
            break;
        }
        case 3: {
            const Vec3& a = simplex[0].point;
            const Vec3& b = simplex[1].point;
            const Vec3& c = simplex[2].point;
            Real u, v, w;
            closest = ClosestPtPointTriangle(Vec3::Zero(), a, b, c, u, v, w);
            // Keep only vertices with non-zero barycentric weight.
            SupportPoint tmp[3];
            u32 n = 0;
            if (u > Real(0)) tmp[n++] = simplex[0];
            if (v > Real(0)) tmp[n++] = simplex[1];
            if (w > Real(0)) tmp[n++] = simplex[2];
            if (n == 0) n = 1;
            for (u32 i = 0; i < n; ++i) simplex[i] = tmp[i];
            count = n;
            break;
        }
        case 4: {
            const Vec3& a = simplex[0].point;
            const Vec3& b = simplex[1].point;
            const Vec3& c = simplex[2].point;
            const Vec3& d = simplex[3].point;

            // Is the origin inside the tetrahedron?
            auto OutsideFace = [](const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& opposite) {
                Vec3 n = (p1 - p0).Cross(p2 - p0);
                if (n.Dot(opposite - p0) > Real(0)) n = -n; // make n point away from opposite
                return n.Dot(Vec3::Zero() - p0) > Real(0);
            };

            bool outsideABC = OutsideFace(a, b, c, d);
            bool outsideACD = OutsideFace(a, c, d, b);
            bool outsideADB = OutsideFace(a, d, b, c);
            bool outsideBDC = OutsideFace(b, d, c, a);

            if (!outsideABC && !outsideACD && !outsideADB && !outsideBDC) {
                return true; // origin inside
            }

            // Find the closest face.
            Real bestDistSq = math::kInfinity;
            SupportPoint bestTri[3];
            u32 bestCount = 0;

            auto TestFace = [&](u32 i0, u32 i1, u32 i2, bool outside) {
                if (!outside) return;
                Real u, v, w;
                Vec3 p = ClosestPtPointTriangle(Vec3::Zero(),
                                                simplex[i0].point, simplex[i1].point, simplex[i2].point,
                                                u, v, w);
                Real d2 = p.LengthSq();
                if (d2 < bestDistSq) {
                    bestDistSq = d2;
                    u32 n = 0;
                    if (u > Real(0)) bestTri[n++] = simplex[i0];
                    if (v > Real(0)) bestTri[n++] = simplex[i1];
                    if (w > Real(0)) bestTri[n++] = simplex[i2];
                    bestCount = n;
                    closest = p;
                }
            };

            TestFace(0, 1, 2, outsideABC);
            TestFace(0, 2, 3, outsideACD);
            TestFace(0, 3, 1, outsideADB);
            TestFace(1, 3, 2, outsideBDC);

            for (u32 i = 0; i < bestCount; ++i) simplex[i] = bestTri[i];
            count = bestCount;
            break;
        }
        default:
            break;
    }
    return false;
}

} // namespace

// ---------------------------------------------------------------------------
// GJK distance
// ---------------------------------------------------------------------------
GjkResult GJK::GetClosestPoints(const SupportMap& map, Real tolerance, u32 maxIterations) {
    GjkResult result;

    // Initial search direction: centers of the two shapes.
    Vec3 dir = map.GetShapeA().GetLocalBounds().Center();
    KZ_UNUSED(dir);
    Vec3 d = Vec3::UnitX();

    SupportPoint simplex[4];
    u32 count = 0;

    // Seed with an arbitrary support point.
    simplex[count++] = map.GetSupport(d);
    Vec3 closest = simplex[0].point;

    const Real tolSq = tolerance * tolerance;

    for (u32 iter = 0; iter < maxIterations; ++iter) {
        result.iterations = iter + 1;

        if (closest.LengthSq() <= tolSq) {
            result.intersect = true;
            result.distance = 0;
            // Best-effort closest points from the barycentric reduction.
            break;
        }

        d = -closest;
        SupportPoint w = map.GetSupport(d);

        // Convergence: no significant progress toward the origin.
        Real progress = closest.LengthSq() - closest.Dot(w.point);
        if (progress <= tolerance * closest.LengthSq()) {
            break;
        }

        // Duplicate guard.
        bool duplicate = false;
        for (u32 i = 0; i < count; ++i) {
            if ((simplex[i].point - w.point).LengthSq() < tolSq) { duplicate = true; break; }
        }
        if (duplicate) break;

        simplex[count++] = w;
        if (ReduceSimplex(simplex, count, closest)) {
            result.intersect = true;
            result.distance = 0;
            break;
        }
    }

    // Store the terminal simplex for EPA.
    result.simplexSize = count;
    for (u32 i = 0; i < count; ++i) result.simplex[i] = simplex[i];

    if (!result.intersect) {
        // Compute closest points by re-evaluating the barycentric weights of the
        // final simplex. For the common 1/2/3-vertex cases we can solve directly.
        Real dist = closest.Length();
        result.distance = dist;
        if (dist < math::kEpsilon) {
            result.intersect = true;
        } else {
            result.normal = -closest / dist; // from B to A
        }

        // Reconstruct world closest points via barycentric weights.
        if (count == 1) {
            result.pointA = simplex[0].pointA;
            result.pointB = simplex[0].pointB;
        } else if (count == 2) {
            const Vec3& a = simplex[0].point;
            const Vec3& b = simplex[1].point;
            Vec3 ab = b - a;
            Real t = math::Clamp(-a.Dot(ab) / math::Max(ab.LengthSq(), math::kEpsilon), Real(0), Real(1));
            result.pointA = Lerp(simplex[0].pointA, simplex[1].pointA, t);
            result.pointB = Lerp(simplex[0].pointB, simplex[1].pointB, t);
        } else if (count == 3) {
            Real u, v, w;
            ClosestPtPointTriangle(Vec3::Zero(), simplex[0].point, simplex[1].point, simplex[2].point, u, v, w);
            result.pointA = simplex[0].pointA * u + simplex[1].pointA * v + simplex[2].pointA * w;
            result.pointB = simplex[0].pointB * u + simplex[1].pointB * v + simplex[2].pointB * w;
        } else {
            result.pointA = simplex[0].pointA;
            result.pointB = simplex[0].pointB;
        }
    }

    return result;
}

bool GJK::Intersects(const SupportMap& map, u32 maxIterations) {
    GjkResult r = GetClosestPoints(map, Real(1.0e-5), maxIterations);
    return r.intersect;
}

} // namespace kizuri
