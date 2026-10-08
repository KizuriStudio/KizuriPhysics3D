// KizuriPhysics - Collision/EPA.cpp
#include "Kizuri/Collision/EPA.h"
#include <cstdio>

namespace kizuri {

namespace {

struct EpaFace {
    u32 a = 0, b = 0, c = 0;
    Vec3 normal = Vec3::UnitY();
    Real dist = 0;   // distance from origin to the face plane along the normal
    bool alive = true;
};

KZ_FORCEINLINE bool BuildFace(EpaFace& f, const SupportPoint* verts) {
    Vec3 A = verts[f.a].point, B = verts[f.b].point, C = verts[f.c].point;
    Vec3 n = (B - A).Cross(C - A);
    Real len = n.Length();
    if (len < Real(1.0e-12)) return false;
    n /= len;
    Real d = n.Dot(A);
    if (d < Real(0)) { // ensure outward (positive distance)
        n = -n;
        d = -d;
        u32 tmp = f.b; f.b = f.c; f.c = tmp;
    }
    f.normal = n;
    f.dist = d;
    return true;
}

/// Returns a unit vector perpendicular to `v` (v need not be normalized).
Vec3 Perp(const Vec3& v) {
    if (math::Abs(v.x) <= math::Abs(v.y) && math::Abs(v.x) <= math::Abs(v.z))
        return Vec3(Real(0), -v.z, v.y);
    if (math::Abs(v.y) <= math::Abs(v.z))
        return Vec3(-v.z, Real(0), v.x);
    return Vec3(-v.y, v.x, Real(0));
}

/// The origin must lie well inside every face plane, so that EPA never starts
/// with the origin sitting on (or near) a face.
bool OriginStrictlyInTetra(const SupportPoint* v) {
    auto ok = [](const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        Vec3 n = (b - a).Cross(c - a);
        if (n.Dot(d - a) > Real(0)) n = -n;
        return n.Dot(Vec3::Zero() - a) < Real(-1.0e-9);
    };
    return ok(v[0].point, v[1].point, v[2].point, v[3].point) &&
           ok(v[0].point, v[2].point, v[3].point, v[1].point) &&
           ok(v[0].point, v[3].point, v[1].point, v[2].point) &&
           ok(v[1].point, v[3].point, v[2].point, v[0].point);
}

/// Build a tetrahedron that encloses the origin, starting from the GJK simplex
/// and completing it with support points (van den Bergen's method).
bool BuildInitialTetra(const SupportMap& map, const GjkResult& gjk, SupportPoint out[4]) {
    u32 n = 0;
    for (u32 i = 0; i < gjk.simplexSize && n < 4; ++i) {
        bool dup = false;
        for (u32 j = 0; j < n; ++j)
            if ((out[j].point - gjk.simplex[i].point).LengthSq() < Real(1.0e-12)) dup = true;
        if (!dup) out[n++] = gjk.simplex[i];
    }
    if (n == 0) out[n++] = map.GetSupport(Vec3::UnitX());
    if (n == 4 && !OriginStrictlyInTetra(out)) n = 3; // seed tetra did not enclose origin

    if (n == 1) {
        out[n] = map.GetSupport(-out[0].point);
        if ((out[1].point - out[0].point).LengthSq() > Real(1.0e-12)) ++n;
    }
    if (n == 2) {
        Vec3 ab = out[1].point - out[0].point;
        if (ab.LengthSq() > Real(1.0e-12)) {
            // Build a diamond of support points perpendicular to the segment.
            // Its equatorial plane passes through the origin whenever the
            // segment does, which is exactly the degenerate case that would
            // otherwise leave the origin on a tetrahedron edge.
            Vec3 d1 = Perp(ab).Normalized();
            Vec3 d2 = ab.Cross(d1).Normalized();
            SupportPoint keep0 = out[0], keep1 = out[1];
            out[0] = map.GetSupport(d1);
            out[1] = map.GetSupport(-d1);
            out[2] = map.GetSupport(d2);
            out[3] = map.GetSupport(-d2);
            if (OriginStrictlyInTetra(out)) return true;
            out[0] = keep0; out[1] = keep1;
        }
        SupportPoint s = map.GetSupport(Perp(ab));
        if ((s.point - out[0].point).LengthSq() > Real(1.0e-12) &&
            (s.point - out[1].point).LengthSq() > Real(1.0e-12))
            out[n++] = s;
    }
    if (n == 3) {
        Vec3 triN = (out[1].point - out[0].point).Cross(out[2].point - out[0].point);
        Real len = triN.Length();
        if (len > Real(1.0e-12)) {
            triN /= len;
            Real sd = triN.Dot(Vec3::Zero() - out[0].point);
            SupportPoint a = map.GetSupport(sd > Real(0) ? -triN : triN);
            SupportPoint b = map.GetSupport(sd > Real(0) ? triN : -triN);
            out[3] = a;
            if (!OriginStrictlyInTetra(out)) out[3] = b;
            if (OriginStrictlyInTetra(out)) return true;
        }
        n = 3;
    }
    if (n == 4 && OriginStrictlyInTetra(out)) return true;

    // --- Robust fallback ---------------------------------------------------
    // Gather candidate support points (GJK simplex + six axis extremes) and
    // search for any 4 that form a tetrahedron enclosing the origin. This
    // handles degenerate configurations (e.g. the origin lying exactly on a
    // GJK simplex edge) that the fast paths above cannot resolve.
    SupportPoint cand[16];
    u32 nc = 0;
    auto addCandidate = [&](const SupportPoint& sp) {
        for (u32 i = 0; i < nc; ++i)
            if ((cand[i].point - sp.point).LengthSq() < Real(1.0e-12)) return;
        if (nc < 16) cand[nc++] = sp;
    };
    for (u32 i = 0; i < gjk.simplexSize && i < 4; ++i) addCandidate(gjk.simplex[i]);
    const Real s = Real(0.5773502692); // 1/sqrt(3)
    const Vec3 dirs[14] = {
        Vec3::UnitX(), -Vec3::UnitX(), Vec3::UnitY(), -Vec3::UnitY(),
        Vec3::UnitZ(), -Vec3::UnitZ(),
        Vec3(s, s, s), Vec3(s, s, -s), Vec3(s, -s, s), Vec3(-s, s, s),
        Vec3(-s, -s, s), Vec3(-s, s, -s), Vec3(s, -s, -s), Vec3(-s, -s, -s)
    };
    for (int i = 0; i < 14; ++i) addCandidate(map.GetSupport(dirs[i]));

    for (u32 i = 0; i + 3 < nc; ++i)
        for (u32 j = i + 1; j + 2 < nc; ++j)
            for (u32 k = j + 1; k + 1 < nc; ++k)
                for (u32 l = k + 1; l < nc; ++l) { // NOLINT
                    SupportPoint t[4] = { cand[i], cand[j], cand[k], cand[l] };
                    Real vol = (t[1].point - t[0].point).Cross(t[2].point - t[0].point)
                                   .Dot(t[3].point - t[0].point);
                    if (math::Abs(vol) < Real(1.0e-12)) continue;
                    if (OriginStrictlyInTetra(t)) {
                        for (u32 m = 0; m < 4; ++m) out[m] = t[m];
                        return true;
                    }
                }
    return false;
}

Vec3 BarycentricOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c,
                           Real& u, Real& v, Real& w) {
    Vec3 v0 = b - a, v1 = c - a, v2 = p - a;
    Real d00 = v0.Dot(v0), d01 = v0.Dot(v1), d11 = v1.Dot(v1);
    Real d20 = v2.Dot(v0), d21 = v2.Dot(v1);
    Real denom = d00 * d11 - d01 * d01;
    if (math::Abs(denom) < Real(1.0e-18)) { u = 1; v = 0; w = 0; return a; }
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = Real(1) - v - w;
    return a + v0 * v + v1 * w;
}

/// Fill an EpaResult from the closest polytope face.
void FinishFace(const EpaFace& face, const SupportPoint* verts, EpaResult& result) {
    Real u, v, w;
    Vec3 proj = face.normal * face.dist;
    BarycentricOnTriangle(proj, verts[face.a].point, verts[face.b].point, verts[face.c].point, u, v, w);
    result.valid = true;
    result.depth = face.dist;
    result.normal = -face.normal; // from B to A
    result.pointA = verts[face.a].pointA * u + verts[face.b].pointA * v + verts[face.c].pointA * w;
    result.pointB = verts[face.a].pointB * u + verts[face.b].pointB * v + verts[face.c].pointB * w;
}

} // namespace

EpaResult EPA::Compute(const SupportMap& map, const GjkResult& gjk, Real tolerance, u32 maxIterations) {
    EpaResult result;

    SupportPoint verts[256];
    u32 numVerts = 0;

    // Complete the GJK simplex into a tetrahedron enclosing the origin.
    SupportPoint tetra[4];
    if (!BuildInitialTetra(map, gjk, tetra)) return result;
    for (u32 i = 0; i < 4; ++i) verts[numVerts++] = tetra[i];

    // Build the initial 4 faces.
    EpaFace faces[512];
    u32 numFaces = 0;
    {
        static const u32 kTris[4][3] = { {0,1,2}, {0,2,3}, {0,3,1}, {1,3,2} };
        for (auto& t : kTris) {
            EpaFace f{ t[0], t[1], t[2] };
            if (BuildFace(f, verts)) faces[numFaces++] = f;
        }
    }
    if (numFaces < 4) return result;
#if defined(KZ_EPA_DEBUG)
    for (u32 i = 0; i < 4; ++i)
        std::fprintf(stderr, "EPA v%u = (%.3f, %.3f, %.3f)\n", i,
                     (double)verts[i].point.x, (double)verts[i].point.y, (double)verts[i].point.z);
#endif

    const Real eps = tolerance;
    const u32 kMaxVerts = 256;
    const u32 kMaxFaces = 512;

    for (u32 iter = 0; iter < maxIterations; ++iter) {
        result.iterations = iter + 1;

        // Find the face closest to the origin.
        Real bestDist = math::kInfinity;
        u32 bestFace = 0xFFFFFFFFu;
        for (u32 i = 0; i < numFaces; ++i) {
            if (faces[i].alive && faces[i].dist < bestDist) {
                bestDist = faces[i].dist;
                bestFace = i;
            }
        }
        if (bestFace == 0xFFFFFFFFu) break;

        EpaFace& face = faces[bestFace];
        SupportPoint w = map.GetSupport(face.normal);
        Real d = w.point.Dot(face.normal);
#if defined(KZ_EPA_DEBUG)
        std::fprintf(stderr, "EPA it%u nv=%u nf=%u bestDist=%.5f d=%.5f n=(%.2f,%.2f,%.2f)\n",
                     iter, numVerts, numFaces, (double)face.dist, (double)d,
                     (double)face.normal.x, (double)face.normal.y, (double)face.normal.z);
#endif

        if (d - face.dist < eps || numVerts >= kMaxVerts) {
            // Converged. Compute witness points from the closest face.
            FinishFace(face, verts, result);
            return result;
        }

        // Add the new support vertex.
        u32 newIndex = numVerts;
        verts[numVerts++] = w;

        // Mark the faces visible from w (only faces alive at the start of this
        // iteration) and collect the horizon edges of that visible set.
        struct Edge { u32 u, v; };
        u32 visible[512];
        u32 numVisible = 0;
        for (u32 i = 0; i < numFaces; ++i) {
            if (faces[i].alive && faces[i].normal.Dot(w.point) - faces[i].dist > eps) {
                faces[i].alive = false;
                visible[numVisible++] = i;
            }
        }

        Edge horizon[256];
        u32 numHorizon = 0;
        for (u32 vi = 0; vi < numVisible; ++vi) {
            const u32 i = visible[vi];
            u32 ev[3][2] = { { faces[i].a, faces[i].b }, { faces[i].b, faces[i].c }, { faces[i].c, faces[i].a } };
            for (auto& e : ev) {
                bool shared = false;
                for (u32 vj = 0; vj < numVisible && !shared; ++vj) {
                    const u32 j = visible[vj];
                    if (j == i) continue;
                    u32 gv[3][2] = { { faces[j].a, faces[j].b }, { faces[j].b, faces[j].c }, { faces[j].c, faces[j].a } };
                    for (auto& ge : gv) {
                        if (ge[0] == e[1] && ge[1] == e[0]) { shared = true; break; }
                    }
                }
                if (!shared && numHorizon < 256) horizon[numHorizon++] = { e[0], e[1] };
            }
        }

        // Compact the face array, then append new faces.
        u32 writeIdx = 0;
        for (u32 i = 0; i < numFaces; ++i) {
            if (faces[i].alive) faces[writeIdx++] = faces[i];
        }
        numFaces = writeIdx;

        for (u32 i = 0; i < numHorizon; ++i) {
            if (numFaces >= kMaxFaces) break;
            EpaFace f{ horizon[i].v, horizon[i].u, newIndex };
            if (BuildFace(f, verts)) faces[numFaces++] = f;
        }
    }

    // Not converged within maxIterations: report the closest face found. This
    // is standard for smooth shapes (spheres/capsules), where an exact
    // polytope fit would require thousands of faces.
    Real bestDist = math::kInfinity;
    u32 bestFace = 0xFFFFFFFFu;
    for (u32 i = 0; i < numFaces; ++i) {
        if (faces[i].alive && faces[i].dist < bestDist) { bestDist = faces[i].dist; bestFace = i; }
    }
    if (bestFace != 0xFFFFFFFFu) FinishFace(faces[bestFace], verts, result);
    return result;
}

} // namespace kizuri
