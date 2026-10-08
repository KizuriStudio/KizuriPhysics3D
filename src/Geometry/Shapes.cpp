// KizuriPhysics - Geometry/Shapes.cpp
#include "Kizuri/Geometry/Shape.h"

#include <algorithm>

namespace kizuri {

const char* ShapeTypeName(ShapeType type) {
    switch (type) {
        case ShapeType::Sphere:         return "Sphere";
        case ShapeType::Box:            return "Box";
        case ShapeType::Capsule:        return "Capsule";
        case ShapeType::Cylinder:       return "Cylinder";
        case ShapeType::ConvexHull:     return "ConvexHull";
        case ShapeType::Mesh:           return "Mesh";
        case ShapeType::Compound:       return "Compound";
        case ShapeType::HeightField:    return "HeightField";
        case ShapeType::Plane:          return "Plane";
        case ShapeType::TaperedCapsule: return "TaperedCapsule";
        default:                        return "Unknown";
    }
}

// ===========================================================================
// Sphere
// ===========================================================================
MassProperties SphereShape::GetMassProperties(Real density) const {
    MassProperties mp;
    mp.mass = GetVolume() * density;
    mp.inertia = Mat3::Identity() * (Real(0.4) * mp.mass * mRadius * mRadius);
    mp.centerOfMass = Vec3::Zero();
    return mp;
}

Real SphereShape::RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const {
    // |o + t d|² = r²  with |d| = 1
    Real b = ray.origin.Dot(ray.direction);
    Real c = ray.origin.LengthSq() - mRadius * mRadius;
    Real disc = b * b - c;
    if (disc < Real(0)) return maxFraction;
    Real sqrtDisc = math::Sqrt(disc);
    Real t = -b - sqrtDisc;
    if (t < Real(0)) t = -b + sqrtDisc; // ray starts inside
    if (t < Real(0) || t > maxFraction) return maxFraction;
    Vec3 point = ray.origin + ray.direction * t;
    outNormal = point.NormalizedOr(Vec3::UnitY());
    return t;
}

// ===========================================================================
// Box
// ===========================================================================
MassProperties BoxShape::GetMassProperties(Real density) const {
    MassProperties mp;
    mp.mass = GetVolume() * density;
    Real m = mp.mass;
    Real hx = mHalfExtent.x, hy = mHalfExtent.y, hz = mHalfExtent.z;
    // Full dimensions are 2h; Ixx = m/12*((2hy)^2+(2hz)^2) = m/3*(hy^2+hz^2).
    Real ixx = m / Real(3) * (hy * hy + hz * hz);
    Real iyy = m / Real(3) * (hx * hx + hz * hz);
    Real izz = m / Real(3) * (hx * hx + hy * hy);
    mp.inertia = Mat3(Vec3(ixx, 0, 0), Vec3(0, iyy, 0), Vec3(0, 0, izz));
    mp.centerOfMass = Vec3::Zero();
    return mp;
}

Real BoxShape::RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const {
    // Slab method.
    Real tMin = Real(0), tMax = maxFraction;
    int hitAxis = -1;
    Real hitSign = Real(1);

    for (int i = 0; i < 3; ++i) {
        Real o = ray.origin[i];
        Real d = ray.direction[i];
        Real e = mHalfExtent[i];
        if (math::Abs(d) < math::kEpsilon) {
            if (o < -e || o > e) return maxFraction;
            continue;
        }
        Real inv = Real(1) / d;
        Real t1 = (-e - o) * inv;
        Real t2 = (e - o) * inv;
        Real sign = Real(-1);
        if (t1 > t2) { Real tmp = t1; t1 = t2; t2 = tmp; sign = Real(1); }
        if (t1 > tMin) { tMin = t1; hitAxis = i; hitSign = sign; }
        if (t2 < tMax) tMax = t2;
        if (tMin > tMax) return maxFraction;
    }

    if (hitAxis < 0) return maxFraction; // origin inside; no entry face
    outNormal = Vec3::Zero();
    outNormal[hitAxis] = hitSign;
    return tMin;
}

Real BoxShape::GetSignedDistance(const Vec3& point) const {
    Vec3 d = point.Abs() - mHalfExtent;
    Vec3 dMax = d.Max(Vec3(Real(0)));
    Real outside = dMax.Length();
    Real inside = math::Min(d.MaxComponent(), Real(0));
    return outside + inside;
}

// ===========================================================================
// Capsule
// ===========================================================================
MassProperties CapsuleShape::GetMassProperties(Real density) const {
    // Composite of a cylinder (radius r, height 2h) and two hemispheres.
    Real r = mRadius, h = mHalfHeight;
    Real volCyl = math::kPi * r * r * (Real(2) * h);
    Real volSph = Real(4.0 / 3.0) * math::kPi * r * r * r;
    Real vol = volCyl + volSph;
    if (vol <= math::kEpsilon) return MassProperties{};

    Real mCyl = volCyl * density;
    Real mSph = volSph * density;
    Real m = vol * density;

    // Cylinder inertia about center.
    Real iCylAxial = mCyl * r * r * Real(0.5);
    Real iCylTrans = mCyl * (Real(3) * r * r + Real(4) * h * h) / Real(12);

    // Sphere inertia about its own center.
    Real iSph = Real(0.4) * mSph * r * r;

    // Each hemisphere's center of mass is at distance 3r/8 from the sphere center,
    // i.e. at y = ±(h + 3r/8) from the capsule center.
    Real dSph = h + Real(3) * r / Real(8);
    Real iSphTrans = iSph + mSph * dSph * dSph; // parallel axis (both hemispheres symmetric)

    Real ixx = iCylTrans + iSphTrans;
    Real iyy = iCylAxial + iSph;
    Real izz = ixx;

    MassProperties mp;
    mp.mass = m;
    mp.inertia = Mat3(Vec3(ixx, 0, 0), Vec3(0, iyy, 0), Vec3(0, 0, izz));
    mp.centerOfMass = Vec3::Zero();
    return mp;
}

Real CapsuleShape::RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const {
    // Ray vs sphere-swept segment: |o + t d - closest(t)| = r.
    // The segment is along Y in [-h, h]. Solve by clamping the closest point.
    Real h = mHalfHeight;
    // Sphere at top and bottom caps, plus the cylinder side.
    Real best = maxFraction;
    Vec3 bestN = Vec3::UnitY();

    // Cap spheres.
    for (int s = 0; s < 2; ++s) {
        Vec3 center(Real(0), s == 0 ? h : -h, Real(0));
        Vec3 oc = ray.origin - center;
        Real b = oc.Dot(ray.direction);
        Real c = oc.LengthSq() - mRadius * mRadius;
        Real disc = b * b - c;
        if (disc < Real(0)) continue;
        Real sq = math::Sqrt(disc);
        Real t = -b - sq;
        if (t < Real(0)) t = -b + sq;
        if (t >= Real(0) && t < best) {
            Vec3 p = ray.origin + ray.direction * t;
            Real y = p.y;
            if ((s == 0 && y >= h) || (s == 1 && y <= -h)) {
                best = t;
                bestN = (p - center).NormalizedOr(Vec3::UnitY());
            }
        }
    }

    // Cylinder side (infinite cylinder clamped to [-h, h]).
    Real ox = ray.origin.x, oz = ray.origin.z;
    Real dx = ray.direction.x, dz = ray.direction.z;
    Real a = dx * dx + dz * dz;
    if (a > math::kEpsilon) {
        Real b = ox * dx + oz * dz;
        Real c = ox * ox + oz * oz - mRadius * mRadius;
        Real disc = b * b - a * c;
        if (disc >= Real(0)) {
            Real sq = math::Sqrt(disc);
            Real t = (-b - sq) / a;
            if (t < Real(0)) t = (-b + sq) / a;
            if (t >= Real(0) && t < best) {
                Vec3 p = ray.origin + ray.direction * t;
                if (math::Abs(p.y) <= h) {
                    best = t;
                    bestN = Vec3(p.x, Real(0), p.z).NormalizedOr(Vec3::UnitX());
                }
            }
        }
    }

    if (best < maxFraction) { outNormal = bestN; return best; }
    return maxFraction;
}

// ===========================================================================
// Cylinder
// ===========================================================================
MassProperties CylinderShape::GetMassProperties(Real density) const {
    MassProperties mp;
    Real r = mRadius, h = Real(2) * mHalfHeight;
    mp.mass = GetVolume() * density;
    Real m = mp.mass;
    Real iAxial = m * r * r * Real(0.5);
    Real iTrans = m * (Real(3) * r * r + h * h) / Real(12);
    mp.inertia = Mat3(Vec3(iTrans, 0, 0), Vec3(0, iAxial, 0), Vec3(0, 0, iTrans));
    mp.centerOfMass = Vec3::Zero();
    return mp;
}

Real CylinderShape::RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const {
    Real best = maxFraction;
    Vec3 bestN = Vec3::UnitY();

    // Infinite cylinder side.
    Real ox = ray.origin.x, oz = ray.origin.z;
    Real dx = ray.direction.x, dz = ray.direction.z;
    Real a = dx * dx + dz * dz;
    if (a > math::kEpsilon) {
        Real b = ox * dx + oz * dz;
        Real c = ox * ox + oz * oz - mRadius * mRadius;
        Real disc = b * b - a * c;
        if (disc >= Real(0)) {
            Real sq = math::Sqrt(disc);
            Real t = (-b - sq) / a;
            if (t < Real(0)) t = (-b + sq) / a;
            if (t >= Real(0) && t < best) {
                Vec3 p = ray.origin + ray.direction * t;
                if (math::Abs(p.y) <= mHalfHeight) {
                    best = t;
                    bestN = Vec3(p.x, Real(0), p.z).NormalizedOr(Vec3::UnitX());
                }
            }
        }
    }

    // End caps (planes y = ±h).
    if (math::Abs(ray.direction.y) > math::kEpsilon) {
        for (int s = 0; s < 2; ++s) {
            Real planeY = s == 0 ? mHalfHeight : -mHalfHeight;
            Real t = (planeY - ray.origin.y) / ray.direction.y;
            if (t >= Real(0) && t < best) {
                Vec3 p = ray.origin + ray.direction * t;
                if (p.x * p.x + p.z * p.z <= mRadius * mRadius) {
                    best = t;
                    bestN = Vec3(Real(0), s == 0 ? Real(1) : Real(-1), Real(0));
                }
            }
        }
    }

    if (best < maxFraction) { outNormal = bestN; return best; }
    return maxFraction;
}

// ===========================================================================
// TaperedCapsule
// ===========================================================================
Vec3 TaperedCapsuleShape::GetSupport(const Vec3& direction) const {
    // Support of the cone part plus the two sphere caps.
    // Cone: bottom at y=-h radius rb, top at y=+h radius rt.
    Vec3 d = direction;
    Vec3 radial(d.x, Real(0), d.z);
    Vec3 rn = radial.NormalizedOr(Vec3::UnitX());
    Real radialMag = radial.Length();

    Vec3 topPoint(Real(0), mHalfHeight, Real(0));
    Vec3 botPoint(Real(0), -mHalfHeight, Real(0));
    Real topDot = topPoint.Dot(d) + radialMag * mTopRadius;
    Real botDot = botPoint.Dot(d) + radialMag * mBottomRadius;

    if (topDot >= botDot) {
        return Vec3(rn.x * mTopRadius, mHalfHeight, rn.z * mTopRadius);
    }
    return Vec3(rn.x * mBottomRadius, -mHalfHeight, rn.z * mBottomRadius);
}

MassProperties TaperedCapsuleShape::GetMassProperties(Real density) const {
    // Approximate by composing the frustum with two hemispherical caps.
    Real h = Real(2) * mHalfHeight;
    Real r1 = mBottomRadius, r2 = mTopRadius;
    Real volFrustum = math::kPi * h / Real(3) * (r1*r1 + r1*r2 + r2*r2);
    Real volCaps = Real(2.0 / 3.0) * math::kPi * (r1*r1*r1 + r2*r2*r2);
    Real vol = volFrustum + volCaps;
    if (vol <= math::kEpsilon) return MassProperties{};

    Real m = vol * density;
    // Use a conservative bounding-box-based inertia approximation.
    AABB b = GetLocalBounds();
    Vec3 e = b.Extent();
    Real ixx = m / Real(3) * (e.y * e.y + e.z * e.z);
    Real iyy = m / Real(3) * (e.x * e.x + e.z * e.z);
    Real izz = m / Real(3) * (e.x * e.x + e.y * e.y);
    MassProperties mp;
    mp.mass = m;
    mp.inertia = Mat3(Vec3(ixx, 0, 0), Vec3(0, iyy, 0), Vec3(0, 0, izz));
    return mp;
}

// ===========================================================================
// ConvexHull
// ===========================================================================
namespace {

struct HullFace {
    u32 a = 0, b = 0, c = 0;
    Vec3 normal = Vec3::UnitY();
    Real dist = 0; // plane distance: dot(normal, a)
    bool alive = true;
};

KZ_FORCEINLINE void MakeFace(HullFace& f, const Vec3* pts, const Vec3& interior) {
    Vec3 A = pts[f.a], B = pts[f.b], C = pts[f.c];
    Vec3 n = (B - A).Cross(C - A);
    Real len = n.Length();
    if (len < Real(1.0e-12)) { f.alive = false; return; }
    n /= len;
    // Ensure outward orientation relative to an interior point.
    if (n.Dot(A - interior) < Real(0)) {
        u32 tmp = f.b; f.b = f.c; f.c = tmp;
        n = -n;
    }
    f.normal = n;
    f.dist = n.Dot(A);
}

} // namespace

Ref<ConvexHullShape> ConvexHullShape::Create(ArrayView<const Vec3> inputPoints, u32 maxPoints) {
    if (inputPoints.Size() < 4) return nullptr;

    // Deduplicate and cap the point count.
    Vector<Vec3> pts;
    pts.Reserve(math::Min(inputPoints.Size(), usize(maxPoints)));
    for (usize i = 0; i < inputPoints.Size(); ++i) {
        const Vec3& p = inputPoints[i];
        bool dup = false;
        for (const Vec3& q : pts) {
            if ((p - q).LengthSq() < Real(1.0e-10)) { dup = true; break; }
        }
        if (!dup) {
            pts.PushBack(p);
            if (pts.Size() >= maxPoints) break;
        }
    }
    if (pts.Size() < 4) return nullptr;

    // --- Build an initial tetrahedron -------------------------------------
    // p0: extreme along X
    u32 i0 = 0;
    for (u32 i = 1; i < pts.Size(); ++i) if (pts[i].x < pts[i0].x) i0 = i;
    // p1: farthest from p0
    u32 i1 = 0; Real best = -1;
    for (u32 i = 0; i < pts.Size(); ++i) {
        if (i == i0) continue;
        Real d = (pts[i] - pts[i0]).LengthSq();
        if (d > best) { best = d; i1 = i; }
    }
    // p2: farthest from line p0p1
    u32 i2 = 0; best = -1;
    Vec3 axis = (pts[i1] - pts[i0]).Normalized();
    for (u32 i = 0; i < pts.Size(); ++i) {
        if (i == i0 || i == i1) continue;
        Vec3 rel = pts[i] - pts[i0];
        Real d = (rel - axis * rel.Dot(axis)).LengthSq();
        if (d > best) { best = d; i2 = i; }
    }
    if (best < Real(1.0e-12)) return nullptr; // all collinear
    // p3: farthest from plane p0p1p2
    Vec3 planeN = (pts[i1] - pts[i0]).Cross(pts[i2] - pts[i0]).Normalized();
    u32 i3 = 0; best = -1;
    for (u32 i = 0; i < pts.Size(); ++i) {
        if (i == i0 || i == i1 || i == i2) continue;
        Real d = math::Abs(planeN.Dot(pts[i] - pts[i0]));
        if (d > best) { best = d; i3 = i; }
    }
    if (best < Real(1.0e-10)) return nullptr; // all coplanar

    Vec3 interior = (pts[i0] + pts[i1] + pts[i2] + pts[i3]) * Real(0.25);

    Vector<HullFace> faces;
    {
        u32 idx[4] = { i0, i1, i2, i3 };
        static const int kTris[4][3] = { {0,1,2}, {0,2,3}, {0,3,1}, {1,3,2} };
        for (auto& t : kTris) {
            HullFace f{ idx[t[0]], idx[t[1]], idx[t[2]] };
            MakeFace(f, pts.Data(), interior);
            if (f.alive) faces.PushBack(f);
        }
    }
    if (faces.Size() < 4) return nullptr;

    // --- Incrementally add the remaining points ---------------------------
    const Real eps = Real(1.0e-7);
    for (u32 pi = 0; pi < pts.Size(); ++pi) {
        if (pi == i0 || pi == i1 || pi == i2 || pi == i3) continue;
        const Vec3& p = pts[pi];

        // Collect the faces visible from p (only faces alive at the start of
        // this iteration count; faces killed in earlier iterations are ignored).
        struct Edge { u32 u, v; };
        Vector<u32> visible;
        for (usize fi = 0; fi < faces.Size(); ++fi) {
            HullFace& f = faces[fi];
            if (f.alive && f.normal.Dot(p) - f.dist > eps) visible.PushBack(u32(fi));
        }
        if (visible.Empty()) continue; // inside the hull

        // Horizon = directed edges of the visible set with no matching reverse
        // edge inside the same set.
        Vector<Edge> horizon;
        for (u32 fi : visible) {
            const HullFace& f = faces[fi];
            u32 ev[3][2] = { { f.a, f.b }, { f.b, f.c }, { f.c, f.a } };
            for (auto& e : ev) {
                bool shared = false;
                for (u32 fj : visible) {
                    if (fj == fi) continue;
                    const HullFace& g = faces[fj];
                    u32 gv[3][2] = { { g.a, g.b }, { g.b, g.c }, { g.c, g.a } };
                    for (auto& ge : gv) {
                        if (ge[0] == e[1] && ge[1] == e[0]) { shared = true; break; }
                    }
                    if (shared) break;
                }
                if (!shared) horizon.PushBack({ e[0], e[1] });
            }
        }

        // Kill the visible faces now that the horizon is known.
        for (u32 fi : visible) faces[fi].alive = false;

        // Create new faces from the horizon edges to p. New face uses the
        // reversed edge so winding stays consistent with outward normals.
        for (const Edge& e : horizon) {
            HullFace f{ e.v, e.u, pi };
            MakeFace(f, pts.Data(), interior);
            if (f.alive) faces.PushBack(f);
        }
    }

    // --- Extract used vertices and faces ----------------------------------
    auto hull = Ref<ConvexHullShape>(new ConvexHullShape());
    hull->mPoints.Reserve(pts.Size());

    // Compact vertices: map original index -> hull index.
    Vector<i32> remap(pts.Size(), GetDefaultAllocator());
    for (auto& r : remap) r = -1;

    for (const HullFace& f : faces) {
        if (!f.alive) continue;
        u32 tri[3] = { f.a, f.b, f.c };
        for (u32 k = 0; k < 3; ++k) {
            if (remap[tri[k]] < 0) {
                remap[tri[k]] = i32(hull->mPoints.Size());
                hull->mPoints.PushBack(pts[tri[k]]);
            }
        }
        hull->mFaceSizes.PushBack(3);
        hull->mFaceIndices.PushBack(u32(remap[f.a]));
        hull->mFaceIndices.PushBack(u32(remap[f.b]));
        hull->mFaceIndices.PushBack(u32(remap[f.c]));
    }

    if (hull->mPoints.Size() < 4 || hull->mFaceSizes.Size() < 4) return nullptr;

    hull->ComputeDerivedData();
    return hull;
}

Ref<ConvexHullShape> ConvexHullShape::CreateFromFaces(ArrayView<const Vec3> points,
                                                      ArrayView<const u32> faceIndices,
                                                      ArrayView<const u8> faceSizes) {
    if (points.Size() < 4) return nullptr;
    auto hull = Ref<ConvexHullShape>(new ConvexHullShape());
    hull->mPoints.Reserve(points.Size());
    for (usize i = 0; i < points.Size(); ++i) hull->mPoints.PushBack(points[i]);
    hull->mFaceIndices.Reserve(faceIndices.Size());
    for (usize i = 0; i < faceIndices.Size(); ++i) hull->mFaceIndices.PushBack(faceIndices[i]);
    hull->mFaceSizes.Reserve(faceSizes.Size());
    for (usize i = 0; i < faceSizes.Size(); ++i) hull->mFaceSizes.PushBack(faceSizes[i]);
    hull->ComputeDerivedData();
    return hull;
}

void ConvexHullShape::ComputeDerivedData() {
    // Bounds and centroid.
    mLocalBounds.SetEmpty();
    Vec3 sum = Vec3::Zero();
    for (const Vec3& p : mPoints) {
        mLocalBounds.Encapsulate(p);
        sum += p;
    }
    mCentroid = sum / Real(mPoints.Size());
    mInteriorPoint = mCentroid;

    Vec3 c = mLocalBounds.Center();
    mBoundingRadius = 0;
    for (const Vec3& p : mPoints) {
        mBoundingRadius = math::Max(mBoundingRadius, (p - c).Length());
    }

    // Face normals.
    mFaceNormals.Clear();
    mFaceNormals.Reserve(mFaceSizes.Size());
    u32 offset = 0;
    for (u32 fi = 0; fi < mFaceSizes.Size(); ++fi) {
        u32 n = mFaceSizes[fi];
        Vec3 normal = Vec3::UnitY();
        if (n >= 3) {
            Vec3 a = mPoints[mFaceIndices[offset]];
            Vec3 b = mPoints[mFaceIndices[offset + 1]];
            Vec3 cc = mPoints[mFaceIndices[offset + 2]];
            normal = (b - a).Cross(cc - a).NormalizedOr(Vec3::UnitY());
        }
        mFaceNormals.PushBack(normal);
        offset += n;
    }

    // Volume via the divergence theorem over the triangulated faces.
    Real vol = 0;
    offset = 0;
    for (u32 fi = 0; fi < mFaceSizes.Size(); ++fi) {
        u32 n = mFaceSizes[fi];
        for (u32 t = 1; t + 1 < n; ++t) {
            Vec3 a = mPoints[mFaceIndices[offset]];
            Vec3 b = mPoints[mFaceIndices[offset + t]];
            Vec3 cc = mPoints[mFaceIndices[offset + t + 1]];
            vol += a.Dot(b.Cross(cc));
        }
        offset += n;
    }
    mVolume = math::Abs(vol) / Real(6);
}

Vec3 ConvexHullShape::GetSupport(const Vec3& direction) const {
    KZ_ASSERT(!mPoints.Empty());
    Real bestDot = -math::kInfinity;
    Vec3 best = mPoints[0];
    for (const Vec3& p : mPoints) {
        Real d = p.Dot(direction);
        if (d > bestDot) { bestDot = d; best = p; }
    }
    return best;
}

MassProperties ConvexHullShape::GetMassProperties(Real density) const {
    // Decompose into tetrahedra (origin, triangle) and integrate.
    MassProperties mp;
    if (mFaceSizes.Empty()) return mp;

    Real totalVol = 0;
    Vec3 centroidAcc = Vec3::Zero();
    Mat3 inertiaAcc = Mat3::Zero();

    u32 offset = 0;
    for (u32 fi = 0; fi < mFaceSizes.Size(); ++fi) {
        u32 n = mFaceSizes[fi];
        for (u32 t = 1; t + 1 < n; ++t) {
            Vec3 a = mPoints[mFaceIndices[offset]];
            Vec3 b = mPoints[mFaceIndices[offset + t]];
            Vec3 cc = mPoints[mFaceIndices[offset + t + 1]];

            Real signedVol6 = a.Dot(b.Cross(cc)); // 6 * signed volume
            Real signedVol = signedVol6 / Real(6);
            if (math::Abs(signedVol) < Real(1.0e-12)) continue;

            Real tetMass = math::Abs(signedVol) * density;
            Real sign = signedVol > 0 ? Real(1) : Real(-1);

            totalVol += math::Abs(signedVol);
            centroidAcc += (a + b + cc) * (signedVol / Real(4));

            // Inertia of tetra (0,a,b,c) about origin, per derivation.
            Vec3 aa = a, bb = b, ccv = cc;
            Real a2 = aa.LengthSq(), b2 = bb.LengthSq(), c2 = ccv.LengthSq();
            Real ab = aa.Dot(bb), ac = aa.Dot(ccv), bc = bb.Dot(ccv);
            Real tr = (a2 + b2 + c2) + (ab + ac + bc);

            Mat3 outerAA = Mat3::OuterProduct(aa, aa);
            Mat3 outerBB = Mat3::OuterProduct(bb, bb);
            Mat3 outerCC = Mat3::OuterProduct(ccv, ccv);
            Mat3 crossAB = Mat3::OuterProduct(aa, bb) + Mat3::OuterProduct(bb, aa);
            Mat3 crossAC = Mat3::OuterProduct(aa, ccv) + Mat3::OuterProduct(ccv, aa);
            Mat3 crossBC = Mat3::OuterProduct(bb, ccv) + Mat3::OuterProduct(ccv, bb);

            Mat3 tetInertia = Mat3::Identity() * (tetMass / Real(10) * tr)
                            - (outerAA + outerBB + outerCC) * (tetMass / Real(10))
                            - (crossAB + crossAC + crossBC) * (tetMass / Real(20));
            inertiaAcc += tetInertia * sign;
        }
        offset += n;
    }

    if (totalVol <= math::kEpsilon) return mp;

    mp.mass = totalVol * density;
    Vec3 com = centroidAcc / totalVol;
    mp.centerOfMass = com;

    // Shift inertia from origin to the center of mass (parallel axis).
    Real c2 = com.LengthSq();
    Mat3 shift = Mat3::Identity() * (mp.mass * c2) - Mat3::OuterProduct(com, com) * mp.mass;
    mp.inertia = inertiaAcc - shift;
    return mp;
}

// ===========================================================================
// MeshShape
// ===========================================================================
struct MeshShape::BVH {
    struct Node {
        AABB bounds;
        u32 left = 0;   // child index or first triangle
        u32 count = 0;  // 0 for internal nodes
        u32 right = 0;
    };
    Vector<Node> nodes;
    Vector<u32> triIndices;

    void Build(const Vector<Vec3>& verts, const Vector<u32>& indices);
    void BuildRecursive(const Vector<Vec3>& verts, const Vector<u32>& indices,
                        u32 nodeIndex, u32 start, u32 count);
};

void MeshShape::BVH::Build(const Vector<Vec3>& verts, const Vector<u32>& indices) {
    u32 triCount = u32(indices.Size()) / 3;
    triIndices.Resize(triCount);
    for (u32 i = 0; i < triCount; ++i) triIndices[i] = i;
    nodes.Reserve(triCount * 2);
    nodes.PushBack(Node{});
    BuildRecursive(verts, indices, 0, 0, triCount);
}

void MeshShape::BVH::BuildRecursive(const Vector<Vec3>& verts, const Vector<u32>& indices,
                                    u32 nodeIndex, u32 start, u32 count) {
    AABB bounds;
    AABB centroids;
    for (u32 i = 0; i < count; ++i) {
        u32 tri = triIndices[start + i];
        Vec3 a = verts[indices[tri * 3]];
        Vec3 b = verts[indices[tri * 3 + 1]];
        Vec3 c = verts[indices[tri * 3 + 2]];
        bounds.Encapsulate(a); bounds.Encapsulate(b); bounds.Encapsulate(c);
        centroids.Encapsulate((a + b + c) / Real(3));
    }
    nodes[nodeIndex].bounds = bounds;

    if (count <= 4) {
        nodes[nodeIndex].left = start;
        nodes[nodeIndex].count = count;
        nodes[nodeIndex].right = 0;
        return;
    }

    int axis = bounds.GetLongestAxis();
    // Split on the centroid median.
    std::sort(triIndices.Data() + start, triIndices.Data() + start + count,
              [&](u32 t0, u32 t1) {
                  Vec3 c0 = (verts[indices[t0*3]] + verts[indices[t0*3+1]] + verts[indices[t0*3+2]]) / Real(3);
                  Vec3 c1 = (verts[indices[t1*3]] + verts[indices[t1*3+1]] + verts[indices[t1*3+2]]) / Real(3);
                  return c0[axis] < c1[axis];
              });
    u32 mid = count / 2;
    u32 leftChild = u32(nodes.Size());
    nodes.PushBack(Node{});
    u32 rightChild = u32(nodes.Size());
    nodes.PushBack(Node{});
    nodes[nodeIndex].count = 0;
    nodes[nodeIndex].left = leftChild;
    nodes[nodeIndex].right = rightChild;
    BuildRecursive(verts, indices, leftChild, start, mid);
    BuildRecursive(verts, indices, rightChild, start + mid, count - mid);
}

MeshShape::MeshShape(ArrayView<const Vec3> vertices, ArrayView<const u32> indices, bool doubleSided)
    : mDoubleSided(doubleSided) {
    mVertices.Reserve(vertices.Size());
    for (usize i = 0; i < vertices.Size(); ++i) mVertices.PushBack(vertices[i]);
    mIndices.Reserve(indices.Size());
    for (usize i = 0; i < indices.Size(); ++i) mIndices.PushBack(indices[i]);

    mLocalBounds.SetEmpty();
    for (const Vec3& v : mVertices) mLocalBounds.Encapsulate(v);
    Vec3 c = mLocalBounds.Center();
    mBoundingRadius = 0;
    for (const Vec3& v : mVertices) mBoundingRadius = math::Max(mBoundingRadius, (v - c).Length());

    mBVH = new BVH();
    mBVH->Build(mVertices, mIndices);
}

MeshShape::~MeshShape() { delete mBVH; }

MassProperties MeshShape::GetMassProperties(Real density) const {
    // Only meaningful for closed, consistently wound meshes.
    MassProperties mp;
    Real totalVol = 0;
    Vec3 centroidAcc = Vec3::Zero();
    for (u32 t = 0; t < GetNumTriangles(); ++t) {
        Vec3 a, b, c;
        GetTriangle(t, a, b, c);
        Real signedVol = a.Dot(b.Cross(c)) / Real(6);
        totalVol += signedVol;
        centroidAcc += (a + b + c) * (signedVol / Real(4));
    }
    totalVol = math::Abs(totalVol);
    if (totalVol <= math::kEpsilon) return mp;
    mp.mass = totalVol * density;
    mp.centerOfMass = centroidAcc / totalVol;
    // Approximate inertia from the bounding box; adequate for static meshes.
    Vec3 e = mLocalBounds.Extent();
    Real m = mp.mass;
    Real ixx = m / Real(3) * (e.y * e.y + e.z * e.z);
    Real iyy = m / Real(3) * (e.x * e.x + e.z * e.z);
    Real izz = m / Real(3) * (e.x * e.x + e.y * e.y);
    mp.inertia = Mat3(Vec3(ixx, 0, 0), Vec3(0, iyy, 0), Vec3(0, 0, izz));
    return mp;
}

Real MeshShape::RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const {
    Real best = maxFraction;
    Vec3 bestN = Vec3::UnitY();
    // Brute-force triangle test for now (BVH-accelerated traversal added later).
    for (u32 t = 0; t < GetNumTriangles(); ++t) {
        Vec3 a, b, c;
        GetTriangle(t, a, b, c);
        Vec3 e1 = b - a, e2 = c - a;
        Vec3 p = ray.direction.Cross(e2);
        Real det = e1.Dot(p);
        if (math::Abs(det) < Real(1.0e-9)) {
            if (!mDoubleSided) continue;
        }
        Real invDet = Real(1) / det;
        Vec3 tv = ray.origin - a;
        Real u = tv.Dot(p) * invDet;
        if (u < Real(0) || u > Real(1)) continue;
        Vec3 q = tv.Cross(e1);
        Real v = ray.direction.Dot(q) * invDet;
        if (v < Real(0) || u + v > Real(1)) continue;
        Real tt = e2.Dot(q) * invDet;
        if (tt >= Real(0) && tt < best) {
            Vec3 n = e1.Cross(e2).NormalizedOr(Vec3::UnitY());
            if (mDoubleSided && n.Dot(ray.direction) > Real(0)) n = -n;
            best = tt;
            bestN = n;
        }
    }
    if (best < maxFraction) { outNormal = bestN; return best; }
    return maxFraction;
}

// ===========================================================================
// CompoundShape
// ===========================================================================
CompoundShape::CompoundShape(ArrayView<const Child> children) {
    for (usize i = 0; i < children.Size(); ++i) mChildren.PushBack(children[i]);
    Finalize();
}

void CompoundShape::AddChild(const Transform& transform, const ShapeRef& shape, u32 userIndex) {
    KZ_ASSERT(shape);
    Child child;
    child.transform = transform;
    child.shape = shape;
    child.userIndex = userIndex;
    mChildren.PushBack(child);
}

void CompoundShape::Finalize() {
    mLocalBounds.SetEmpty();
    Vec3 center = Vec3::Zero();
    if (!mChildren.Empty()) {
        for (const Child& c : mChildren) {
            mLocalBounds.Encapsulate(c.shape->GetWorldBounds(c.transform));
        }
        center = mLocalBounds.Center();
    }
    mBoundingRadius = 0;
    for (const Child& c : mChildren) {
        Real r = (c.transform.translation - center).Length() + c.shape->GetBoundingRadius();
        mBoundingRadius = math::Max(mBoundingRadius, r);
    }
}

Real CompoundShape::GetVolume() const {
    Real v = 0;
    for (const Child& c : mChildren) v += c.shape->GetVolume();
    return v;
}

MassProperties CompoundShape::GetMassProperties(Real density) const {
    MassProperties mp;
    for (const Child& c : mChildren) {
        MassProperties child = c.shape->GetMassProperties(density);
        // Express the child's properties in the compound frame.
        child.centerOfMass = c.transform * child.centerOfMass;
        child.inertia = c.transform.rotation.ToMat3() * child.inertia *
                        c.transform.rotation.ToMat3().Transposed();
        mp.Add(child);
    }
    return mp;
}

Real CompoundShape::RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const {
    Real best = maxFraction;
    Vec3 bestN = Vec3::UnitY();
    for (const Child& c : mChildren) {
        Transform inv = c.transform.Inverse();
        Ray localRay(inv * ray.origin, inv.TransformDirection(ray.direction));
        Vec3 n;
        Real t = c.shape->RayCastLocal(localRay, best, n);
        if (t < best) {
            best = t;
            bestN = c.transform.rotation.Rotate(n);
        }
    }
    if (best < maxFraction) { outNormal = bestN; return best; }
    return maxFraction;
}

bool CompoundShape::ContainsPoint(const Vec3& point) const {
    for (const Child& c : mChildren) {
        if (c.shape->ContainsPoint(c.transform.InverseTransformPoint(point))) return true;
    }
    return false;
}

// ===========================================================================
// HeightFieldShape
// ===========================================================================
HeightFieldShape::HeightFieldShape(u32 sampleCountX, u32 sampleCountZ, Real scaleX, Real scaleZ,
                                   ArrayView<const Real> heights, bool doubleSided)
    : mSampleCountX(sampleCountX), mSampleCountZ(sampleCountZ),
      mScaleX(scaleX), mScaleZ(scaleZ), mDoubleSided(doubleSided) {
    KZ_ASSERT(sampleCountX >= 2 && sampleCountZ >= 2);
    KZ_ASSERT(heights.Size() >= usize(sampleCountX) * usize(sampleCountZ));
    mHeights.Reserve(usize(sampleCountX) * usize(sampleCountZ));
    for (usize i = 0; i < usize(sampleCountX) * usize(sampleCountZ); ++i) {
        mHeights.PushBack(heights[i]);
    }

    Real minH = mHeights[0], maxH = mHeights[0];
    for (Real h : mHeights) { minH = math::Min(minH, h); maxH = math::Max(maxH, h); }
    mLocalBounds = AABB(Vec3(-scaleX, minH, -scaleZ), Vec3(scaleX, maxH, scaleZ));
    mBoundingRadius = math::Sqrt(scaleX * scaleX + scaleZ * scaleZ + math::Max(math::Abs(minH), math::Abs(maxH)) * math::Max(math::Abs(minH), math::Abs(maxH)));
}

MassProperties HeightFieldShape::GetMassProperties(Real) const { return MassProperties{}; }

Real HeightFieldShape::GetInterpolatedHeight(Real localX, Real localZ) const {
    Real fx = (localX + mScaleX) / (Real(2) * mScaleX) * Real(mSampleCountX - 1);
    Real fz = (localZ + mScaleZ) / (Real(2) * mScaleZ) * Real(mSampleCountZ - 1);
    fx = math::Clamp(fx, Real(0), Real(mSampleCountX - 1));
    fz = math::Clamp(fz, Real(0), Real(mSampleCountZ - 1));
    u32 x0 = u32(fx), z0 = u32(fz);
    u32 x1 = math::Min(x0 + 1, mSampleCountX - 1);
    u32 z1 = math::Min(z0 + 1, mSampleCountZ - 1);
    Real tx = fx - Real(x0), tz = fz - Real(z0);
    Real h00 = GetHeight(x0, z0), h10 = GetHeight(x1, z0);
    Real h01 = GetHeight(x0, z1), h11 = GetHeight(x1, z1);
    return math::Lerp(math::Lerp(h00, h10, tx), math::Lerp(h01, h11, tx), tz);
}

Vec3 HeightFieldShape::GetNormal(u32 x, u32 z) const {
    Real cellX = Real(2) * mScaleX / Real(mSampleCountX - 1);
    Real cellZ = Real(2) * mScaleZ / Real(mSampleCountZ - 1);
    Real hl = GetHeight(x > 0 ? x - 1 : x, z);
    Real hr = GetHeight(x + 1 < mSampleCountX ? x + 1 : x, z);
    Real hd = GetHeight(x, z > 0 ? z - 1 : z);
    Real hu = GetHeight(x, z + 1 < mSampleCountZ ? z + 1 : z);
    Vec3 n((hl - hr) * cellZ, Real(2) * cellX * cellZ, (hd - hu) * cellX);
    return n.NormalizedOr(Vec3::UnitY());
}

Real HeightFieldShape::RayCastLocal(const Ray& ray, Real maxFraction, Vec3& outNormal) const {
    // March the grid cells the ray passes through and test the two triangles.
    // A robust implementation uses a DDA traversal; here we sample at fixed
    // world steps for correctness with a bounded iteration count.
    Real best = maxFraction;
    Vec3 bestN = Vec3::UnitY();

    // Clip the ray to the height field bounds first.
    AABB b = mLocalBounds;
    Real tEnter = Real(0), tExit = maxFraction;
    for (int i = 0; i < 3; ++i) {
        Real o = ray.origin[i], d = ray.direction[i];
        if (math::Abs(d) < math::kEpsilon) {
            if (o < b.min[i] || o > b.max[i]) return maxFraction;
            continue;
        }
        Real inv = Real(1) / d;
        Real t1 = (b.min[i] - o) * inv;
        Real t2 = (b.max[i] - o) * inv;
        if (t1 > t2) { Real tmp = t1; t1 = t2; t2 = tmp; }
        tEnter = math::Max(tEnter, t1);
        tExit = math::Min(tExit, t2);
    }
    if (tEnter > tExit) return maxFraction;

    const int kSteps = 512;
    Real dt = (tExit - tEnter) / Real(kSteps);
    Vec3 prev = ray.PointAt(tEnter);
    Real prevDiff = prev.y - GetInterpolatedHeight(prev.x, prev.z);
    for (int s = 1; s <= kSteps; ++s) {
        Real t = tEnter + dt * Real(s);
        Vec3 cur = ray.PointAt(t);
        Real curDiff = cur.y - GetInterpolatedHeight(cur.x, cur.z);
        if ((prevDiff > Real(0)) != (curDiff > Real(0)) || curDiff == Real(0)) {
            // Linear refine between prev and cur.
            Real frac = prevDiff / (prevDiff - curDiff);
            Vec3 hit = Lerp(prev, cur, frac);
            Real tHit = tEnter + dt * (Real(s - 1) + frac);
            if (tHit < best) {
                Real hx = Real(1.0e-2);
                Real hL = GetInterpolatedHeight(hit.x - hx, hit.z);
                Real hR = GetInterpolatedHeight(hit.x + hx, hit.z);
                Real hD = GetInterpolatedHeight(hit.x, hit.z - hx);
                Real hU = GetInterpolatedHeight(hit.x, hit.z + hx);
                Vec3 n((hL - hR), Real(2) * hx, (hD - hU));
                n = n.NormalizedOr(Vec3::UnitY());
                if (mDoubleSided && n.Dot(ray.direction) > Real(0)) n = -n;
                best = tHit;
                bestN = n;
            }
            break;
        }
        prev = cur;
        prevDiff = curDiff;
    }

    if (best < maxFraction) { outNormal = bestN; return best; }
    return maxFraction;
}

} // namespace kizuri
