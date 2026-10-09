// KizuriPhysics - SoftBody/SoftBody.cpp
#include "Kizuri/SoftBody/SoftBody.h"

#include <cmath>

namespace kizuri {

namespace {

/// Push a sphere of `radius` at `localPoint` out of a shape. Returns false for
/// shapes without a signed distance (the collision is then skipped).
bool ShapePushOut(const Shape& shape, const Vec3& localPoint, Real radius,
                  Vec3& outNormal, Real& outDepth) {
    const Real s = shape.GetSignedDistance(localPoint);
    if (!(s < Real(1.0e30))) return false; // infinite / unsupported
    if (s > radius) return false;

    const Real h = Real(1.0e-3);
    Vec3 gradient(
        shape.GetSignedDistance(localPoint + Vec3(h, 0, 0)) -
        shape.GetSignedDistance(localPoint - Vec3(h, 0, 0)),
        shape.GetSignedDistance(localPoint + Vec3(0, h, 0)) -
        shape.GetSignedDistance(localPoint - Vec3(0, h, 0)),
        shape.GetSignedDistance(localPoint + Vec3(0, 0, h)) -
        shape.GetSignedDistance(localPoint - Vec3(0, 0, h)));
    outNormal = gradient.NormalizedOr(Vec3::UnitY());
    outDepth = radius - s;
    return true;
}

} // namespace

SoftBody::SoftBody(PhysicsWorld& world, const SoftBodySettings& settings)
    : mWorld(world), mSettings(settings) {}

u32 SoftBody::AddParticle(const Vec3& position, Real inverseMass) {
    Particle p;
    p.position = position;
    p.previous = position;
    p.inverseMass = inverseMass;
    mParticles.PushBack(p);
    return u32(mParticles.Size()) - 1;
}

void SoftBody::AddEdge(u32 a, u32 b) {
    if (a >= mParticles.Size() || b >= mParticles.Size() || a == b) return;
    Edge e;
    e.a = a;
    e.b = b;
    e.restLength = (mParticles[a].position - mParticles[b].position).Length();
    mEdges.PushBack(e);
}

void SoftBody::AddTriangle(u32 a, u32 b, u32 c) {
    mTriangles.PushBack({ a, b, c });
}

void SoftBody::CreateBox(const Vec3& center, const Vec3& halfExtents, u32 resolution) {
    if (resolution < 2) resolution = 2;
    const u32 base = u32(mParticles.Size());
    const Real stepX = (2 * halfExtents.x) / Real(resolution - 1);
    const Real stepY = (2 * halfExtents.y) / Real(resolution - 1);
    const Real stepZ = (2 * halfExtents.z) / Real(resolution - 1);

    auto index = [&](u32 x, u32 y, u32 z) {
        return base + (z * resolution + y) * resolution + x;
    };

    for (u32 z = 0; z < resolution; ++z) {
        for (u32 y = 0; y < resolution; ++y) {
            for (u32 x = 0; x < resolution; ++x) {
                Vec3 p = center + Vec3(
                    -halfExtents.x + stepX * Real(x),
                    -halfExtents.y + stepY * Real(y),
                    -halfExtents.z + stepZ * Real(z));
                AddParticle(p, Real(1));
            }
        }
    }

    // Structural edges along each axis.
    for (u32 z = 0; z < resolution; ++z) {
        for (u32 y = 0; y < resolution; ++y) {
            for (u32 x = 0; x < resolution; ++x) {
                if (x + 1 < resolution) AddEdge(index(x, y, z), index(x + 1, y, z));
                if (y + 1 < resolution) AddEdge(index(x, y, z), index(x, y + 1, z));
                if (z + 1 < resolution) AddEdge(index(x, y, z), index(x, y, z + 1));
                // Shear (face diagonals) and bend edges for stability.
                if (x + 1 < resolution && y + 1 < resolution) {
                    AddEdge(index(x, y, z), index(x + 1, y + 1, z));
                    AddEdge(index(x + 1, y, z), index(x, y + 1, z));
                }
                if (x + 1 < resolution && z + 1 < resolution) {
                    AddEdge(index(x, y, z), index(x + 1, y, z + 1));
                    AddEdge(index(x + 1, y, z), index(x, y, z + 1));
                }
                if (y + 1 < resolution && z + 1 < resolution) {
                    AddEdge(index(x, y, z), index(x, y + 1, z + 1));
                    AddEdge(index(x, y + 1, z), index(x, y, z + 1));
                }
                if (x + 2 < resolution) AddEdge(index(x, y, z), index(x + 2, y, z));
                if (y + 2 < resolution) AddEdge(index(x, y, z), index(x, y + 2, z));
                if (z + 2 < resolution) AddEdge(index(x, y, z), index(x, y, z + 2));
            }
        }
    }
}

void SoftBody::CreateCloth(const Vec3& center, Real width, Real height, u32 resX, u32 resY) {
    if (resX < 2) resX = 2;
    if (resY < 2) resY = 2;
    const u32 base = u32(mParticles.Size());
    const Real stepX = width / Real(resX - 1);
    const Real stepY = height / Real(resY - 1);

    auto index = [&](u32 x, u32 y) { return base + y * resX + x; };

    for (u32 y = 0; y < resY; ++y) {
        for (u32 x = 0; x < resX; ++x) {
            Vec3 p = center + Vec3(-width * Real(0.5) + stepX * Real(x),
                                   Real(0),
                                   -height * Real(0.5) + stepY * Real(y));
            // Pin the first row so the cloth hangs.
            const Real invMass = (y == 0) ? Real(0) : Real(1);
            AddParticle(p, invMass);
        }
    }

    for (u32 y = 0; y < resY; ++y) {
        for (u32 x = 0; x < resX; ++x) {
            if (x + 1 < resX) AddEdge(index(x, y), index(x + 1, y));
            if (y + 1 < resY) AddEdge(index(x, y), index(x, y + 1));
            if (x + 1 < resX && y + 1 < resY) {
                AddEdge(index(x, y), index(x + 1, y + 1));
                AddEdge(index(x + 1, y), index(x, y + 1));
                AddTriangle(index(x, y), index(x + 1, y), index(x + 1, y + 1));
                AddTriangle(index(x, y), index(x + 1, y + 1), index(x, y + 1));
            }
            if (x + 2 < resX) AddEdge(index(x, y), index(x + 2, y));
            if (y + 2 < resY) AddEdge(index(x, y), index(x, y + 2));
        }
    }
}

void SoftBody::SolveDistance(Edge& e, u32 iterations) {
    Particle& a = mParticles[e.a];
    Particle& b = mParticles[e.b];
    const Real wA = a.inverseMass;
    const Real wB = b.inverseMass;
    const Real wSum = wA + wB;
    if (wSum <= Real(0)) return;

    KZ_UNUSED(iterations);
    Vec3 d = b.position - a.position;
    Real len = d.Length();
    if (len < math::kEpsilon) return;

    const Real diff = (len - e.restLength) / len;
    a.position += d * (diff * wA / wSum);
    b.position -= d * (diff * wB / wSum);
}

void SoftBody::ResolveCollisions() {
    const Real radius = mSettings.particleRadius;
    BodyID ids[16];
    for (Particle& p : mParticles) {
        if (p.inverseMass <= Real(0)) continue;

        AABB box(p.position - Vec3(radius, radius, radius),
                 p.position + Vec3(radius, radius, radius));
        const u32 count = mWorld.QueryAABB(box, ids, 16);
        for (u32 i = 0; i < count; ++i) {
            const Body* body = mWorld.GetBody(ids[i]);
            if (!body || body->IsSensor()) continue;
            const Shape* shape = body->GetShape().Get();
            if (!shape) continue;

            const Transform& t = body->GetTransform();
            const Vec3 local = t.Inverse() * p.position;
            Vec3 normal;
            Real depth;
            if (!ShapePushOut(*shape, local, radius, normal, depth)) continue;

            const Vec3 worldNormal = t.rotation.Rotate(normal);
            p.position += worldNormal * depth;
            // Kill the velocity component into the surface.
            const Real vn = p.velocity.Dot(worldNormal);
            if (vn < Real(0)) p.velocity -= worldNormal * vn;
        }
    }
}

void SoftBody::Update(Real dt) {
    if (dt <= Real(0)) return;

    // 1. Predict positions.
    for (Particle& p : mParticles) {
        p.previous = p.position;
        if (p.inverseMass <= Real(0)) continue;
        p.velocity += mSettings.gravity * dt;
        p.velocity *= (Real(1) - mSettings.damping);
        p.position += p.velocity * dt;
    }

    // 2. Project distance constraints.
    const u32 iterations = math::Max(mSettings.solverIterations, u32(1));
    for (u32 it = 0; it < iterations; ++it) {
        for (Edge& e : mEdges) SolveDistance(e, iterations);
    }

    // 3. Collisions against the rigid world.
    ResolveCollisions();

    // 4. Update velocities from the corrected positions.
    const Real invDt = Real(1) / dt;
    for (Particle& p : mParticles) {
        if (p.inverseMass <= Real(0)) {
            p.velocity = Vec3::Zero();
            continue;
        }
        p.velocity = (p.position - p.previous) * invDt;
    }
}

} // namespace kizuri
