// KizuriPhysics - SoftBody/SoftBody.h
// Position-based dynamics soft body: particles, distance constraints and
// collisions against the rigid-body world.
#pragma once

#include "Kizuri/World/PhysicsWorld.h"

namespace kizuri {

struct SoftBodySettings {
    Vec3 gravity = Vec3(Real(0), Real(-9.81), Real(0));
    Real damping = Real(0.01);
    u32 solverIterations = 8;
    /// Collision radius of each particle.
    Real particleRadius = Real(0.05);
    /// Number of particles per edge below which the shape is considered stiff.
    u32 stiffnessScale = 1;
};

class SoftBody {
public:
    explicit SoftBody(PhysicsWorld& world, const SoftBodySettings& settings = SoftBodySettings());

    u32 AddParticle(const Vec3& position, Real inverseMass = Real(1));
    void AddEdge(u32 a, u32 b);
    void AddTriangle(u32 a, u32 b, u32 c);

    /// Build a lattice box of particles with structural + shear + bend edges.
    void CreateBox(const Vec3& center, const Vec3& halfExtents, u32 resolution);
    /// Build a cloth grid in the XZ plane at height `y`.
    void CreateCloth(const Vec3& center, Real width, Real height, u32 resX, u32 resY);

    void Update(Real dt);

    u32 GetNumParticles() const { return u32(mParticles.Size()); }
    const Vec3& GetParticlePosition(u32 i) const { return mParticles[i].position; }
    const Vec3& GetParticleVelocity(u32 i) const { return mParticles[i].velocity; }
    void SetParticlePosition(u32 i, const Vec3& p) { mParticles[i].position = p; }
    Real GetParticleInverseMass(u32 i) const { return mParticles[i].inverseMass; }

    u32 GetNumEdges() const { return u32(mEdges.Size()); }
    void GetEdge(u32 i, u32& a, u32& b) const { a = mEdges[i].a; b = mEdges[i].b; }

    u32 GetNumTriangles() const { return u32(mTriangles.Size()); }
    void GetTriangle(u32 i, u32& a, u32& b, u32& c) const {
        a = mTriangles[i].a; b = mTriangles[i].b; c = mTriangles[i].c;
    }

    const SoftBodySettings& GetSettings() const { return mSettings; }
    SoftBodySettings& GetSettings() { return mSettings; }

private:
    struct Particle {
        Vec3 position = Vec3::Zero();
        Vec3 previous = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
        Real inverseMass = Real(1);
    };
    struct Edge {
        u32 a = 0, b = 0;
        Real restLength = Real(0);
    };
    struct Triangle { u32 a = 0, b = 0, c = 0; };

    void SolveDistance(Edge& e, u32 iterations);
    void ResolveCollisions();

    PhysicsWorld& mWorld;
    SoftBodySettings mSettings;
    Vector<Particle> mParticles;
    Vector<Edge> mEdges;
    Vector<Triangle> mTriangles;
};

} // namespace kizuri
