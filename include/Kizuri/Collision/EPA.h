// KizuriPhysics - Collision/EPA.h
// Expanding Polytope Algorithm - penetration depth for intersecting convex shapes.
#pragma once

#include "Kizuri/Collision/GJK.h"

namespace kizuri {

struct EpaResult {
    bool valid = false;
    Real depth = 0;            // penetration depth (>= 0)
    Vec3 normal = Vec3::UnitY(); // unit direction from B to A
    Vec3 pointA = Vec3::Zero();  // deepest point on A (world)
    Vec3 pointB = Vec3::Zero();  // deepest point on B (world)
    u32 iterations = 0;
};

class EPA {
public:
    /// Compute the penetration between two overlapping convex shapes.
    /// `gjk` must report an intersection. `tolerance` controls convergence.
    static EpaResult Compute(const SupportMap& map, const GjkResult& gjk,
                             Real tolerance = Real(1.0e-4), u32 maxIterations = 64);
};

} // namespace kizuri
