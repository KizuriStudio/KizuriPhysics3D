// KizuriPhysics - examples/Benchmark.cpp
//
// Spawns a configurable number of dynamic bodies and measures simulation
// throughput. Usage: kizuri_Benchmark [numBodies] [steps]
#include "Kizuri/Kizuri.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <random>

using namespace kizuri;

int main(int argc, char** argv) {
    int numBodies = argc > 1 ? std::atoi(argv[1]) : 2000;
    int numSteps = argc > 2 ? std::atoi(argv[2]) : 600;

    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81f, 0));

    WorldSettings& ws = world.GetSettings();
    ws.solver.velocityIterations = 8;
    ws.solver.positionIterations = 3;
    ws.useMultithreading = true;

    // Container: a static box floor and four walls.
    auto addStatic = [&](const Vec3& pos, const Vec3& half) {
        BodySettings s;
        s.shape = MakeRef<BoxShape>(half);
        s.position = pos;
        s.motionType = MotionType::Static;
        s.friction = 0.6f;
        world.CreateBody(s);
    };
    const Real R = Real(20);
    addStatic(Vec3(0, -0.5f, 0), Vec3(R, 0.5f, R));
    addStatic(Vec3(-R, 5, 0), Vec3(0.5f, 5, R));
    addStatic(Vec3(R, 5, 0), Vec3(0.5f, 5, R));
    addStatic(Vec3(0, 5, -R), Vec3(R, 5, 0.5f));
    addStatic(Vec3(0, 5, R), Vec3(R, 5, 0.5f));

    // Spawn a grid of boxes with a little jitter.
    std::mt19937 rng(12345);
    std::uniform_real_distribution<Real> jitter(-0.1f, 0.1f);
    int perRow = int(std::sqrt(double(numBodies))) + 1;
    for (int i = 0; i < numBodies; ++i) {
        int gx = i % perRow;
        int gz = (i / perRow) % perRow;
        int gy = i / (perRow * perRow);
        BodySettings s;
        s.shape = MakeRef<BoxShape>(Vec3(0.5f, 0.5f, 0.5f));
        s.position = Vec3(Real(gx) * 1.05f - R * 0.5f + jitter(rng),
                          Real(gy) * 1.05f + 1.0f,
                          Real(gz) * 1.05f - R * 0.5f + jitter(rng));
        s.motionType = MotionType::Dynamic;
        s.friction = 0.5f;
        s.restitution = 0.0f;
        world.CreateBody(s);
    }

    std::printf("Benchmark: %d bodies, %d steps, dt = 1/60 s\n", numBodies, numSteps);

    // Warm up.
    for (int i = 0; i < 30; ++i) world.StepFixed(Real(1.0 / 60.0));

    double totalMs = 0;
    double maxMs = 0;
    double totalBp = 0, totalNp = 0, totalSolve = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < numSteps; ++i) {
        world.StepFixed(Real(1.0 / 60.0));
        const WorldStats& st = world.GetStats();
        totalMs += st.stepTimeMs;
        if (st.stepTimeMs > maxMs) maxMs = st.stepTimeMs;
        totalBp += st.broadPhaseMs;
        totalNp += st.narrowPhaseMs;
        totalSolve += st.solveMs;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double wallMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::printf("  average step : %.3f ms  (%.1f FPS headroom)\n",
                totalMs / numSteps, 1000.0 / (totalMs / numSteps));
    std::printf("  max step     : %.3f ms\n", maxMs);
    std::printf("  broad phase  : %.3f ms\n", totalBp / numSteps);
    std::printf("  narrow phase : %.3f ms\n", totalNp / numSteps);
    std::printf("  solve        : %.3f ms\n", totalSolve / numSteps);
    std::printf("  wall time    : %.1f ms total\n", wallMs);
    std::printf("  final active : %u / %u bodies, %u contacts\n",
                world.GetStats().numActiveBodies, world.GetStats().numBodies,
                world.GetStats().numContactPoints);
    return 0;
}
