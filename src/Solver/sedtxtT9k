// KizuriPhysics - Solver/ContactSolverSIMD.cpp
//
// 4-wide SoA velocity solver for contact points. Contact points are grouped
// into batches of up to four where no body appears twice; the per-axis impulse
// math (relative velocity, cross products, effective mass, Coulomb clamp and
// impulse application) is then evaluated with SSE on four points at once.
//
// The scalar path in ContactSolver.cpp remains the reference implementation and
// is used whenever SSE is unavailable or a batch cannot be formed.
#include "Kizuri/Solver/ContactSolver.h"

#if defined(__SSE2__)
#include <immintrin.h>
#endif

namespace kizuri {

#if defined(__SSE2__)

namespace {

KZ_FORCEINLINE __m128 Load4(const Real* p) { return _mm_load_ps(p); }
KZ_FORCEINLINE void Store4(Real* p, __m128 v) { _mm_store_ps(p, v); }

KZ_FORCEINLINE void Cross3(__m128 ax, __m128 ay, __m128 az,
                           __m128 bx, __m128 by, __m128 bz,
                           __m128& ox, __m128& oy, __m128& oz) {
    ox = _mm_sub_ps(_mm_mul_ps(ay, bz), _mm_mul_ps(az, by));
    oy = _mm_sub_ps(_mm_mul_ps(az, bx), _mm_mul_ps(ax, bz));
    oz = _mm_sub_ps(_mm_mul_ps(ax, by), _mm_mul_ps(ay, bx));
}

KZ_FORCEINLINE __m128 Dot3(__m128 ax, __m128 ay, __m128 az,
                           __m128 bx, __m128 by, __m128 bz) {
    return _mm_add_ps(_mm_add_ps(_mm_mul_ps(ax, bx), _mm_mul_ps(ay, by)),
                      _mm_mul_ps(az, bz));
}

/// Column-major Mat3 * vector for four lanes at once.
KZ_FORCEINLINE void Mat3Mul(__m128 c0x, __m128 c0y, __m128 c0z,
                            __m128 c1x, __m128 c1y, __m128 c1z,
                            __m128 c2x, __m128 c2y, __m128 c2z,
                            __m128 vx, __m128 vy, __m128 vz,
                            __m128& ox, __m128& oy, __m128& oz) {
    ox = _mm_add_ps(_mm_add_ps(_mm_mul_ps(c0x, vx), _mm_mul_ps(c1x, vy)),
                    _mm_mul_ps(c2x, vz));
    oy = _mm_add_ps(_mm_add_ps(_mm_mul_ps(c0y, vx), _mm_mul_ps(c1y, vy)),
                    _mm_mul_ps(c2y, vz));
    oz = _mm_add_ps(_mm_add_ps(_mm_mul_ps(c0z, vx), _mm_mul_ps(c1z, vy)),
                    _mm_mul_ps(c2z, vz));
}

} // namespace

#endif // __SSE2__

void ContactSolver::SolveVelocityBatchSIMD(const ContactPointRef* refs, u32 count, Real dt) {
    KZ_UNUSED(dt);
    if (count == 0) return;
    if (count > 4) count = 4;

#if !defined(__SSE2__)
    // Scalar fallback.
    for (u32 i = 0; i < count; ++i) {
        if (refs[i].constraint) SolveVelocityConstraint(*refs[i].constraint, dt);
    }
#else
    // ---- Gather -----------------------------------------------------------
    ContactConstraint* cons[4] = { nullptr, nullptr, nullptr, nullptr };
    ContactPointConstraint* pt[4] = { nullptr, nullptr, nullptr, nullptr };
    Body* bodyA[4] = { nullptr, nullptr, nullptr, nullptr };
    Body* bodyB[4] = { nullptr, nullptr, nullptr, nullptr };

    alignas(16) Real rAx[4] = { 0, 0, 0, 0 }, rAy[4] = { 0, 0, 0, 0 }, rAz[4] = { 0, 0, 0, 0 };
    alignas(16) Real rBx[4] = { 0, 0, 0, 0 }, rBy[4] = { 0, 0, 0, 0 }, rBz[4] = { 0, 0, 0, 0 };
    alignas(16) Real vAx[4] = { 0, 0, 0, 0 }, vAy[4] = { 0, 0, 0, 0 }, vAz[4] = { 0, 0, 0, 0 };
    alignas(16) Real wAx[4] = { 0, 0, 0, 0 }, wAy[4] = { 0, 0, 0, 0 }, wAz[4] = { 0, 0, 0, 0 };
    alignas(16) Real vBx[4] = { 0, 0, 0, 0 }, vBy[4] = { 0, 0, 0, 0 }, vBz[4] = { 0, 0, 0, 0 };
    alignas(16) Real wBx[4] = { 0, 0, 0, 0 }, wBy[4] = { 0, 0, 0, 0 }, wBz[4] = { 0, 0, 0, 0 };
    alignas(16) Real nx[4] = { 0, 0, 0, 0 }, ny[4] = { 0, 1, 1, 1 }, nz[4] = { 0, 0, 0, 0 };
    alignas(16) Real t1x[4] = { 0, 0, 0, 0 }, t1y[4] = { 0, 0, 0, 0 }, t1z[4] = { 0, 0, 0, 0 };
    alignas(16) Real t2x[4] = { 0, 0, 0, 0 }, t2y[4] = { 0, 0, 0, 0 }, t2z[4] = { 0, 0, 0, 0 };
    alignas(16) Real nMass[4] = { 0, 0, 0, 0 }, t1Mass[4] = { 0, 0, 0, 0 }, t2Mass[4] = { 0, 0, 0, 0 };
    alignas(16) Real nAcc[4] = { 0, 0, 0, 0 }, t1Acc[4] = { 0, 0, 0, 0 }, t2Acc[4] = { 0, 0, 0, 0 };
    alignas(16) Real nBias[4] = { 0, 0, 0, 0 };
    alignas(16) Real fric[4] = { 0, 0, 0, 0 };
    alignas(16) Real invMA[4] = { 0, 0, 0, 0 }, invMB[4] = { 0, 0, 0, 0 };
    alignas(16) Real iA[9][4] = {};
    alignas(16) Real iB[9][4] = {};
    bool dynA[4] = { false, false, false, false };
    bool dynB[4] = { false, false, false, false };

    for (u32 i = 0; i < count; ++i) {
        cons[i] = refs[i].constraint;
        if (!cons[i] || cons[i]->isSensor) continue;
        pt[i] = &cons[i]->points[refs[i].point];
        Body* A = cons[i]->bodyA;
        Body* B = cons[i]->bodyB;
        bodyA[i] = A;
        bodyB[i] = B;
        if (!A || !B) continue;
        dynA[i] = cons[i]->dynamicA;
        dynB[i] = cons[i]->dynamicB;

        ContactPointConstraint& p = *pt[i];
        Vec3 rA = A->GetRotation().Rotate(p.localAnchorA);
        Vec3 rB = B->GetRotation().Rotate(p.localAnchorB);
        rAx[i] = rA.x; rAy[i] = rA.y; rAz[i] = rA.z;
        rBx[i] = rB.x; rBy[i] = rB.y; rBz[i] = rB.z;

        const MotionProperties& ma = A->GetMotionProperties();
        const MotionProperties& mb = B->GetMotionProperties();
        vAx[i] = ma.linearVelocity.x; vAy[i] = ma.linearVelocity.y; vAz[i] = ma.linearVelocity.z;
        wAx[i] = ma.angularVelocity.x; wAy[i] = ma.angularVelocity.y; wAz[i] = ma.angularVelocity.z;
        vBx[i] = mb.linearVelocity.x; vBy[i] = mb.linearVelocity.y; vBz[i] = mb.linearVelocity.z;
        wBx[i] = mb.angularVelocity.x; wBy[i] = mb.angularVelocity.y; wBz[i] = mb.angularVelocity.z;

        nx[i] = p.normal.x; ny[i] = p.normal.y; nz[i] = p.normal.z;
        t1x[i] = p.tangent1.x; t1y[i] = p.tangent1.y; t1z[i] = p.tangent1.z;
        t2x[i] = p.tangent2.x; t2y[i] = p.tangent2.y; t2z[i] = p.tangent2.z;

        nMass[i] = p.normalMass;
        t1Mass[i] = p.tangentMass1;
        t2Mass[i] = p.tangentMass2;
        nAcc[i] = p.normalImpulse;
        t1Acc[i] = p.tangentImpulse1;
        t2Acc[i] = p.tangentImpulse2;
        nBias[i] = p.restitutionBias + (mSettings.useVelocityBias ? p.positionBias : Real(0));
        fric[i] = p.combinedFriction;
        invMA[i] = ma.inverseMass;
        invMB[i] = mb.inverseMass;
        for (int c = 0; c < 3; ++c) {
            iA[c * 3 + 0][i] = ma.inverseInertiaWorld[c].x;
            iA[c * 3 + 1][i] = ma.inverseInertiaWorld[c].y;
            iA[c * 3 + 2][i] = ma.inverseInertiaWorld[c].z;
            iB[c * 3 + 0][i] = mb.inverseInertiaWorld[c].x;
            iB[c * 3 + 1][i] = mb.inverseInertiaWorld[c].y;
            iB[c * 3 + 2][i] = mb.inverseInertiaWorld[c].z;
        }
    }

    // ---- Load into SoA registers -----------------------------------------
    const __m128 rAxV = Load4(rAx), rAyV = Load4(rAy), rAzV = Load4(rAz);
    const __m128 rBxV = Load4(rBx), rByV = Load4(rBy), rBzV = Load4(rBz);
    __m128 vAxV = Load4(vAx), vAyV = Load4(vAy), vAzV = Load4(vAz);
    __m128 wAxV = Load4(wAx), wAyV = Load4(wAy), wAzV = Load4(wAz);
    __m128 vBxV = Load4(vBx), vByV = Load4(vBy), vBzV = Load4(vBz);
    __m128 wBxV = Load4(wBx), wByV = Load4(wBy), wBzV = Load4(wBz);
    const __m128 nxV = Load4(nx), nyV = Load4(ny), nzV = Load4(nz);
    const __m128 t1xV = Load4(t1x), t1yV = Load4(t1y), t1zV = Load4(t1z);
    const __m128 t2xV = Load4(t2x), t2yV = Load4(t2y), t2zV = Load4(t2z);
    const __m128 nMassV = Load4(nMass), t1MassV = Load4(t1Mass), t2MassV = Load4(t2Mass);
    __m128 nAccV = Load4(nAcc), t1AccV = Load4(t1Acc), t2AccV = Load4(t2Acc);
    const __m128 nBiasV = Load4(nBias);
    const __m128 fricV = Load4(fric);
    const __m128 invMAV = Load4(invMA), invMBV = Load4(invMB);
    const __m128 iA0x = Load4(iA[0]), iA0y = Load4(iA[1]), iA0z = Load4(iA[2]);
    const __m128 iA1x = Load4(iA[3]), iA1y = Load4(iA[4]), iA1z = Load4(iA[5]);
    const __m128 iA2x = Load4(iA[6]), iA2y = Load4(iA[7]), iA2z = Load4(iA[8]);
    const __m128 iB0x = Load4(iB[0]), iB0y = Load4(iB[1]), iB0z = Load4(iB[2]);
    const __m128 iB1x = Load4(iB[3]), iB1y = Load4(iB[4]), iB1z = Load4(iB[5]);
    const __m128 iB2x = Load4(iB[6]), iB2y = Load4(iB[7]), iB2z = Load4(iB[8]);

    const __m128 zero = _mm_setzero_ps();

    // Relative velocity at the contact: (vA + wA x rA) - (vB + wB x rB).
    auto relVel = [&](__m128& rx, __m128& ry, __m128& rz) {
        __m128 wax, way, waz, wbx, wby, wbz;
        Cross3(wAxV, wAyV, wAzV, rAxV, rAyV, rAzV, wax, way, waz);
        Cross3(wBxV, wByV, wBzV, rBxV, rByV, rBzV, wbx, wby, wbz);
        rx = _mm_sub_ps(_mm_add_ps(vAxV, wax), _mm_add_ps(vBxV, wbx));
        ry = _mm_sub_ps(_mm_add_ps(vAyV, way), _mm_add_ps(vByV, wby));
        rz = _mm_sub_ps(_mm_add_ps(vAzV, waz), _mm_add_ps(vBzV, wbz));
    };

    // Apply an impulse along (ix,iy,iz) to the two bodies.
    auto applyImpulse = [&](__m128 ix, __m128 iy, __m128 iz) {
        __m128 cx, cy, cz;
        // Body A.
        vAxV = _mm_add_ps(vAxV, _mm_mul_ps(ix, invMAV));
        vAyV = _mm_add_ps(vAyV, _mm_mul_ps(iy, invMAV));
        vAzV = _mm_add_ps(vAzV, _mm_mul_ps(iz, invMAV));
        Cross3(rAxV, rAyV, rAzV, ix, iy, iz, cx, cy, cz);
        __m128 wx, wy, wz;
        Mat3Mul(iA0x, iA0y, iA0z, iA1x, iA1y, iA1z, iA2x, iA2y, iA2z, cx, cy, cz, wx, wy, wz);
        wAxV = _mm_add_ps(wAxV, wx);
        wAyV = _mm_add_ps(wAyV, wy);
        wAzV = _mm_add_ps(wAzV, wz);
        // Body B (opposite impulse).
        vBxV = _mm_sub_ps(vBxV, _mm_mul_ps(ix, invMBV));
        vByV = _mm_sub_ps(vByV, _mm_mul_ps(iy, invMBV));
        vBzV = _mm_sub_ps(vBzV, _mm_mul_ps(iz, invMBV));
        Cross3(rBxV, rByV, rBzV, ix, iy, iz, cx, cy, cz);
        Mat3Mul(iB0x, iB0y, iB0z, iB1x, iB1y, iB1z, iB2x, iB2y, iB2z, cx, cy, cz, wx, wy, wz);
        wBxV = _mm_sub_ps(wBxV, wx);
        wByV = _mm_sub_ps(wByV, wy);
        wBzV = _mm_sub_ps(wBzV, wz);
    };

    // ---- Friction (two tangents), using the previous normal impulse -------
    for (int t = 0; t < 2; ++t) {
        const __m128 tx = (t == 0) ? t1xV : t2xV;
        const __m128 ty = (t == 0) ? t1yV : t2yV;
        const __m128 tz = (t == 0) ? t1zV : t2zV;
        const __m128 mass = (t == 0) ? t1MassV : t2MassV;
        __m128 accum = (t == 0) ? t1AccV : t2AccV;

        __m128 rx, ry, rz;
        relVel(rx, ry, rz);
        const __m128 vt = Dot3(rx, ry, rz, tx, ty, tz);
        const __m128 lambda = _mm_mul_ps(_mm_sub_ps(zero, mass), vt);
        const __m128 maxFric = _mm_mul_ps(fricV, nAccV);
        __m128 newAccum = _mm_add_ps(accum, lambda);
        newAccum = _mm_min_ps(_mm_max_ps(newAccum, _mm_sub_ps(zero, maxFric)), maxFric);
        const __m128 dLambda = _mm_sub_ps(newAccum, accum);
        accum = newAccum;

        applyImpulse(_mm_mul_ps(tx, dLambda), _mm_mul_ps(ty, dLambda), _mm_mul_ps(tz, dLambda));
        if (t == 0) t1AccV = accum; else t2AccV = accum;
    }

    // ---- Normal -----------------------------------------------------------
    {
        __m128 rx, ry, rz;
        relVel(rx, ry, rz);
        const __m128 vn = Dot3(rx, ry, rz, nxV, nyV, nzV);
        const __m128 lambda = _mm_mul_ps(_mm_sub_ps(zero, nMassV), _mm_sub_ps(vn, nBiasV));
        __m128 newAccum = _mm_max_ps(_mm_add_ps(nAccV, lambda), zero);
        const __m128 dLambda = _mm_sub_ps(newAccum, nAccV);
        nAccV = newAccum;
        applyImpulse(_mm_mul_ps(nxV, dLambda), _mm_mul_ps(nyV, dLambda), _mm_mul_ps(nzV, dLambda));
    }

    // ---- Scatter ----------------------------------------------------------
    Store4(vAx, vAxV); Store4(vAy, vAyV); Store4(vAz, vAzV);
    Store4(wAx, wAxV); Store4(wAy, wAyV); Store4(wAz, wAzV);
    Store4(vBx, vBxV); Store4(vBy, vByV); Store4(vBz, vBzV);
    Store4(wBx, wBxV); Store4(wBy, wByV); Store4(wBz, wBzV);
    Store4(nAcc, nAccV); Store4(t1Acc, t1AccV); Store4(t2Acc, t2AccV);

    for (u32 i = 0; i < count; ++i) {
        if (!bodyA[i] || !bodyB[i] || !pt[i]) continue;
        MotionProperties& ma = bodyA[i]->GetMotionProperties();
        MotionProperties& mb = bodyB[i]->GetMotionProperties();
        if (dynA[i]) {
            ma.linearVelocity = Vec3(vAx[i], vAy[i], vAz[i]);
            ma.angularVelocity = Vec3(wAx[i], wAy[i], wAz[i]);
        }
        if (dynB[i]) {
            mb.linearVelocity = Vec3(vBx[i], vBy[i], vBz[i]);
            mb.angularVelocity = Vec3(wBx[i], wBy[i], wBz[i]);
        }
        pt[i]->normalImpulse = nAcc[i];
        pt[i]->tangentImpulse1 = t1Acc[i];
        pt[i]->tangentImpulse2 = t2Acc[i];
    }
#endif // __SSE2__
}

} // namespace kizuri
