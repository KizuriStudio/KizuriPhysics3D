// KizuriPhysics - Debug/DebugRenderer.h
// Renderer-agnostic debug drawing interface.
//
// The physics engine never talks to a graphics API directly.  Instead it
// calls into a DebugRenderer implementation provided by the host (OpenGL,
// Vulkan, DirectX, ImGui, a software rasteriser, ...).  This keeps the
// library free of rendering dependencies while still allowing full
// visualisation of bodies, contacts, joints and queries.
#pragma once

#include "Kizuri/Core/Math.h"

namespace kizuri {

/// Colour in linear RGBA, components in [0, 1].
struct DebugColor {
    Real r = 1, g = 1, b = 1, a = 1;

    DebugColor() = default;
    DebugColor(Real rr, Real gg, Real bb, Real aa = Real(1)) : r(rr), g(gg), b(bb), a(aa) {}

    static DebugColor Red()      { return { 1.0f, 0.15f, 0.15f }; }
    static DebugColor Green()    { return { 0.15f, 1.0f, 0.15f }; }
    static DebugColor Blue()     { return { 0.2f, 0.4f, 1.0f }; }
    static DebugColor Yellow()   { return { 1.0f, 0.9f, 0.1f }; }
    static DebugColor Cyan()     { return { 0.1f, 0.9f, 0.9f }; }
    static DebugColor Magenta()  { return { 1.0f, 0.2f, 0.9f }; }
    static DebugColor White()    { return { 1.0f, 1.0f, 1.0f }; }
    static DebugColor Orange()   { return { 1.0f, 0.55f, 0.1f }; }
    static DebugColor Gray()     { return { 0.5f, 0.5f, 0.5f }; }
};

/// Abstract debug renderer.  Implement the four primitive calls and the
/// engine will draw everything it knows about.
class DebugRenderer {
public:
    virtual ~DebugRenderer() = default;

    /// Draw a single line segment in world space.
    virtual void DrawLine(const Vec3& from, const Vec3& to, const DebugColor& color) = 0;

    /// Draw a wireframe triangle in world space.
    virtual void DrawTriangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, const DebugColor& color) = 0;

    /// Draw an oriented wireframe box.
    virtual void DrawBox(const Vec3& center, const Vec3& halfExtent, const Quat& rotation,
                         const DebugColor& color) {
        const Vec3 c = center;
        const Vec3 e = halfExtent;
        Vec3 corners[8];
        for (int i = 0; i < 8; ++i) {
            Vec3 local((i & 1) ? e.x : -e.x, (i & 2) ? e.y : -e.y, (i & 4) ? e.z : -e.z);
            corners[i] = c + rotation.Rotate(local);
        }
        static const int edges[12][2] = {
            {0,1},{1,3},{3,2},{2,0}, {4,5},{5,7},{7,6},{6,4}, {0,4},{1,5},{2,6},{3,7}
        };
        for (auto& ed : edges) DrawLine(corners[ed[0]], corners[ed[1]], color);
    }

    /// Draw a wireframe sphere.
    virtual void DrawSphere(const Vec3& center, Real radius, const DebugColor& color, u32 segments = 16) {
        for (u32 i = 0; i < segments; ++i) {
            const Real a0 = Real(2.0 * 3.14159265358979) * Real(i) / Real(segments);
            const Real a1 = Real(2.0 * 3.14159265358979) * Real(i + 1) / Real(segments);
            // Three great circles.
            DrawLine(center + Vec3(radius * math::Cos(a0), radius * math::Sin(a0), 0),
                     center + Vec3(radius * math::Cos(a1), radius * math::Sin(a1), 0), color);
            DrawLine(center + Vec3(0, radius * math::Cos(a0), radius * math::Sin(a0)),
                     center + Vec3(0, radius * math::Cos(a1), radius * math::Sin(a1)), color);
            DrawLine(center + Vec3(radius * math::Sin(a0), 0, radius * math::Cos(a0)),
                     center + Vec3(radius * math::Sin(a1), 0, radius * math::Cos(a1)), color);
        }
    }

    /// Draw a world-space arrow (used for normals and velocities).
    virtual void DrawArrow(const Vec3& from, const Vec3& to, const DebugColor& color) {
        DrawLine(from, to, color);
        Vec3 d = to - from;
        Real len = d.Length();
        if (len < Real(1.0e-6)) return;
        d /= len;
        Vec3 u, v;
        d.GetBasis(u, v);
        const Real head = len * Real(0.15);
        const Real w = len * Real(0.06);
        DrawLine(to, to - d * head + u * w, color);
        DrawLine(to, to - d * head - u * w, color);
        DrawLine(to, to - d * head + v * w, color);
        DrawLine(to, to - d * head - v * w, color);
    }
};

} // namespace kizuri
