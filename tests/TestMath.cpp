// KizuriPhysics - tests/TestMath.cpp
#include "TestFramework.h"
#include "Kizuri/Core/Math.h"

using namespace kizuri;

KZ_TEST(Vec3_BasicOps) {
    Vec3 a(1, 2, 3);
    Vec3 b(4, 5, 6);
    CHECK_NEAR(a.Dot(b), 32.0, 1e-6);
    Vec3 c = a.Cross(b);
    CHECK_NEAR(c.x, -3.0, 1e-6);
    CHECK_NEAR(c.y, 6.0, 1e-6);
    CHECK_NEAR(c.z, -3.0, 1e-6);
    CHECK_NEAR(a.LengthSq(), 14.0, 1e-6);
    Vec3 n = a.Normalized();
    CHECK_NEAR(n.Length(), 1.0, 1e-5);
}

KZ_TEST(Vec3_OrthonormalBasis) {
    Vec3 v = Vec3(0.3, -0.7, 0.2).Normalized();
    Vec3 b, c;
    v.GetBasis(b, c);
    CHECK_NEAR(v.Dot(b), 0.0, 1e-5);
    CHECK_NEAR(v.Dot(c), 0.0, 1e-5);
    CHECK_NEAR(b.Dot(c), 0.0, 1e-5);
    CHECK_NEAR(b.Length(), 1.0, 1e-5);
    CHECK_NEAR(c.Length(), 1.0, 1e-5);
}

KZ_TEST(Quat_Rotate) {
    Quat q = Quat::AxisAngle(Vec3::UnitZ(), math::kHalfPi);
    Vec3 r = q.Rotate(Vec3::UnitX());
    CHECK_NEAR(r.x, 0.0, 1e-5);
    CHECK_NEAR(r.y, 1.0, 1e-5);
    CHECK_NEAR(r.z, 0.0, 1e-5);
    // Inverse rotation restores the original.
    Vec3 back = q.InverseRotate(r);
    CHECK_NEAR(back.x, 1.0, 1e-5);
}

KZ_TEST(Quat_MatrixRoundTrip) {
    Quat q = Quat::FromEuler(Vec3(0.3, -0.4, 0.9)).Normalized();
    Mat3 m = q.ToMat3();
    Quat q2 = Quat::FromMat3(m);
    // q and q2 may differ by sign; compare the rotation matrices.
    Mat3 m2 = q2.ToMat3();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(m[i][j], m2[i][j], 1e-5);
}

KZ_TEST(Mat3_Inverse) {
    Mat3 m(Vec3(2, 0, 0), Vec3(0, 3, 0), Vec3(1, 1, 4));
    Mat3 inv = m.Inversed();
    Mat3 id = m * inv;
    CHECK_NEAR(id[0].x, 1.0, 1e-5);
    CHECK_NEAR(id[1].y, 1.0, 1e-5);
    CHECK_NEAR(id[2].z, 1.0, 1e-5);
    CHECK_NEAR(id[0].y, 0.0, 1e-5);
}

KZ_TEST(Transform_Compose) {
    Transform a(Quat::AxisAngle(Vec3::UnitY(), math::kHalfPi), Vec3(1, 0, 0));
    Transform b(Quat::Identity(), Vec3(0, 2, 0));
    Transform c = a * b;
    // Rotating b's translation (0,2,0) by 90deg about Y gives (0,2,0).
    CHECK_NEAR(c.translation.x, 1.0, 1e-5);
    CHECK_NEAR(c.translation.y, 2.0, 1e-5);
    // Inverse should undo the point transform.
    Vec3 p(5, 6, 7);
    Vec3 tp = c * p;
    Vec3 back = c.Inverse() * tp;
    CHECK_NEAR(back.x, p.x, 1e-4);
    CHECK_NEAR(back.y, p.y, 1e-4);
    CHECK_NEAR(back.z, p.z, 1e-4);
}

KZ_TEST(AABB_Ops) {
    AABB a(Vec3(0, 0, 0), Vec3(1, 1, 1));
    AABB b(Vec3(0.5, 0.5, 0.5), Vec3(2, 2, 2));
    CHECK(a.Overlaps(b));
    CHECK(a.Contains(Vec3(0.5, 0.5, 0.5)));
    AABB m = Merge(a, b);
    CHECK_NEAR(m.min.x, 0.0, 1e-6);
    CHECK_NEAR(m.max.x, 2.0, 1e-6);
    CHECK_NEAR(a.SurfaceArea(), 6.0, 1e-6);
    CHECK_NEAR(a.Volume(), 1.0, 1e-6);
}

KZ_TEST(AABB_Transformed) {
    AABB box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    Transform t(Quat::AxisAngle(Vec3::UnitZ(), math::kHalfPi), Vec3(5, 0, 0));
    AABB r = box.Transformed(t);
    CHECK_NEAR(r.Center().x, 5.0, 1e-5);
    CHECK_NEAR(r.Extent().x, 1.0, 1e-5);
    CHECK_NEAR(r.Extent().y, 1.0, 1e-5);
}
