#include <doctest/doctest.h>

#include "math/quat.h"

using namespace ncotb;

namespace {
bool near(const Vec3& a, const Vec3& b, Real eps = 1e-12) { return length(a - b) < eps; }
}

TEST_CASE("cross product follows the right-hand rule") {
    CHECK(near(cross({1, 0, 0}, {0, 1, 0}), {0, 0, 1}));
    CHECK(near(Mat3::star({1, 2, 3}) * Vec3{4, 5, 6}, cross({1, 2, 3}, {4, 5, 6})));
}

TEST_CASE("matrix inverse") {
    Mat3 a{2, 1, 0, 1, 3, 1, 0, 1, 4};
    Mat3 p = a * inverse(a);
    for (int i = 0; i < 3; ++i)
        CHECK(near(p.row(i), Mat3::identity().row(i)));
}

TEST_CASE("quaternion rotation matches rotation matrix") {
    Quat q = Quat::fromAxisAngle({1, 2, 3}, 0.7);
    Vec3 p{0.3, -1.2, 2.0};
    CHECK(near(rotate(q, p), toMat3(q) * p));
    CHECK(doctest::Approx(determinant(toMat3(q))) == 1.0);
    // 90 degrees about z takes x to y
    CHECK(near(rotate(Quat::fromAxisAngle({0, 0, 1}, M_PI / 2), {1, 0, 0}), {0, 1, 0}));
}

TEST_CASE("quaternion product composes rotations") {
    Quat a = Quat::fromAxisAngle({0, 1, 0}, 0.4);
    Quat b = Quat::fromAxisAngle({1, 0, 1}, -1.1);
    Vec3 p{1, 2, 3};
    CHECK(near(rotate(a * b, p), rotate(a, rotate(b, p))));
}
