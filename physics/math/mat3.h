#pragma once

#include "vec3.h"

namespace ncotb {

/// 3x3 matrix stored row-major: m[row][col].
struct Mat3 {
    Real m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    constexpr Mat3() = default;
    constexpr Mat3(Real a00, Real a01, Real a02,
                   Real a10, Real a11, Real a12,
                   Real a20, Real a21, Real a22)
        : m{{a00, a01, a02}, {a10, a11, a12}, {a20, a21, a22}} {}

    static constexpr Mat3 identity() { return {1, 0, 0, 0, 1, 0, 0, 0, 1}; }
    static constexpr Mat3 diagonal(const Vec3& d) { return {d.x, 0, 0, 0, d.y, 0, 0, 0, d.z}; }

    /// Matrix from columns (e.g. the body axes of a rotation matrix).
    static constexpr Mat3 fromColumns(const Vec3& c0, const Vec3& c1, const Vec3& c2) {
        return {c0.x, c1.x, c2.x, c0.y, c1.y, c2.y, c0.z, c1.z, c2.z};
    }

    /// The "star" matrix a* such that a* b = a x b (Baraff, Rigid Body Simulation I).
    static constexpr Mat3 star(const Vec3& a) { return {0, -a.z, a.y, a.z, 0, -a.x, -a.y, a.x, 0}; }

    constexpr Vec3 row(int r) const { return {m[r][0], m[r][1], m[r][2]}; }
    constexpr Vec3 column(int c) const { return {m[0][c], m[1][c], m[2][c]}; }

    constexpr Mat3& operator+=(const Mat3& b) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) m[i][j] += b.m[i][j];
        return *this;
    }
    constexpr Mat3& operator-=(const Mat3& b) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) m[i][j] -= b.m[i][j];
        return *this;
    }
    constexpr Mat3& operator*=(Real s) {
        for (auto& r : m)
            for (auto& v : r) v *= s;
        return *this;
    }
};

constexpr Mat3 operator+(Mat3 a, const Mat3& b) { return a += b; }
constexpr Mat3 operator-(Mat3 a, const Mat3& b) { return a -= b; }
constexpr Mat3 operator*(Mat3 a, Real s) { return a *= s; }
constexpr Mat3 operator*(Real s, Mat3 a) { return a *= s; }

constexpr Vec3 operator*(const Mat3& a, const Vec3& v) {
    return {dot(a.row(0), v), dot(a.row(1), v), dot(a.row(2), v)};
}

constexpr Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}

constexpr Mat3 transpose(const Mat3& a) {
    return {a.m[0][0], a.m[1][0], a.m[2][0],
            a.m[0][1], a.m[1][1], a.m[2][1],
            a.m[0][2], a.m[1][2], a.m[2][2]};
}

constexpr Real determinant(const Mat3& a) { return dot(a.row(0), cross(a.row(1), a.row(2))); }

/// Inverse via the adjugate. Returns the zero matrix if singular.
constexpr Mat3 inverse(const Mat3& a) {
    Vec3 r0 = a.row(0), r1 = a.row(1), r2 = a.row(2);
    Vec3 c0 = cross(r1, r2), c1 = cross(r2, r0), c2 = cross(r0, r1);
    Real det = dot(r0, c0);
    if (det == 0) return {};
    return Mat3::fromColumns(c0, c1, c2) * (1 / det);
}

} // namespace ncotb
