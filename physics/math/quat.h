#pragma once

#include "mat3.h"

namespace ncotb {

/// Quaternion q = [s, v] (Baraff's notation). Unit quaternions represent rotations.
struct Quat {
    Real s = 1;
    Vec3 v{};

    constexpr Quat() = default;
    constexpr Quat(Real s_, const Vec3& v_) : s(s_), v(v_) {}

    static Quat fromAxisAngle(const Vec3& axis, Real angle) {
        return {std::cos(angle / 2), normalize(axis) * std::sin(angle / 2)};
    }

    /// Rotation by the vector theta: angle |theta| about theta/|theta| (the exponential map).
    static Quat fromRotationVector(const Vec3& theta) {
        Real angle = length(theta);
        // sin(a/2)/a, with a Taylor expansion near zero to avoid 0/0
        Real k = angle < 1e-6 ? Real(0.5) - angle * angle / 48 : std::sin(angle / 2) / angle;
        return {std::cos(angle / 2), theta * k};
    }

    constexpr Quat& operator+=(const Quat& q) {
        s += q.s;
        v += q.v;
        return *this;
    }
    constexpr Quat& operator*=(Real k) {
        s *= k;
        v *= k;
        return *this;
    }
};

constexpr Quat operator+(Quat a, const Quat& b) { return a += b; }
constexpr Quat operator*(Quat a, Real k) { return a *= k; }
constexpr Quat operator*(Real k, Quat a) { return a *= k; }

/// Quaternion product [s1,v1][s2,v2] = [s1 s2 - v1.v2, s1 v2 + s2 v1 + v1 x v2].
constexpr Quat operator*(const Quat& a, const Quat& b) {
    return {a.s * b.s - dot(a.v, b.v), a.s * b.v + b.s * a.v + cross(a.v, b.v)};
}

constexpr Quat conjugate(const Quat& q) { return {q.s, -q.v}; }
constexpr Real dot(const Quat& a, const Quat& b) { return a.s * b.s + dot(a.v, b.v); }

inline Quat normalize(const Quat& q) {
    Real len = std::sqrt(dot(q, q));
    return len > 0 ? q * (1 / len) : Quat{};
}

/// Rotate vector by unit quaternion: q [0,p] q*.
constexpr Vec3 rotate(const Quat& q, const Vec3& p) {
    Vec3 t = 2 * cross(q.v, p);
    return p + q.s * t + cross(q.v, t);
}

// clang-format off
/// Rotation matrix of a unit quaternion (Baraff, Rigid Body Simulation I, eq. 4-2).
constexpr Mat3 toMat3(const Quat& q) {
    Real s = q.s, x = q.v.x, y = q.v.y, z = q.v.z;
    return {1 - 2 * (y * y + z * z), 2 * (x * y - s * z),     2 * (x * z + s * y),
            2 * (x * y + s * z),     1 - 2 * (x * x + z * z), 2 * (y * z - s * x),
            2 * (x * z - s * y),     2 * (y * z + s * x),     1 - 2 * (x * x + y * y)};
}
// clang-format on

} // namespace ncotb
