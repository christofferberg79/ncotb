#pragma once

#include <cmath>

namespace ncotb {

using Real = double;

struct Vec3 {
    Real x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}

    constexpr Real& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    constexpr Real operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }

    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    constexpr Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
    constexpr Vec3& operator/=(Real s) { x /= s; y /= s; z /= s; return *this; }
};

constexpr Vec3 operator+(Vec3 a, const Vec3& b) { return a += b; }
constexpr Vec3 operator-(Vec3 a, const Vec3& b) { return a -= b; }
constexpr Vec3 operator*(Vec3 a, Real s) { return a *= s; }
constexpr Vec3 operator*(Real s, Vec3 a) { return a *= s; }
constexpr Vec3 operator/(Vec3 a, Real s) { return a /= s; }

constexpr Real dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// Component-wise product.
constexpr Vec3 hadamard(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }

constexpr Real lengthSquared(const Vec3& v) { return dot(v, v); }
inline Real length(const Vec3& v) { return std::sqrt(lengthSquared(v)); }

inline Vec3 normalize(const Vec3& v) {
    Real len = length(v);
    return len > 0 ? v / len : Vec3{};
}

} // namespace ncotb
