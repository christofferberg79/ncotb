#pragma once

#include "math/mat3.h"

namespace ncotb {

/// Geometric shape of a rigid body, in body space (centred on the centre of mass).
struct Shape {
    enum class Type { Sphere, Box };

    Type type = Type::Sphere;
    Real radius = 0.5;     ///< Sphere
    Vec3 halfExtents{};    ///< Box

    static Shape sphere(Real r) { return {Type::Sphere, r, {}}; }
    static Shape box(const Vec3& halfExtents) { return {Type::Box, 0, halfExtents}; }

    /// Inertia tensor in body space for a solid body of the given mass.
    Mat3 inertia(Real mass) const {
        if (type == Type::Sphere) {
            Real i = Real(2) / 5 * mass * radius * radius;
            return Mat3::diagonal({i, i, i});
        }
        Vec3 d = 2 * halfExtents; // full side lengths
        Real k = mass / 12;
        return Mat3::diagonal({k * (d.y * d.y + d.z * d.z),
                               k * (d.x * d.x + d.z * d.z),
                               k * (d.x * d.x + d.y * d.y)});
    }
};

} // namespace ncotb
