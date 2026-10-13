#pragma once

#include "body/heightfield.h"
#include "math/mat3.h"

#include <memory>

namespace ncotb {

/// Geometric shape of a rigid body, in body space (centred on the centre of mass).
struct Shape {
    /// Plane: infinite, through the body position, normal along the body's +y axis.
    /// Heightfield: terrain over the body's xz-plane (see Heightfield).
    /// Planes and heightfields are only meaningful for static bodies (mass 0).
    enum class Type { Sphere, Box, Plane, Heightfield };

    Type type = Type::Sphere;
    Real radius = 0.5;  ///< Sphere
    Vec3 halfExtents{}; ///< Box
    /// Heightfield (shared, since bodies copy their shape).
    std::shared_ptr<const Heightfield> terrain;

    static Shape sphere(Real r) {
        Shape s;
        s.radius = r;
        return s;
    }
    static Shape box(const Vec3& halfExtents) {
        Shape s;
        s.type = Type::Box;
        s.halfExtents = halfExtents;
        return s;
    }
    static Shape plane() {
        Shape s;
        s.type = Type::Plane;
        return s;
    }
    static Shape heightfield(std::shared_ptr<const Heightfield> terrain) {
        Shape s;
        s.type = Type::Heightfield;
        s.terrain = std::move(terrain);
        return s;
    }

    /// Radius of a sphere around the centre that contains the shape (infinite for a plane).
    Real boundingRadius() const {
        switch (type) {
        case Type::Sphere: return radius;
        case Type::Box: return length(halfExtents);
        case Type::Plane:
        case Type::Heightfield: return INFINITY;
        }
        return INFINITY;
    }

    /// Inertia tensor in body space for a solid body of the given mass.
    Mat3 inertia(Real mass) const {
        if (type == Type::Plane || type == Type::Heightfield) return {};
        if (type == Type::Sphere) {
            Real i = Real(2) / 5 * mass * radius * radius;
            return Mat3::diagonal({i, i, i});
        }
        Vec3 d = 2 * halfExtents; // full side lengths
        Real k = mass / 12;
        return Mat3::diagonal(
            {k * (d.y * d.y + d.z * d.z), k * (d.x * d.x + d.z * d.z), k * (d.x * d.x + d.y * d.y)});
    }
};

} // namespace ncotb
