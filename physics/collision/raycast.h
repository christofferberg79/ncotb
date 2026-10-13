#pragma once

#include "math/vec3.h"

#include <optional>

namespace ncotb {

class RigidBody;

/// Where a ray first meets a body.
struct RayHit {
    RigidBody* body = nullptr;
    Vec3 point{};      ///< world space
    Vec3 normal{};     ///< outward surface normal at the point (or -dir if the ray starts inside)
    Real distance = 0; ///< along the ray from its origin (0 if it starts inside)
};

/// Ray origin + t dir (dir a unit vector, t >= 0) against one body's shape:
/// sphere, box (slab test in box coordinates), plane (a half-space) or
/// heightfield (its triangles, hit from above).
std::optional<RayHit> raycast(RigidBody& body, const Vec3& origin, const Vec3& dir);

} // namespace ncotb
