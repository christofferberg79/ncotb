#pragma once

// Shared between the collision files; not part of the library's interface.

#include "collision/collide.h"

#include <array>
#include <vector>

namespace ncotb::detail {

/// Picks at most four points that keep the contact area as large as possible.
void reduceToFour(std::vector<ContactPoint>& pts, const Vec3& n);
/// World-space corners of a box.
std::array<Vec3, 8> boxCorners(const RigidBody& box);

/// Contacts of a sphere / box (a in the manifolds) with a heightfield (b).
void sphereTerrain(RigidBody& sphere, RigidBody& terrain, std::vector<ContactManifold>& out, Real margin);
void boxTerrain(RigidBody& box, RigidBody& terrain, std::vector<ContactManifold>& out, Real margin);

} // namespace ncotb::detail
