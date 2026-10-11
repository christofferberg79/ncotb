#pragma once

#include "world.h"

#include <raylib.h>

#include <cstdint>
#include <vector>

namespace ncotb::app {

inline ::Vector3 toRl(const Vec3& v) { return {float(v.x), float(v.y), float(v.z)}; }

void drawBody(const RigidBody& b, Color color);
void drawArrow(const Vec3& from, const Vec3& dir, Color color);
void drawContacts(const std::vector<ContactManifold>& contacts, bool normals, Real forceScale, Real h);
void drawJoints(const World& world);
void drawSprings(const World& world);

/// Body colours are packed into RigidBody::userData as 0xRRGGBBAA.
std::uintptr_t packColor(int r, int g, int b);
Color colorOf(const RigidBody& b, Color fallback);

} // namespace ncotb::app
