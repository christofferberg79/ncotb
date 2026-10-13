#pragma once

#include "world.h"

#include <raylib.h>

#include <cstdint>
#include <vector>

namespace ncotb::app {

inline ::Vector3 toRl(const Vec3& v) { return {float(v.x), float(v.y), float(v.z)}; }

void drawBody(const RigidBody& b, Color color);
/// Frees the meshes of terrain no body uses any more (call once per frame).
void releaseUnusedTerrainMeshes();
/// A slightly enlarged wireframe around the body (highlight).
void drawOutline(const RigidBody& b, Color color);
void drawArrow(const Vec3& from, const Vec3& dir, Color color);
void drawContacts(const std::vector<ContactManifold>& contacts, bool normals, Real forceScale, Real h);
/// Joints: rods always; with details also anchors and hinge / slider axes.
void drawJoints(const World& world, bool details = true);
/// A wheel body drawn as a tyre: a cylinder about axle (unit, world space)
/// with tread lines that turn with the wheel.
void drawTyre(const RigidBody& wheel, const Vec3& axle, Real radius, Real width);
void drawSprings(const World& world);

/// Body colours are packed into RigidBody::userData as 0xRRGGBBAA.
std::uintptr_t packColor(int r, int g, int b);
Color colorOf(const RigidBody& b, Color fallback);

} // namespace ncotb::app
