#include "draw.h"

#include <rlgl.h>

#include <cmath>

namespace ncotb::app {

void drawBody(const RigidBody& b, Color color) {
    // Column-major 4x4 transform from R and x.
    const Mat3& R = b.R();
    // clang-format off
    float m[16] = {float(R.m[0][0]), float(R.m[1][0]), float(R.m[2][0]), 0,
                   float(R.m[0][1]), float(R.m[1][1]), float(R.m[2][1]), 0,
                   float(R.m[0][2]), float(R.m[1][2]), float(R.m[2][2]), 0,
                   float(b.x.x),     float(b.x.y),     float(b.x.z),     1};
    // clang-format on
    rlPushMatrix();
    rlMultMatrixf(m);
    const Shape& s = b.shape();
    switch (s.type) {
    case Shape::Type::Box: {
        ::Vector3 size = toRl(2 * s.halfExtents);
        DrawCubeV({0, 0, 0}, size, color);
        DrawCubeWiresV({0, 0, 0}, size, BLACK);
        break;
    }
    case Shape::Type::Sphere:
        DrawSphereEx({0, 0, 0}, float(s.radius), 16, 16, color);
        DrawSphereWires({0, 0, 0}, float(s.radius) * 1.002f, 8, 8, BLACK);
        break;
    case Shape::Type::Plane: {
        // Infinite plane drawn as a 40 x 40 patch in the body's xz-plane.
        const float half = 20;
        DrawPlane({0, -0.001f, 0}, {2 * half, 2 * half}, color);
        for (float t = -half; t <= half; t += 1) {
            DrawLine3D({-half, 0, t}, {half, 0, t}, {0, 0, 0, 40});
            DrawLine3D({t, 0, -half}, {t, 0, half}, {0, 0, 0, 40});
        }
        break;
    }
    }
    rlPopMatrix();
}

void drawArrow(const Vec3& from, const Vec3& dir, Color color) {
    Vec3 to = from + dir;
    DrawLine3D(toRl(from), toRl(to), color);
    DrawSphere(toRl(to), 0.04f, color);
}

/// Contact points (red), normals from b towards a (orange) and penetration
/// depth drawn along the normal (dark red).
/// Contact forces on body a: normal (green) and friction (blue), as impulse /
/// step length, scaled to metres.
void drawContacts(const std::vector<ContactManifold>& contacts, bool normals, Real forceScale, Real h) {
    for (const ContactManifold& m : contacts) {
        for (int i = 0; i < m.count; ++i) {
            const ContactPoint& p = m.points[i];
            DrawSphere(toRl(p.position), 0.06f, RED);
            if (normals) drawArrow(p.position, m.normal * 0.5, ORANGE);
            Vec3 half = m.normal * (p.depth / 2);
            DrawLine3D(toRl(p.position - half), toRl(p.position + half), MAROON);
            if (forceScale > 0 && p.normalImpulse > 0)
                drawArrow(p.position, m.normal * (p.normalImpulse / h * forceScale), {0, 160, 60, 255});
            if (forceScale > 0 && lengthSquared(p.tangentImpulse) > 0)
                drawArrow(p.position, p.tangentImpulse * (forceScale / h), {30, 90, 230, 255});
        }
    }
}

/// Joints: anchors (dark), lines from the bodies' centres to them (grey), hinge
/// axes (blue) and slider axes (purple).
void drawJoints(const World& world) {
    for (const auto& jp : world.joints()) {
        const Joint& j = *jp;
        Vec3 pa = j.anchorA(), pb = j.anchorB();
        for (const RigidBody* b : {&j.bodyA(), &j.bodyB()})
            if (!b->isStatic()) DrawLine3D(toRl(b->x), toRl(b == &j.bodyA() ? pa : pb), {90, 90, 100, 255});
        DrawSphere(toRl(pa), 0.035f, {40, 40, 50, 255});
        if (auto* h = dynamic_cast<const HingeJoint*>(&j)) {
            Vec3 n = h->axis() * 0.3;
            DrawLine3D(toRl(pa - n), toRl(pa + n), {40, 90, 220, 255});
        } else if (auto* s = dynamic_cast<const SliderJoint*>(&j)) {
            Vec3 n = s->axis();
            Real lo = s->hasLimits() ? s->lowerLimit() : -1, hi = s->hasLimits() ? s->upperLimit() : 1;
            DrawLine3D(toRl(pa + n * lo), toRl(pa + n * hi), {150, 70, 200, 255});
        }
    }
}

/// Springs as coils: 10 turns around the line between the anchors.
void drawSprings(const World& world) {
    for (const Spring& sp : world.springs()) {
        Vec3 pa = sp.anchorA(), pb = sp.anchorB(), d = pb - pa;
        Real len = length(d);
        if (len < 1e-6) continue;
        Vec3 n = d / len;
        Vec3 u = anyPerpendicular(n), v = cross(n, u);
        const int segments = 120;
        const Real radius = 0.06;
        Vec3 prev = pa;
        for (int i = 1; i <= segments; ++i) {
            Real t = Real(i) / segments, phi = t * 10 * 2 * M_PI;
            Real r = (i < 4 || i > segments - 4) ? 0 : radius; // straight ends
            Vec3 p = pa + d * t + (u * std::cos(phi) + v * std::sin(phi)) * r;
            DrawLine3D(toRl(prev), toRl(p), {60, 60, 70, 255});
            prev = p;
        }
    }
}

std::uintptr_t packColor(int r, int g, int b) {
    return (std::uintptr_t(r) << 24) | (std::uintptr_t(g) << 16) | (std::uintptr_t(b) << 8) | 0xff;
}

Color colorOf(const RigidBody& b, Color fallback) {
    if (b.userData == 0) return fallback;
    auto c = std::uint32_t(b.userData);
    return {(unsigned char)(c >> 24), (unsigned char)(c >> 16), (unsigned char)(c >> 8), (unsigned char)c};
}

} // namespace ncotb::app
