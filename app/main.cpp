// ncotb — rigid body sandbox (raylib + Dear ImGui front-end for the physics library).

#include "world.h"

#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>
#include <rlgl.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <functional>
#include <optional>
#include <random>
#include <string>
#include <vector>

using namespace ncotb;

namespace {

::Vector3 toRl(const Vec3& v) { return {float(v.x), float(v.y), float(v.z)}; }

// ---------------------------------------------------------------- camera

struct OrbitCamera {
    float yaw = 0.6f, pitch = 0.35f, distance = 12.0f;
    ::Vector3 target{0, 1, 0};

    void update(bool mouseFree) {
        if (mouseFree) {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                ::Vector2 d = GetMouseDelta();
                yaw -= d.x * 0.005f;
                pitch = std::clamp(pitch + d.y * 0.005f, -1.5f, 1.5f);
            }
            distance = std::clamp(distance * (1.0f - GetMouseWheelMove() * 0.1f), 1.0f, 200.0f);
        }
    }

    Camera3D camera() const {
        Camera3D c{};
        c.target = target;
        c.position = {target.x + distance * std::cos(pitch) * std::sin(yaw),
                      target.y + distance * std::sin(pitch),
                      target.z + distance * std::cos(pitch) * std::cos(yaw)};
        c.up = {0, 1, 0};
        c.fovy = 45;
        c.projection = CAMERA_PERSPECTIVE;
        return c;
    }
};

// ---------------------------------------------------------------- drawing

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
        Vec3 u = normalize(cross(n, std::abs(n.x) < 0.57 ? Vec3{1, 0, 0} : Vec3{0, 1, 0})), v = cross(n, u);
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

// ---------------------------------------------------------------- scenes

struct Scene {
    const char* name;
    /// Contact scenes use semi-implicit Euler (the only integrator that resolves
    /// contacts) and show the contact solver settings.
    bool contacts;
    std::function<void(World&)> build;
    // Optional:
    int substeps = 0;                    ///< substeps for this scene (0: the slider's value)
    std::optional<OrbitCamera> camera{}; ///< camera to start from
    bool showVectors = true;             ///< default for the omega/L arrows
    bool showContacts = true;            ///< default for the contact overlay
    const char* actionLabel = nullptr;   ///< e.g. "Fire": a button and the F key
    std::function<void(World&)> action{};
    Real autoActionAt = -1;               ///< run the action once at this time (s)
    std::function<void()> decorate{};     ///< extra drawing (in 3D mode), not physics
    std::function<void(World&)> update{}; ///< called after every step (scripted motors etc.)
};

/// Body colours are packed into RigidBody::userData as 0xRRGGBBAA.
std::uintptr_t packColor(int r, int g, int b) {
    return (std::uintptr_t(r) << 24) | (std::uintptr_t(g) << 16) | (std::uintptr_t(b) << 8) | 0xff;
}

Color colorOf(const RigidBody& b, Color fallback) {
    if (b.userData == 0) return fallback;
    auto c = std::uint32_t(b.userData);
    return {(unsigned char)(c >> 24), (unsigned char)(c >> 16), (unsigned char)(c >> 8), (unsigned char)c};
}

// ---------------------------------------------------------------- cannon scene

/// The cannon fires from the muzzle at kCannonSpeed (horizontal), aimed so the
/// ball's arc passes through kCannonTarget on the wall.
constexpr Vec3 kMuzzle{0.3, 1.05, 13.0};
constexpr Vec3 kCannonTarget{0.3, 1.7, 0};
constexpr Real kCannonSpeed = 25;

Vec3 cannonVelocity() {
    Vec3 d = kCannonTarget - kMuzzle;
    Real horizontal = std::hypot(d.x, d.z);
    Real t = horizontal / kCannonSpeed;
    Real vy = (d.y + 0.5 * 9.81 * t * t) / t; // rise so gravity brings it down onto the target
    return {d.x / t, vy, d.z / t};
}

void drawCannon() {
    const Vec3 dir = normalize(cannonVelocity());
    const Vec3 breech = kMuzzle - dir * 2.4;
    const Color iron{45, 47, 52, 255}, wood{110, 70, 40, 255};
    DrawCylinderEx(toRl(breech), toRl(kMuzzle), 0.5f, 0.4f, 24, iron);
    DrawCylinderWiresEx(toRl(kMuzzle - dir * 0.15), toRl(kMuzzle), 0.44f, 0.44f, 24, {20, 20, 22, 255});
    DrawSphere(toRl(breech), 0.5f, iron);
    // carriage and wheels
    Vec3 axle = breech + dir * 0.9;
    DrawCube({float(axle.x), 0.45f, float(axle.z)}, 0.8f, 0.3f, 2.2f, wood);
    for (int side : {-1, 1}) {
        ::Vector3 c{float(axle.x + side * 0.55), 0.6f, float(axle.z)};
        ::Vector3 c2{c.x + side * 0.14f, c.y, c.z};
        DrawCylinderEx(c, c2, 0.6f, 0.6f, 24, wood);
        DrawCylinderWiresEx(c, c2, 0.6f, 0.6f, 12, {60, 35, 20, 255});
    }
}

void fireCannon(World& w) {
    RigidBody& ball = w.addBody(Shape::sphere(0.35), 30); // cast iron, about 30 kg
    ball.x = kMuzzle;
    ball.restitution = 0.2;
    ball.friction = 0.4;
    ball.userData = packColor(40, 42, 48);
    ball.updateDerived();
    ball.setLinearVelocity(cannonVelocity());
}

/// A wall in running bond: every other row is offset by half a brick, with half
/// bricks at its ends.
void buildBrickWall(World& w) {
    w.gravity = {0, -9.81, 0};
    RigidBody& ground = w.addBody(Shape::plane(), 0);
    ground.friction = 0.8;
    ground.restitution = 0.1;
    const int cols = 10, rows = 14;
    const Real L = 0.6, H = 0.24, D = 0.3, mass = 2.0;
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> shade(-18, 18);
    for (int r = 0; r < rows; ++r) {
        const bool offset = r % 2 == 1;
        const int n = offset ? cols + 1 : cols;
        Real x = -cols * L / 2;
        for (int i = 0; i < n; ++i) {
            const Real len = offset && (i == 0 || i == n - 1) ? L / 2 : L;
            RigidBody& b = w.addBody(Shape::box({len / 2, H / 2, D / 2}), mass * len / L);
            b.x = {x + len / 2, H / 2 + r * H, 0};
            b.friction = 0.7;
            b.restitution = 0.05;
            int s = shade(rng);
            b.userData = packColor(160 + s, 62 + s / 2, 45 + s / 3);
            b.updateDerived();
            x += len;
        }
    }
}

/// Adds a dynamic body at x with orientation q.
RigidBody& place(World& w, const Shape& shape, Real mass, const Vec3& x, const Quat& q = {}) {
    RigidBody& b = w.addBody(shape, mass);
    b.x = x;
    b.q = q;
    b.updateDerived();
    return b;
}

const std::vector<Scene>& scenes() {
    static const std::vector<Scene> list = {
        {"Intermediate axis (Dzhanibekov)", false,
         [](World& w) {
             w.gravity = {};
             RigidBody& b = w.addBody(Shape::box({1.2, 0.6, 0.2}), 1.0);
             b.x = {0, 2, 0};
             b.setAngularVelocity({0.01, 5.0, 0.01});
         }},
        {"Three principal axes", false,
         [](World& w) {
             w.gravity = {};
             const Vec3 spins[3] = {{6, 0.05, 0.05}, {0.05, 6, 0.05}, {0.05, 0.05, 6}};
             for (int i = 0; i < 3; ++i) {
                 RigidBody& b = w.addBody(Shape::box({1.0, 0.5, 0.15}), 1.0);
                 b.x = {(i - 1) * 3.5, 2, 0};
                 b.setAngularVelocity(spins[i]);
             }
         }},
        {"Thrown boxes and balls", false,
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             for (int i = 0; i < 6; ++i) {
                 bool box = i % 2 == 0;
                 RigidBody& b = w.addBody(box ? Shape::box({0.4, 0.25, 0.15}) : Shape::sphere(0.3), 1.0);
                 b.x = {-6, 1 + 0.3 * i, (i - 2.5) * 1.2};
                 b.setLinearVelocity({5 + 0.4 * i, 9, 0});
                 b.setAngularVelocity({1.0 * i, 3, 2});
             }
         }},
        {"Collision test (move bodies in the panel)", true,
         [](World& w) {
             // Bodies start just touching: overlapping starts have no clean
             // answer (see docs/notes.md) and are best avoided.
             w.gravity = {};
             w.addBody(Shape::plane(), 0); // ground
             RigidBody& ramp = w.addBody(Shape::plane(), 0);
             ramp.x = {6, 0, 0};
             ramp.q = Quat::fromAxisAngle({0, 0, 1}, 0.5);
             ramp.updateDerived();
             place(w, Shape::box({1, 0.5, 1}), 0, {-3, 0.5, 0});
             place(w, Shape::box({0.5, 0.5, 0.5}), 1, {-3, 1.5, 0.3}, Quat::fromAxisAngle({0, 1, 0}, 0.4));
             place(w, Shape::sphere(0.5), 1, {0, 0.5, 0});
             // Tilted box with its lowest corner on the ground.
             RigidBody& b2 =
                 place(w, Shape::box({0.4, 0.3, 0.6}), 1, {1.4, 0, 0}, Quat::fromAxisAngle({1, 0, 1}, 0.6));
             Real lowest = 0;
             for (int k = 0; k < 8; ++k) {
                 Vec3 c{k & 1 ? 0.4 : -0.4, k & 2 ? 0.3 : -0.3, k & 4 ? 0.6 : -0.6};
                 lowest = std::min(lowest, (b2.R() * c).y);
             }
             b2.x.y = -lowest;
             b2.updateDerived();
         }},
        {"Balls and boxes", true,
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             w.addBody(Shape::plane(), 0);
             RigidBody& block = w.addBody(Shape::box({1.5, 0.4, 1.0}), 0);
             block.x = {-1.5, 0.4, 0};
             block.updateDerived();
             for (int i = 0; i < 12; ++i) {
                 bool box = i % 2 == 0;
                 Shape shape = box ? Shape::box({0.35, 0.25, 0.3}) : Shape::sphere(0.3);
                 Vec3 x{(i % 4 - 1.5) * 1.1, 2.0 + 0.8 * i, (i % 3 - 1) * 0.9};
                 RigidBody& b = place(w, shape, 1.0, x, Quat::fromAxisAngle({1, 2, 3}, 0.7 * i));
                 b.restitution = box ? 0.2 : 0.5;
                 b.setAngularVelocity({0.5, 0.8 * (i % 5 - 2), 0.3});
             }
         }},
        {"Box stack", true,
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             w.addBody(Shape::plane(), 0).restitution = 0;
             for (int i = 0; i < 6; ++i)
                 place(w, Shape::box({0.5, 0.5, 0.5}), 1, {-2, 0.5 + 1.0 * i, 0}).restitution = 0;
             // pyramid of 3 + 2 + 1 boxes
             for (int row = 0; row < 3; ++row)
                 for (int k = 0; k < 3 - row; ++k)
                     place(w, Shape::box({0.4, 0.4, 0.4}), 1,
                           {1.5 + (k - (2 - row) / 2.0) * 0.82, 0.4 + 0.8 * row, 0})
                         .restitution = 0;
         }},
        {"Friction ramp (mu = 0.2, 0.4, 0.6, 0.8)", true,
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             w.addBody(Shape::plane(), 0).friction = 0.5;
             // 25 degree ramp: tan 25 = 0.47, so the first two boxes slide.
             const Real theta = 25 * M_PI / 180;
             // Tilted so it faces the default camera: it rises towards -x.
             const Quat q = Quat::fromAxisAngle({0, 0, 1}, -theta);
             const Vec3 n = rotate(q, {0, 1, 0}), uphill = -rotate(q, {1, 0, 0}), across{0, 0, 1};
             const Vec3 centre{0, 2.2, 0};
             RigidBody& ramp = place(w, Shape::box({4.5, 0.1, 3.2}), 0, centre - n * 0.1, q);
             ramp.friction = 1; // combined mu = sqrt(mu_box * 1), so mu_box = mu^2
             const Real mus[4] = {0.2, 0.4, 0.6, 0.8};
             for (int i = 0; i < 4; ++i) {
                 RigidBody& b = place(w, Shape::box({0.3, 0.2, 0.3}), 1,
                                      centre + uphill * 2.5 + across * (i - 1.5) * 0.9 + n * 0.2, q);
                 b.friction = mus[i] * mus[i];
                 b.restitution = 0;
             }
             for (int i = 0; i < 2; ++i) {
                 RigidBody& ball = place(w, Shape::sphere(0.25), 1,
                                         centre + uphill * 3.5 + across * (i * 2 - 1) * 2.4 + n * 0.25);
                 ball.friction = 0.25;
                 ball.restitution = 0;
             }
         }},
        {.name = "Cannon vs brick wall (F: fire)",
         .contacts = true,
         .build = buildBrickWall,
         .substeps = 2, // ~150 bricks and ~1600 contact points: 2 substeps keep it real-time
         .camera = OrbitCamera{.yaw = 1.3f, .pitch = 0.3f, .distance = 20.0f, .target = {-2.5f, 1.2f, 6.0f}},
         .showVectors = false,
         .showContacts = false,
         .actionLabel = "Fire",
         .action = fireCannon,
         .autoActionAt = 1.0,
         .decorate = drawCannon},
        {.name = "Pendulums: simple, chain, double, Newton's cradle",
         .contacts = true,
         .build =
             [](World& w) {
                 w.gravity = {0, -9.81, 0};
                 // Simple pendulum: a ball on a ball joint, released at 40 degrees.
                 const Vec3 p1{-4.5, 4, 0};
                 RigidBody& bob =
                     place(w, Shape::sphere(0.2), 1, p1 + Vec3{1.5 * std::sin(0.7), -1.5 * std::cos(0.7), 0});
                 w.addBallJoint(w.ground(), bob, p1);
                 // Chain of 8 links hanging from a point, started sideways.
                 const Vec3 p2{-1.8, 4, 0};
                 RigidBody* prev = &w.ground();
                 for (int i = 0; i < 8; ++i) {
                     RigidBody& link =
                         place(w, Shape::box({0.15, 0.04, 0.04}), 0.4, p2 + Vec3{0.3 * i + 0.15, 0, 0});
                     w.addBallJoint(*prev, link, p2 + Vec3{0.3 * i, 0, 0});
                     prev = &link;
                 }
                 // Double pendulum: two bars on hinges (chaotic).
                 const Vec3 p3{1.2, 4, 0};
                 Quat tilt = Quat::fromAxisAngle({0, 0, 1}, 2.2);
                 Vec3 down = rotate(tilt, {0, -1, 0});
                 RigidBody& upper = place(w, Shape::box({0.04, 0.4, 0.04}), 1, p3 + down * 0.4, tilt);
                 w.addHinge(w.ground(), upper, p3, {0, 0, 1});
                 RigidBody& lower = place(w, Shape::box({0.04, 0.4, 0.04}), 1, p3 + down * 1.2, tilt);
                 w.addHinge(upper, lower, p3 + down * 0.8, {0, 0, 1});
                 // Newton's cradle: 5 elastic balls, the first raised. The 3 cm gaps
                 // are wider than the contact margin, so each impact is a separate
                 // two-ball collision (see docs/notes.md: with 1 mm gaps the solver
                 // treats the five as one inelastic chain).
                 const Real r = 0.2, gap = 0.03, len = 1.6;
                 for (int i = 0; i < 5; ++i) {
                     Vec3 pivot{4 + i * (2 * r + gap), 4, 0};
                     Real a = i == 0 ? -0.6 : 0;
                     RigidBody& ball = place(w, Shape::sphere(r), 1,
                                             pivot + Vec3{len * std::sin(a), -len * std::cos(a), 0});
                     ball.restitution = 1;
                     ball.friction = 0;
                     ball.userData = packColor(190, 195, 205);
                     w.addBallJoint(w.ground(), ball, pivot);
                 }
             },
         .camera = OrbitCamera{.yaw = 0.2f, .pitch = 0.12f, .distance = 14.0f, .target = {-1.2f, 2.6f, 0}},
         .showVectors = false},
        {.name = "Springs and dampers",
         .contacts = true,
         .build =
             [](World& w) {
                 w.gravity = {0, -9.81, 0};
                 w.addBody(Shape::plane(), 0);
                 // Three masses dropped from the springs' rest length: no damping,
                 // light damping and critical damping (c = 2 sqrt(k m)).
                 const Real k = 60, m = 1;
                 const Real dampings[3] = {0, 1.5, 2 * std::sqrt(k * m)};
                 for (int i = 0; i < 3; ++i) {
                     Vec3 top{-3.5 + i * 1.0, 4.5, 0};
                     RigidBody& mass = place(w, Shape::box({0.15, 0.15, 0.15}), m, top - Vec3{0, 1.2, 0});
                     w.addSpring(w.ground(), top, mass, mass.x + Vec3{0, 0.15, 0}, k, dampings[i], 1.05);
                 }
                 // A crate hanging from four springs, released from their rest
                 // length. (Standing on compression springs it would tip over:
                 // springs only push along their own line, with no sideways
                 // stiffness, which is why real suspensions guide them.)
                 const Vec3 h{0.8, 0.15, 0.5};
                 RigidBody& crate =
                     place(w, Shape::box(h), 20, {2.2, 2.05, 0}, Quat::fromAxisAngle({1, 0, 0}, 0.1));
                 crate.userData = packColor(170, 130, 80);
                 const Real ks = 330, cs = 25; // static stretch m g / 4k = 0.15 m, damping ratio ~0.3
                 for (int sx : {-1, 1})
                     for (int sz : {-1, 1}) {
                         Vec3 corner = crate.x + crate.R() * Vec3{sx * h.x, h.y, sz * h.z};
                         w.addSpring(crate, corner, w.ground(), {2.2 + sx * h.x, 3.2, sz * h.z}, ks, cs, 1.0);
                     }
             },
         .camera = OrbitCamera{.yaw = 0.35f, .pitch = 0.18f, .distance = 11.0f, .target = {-0.5f, 2.2f, 0}},
         .showVectors = false},
        {.name = "Hinges and sliders (F: throw a ball at the door)",
         .contacts = true,
         .build =
             [](World& w) {
                 w.gravity = {0, -9.81, 0};
                 w.addBody(Shape::plane(), 0);
                 // Door: hinged on its left edge, opens up to 100 degrees either way.
                 RigidBody& door = place(w, Shape::box({0.45, 1.0, 0.03}), 15, {-3.05, 1.02, 0});
                 door.userData = packColor(150, 110, 70);
                 w.addHinge(w.ground(), door, {-3.5, 1.02, 0}, {0, 1, 0}).setLimits(-1.75, 1.75);
                 // Turntable: a disc driven at 0.8 rad/s, carrying three boxes.
                 RigidBody& table = place(w, Shape::box({1.2, 0.05, 1.2}), 40, {0.5, 0.35, 0});
                 table.userData = packColor(120, 125, 135);
                 w.addHinge(w.ground(), table, table.x, {0, 1, 0}).setMotor(0.8, 400);
                 place(w, Shape::box({0.2, 0.15, 0.2}), 0, {0.5, 0.15, 0}); // pedestal under the disc
                 for (int i = 0; i < 3; ++i) {
                     Real a = i * 2.1;
                     place(w, Shape::box({0.15, 0.15, 0.15}), 1,
                           {0.5 + 0.8 * std::cos(a), 0.55, 0.8 * std::sin(a)})
                         .friction = 0.8;
                 }
                 // Elevator: a platform on a vertical slider moving between 0 and 2 m.
                 RigidBody& platform = place(w, Shape::box({0.6, 0.05, 0.6}), 30, {3.5, 0.3, 0});
                 platform.userData = packColor(120, 125, 135);
                 SliderJoint& lift = w.addSlider(w.ground(), platform, platform.x, {0, 1, 0});
                 lift.setLimits(0, 2);
                 lift.setMotor(0.6, 2000);
                 place(w, Shape::box({0.2, 0.2, 0.2}), 2, {3.5, 0.55, 0}).friction = 0.8;
             },
         .camera = OrbitCamera{.yaw = 0.55f, .pitch = 0.3f, .distance = 11.0f, .target = {0.2f, 0.9f, 0}},
         .showVectors = false,
         .actionLabel = "Throw ball",
         .action =
             [](World& w) {
                 RigidBody& ball = place(w, Shape::sphere(0.15), 3, {-2.7, 1.3, 4});
                 ball.userData = packColor(40, 42, 48);
                 ball.setLinearVelocity({0, 1.5, -9});
             },
         .update =
             [](World& w) {
                 // Reverse the elevator at the ends of its travel.
                 for (const auto& j : w.joints())
                     if (auto* s = dynamic_cast<SliderJoint*>(j.get())) {
                         if (s->position() > s->upperLimit() - 0.01 && s->motorSpeed() > 0)
                             s->setMotor(-s->motorSpeed(), s->maxMotorForce());
                         if (s->position() < s->lowerLimit() + 0.01 && s->motorSpeed() < 0)
                             s->setMotor(-s->motorSpeed(), s->maxMotorForce());
                     }
             }},
        {"Restitution (e = 0, 0.25, 0.5, 0.75, 1)", true,
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             w.addBody(Shape::plane(), 0).restitution = 0;
             for (int i = 0; i < 5; ++i)
                 place(w, Shape::sphere(0.3), 1, {(i - 2) * 1.0, 2.3, 0}).restitution = 0.25 * i;
         }},
    };
    return list;
}

// ---------------------------------------------------------------- app state

struct App {
    World world;
    int sceneIndex = 0;
    bool paused = false;
    bool stepOnce = false;
    float timeScale = 1.0f;
    int stepsPerSecond = 240;
    bool showVectors = true;
    bool showContacts = true;
    bool showNormals = false;
    float forceScale = 0.05f; ///< metres per newton for the force arrows (0: off)
    int contactSubsteps = 4;  ///< substeps in contact scenes
    OrbitCamera orbit;
    bool autoActionDone = false;
    Vec3 sceneGravity{}; ///< the scene's own gravity, for the on/off switch
    double accumulator = 0;

    // "Move a body" editor: rotation is applied on top of the orientation the
    // body had when it was selected.
    IntegratorType freeIntegrator = IntegratorType::SemiImplicitEuler; ///< choice for free-motion scenes
    int editIndex = -1;
    Quat editBaseQ{};
    float editEulerDeg[3]{};

    static constexpr int kHistory = 600;
    std::array<float, kHistory> omegaHistory[3]{};
    std::array<float, kHistory> energyErrorHistory{};
    int historyPos = 0;
    Real energy0 = 0; ///< total energy when the scene was loaded

    void loadScene(int i) {
        sceneIndex = i;
        world.clear();
        const Scene& scene = scenes()[i];
        if (scene.substeps > 0) contactSubsteps = scene.substeps;
        if (scene.camera) orbit = *scene.camera;
        showVectors = scene.showVectors;
        showContacts = scene.showContacts;
        autoActionDone = false;
        if (scene.contacts) {
            world.integrator = IntegratorType::SemiImplicitEuler;
            world.substeps = contactSubsteps;
        } else {
            world.integrator = freeIntegrator;
            world.substeps = 1; // keep the integrator comparison at the plain step length
        }
        scene.build(world);
        sceneGravity = world.gravity;
        world.detectCollisions();
        accumulator = 0;
        editIndex = -1;
        for (auto& h : omegaHistory)
            h.fill(0);
        energyErrorHistory.fill(0);
        historyPos = 0;
        energy0 = world.totalEnergy();
    }

    void recordHistory() {
        if (world.bodies().empty()) return;
        const RigidBody& b = *world.bodies().front();
        // Angular velocity in body coordinates shows the axis flip clearly.
        Vec3 wb = transpose(b.R()) * b.omega();
        for (int k = 0; k < 3; ++k)
            omegaHistory[k][historyPos] = float(wb[k]);
        energyErrorHistory[historyPos] = float(world.totalEnergy() - energy0);
        historyPos = (historyPos + 1) % kHistory;
    }

    void stepOnceNow(double h) {
        world.step(h);
        recordHistory();
        const Scene& scene = scenes()[sceneIndex];
        if (scene.update) scene.update(world);
        if (!autoActionDone && scene.action && scene.autoActionAt >= 0 &&
            world.time() >= scene.autoActionAt) {
            autoActionDone = true;
            runAction();
        }
    }

    void runAction() {
        const Scene& scene = scenes()[sceneIndex];
        if (!scene.action) return;
        scene.action(world);
        world.detectCollisions();
    }

    void advance(double frameTime) {
        const double h = 1.0 / stepsPerSecond;
        if (paused) {
            if (stepOnce) stepOnceNow(h);
            stepOnce = false;
            return;
        }
        accumulator += std::min(frameTime, 0.1) * timeScale;
        while (accumulator >= h) {
            stepOnceNow(h);
            accumulator -= h;
        }
    }

    void gui() {
        ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({360, 0}, ImGuiCond_FirstUseEver);
        ImGui::Begin("ncotb");

        if (ImGui::BeginCombo("Scene", scenes()[sceneIndex].name)) {
            for (int i = 0; i < int(scenes().size()); ++i)
                if (ImGui::Selectable(scenes()[i].name, i == sceneIndex)) loadScene(i);
            ImGui::EndCombo();
        }
        if (ImGui::Button("Reset (R)")) loadScene(sceneIndex);
        ImGui::SameLine();
        ImGui::Checkbox("Paused (Space)", &paused);
        ImGui::SameLine();
        if (ImGui::Button("Step (S)")) stepOnce = true;
        if (const char* label = scenes()[sceneIndex].actionLabel) {
            if (ImGui::Button((std::string(label) + " (F)").c_str())) runAction();
        }

        const bool contactScene = scenes()[sceneIndex].contacts;
        ImGui::BeginDisabled(contactScene);
        if (ImGui::BeginCombo("Integrator", name(world.integrator))) {
            for (IntegratorType type : kAllIntegrators)
                if (ImGui::Selectable(name(type), type == world.integrator)) {
                    freeIntegrator = type;
                    loadScene(sceneIndex); // restart so E - E0 compares like with like
                }
            ImGui::EndCombo();
        }
        ImGui::EndDisabled();
        if (contactScene)
            ImGui::SetItemTooltip("Contacts are resolved between the velocity and position updates,\n"
                                  "which only semi-implicit Euler has. See docs/notes.md.");
        ImGui::SliderFloat("Time scale", &timeScale, 0.05f, 2.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderInt("Steps / s", &stepsPerSecond, 30, 1000);
        ImGui::Checkbox("Show omega (yellow) and L (magenta)", &showVectors);
        bool gravityOn = lengthSquared(world.gravity) > 0;
        if (ImGui::Checkbox("Gravity", &gravityOn))
            world.gravity =
                gravityOn ? (lengthSquared(sceneGravity) > 0 ? sceneGravity : Vec3{0, -9.81, 0}) : Vec3{};
        ImGui::Checkbox("Show contacts", &showContacts);

        if (contactScene) solverGui();
        contactsGui();
        editGui();

        if (!ImGui::CollapsingHeader("Energy and momentum", ImGuiTreeNodeFlags_DefaultOpen)) {
            footer();
            return;
        }
        Real energy = world.totalEnergy();
        Vec3 L{}, P{};
        for (auto& b : world.bodies()) {
            L += b->L + cross(b->x, b->P);
            P += b->P;
        }
        ImGui::Text("t = %.2f s   bodies = %zu", world.time(), world.bodies().size());
        ImGui::Text("Total energy  E   = %.5f J", energy);
        ImGui::Text("Energy error E-E0 = %+.2e J", energy - energy0);
        ImGui::PlotLines("E-E0", energyErrorHistory.data(), kHistory, historyPos, nullptr, FLT_MAX, FLT_MAX,
                         {0, 50});
        ImGui::Text("Ang. momentum |L| = %.5f", length(L));
        ImGui::Text("Lin. momentum |P| = %.5f", length(P));

        if (!world.bodies().empty() && ImGui::CollapsingHeader("Body 0 angular velocity")) {
            ImGui::TextUnformatted("omega in body frame (x, y, z):");
            const char* labels[3] = {"wx", "wy", "wz"};
            for (int k = 0; k < 3; ++k)
                ImGui::PlotLines(labels[k], omegaHistory[k].data(), kHistory, historyPos, nullptr, -8.0f,
                                 8.0f, {0, 50});
        }
        footer();
    }

    void footer() {
        ImGui::Separator();
        ImGui::TextDisabled("Drag: orbit   Wheel: zoom   %d FPS", GetFPS());
        ImGui::End();
    }

    void solverGui() {
        if (!ImGui::CollapsingHeader("Contact solver", ImGuiTreeNodeFlags_DefaultOpen)) return;
        ContactSettings& cs = world.solver.settings;
        if (ImGui::BeginCombo("Solver", name(cs.type))) {
            for (ContactSolverType t : {ContactSolverType::SequentialImpulses, ContactSolverType::BaraffLcp})
                if (ImGui::Selectable(name(t), t == cs.type)) cs.type = t;
            ImGui::EndCombo();
        }
        if (ImGui::SliderInt("Substeps", &contactSubsteps, 1, 16)) world.substeps = contactSubsteps;
        ImGui::BeginDisabled(cs.type != ContactSolverType::SequentialImpulses);
        ImGui::SliderInt("Iterations", &cs.iterations, 1, 50);
        ImGui::Checkbox("Warm start", &cs.warmStart);
        ImGui::EndDisabled();
        float beta = float(cs.baumgarte), slopMm = float(cs.slop * 1000);
        if (ImGui::SliderFloat("Baumgarte", &beta, 0, 1)) cs.baumgarte = beta;
        if (ImGui::SliderFloat("Slop (mm)", &slopMm, 0, 20)) cs.slop = slopMm / 1000;
        float maxPush = float(cs.maxPushVelocity);
        if (ImGui::SliderFloat("Max push (m/s)", &maxPush, 0.1f, 10, "%.1f", ImGuiSliderFlags_Logarithmic))
            cs.maxPushVelocity = maxPush;
        ImGui::Checkbox("Relax (remove push-out velocity)", &cs.relax);
        ImGui::Checkbox("Friction", &cs.friction);
        if (cs.type == ContactSolverType::BaraffLcp && cs.friction)
            ImGui::SliderInt("Normal/friction rounds", &cs.staggers, 1, 10);
        ImGui::Checkbox("Resolve contacts", &world.resolveContacts);
        const ContactSolverStats& st = world.solver.stats();
        if (cs.type == ContactSolverType::BaraffLcp && st.lcpTooLarge)
            ImGui::TextColored({1, 0.7f, 0.3f, 1}, "%d rows > %d: used sequential impulses", st.rows,
                               cs.lcpMaxRows);
        else if (cs.type == ContactSolverType::BaraffLcp)
            ImGui::Text("%d rows, %d pivots%s", st.rows, st.pivots, st.lcpFailed ? "  PIVOT LIMIT HIT" : "");
        else
            ImGui::Text("%d rows", st.rows);
    }

    void contactsGui() {
        if (!ImGui::CollapsingHeader("Contacts", ImGuiTreeNodeFlags_DefaultOpen)) return;
        ImGui::Checkbox("Normals", &showNormals);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::SliderFloat("Force arrows (m/N)", &forceScale, 0, 0.2f);
        ImGui::TextDisabled("green: normal force, blue: friction (on body a)");
        int points = 0;
        Real maxDepth = 0;
        for (const ContactManifold& m : world.contacts()) {
            points += m.count;
            for (int i = 0; i < m.count; ++i)
                maxDepth = std::max(maxDepth, m.points[i].depth);
        }
        ImGui::Text("%zu contacting pairs, %d points, max depth %.3f", world.contacts().size(), points,
                    maxDepth);
        if (ImGui::BeginTable("contacts", 5, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("a");
            ImGui::TableSetupColumn("b");
            ImGui::TableSetupColumn("points");
            ImGui::TableSetupColumn("normal (b to a)");
            ImGui::TableSetupColumn("force (N)");
            ImGui::TableHeadersRow();
            for (const ContactManifold& m : world.contacts()) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%d", bodyIndex(m.a));
                ImGui::TableNextColumn();
                ImGui::Text("%d", bodyIndex(m.b));
                ImGui::TableNextColumn();
                ImGui::Text("%d", m.count);
                ImGui::TableNextColumn();
                ImGui::Text("%+.2f %+.2f %+.2f", m.normal.x, m.normal.y, m.normal.z);
                ImGui::TableNextColumn();
                Real impulse = 0;
                for (int k = 0; k < m.count; ++k)
                    impulse += m.points[k].normalImpulse;
                ImGui::Text("%.2f", world.lastSubstep() > 0 ? impulse / world.lastSubstep() : 0.0);
            }
            ImGui::EndTable();
        }
    }

    int bodyIndex(const RigidBody* b) const {
        for (size_t i = 0; i < world.bodies().size(); ++i)
            if (world.bodies()[i].get() == b) return int(i);
        return -1;
    }

    static const char* shapeName(const RigidBody& b) {
        switch (b.shape().type) {
        case Shape::Type::Sphere: return "sphere";
        case Shape::Type::Box: return "box";
        case Shape::Type::Plane: return "plane";
        }
        return "?";
    }

    void editGui() {
        if (!ImGui::CollapsingHeader("Move a body")) return;
        const auto& bodies = world.bodies();
        std::string current =
            editIndex >= 0 ? std::to_string(editIndex) + " " + shapeName(*bodies[editIndex]) : "none";
        if (ImGui::BeginCombo("Body", current.c_str())) {
            for (int i = 0; i < int(bodies.size()); ++i) {
                if (bodies[i]->shape().type == Shape::Type::Plane) continue;
                std::string label = std::to_string(i) + " " + shapeName(*bodies[i]);
                if (ImGui::Selectable(label.c_str(), i == editIndex)) {
                    editIndex = i;
                    editBaseQ = bodies[i]->q;
                    for (float& e : editEulerDeg)
                        e = 0;
                }
            }
            ImGui::EndCombo();
        }
        if (editIndex < 0) return;

        RigidBody& b = *bodies[editIndex];
        float pos[3] = {float(b.x.x), float(b.x.y), float(b.x.z)};
        bool changed = ImGui::DragFloat3("Position", pos, 0.01f);
        changed |= ImGui::DragFloat3("Rotate x/y/z (deg)", editEulerDeg, 0.5f);
        if (changed) {
            b.x = {pos[0], pos[1], pos[2]};
            const Real k = M_PI / 180;
            Quat r = Quat::fromAxisAngle({0, 0, 1}, editEulerDeg[2] * k) *
                     Quat::fromAxisAngle({0, 1, 0}, editEulerDeg[1] * k) *
                     Quat::fromAxisAngle({1, 0, 0}, editEulerDeg[0] * k);
            b.q = normalize(r * editBaseQ);
            b.updateDerived();
            world.detectCollisions();
        }
    }
};

} // namespace

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI | FLAG_VSYNC_HINT);
    InitWindow(1400, 900, "ncotb — rigid body simulation");
    SetTargetFPS(120);
    {
        // If the window doesn't fit the screen, macOS shrinks it without raylib
        // noticing; ImGui then clips with the stale size (empty combo popups).
        int monitor = GetCurrentMonitor();
        int mw = GetMonitorWidth(monitor), mh = GetMonitorHeight(monitor);
        int w = std::min(1400, mw * 9 / 10), h = std::min(900, mh * 8 / 10);
        SetWindowSize(w, h);
        SetWindowPosition((mw - w) / 2, (mh - h) / 2);
    }
    rlImGuiSetup(true);

    App app;
    app.loadScene(0);

    const Color palette[] = {{230, 120, 70, 255}, {80, 150, 220, 255},  {120, 190, 100, 255},
                             {220, 190, 70, 255}, {170, 110, 200, 255}, {90, 200, 190, 255}};

    while (!WindowShouldClose()) {
        bool mouseFree = !ImGui::GetIO().WantCaptureMouse;
        bool keysFree = !ImGui::GetIO().WantCaptureKeyboard;
        app.orbit.update(mouseFree);
        if (keysFree) {
            if (IsKeyPressed(KEY_SPACE)) app.paused = !app.paused;
            if (IsKeyPressed(KEY_S)) app.stepOnce = true;
            if (IsKeyPressed(KEY_R)) app.loadScene(app.sceneIndex);
            if (IsKeyPressed(KEY_F)) app.runAction();
        }

        app.advance(GetFrameTime());

        BeginDrawing();
        ClearBackground({235, 238, 242, 255});
        BeginMode3D(app.orbit.camera());
        DrawGrid(40, 1.0f);
        int i = 0;
        for (auto& b : app.world.bodies()) {
            if (b->shape().type == Shape::Type::Plane) continue; // drawn below
            if (b->isStatic()) {
                drawBody(*b, colorOf(*b, {150, 155, 165, 255}));
                continue;
            }
            drawBody(*b, colorOf(*b, palette[i++ % std::size(palette)]));
            if (app.showVectors) {
                drawArrow(b->x, b->omega() * 0.3, {200, 160, 0, 255});
                drawArrow(b->x, b->L * 0.3, MAGENTA);
            }
        }
        if (const auto& decorate = scenes()[app.sceneIndex].decorate) decorate();
        drawJoints(app.world);
        drawSprings(app.world);
        // Translucent planes last and without depth writes, so bodies that sink
        // below them stay visible.
        rlDrawRenderBatchActive();
        rlDisableDepthMask();
        for (auto& b : app.world.bodies())
            if (b->shape().type == Shape::Type::Plane) drawBody(*b, {120, 130, 145, 90});
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        if (app.showContacts) {
            // Overlay without depth test: force arrows point into body a and
            // would otherwise be hidden inside it.
            rlDrawRenderBatchActive();
            rlDisableDepthTest();
            drawContacts(app.world.contacts(), app.showNormals, app.forceScale, app.world.lastSubstep());
            rlDrawRenderBatchActive();
            rlEnableDepthTest();
        }
        EndMode3D();

        rlImGuiBegin();
        app.gui();
        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
