#include "scenes.h"

#include "draw.h"

#include <cmath>
#include <random>

namespace ncotb::app {

namespace {

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

} // namespace

const std::vector<Scene>& scenes() {
    static const std::vector<Scene> list = {
        {.name = "Playground (drag, Option+click, 1/2/3, Backspace)",
         .contacts = true,
         .build =
             [](World& w) {
                 w.gravity = {0, -9.81, 0};
                 w.addBody(Shape::plane(), 0).friction = 0.6;
                 // A tower of boxes.
                 for (int i = 0; i < 5; ++i)
                     w.addBody(Shape::box({0.3, 0.3, 0.3}), 2, {-2.5, 0.3 + 0.6 * i, -0.5});
                 // Loose balls and boxes.
                 for (int i = 0; i < 4; ++i)
                     w.addBody(Shape::sphere(0.25 + 0.05 * i), 1 + i, {-0.5 + 0.8 * i, 0.25 + 0.05 * i, 1.5});
                 w.addBody(Shape::box({0.5, 0.2, 0.3}), 3, {2.6, 0.2, 2.4},
                           Quat::fromAxisAngle({0, 1, 0}, 0.5));
                 // A seesaw: a plank on a hinge over a static block.
                 w.addBody(Shape::box({0.15, 0.3, 0.6}), 0, {0.5, 0.3, -1.5});
                 RigidBody& plank = w.addBody(Shape::box({1.6, 0.05, 0.4}), 4, {0.5, 0.65, -1.5},
                                              Quat::fromAxisAngle({0, 0, 1}, 0.2));
                 plank.userData = packColor(150, 110, 70);
                 w.addHinge(w.ground(), plank, {0.5, 0.65, -1.5}, {0, 0, 1});
                 // A ramp.
                 w.addBody(Shape::box({1.5, 0.1, 1}), 0, {3.5, 0.6, 0.8},
                           Quat::fromAxisAngle({0, 0, 1}, 0.4));
                 // A ball hanging on a chain of three links.
                 RigidBody* prev = &w.ground();
                 const Vec3 top{3.5, 3.6, -2};
                 for (int i = 0; i < 3; ++i) {
                     RigidBody& link =
                         w.addBody(Shape::box({0.04, 0.2, 0.04}), 0.3, top - Vec3{0, 0.4 * i + 0.2, 0});
                     w.addBallJoint(*prev, link, top - Vec3{0, 0.4 * i, 0});
                     prev = &link;
                 }
                 RigidBody& ball = w.addBody(Shape::sphere(0.3), 3, top - Vec3{0, 1.5, 0});
                 w.addBallJoint(*prev, ball, top - Vec3{0, 1.2, 0});
             },
         .showVectors = false},
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
             w.addBody(Shape::box({1, 0.5, 1}), 0, {-3, 0.5, 0});
             w.addBody(Shape::box({0.5, 0.5, 0.5}), 1, {-3, 1.5, 0.3}, Quat::fromAxisAngle({0, 1, 0}, 0.4));
             w.addBody(Shape::sphere(0.5), 1, {0, 0.5, 0});
             // Tilted box with its lowest corner on the ground.
             RigidBody& b2 =
                 w.addBody(Shape::box({0.4, 0.3, 0.6}), 1, {1.4, 0, 0}, Quat::fromAxisAngle({1, 0, 1}, 0.6));
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
                 RigidBody& b = w.addBody(shape, 1.0, x, Quat::fromAxisAngle({1, 2, 3}, 0.7 * i));
                 b.restitution = box ? 0.2 : 0.5;
                 b.setAngularVelocity({0.5, 0.8 * (i % 5 - 2), 0.3});
             }
         }},
        {"Box stack", true,
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             w.addBody(Shape::plane(), 0).restitution = 0;
             for (int i = 0; i < 6; ++i)
                 w.addBody(Shape::box({0.5, 0.5, 0.5}), 1, {-2, 0.5 + 1.0 * i, 0}).restitution = 0;
             // pyramid of 3 + 2 + 1 boxes
             for (int row = 0; row < 3; ++row)
                 for (int k = 0; k < 3 - row; ++k)
                     w.addBody(Shape::box({0.4, 0.4, 0.4}), 1,
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
             RigidBody& ramp = w.addBody(Shape::box({4.5, 0.1, 3.2}), 0, centre - n * 0.1, q);
             ramp.friction = 1; // combined mu = sqrt(mu_box * 1), so mu_box = mu^2
             const Real mus[4] = {0.2, 0.4, 0.6, 0.8};
             for (int i = 0; i < 4; ++i) {
                 RigidBody& b = w.addBody(Shape::box({0.3, 0.2, 0.3}), 1,
                                          centre + uphill * 2.5 + across * (i - 1.5) * 0.9 + n * 0.2, q);
                 b.friction = mus[i] * mus[i];
                 b.restitution = 0;
             }
             for (int i = 0; i < 2; ++i) {
                 RigidBody& ball = w.addBody(Shape::sphere(0.25), 1,
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
                 RigidBody& bob = w.addBody(Shape::sphere(0.2), 1,
                                            p1 + Vec3{1.5 * std::sin(0.7), -1.5 * std::cos(0.7), 0});
                 w.addBallJoint(w.ground(), bob, p1);
                 // Chain of 8 links hanging from a point, started sideways.
                 const Vec3 p2{-1.8, 4, 0};
                 RigidBody* prev = &w.ground();
                 for (int i = 0; i < 8; ++i) {
                     RigidBody& link =
                         w.addBody(Shape::box({0.15, 0.04, 0.04}), 0.4, p2 + Vec3{0.3 * i + 0.15, 0, 0});
                     w.addBallJoint(*prev, link, p2 + Vec3{0.3 * i, 0, 0});
                     prev = &link;
                 }
                 // Double pendulum: two bars on hinges (chaotic).
                 const Vec3 p3{1.2, 4, 0};
                 Quat tilt = Quat::fromAxisAngle({0, 0, 1}, 2.2);
                 Vec3 down = rotate(tilt, {0, -1, 0});
                 RigidBody& upper = w.addBody(Shape::box({0.04, 0.4, 0.04}), 1, p3 + down * 0.4, tilt);
                 w.addHinge(w.ground(), upper, p3, {0, 0, 1});
                 RigidBody& lower = w.addBody(Shape::box({0.04, 0.4, 0.04}), 1, p3 + down * 1.2, tilt);
                 w.addHinge(upper, lower, p3 + down * 0.8, {0, 0, 1});
                 // Newton's cradle: 5 elastic balls, the first raised. The 3 cm gaps
                 // are wider than the contact margin, so each impact is a separate
                 // two-ball collision (see docs/notes.md: with 1 mm gaps the solver
                 // treats the five as one inelastic chain).
                 const Real r = 0.2, gap = 0.03, len = 1.6;
                 for (int i = 0; i < 5; ++i) {
                     Vec3 pivot{4 + i * (2 * r + gap), 4, 0};
                     Real a = i == 0 ? -0.6 : 0;
                     RigidBody& ball = w.addBody(Shape::sphere(r), 1,
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
                     RigidBody& mass = w.addBody(Shape::box({0.15, 0.15, 0.15}), m, top - Vec3{0, 1.2, 0});
                     w.addSpring(w.ground(), top, mass, mass.x + Vec3{0, 0.15, 0}, k, dampings[i], 1.05);
                 }
                 // A crate hanging from four springs, released from their rest
                 // length. (Standing on compression springs it would tip over:
                 // springs only push along their own line, with no sideways
                 // stiffness, which is why real suspensions guide them.)
                 const Vec3 h{0.8, 0.15, 0.5};
                 RigidBody& crate =
                     w.addBody(Shape::box(h), 20, {2.2, 2.05, 0}, Quat::fromAxisAngle({1, 0, 0}, 0.1));
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
                 RigidBody& door = w.addBody(Shape::box({0.45, 1.0, 0.03}), 15, {-3.05, 1.02, 0});
                 door.userData = packColor(150, 110, 70);
                 w.addHinge(w.ground(), door, {-3.5, 1.02, 0}, {0, 1, 0}).setLimits(-1.75, 1.75);
                 // Turntable: a disc driven at 0.8 rad/s, carrying three boxes. Its
                 // underside (0.25 m) is lower than the boxes are tall (0.3 m), so a
                 // box that slides off gets swept along by the edge. (At exactly the
                 // same height the disc would glide flat over the box tops.)
                 RigidBody& table = w.addBody(Shape::box({1.2, 0.05, 1.2}), 40, {0.5, 0.30, 0});
                 table.userData = packColor(120, 125, 135);
                 w.addHinge(w.ground(), table, table.x, {0, 1, 0}).setMotor(0.8, 400);
                 w.addBody(Shape::box({0.2, 0.125, 0.2}), 0, {0.5, 0.125, 0}); // pedestal under the disc
                 for (int i = 0; i < 3; ++i) {
                     Real a = i * 2.1;
                     w.addBody(Shape::box({0.15, 0.15, 0.15}), 1,
                               {0.5 + 0.8 * std::cos(a), 0.50, 0.8 * std::sin(a)})
                         .friction = 0.8;
                 }
                 // Elevator: a platform on a vertical slider moving between 0 and 2 m.
                 RigidBody& platform = w.addBody(Shape::box({0.6, 0.05, 0.6}), 30, {3.5, 0.3, 0});
                 platform.userData = packColor(120, 125, 135);
                 SliderJoint& lift = w.addSlider(w.ground(), platform, platform.x, {0, 1, 0});
                 lift.setLimits(0, 2);
                 lift.setMotor(0.6, 2000);
                 w.addBody(Shape::box({0.2, 0.2, 0.2}), 2, {3.5, 0.55, 0}).friction = 0.8;
             },
         .camera = OrbitCamera{.yaw = 0.55f, .pitch = 0.3f, .distance = 11.0f, .target = {0.2f, 0.9f, 0}},
         .showVectors = false,
         .actionLabel = "Throw ball",
         .action =
             [](World& w) {
                 RigidBody& ball = w.addBody(Shape::sphere(0.15), 3, {-2.7, 1.3, 4});
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
                 w.addBody(Shape::sphere(0.3), 1, {(i - 2) * 1.0, 2.3, 0}).restitution = 0.25 * i;
         }},
    };
    return list;
}

} // namespace ncotb::app
