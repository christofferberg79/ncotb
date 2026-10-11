#include <doctest/doctest.h>

#include "world.h"

#include <cmath>

using namespace ncotb;

namespace {

constexpr Real kH = 1.0 / 240;

bool near(const Vec3& a, const Vec3& b, Real eps = 1e-9) { return length(a - b) < eps; }

} // namespace

TEST_CASE("ray casts: sphere hit, miss, tangent and from inside") {
    World w;
    RigidBody& ball = w.addBody(Shape::sphere(1), 1, {0, 0, 0});
    auto hit = raycast(ball, {-5, 0, 0}, {1, 0, 0});
    REQUIRE(hit);
    CHECK(hit->distance == doctest::Approx(4));
    CHECK(near(hit->point, {-1, 0, 0}));
    CHECK(near(hit->normal, {-1, 0, 0}));
    CHECK_FALSE(raycast(ball, {-5, 1.01, 0}, {1, 0, 0})); // just above
    CHECK(raycast(ball, {-5, 0.999, 0}, {1, 0, 0}));      // just grazing
    CHECK_FALSE(raycast(ball, {-5, 0, 0}, {-1, 0, 0}));   // pointing away
    auto inside = raycast(ball, {0.2, 0, 0}, {0, 1, 0});
    REQUIRE(inside);
    CHECK(inside->distance == 0);
}

TEST_CASE("ray casts: rotated box gives the face hit point and normal") {
    World w;
    Quat q = Quat::fromAxisAngle({0, 0, 1}, M_PI / 4); // diamond in the xy-plane
    RigidBody& box = w.addBody(Shape::box({1, 1, 1}), 1, {0, 0, 0}, q);
    // Along the face normal (1, 1, 0)/sqrt 2 the face is at distance 1 from the centre.
    Vec3 n = normalize(Vec3{1, 1, 0});
    auto hit = raycast(box, n * 5, -n);
    REQUIRE(hit);
    CHECK(hit->distance == doctest::Approx(4));
    CHECK(near(hit->normal, n, 1e-9));
    // Straight down onto the top corner of the diamond at y = sqrt 2.
    auto top = raycast(box, {0, 5, 0}, {0, -1, 0});
    REQUIRE(top);
    CHECK(top->point.y == doctest::Approx(std::sqrt(2.0)));
    CHECK_FALSE(raycast(box, {3, 5, 0}, {0, -1, 0})); // passes beside it
    CHECK_FALSE(raycast(box, {0, 5, 0}, {0, 1, 0}));  // box behind the ray
}

TEST_CASE("World::raycast finds the nearest body and skips static ones unless asked") {
    World w;
    w.addBody(Shape::plane(), 0);
    RigidBody& far = w.addBody(Shape::box({0.5, 0.5, 0.5}), 1, {0, 0.5, -6});
    RigidBody& near_ = w.addBody(Shape::sphere(0.5), 1, {0, 0.5, -3});
    auto hit = w.raycast({0, 0.5, 5}, {0, 0, -1});
    REQUIRE(hit);
    CHECK(hit->body == &near_);
    w.removeBody(near_);
    hit = w.raycast({0, 0.5, 5}, {0, 0, -2}); // direction needn't be normalized
    REQUIRE(hit);
    CHECK(hit->body == &far);
    CHECK(hit->distance == doctest::Approx(10.5));
    // Looking down at empty ground: only found when static bodies are included.
    CHECK_FALSE(w.raycast({5, 3, 0}, {0, -1, 0}));
    auto ground = w.raycast({5, 3, 0}, {0, -1, 0}, INFINITY, true);
    REQUIRE(ground);
    CHECK(ground->point.y == doctest::Approx(0));
    CHECK_FALSE(w.raycast({0, 0.5, 5}, {0, 0, -1}, 5.0)); // the box is beyond maxDistance
}

TEST_CASE("a mouse joint pulls a body to the target and holds it") {
    World w;
    w.substeps = 4;
    RigidBody& box = w.addBody(Shape::box({0.3, 0.3, 0.3}), 5, {0, 1, 0});
    MouseJoint& grab = w.addMouseJoint(box, box.x + Vec3{0, 0.3, 0}, 5, 0.7, 30 * 5 * 9.81);
    grab.setTarget({2, 3, 0}); // a sudden 2.6 m jerk: it swings, then settles
    for (int i = 0; i < 240 * 4; ++i)
        w.step(kH);
    CHECK(length(grab.anchorB() - grab.target()) < 0.02); // held close (sags a little under gravity)
    CHECK(length(box.v()) < 0.01);
    CHECK(grab.force() == doctest::Approx(5 * 9.81).epsilon(0.02)); // carrying the weight
}

TEST_CASE("a mouse joint never pulls harder than its maximum force") {
    World w;
    w.substeps = 4;
    RigidBody& box = w.addBody(Shape::box({0.3, 0.3, 0.3}), 10, {0, 1, 0});
    const Real maxForce = 0.5 * 10 * 9.81; // half the weight
    MouseJoint& grab = w.addMouseJoint(box, box.x, 5, 0.7, maxForce);
    Real largest = 0;
    for (int i = 0; i < 240; ++i) {
        w.step(kH);
        largest = std::max(largest, grab.force());
    }
    CHECK(largest <= maxForce * (1 + 1e-9));
    // It sinks with a = g - F/m = g/2.
    CHECK(box.v().y == doctest::Approx(-0.5 * 9.81).epsilon(0.02));
}

TEST_CASE("a push changes momentum by exactly the impulse") {
    World w;
    w.gravity = {};
    RigidBody& box =
        w.addBody(Shape::box({0.5, 0.2, 0.3}), 2, {1, 2, 3}, Quat::fromAxisAngle({1, 1, 0}, 0.4));
    Vec3 P0 = box.P, L0 = box.L;
    const Vec3 J{0, 0, -6}, at = box.x + Vec3{0.3, 0.1, 0.2};
    box.applyImpulse(J, at);
    CHECK(near(box.P - P0, J));
    CHECK(near(box.L - L0, cross(at - box.x, J)));
}

TEST_CASE("removing a body removes its joints, springs and contacts") {
    World w;
    w.addBody(Shape::plane(), 0);
    RigidBody& a = w.addBody(Shape::box({0.3, 0.3, 0.3}), 1, {0, 0.3, 0});
    RigidBody& b = w.addBody(Shape::sphere(0.3), 1, {2, 0.3, 0});
    w.addBallJoint(w.ground(), a, {0, 2, 0});
    w.addSpring(a, a.x, b, b.x, 50, 1);
    w.addMouseJoint(b, b.x, 5, 0.7, 100);
    for (int i = 0; i < 10; ++i)
        w.step(kH);
    REQUIRE(w.contacts().size() == 2);
    w.removeBody(a);
    CHECK(w.bodies().size() == 2);
    CHECK(w.joints().size() == 1); // the mouse joint on b stays
    CHECK(w.springs().empty());
    for (const ContactManifold& m : w.contacts())
        CHECK((m.a != &a && m.b != &a));
    for (int i = 0; i < 240; ++i) // steps on without the removed body (checked under ASan)
        w.step(kH);
    CHECK(w.contacts().size() == 1);
}
