#include <doctest/doctest.h>

#include "body/rigid_body.h"
#include "collision/collide.h"
#include "world.h"

#include <cmath>
#include <random>

using namespace ncotb;

namespace {

bool near(const Vec3& a, const Vec3& b, Real eps = 1e-9) { return length(a - b) < eps; }

RigidBody sphere(Real r, const Vec3& x) {
    RigidBody b(Shape::sphere(r), 1);
    b.x = x;
    b.updateDerived();
    return b;
}

RigidBody box(const Vec3& h, const Vec3& x, const Quat& q = {}) {
    RigidBody b(Shape::box(h), 1);
    b.x = x;
    b.q = q;
    b.updateDerived();
    return b;
}

RigidBody plane(const Vec3& x = {}, const Quat& q = {}) {
    RigidBody b(Shape::plane(), 0);
    b.x = x;
    b.q = q;
    b.updateDerived();
    return b;
}

/// Signed distance from p to the surface of a box (negative inside).
Real boxDistance(const RigidBody& b, const Vec3& p) {
    Vec3 local = transpose(b.R()) * (p - b.x);
    const Vec3& h = b.shape().halfExtents;
    Vec3 q{std::abs(local.x) - h.x, std::abs(local.y) - h.y, std::abs(local.z) - h.z};
    Vec3 outside{std::max(q.x, Real(0)), std::max(q.y, Real(0)), std::max(q.z, Real(0))};
    return length(outside) + std::min(std::max(q.x, std::max(q.y, q.z)), Real(0));
}

} // namespace

TEST_CASE("sphere-sphere") {
    RigidBody a = sphere(1, {1.5, 0, 0}), b = sphere(1, {0, 0, 0});
    ContactManifold m;
    REQUIRE(collide(a, b, m));
    CHECK(m.a == &a);
    CHECK(m.count == 1);
    CHECK(near(m.normal, {1, 0, 0})); // from b towards a
    CHECK(m.points[0].depth == doctest::Approx(0.5));
    CHECK(near(m.points[0].position, {0.75, 0, 0}));

    RigidBody far = sphere(1, {2.1, 0, 0});
    CHECK_FALSE(collide(far, b, m));
    CHECK(collide(far, b, m, 0.2)); // within the margin
    CHECK(m.points[0].depth == doctest::Approx(-0.1));
}

TEST_CASE("sphere-plane, both argument orders") {
    Quat tilt = Quat::fromAxisAngle({0, 0, 1}, 0.3);
    RigidBody p = plane({0, 1, 0}, tilt);
    Vec3 n = rotate(tilt, {0, 1, 0});
    RigidBody s = sphere(0.5, Vec3{0, 1, 0} + n * 0.4);
    ContactManifold m;
    REQUIRE(collide(s, p, m));
    CHECK(near(m.normal, n));
    CHECK(m.points[0].depth == doctest::Approx(0.1));
    REQUIRE(collide(p, s, m));
    CHECK(near(m.normal, -n)); // now from the sphere towards the plane
    CHECK(m.a == &p);
}

TEST_CASE("sphere-box: face, edge and centre inside") {
    RigidBody b = box({1, 1, 1}, {0, 0, 0});
    ContactManifold m;

    RigidBody s = sphere(0.5, {0, 1.3, 0});
    REQUIRE(collide(s, b, m));
    CHECK(near(m.normal, {0, 1, 0}));
    CHECK(m.points[0].depth == doctest::Approx(0.2));
    CHECK(near(m.points[0].position, {0, 0.9, 0}));

    s = sphere(0.5, {1.3, 1.3, 0}); // over the edge x = y = 1
    REQUIRE(collide(s, b, m));
    CHECK(near(m.normal, normalize(Vec3{1, 1, 0})));
    CHECK(m.points[0].depth == doctest::Approx(0.5 - 0.3 * std::sqrt(2.0)));

    s = sphere(0.25, {0, 0, 0.9}); // centre inside, nearest face is +z
    REQUIRE(collide(s, b, m));
    CHECK(near(m.normal, {0, 0, 1}));
    CHECK(m.points[0].depth == doctest::Approx(0.35));

    s = sphere(0.5, {0, 1.6, 0});
    CHECK_FALSE(collide(s, b, m));
}

TEST_CASE("box-plane: face, edge and vertex") {
    RigidBody p = plane();
    ContactManifold m;

    RigidBody b = box({1, 0.5, 1}, {0, 0.49, 0});
    REQUIRE(collide(b, p, m));
    CHECK(m.count == 4);
    CHECK(near(m.normal, {0, 1, 0}));
    for (int i = 0; i < m.count; ++i)
        CHECK(m.points[i].depth == doctest::Approx(0.01));

    // Resting on an edge: rotated 45 degrees about z.
    Real r = std::sqrt(2.0) / 2;
    b = box({0.5, 0.5, 0.5}, {0, r - 0.01, 0}, Quat::fromAxisAngle({0, 0, 1}, M_PI / 4));
    REQUIRE(collide(b, p, m));
    CHECK(m.count == 2);

    // Resting on a vertex: diagonal pointing down.
    Quat corner = Quat::fromAxisAngle({1, 0, -1}, std::acos(1 / std::sqrt(3.0)));
    b = box({0.5, 0.5, 0.5}, {0, std::sqrt(3.0) / 2 - 0.01, 0}, corner);
    REQUIRE(collide(b, p, m));
    CHECK(m.count == 1);
    CHECK(m.points[0].depth == doctest::Approx(0.01).epsilon(1e-6));

    b = box({0.5, 0.5, 0.5}, {0, 0.6, 0});
    CHECK_FALSE(collide(b, p, m));
}

TEST_CASE("box-box: stacked face to face") {
    RigidBody bottom = box({1, 0.5, 1}, {0, 0, 0});
    RigidBody top = box({0.5, 0.5, 0.5}, {0.2, 0.98, 0.1});
    ContactManifold m;
    REQUIRE(collide(top, bottom, m));
    CHECK(near(m.normal, {0, 1, 0}));
    CHECK(m.count == 4);
    for (int i = 0; i < m.count; ++i) {
        CHECK(m.points[i].depth == doctest::Approx(0.02));
        CHECK(m.points[i].position.y == doctest::Approx(0.49));
    }
    // Same pair the other way round: normal flips, points stay.
    REQUIRE(collide(bottom, top, m));
    CHECK(near(m.normal, {0, -1, 0}));
    CHECK(m.count == 4);
}

TEST_CASE("box-box: top box twisted 45 degrees keeps 4 points spanning the overlap") {
    RigidBody bottom = box({0.5, 0.5, 0.5}, {0, 0, 0});
    RigidBody top = box({0.5, 0.5, 0.5}, {0, 0.99, 0}, Quat::fromAxisAngle({0, 1, 0}, M_PI / 4));
    ContactManifold m;
    REQUIRE(collide(top, bottom, m));
    CHECK(near(m.normal, {0, 1, 0}));
    CHECK(m.count == 4); // the clipped octagon is reduced to 4 points
    Real maxExtent = 0;
    for (int i = 0; i < m.count; ++i) {
        CHECK(m.points[i].depth == doctest::Approx(0.01));
        maxExtent = std::max(maxExtent, std::hypot(m.points[i].position.x, m.points[i].position.z));
    }
    CHECK(maxExtent > 0.5); // not all bunched in the middle
}

TEST_CASE("box-box: crossed edges give one point") {
    // Two unit cubes, each turned 45 degrees so an edge points at the other,
    // with the edges crossing at right angles.
    Real d = std::sqrt(2.0) / 2;
    RigidBody a = box({0.5, 0.5, 0.5}, {0, 2 * d - 0.02, 0}, Quat::fromAxisAngle({1, 0, 0}, M_PI / 4));
    RigidBody b = box({0.5, 0.5, 0.5}, {0, 0, 0}, Quat::fromAxisAngle({0, 0, 1}, M_PI / 4));
    ContactManifold m;
    REQUIRE(collide(a, b, m));
    CHECK(m.count == 1);
    CHECK(near(m.normal, {0, 1, 0}, 1e-6));
    CHECK(m.points[0].depth == doctest::Approx(0.02).epsilon(1e-6));
    CHECK(near(m.points[0].position, {0, d - 0.01, 0}, 1e-6));
}

TEST_CASE("box-box: crossed edges with a small gap are separated only by an edge axis") {
    Real d = std::sqrt(2.0) / 2;
    RigidBody a = box({0.5, 0.5, 0.5}, {0, 2 * d + 0.02, 0}, Quat::fromAxisAngle({1, 0, 0}, M_PI / 4));
    RigidBody b = box({0.5, 0.5, 0.5}, {0, 0, 0}, Quat::fromAxisAngle({0, 0, 1}, M_PI / 4));
    ContactManifold m;
    CHECK_FALSE(collide(a, b, m));
    REQUIRE(collide(a, b, m, 0.05)); // inside the margin: reported with negative depth
    CHECK(m.count == 1);
    CHECK(m.points[0].depth == doctest::Approx(-0.02).epsilon(1e-6));
}

TEST_CASE("box-box: random overlaps give sane manifolds") {
    std::mt19937 rng(1234);
    std::uniform_real_distribution<Real> u(-1, 1);
    auto randomQuat = [&] { return normalize(Quat{u(rng), {u(rng), u(rng), u(rng)}}); };
    int hits = 0;
    for (int trial = 0; trial < 2000; ++trial) {
        RigidBody a =
            box({0.3 + 0.2 * std::abs(u(rng)), 0.3 + 0.2 * std::abs(u(rng)), 0.3 + 0.2 * std::abs(u(rng))},
                {u(rng), u(rng), u(rng)}, randomQuat());
        RigidBody b =
            box({0.3 + 0.2 * std::abs(u(rng)), 0.3 + 0.2 * std::abs(u(rng)), 0.3 + 0.2 * std::abs(u(rng))},
                {0, 0, 0}, randomQuat());
        ContactManifold m;
        if (!collide(a, b, m)) continue;
        ++hits;
        CAPTURE(trial);
        REQUIRE(m.count >= 1);
        REQUIRE(m.count <= 4);
        CHECK(length(m.normal) == doctest::Approx(1));
        for (int i = 0; i < m.count; ++i) {
            const ContactPoint& p = m.points[i];
            CHECK(p.depth >= -1e-9);
            // The point is midway between the surfaces, so it is inside or on
            // both boxes, at most half the depth from each surface.
            CHECK(boxDistance(a, p.position) <= p.depth / 2 + 1e-6);
            CHECK(boxDistance(b, p.position) <= p.depth / 2 + 1e-6);
        }
    }
    CHECK(hits > 300);
}

TEST_CASE("World reports contacts with the dynamic body as a") {
    World world;
    RigidBody& ground = world.addBody(Shape::plane(), 0); // added first, so it would be a
    RigidBody& ball = world.addBody(Shape::sphere(0.5), 1);
    ball.x = {0, 0.4, 0};
    ball.updateDerived();
    world.detectCollisions();
    REQUIRE(world.contacts().size() == 1);
    CHECK(world.contacts()[0].a == &ball);
    CHECK(world.contacts()[0].b == &ground);
    CHECK(near(world.contacts()[0].normal, {0, 1, 0})); // out of the ground
}
