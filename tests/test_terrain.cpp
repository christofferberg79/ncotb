#include <doctest/doctest.h>

#include "world.h"

#include <cmath>
#include <functional>
#include <memory>

using namespace ncotb;

namespace {

constexpr Real kH = 1.0 / 240;
constexpr Real kG = 9.81;

/// A world with gravity and terrain from f over a cells x cells grid of 1 m cells.
struct TerrainWorld {
    World w;
    std::shared_ptr<const Heightfield> hf;
    RigidBody* ground;

    explicit TerrainWorld(const std::function<Real(Real, Real)>& f, int cells = 40, Real cellSize = 1) {
        w.gravity = {0, -kG, 0};
        w.substeps = 4;
        hf = std::make_shared<Heightfield>(Heightfield::fromFunction(cells, cells, cellSize, f));
        ground = &w.addBody(Shape::heightfield(hf), 0);
    }
    void run(Real seconds) {
        for (int i = 0; i < int(std::lround(seconds / kH)); ++i)
            w.step(kH);
    }
    /// Sum of the normal forces on body, along +y.
    Real support(const RigidBody& body) const {
        Real f = 0;
        for (const ContactManifold& m : w.contacts())
            if (m.a == &body)
                for (int i = 0; i < m.count; ++i)
                    f += m.points[i].normalImpulse * m.normal.y;
        return f / w.lastSubstep();
    }
};

Real flat(Real, Real) { return 0; }

} // namespace

TEST_CASE("heightfield: heights and normals on the triangles") {
    // A plane is reproduced exactly, everywhere.
    Heightfield plane =
        Heightfield::fromFunction(10, 8, 0.5, [](Real x, Real z) { return 1 + 0.1 * x + 0.2 * z; });
    CHECK(plane.nx() == 11);
    CHECK(plane.nz() == 9);
    CHECK(plane.halfSizeX() == doctest::Approx(2.5));
    for (Real x : {-2.5, -1.3, 0.0, 0.26, 2.5})
        for (Real z : {-2.0, -0.7, 0.1, 2.0})
            CHECK(plane.height(x, z) == doctest::Approx(1 + 0.1 * x + 0.2 * z));
    Vec3 n = plane.normal(0.3, 0.4);
    CHECK(length(n - normalize(Vec3{-0.1, 1, -0.2})) < 1e-12);

    // A saddle h = x z: exact at the vertices, linear along the cell diagonal.
    Heightfield saddle = Heightfield::fromFunction(4, 4, 1, [](Real x, Real z) { return x * z; });
    CHECK(saddle.height(1, -1) == doctest::Approx(-1));
    CHECK(saddle.height(0.5, 0.5) == doctest::Approx(0.5));    // on the diagonal from (0, 0) to (1, 1)
    CHECK(saddle.height(0.25, 0.75) == doctest::Approx(0.25)); // triangle (0,0) (0,1) (1,1): h = x
    CHECK(saddle.height(0.75, 0.25) == doctest::Approx(0.25)); // triangle (0,0) (1,1) (1,0): h = z

    CHECK(saddle.contains(2, -2));
    CHECK_FALSE(saddle.contains(2.01, 0));
    CHECK(saddle.minHeight() == -4);
    CHECK(saddle.maxHeight() == 4);
}

TEST_CASE("flat terrain holds bodies like the ground plane") {
    TerrainWorld t(flat);
    RigidBody& ball = t.w.addBody(Shape::sphere(0.3), 2, {0.37, 0.3, -0.61});
    RigidBody& box = t.w.addBody(Shape::box({0.4, 0.25, 0.3}), 3, {3.2, 0.25, 2.5});
    t.run(2);
    CHECK(ball.x.y == doctest::Approx(0.3).epsilon(0.02));
    CHECK(box.x.y == doctest::Approx(0.25).epsilon(0.02));
    CHECK(length(ball.v()) < 1e-3);
    CHECK(length(box.v()) < 1e-3);
    CHECK(t.support(ball) == doctest::Approx(2 * kG).epsilon(0.01));
    CHECK(t.support(box) == doctest::Approx(3 * kG).epsilon(0.01));
}

TEST_CASE("a ball rolls down a terrain slope at 5/7 g sin(theta)") {
    const Real theta = 0.2;
    TerrainWorld t([&](Real x, Real) { return -std::tan(theta) * x; });
    const Real r = 0.3;
    // Off the cell grid lines, and rolling across edges and diagonals.
    RigidBody& ball =
        t.w.addBody(Shape::sphere(r), 1, {-5.3, std::tan(theta) * 5.3 + r / std::cos(theta), 0.4});
    t.run(2);
    const Real a = 5.0 / 7 * kG * std::sin(theta);
    CHECK(length(ball.v()) == doctest::Approx(a * 2).epsilon(0.01));
    CHECK(std::abs(ball.v().z) < 1e-6);
    // Still on the surface, not hopping.
    CHECK(ball.x.y - t.hf->height(ball.x.x, ball.x.z) == doctest::Approx(r / std::cos(theta)).epsilon(0.01));
}

TEST_CASE("a box on a terrain slope sticks below the friction angle and slides above it") {
    // mu = sqrt(0.5 * 0.5) = 0.5: the friction angle is atan 0.5 = 26.6 degrees.
    for (Real deg : {20.0, 35.0}) {
        CAPTURE(deg);
        const Real theta = deg * M_PI / 180;
        TerrainWorld t([&](Real x, Real) { return -std::tan(theta) * x; });
        const Vec3 down = normalize(Vec3{1, -std::tan(theta), 0});
        const Vec3 n{std::sin(theta), std::cos(theta), 0};
        const Vec3 start = Vec3{0.2, -std::tan(theta) * 0.2, 0.3} + n * 0.2;
        RigidBody& box =
            t.w.addBody(Shape::box({0.3, 0.2, 0.3}), 1, start, Quat::fromAxisAngle({0, 0, 1}, -theta));
        t.run(1);
        const Real moved = dot(box.x - start, down);
        if (deg < 26) {
            CHECK(std::abs(moved) < 0.01);
        } else {
            const Real a = kG * (std::sin(theta) - 0.5 * std::cos(theta));
            CHECK(moved == doctest::Approx(0.5 * a).epsilon(0.03));
        }
    }
}

TEST_CASE("a ball crossing terrain triangle edges keeps rolling smoothly (no internal edges)") {
    TerrainWorld t(flat);
    const Real r = 0.25, speed = 6;
    const Vec3 dir = normalize(Vec3{1, 0, 0.45}); // across edges, rows and diagonals
    RigidBody& ball = t.w.addBody(Shape::sphere(r), 1, {-12, r, -6});
    ball.setLinearVelocity(dir * speed);
    ball.setAngularVelocity(cross(Vec3{0, 1, 0}, dir) * (speed / r)); // rolling, so friction is idle
    Real lowest = INFINITY, highest = -INFINITY;
    for (int i = 0; i < 2 * 240; ++i) {
        t.w.step(kH);
        lowest = std::min(lowest, ball.x.y);
        highest = std::max(highest, ball.x.y);
    }
    CHECK(highest - lowest < 1e-3);
    CHECK(length(ball.v()) == doctest::Approx(speed).epsilon(1e-3));
    CHECK(std::abs(dot(normalize(ball.v()), dir) - 1) < 1e-6);
}

TEST_CASE("a box sliding over terrain triangle edges doesn't catch on them") {
    TerrainWorld t(flat);
    RigidBody& box = t.w.addBody(Shape::box({0.4, 0.2, 0.3}), 1, {-10, 0.2, -3});
    box.friction = 0;
    const Vec3 v = normalize(Vec3{1, 0, 0.3}) * 5;
    box.setLinearVelocity(v);
    t.run(2);
    CHECK(length(box.v() - v) < 1e-3);
    CHECK(length(box.omega()) < 1e-3);
    CHECK(box.x.y == doctest::Approx(0.2).epsilon(0.02));
}

TEST_CASE("a ball rolling in a terrain bowl doesn't gain energy") {
    TerrainWorld t([](Real x, Real z) { return 0.05 * (x * x + z * z); });
    RigidBody& ball = t.w.addBody(Shape::sphere(0.3), 1, {4, 0.05 * 16 + 0.35, 1});
    t.run(0.5); // landed
    const Real e0 = t.w.totalEnergy();
    t.run(10);
    CHECK(t.w.totalEnergy() <= e0 * 1.001);
    CHECK(t.w.totalEnergy() > e0 * 0.9); // rolling: little is lost
    CHECK(ball.x.y - t.hf->height(ball.x.x, ball.x.z) > 0.25);
}

TEST_CASE("a box dropped into a terrain bowl comes to rest near the bottom") {
    TerrainWorld t([](Real x, Real z) { return 0.1 * (x * x + z * z); });
    t.ground->friction = 0.1;
    RigidBody& box = t.w.addBody(Shape::box({0.3, 0.2, 0.25}), 1, {2, 0.1 * 4 + 1, 0.5},
                                 Quat::fromAxisAngle({1, 2, 3}, 0.8));
    box.friction = 0.1;
    t.run(8);
    // mu = 0.1 holds it where the slope 0.2 r is below 0.1: within 0.5 m.
    CHECK(std::hypot(box.x.x, box.x.z) < 0.55);
    CHECK(length(box.v()) < 0.01);
}

TEST_CASE("a board resting on terrain peaks is held up by them") {
    // Four single-vertex spikes 0.3 m high under a 3 x 3 m board. Without the
    // vertex-in-box test the board would sink until its corners reach the ground.
    TerrainWorld t([](Real x, Real z) { return std::abs(x) == 1 && std::abs(z) == 1 ? 0.3 : 0.0; }, 10);
    RigidBody& board = t.w.addBody(Shape::box({1.5, 0.1, 1.5}), 5, {0.1, 0.4, -0.05});
    t.run(2);
    CHECK(board.x.y == doctest::Approx(0.4).epsilon(0.02));
    CHECK(length(board.v()) < 1e-3);
    CHECK(t.support(board) == doctest::Approx(5 * kG).epsilon(0.01));
}

TEST_CASE("a fast ball doesn't tunnel into a steep terrain slope at 30 steps/s") {
    for (bool sphere : {true, false}) {
        CAPTURE(sphere);
        TerrainWorld t([](Real x, Real) { return 0.8 * x; }, 40);
        Shape shape = sphere ? Shape::sphere(0.2) : Shape::box({0.2, 0.2, 0.2});
        RigidBody& body = t.w.addBody(shape, 1, {-6, 6, 0.3});
        body.setLinearVelocity(normalize(Vec3{1, -0.3, 0}) * 40); // into the hillside
        Real lowest = INFINITY;
        for (int i = 0; i < 60; ++i) {
            t.w.step(1.0 / 30);
            lowest = std::min(lowest, body.x.y - t.hf->height(body.x.x, body.x.z));
        }
        CHECK(lowest > 0.1); // the centre never got near (let alone under) the surface
    }
}

TEST_CASE("a body rolling off the edge of the terrain falls") {
    TerrainWorld t(flat, 10);
    RigidBody& ball = t.w.addBody(Shape::sphere(0.3), 1, {3, 0.3, 0.5});
    ball.setLinearVelocity({3, 0, 0});
    t.run(2);
    CHECK(ball.x.y < -3);
}

TEST_CASE("ray casts hit terrain on its surface") {
    TerrainWorld t([](Real x, Real z) { return std::sin(0.5 * x) + 0.3 * std::cos(0.7 * z); });
    RigidBody& terrain = *t.ground;
    // Straight down at many points.
    for (Real x = -19.5; x < 19.5; x += 2.3) {
        for (Real z = -19.5; z < 19.5; z += 3.1) {
            auto hit = raycast(terrain, {x, 10, z}, {0, -1, 0});
            REQUIRE(hit);
            CHECK(hit->point.y == doctest::Approx(t.hf->height(x, z)));
            CHECK(length(hit->normal - t.hf->normal(x, z)) < 1e-9);
        }
    }
    // At a grazing angle across many cells: the hit is on the surface, and
    // nothing was skipped before it.
    const Vec3 o{-19, 3, -15}, d = normalize(Vec3{1, -0.08, 0.7});
    auto hit = raycast(terrain, o, d);
    REQUIRE(hit);
    CHECK(hit->point.y == doctest::Approx(t.hf->height(hit->point.x, hit->point.z)));
    for (Real s = 0; s < hit->distance - 1e-6; s += 0.01) {
        Vec3 p = o + d * s;
        CHECK(p.y >= t.hf->height(p.x, p.z) - 1e-9);
    }
    // World::raycast finds it among static bodies; rays off the grid or from below miss.
    CHECK(t.w.raycast({0, 10, 0}, {0, -1, 0}, INFINITY, true));
    CHECK_FALSE(raycast(terrain, {30, 10, 0}, {0, -1, 0}));
    CHECK_FALSE(raycast(terrain, {0, -5, 0}, {0, 1, 0}));
}
