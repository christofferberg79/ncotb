#include <doctest/doctest.h>

#include "world.h"

using namespace ncotb;

TEST_CASE("free fall under gravity") {
    World world;
    RigidBody& b = world.addBody(Shape::sphere(0.5), 2.0);
    const Real h = 1.0 / 240;
    for (int i = 0; i < 240; ++i)
        world.step(h);
    CHECK(b.v().y == doctest::Approx(-9.81).epsilon(1e-9));
    // semi-implicit Euler: y = -g h^2 n(n+1)/2
    CHECK(b.x.y == doctest::Approx(-9.81 * h * h * 240 * 241 / 2));
}

TEST_CASE("static bodies do not move") {
    World world;
    RigidBody& b = world.addBody(Shape::box({1, 1, 1}), 0);
    world.step(0.1);
    CHECK(b.isStatic());
    CHECK(length(b.x) == 0);
}

TEST_CASE("torque-free spin conserves angular momentum and (nearly) energy") {
    World world;
    world.gravity = {};
    RigidBody& b = world.addBody(Shape::box({0.6, 0.3, 0.1}), 1.0);
    b.setAngularVelocity({0.1, 8.0, 0.1}); // near the intermediate axis
    Vec3 L0 = b.L;
    Real E0 = b.kineticEnergy();
    for (int i = 0; i < 240 * 10; ++i)
        world.step(1.0 / 240);
    CHECK(length(b.L - L0) < 1e-12);
    CHECK(b.kineticEnergy() == doctest::Approx(E0).epsilon(0.001));
}

TEST_CASE("intermediate axis is unstable (Dzhanibekov effect)") {
    World world;
    world.gravity = {};
    RigidBody& b = world.addBody(Shape::box({0.6, 0.3, 0.1}), 1.0);
    b.setAngularVelocity({0.01, 8.0, 0.01});
    // The flip is seen in body coordinates: in the world frame omega stays close to L.
    auto bodyOmegaY = [&] { return (transpose(b.R()) * b.omega()).y; };
    Real minOmegaY = bodyOmegaY();
    for (int i = 0; i < 240 * 10; ++i) {
        world.step(1.0 / 240);
        minOmegaY = std::min(minOmegaY, bodyOmegaY());
    }
    CHECK(minOmegaY < -4.0); // the spin about y has flipped at least once
}

TEST_CASE("major axis spin is stable") {
    World world;
    world.gravity = {};
    RigidBody& b = world.addBody(Shape::box({0.6, 0.3, 0.1}), 1.0);
    b.setAngularVelocity({0.01, 0.01, 8.0}); // z has the largest inertia
    for (int i = 0; i < 240 * 10; ++i) {
        world.step(1.0 / 240);
        REQUIRE((transpose(b.R()) * b.omega()).z > 7.0);
    }
}
