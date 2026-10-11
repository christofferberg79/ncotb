#include <doctest/doctest.h>

#include "world.h"

#include <cmath>

using namespace ncotb;

namespace {

/// Energy change after t seconds of free fall for one 1 kg ball.
Real freeFallEnergyChange(IntegratorType type, Real h, Real t) {
    World world;
    world.integrator = type;
    RigidBody& b = world.addBody(Shape::sphere(0.3), 1.0);
    b.setLinearVelocity({5, 9, 0});
    Real e0 = world.totalEnergy();
    for (int i = 0; i < int(std::lround(t / h)); ++i)
        world.step(h);
    return world.totalEnergy() - e0;
}

/// A torque-free box tumbling near its unstable intermediate axis.
World tumblingBox(IntegratorType type) {
    World world;
    world.gravity = {};
    world.integrator = type;
    RigidBody& b = world.addBody(Shape::box({0.6, 0.3, 0.1}), 1.0);
    b.setAngularVelocity({0.5, 4.0, 0.5});
    return world;
}

/// Orientation of the tumbling box after t seconds, as the rotated x and y axes.
std::pair<Vec3, Vec3> tumbleAxes(IntegratorType type, Real h, Real t) {
    World world = tumblingBox(type);
    for (int i = 0; i < int(std::lround(t / h)); ++i)
        world.step(h);
    const RigidBody& b = *world.bodies().front();
    return {b.R().column(0), b.R().column(1)};
}

Real orientationError(IntegratorType type, Real h) {
    static const auto ref = tumbleAxes(IntegratorType::RK4, 1.0 / 8000, 1.0);
    auto [ax, ay] = tumbleAxes(type, h, 1.0);
    return length(ax - ref.first) + length(ay - ref.second);
}

} // namespace

TEST_CASE("free fall: semi-implicit and explicit Euler drift at m g^2 h / 2") {
    const Real h = 1.0 / 240, t = 2.0, rate = 9.81 * 9.81 * h / 2;
    CHECK(freeFallEnergyChange(IntegratorType::SemiImplicitEuler, h, t) == doctest::Approx(-rate * t));
    CHECK(freeFallEnergyChange(IntegratorType::Euler, h, t) == doctest::Approx(rate * t));
}

TEST_CASE("free fall: Midpoint, Heun and RK4 are exact for a parabola") {
    for (IntegratorType type : {IntegratorType::Midpoint, IntegratorType::Heun, IntegratorType::RK4}) {
        INFO(name(type));
        CHECK(std::abs(freeFallEnergyChange(type, 1.0 / 240, 2.0)) < 1e-9);
    }
}

TEST_CASE("convergence order on a tumbling box") {
    struct Case {
        IntegratorType type;
        Real h;
        Real expectedRatio;
    };
    const Case cases[] = {
        {IntegratorType::Euler, 1.0 / 400, 2},
        {IntegratorType::Midpoint, 1.0 / 200, 4},
        {IntegratorType::Heun, 1.0 / 200, 4},
        {IntegratorType::RK4, 1.0 / 50, 16},
    };
    for (const Case& c : cases) {
        CAPTURE(name(c.type));
        Real ratio = orientationError(c.type, c.h) / orientationError(c.type, c.h / 2);
        CHECK(ratio == doctest::Approx(c.expectedRatio).epsilon(0.15));
    }
}

TEST_CASE("torque-free tumbling conserves |L| for every integrator") {
    for (IntegratorType type : kAllIntegrators) {
        INFO(name(type));
        World world = tumblingBox(type);
        Vec3 L0 = world.bodies().front()->L;
        for (int i = 0; i < 240 * 10; ++i)
            world.step(1.0 / 240);
        CHECK(length(world.bodies().front()->L - L0) < 1e-12);
    }
}

TEST_CASE("RK4 keeps the energy of a tumbling box within 0.1% over 10 s") {
    World world = tumblingBox(IntegratorType::RK4);
    Real e0 = world.totalEnergy();
    for (int i = 0; i < 240 * 10; ++i)
        world.step(1.0 / 240);
    CHECK(world.totalEnergy() == doctest::Approx(e0).epsilon(0.001));
}
