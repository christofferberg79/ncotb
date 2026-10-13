#include <doctest/doctest.h>

#include "vehicle/car.h"

#include <cmath>
#include <memory>

using namespace ncotb;

namespace {

constexpr Real kH = 1.0 / 240;
constexpr Real kG = 9.81;

/// A world with the car scene's solver settings and flat ground.
struct CarWorld {
    World w;
    std::unique_ptr<Car> car;

    explicit CarWorld(bool terrain = false, Vec3 start = {}, Real heading = 0) {
        w.gravity = {0, -kG, 0};
        w.substeps = 4;
        w.solver.settings.iterations = 60; // the truck's joint loops need them (see docs/notes.md)
        if (terrain) {
            auto hf = std::make_shared<const Heightfield>(Heightfield::hills(80, 0.5, 1.2, 8, 3));
            w.addBody(Shape::heightfield(hf), 0).friction = 0.8;
            start.y = hf->height(start.x, start.z) + 0.02;
        } else {
            w.addBody(Shape::plane(), 0).friction = 0.8;
        }
        car = std::make_unique<Car>(w, CarSpec{}, start, heading);
    }
    /// Steps with the given input; returns the largest joint error seen.
    Real run(Real seconds, Real throttle = 0, Real steer = 0, Real brake = 0) {
        Real worst = 0;
        for (int i = 0; i < int(std::lround(seconds / kH)); ++i) {
            car->drive(throttle, steer, brake);
            w.step(kH);
            worst = std::max(worst, car->jointError());
        }
        return worst;
    }
};

} // namespace

TEST_CASE("the car stands level on its suspension, arms horizontal") {
    CarWorld t;
    const Real worst = t.run(3);
    Car& car = *t.car;
    CHECK(worst < 1e-3);
    CHECK(length(car.chassis().v()) < 1e-3);
    CHECK(length(car.chassis().omega()) < 1e-3);
    // The shocks were tuned to hold it at the design pose.
    CHECK(car.chassis().x.y == doctest::Approx(car.spec().wheelRadius + car.spec().armDrop).epsilon(0.03));
    CHECK(std::abs(car.chassis().R().column(1).y - 1) < 1e-4);
    for (int c = 0; c < 4; ++c) {
        CAPTURE(c);
        CHECK(std::abs(car.suspensionTravel(c)) < 0.004);
    }
}

TEST_CASE("the car's joints hold after a drop") {
    CarWorld t(false, {0, 0.3, 0});
    const Real worst = t.run(2);
    CHECK(worst < 0.005);
    CHECK(t.car->jointError() < 1e-3);
    CHECK(length(t.car->chassis().v()) < 0.01); // and the bouncing has died down
}

TEST_CASE("full throttle: the car drives straight and approaches its top speed") {
    CarWorld t;
    t.run(0.5);
    const Real heading0 = t.car->heading();
    t.run(1, 1);
    const Real early = t.car->speed();
    CHECK(early > 3);
    t.run(5, 1);
    // Where the motor's torque T (1 - w / w_max) equals its drag c w.
    const CarSpec& s = t.car->spec();
    const Real maxSpin = s.topSpeed / s.wheelRadius;
    const Real top = s.topSpeed * s.motorTorque / (s.motorTorque + s.coastDrag * maxSpin);
    CHECK(t.car->speed() > 0.95 * top);
    CHECK(t.car->speed() < top * 1.01);
    CHECK(std::abs(t.car->heading() - heading0) < 1 * M_PI / 180);
    CHECK(std::abs(t.car->chassis().x.z) < 0.5); // little sideways drift over ~50 m
}

TEST_CASE("braking from 5 m/s stops the car in about v^2 / (2 mu g)") {
    CarWorld t;
    t.run(0.5);
    while (t.car->speed() < 5)
        t.run(kH, 1);
    const Real x0 = t.car->chassis().x.x, v0 = t.car->speed();
    t.run(3, 0, 0, 1);
    CHECK(std::abs(t.car->speed()) < 0.01);
    // Brakes that lock the wheels: friction limits the deceleration to mu g,
    // mu = sqrt(1.0 * 0.8). The brake torque may be a little short of that.
    const Real mu = std::sqrt(0.8);
    const Real distance = t.car->chassis().x.x - x0;
    CHECK(distance >= v0 * v0 / (2 * mu * kG) * 0.95);
    CHECK(distance < v0 * v0 / (2 * mu * kG) * 1.6);
}

TEST_CASE("steering: Ackermann angles and the turning circle") {
    CarWorld t;
    t.run(0.5);
    t.run(2, 0.15, 1); // slowly, full left
    Car& car = *t.car;
    const Real left = car.steering(0).angle(), right = car.steering(1).angle();
    CHECK(left > 0);
    CHECK(right > 0);
    CHECK(left > right); // the inner (left) wheel turns more
    // Both wheels point at one centre on the rear axle line:
    // tan(left) = L / (R - k), tan(right) = L / (R + k).
    const CarSpec& s = car.spec();
    const Real R = s.wheelbase / std::tan(s.maxSteer);
    CHECK(left == doctest::Approx(std::atan(s.wheelbase / (R - s.kingpin))).epsilon(0.03));
    CHECK(right == doctest::Approx(std::atan(s.wheelbase / (R + s.kingpin))).epsilon(0.03));

    // The yaw rate gives the turning radius of the rear axle's middle: v / omega.
    t.run(3, 0.15, 1);
    const Real omega = car.chassis().omega().y;
    CHECK(omega > 0); // turning left
    const Vec3 rearAxle = car.chassis().x - car.forward() * (s.wheelbase / 2);
    const Vec3 vRear = car.chassis().v() + cross(car.chassis().omega(), rearAxle - car.chassis().x);
    const Real radius = length(vRear) / omega;
    CHECK(radius == doctest::Approx(R).epsilon(0.15));
}

TEST_CASE("the car stays in one piece on rough terrain") {
    CarWorld t(true, {0, 0, 0}, 0.3);
    t.run(0.5);
    Real worst = 0, lowest = INFINITY;
    for (int i = 0; i < 10 * 240; ++i) {
        // Loops of changing size at full throttle, so it stays on the 40 m terrain.
        const Real steer = 0.6 + 0.4 * std::sin(i * kH * 1.3);
        t.car->drive(1, steer, 0);
        t.w.step(kH);
        worst = std::max(worst, t.car->jointError());
        const Heightfield& hf = *t.w.bodies()[0]->shape().terrain;
        for (const RigidBody* wheel : t.car->wheels())
            if (hf.contains(wheel->x.x, wheel->x.z))
                lowest = std::min(lowest, wheel->x.y - hf.height(wheel->x.x, wheel->x.z));
    }
    CHECK(worst < 0.01);       // joints stay together (within 1 cm at worst)
    CHECK(lowest > 0.02);      // no wheel sinks into the ground
    CHECK(t.car->intact(t.w)); // trivially, but checks intact()
    const Heightfield& hf = *t.w.bodies()[0]->shape().terrain;
    CHECK(hf.contains(t.car->chassis().x.x, t.car->chassis().x.z));
    CHECK(length(t.car->chassis().v()) < 15);
}

TEST_CASE("the car is deterministic") {
    auto path = [] {
        CarWorld t(true, {0, 0, 0}, 0.3);
        for (int i = 0; i < 3 * 240; ++i) {
            t.car->drive(0.8, std::sin(i * 0.01), i > 600 ? 1 : 0);
            t.w.step(kH);
        }
        return t.car->chassis().x;
    };
    Vec3 a = path(), b = path();
    CHECK(a.x == b.x);
    CHECK(a.y == b.y);
    CHECK(a.z == b.z);
}
