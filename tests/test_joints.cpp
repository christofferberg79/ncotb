#include <doctest/doctest.h>

#include "world.h"

#include <cmath>

using namespace ncotb;

namespace {

constexpr Real kH = 1.0 / 240;
constexpr Real kG = 9.81;
constexpr ContactSolverType kSolvers[] = {ContactSolverType::SequentialImpulses, ContactSolverType::BaraffLcp};

RigidBody& place(World& w, const Shape& shape, Real mass, const Vec3& x, const Quat& q = {}) {
    RigidBody& b = w.addBody(shape, mass);
    b.x = x;
    b.q = q;
    b.updateDerived();
    return b;
}

/// Period from successive downward zero crossings of f(t), over `seconds`.
template <class F> Real measurePeriod(World& w, Real seconds, F f) {
    Real prev = f(), first = -1, last = -1;
    int crossings = 0;
    for (int i = 0; i < int(seconds / kH); ++i) {
        w.step(kH);
        Real now = f();
        if (prev > 0 && now <= 0) {
            Real t = w.time() - kH * now / (now - prev); // interpolate the crossing
            if (first < 0) first = t;
            last = t;
            ++crossings;
        }
        prev = now;
    }
    return crossings > 1 ? (last - first) / (crossings - 1) : -1;
}

} // namespace

TEST_CASE("a ball on a ball joint swings with the pendulum period") {
    for (ContactSolverType solver : kSolvers) {
        INFO(name(solver));
        World w;
        w.solver.settings.type = solver;
        w.substeps = 4;
        const Real L = 1, r = 0.1, m = 1, theta0 = 5 * M_PI / 180;
        const Vec3 pivot{0, 2, 0};
        RigidBody& bob = place(w, Shape::sphere(r), m, pivot + Vec3{L * std::sin(theta0), -L * std::cos(theta0), 0});
        BallJoint& j = w.addBallJoint(w.ground(), bob, pivot);
        Real E0 = w.totalEnergy();
        Real period = measurePeriod(w, 10, [&] { return bob.x.x; });
        Real I = m * L * L + 0.4 * m * r * r; // about the pivot
        Real small = 2 * M_PI * std::sqrt(I / (m * kG * L));
        Real amplitude = 1 + theta0 * theta0 / 16; // finite-amplitude correction
        CHECK(period == doctest::Approx(small * amplitude).epsilon(0.003));
        CHECK(length(j.anchorA() - j.anchorB()) < 1e-3);
        CHECK(w.totalEnergy() == doctest::Approx(E0).epsilon(0.01));
    }
}

TEST_CASE("a bar on a hinge swings only about the hinge axis") {
    World w;
    w.substeps = 4;
    const Vec3 pivot{0, 2, 0};
    const Real m = 2, half = 0.5;
    RigidBody& bar = place(w, Shape::box({0.05, half, 0.05}), m, pivot - Vec3{0, half, 0});
    HingeJoint& hinge = w.addHinge(w.ground(), bar, pivot, {0, 0, 1});
    bar.setAngularVelocity({1.5, 0.8, 0.5}); // mostly off-axis: the hinge must remove it
    for (int i = 0; i < 24; ++i)
        w.step(kH);
    Vec3 wv = bar.omega();
    CHECK(std::hypot(wv.x, wv.y) < 1e-3 * std::max(Real(1), std::abs(wv.z)));
    CHECK(length(hinge.anchorA() - hinge.anchorB()) < 2e-3);
    CHECK(length(cross(hinge.axis(), bar.R().column(2))) < 1e-3);

    // physical pendulum period, small amplitude
    World w2;
    w2.substeps = 4;
    RigidBody& b2 =
        place(w2, Shape::box({0.05, half, 0.05}), m, pivot + Vec3{half * std::sin(0.05), -half * std::cos(0.05), 0},
              Quat::fromAxisAngle({0, 0, 1}, 0.05));
    w2.addHinge(w2.ground(), b2, pivot, {0, 0, 1});
    Real I = m * (1.0 * 1.0 + 0.1 * 0.1) / 12 + m * half * half;
    Real period = measurePeriod(w2, 10, [&] { return b2.x.x; });
    CHECK(period == doctest::Approx(2 * M_PI * std::sqrt(I / (m * kG * half))).epsilon(0.003));
}

TEST_CASE("a hanging chain is still and the top joint carries its whole weight") {
    for (ContactSolverType solver : kSolvers) {
        INFO(name(solver));
        World w;
        w.solver.settings.type = solver;
        w.substeps = 4;
        const int n = 10;
        const Real link = 0.3, m = 0.5;
        RigidBody* prev = &w.ground();
        BallJoint* top = nullptr;
        const Vec3 pivot{0, 4, 0};
        for (int i = 0; i < n; ++i) {
            RigidBody& b = place(w, Shape::box({0.04, link / 2, 0.04}), m, pivot - Vec3{0, link * (i + 0.5), 0});
            BallJoint& j = w.addBallJoint(*prev, b, pivot - Vec3{0, link * i, 0});
            if (i == 0) top = &j;
            prev = &b;
        }
        for (int i = 0; i < 240 * 2; ++i)
            w.step(kH);
        CHECK(top->impulse().y / w.lastSubstep() == doctest::Approx(n * m * kG).epsilon(0.01));
        CHECK(prev->x.y == doctest::Approx(pivot.y - link * (n - 0.5)).epsilon(0.002)); // no stretch
        CHECK(length(prev->v()) < 1e-3);
    }
}

TEST_CASE("hinge limits stop a door, and a motor drives a wheel") {
    World w;
    w.gravity = {};
    w.substeps = 4;
    // Door: hinge along y at its left edge, may open 0..90 degrees.
    RigidBody& door = place(w, Shape::box({0.45, 1, 0.03}), 10, {0.45, 1, 0});
    HingeJoint& hinge = w.addHinge(w.ground(), door, {0, 1, 0}, {0, 1, 0});
    hinge.setLimits(0, M_PI / 2);
    // Spinning about the hinge, not about its centre: v = omega x r. (Spin about
    // the centre alone would become 3 I_cm / I_hinge = 0.75 rad/s in the first
    // step, as angular momentum about the hinge is conserved.)
    door.setAngularVelocity({0, 3, 0});
    door.setLinearVelocity(cross(Vec3{0, 3, 0}, door.x - Vec3{0, 1, 0}));
    for (int i = 0; i < 240 * 2; ++i)
        w.step(kH);
    CHECK(hinge.angle() == doctest::Approx(M_PI / 2).epsilon(0.02));
    CHECK(std::abs(door.omega().y) < 0.05);

    // Wheel: motor at 5 rad/s with limited torque: spins up at tau / I, then holds the speed.
    RigidBody& wheel = place(w, Shape::box({0.3, 0.3, 0.05}), 2, {3, 1, 0});
    HingeJoint& axle = w.addHinge(w.ground(), wheel, {3, 1, 0}, {0, 0, 1});
    const Real tau = 0.5, I = 2 * (0.6 * 0.6 + 0.6 * 0.6) / 12;
    axle.setMotor(5, tau);
    w.step(kH);
    for (int i = 0; i < 59; ++i)
        w.step(kH);
    CHECK(wheel.omega().z == doctest::Approx(tau / I * 60 * kH).epsilon(0.02)); // still spinning up
    for (int i = 0; i < 240 * 3; ++i)
        w.step(kH);
    CHECK(wheel.omega().z == doctest::Approx(5).epsilon(0.001));
}

TEST_CASE("a slider allows motion only along its axis, with limits and a motor") {
    World w;
    w.substeps = 4;
    const Real angle = 30 * M_PI / 180;
    const Vec3 axis{std::cos(angle), -std::sin(angle), 0}; // downhill
    RigidBody& box = place(w, Shape::box({0.2, 0.1, 0.1}), 1, {0, 2, 0}, Quat::fromAxisAngle({0, 1, 0}, 0.3));
    SliderJoint& slider = w.addSlider(w.ground(), box, box.x, axis);
    slider.setLimits(-1, 1.5);
    Quat q0 = box.q;
    for (int i = 0; i < 120; ++i)
        w.step(kH);
    // frictionless slope: x = g sin(angle) t^2 / 2 along the axis
    CHECK(slider.position() == doctest::Approx(0.5 * kG * std::sin(angle) * 0.25).epsilon(0.01));
    Vec3 off = (box.x - Vec3{0, 2, 0}) - axis * slider.position();
    CHECK(length(off) < 1e-3);
    CHECK(std::abs(dot(box.q, q0)) > 1 - 1e-6); // no rotation
    for (int i = 0; i < 240 * 2; ++i)
        w.step(kH);
    CHECK(slider.position() == doctest::Approx(1.5).epsilon(0.01)); // stopped at the limit
    CHECK(length(box.v()) < 0.02);

    // Motor: drive it back up the slope at 1 m/s (needs m g sin = 4.9 N).
    slider.setMotor(-1, 20);
    for (int i = 0; i < 120; ++i)
        w.step(kH);
    CHECK(dot(box.v(), axis) == doctest::Approx(-1).epsilon(0.01));
}

TEST_CASE("a mass on a spring: equilibrium m g / k and period 2 pi sqrt(m / k)") {
    for (IntegratorType integrator : {IntegratorType::SemiImplicitEuler, IntegratorType::RK4}) {
        INFO(name(integrator));
        World w;
        w.integrator = integrator;
        const Real k = 100, m = 1;
        const Vec3 top{0, 3, 0};
        RigidBody& mass = place(w, Shape::sphere(0.1), m, top - Vec3{0, 1, 0});
        w.addSpring(w.ground(), top, mass, mass.x, k, 0, 1.0); // at rest length: it will drop and oscillate
        const Real equilibrium = top.y - 1 - m * kG / k;
        Real period = measurePeriod(w, 5, [&] { return mass.x.y - equilibrium; });
        CHECK(period == doctest::Approx(2 * M_PI * std::sqrt(m / k)).epsilon(0.005));
    }

    // Critically damped: settles at the equilibrium without overshooting.
    World w;
    const Real k = 100, m = 1, c = 2 * std::sqrt(k * m);
    const Vec3 top{0, 3, 0};
    RigidBody& mass = place(w, Shape::sphere(0.1), m, top - Vec3{0, 1, 0});
    w.addSpring(w.ground(), top, mass, mass.x, k, c, 1.0);
    const Real equilibrium = top.y - 1 - m * kG / k;
    Real lowest = mass.x.y;
    for (int i = 0; i < 240 * 3; ++i) {
        w.step(kH);
        lowest = std::min(lowest, mass.x.y);
    }
    CHECK(lowest >= equilibrium - 1e-4);
    CHECK(mass.x.y == doctest::Approx(equilibrium).epsilon(1e-4));
}

TEST_CASE("joints and contacts work together: a hinged flap rests on a box") {
    World w;
    w.substeps = 4;
    w.addBody(Shape::plane(), 0);
    place(w, Shape::box({0.3, 0.3, 0.3}), 0, {1.2, 0.3, 0}); // static support
    // 1.4 m flap hinged at x = 0, y = 0.6, resting on the support's top at x = 1.2
    RigidBody& flap = place(w, Shape::box({0.7, 0.02, 0.2}), 3, {0.7, 0.62, 0});
    w.addHinge(w.ground(), flap, {0, 0.62, 0}, {0, 0, 1});
    for (int i = 0; i < 240 * 2; ++i)
        w.step(kH);
    CHECK(length(flap.v()) < 1e-3);
    CHECK(flap.x.y == doctest::Approx(0.62).epsilon(0.005));
}

TEST_CASE("bodies connected by a joint don't collide with each other") {
    World w;
    w.gravity = {};
    RigidBody& a = place(w, Shape::box({0.5, 0.5, 0.5}), 1, {0, 0, 0});
    RigidBody& b = place(w, Shape::box({0.5, 0.5, 0.5}), 1, {0.9, 0, 0}); // overlapping by 10 cm
    w.detectCollisions();
    CHECK(w.contacts().size() == 1);
    w.addBallJoint(a, b, {0.45, 0, 0});
    w.detectCollisions();
    CHECK(w.contacts().empty());
}

TEST_CASE("Newton's cradle passes the momentum to the last ball (gaps wider than the margin)") {
    for (ContactSolverType solver : kSolvers) {
        INFO(name(solver));
        World w;
        w.solver.settings.type = solver;
        w.substeps = 4;
        const Real r = 0.2, len = 1.6, gap = 0.03, a0 = 0.6;
        std::vector<RigidBody*> balls;
        for (int i = 0; i < 5; ++i) {
            Vec3 pivot{i * (2 * r + gap), 4, 0};
            Real a = i == 0 ? -a0 : 0;
            RigidBody& b = place(w, Shape::sphere(r), 1, pivot + Vec3{len * std::sin(a), -len * std::cos(a), 0});
            b.restitution = 1;
            b.friction = 0;
            w.addBallJoint(w.ground(), b, pivot);
            balls.push_back(&b);
        }
        Real lastPeak = 0, othersAfter = 0;
        for (int i = 0; i < 240; ++i) {
            w.step(kH);
            lastPeak = std::max(lastPeak, balls[4]->v().x);
            if (w.time() > 0.75)
                for (int k = 0; k < 4; ++k)
                    othersAfter = std::max(othersAfter, std::abs(balls[k]->v().x));
        }
        Real ideal = std::sqrt(2 * kG * len * (1 - std::cos(a0)));
        CHECK(lastPeak > 0.95 * ideal);
        CHECK(othersAfter < 0.05 * ideal);
    }
}
