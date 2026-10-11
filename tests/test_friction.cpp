#include <doctest/doctest.h>

#include "world.h"

#include <cmath>

using namespace ncotb;

namespace {

constexpr Real kH = 1.0 / 240;
constexpr Real kG = 9.81;
constexpr SolverType kSolvers[] = {SolverType::SequentialImpulses, SolverType::BaraffLcp};

/// A world with a ground plane tilted by theta about z, and both bodies' mu set
/// so the combined sqrt(mu_a mu_b) is mu.
struct Ramp {
    World w;
    RigidBody* ground;
    Vec3 n;    ///< ramp normal
    Vec3 down; ///< unit vector down the slope
    Quat q;    ///< ramp orientation

    Ramp(SolverType solver, Real theta) {
        w.solver.settings.type = solver;
        w.substeps = 4;
        q = Quat::fromAxisAngle({0, 0, 1}, theta);
        ground = &w.addBody(Shape::plane(), 0);
        ground->q = q;
        ground->restitution = 0;
        ground->updateDerived();
        n = rotate(q, {0, 1, 0});
        down = -rotate(q, {1, 0, 0}); // tilting about +z lifts +x
    }

    RigidBody& add(const Shape& shape, Real mass, Real mu, Real height) {
        RigidBody& b = w.addBody(shape, mass);
        b.friction = mu;
        b.restitution = 0;
        ground->friction = mu;
        b.x = n * height;
        b.q = q;
        b.updateDerived();
        return b;
    }

    void run(Real seconds) {
        for (int i = 0; i < int(std::lround(seconds / kH)); ++i)
            w.step(kH);
    }
};

/// Acceleration down the slope, measured between t = 0.5 s and 1.5 s.
Real slopeAcceleration(Ramp& r, const RigidBody& b) {
    r.run(0.5);
    Real v0 = dot(b.v(), r.down);
    r.run(1.0);
    return dot(b.v(), r.down) - v0;
}

} // namespace

TEST_CASE("a box stays put on a ramp when tan(theta) < mu") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        Ramp r(solver, 20 * M_PI / 180); // tan 20 deg = 0.36
        RigidBody& box = r.add(Shape::box({0.4, 0.25, 0.3}), 2, 0.5, 0.25);
        Vec3 start = box.x;
        r.run(2);
        CHECK(length(box.v()) < 1e-3);
        CHECK(length(box.x - start) < 2e-3);
    }
}

TEST_CASE("a box slides down a ramp with a = g (sin theta - mu cos theta)") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        const Real theta = 30 * M_PI / 180, mu = 0.2;
        Ramp r(solver, theta);
        RigidBody& box = r.add(Shape::box({0.4, 0.25, 0.3}), 2, mu, 0.25);
        CHECK(slopeAcceleration(r, box) ==
              doctest::Approx(kG * (std::sin(theta) - mu * std::cos(theta))).epsilon(0.02));
    }
}

TEST_CASE("without friction the box slides with a = g sin theta") {
    Ramp r(SolverType::SequentialImpulses, 20 * M_PI / 180);
    r.w.solver.settings.friction = false;
    RigidBody& box = r.add(Shape::box({0.4, 0.25, 0.3}), 2, 0.5, 0.25);
    CHECK(slopeAcceleration(r, box) == doctest::Approx(kG * std::sin(20 * M_PI / 180)).epsilon(0.01));
}

TEST_CASE("a ball rolls down a ramp without slipping, a = 5/7 g sin theta") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        const Real theta = 20 * M_PI / 180, radius = 0.3;
        Ramp r(solver, theta);
        RigidBody& ball = r.add(Shape::sphere(radius), 1, 0.5, radius);
        CHECK(slopeAcceleration(r, ball) == doctest::Approx(5.0 / 7 * kG * std::sin(theta)).epsilon(0.02));
        // Rolling: the contact point is at rest.
        Vec3 contactVelocity = ball.v() + cross(ball.omega(), -r.n * radius);
        CHECK(length(contactVelocity) < 0.01 * length(ball.v()));
    }
}

TEST_CASE("a ball skidding without spin ends up rolling at 5/7 of its speed") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        const Real v0 = 4, mu = 0.3, radius = 0.3;
        Ramp r(solver, 0);
        RigidBody& ball = r.add(Shape::sphere(radius), 1, mu, radius);
        ball.setLinearVelocity({v0, 0, 0});
        // Sliding friction mu g slows it and spins it up until it rolls, after
        // t = 2 v0 / (7 mu g) = 0.39 s.
        r.run(1);
        CHECK(ball.v().x == doctest::Approx(5.0 / 7 * v0).epsilon(0.01));
        CHECK(ball.omega().z == doctest::Approx(-ball.v().x / radius).epsilon(0.01));
    }
}

TEST_CASE("a sliding box stops after v0^2 / (2 mu g), and friction stays inside the cone") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        const Real v0 = 3, mu = 0.4;
        Ramp r(solver, 0);
        RigidBody& box = r.add(Shape::box({0.4, 0.25, 0.3}), 2, mu, 0.25);
        box.setLinearVelocity({v0, 0, 0});
        bool insideCone = true;
        for (int i = 0; i < 240 * 2; ++i) {
            r.w.step(kH);
            for (const ContactManifold& m : r.w.contacts())
                for (int k = 0; k < m.count; ++k)
                    insideCone &=
                        length(m.points[k].tangentImpulse) <= mu * m.points[k].normalImpulse * (1 + 1e-9);
        }
        CHECK(insideCone);
        CHECK(length(box.v()) < 1e-3);
        CHECK(box.x.x == doctest::Approx(v0 * v0 / (2 * mu * kG)).epsilon(0.03));
    }
}

TEST_CASE("friction stops a stack from drifting (1 substep, 10 iterations)") {
    // Without friction this stack's top box drifted 12 mm in 5 s. Static
    // friction holds it, up to what 10 Gauss-Seidel iterations leave unconverged.
    World w;
    w.addBody(Shape::plane(), 0);
    RigidBody* top = nullptr;
    for (int i = 0; i < 5; ++i) {
        RigidBody& b = w.addBody(Shape::box({0.5, 0.5, 0.5}), 1);
        b.restitution = 0;
        b.x = {0, 0.5 + 1.0 * i, 0};
        b.updateDerived();
        top = &b;
    }
    for (int i = 0; i < 240 * 5; ++i)
        w.step(kH);
    CHECK(std::hypot(top->x.x, top->x.z) < 1e-3);
    CHECK(length(top->v()) < 1e-3);
}
