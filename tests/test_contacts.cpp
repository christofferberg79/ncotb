#include <doctest/doctest.h>

#include "solver/lcp.h"
#include "world.h"

#include <cmath>
#include <random>

using namespace ncotb;

namespace {

constexpr Real kH = 1.0 / 240;
constexpr SolverType kSolvers[] = {SolverType::SequentialImpulses, SolverType::BaraffLcp};

World groundWorld(SolverType solver) {
    World w;
    w.solver.settings.type = solver;
    w.addBody(Shape::plane(), 0);
    return w;
}

void run(World& w, Real seconds) {
    for (int i = 0; i < int(std::lround(seconds / kH)); ++i)
        w.step(kH);
}

/// Sum of normal contact forces acting on body (from all its contacts), along +y.
Real supportForce(const World& w, const RigidBody& body) {
    Real f = 0;
    for (const ContactManifold& m : w.contacts()) {
        Real sign = m.a == &body ? 1 : (m.b == &body ? -1 : 0);
        for (int i = 0; i < m.count; ++i)
            f += sign * m.points[i].normalImpulse * m.normal.y;
    }
    return f / w.lastSubstep();
}

/// Height reached after the first bounce of a sphere dropped with its bottom at h0.
Real reboundHeight(SolverType solver, Real e, Real h0) {
    World w = groundWorld(solver);
    RigidBody& ball = w.addBody(Shape::sphere(0.25), 1);
    ball.restitution = e;
    w.bodies()[0]->restitution = 0; // the larger value is used
    ball.x = {0, 0.25 + h0, 0};
    ball.updateDerived();
    bool bounced = false;
    Real top = 0;
    for (int i = 0; i < 240 * 3; ++i) {
        Real vBefore = ball.v().y;
        w.step(kH);
        if (!bounced && vBefore < 0 && ball.v().y > 0) bounced = true;
        if (bounced) {
            top = std::max(top, ball.x.y - 0.25);
            if (ball.v().y < 0) break; // past the top of the first rebound
        }
    }
    return top;
}

} // namespace

TEST_CASE("a sphere dropped on the ground comes to rest on it") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        World w = groundWorld(solver);
        RigidBody& ball = w.addBody(Shape::sphere(0.5), 2);
        ball.restitution = 0;
        ball.x = {0, 1.5, 0};
        ball.updateDerived();
        run(w, 3);
        CHECK(length(ball.v()) < 1e-3);
        CHECK(ball.x.y == doctest::Approx(0.5).epsilon(0.02)); // within the 5 mm slop
        CHECK(ball.x.y <= 0.5 + 1e-6);
        CHECK(supportForce(w, ball) == doctest::Approx(2 * 9.81).epsilon(0.01));
    }
}

TEST_CASE("a box resting on its face is held up by m g, shared over 4 points") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        World w = groundWorld(solver);
        RigidBody& box = w.addBody(Shape::box({0.5, 0.25, 0.4}), 3);
        box.restitution = 0;
        box.x = {0, 0.3, 0};
        box.updateDerived();
        run(w, 2);
        CHECK(length(box.v()) < 1e-3);
        CHECK(length(box.omega()) < 1e-3);
        REQUIRE(w.contacts().size() == 1);
        CHECK(w.contacts()[0].count == 4);
        CHECK(supportForce(w, box) == doctest::Approx(3 * 9.81).epsilon(0.01));
    }
}

TEST_CASE("bounce height is about e^2 times the drop height") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        CHECK(reboundHeight(solver, 0.5, 1.0) == doctest::Approx(0.25).epsilon(0.08));
        CHECK(reboundHeight(solver, 0.8, 1.0) == doctest::Approx(0.64).epsilon(0.05));
        CHECK(reboundHeight(solver, 0.0, 1.0) < 0.01);
        Real elastic = reboundHeight(solver, 1.0, 1.0);
        CHECK(elastic > 0.95);
        CHECK(elastic < 1.001); // no energy gained
    }
}

TEST_CASE("a stack of five boxes stands still (4 substeps)") {
    // Without friction (milestone 4) nothing resists sideways sliding, so the
    // tiny tilt left by an unconverged Gauss-Seidel solve makes the stack drift.
    // Substeps shrink that 16x (1.6 cm -> 1 mm in 5 s at 10 iterations).
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        World w = groundWorld(solver);
        w.substeps = 4;
        std::vector<RigidBody*> boxes;
        for (int i = 0; i < 5; ++i) {
            RigidBody& b = w.addBody(Shape::box({0.5, 0.5, 0.5}), 1);
            b.restitution = 0;
            b.x = {0, 0.5 + 1.0 * i, 0};
            b.updateDerived();
            boxes.push_back(&b);
        }
        run(w, 5);
        for (RigidBody* b : boxes)
            CHECK(length(b->v()) < 1e-3);
        const RigidBody& top = *boxes.back();
        CHECK(std::hypot(top.x.x, top.x.z) < 2e-3);
        CHECK(top.x.y == doctest::Approx(4.5).epsilon(0.01));
        CHECK(supportForce(w, *boxes.front()) == doctest::Approx(9.81).epsilon(0.02)); // net: own weight
    }
}

TEST_CASE("an elastic head-on collision of equal spheres swaps their velocities") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        World w;
        w.gravity = {};
        w.solver.settings.type = solver;
        RigidBody& a = w.addBody(Shape::sphere(0.5), 1);
        RigidBody& b = w.addBody(Shape::sphere(0.5), 1);
        a.restitution = b.restitution = 1;
        a.x = {-2, 0, 0};
        b.x = {2, 0, 0};
        a.setLinearVelocity({3, 0, 0});
        b.setLinearVelocity({-1, 0, 0});
        run(w, 2);
        CHECK(a.v().x == doctest::Approx(-1).epsilon(0.01));
        CHECK(b.v().x == doctest::Approx(3).epsilon(0.01));
        CHECK(a.P.x + b.P.x == doctest::Approx(2));
    }
}

TEST_CASE("Baraff LCP: random positive definite problems satisfy complementarity") {
    std::mt19937 rng(7);
    std::normal_distribution<Real> g;
    for (int trial = 0; trial < 300; ++trial) {
        const int n = 1 + trial % 12;
        const int rank = trial % 3 == 0 ? std::max(1, n / 2) : n + 2; // every third is singular
        std::vector<std::vector<Real>> G(static_cast<size_t>(n),
                                         std::vector<Real>(static_cast<size_t>(rank)));
        for (auto& row : G)
            for (Real& v : row)
                v = g(rng);
        DenseMatrix A(n);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                Real s = 0;
                for (int k = 0; k < rank; ++k)
                    s += G[size_t(i)][size_t(k)] * G[size_t(j)][size_t(k)];
                A(i, j) = s;
            }
        std::vector<Real> b(static_cast<size_t>(n)), x;
        for (Real& v : b)
            v = g(rng);
        // Singular A only has a solution when b is in the right range; make one
        // by construction: b = -A x* + w* with x*, w* complementary.
        if (rank < n) {
            std::vector<Real> xs(static_cast<size_t>(n)), ws(static_cast<size_t>(n));
            for (int i = 0; i < n; ++i)
                (i % 2 ? xs : ws)[size_t(i)] = std::abs(g(rng));
            for (int i = 0; i < n; ++i) {
                Real s = 0;
                for (int j = 0; j < n; ++j)
                    s += A(i, j) * xs[size_t(j)];
                b[size_t(i)] = ws[size_t(i)] - s;
            }
        }
        CAPTURE(trial);
        LcpResult res = solveLcpBaraff(A, b, x);
        REQUIRE(res.solved);
        for (int i = 0; i < n; ++i) {
            Real w = b[size_t(i)];
            for (int j = 0; j < n; ++j)
                w += A(i, j) * x[size_t(j)];
            CHECK(x[size_t(i)] >= 0);
            CHECK(w >= -1e-7);
            CHECK(std::abs(x[size_t(i)] * w) < 1e-7);
        }
    }
}

TEST_CASE("Baraff LCP: the old freeze case (redundant contacts) terminates") {
    // Four coplanar contacts of a box on the ground: A has rank 3.
    World w = groundWorld(SolverType::BaraffLcp);
    RigidBody& box = w.addBody(Shape::box({0.5, 0.5, 0.5}), 1);
    box.x = {0, 0.49, 0};
    box.updateDerived();
    for (int i = 0; i < 240; ++i) {
        w.step(kH);
        REQUIRE_FALSE(w.solver.stats().lcpFailed);
    }
    CHECK(supportForce(w, box) == doctest::Approx(9.81).epsilon(0.01));
}

TEST_CASE("an overlap is pushed out slowly and does not launch the body") {
    // Sphere starting 5 cm into the floor, no gravity: it must end up resting on
    // the surface, not fly away with the push-out velocity.
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        World w = groundWorld(solver);
        w.gravity = {};
        w.substeps = 4;
        RigidBody& ball = w.addBody(Shape::sphere(0.5), 1);
        ball.x = {0, 0.45, 0};
        ball.updateDerived();
        Real maxSpeed = 0;
        for (int i = 0; i < 240; ++i) {
            w.step(kH);
            maxSpeed = std::max(maxSpeed, length(ball.v()));
        }
        CHECK(maxSpeed < 1e-6); // the push-out moves the ball but leaves no velocity
        CHECK(ball.x.y == doctest::Approx(0.5).epsilon(0.011)); // out to within the 5 mm slop
    }
}

TEST_CASE("a stack started 3 cm into itself settles without exploding") {
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        World w = groundWorld(solver);
        w.substeps = 4;
        std::vector<RigidBody*> boxes;
        for (int i = 0; i < 3; ++i) {
            RigidBody& b = w.addBody(Shape::box({0.5, 0.5, 0.5}), 1);
            b.restitution = 0;
            b.x = {0, 0.47 + 0.97 * i, 0};
            b.updateDerived();
            boxes.push_back(&b);
        }
        Real maxSpeed = 0;
        for (int i = 0; i < 240 * 2; ++i) {
            w.step(kH);
            for (RigidBody* b : boxes)
                maxSpeed = std::max(maxSpeed, length(b->v()));
        }
        // Sequential impulses' relax pass doesn't fully converge in a coupled
        // stack, so a little push-out speed remains for a few steps (0.27 m/s
        // here; it was 8.6 m/s for a single overlapping sphere before the relax
        // pass). The exact LCP leaves none.
        CHECK(maxSpeed < 0.5);
        for (RigidBody* b : boxes)
            CHECK(length(b->v()) < 1e-3);
        CHECK(boxes.back()->x.y == doctest::Approx(2.5).epsilon(0.01));
    }
}

TEST_CASE("a ball wider than the gap between two walls stays jammed and calm") {
    // No valid position exists (1 m ball, walls 95 cm apart). The solver can't
    // fix that, but it must not jitter, launch the ball or hang.
    for (SolverType solver : kSolvers) {
        INFO(name(solver));
        World w;
        w.gravity = {};
        w.substeps = 4;
        w.solver.settings.type = solver;
        for (int side : {-1, 1}) {
            RigidBody& wall = w.addBody(Shape::box({0.5, 1, 1}), 0);
            wall.x = {side * (0.475 + 0.5), 0, 0};
            wall.updateDerived();
        }
        RigidBody& ball = w.addBody(Shape::sphere(0.5), 1);
        Real maxSpeed = 0;
        for (int i = 0; i < 240 * 2; ++i) {
            w.step(kH);
            maxSpeed = std::max(maxSpeed, length(ball.v()));
        }
        CHECK(maxSpeed < 1e-6);
        // The LCP keeps it centred. Gauss-Seidel solves one wall after the other,
        // so the last one wins and the ball shifts 2 cm (until that side's overlap
        // is down to the slop), then stays put.
        CHECK(length(ball.x) < 0.025);
    }
}

TEST_CASE("Baraff LCP falls back to sequential impulses for very large contact sets") {
    World w = groundWorld(SolverType::BaraffLcp);
    w.sleeping = false;               // keep solving the resting stack
    w.solver.settings.lcpMaxRows = 8; // a 3-box stack has 12 contact points
    std::vector<RigidBody*> boxes;
    for (int i = 0; i < 3; ++i) {
        RigidBody& b = w.addBody(Shape::box({0.5, 0.5, 0.5}), 1);
        b.x = {0, 0.5 + 1.0 * i, 0};
        b.updateDerived();
        boxes.push_back(&b);
    }
    run(w, 1);
    CHECK(w.solver.stats().lcpTooLarge);
    CHECK_FALSE(w.solver.stats().lcpFailed);
    CHECK(w.solver.stats().pivots == 0);
    CHECK(length(boxes.back()->v()) < 1e-3);
}

TEST_CASE("fast bodies don't tunnel through thin walls at a coarse step") {
    // 40 m/s at 30 steps/s moves 1.33 m per step: far more than the 10 cm wall
    // and the bodies themselves are thick.
    constexpr Real h = 1.0 / 30, speed = 40;
    for (SolverType solver : kSolvers) {
        for (bool sphere : {true, false}) {
            CAPTURE(solver);
            CAPTURE(sphere);
            World w;
            w.gravity = {};
            w.solver.settings.type = solver;
            w.addBody(Shape::box({2, 2, 0.05}), 0, {0, 0, 0}).restitution = 0; // wall in the xy-plane
            Shape shape = sphere ? Shape::sphere(0.1) : Shape::box({0.1, 0.1, 0.1});
            RigidBody& body = w.addBody(shape, 1, {0.3, -0.2, 3}); // off centre: not a lucky hit
            body.restitution = 0;
            body.setLinearVelocity({0, 0, -speed});
            for (int i = 0; i < 30; ++i)
                w.step(h);
            CHECK(body.x.z > 0.05); // still in front of the wall
            CHECK(body.x.z < 0.2);  // and resting on it, not bounced away
            CHECK(std::abs(body.v().z) < 0.1);
        }
    }
}

TEST_CASE("a fast ball bounces off a thin wall with its restitution") {
    constexpr Real h = 1.0 / 30, speed = 40;
    World w;
    w.gravity = {};
    w.addBody(Shape::box({2, 2, 0.05}), 0, {0, 0, 0}).restitution = 0;
    RigidBody& ball = w.addBody(Shape::sphere(0.1), 1, {0, 0, 3});
    ball.restitution = 0.5;
    ball.setLinearVelocity({0, 0, -speed});
    for (int i = 0; i < 30; ++i)
        w.step(h);
    CHECK(ball.v().z == doctest::Approx(0.5 * speed).epsilon(0.01));
}

TEST_CASE("a fast body passing beside a box is not slowed by speculative contacts") {
    // Moving parallel to the box's face, 5 cm from it: it never closes the gap.
    constexpr Real h = 1.0 / 30, speed = 40;
    for (bool sphere : {true, false}) {
        CAPTURE(sphere);
        World w;
        w.gravity = {};
        w.addBody(Shape::box({0.5, 0.5, 0.5}), 0, {0, 0, 0});
        Shape shape = sphere ? Shape::sphere(0.1) : Shape::box({0.1, 0.1, 0.1});
        RigidBody& body = w.addBody(shape, 1, {0.65, 0, 3});
        body.setLinearVelocity({0, 0, -speed});
        for (int i = 0; i < 6; ++i)
            w.step(h);
        CHECK(body.x.z < -3);
        CHECK(body.v().z == -speed);
        CHECK(body.x.x == 0.65);
    }
}

TEST_CASE("a fast ball knocks a free thin plate along instead of passing through") {
    constexpr Real h = 1.0 / 30;
    World w;
    w.gravity = {};
    RigidBody& plate = w.addBody(Shape::box({1, 1, 0.02}), 1, {0, 0, 0});
    RigidBody& ball = w.addBody(Shape::sphere(0.1), 1, {0, 0, 3});
    ball.restitution = plate.restitution = 0;
    ball.setLinearVelocity({0, 0, -30});
    for (int i = 0; i < 15; ++i)
        w.step(h);
    // Perfectly inelastic: they move on together at half the speed.
    CHECK(plate.v().z == doctest::Approx(-15).epsilon(0.02));
    CHECK(ball.v().z == doctest::Approx(-15).epsilon(0.02));
    CHECK(ball.x.z > plate.x.z);
}

TEST_CASE("a fast rolling ball stays on the ground (no phantom gap from its rotation)") {
    // Rolling at 20 m/s with r = 0.4 turns the ball 12 degrees per step. If the
    // contact point turned with it, it would seem to lift off by r (1 - cos),
    // and the ball would sink into that gap at ~2 m/s and then bounce.
    for (SolverType solver : kSolvers) {
        CAPTURE(solver);
        World w = groundWorld(solver);
        const Real r = 0.4, speed = 20;
        RigidBody& ball = w.addBody(Shape::sphere(r), 1, {0, r, 0});
        ball.restitution = 0.8;
        ball.setLinearVelocity({speed, 0, 0});
        ball.setAngularVelocity({0, 0, -speed / r});
        Real highest = 0, fastestY = 0;
        for (int i = 0; i < 240; ++i) {
            w.step(kH);
            highest = std::max(highest, ball.x.y - r);
            fastestY = std::max(fastestY, std::abs(ball.v().y));
        }
        CHECK(highest < 1e-6);
        CHECK(fastestY < 1e-6);
        CHECK(ball.v().x == doctest::Approx(speed));
    }
}
