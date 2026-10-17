#include <doctest/doctest.h>

#include "world.h"

#include <cmath>

using namespace ncotb;

namespace {

constexpr Real kH = 1.0 / 240;

void run(World& w, Real seconds) {
    for (int i = 0; i < int(std::lround(seconds / kH)); ++i)
        w.step(kH);
}

/// A 3-box stack on the ground, 4 substeps.
struct Stack {
    World w;
    RigidBody* box[3];
    Stack() {
        w.substeps = 4;
        w.addBody(Shape::plane(), 0);
        for (int i = 0; i < 3; ++i)
            box[i] = &w.addBody(Shape::box({0.3, 0.3, 0.3}), 1, {0, 0.3 + 0.6 * i, 0});
    }
};

Real support(const World& w, const RigidBody& body) {
    Real f = 0;
    for (const ContactManifold& m : w.contacts())
        if (m.a == &body && m.b->isStatic())
            for (int i = 0; i < m.count; ++i)
                f += m.points[i].normalImpulse * m.normal.y;
    return f / w.lastSubstep();
}

} // namespace

TEST_CASE("a resting stack falls asleep, stays exactly still, and keeps its contact forces") {
    Stack s;
    run(s.w, 1.5);
    CHECK(s.w.sleepingBodies() == 3);
    const Vec3 x = s.box[2]->x;
    run(s.w, 2);
    CHECK(s.box[2]->x.x == x.x); // not a bit of drift while asleep
    CHECK(s.box[2]->x.y == x.y);
    CHECK(length(s.box[2]->v()) == 0);
    // The frozen contacts still show the ground carrying the whole stack.
    CHECK(support(s.w, *s.box[0]) == doctest::Approx(3 * 9.81).epsilon(0.01));
}

TEST_CASE("an awake body hitting a sleeping stack wakes it") {
    Stack s;
    run(s.w, 1.5);
    REQUIRE(s.w.sleepingBodies() == 3);
    // Thrown sideways at the top box.
    RigidBody& ball = s.w.addBody(Shape::sphere(0.2), 5, {-3, 1.5, 0});
    ball.setLinearVelocity({10, 0, 0});
    run(s.w, 0.4);
    CHECK(s.w.sleepingBodies() == 0);
    run(s.w, 3);
    CHECK(s.box[2]->x.x > 0.6); // knocked off the stack
}

TEST_CASE("removing the support of a sleeping body wakes it and it falls") {
    Stack s;
    run(s.w, 1.5);
    REQUIRE(s.box[2]->asleep);
    const Real y = s.box[2]->x.y;
    s.w.removeBody(*s.box[1]);
    run(s.w, 1);
    CHECK(s.box[2]->x.y < y - 0.5);
}

TEST_CASE("a force or a new velocity wakes a sleeping body") {
    Stack s;
    run(s.w, 1.5);
    REQUIRE(s.box[2]->asleep);
    s.box[2]->addForce({50, 0, 0});
    s.w.step(kH);
    CHECK_FALSE(s.box[2]->asleep);
    CHECK(s.box[2]->v().x > 0);

    run(s.w, 3);
    REQUIRE(s.box[0]->asleep);
    s.box[0]->setLinearVelocity({0, 0, 1});
    CHECK_FALSE(s.box[0]->asleep);
}

TEST_CASE("moving bodies don't fall asleep, and sleeping can be turned off") {
    World w;
    w.substeps = 4;
    RigidBody& bob = w.addBody(Shape::sphere(0.1), 1, {1, 2, 0});
    w.addBallJoint(w.ground(), bob, {0, 2, 0}); // a pendulum, briefly still at each end of its swing
    run(w, 5);
    CHECK_FALSE(bob.asleep);

    Stack s;
    s.w.sleeping = false;
    run(s.w, 3);
    CHECK(s.w.sleepingBodies() == 0);
}
