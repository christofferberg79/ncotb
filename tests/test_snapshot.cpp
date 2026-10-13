#include <doctest/doctest.h>

#include "io/snapshot.h"
#include "vehicle/car.h"

#include <cmath>
#include <memory>
#include <sstream>

using namespace ncotb;

namespace {

constexpr Real kH = 1.0 / 240;

/// A bit of everything: terrain, a box stack, a pendulum on a ball joint, a
/// door on a hinge with a motor, a slider with limits, a rod, a spring, and the truck.
struct Everything {
    World w;
    std::unique_ptr<Car> car;

    Everything() {
        w.substeps = 4;
        w.solver.settings.iterations = 60;
        auto hf = std::make_shared<const Heightfield>(Heightfield::hills(40, 0.5, 0.8, 6, 5));
        w.addBody(Shape::heightfield(hf), 0).friction = 0.8;
        for (int i = 0; i < 3; ++i)
            w.addBody(Shape::box({0.2, 0.2, 0.2}), 1, {4, hf->height(4, 3) + 0.25 + 0.41 * i, 3});
        RigidBody& bob = w.addBody(Shape::sphere(0.1), 1, {-3, 3, 0});
        w.addBallJoint(w.ground(), bob, {-3, 4, 0.5});
        RigidBody& door = w.addBody(Shape::box({0.4, 0.5, 0.03}), 2, {-3, 3, 3});
        w.addHinge(w.ground(), door, {-3.4, 3, 3}, {0, 1, 0}).setMotor(1.5, 5);
        RigidBody& carriage = w.addBody(Shape::box({0.1, 0.1, 0.1}), 1, {3, 3, -3});
        w.addSlider(w.ground(), carriage, carriage.x, {1, 0.5, 0}).setLimits(-0.5, 0.5);
        RigidBody& link = w.addBody(Shape::box({0.05, 0.3, 0.05}), 0.5, {3, 2.4, -3});
        w.addDistanceJoint(carriage, link, carriage.x, link.x + Vec3{0, 0.3, 0});
        w.addSpring(w.ground(), {0, 4, -4}, link, link.x, 30, 0.5);
        car = std::make_unique<Car>(w, CarSpec{}, Vec3{0, hf->height(0, 0) + 0.02, 0}, 0.4);
    }
    void run(int steps, int offset = 0) {
        for (int i = 0; i < steps; ++i) {
            car->drive(0.8, std::sin((i + offset) * 0.01), 0);
            w.step(kH);
        }
    }
};

std::string save(const World& w, const Car& car) {
    std::ostringstream out;
    SnapshotWriter writer(out);
    saveWorld(w, writer);
    saveCar(car, w, writer);
    return out.str();
}

bool sameState(const World& a, const World& b) {
    if (a.bodies().size() != b.bodies().size() || a.time() != b.time()) return false;
    for (size_t i = 0; i < a.bodies().size(); ++i) {
        const RigidBody &p = *a.bodies()[i], &q = *b.bodies()[i];
        auto same = [](const Vec3& u, const Vec3& v) { return u.x == v.x && u.y == v.y && u.z == v.z; };
        if (!same(p.x, q.x) || !same(p.P, q.P) || !same(p.L, q.L) || p.q.s != q.q.s || !same(p.q.v, q.q.v))
            return false;
    }
    return true;
}

} // namespace

TEST_CASE("a loaded snapshot steps on exactly like the original") {
    Everything original;
    original.run(240); // in the middle of things: driving, swinging, sliding
    const std::string text = save(original.w, *original.car);

    World copy;
    std::istringstream in(text);
    SnapshotReader reader(in);
    REQUIRE(loadWorld(copy, reader));
    std::unique_ptr<Car> car = loadCar(copy, reader);
    REQUIRE(car);
    CHECK(reader.error().empty());
    CHECK(sameState(original.w, copy));
    CHECK(copy.joints().size() == original.w.joints().size());
    CHECK(copy.springs().size() == original.w.springs().size());

    // Both step on with the same driving: bit for bit the same.
    for (int i = 0; i < 480; ++i) {
        const Real steer = std::sin((240 + i) * 0.01);
        original.car->drive(0.8, steer, 0);
        car->drive(0.8, steer, 0);
        original.w.step(kH);
        copy.step(kH);
    }
    CHECK(sameState(original.w, copy));
    CHECK(car->speed() == original.car->speed());

    // And saving the copy gives the same text again.
    CHECK(save(copy, *car) == save(original.w, *original.car));
}

TEST_CASE("snapshots skip mouse joints but keep the joint indices of the car right") {
    Everything e;
    e.run(10);
    e.w.addMouseJoint(e.car->chassis(), e.car->chassis().x, 5, 0.7, 100); // a grab
    std::ostringstream out;
    SnapshotWriter writer(out);
    saveWorld(e.w, writer);
    saveCar(*e.car, e.w, writer);
    World copy;
    std::istringstream in(out.str());
    SnapshotReader reader(in);
    REQUIRE(loadWorld(copy, reader));
    auto car = loadCar(copy, reader);
    REQUIRE(car);
    CHECK(copy.joints().size() == e.w.joints().size() - 1);
    CHECK(&car->steering(0).bodyB() == car->knuckles()[0]);
}

TEST_CASE("broken snapshots are refused with the line of the problem") {
    Everything e;
    const std::string text = save(e.w, *e.car);
    // Cut it off half way, and garble a number.
    for (const std::string& bad : {text.substr(0, text.size() / 2), [&] {
                                       std::string t = text;
                                       t.replace(t.find("mass") + 5, 1, "x");
                                       return t;
                                   }()}) {
        World w;
        std::istringstream in(bad);
        SnapshotReader reader(in);
        bool ok = loadWorld(w, reader) && loadCar(w, reader) != nullptr;
        CHECK_FALSE(ok);
        CHECK(reader.error().rfind("line ", 0) == 0);
    }
}
