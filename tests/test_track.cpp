#include <doctest/doctest.h>

#include "track.h"
#include "vehicle/car.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>

using namespace ncotb;
using namespace ncotb::app;

namespace {

constexpr Real kH = 1.0 / 240;

/// The truck on the track, with the app's solver settings and boost pads.
struct TrackWorld {
    World w;
    std::unique_ptr<Car> car;
    Booster booster;
    Real worstJointError = 0;
    Real lowestUp = 1; ///< smallest chassis up.y seen (-1: upside down)

    TrackWorld(Vec3 start, Real heading) {
        w.gravity = {0, -9.81, 0};
        w.substeps = 4;
        w.solver.settings.iterations = 60;
        buildTruckTrack(w, [](int, int, int) { return std::uintptr_t(0); });
        start.y = truckTrackTerrain()->height(start.x, start.z) + 0.02;
        car = std::make_unique<Car>(w, CarSpec{}, start, heading);
        w.detectCollisions();
        for (int i = 0; i < 120; ++i) { // settle on the suspension
            car->drive(0, 0, 0);
            w.step(kH);
        }
    }
    Real up() const { return car->chassis().R().column(1).y; }
    /// Full throttle, straight ahead; calls watch() after every step.
    template <class F> void drive(Real seconds, bool boostPads, F watch) {
        for (int i = 0; i < int(std::lround(seconds / kH)); ++i) {
            Real boost =
                boostPads ? booster.update(truckTrackPads(), car->chassis().x, car->forward(), kH) : 0;
            car->drive(1, 0, 0, boost);
            w.step(kH);
            worstJointError = std::max(worstJointError, car->jointError());
            lowestUp = std::min(lowestUp, up());
            watch();
        }
    }
};

} // namespace

TEST_CASE("track: straight A's whoops, tabletop and small double at normal speed") {
    TrackWorld t({-18, 0, -track::laneZ}, 0);
    t.drive(7.5, false, [] {});
    const Vec3 p = t.car->chassis().x;
    CHECK(p.x > 11);                           // through it all
    CHECK(std::abs(p.z + track::laneZ) < 1.0); // and still on the straight
    CHECK(t.up() > 0.9);                       // on its wheels
    CHECK(t.lowestUp > 0.3);                   // never on its side or roof
    CHECK(t.worstJointError < 0.02);
}

TEST_CASE("track: the boost pad launches the truck over the big double's gap") {
    TrackWorld t({19.5, 0, track::laneZ}, std::numbers::pi);
    Real lowestOverGap = INFINITY;
    t.drive(4, true, [&] {
        const Vec3 p = t.car->chassis().x;
        if (p.x < 9.8 && p.x > 7.6) lowestOverGap = std::min(lowestOverGap, p.y); // gap floor: x 10.0 to 7.4
    });
    INFO("lowest over the gap ", lowestOverGap);
    CHECK(lowestOverGap > 0.5); // flew over, not down into it
    CHECK(t.car->chassis().x.x < 2);
    CHECK(t.up() > 0.9);
    CHECK(t.lowestUp > 0.3);
    CHECK(t.worstJointError < 0.02);
}

TEST_CASE("track: the boost pad takes the truck round the loop") {
    // Lined up 13 m before the loop, or a little off; the funnel and the rib
    // do the rest. (The truck doesn't steer here; a driver can.)
    struct Start {
        Real heading, side;
    };
    for (const Start& s : {Start{0, 0}, Start{0.025, 0}, Start{0, -0.25}}) {
        CAPTURE(s.heading);
        CAPTURE(s.side);
        TrackWorld t(loopEntry(-13) + Vec3{0, 0, s.side}, loopHeading() + s.heading);
        Real highest = 0, worstOffset = 0;
        bool out = false; // past the exit
        Vec3 p{};
        Real up = 0;
        t.drive(5, true, [&] {
            const Vec3 x = t.car->chassis().x;
            highest = std::max(highest, x.y);
            if (x.y > 0.3) worstOffset = std::max(worstOffset, std::abs(loopOffset(x)));
            if (!out && highest > 1.5 && x.x < loopExit().x - 1) {
                out = true;
                p = x;
                up = t.up();
            }
        });
        INFO("out at ", p.x, ", ", p.y, ", ", p.z, "; highest ", highest);
        CHECK(highest > 2.0);                      // up to the top
        CHECK(t.lowestUp < -0.9);                  // upside down there
        CHECK(worstOffset < 0.15);                 // on the rib all the way
        REQUIRE(out);                              // and out of it
        CHECK(std::abs(p.z - loopExit().z) < 0.3); // beside the entry
        CHECK(p.y < 0.3);                          // on the ground
        CHECK(up > 0.9);                           // on its wheels
        CHECK(t.worstJointError < 0.02);
    }
}
