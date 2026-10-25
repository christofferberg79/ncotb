#pragma once

// The RC truck track: an oval of dirt with jumps, banked turns, boost pads and
// a loop, in an arena with a low wall. Physics only (no raylib), so the tests
// can drive the truck round it.

#include "world.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace ncotb::app {

/// A patch of ground that boosts the truck (see CarSpec::boostSpeed): drawn as
/// arrows, flat on the ground.
struct BoostPad {
    Vec3 centre;     ///< on the ground
    Real heading;    ///< the way it boosts, as Car::heading() (0 along +x, positive to the left)
    Real halfLength; ///< along heading
    Real halfWidth;

    Vec3 direction() const { return {std::cos(heading), 0, -std::sin(heading)}; }
    /// True if a car at p (chassis centre) facing forward is on the pad: over
    /// it, close to the ground and going roughly its way.
    bool boosts(const Vec3& p, const Vec3& forward) const;
};

/// Boost from the pads: full on a pad and for boostTime after leaving it.
class Booster {
public:
    static constexpr Real boostTime = 1.2; ///< s

    /// Call once per step of length h, before Car::drive; returns the boost.
    Real update(const std::vector<BoostPad>& pads, const Vec3& chassis, const Vec3& forward, Real h);
    bool onPad() const { return onPad_; } ///< on a pad: the truck goes at full throttle
    Real boost() const { return timer_ > 0 ? 1 : 0; }
    void reset() { *this = {}; }

private:
    Real timer_ = 0;
    bool onPad_ = false;
};

/// The layout. The straights run along x, straight A at z = -laneZ (driven
/// towards +x) and B at z = +laneZ (towards -x); the turns are half circles
/// round (+-straightHalf, 0).
namespace track {
constexpr Real straightHalf = 20;
constexpr Real laneZ = 10;
constexpr Real halfWidth = 2;                 ///< of the dirt on the straights
constexpr Real turnInner = 8, turnOuter = 12; ///< radii of the turns' edges
constexpr Real loopX = -12.5;                 ///< bottom of the loop's entry, on straight B
constexpr Real loopEntryRadius = 2.5;         ///< of curvature of its riding surface, at the bottom
constexpr Real loopTopRadius = 0.8;           ///< and at the top (see buildLoop)
constexpr Real loopShift = 2.0;               ///< sideways from entry to exit
constexpr Real loopEntryZ = laneZ + 0.8;      ///< the middle of the entry
} // namespace track

/// The track's terrain (built once, shared).
std::shared_ptr<const Heightfield> truckTrackTerrain();
/// How much (x, z) is dirt track rather than grass, 0 to 1 (for drawing).
Real truckTrackDirt(Real x, Real z);
/// Adds the terrain, the loop and the arena wall. color(r, g, b) gives a
/// body's userData for a colour.
void buildTruckTrack(World& w, const std::function<std::uintptr_t(int, int, int)>& color);
/// The heading to drive into the loop at (towards -x and a little +z, with
/// the loop's sideways drift).
Real loopHeading();
/// The point `along` metres along that heading from the bottom of the entry
/// (negative: before it), on the ground.
Vec3 loopEntry(Real along);
/// How far p is off the middle of the loop's lane, along z (for tests).
Real loopOffset(const Vec3& p);
/// The bottom of the exit, on the ground.
Vec3 loopExit();
/// The boost pads: one before the big double jump and one before the loop.
const std::vector<BoostPad>& truckTrackPads();

} // namespace ncotb::app
