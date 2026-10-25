#include "track.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace ncotb::app {

using namespace track;

namespace {

constexpr Real kPi = std::numbers::pi_v<Real>;
constexpr Real kCell = 0.2;              // fine enough for the jumps' faces and lips
constexpr Real kHalfX = 35, kHalfZ = 15; // the arena, inside its wall

Real smoothstep(Real t) {
    t = std::clamp(t, Real(0), Real(1));
    return t * t * (3 - 2 * t);
}
/// 0 before a, 1 after b, a half cosine between.
Real rise(Real u, Real a, Real b) {
    return (1 - std::cos(kPi * std::clamp((u - a) / (b - a), Real(0), Real(1)))) / 2;
}
/// A kicker: from the ground at a, curving up ever steeper (a parabola) to the
/// lip at b, height h, with the slope 2 h / (b - a) there; then a steep back
/// down to the ground at c.
Real kicker(Real u, Real a, Real b, Real c, Real h) {
    if (u <= a || u >= c) return 0;
    if (u <= b) return h * (u - a) * (u - a) / ((b - a) * (b - a));
    return h * (1 - rise(u, b, c));
}
/// A landing: a steep face from a to b up to height h, flat to c, then a long
/// smooth slope down to the ground at d.
Real landing(Real u, Real a, Real b, Real c, Real d, Real h) {
    return h * std::min(rise(u, a, b), 1 - rise(u, c, d));
}

/// Straight A, driven towards +x (u = x): features to take at normal speed.
Real profileA(Real u) {
    Real h = 0;
    if (u > -14 && u < -9) h = 0.12 * (1 - std::cos(2 * kPi * (u + 14) / 1.25)) / 2; // four whoops
    h = std::max(h, 0.3 * std::min(rise(u, -5, -3.6), 1 - rise(u, -1.4, 0)));        // a tabletop
    h = std::max(h, kicker(u, 4, 5.4, 5.8, 0.35));                                   // a small double,
    h = std::max(h, landing(u, 6.6, 7.2, 7.4, 9.8, 0.35));                           // 1.2 m gap
    return h;
}
/// Straight B, driven towards -x (u = -x): the big double after the first boost
/// pad. The loop (boxes, not terrain) is at u = -loopX, after the second.
Real profileB(Real u) {
    Real h = kicker(u, -13, -10.6, -10.0, 0.7); // 30 degrees at the lip
    return std::max(h, landing(u, -7.4, -6.8, -6.4, -2.4, 0.7));
}

/// 1 across the middle of a straight, falling to 0 at the edges of its dirt:
/// the mounds have sloping sides.
Real across(Real dz) { return 1 - smoothstep((std::abs(dz) - (halfWidth - 0.4)) / 0.8); }

/// A banked turn round (cx, 0), on the side of it away from the middle: a bowl
/// rising to 0.9 m at the outer edge, faded in over the first metres of the
/// turn, and falling back to the ground outside.
Real berm(Real x, Real z, Real cx) {
    const Real beyond = (x - cx) * (cx > 0 ? 1 : -1);
    if (beyond <= 0) return 0;
    const Real r = std::hypot(x - cx, z), top = 0.9, edge = turnOuter + 0.5;
    Real h = r < edge ? top * std::pow(std::max(r - turnInner - 0.5, Real(0)) / (edge - turnInner - 0.5), 2)
                      : top * (1 - smoothstep((r - edge) / 2));
    return h * smoothstep(beyond / 4);
}

Real height(Real x, Real z) {
    Real h = berm(x, z, straightHalf) + berm(x, z, -straightHalf);
    if (std::abs(x) <= straightHalf) {
        h += across(z + laneZ) * profileA(x);
        h += across(z - laneZ) * profileB(-x);
    }
    return h;
}

/// A static box turned by q, with its +y face centred on top.
RigidBody& slab(World& w, const Vec3& half, const Vec3& top, const Quat& q) {
    return w.addBody(Shape::box(half), 0, top - rotate(q, {0, half.y, 0}), q);
}

/// The loop's curve, seen from the side: a teardrop. A circle would press the
/// truck into the planks at about 7 g at the speed it needs for the top, and
/// the suspension bottoms out and throws away half the energy. So the radius
/// of curvature eases from loopEntryRadius at the bottom down to
/// loopTopRadius at the top, rho = top + (entry - top) c^3 with
/// c = (1 + cos phi) / 2 for the direction phi of travel (0 at the entry,
/// going -x): about 3 g at most. The exit comes out a few metres further on.
struct LoopPoint {
    Real s, phi, rho, x, y; ///< arc length, direction, radius of curvature, position
};
const std::vector<LoopPoint>& loopCurve() {
    static const std::vector<LoopPoint> curve = [] {
        std::vector<LoopPoint> c;
        const int steps = 4800; // small steps of phi
        LoopPoint p{0, 0, 0, loopX, 0};
        for (int i = 0; i <= steps; ++i) {
            const Real k = (1 + std::cos(p.phi)) / 2, dphi = 2 * kPi / steps;
            p.rho = loopTopRadius + (loopEntryRadius - loopTopRadius) * k * k * k;
            c.push_back(p);
            p.s += p.rho * dphi;
            p.x -= p.rho * std::cos(p.phi) * dphi;
            p.y += p.rho * std::sin(p.phi) * dphi;
            p.phi += dphi;
        }
        return c;
    }();
    return curve;
}

/// The loop's lane: flat, with a guide rib down the middle, like a slot car's
/// slot: the truck straddles it (it's lower than the chassis and arms), and a
/// wheel that drifts sideways meets its side and is pushed back. With both
/// axles held, it keeps the truck's heading too. Low kerbs at the edges.
constexpr Real kLaneHalf = 0.38;                    ///< half width of the flat lane
constexpr Real kRibHeight = 0.07, kRibHalf = 0.025; ///< the guide rib
/// The kerbs: strips this wide at these slopes (radians), outwards.
constexpr std::pair<Real, Real> kKerbs[] = {{0.12, 0.45}, {0.1, 1.05}};

using ColorFn = std::function<std::uintptr_t(int, int, int)>;

/// A kerb along one side (k = +1: +z of the frame r, -1: -z) of a lane whose
/// top is centred on top, halfLength long along the frame's x.
void kerb(World& w, const ColorFn& color, const Vec3& top, const Quat& r, Real halfLength, Real k, bool red) {
    Vec3 edge{0, 0, k * kLaneHalf}; // in the frame: where the next strip starts
    for (auto [width, slope] : kKerbs) {
        const Vec3 e{0, std::sin(slope), k * std::cos(slope)};
        RigidBody& b = slab(w, {halfLength, 0.05, width / 2 + 0.005}, top + rotate(r, edge + e * (width / 2)),
                            r * Quat::fromAxisAngle({1, 0, 0}, -k * slope));
        b.userData = red ? color(220, 50, 40) : color(235, 235, 235);
        b.friction = 0.9;
        edge += e * width;
    }
}

/// The guide rib on a lane whose top is centred on top.
RigidBody& rib(World& w, const ColorFn& color, const Vec3& top, const Quat& r, Real halfLength) {
    RigidBody& b = w.addBody(Shape::box({halfLength, kRibHeight / 2, kRibHalf}), 0,
                             top + rotate(r, {0, kRibHeight / 2, 0}), r);
    b.userData = color(250, 200, 30);
    b.friction = 0.05; // a wheel rubbing along it isn't braked
    return b;
}

/// A stretch of lane: flat, the rib and the kerbs.
void lane(World& w, const ColorFn& color, const Vec3& top, const Quat& r, Real halfLength, bool red) {
    RigidBody& flat = slab(w, {halfLength, 0.05, kLaneHalf}, top, r);
    flat.userData = color(200, 205, 215);
    flat.friction = 0.9;
    rib(w, color, top, r, halfLength);
    for (Real k : {-1.0, 1.0})
        kerb(w, color, top, r, halfLength, k, red);
}

/// The loop: the teardrop, swept sideways at a constant rate (loopShift over
/// its length), so the exit is beside the entry. It's a cylinder (not a round
/// one), and a truck rolling straight on a cylinder follows a geodesic, which
/// unrolls to a straight line: driven in at loopHeading(), it goes round with
/// the sideways drift by itself, and the rib keeps it there. Planks of equal
/// angle, short where the curve is tight: at each seam the wheels' path turns
/// a corner, which costs a fraction of about angle^2 of the speed^2
/// (4 pi^2 / n round the loop). The inside is concave, so the wheels
/// (spheres) don't catch on the planks' edges.
void buildLoop(World& w, const ColorFn& color) {
    const auto& curve = loopCurve();
    const Real length = curve.back().s, beta = std::atan(loopShift / length);
    const int n = 240, stripe = 6;
    for (int i = 0; i <= n; ++i) {
        const LoopPoint& q = curve[size_t(i) * (curve.size() - 1) / n];
        const Vec3 top{q.x, q.y + 0.002, loopEntryZ + loopShift * q.s / length};
        // Plank x along the curve backwards (-direction), y towards the inside,
        // turned by beta about y to line up with the drift. A little longer
        // than the angle needs, so they overlap.
        const Quat r = Quat::fromAxisAngle({0, 0, 1}, -q.phi) * Quat::fromAxisAngle({0, 1, 0}, beta);
        const Real halfLength = q.rho * std::tan(kPi / n) / std::cos(beta) + 0.01;
        lane(w, color, top, r, halfLength, (i / stripe) % 2 == 0);
    }
    // The way in, on the ground: kerbs, flared at the mouth, line the truck up
    // roughly, then the rib takes over (its start ramped, so a wheel that
    // meets it end on rolls over).
    const Real h = loopHeading(), channel = 7, flare = 5, flareAngle = 0.08, ribLength = 4.5, ramp = 0.5;
    const Quat along = Quat::fromAxisAngle({0, 1, 0}, h);
    for (Real k : {-1.0, 1.0}) {
        kerb(w, color, loopEntry(-channel / 2), along, channel / 2, k, true);
        // Turned out about the inner edge of the kerb at the mouth.
        const Quat q =
            along * Quat::fromAxisAngle({0, 1, 0}, k * flareAngle); // heading is positive to the left
        const Vec3 mouth = loopEntry(-channel) + rotate(along, {0, 0, k * kLaneHalf});
        kerb(w, color, mouth - rotate(q, {flare / 2, 0, k * kLaneHalf}), q, flare / 2, k, false);
    }
    rib(w, color, loopEntry(-ribLength / 2), along, ribLength / 2);
    const Real slope = std::atan(kRibHeight / ramp);
    rib(w, color, loopEntry(-ribLength - ramp / 2) - Vec3{0, kRibHeight / 2, 0},
        along * Quat::fromAxisAngle({0, 0, 1}, slope), std::hypot(ramp, kRibHeight) / 2);
}

} // namespace

bool BoostPad::boosts(const Vec3& p, const Vec3& forward) const {
    const Vec3 d = direction(), side{-d.z, 0, d.x}, r = p - centre;
    return std::abs(dot(r, d)) <= halfLength && std::abs(dot(r, side)) <= halfWidth && r.y < 0.4 &&
           dot(forward, d) > 0.5;
}

Real Booster::update(const std::vector<BoostPad>& pads, const Vec3& chassis, const Vec3& forward, Real h) {
    onPad_ =
        std::any_of(pads.begin(), pads.end(), [&](const BoostPad& p) { return p.boosts(chassis, forward); });
    timer_ = onPad_ ? boostTime : std::max(timer_ - h, Real(0));
    return boost();
}

std::shared_ptr<const Heightfield> truckTrackTerrain() {
    static const auto hf = std::make_shared<const Heightfield>(
        Heightfield::fromFunction(int(2 * kHalfX / kCell), int(2 * kHalfZ / kCell), kCell, height));
    return hf;
}

Real truckTrackDirt(Real x, Real z) {
    auto band = [](Real d, Real half) { return 1 - smoothstep((std::abs(d) - half) / 0.4); };
    // The straights, and the way into and out of the loop.
    Real dirt = 0;
    if (std::abs(x) <= straightHalf) dirt = std::max(band(z + laneZ, halfWidth), band(z - laneZ, halfWidth));
    const Vec3 p{x, 0, z}, a = loopEntry(-13), b = loopExit() + (loopExit() - loopEntry(0)) * 0.5;
    const Real t = std::clamp(dot(p - a, b - a) / dot(b - a, b - a), Real(0), Real(1));
    dirt = std::max(dirt, band(length(p - (a + (b - a) * t)), 1.2));
    if (std::abs(x) <= straightHalf) return dirt;
    // The banked turns: dirt up to the top of the berm.
    const Real cx = x > 0 ? straightHalf : -straightHalf, mid = (turnInner + turnOuter) / 2;
    return std::max(dirt, band(std::hypot(x - cx, z) - mid - 0.5, (turnOuter - turnInner) / 2 + 0.5));
}

void buildTruckTrack(World& w, const ColorFn& color) {
    RigidBody& ground = w.addBody(Shape::heightfield(truckTrackTerrain()), 0);
    ground.friction = 0.8;
    ground.restitution = 0.1;
    buildLoop(w, color);
    // The arena wall.
    const Real wh = 0.2, wt = 0.15;
    for (Real s : {-1.0, 1.0}) {
        w.addBody(Shape::box({kHalfX, wh, wt}), 0, {0, wh, s * (kHalfZ - wt)}).userData = color(40, 45, 55);
        w.addBody(Shape::box({wt, wh, kHalfZ - 2 * wt}), 0, {s * (kHalfX - wt), wh, 0}).userData =
            color(40, 45, 55);
    }
}

Vec3 loopEntry(Real along) {
    const Real h = loopHeading();
    return Vec3{loopX, 0, loopEntryZ} + Vec3{std::cos(h), 0, -std::sin(h)} * along;
}

Real loopHeading() { return kPi + std::atan(loopShift / loopCurve().back().s); }

Real loopOffset(const Vec3& p) {
    const auto& curve = loopCurve();
    const Real length = curve.back().s;
    // The nearest point of the curve, seen from the side, of those with p
    // beside their lane (seen from the side, the entry and exit cross).
    Real best = INFINITY, offset = 0;
    for (const LoopPoint& q : curve) {
        const Real lane = loopEntryZ + loopShift * q.s / length;
        if (std::abs(p.z - lane) > 0.8) continue;
        if (const Real d = std::hypot(p.x - q.x, p.y - q.y); d < best) {
            best = d;
            offset = p.z - lane;
        }
    }
    return offset;
}

Vec3 loopExit() {
    const LoopPoint& q = loopCurve().back();
    return {q.x, 0, loopEntryZ + loopShift};
}

const std::vector<BoostPad>& truckTrackPads() {
    static const std::vector<BoostPad> pads = {
        {.centre = {16.5, 0, laneZ}, .heading = kPi, .halfLength = 1.5, .halfWidth = 0.8},
        {.centre = loopEntry(-5.5), .heading = loopHeading(), .halfLength = 2.5, .halfWidth = 0.3},
    };
    return pads;
}

} // namespace ncotb::app
