#pragma once

#include "world.h"

namespace ncotb::app {

/// The cannon in the brick wall scene: a barrel on a wheeled carriage that the
/// player aims and fires. Drawn only; the balls it fires are bodies.
///
///   W / S        barrel up / down          A / D   turn left / right
///   Z / X        less / more powder        F, Space   fire
///   Shift        fine steps
///
/// A dashed arc shows where a ball would fly, with the current gravity, up to
/// the first thing it would hit.
class Cannon {
public:
    /// Placed in front of the wall and aimed at its middle for the world's gravity.
    explicit Cannon(const World& world);

    Real elevation = 0; ///< radians above horizontal
    Real azimuth = 0;   ///< radians, 0 towards the wall (-z), positive to the left
    Real speed = 25;    ///< muzzle speed (m/s)
    bool showTrajectory = true;

    /// Keyboard aiming and firing (call once per frame with the keys free).
    void keys(World& world, float dt);
    void fire(World& world);
    void draw(const World& world) const;
    void gui(World& world);

    Vec3 direction() const; ///< unit vector along the barrel
    Vec3 muzzle() const;
    int fired() const { return fired_; }

private:
    int fired_ = 0;
};

} // namespace ncotb::app
