#pragma once

#include "world.h"

#include <raylib.h>

#include <utility>
#include <vector>

namespace ncotb::app {

/// Acting on the simulation with the mouse (trackpad) and keyboard:
///
///   click-drag on a dynamic body   grab it (a soft spring to the cursor)
///   scroll while grabbing          move the grabbed point nearer / farther
///   Option + click on a body       push it along the view ray
///   1 / 2                          drop a box / ball above the point under the cursor
///   3                              throw a ball from the camera at the cursor
///   Backspace                      delete the body under the cursor
/// This frame's mouse and keyboard state (filled from raylib by Input::read(), or
/// by a script for testing).
struct Input {
    ::Vector2 mouse{};
    bool leftDown = false, leftPressed = false;
    float wheel = 0;
    bool shift = false, option = false;
    bool key1 = false, key2 = false, key3 = false, backspace = false; ///< pressed this frame
    float dt = 0;

    static Input read();
};

class Interaction {
public:
    // Settings (shown in the panel).
    float grabFrequency = 5; ///< Hz: how stiffly a grabbed body follows the cursor
    float grabDampingRatio = 0.7f;
    float grabMaxForceG = 30;      ///< maximum grab force, in multiples of the body's weight
    float grabAngularDamping = 10; ///< 1/s: calms a held body's swinging
    float pushSpeed = 3;           ///< velocity change of a pushed body (m/s), so J = m * pushSpeed
    float throwSpeed = 15;         ///< m/s
    float spawnSize = 0.3f;        ///< half extent / radius (m)
    float spawnMass = 2;           ///< kg

    /// Handles this frame's input. Returns true if it took the mouse (a grab or
    /// push started on a body), so the camera mustn't use it.
    bool update(World& world, const Camera3D& camera, const Input& in, bool mouseFree, bool keysFree);
    /// Forget everything (the world was rebuilt).
    void reset();
    /// Bodies added with 1/2/3 (for "Remove spawned"; saved in snapshots).
    const std::vector<RigidBody*>& spawned() const { return spawned_; }
    void setSpawned(std::vector<RigidBody*> bodies) { spawned_ = std::move(bodies); }
    /// True if bodies were removed this frame (indices into world.bodies() changed).
    bool removedBodies() const { return removedBodies_; }

    /// Highlights, the grab spring and push arrows (between BeginMode3D and EndMode3D).
    void draw(const World& world) const;
    void gui(World& world);

private:
    void startGrab(World& world, const RayHit& hit, const Camera3D& camera);
    void releaseGrab(World& world);
    void spawn(World& world, const Vec3& origin, const Vec3& dir, int key);
    void remove(World& world, RigidBody& body);

    RigidBody* hovered_ = nullptr;
    MouseJoint* grab_ = nullptr;
    float grabDepth_ = 0;     ///< distance of the grab plane from the camera, along the view
    bool mouseOwned_ = false; ///< from a press on a body until the button is released
    Vec3 pushPoint_{}, pushDir_{};
    float pushFlash_ = 0; ///< seconds left to show the last push
    std::vector<RigidBody*> spawned_;
    bool removedBodies_ = false;
};

} // namespace ncotb::app
