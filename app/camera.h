#pragma once

#include <raylib.h>

#include <algorithm>
#include <cmath>

namespace ncotb::app {

/// Orbits around a target point, which can itself be moved (trackpad friendly):
///
///   drag                         orbit
///   Shift + drag, right-drag     pan: the point under the cursor follows it
///     (two-finger click + drag on a trackpad)
///   scroll (two fingers)         zoom
///   arrow keys                   move the target over the ground (Shift: faster)
///   Q / E                        move the target down / up
struct OrbitCamera {
    float yaw = 0.6f, pitch = 0.35f, distance = 12.0f;
    ::Vector3 target{0, 1, 0};

    static constexpr float kFovY = 45; ///< vertical field of view (degrees)

    void update(bool mouseFree, bool keysFree, float dt) {
        if (mouseFree) {
            ::Vector2 d = GetMouseDelta();
            bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            bool left = IsMouseButtonDown(MOUSE_BUTTON_LEFT), right = IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
            if (right || (left && shift)) {
                panPixels(d.x, d.y, float(GetScreenHeight()));
            } else if (left) {
                yaw -= d.x * 0.005f;
                pitch = std::clamp(pitch + d.y * 0.005f, -1.5f, 1.5f);
            }
            distance = std::clamp(distance * (1.0f - GetMouseWheelMove() * 0.1f), 1.0f, 200.0f);
        }
        if (keysFree) {
            float speed = distance * 0.6f * dt; // a fraction of the view per second, at any zoom
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) speed *= 3;
            ::Vector3 f = forward(), r = right();
            auto move = [&](::Vector3 dir, float amount) {
                target.x += dir.x * amount;
                target.y += dir.y * amount;
                target.z += dir.z * amount;
            };
            if (IsKeyDown(KEY_UP)) move(f, speed);
            if (IsKeyDown(KEY_DOWN)) move(f, -speed);
            if (IsKeyDown(KEY_RIGHT)) move(r, speed);
            if (IsKeyDown(KEY_LEFT)) move(r, -speed);
            if (IsKeyDown(KEY_E)) move({0, 1, 0}, speed);
            if (IsKeyDown(KEY_Q)) move({0, -1, 0}, speed);
        }
    }

    /// Moves the view as if the scene were dragged by (dx, dy) pixels: the point
    /// at the target's depth stays under the cursor.
    void panPixels(float dx, float dy, float screenHeight) {
        float metresPerPixel = 2 * distance * std::tan(kFovY * DEG2RAD / 2) / screenHeight;
        ::Vector3 r = right(), u = up();
        target.x += (-dx * r.x + dy * u.x) * metresPerPixel;
        target.y += (-dx * r.y + dy * u.y) * metresPerPixel;
        target.z += (-dx * r.z + dy * u.z) * metresPerPixel;
    }

    Camera3D camera() const {
        Camera3D c{};
        c.target = target;
        c.position = {target.x + distance * std::cos(pitch) * std::sin(yaw),
                      target.y + distance * std::sin(pitch),
                      target.z + distance * std::cos(pitch) * std::cos(yaw)};
        c.up = {0, 1, 0};
        c.fovy = kFovY;
        c.projection = CAMERA_PERSPECTIVE;
        return c;
    }

private:
    /// Horizontal direction the camera looks in.
    ::Vector3 forward() const { return {-std::sin(yaw), 0, -std::cos(yaw)}; }
    /// Screen right, horizontal.
    ::Vector3 right() const { return {std::cos(yaw), 0, -std::sin(yaw)}; }
    /// Screen up (perpendicular to the view direction and right).
    ::Vector3 up() const {
        return {-std::sin(pitch) * std::sin(yaw), std::cos(pitch), -std::sin(pitch) * std::cos(yaw)};
    }
};

} // namespace ncotb::app
