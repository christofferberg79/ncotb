#pragma once

#include <raylib.h>

#include <algorithm>
#include <cmath>

namespace ncotb::app {

/// Orbits around a target: drag to turn, wheel to zoom.
struct OrbitCamera {
    float yaw = 0.6f, pitch = 0.35f, distance = 12.0f;
    ::Vector3 target{0, 1, 0};

    void update(bool mouseFree) {
        if (mouseFree) {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                ::Vector2 d = GetMouseDelta();
                yaw -= d.x * 0.005f;
                pitch = std::clamp(pitch + d.y * 0.005f, -1.5f, 1.5f);
            }
            distance = std::clamp(distance * (1.0f - GetMouseWheelMove() * 0.1f), 1.0f, 200.0f);
        }
    }

    Camera3D camera() const {
        Camera3D c{};
        c.target = target;
        c.position = {target.x + distance * std::cos(pitch) * std::sin(yaw),
                      target.y + distance * std::sin(pitch),
                      target.z + distance * std::cos(pitch) * std::cos(yaw)};
        c.up = {0, 1, 0};
        c.fovy = 45;
        c.projection = CAMERA_PERSPECTIVE;
        return c;
    }
};

} // namespace ncotb::app
