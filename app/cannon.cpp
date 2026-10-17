#include "cannon.h"

#include "draw.h"

#include <imgui.h>
#include <raylib.h>
#include <rlgl.h>

#include <algorithm>
#include <cmath>

namespace ncotb::app {

namespace {

/// Where the barrel turns (the trunnions), 13.5 m in front of the wall.
constexpr Vec3 kPivot{0.3, 0.85, 14.5};
constexpr Real kHalfBarrel = 1.5;
/// Aim for the middle of the wall by default.
constexpr Vec3 kWallTarget{0.3, 1.7, 0};

constexpr Real kMinElevation = -5 * M_PI / 180, kMaxElevation = 60 * M_PI / 180;
constexpr Real kMaxAzimuth = 60 * M_PI / 180;
constexpr Real kMinSpeed = 5, kMaxSpeed = 50;

/// Barrel direction for an elevation and azimuth.
Vec3 aim(Real elevation, Real azimuth) {
    return {-std::sin(azimuth) * std::cos(elevation), std::sin(elevation),
            -std::cos(azimuth) * std::cos(elevation)};
}

/// Height at which a ball fired with this elevation crosses the wall's plane (z = target.z).
Real heightAtWall(Real elevation, Real speed, Real g) {
    const Vec3 d = aim(elevation, 0);
    const Vec3 m = kPivot + d * kHalfBarrel;
    const Real t = (m.z - kWallTarget.z) / (speed * std::cos(elevation));
    return m.y + speed * std::sin(elevation) * t - 0.5 * g * t * t;
}

} // namespace

Cannon::Cannon(const World& world) {
    // The lower of the two elevations that reach the target (bisection; the
    // height at the wall rises with elevation up to well past 30 degrees).
    const Real g = length(world.gravity);
    Real lo = kMinElevation, hi = 30 * M_PI / 180;
    for (int i = 0; i < 60; ++i) {
        Real mid = (lo + hi) / 2;
        (heightAtWall(mid, speed, g) < kWallTarget.y ? lo : hi) = mid;
    }
    elevation = (lo + hi) / 2;
}

Vec3 Cannon::direction() const { return aim(elevation, azimuth); }

Vec3 Cannon::muzzle() const { return kPivot + direction() * kHalfBarrel; }

void Cannon::keys(World& world, float dt) {
    const Real fine = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) ? 0.2 : 1;
    const Real turn = 25 * M_PI / 180 * dt * fine; // degrees per second, as radians per frame
    if (IsKeyDown(KEY_W)) elevation += turn;
    if (IsKeyDown(KEY_S)) elevation -= turn;
    if (IsKeyDown(KEY_A)) azimuth += turn;
    if (IsKeyDown(KEY_D)) azimuth -= turn;
    if (IsKeyDown(KEY_X)) speed += 10 * dt * fine;
    if (IsKeyDown(KEY_Z)) speed -= 10 * dt * fine;
    elevation = std::clamp(elevation, kMinElevation, kMaxElevation);
    azimuth = std::clamp(azimuth, -kMaxAzimuth, kMaxAzimuth);
    speed = std::clamp(speed, kMinSpeed, kMaxSpeed);
    if (IsKeyPressed(KEY_F) || IsKeyPressed(KEY_SPACE)) fire(world);
}

void Cannon::fire(World& world) {
    RigidBody& ball = world.addBody(Shape::sphere(0.35), 30, muzzle()); // cast iron, about 30 kg
    ball.restitution = 0.2;
    ball.friction = 0.4;
    ball.userData = packColor(40, 42, 48);
    ball.setLinearVelocity(direction() * speed);
    world.detectCollisions();
    ++fired_;
}

void Cannon::draw(const World& world) const {
    const Color iron{45, 47, 52, 255}, wood{110, 70, 40, 255};
    // The carriage turns with the azimuth: draw it in its own frame (barrel along -z).
    rlPushMatrix();
    rlTranslatef(float(kPivot.x), 0, float(kPivot.z));
    rlRotatef(float(azimuth * 180 / M_PI), 0, 1, 0);
    const Vec3 d{0, std::sin(elevation), -std::cos(elevation)}, pivot{0, kPivot.y, 0};
    const Vec3 breech = pivot - d * kHalfBarrel, mouth = pivot + d * kHalfBarrel;
    DrawCylinderEx(toRl(breech), toRl(mouth), 0.5f, 0.4f, 24, iron);
    DrawCylinderWiresEx(toRl(mouth - d * 0.15), toRl(mouth), 0.44f, 0.44f, 24, {20, 20, 22, 255});
    DrawSphere(toRl(breech), 0.5f, iron);
    DrawCube({0, 0.4f, 0.7f}, 0.8f, 0.3f, 2.2f, wood); // the trail, behind the axle
    for (int side : {-1, 1}) {
        ::Vector3 c{side * 0.55f, 0.6f, 0}, c2{side * 0.69f, 0.6f, 0};
        DrawCylinderEx(c, c2, 0.6f, 0.6f, 24, wood);
        DrawCylinderWiresEx(c, c2, 0.6f, 0.6f, 12, {60, 35, 20, 255});
    }
    rlPopMatrix();

    if (!showTrajectory) return;
    // The ball's arc, dashed, until it would hit something (or after 6 s).
    const Vec3 g = world.gravity, v = direction() * speed;
    Vec3 p = muzzle();
    const Real dt = 0.02;
    const Color dash{230, 120, 40, 255};
    for (int i = 1; i <= 300; ++i) {
        const Real t = i * dt;
        const Vec3 q = muzzle() + v * t + g * (0.5 * t * t);
        const Vec3 step = q - p;
        if (auto hit = world.raycast(p, step, length(step), true)) {
            DrawSphere(toRl(hit->point), 0.12f, dash);
            break;
        }
        if (i % 2 == 0) DrawLine3D(toRl(p), toRl(q), dash);
        p = q;
    }
}

void Cannon::gui(World& world) {
    if (!ImGui::CollapsingHeader("Cannon", ImGuiTreeNodeFlags_DefaultOpen)) return;
    ImGui::TextDisabled("W/S: up/down  A/D: left/right  Z/X: powder");
    ImGui::TextDisabled("Shift: fine   F or Space: fire");
    float deg = float(elevation * 180 / M_PI);
    if (ImGui::SliderFloat("Elevation (deg)", &deg, float(kMinElevation * 180 / M_PI),
                           float(kMaxElevation * 180 / M_PI), "%.1f"))
        elevation = deg * M_PI / 180;
    deg = float(azimuth * 180 / M_PI);
    if (ImGui::SliderFloat("Direction (deg)", &deg, -60, 60, "%.1f")) azimuth = deg * M_PI / 180;
    float s = float(speed);
    if (ImGui::SliderFloat("Speed (m/s)", &s, float(kMinSpeed), float(kMaxSpeed), "%.1f")) speed = s;
    if (ImGui::Button("Fire (F)")) fire(world);
    ImGui::SameLine();
    ImGui::Checkbox("Trajectory", &showTrajectory);
    ImGui::SameLine();
    ImGui::Text("%d fired", fired_);
}

} // namespace ncotb::app
