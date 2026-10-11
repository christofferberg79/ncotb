// ncotb — rigid body sandbox (raylib + Dear ImGui front-end for the physics library).

#include "world.h"

#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>
#include <rlgl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

using namespace ncotb;

namespace {

::Vector3 toRl(const Vec3& v) { return {float(v.x), float(v.y), float(v.z)}; }

// ---------------------------------------------------------------- camera

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

// ---------------------------------------------------------------- drawing

void drawBody(const RigidBody& b, Color color) {
    // Column-major 4x4 transform from R and x.
    const Mat3& R = b.R();
    float m[16] = {float(R.m[0][0]), float(R.m[1][0]), float(R.m[2][0]), 0,
                   float(R.m[0][1]), float(R.m[1][1]), float(R.m[2][1]), 0,
                   float(R.m[0][2]), float(R.m[1][2]), float(R.m[2][2]), 0,
                   float(b.x.x),     float(b.x.y),     float(b.x.z),     1};
    rlPushMatrix();
    rlMultMatrixf(m);
    const Shape& s = b.shape();
    if (s.type == Shape::Type::Box) {
        ::Vector3 size = toRl(2 * s.halfExtents);
        DrawCubeV({0, 0, 0}, size, color);
        DrawCubeWiresV({0, 0, 0}, size, BLACK);
    } else {
        DrawSphereEx({0, 0, 0}, float(s.radius), 16, 16, color);
        DrawSphereWires({0, 0, 0}, float(s.radius) * 1.002f, 8, 8, BLACK);
    }
    rlPopMatrix();
}

void drawArrow(const Vec3& from, const Vec3& dir, Color color) {
    Vec3 to = from + dir;
    DrawLine3D(toRl(from), toRl(to), color);
    DrawSphere(toRl(to), 0.04f, color);
}

// ---------------------------------------------------------------- scenes

struct Scene {
    const char* name;
    std::function<void(World&)> build;
};

const std::vector<Scene>& scenes() {
    static const std::vector<Scene> list = {
        {"Intermediate axis (Dzhanibekov)",
         [](World& w) {
             w.gravity = {};
             RigidBody& b = w.addBody(Shape::box({1.2, 0.6, 0.2}), 1.0);
             b.x = {0, 2, 0};
             b.setAngularVelocity({0.01, 5.0, 0.01});
         }},
        {"Three principal axes",
         [](World& w) {
             w.gravity = {};
             const Vec3 spins[3] = {{6, 0.05, 0.05}, {0.05, 6, 0.05}, {0.05, 0.05, 6}};
             for (int i = 0; i < 3; ++i) {
                 RigidBody& b = w.addBody(Shape::box({1.0, 0.5, 0.15}), 1.0);
                 b.x = {(i - 1) * 3.5, 2, 0};
                 b.setAngularVelocity(spins[i]);
             }
         }},
        {"Thrown boxes and balls",
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             for (int i = 0; i < 6; ++i) {
                 bool box = i % 2 == 0;
                 RigidBody& b = w.addBody(box ? Shape::box({0.4, 0.25, 0.15}) : Shape::sphere(0.3), 1.0);
                 b.x = {-6, 1 + 0.3 * i, (i - 2.5) * 1.2};
                 b.setLinearVelocity({5 + 0.4 * i, 9, 0});
                 b.setAngularVelocity({1.0 * i, 3, 2});
             }
         }},
    };
    return list;
}

// ---------------------------------------------------------------- app state

struct App {
    World world;
    int sceneIndex = 0;
    bool paused = false;
    bool stepOnce = false;
    float timeScale = 1.0f;
    int stepsPerSecond = 240;
    bool showVectors = true;
    double accumulator = 0;

    static constexpr int kHistory = 600;
    std::array<float, kHistory> omegaHistory[3]{};
    int historyPos = 0;

    void loadScene(int i) {
        sceneIndex = i;
        world.clear();
        scenes()[i].build(world);
        accumulator = 0;
        for (auto& h : omegaHistory) h.fill(0);
        historyPos = 0;
    }

    void recordHistory() {
        if (world.bodies().empty()) return;
        const RigidBody& b = *world.bodies().front();
        // Angular velocity in body coordinates shows the axis flip clearly.
        Vec3 wb = transpose(b.R()) * b.omega();
        for (int k = 0; k < 3; ++k) omegaHistory[k][historyPos] = float(wb[k]);
        historyPos = (historyPos + 1) % kHistory;
    }

    void advance(double frameTime) {
        const double h = 1.0 / stepsPerSecond;
        if (paused) {
            if (stepOnce) {
                world.step(h);
                recordHistory();
            }
            stepOnce = false;
            return;
        }
        accumulator += std::min(frameTime, 0.1) * timeScale;
        while (accumulator >= h) {
            world.step(h);
            accumulator -= h;
            recordHistory();
        }
    }

    void gui() {
        ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({360, 0}, ImGuiCond_FirstUseEver);
        ImGui::Begin("ncotb");

        if (ImGui::BeginCombo("Scene", scenes()[sceneIndex].name)) {
            for (int i = 0; i < int(scenes().size()); ++i)
                if (ImGui::Selectable(scenes()[i].name, i == sceneIndex)) loadScene(i);
            ImGui::EndCombo();
        }
        if (ImGui::Button("Reset (R)")) loadScene(sceneIndex);
        ImGui::SameLine();
        ImGui::Checkbox("Paused (Space)", &paused);
        ImGui::SameLine();
        if (ImGui::Button("Step (S)")) stepOnce = true;

        ImGui::SliderFloat("Time scale", &timeScale, 0.05f, 2.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderInt("Steps / s", &stepsPerSecond, 30, 1000);
        ImGui::Checkbox("Show omega (yellow) and L (magenta)", &showVectors);

        ImGui::Separator();
        Real energy = 0;
        Vec3 L{}, P{};
        for (auto& b : world.bodies()) {
            energy += b->kineticEnergy() - b->mass() * dot(world.gravity, b->x);
            L += b->L + cross(b->x, b->P);
            P += b->P;
        }
        ImGui::Text("t = %.2f s   bodies = %zu", world.time(), world.bodies().size());
        ImGui::Text("Total energy  E   = %.5f J", energy);
        ImGui::Text("Ang. momentum |L| = %.5f", length(L));
        ImGui::Text("Lin. momentum |P| = %.5f", length(P));

        if (!world.bodies().empty()) {
            ImGui::Separator();
            ImGui::TextUnformatted("Body 0 omega in body frame (x, y, z):");
            const char* labels[3] = {"wx", "wy", "wz"};
            for (int k = 0; k < 3; ++k)
                ImGui::PlotLines(labels[k], omegaHistory[k].data(), kHistory, historyPos, nullptr,
                                 -8.0f, 8.0f, {0, 50});
        }
        ImGui::Separator();
        ImGui::TextDisabled("Drag: orbit   Wheel: zoom   %d FPS", GetFPS());
        ImGui::End();
    }
};

} // namespace

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI | FLAG_VSYNC_HINT);
    InitWindow(1400, 900, "ncotb — rigid body simulation");
    SetTargetFPS(120);
    {
        // If the window doesn't fit the screen, macOS shrinks it without raylib
        // noticing; ImGui then clips with the stale size (empty combo popups).
        int monitor = GetCurrentMonitor();
        int mw = GetMonitorWidth(monitor), mh = GetMonitorHeight(monitor);
        int w = std::min(1400, mw * 9 / 10), h = std::min(900, mh * 8 / 10);
        SetWindowSize(w, h);
        SetWindowPosition((mw - w) / 2, (mh - h) / 2);
    }
    rlImGuiSetup(true);

    App app;
    app.loadScene(0);
    OrbitCamera orbit;

    const Color palette[] = {{230, 120, 70, 255}, {80, 150, 220, 255}, {120, 190, 100, 255},
                             {220, 190, 70, 255}, {170, 110, 200, 255}, {90, 200, 190, 255}};

    while (!WindowShouldClose()) {
        bool mouseFree = !ImGui::GetIO().WantCaptureMouse;
        bool keysFree = !ImGui::GetIO().WantCaptureKeyboard;
        orbit.update(mouseFree);
        if (keysFree) {
            if (IsKeyPressed(KEY_SPACE)) app.paused = !app.paused;
            if (IsKeyPressed(KEY_S)) app.stepOnce = true;
            if (IsKeyPressed(KEY_R)) app.loadScene(app.sceneIndex);
        }

        app.advance(GetFrameTime());

        BeginDrawing();
        ClearBackground({235, 238, 242, 255});
        BeginMode3D(orbit.camera());
        DrawGrid(40, 1.0f);
        int i = 0;
        for (auto& b : app.world.bodies()) {
            drawBody(*b, palette[i++ % std::size(palette)]);
            if (app.showVectors) {
                drawArrow(b->x, b->omega() * 0.3, {200, 160, 0, 255});
                drawArrow(b->x, b->L * 0.3, MAGENTA);
            }
        }
        EndMode3D();

        rlImGuiBegin();
        app.gui();
        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
