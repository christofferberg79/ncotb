// ncotb — rigid body sandbox (raylib + Dear ImGui front-end for the physics library).

#include "app.h"

#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>

#include <algorithm>

using namespace ncotb::app;

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

    while (!WindowShouldClose()) {
        bool mouseFree = !ImGui::GetIO().WantCaptureMouse;
        bool keysFree = !ImGui::GetIO().WantCaptureKeyboard;
        app.handleInput(mouseFree, keysFree, GetFrameTime());

        app.advance(GetFrameTime());

        BeginDrawing();
        ClearBackground({235, 238, 242, 255});
        BeginMode3D(app.orbit.camera());
        app.drawScene();
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
