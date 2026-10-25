// ncotb — rigid body sandbox (raylib + Dear ImGui front-end for the physics library).

#include "app.h"

#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>

using namespace ncotb::app;

int main() {
#ifdef __APPLE__
    // Started from Finder, an app bundle's working directory is / (read-only), so put
    // snapshots/ and imgui.ini in Application Support instead. Builds run from a
    // terminal (not a bundle) keep the working directory.
    if (std::strstr(GetApplicationDirectory(), ".app/Contents/MacOS")) {
        if (const char* home = std::getenv("HOME")) {
            std::error_code ec;
            const auto dir = std::filesystem::path(home) / "Library/Application Support/ncotb";
            std::filesystem::create_directories(dir, ec);
            std::filesystem::current_path(dir, ec);
        }
    }
#endif
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI | FLAG_VSYNC_HINT);
    InitWindow(1400, 900, "ncotb — rigid body simulation");
    SetTargetFPS(120);
    {
        // If the window doesn't fit the screen, macOS shrinks it without raylib
        // noticing; ImGui then clips with the stale size (empty combo popups).
        // Monitor and window sizes are in points on macOS, but in pixels on Windows,
        // where the 1400 x 900 (logical) window is scaled by the display's DPI scale.
#ifdef __APPLE__
        const Vector2 scale{1, 1};
#else
        const Vector2 scale = GetWindowScaleDPI();
#endif
        int monitor = GetCurrentMonitor();
        int mw = GetMonitorWidth(monitor), mh = GetMonitorHeight(monitor);
        int w = std::min(int(1400 * scale.x), mw * 9 / 10), h = std::min(int(900 * scale.y), mh * 8 / 10);
        SetWindowSize(w, h);
        SetWindowPosition((mw - w) / 2, (mh - h) / 2);
    }
    // rlImGui's own default font is scaled by the display's DPI scale except on macOS,
    // on top of ImGui scaling everything by the framebuffer scale: on a 200% Windows
    // display the text came out twice too large. ImGui's scaling alone is right.
    rlImGuiSetLoadFontsCallback([] {
        ImFontConfig config;
        config.SizePixels = 13;
        config.PixelSnapH = true;
        ImGui::GetIO().Fonts->AddFontDefault(&config);
    });
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
