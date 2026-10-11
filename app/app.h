#pragma once

#include "camera.h"
#include "world.h"

#include <array>

namespace ncotb::app {

/// The sandbox: the world, the current scene, the settings shown in the panel
/// and the fixed-step clock.
struct App {
    World world;
    int sceneIndex = 0;
    bool paused = false;
    bool stepOnce = false;
    float timeScale = 1.0f;
    int stepsPerSecond = 240;
    bool showVectors = true;
    bool showContacts = true;
    bool showNormals = false;
    float forceScale = 0.05f; ///< metres per newton for the force arrows (0: off)
    int contactSubsteps = 4;  ///< substeps in contact scenes
    OrbitCamera orbit;
    OrbitCamera startView;                    ///< the scene's starting view, for "reset view" (C)
    static constexpr float kPanelWidth = 360; ///< control panel width (pixels)
    bool autoActionDone = false;
    Vec3 sceneGravity{}; ///< the scene's own gravity, for the on/off switch
    double accumulator = 0;

    // "Move a body" editor: rotation is applied on top of the orientation the
    // body had when it was selected.
    IntegratorType freeIntegrator = IntegratorType::SemiImplicitEuler; ///< choice for free-motion scenes
    int editIndex = -1;
    Quat editBaseQ{};
    float editEulerDeg[3]{};

    static constexpr int kHistory = 600;
    std::array<float, kHistory> omegaHistory[3]{};
    std::array<float, kHistory> energyErrorHistory{};
    int historyPos = 0;
    Real energy0 = 0; ///< total energy when the scene was loaded

    void loadScene(int i);
    void resetView() { orbit = startView; }
    /// The scene's action (e.g. "Fire"), if it has one.
    void runAction();
    /// Runs as many fixed steps as fit in frameTime (scaled by timeScale).
    void advance(double frameTime);
    /// The control panel.
    void gui();
    /// Draws the world (call between BeginMode3D and EndMode3D).
    void drawScene() const;

private:
    void stepOnceNow(double h);
    void recordHistory();
    void footer();
    void solverGui();
    void contactsGui();
    void editGui();
    int bodyIndex(const RigidBody* b) const;
    static const char* shapeName(const RigidBody& b);
};

} // namespace ncotb::app
