#pragma once

#include "camera.h"
#include "cannon.h"
#include "interaction.h"
#include "vehicle/car.h"
#include "world.h"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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
    Interaction interaction;
    OrbitCamera startView;                    ///< the scene's starting view, for "reset view" (C)
    static constexpr float kPanelWidth = 360; ///< control panel width (pixels)
    bool autoActionDone = false;
    float gravity = float(kEarthGravity); ///< strength (m/s^2), kept across scenes
    bool gravityOn = true;                ///< off in the zero-g scenes
    double accumulator = 0;
    double simulationRate = 1; ///< simulated seconds per real second, when the steps can't keep up
    bool showJoints = true;
    bool showSleeping = false; ///< draw sleeping bodies darker

    std::optional<Cannon> cannon; ///< the scene's cannon, if it has one
    /// The scene takes WASD and Space (a car or a cannon): P pauses instead.
    bool sceneKeys() const { return car || cannon; }

    // The scene's car, if it has one, and the driver's input.
    std::unique_ptr<Car> car;
    Real throttle = 0, steer = 0, brake = 0;
    bool chaseCamera = true;
    float manualOrbit = 0; ///< seconds left in which the chase camera leaves the yaw alone

    // Snapshots (docs/notes.md): files in snapshots/, newest first.
    std::string snapshotStatus;
    std::vector<std::string> snapshotFiles;
    int snapshotChoice = 0;
    /// Saves the complete state to snapshots/<time>-<scene>.snap; returns the path ("" on failure).
    std::string saveSnapshot();
    /// Loads a snapshot (paused); on failure keeps or restores a working scene.
    bool loadSnapshot(const std::string& path);

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

    /// Loads scene i with its starting view.
    void loadScene(int i);
    /// Restarts the current scene, keeping the camera where it is.
    void resetScene() {
        OrbitCamera view = orbit;
        loadScene(sceneIndex);
        orbit = view;
    }
    void resetView() { orbit = startView; }
    /// Keyboard and mouse for this frame: interaction first (it may take the
    /// mouse), then the camera, then the keys for pause, step, reset, ...
    void handleInput(bool mouseFree, bool keysFree, float dt);
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
    /// Driving keys (W/S, A/D, Space, T, V) and the chase camera.
    void driveInput(float dt);
    void chase(float dt);
    /// Drops the car if the user deleted one of its parts.
    void checkCar();
    void carGui();
    void snapshotGui();
    void listSnapshots();
    void recordHistory();
    void footer();
    void solverGui();
    void contactsGui();
    void editGui();
    int bodyIndex(const RigidBody* b) const;
    static const char* shapeName(const RigidBody& b);
};

} // namespace ncotb::app
