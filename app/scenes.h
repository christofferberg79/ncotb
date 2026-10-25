#pragma once

#include "camera.h"
#include "track.h"
#include "world.h"

#include <functional>
#include <optional>
#include <vector>

namespace ncotb::app {

/// Scenes have gravity of the strength set in the panel (Earth by default);
/// a scene's build() turns it off with w.gravity = {}.
struct Scene {
    const char* name;
    /// Contact scenes use semi-implicit Euler (the only integrator that resolves
    /// contacts) and show the contact solver settings.
    bool contacts;
    std::function<void(World&)> build;
    // Optional:
    int substeps = 0;                    ///< substeps for this scene (0: the slider's value)
    std::optional<OrbitCamera> camera{}; ///< camera to start from
    bool showVectors = true;             ///< default for the omega/L arrows
    bool showContacts = true;            ///< default for the contact overlay
    const char* actionLabel = nullptr;   ///< e.g. "Fire": a button and the F key
    std::function<void(World&)> action{};
    Real autoActionAt = -1;                       ///< run the action once at this time (s)
    std::function<void(const World&)> decorate{}; ///< extra drawing (in 3D mode), not physics
    std::function<void(World&)> update{};         ///< called after every step (scripted motors etc.)
    int iterations = 0;                           ///< solver iterations for this scene (0: the default)
    bool showJoints = true;                       ///< draw joint anchors and axes
    // A drivable car (WASD), standing on the ground below carStart (x, z) and facing carHeading.
    bool car = false;
    Vec3 carStart{};
    Real carHeading = 0;
    bool cannon = false;               ///< an aimable cannon (WASD, F or Space to fire), see cannon.h
    std::vector<BoostPad> boostPads{}; ///< for the car (see track.h)
};

/// All scenes, in the order of the scene menu.
const std::vector<Scene>& scenes();

} // namespace ncotb::app
