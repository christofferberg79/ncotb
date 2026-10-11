#pragma once

#include "camera.h"
#include "world.h"

#include <functional>
#include <optional>
#include <vector>

namespace ncotb::app {

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
    Real autoActionAt = -1;               ///< run the action once at this time (s)
    std::function<void()> decorate{};     ///< extra drawing (in 3D mode), not physics
    std::function<void(World&)> update{}; ///< called after every step (scripted motors etc.)
};

/// All scenes, in the order of the scene menu.
const std::vector<Scene>& scenes();

} // namespace ncotb::app
