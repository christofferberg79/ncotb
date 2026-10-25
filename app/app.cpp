#include "app.h"

#include "draw.h"
#include "keys.h"
#include "scenes.h"

#include <imgui.h>
#include <raylib.h>
#include <rlgl.h>

#include "io/snapshot.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>

namespace ncotb::app {

void App::loadScene(int i) {
    sceneIndex = i;
    world.clear();
    const Scene& scene = scenes()[i];
    if (scene.substeps > 0) contactSubsteps = scene.substeps;
    // Start from the scene's view, shifted right by half the panel's width so the
    // scene sits in the middle of the area next to the panel.
    startView = scene.camera.value_or(OrbitCamera{});
    startView.panPixels((kPanelWidth + 10) / 2, 0, float(GetScreenHeight()));
    resetView();
    showVectors = scene.showVectors;
    showContacts = scene.showContacts;
    autoActionDone = false;
    if (scene.contacts) {
        world.integrator = IntegratorType::SemiImplicitEuler;
        world.substeps = contactSubsteps;
    } else {
        world.integrator = freeIntegrator;
        world.substeps = 1; // keep the integrator comparison at the plain step length
    }
    interaction.reset();
    world.solver.settings.iterations = scene.iterations > 0 ? scene.iterations : SolverSettings{}.iterations;
    showJoints = scene.showJoints;
    car.reset();
    cannon.reset();
    world.gravity = {0, -gravity, 0}; // the panel's strength; a scene may turn it off
    scene.build(world);
    if (scene.cannon) cannon.emplace(world); // aimed at the wall for this gravity
    if (scene.car) {
        // Stand it on whatever is below the start point.
        Vec3 start = scene.carStart;
        if (auto hit = world.raycast(start + Vec3{0, 100, 0}, {0, -1, 0}, INFINITY, true)) start = hit->point;
        car = std::make_unique<Car>(world, CarSpec{}, start, scene.carHeading);
        car->chassis().userData = packColor(40, 110, 200); // blue body
        for (RigidBody* b : car->arms())
            b->userData = packColor(35, 35, 40);
        for (RigidBody* b : car->uprights())
            b->userData = packColor(70, 160, 220); // anodised aluminium
        for (RigidBody* b : car->knuckles())
            b->userData = packColor(70, 160, 220);
        throttle = steer = brake = 0;
        booster.reset();
        manualOrbit = 0;
    }
    gravityOn = lengthSquared(world.gravity) > 0; // the scene may have turned it off
    world.detectCollisions();
    accumulator = 0;
    editIndex = -1;
    for (auto& h : omegaHistory)
        h.fill(0);
    energyErrorHistory.fill(0);
    historyPos = 0;
    energy0 = world.totalEnergy();
}

void App::handleInput(bool mouseFree, bool keysFree, float dt) {
    bool mouseTaken = interaction.update(world, orbit.camera(), Input::read(), mouseFree, keysFree);
    if (interaction.removedBodies()) {
        editIndex = -1; // body indices changed
        checkCar();
    }
    // Orbiting by hand: the chase camera leaves the direction alone for a while.
    if (mouseFree && !mouseTaken &&
        (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT)))
        manualOrbit = 2;
    orbit.update(mouseFree && !mouseTaken, keysFree, dt);
    if (car) {
        driveInput(keysFree ? dt : 0);
        chase(dt);
    }
    // Cmd+S (Ctrl+S off macOS) saves a snapshot; with Cmd held the other keys are left alone.
    if (cmdKeyDown()) {
        if (IsKeyPressed(KEY_S)) saveSnapshot();
        return;
    }
    if (cannon && keysFree) cannon->keys(world, dt);
    if (keysFree) {
        if (sceneKeys()) {
            if (IsKeyPressed(KEY_P)) paused = !paused; // Space brakes the car or fires the cannon
        } else {
            if (IsKeyPressed(KEY_SPACE)) paused = !paused;
            if (IsKeyPressed(KEY_S)) stepOnce = true;
        }
        if (IsKeyPressed(KEY_R)) resetScene();
        if (IsKeyPressed(KEY_F)) runAction();
        if (IsKeyPressed(KEY_C)) resetView();
    }
}

void App::recordHistory() {
    if (world.bodies().empty()) return;
    const RigidBody& b = *world.bodies().front();
    // Angular velocity in body coordinates shows the axis flip clearly.
    Vec3 wb = transpose(b.R()) * b.omega();
    for (int k = 0; k < 3; ++k)
        omegaHistory[k][historyPos] = float(wb[k]);
    energyErrorHistory[historyPos] = float(world.totalEnergy() - energy0);
    historyPos = (historyPos + 1) % kHistory;
}

void App::driveInput(float dt) {
    // dt = 0: the keyboard belongs to the panel; let go of everything.
    const bool keys = dt > 0;
    const bool forward = keys && IsKeyDown(KEY_W), back = keys && IsKeyDown(KEY_S);
    const bool left = keys && IsKeyDown(KEY_A), right = keys && IsKeyDown(KEY_D);
    // S brakes while still rolling forward, then reverses (and W the other way round).
    const Real v = car->speed();
    throttle = 0;
    brake = keys && IsKeyDown(KEY_SPACE) ? 1 : 0;
    if (forward) {
        if (v < -0.5)
            brake = 1;
        else
            throttle = 1;
    } else if (back) {
        if (v > 0.5)
            brake = 1;
        else
            throttle = -0.6;
    }
    // Steering eases in and back, like a thumb on a transmitter.
    const Real target = (left ? 1 : 0) - (right ? 1 : 0);
    const Real rate = target == 0 ? 6 : 4;
    const Real step = rate * Real(keys ? dt : GetFrameTime());
    steer += std::clamp(target - steer, -step, step);
    if (keys && IsKeyPressed(KEY_T)) {
        car->flip();
        world.detectCollisions();
    }
    if (keys && IsKeyPressed(KEY_V)) chaseCamera = !chaseCamera;
}

void App::chase(float dt) {
    manualOrbit = std::max(0.0f, manualOrbit - dt);
    if (!chaseCamera) return;
    // Follow the chassis, and swing round behind it unless orbited by hand.
    const Vec3 p = car->chassis().x;
    const float follow = 1 - std::exp(-12 * dt);
    orbit.target.x += (float(p.x) - orbit.target.x) * follow;
    orbit.target.y += (float(p.y) + 0.05f - orbit.target.y) * follow;
    orbit.target.z += (float(p.z) - orbit.target.z) * follow;
    const Vec3 f = car->forward();
    if (manualOrbit > 0 || std::hypot(f.x, f.z) < 0.3) return; // or when it's on its side
    const float behind = float(std::atan2(-f.x, -f.z));
    float turn = std::remainder(behind - orbit.yaw, 2 * std::numbers::pi_v<float>);
    // Faster when driving, so the view isn't dragged round while parked.
    const float rate = 0.5f + float(std::min(std::abs(car->speed()), 4.0)) * 0.6f;
    orbit.yaw += turn * (1 - std::exp(-rate * dt));
}

void App::checkCar() {
    if (car && !car->intact(world)) car.reset(); // its other parts stay, but can't be driven
}

void App::stepOnceNow(double h) {
    if (car) {
        // A boost pad boosts the motor, and on the pad gives full throttle (unless braking).
        const auto& pads = scenes()[sceneIndex].boostPads;
        const Real boost = pads.empty() ? 0 : booster.update(pads, car->chassis().x, car->forward(), h);
        car->drive(booster.onPad() && brake == 0 ? 1 : throttle, steer, brake, boost);
    }
    world.step(h);
    recordHistory();
    const Scene& scene = scenes()[sceneIndex];
    if (scene.update) scene.update(world);
    if (!autoActionDone && scene.action && scene.autoActionAt >= 0 && world.time() >= scene.autoActionAt) {
        autoActionDone = true;
        runAction();
    }
}

void App::runAction() {
    const Scene& scene = scenes()[sceneIndex];
    if (!scene.action) return;
    scene.action(world);
    world.detectCollisions();
}

void App::advance(double frameTime) {
    const double h = 1.0 / stepsPerSecond;
    if (paused) {
        if (stepOnce) stepOnceNow(h);
        stepOnce = false;
        return;
    }
    // Fixed steps for the time that passed. If a frame's steps take longer than
    // the frame budget, stop and drop the rest: a scene too heavy for real time
    // runs in slow motion at a smooth frame rate, instead of each frame owing
    // more steps than the last (the "spiral of death") down to a few frames per second.
    accumulator += std::min(frameTime, 0.1) * timeScale;
    const auto start = std::chrono::steady_clock::now();
    const double budget = 0.020; // s of stepping per frame: about 45 frames per second at worst
    int steps = 0;
    bool behind = false;
    while (accumulator >= h) {
        stepOnceNow(h);
        accumulator -= h;
        ++steps;
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > budget) {
            behind = accumulator >= h;
            accumulator = 0;
            break;
        }
    }
    // How fast simulated time runs against the clock (1 = real time), smoothed for the panel.
    const double wanted = std::min(frameTime, 0.1) * timeScale;
    const double rate = wanted > 0 ? (behind ? steps * h / wanted : 1.0) : 1.0;
    simulationRate += (rate - simulationRate) * 0.1;
}

void App::gui() {
    ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({kPanelWidth, 0}, ImGuiCond_FirstUseEver);
    ImGui::Begin("ncotb");

    if (ImGui::BeginCombo("Scene", scenes()[sceneIndex].name)) {
        for (int i = 0; i < int(scenes().size()); ++i)
            if (ImGui::Selectable(scenes()[i].name, i == sceneIndex)) loadScene(i);
        ImGui::EndCombo();
    }
    if (ImGui::Button("Reset (R)")) resetScene();
    ImGui::SameLine();
    ImGui::Checkbox(sceneKeys() ? "Paused (P)" : "Paused (Space)", &paused);
    ImGui::SameLine();
    if (ImGui::Button("Step (S)")) stepOnce = true;
    if (const char* label = scenes()[sceneIndex].actionLabel) {
        if (ImGui::Button((std::string(label) + " (F)").c_str())) runAction();
    }

    const bool contactScene = scenes()[sceneIndex].contacts;
    ImGui::BeginDisabled(contactScene);
    if (ImGui::BeginCombo("Integrator", name(world.integrator))) {
        for (IntegratorType type : kAllIntegrators)
            if (ImGui::Selectable(name(type), type == world.integrator)) {
                freeIntegrator = type;
                resetScene(); // restart so E - E0 compares like with like
            }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    if (contactScene)
        ImGui::SetItemTooltip("Contacts are resolved between the velocity and position updates,\n"
                              "which only semi-implicit Euler has. See docs/notes.md.");
    ImGui::SliderFloat("Time scale", &timeScale, 0.05f, 2.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderInt("Steps / s", &stepsPerSecond, 30, 1000);
    if (simulationRate < 0.97) {
        ImGui::SameLine();
        ImGui::TextColored({1, 0.6f, 0.2f, 1}, "slow motion %.0f%%", simulationRate * 100);
        ImGui::SetItemTooltip("The steps take longer than real time: the simulation runs slower so\n"
                              "the frame rate stays smooth. Fewer steps or substeps speed it up.");
    }
    ImGui::Checkbox("Show omega (yellow) and L (magenta)", &showVectors);
    bool gravityChanged = ImGui::Checkbox("Gravity", &gravityOn);
    ImGui::SameLine();
    if (ImGui::Button("Earth")) {
        gravity = float(kEarthGravity);
        gravityOn = gravityChanged = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Moon")) {
        gravity = float(kMoonGravity);
        gravityOn = gravityChanged = true;
    }
    ImGui::BeginDisabled(!gravityOn);
    gravityChanged |= ImGui::SliderFloat("g (m/s^2)", &gravity, 0, 30, "%.2f");
    ImGui::EndDisabled();
    if (gravityChanged) {
        world.gravity = gravityOn ? Vec3{0, -gravity, 0} : Vec3{};
        world.wakeAll(); // sleeping bodies don't feel gravity
    }
    ImGui::Checkbox("Show contacts", &showContacts);
    ImGui::SameLine();
    ImGui::Checkbox("Show joints", &showJoints);

    if (car) carGui();
    if (cannon) cannon->gui(world);
    snapshotGui();
    if (contactScene) solverGui();
    contactsGui();
    interaction.gui(world);
    if (interaction.removedBodies()) {
        editIndex = -1;
        checkCar();
    }
    editGui();

    if (!ImGui::CollapsingHeader("Energy and momentum", ImGuiTreeNodeFlags_DefaultOpen)) {
        footer();
        return;
    }
    Real energy = world.totalEnergy();
    Vec3 L{}, P{};
    for (auto& b : world.bodies()) {
        L += b->L + cross(b->x, b->P);
        P += b->P;
    }
    ImGui::Text("t = %.2f s   bodies = %zu", world.time(), world.bodies().size());
    ImGui::Text("Total energy  E   = %.5f J", energy);
    ImGui::Text("Energy error E-E0 = %+.2e J", energy - energy0);
    ImGui::PlotLines("E-E0", energyErrorHistory.data(), kHistory, historyPos, nullptr, FLT_MAX, FLT_MAX,
                     {0, 50});
    ImGui::Text("Ang. momentum |L| = %.5f", length(L));
    ImGui::Text("Lin. momentum |P| = %.5f", length(P));

    if (!world.bodies().empty() && ImGui::CollapsingHeader("Body 0 angular velocity")) {
        ImGui::TextUnformatted("omega in body frame (x, y, z):");
        const char* labels[3] = {"wx", "wy", "wz"};
        for (int k = 0; k < 3; ++k)
            ImGui::PlotLines(labels[k], omegaHistory[k].data(), kHistory, historyPos, nullptr, -8.0f, 8.0f,
                             {0, 50});
    }
    footer();
}

void App::footer() {
    ImGui::Separator();
    if (car) ImGui::TextDisabled("W/S: drive  A/D: steer  Space: brake  T: flip  V: chase  P: pause");
    if (cannon) ImGui::TextDisabled("WASD: aim  Z/X: powder  F/Space: fire  P: pause");
    ImGui::TextDisabled("Drag: orbit   Shift+drag: pan   Scroll: zoom");
    ImGui::TextDisabled("Arrows, Q/E: move   C: reset view   %d FPS", GetFPS());
    ImGui::TextDisabled("Drag a body: grab   Opt+click: push   1/2/3: spawn");
    ImGui::End();
}

void App::carGui() {
    if (!ImGui::CollapsingHeader("RC truck", ImGuiTreeNodeFlags_DefaultOpen)) return;
    ImGui::Text("%5.1f km/h", car->speed() * 3.6);
    ImGui::SameLine();
    ImGui::ProgressBar(float(std::abs(throttle)), {70, 0}, throttle < 0 ? "reverse" : "throttle");
    ImGui::SameLine();
    ImGui::ProgressBar(float(brake), {60, 0}, "brake");
    if (booster.boost() > 0) {
        ImGui::SameLine();
        ImGui::TextColored({1.0f, 0.55f, 0.1f, 1.0f}, "BOOST");
    }
    float steerShown = float(steer); // display only
    ImGui::SliderFloat("Steering", &steerShown, -1, 1, "%.2f", ImGuiSliderFlags_NoInput);
    // Front left, front right | rear left, rear right.
    ImGui::Text("Spin (rad/s)  %4.0f %4.0f | %4.0f %4.0f", car->wheelSpin(0), car->wheelSpin(1),
                car->wheelSpin(2), car->wheelSpin(3));
    ImGui::Text("Travel (mm)   %4.0f %4.0f | %4.0f %4.0f", car->suspensionTravel(0) * 1000,
                car->suspensionTravel(1) * 1000, car->suspensionTravel(2) * 1000,
                car->suspensionTravel(3) * 1000);
    ImGui::SetItemTooltip(
        "Front left, front right | rear left, rear right. Travel: up from the resting position.");
    ImGui::Checkbox("Chase camera (V)", &chaseCamera);
    ImGui::SameLine();
    if (ImGui::Button("Flip back (T)")) {
        car->flip();
        world.detectCollisions();
    }
    CarSpec& s = car->spec();
    auto slider = [](const char* label, Real& value, float lo, float hi, const char* format) {
        float v = float(value);
        bool changed = ImGui::SliderFloat(label, &v, lo, hi, format);
        if (changed) value = v;
        return changed;
    };
    bool retune = slider("Ride frequency (Hz)", s.suspensionFrequency, 1, 8, "%.1f");
    retune |= slider("Damping ratio", s.suspensionDamping, 0.05f, 2, "%.2f");
    if (retune) car->retune();
    slider("Motor torque (N m)", s.motorTorque, 0.05f, 2, "%.2f");
    slider("Top speed (m/s)", s.topSpeed, 2, 30, "%.0f");
    slider("Brake torque (N m)", s.brakeTorque, 0.05f, 4, "%.2f");
    slider("Drag brake (N m)", s.dragBrake, 0, 1, "%.2f");
    ImGui::SetItemTooltip("Braking while the throttle is at neutral, like an RC speed controller's\n"
                          "drag brake (gears, bearings, motor). 0: coasts freely.");
    slider("Max steering (rad)", s.maxSteer, 0.1f, 0.6f, "%.2f");
}

// ---------------------------------------------------------------- snapshots

namespace {
constexpr const char* kSnapshotDir = "snapshots";
constexpr const char* kSnapshotMagic = "ncotb-snapshot";
constexpr int kSnapshotVersion = 1;
} // namespace

std::string App::saveSnapshot() {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(kSnapshotDir, ec);
    // snapshots/20261014-153012-rc-truck.snap
    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", std::localtime(&now));
    std::string slug;
    for (const char* c = scenes()[sceneIndex].name; *c && *c != '('; ++c)
        slug += std::isalnum((unsigned char)*c) ? char(std::tolower((unsigned char)*c)) : '-';
    while (!slug.empty() && slug.back() == '-')
        slug.pop_back();
    std::string path = std::string(kSnapshotDir) + "/" + stamp + "-" + slug + ".snap";

    std::ofstream file(path);
    if (!file) {
        snapshotStatus = "Couldn't write " + path;
        return "";
    }
    SnapshotWriter out(file);
    out.key(kSnapshotMagic) << kSnapshotVersion;
    out.comment(std::string("scene: ") + scenes()[sceneIndex].name);
    out.comment("saved " + std::string(stamp) + ", t = " + std::to_string(world.time()) + " s");
    out.key("app");
    out.key("scene") << sceneIndex;
    out.key("stepsPerSecond") << stepsPerSecond;
    out.key("timeScale") << Real(timeScale);
    out.key("contactSubsteps") << contactSubsteps;
    out.key("camera") << Real(orbit.yaw) << Real(orbit.pitch) << Real(orbit.distance)
                      << Vec3{orbit.target.x, orbit.target.y, orbit.target.z};
    out.key("view") << int(showVectors) << int(showContacts) << int(showNormals) << int(showJoints)
                    << Real(forceScale);
    out.key("autoActionDone") << int(autoActionDone);
    out.key("input") << throttle << steer << brake << int(chaseCamera);
    out.key("spawned") << int(interaction.spawned().size());
    for (const RigidBody* b : interaction.spawned())
        out << ncotb::bodyIndex(world, b);
    out.key("hasCar") << int(car != nullptr);
    if (cannon) out.key("cannon") << cannon->elevation << cannon->azimuth << cannon->speed;
    saveWorld(world, out);
    if (car) saveCar(*car, world, out);
    out.end();
    file.close();
    if (!file) {
        snapshotStatus = "Couldn't write " + path;
        return "";
    }
    snapshotStatus = "Saved " + fs::absolute(path).string();
    listSnapshots();
    return path;
}

bool App::loadSnapshot(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        snapshotStatus = "Couldn't open " + path;
        return false;
    }
    SnapshotReader in(file);
    in.expect(kSnapshotMagic);
    if (in.integer() != kSnapshotVersion && !in.failed()) in.fail("unsupported snapshot version");
    in.expect("app");
    in.expect("scene");
    const int scene = in.integer();
    if (!in.failed() && (scene < 0 || scene >= int(scenes().size())))
        in.fail("no scene " + std::to_string(scene));
    if (in.failed()) {
        snapshotStatus = path + ": " + in.error();
        return false;
    }
    // Set up the scene as usual (hooks, panel defaults), then replace its contents.
    loadScene(scene);
    in.expect("stepsPerSecond");
    stepsPerSecond = in.integer();
    in.expect("timeScale");
    timeScale = float(in.real());
    in.expect("contactSubsteps");
    contactSubsteps = in.integer();
    in.expect("camera");
    orbit.yaw = float(in.real());
    orbit.pitch = float(in.real());
    orbit.distance = float(in.real());
    const Vec3 target = in.vec3();
    orbit.target = {float(target.x), float(target.y), float(target.z)};
    in.expect("view");
    showVectors = in.integer() != 0;
    showContacts = in.integer() != 0;
    showNormals = in.integer() != 0;
    showJoints = in.integer() != 0;
    forceScale = float(in.real());
    in.expect("autoActionDone");
    autoActionDone = in.integer() != 0;
    in.expect("input");
    throttle = in.real();
    steer = in.real();
    brake = in.real();
    chaseCamera = in.integer() != 0;
    in.expect("spawned");
    std::vector<int> spawned(size_t(std::max(in.integer(), 0)));
    for (int& i : spawned)
        i = in.integer();
    in.expect("hasCar");
    const bool hasCar = in.integer() != 0;
    if (in.peek() == "cannon") { // only in scenes with one
        in.expect("cannon");
        const Real elevation = in.real(), azimuth = in.real(), speed = in.real();
        if (cannon) {
            cannon->elevation = elevation;
            cannon->azimuth = azimuth;
            cannon->speed = speed;
        }
    }

    car.reset();
    interaction.reset();
    bool ok = !in.failed() && loadWorld(world, in);
    if (ok && hasCar) {
        car = loadCar(world, in);
        ok = car != nullptr;
    }
    if (!ok) {
        snapshotStatus = path + ": " + in.error();
        loadScene(scene); // back to a working scene
        return false;
    }
    std::vector<RigidBody*> bodies;
    for (int i : spawned)
        if (i >= 0 && i < int(world.bodies().size())) bodies.push_back(world.bodies()[size_t(i)].get());
    interaction.setSpawned(std::move(bodies));
    gravityOn = lengthSquared(world.gravity) > 0; // the panel shows the snapshot's gravity
    if (gravityOn) gravity = float(length(world.gravity));
    world.detectCollisions(); // for display
    energy0 = world.totalEnergy();
    accumulator = 0;
    paused = true; // look at the moment first
    snapshotStatus = "Loaded " + path + " (paused)";
    return true;
}

void App::listSnapshots() {
    namespace fs = std::filesystem;
    snapshotFiles.clear();
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(kSnapshotDir, ec))
        if (entry.path().extension() == ".snap") snapshotFiles.push_back(entry.path().string());
    std::sort(snapshotFiles.rbegin(), snapshotFiles.rend()); // the names start with the time: newest first
    snapshotChoice = std::clamp(snapshotChoice, 0, std::max(int(snapshotFiles.size()) - 1, 0));
}

void App::snapshotGui() {
    if (!ImGui::CollapsingHeader("Snapshots")) return;
    if (ImGui::Button("Save (" NCOTB_CMD "+S)")) saveSnapshot();
    ImGui::SameLine();
    if (ImGui::Button("Refresh list") || snapshotFiles.empty()) listSnapshots();
    if (!snapshotFiles.empty()) {
        const std::string& current = snapshotFiles[size_t(snapshotChoice)];
        const std::string shown = std::filesystem::path(current).filename().string();
        if (ImGui::BeginCombo("File", shown.c_str())) {
            for (int i = 0; i < int(snapshotFiles.size()); ++i) {
                const std::string name = std::filesystem::path(snapshotFiles[size_t(i)]).filename().string();
                if (ImGui::Selectable(name.c_str(), i == snapshotChoice)) snapshotChoice = i;
            }
            ImGui::EndCombo();
        }
        if (ImGui::Button("Load")) loadSnapshot(snapshotFiles[size_t(snapshotChoice)]);
    }
    if (!snapshotStatus.empty()) ImGui::TextWrapped("%s", snapshotStatus.c_str());
}

void App::solverGui() {
    if (!ImGui::CollapsingHeader("Solver (contacts and joints)", ImGuiTreeNodeFlags_DefaultOpen)) return;
    SolverSettings& cs = world.solver.settings;
    if (ImGui::BeginCombo("Solver", name(cs.type))) {
        for (SolverType t : {SolverType::SequentialImpulses, SolverType::BaraffLcp})
            if (ImGui::Selectable(name(t), t == cs.type)) cs.type = t;
        ImGui::EndCombo();
    }
    if (ImGui::SliderInt("Substeps", &contactSubsteps, 1, 16)) world.substeps = contactSubsteps;
    if (ImGui::Checkbox("Sleeping", &world.sleeping) && !world.sleeping) world.wakeAll();
    ImGui::SetItemTooltip("Bodies at rest for %.1f s are left out of the step until something\n"
                          "touches or pushes them.",
                          world.sleepAfter);
    ImGui::SameLine();
    ImGui::Checkbox("darker", &showSleeping);
    ImGui::SameLine();
    ImGui::Text("%d asleep", world.sleepingBodies());
    ImGui::BeginDisabled(cs.type != SolverType::SequentialImpulses);
    ImGui::SliderInt("Iterations", &cs.iterations, 1, 100);
    ImGui::Checkbox("Warm start", &cs.warmStart);
    ImGui::EndDisabled();
    ImGui::Checkbox("Joint position projection", &cs.jointProjection);
    ImGui::SetItemTooltip("On: joints that drifted apart are moved back together after each step.\n"
                          "Off: they get a correction velocity (Baumgarte), which can keep a car\n"
                          "rocking. See docs/notes.md.");
    if (cs.jointProjection) {
        ImGui::SliderInt("Projection passes", &cs.positionIterations, 1, 16);
        float fraction = float(cs.projection);
        if (ImGui::SliderFloat("Projection fraction", &fraction, 0.1f, 1)) cs.projection = fraction;
    }
    float beta = float(cs.baumgarte), slopMm = float(cs.slop * 1000);
    if (ImGui::SliderFloat("Baumgarte", &beta, 0, 1)) cs.baumgarte = beta;
    if (ImGui::SliderFloat("Slop (mm)", &slopMm, 0, 20)) cs.slop = slopMm / 1000;
    float maxPush = float(cs.maxPushVelocity);
    if (ImGui::SliderFloat("Max push (m/s)", &maxPush, 0.1f, 10, "%.1f", ImGuiSliderFlags_Logarithmic))
        cs.maxPushVelocity = maxPush;
    ImGui::Checkbox("Relax (remove push-out velocity)", &cs.relax);
    ImGui::Checkbox("Friction", &cs.friction);
    if (cs.type == SolverType::BaraffLcp && cs.friction)
        ImGui::SliderInt("Normal/friction rounds", &cs.staggers, 1, 10);
    ImGui::Checkbox("Resolve contacts", &world.resolveContacts);
    const SolverStats& st = world.solver.stats();
    if (cs.type == SolverType::BaraffLcp && st.lcpTooLarge)
        ImGui::TextColored({1, 0.7f, 0.3f, 1}, "%d rows > %d: used sequential impulses", st.rows,
                           cs.lcpMaxRows);
    else if (cs.type == SolverType::BaraffLcp)
        ImGui::Text("%d rows, %d pivots%s", st.rows, st.pivots, st.lcpFailed ? "  PIVOT LIMIT HIT" : "");
    else
        ImGui::Text("%d rows", st.rows);
}

void App::contactsGui() {
    if (!ImGui::CollapsingHeader("Contacts", ImGuiTreeNodeFlags_DefaultOpen)) return;
    ImGui::Checkbox("Normals", &showNormals);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    ImGui::SliderFloat("Force arrows (m/N)", &forceScale, 0, 0.2f);
    ImGui::TextDisabled("green: normal force, blue: friction (on body a)");
    int points = 0;
    Real maxDepth = 0;
    for (const ContactManifold& m : world.contacts()) {
        points += m.count;
        for (int i = 0; i < m.count; ++i)
            maxDepth = std::max(maxDepth, m.points[i].depth);
    }
    ImGui::Text("%zu contacting pairs, %d points, max depth %.3f", world.contacts().size(), points, maxDepth);
    if (ImGui::BeginTable("contacts", 5, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("a");
        ImGui::TableSetupColumn("b");
        ImGui::TableSetupColumn("points");
        ImGui::TableSetupColumn("normal (b to a)");
        ImGui::TableSetupColumn("force (N)");
        ImGui::TableHeadersRow();
        for (const ContactManifold& m : world.contacts()) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", bodyIndex(m.a));
            ImGui::TableNextColumn();
            ImGui::Text("%d", bodyIndex(m.b));
            ImGui::TableNextColumn();
            ImGui::Text("%d", m.count);
            ImGui::TableNextColumn();
            ImGui::Text("%+.2f %+.2f %+.2f", m.normal.x, m.normal.y, m.normal.z);
            ImGui::TableNextColumn();
            Real impulse = 0;
            for (int k = 0; k < m.count; ++k)
                impulse += m.points[k].normalImpulse;
            ImGui::Text("%.2f", world.lastSubstep() > 0 ? impulse / world.lastSubstep() : 0.0);
        }
        ImGui::EndTable();
    }
}

int App::bodyIndex(const RigidBody* b) const {
    for (size_t i = 0; i < world.bodies().size(); ++i)
        if (world.bodies()[i].get() == b) return int(i);
    return -1;
}

const char* App::shapeName(const RigidBody& b) {
    switch (b.shape().type) {
    case Shape::Type::Sphere: return "sphere";
    case Shape::Type::Box: return "box";
    case Shape::Type::Plane: return "plane";
    case Shape::Type::Heightfield: return "terrain";
    }
    return "?";
}

void App::editGui() {
    if (!ImGui::CollapsingHeader("Move a body")) return;
    const auto& bodies = world.bodies();
    std::string current =
        editIndex >= 0 ? std::to_string(editIndex) + " " + shapeName(*bodies[editIndex]) : "none";
    if (ImGui::BeginCombo("Body", current.c_str())) {
        for (int i = 0; i < int(bodies.size()); ++i) {
            if (bodies[i]->shape().type == Shape::Type::Plane ||
                bodies[i]->shape().type == Shape::Type::Heightfield)
                continue;
            std::string label = std::to_string(i) + " " + shapeName(*bodies[i]);
            if (ImGui::Selectable(label.c_str(), i == editIndex)) {
                editIndex = i;
                editBaseQ = bodies[i]->q;
                for (float& e : editEulerDeg)
                    e = 0;
            }
        }
        ImGui::EndCombo();
    }
    if (editIndex < 0) return;

    RigidBody& b = *bodies[editIndex];
    float pos[3] = {float(b.x.x), float(b.x.y), float(b.x.z)};
    bool changed = ImGui::DragFloat3("Position", pos, 0.01f);
    changed |= ImGui::DragFloat3("Rotate x/y/z (deg)", editEulerDeg, 0.5f);
    if (changed) {
        b.x = {pos[0], pos[1], pos[2]};
        const Real k = std::numbers::pi / 180;
        Quat r = Quat::fromAxisAngle({0, 0, 1}, editEulerDeg[2] * k) *
                 Quat::fromAxisAngle({0, 1, 0}, editEulerDeg[1] * k) *
                 Quat::fromAxisAngle({1, 0, 0}, editEulerDeg[0] * k);
        b.q = normalize(r * editBaseQ);
        b.updateDerived();
        b.wake();
        world.detectCollisions();
    }
}

void App::drawScene() const {
    const Color palette[] = {{230, 120, 70, 255}, {80, 150, 220, 255},  {120, 190, 100, 255},
                             {220, 190, 70, 255}, {170, 110, 200, 255}, {90, 200, 190, 255}};

    releaseUnusedTerrainMeshes();
    const bool terrain = std::any_of(world.bodies().begin(), world.bodies().end(), [](const auto& b) {
        return b->shape().type == Shape::Type::Heightfield;
    });
    if (!terrain) DrawGrid(40, 1.0f); // it would cut through the hills
    int i = 0;
    for (auto& b : world.bodies()) {
        if (b->shape().type == Shape::Type::Plane) continue; // drawn below
        if (car && std::find(car->wheels().begin(), car->wheels().end(), b.get()) != car->wheels().end())
            continue; // drawn as tyres below
        if (b->isStatic()) {
            drawBody(*b, colorOf(*b, {150, 155, 165, 255}));
            continue;
        }
        Color c = colorOf(*b, palette[i++ % std::size(palette)]);
        if (showSleeping && b->asleep)
            c = {(unsigned char)(c.r * 0.45), (unsigned char)(c.g * 0.45), (unsigned char)(c.b * 0.45), c.a};
        drawBody(*b, c);
        if (showVectors) {
            drawArrow(b->x, b->omega() * 0.3, {200, 160, 0, 255});
            drawArrow(b->x, b->L * 0.3, MAGENTA);
        }
    }
    if (const auto& decorate = scenes()[sceneIndex].decorate) decorate(world);
    for (const BoostPad& pad : scenes()[sceneIndex].boostPads)
        drawBoostPad(pad, GetTime());
    if (cannon) cannon->draw(world);
    drawJoints(world, showJoints);
    if (car)
        for (int c = 0; c < 4; ++c)
            drawTyre(car->wheel(c), car->axle(c), car->spec().wheelRadius, car->spec().wheelWidth);
    drawSprings(world);
    // Translucent planes last and without depth writes, so bodies that sink
    // below them stay visible.
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    for (auto& b : world.bodies())
        if (b->shape().type == Shape::Type::Plane) drawBody(*b, {120, 130, 145, 90});
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
    if (showContacts) {
        // Overlay without depth test: force arrows point into body a and
        // would otherwise be hidden inside it.
        rlDrawRenderBatchActive();
        rlDisableDepthTest();
        drawContacts(world.contacts(), showNormals, forceScale, world.lastSubstep());
        rlDrawRenderBatchActive();
        rlEnableDepthTest();
    }
    interaction.draw(world);
}

} // namespace ncotb::app
