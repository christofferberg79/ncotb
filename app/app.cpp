#include "app.h"

#include "draw.h"
#include "scenes.h"

#include <imgui.h>
#include <raylib.h>
#include <rlgl.h>

#include <algorithm>
#include <cfloat>
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
    scene.build(world);
    sceneGravity = world.gravity;
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
    if (interaction.removedBodies()) editIndex = -1; // body indices changed
    orbit.update(mouseFree && !mouseTaken, keysFree, dt);
    if (keysFree) {
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_S)) stepOnce = true;
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

void App::stepOnceNow(double h) {
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
    accumulator += std::min(frameTime, 0.1) * timeScale;
    while (accumulator >= h) {
        stepOnceNow(h);
        accumulator -= h;
    }
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
    ImGui::Checkbox("Paused (Space)", &paused);
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
    ImGui::Checkbox("Show omega (yellow) and L (magenta)", &showVectors);
    bool gravityOn = lengthSquared(world.gravity) > 0;
    if (ImGui::Checkbox("Gravity", &gravityOn))
        world.gravity =
            gravityOn ? (lengthSquared(sceneGravity) > 0 ? sceneGravity : Vec3{0, -9.81, 0}) : Vec3{};
    ImGui::Checkbox("Show contacts", &showContacts);

    if (contactScene) solverGui();
    contactsGui();
    interaction.gui(world);
    if (interaction.removedBodies()) editIndex = -1;
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
    ImGui::TextDisabled("Drag: orbit   Shift+drag: pan   Scroll: zoom");
    ImGui::TextDisabled("Arrows, Q/E: move   C: reset view   %d FPS", GetFPS());
    ImGui::TextDisabled("Drag a body: grab   Opt+click: push   1/2/3: spawn");
    ImGui::End();
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
    ImGui::BeginDisabled(cs.type != SolverType::SequentialImpulses);
    ImGui::SliderInt("Iterations", &cs.iterations, 1, 50);
    ImGui::Checkbox("Warm start", &cs.warmStart);
    ImGui::EndDisabled();
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
            if (bodies[i]->shape().type == Shape::Type::Plane) continue;
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
        world.detectCollisions();
    }
}

void App::drawScene() const {
    const Color palette[] = {{230, 120, 70, 255}, {80, 150, 220, 255},  {120, 190, 100, 255},
                             {220, 190, 70, 255}, {170, 110, 200, 255}, {90, 200, 190, 255}};

    DrawGrid(40, 1.0f);
    int i = 0;
    for (auto& b : world.bodies()) {
        if (b->shape().type == Shape::Type::Plane) continue; // drawn below
        if (b->isStatic()) {
            drawBody(*b, colorOf(*b, {150, 155, 165, 255}));
            continue;
        }
        drawBody(*b, colorOf(*b, palette[i++ % std::size(palette)]));
        if (showVectors) {
            drawArrow(b->x, b->omega() * 0.3, {200, 160, 0, 255});
            drawArrow(b->x, b->L * 0.3, MAGENTA);
        }
    }
    if (const auto& decorate = scenes()[sceneIndex].decorate) decorate();
    drawJoints(world);
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
