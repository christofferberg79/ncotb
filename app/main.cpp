// ncotb — rigid body sandbox (raylib + Dear ImGui front-end for the physics library).

#include "world.h"

#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>
#include <rlgl.h>

#include <algorithm>
#include <array>
#include <cfloat>
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
    // clang-format off
    float m[16] = {float(R.m[0][0]), float(R.m[1][0]), float(R.m[2][0]), 0,
                   float(R.m[0][1]), float(R.m[1][1]), float(R.m[2][1]), 0,
                   float(R.m[0][2]), float(R.m[1][2]), float(R.m[2][2]), 0,
                   float(b.x.x),     float(b.x.y),     float(b.x.z),     1};
    // clang-format on
    rlPushMatrix();
    rlMultMatrixf(m);
    const Shape& s = b.shape();
    switch (s.type) {
    case Shape::Type::Box: {
        ::Vector3 size = toRl(2 * s.halfExtents);
        DrawCubeV({0, 0, 0}, size, color);
        DrawCubeWiresV({0, 0, 0}, size, BLACK);
        break;
    }
    case Shape::Type::Sphere:
        DrawSphereEx({0, 0, 0}, float(s.radius), 16, 16, color);
        DrawSphereWires({0, 0, 0}, float(s.radius) * 1.002f, 8, 8, BLACK);
        break;
    case Shape::Type::Plane: {
        // Infinite plane drawn as a 20 x 20 patch in the body's xz-plane.
        const float half = 10;
        DrawPlane({0, -0.001f, 0}, {2 * half, 2 * half}, color);
        for (float t = -half; t <= half; t += 1) {
            DrawLine3D({-half, 0, t}, {half, 0, t}, {0, 0, 0, 40});
            DrawLine3D({t, 0, -half}, {t, 0, half}, {0, 0, 0, 40});
        }
        break;
    }
    }
    rlPopMatrix();
}

void drawArrow(const Vec3& from, const Vec3& dir, Color color) {
    Vec3 to = from + dir;
    DrawLine3D(toRl(from), toRl(to), color);
    DrawSphere(toRl(to), 0.04f, color);
}

/// Contact points (red), normals from b towards a (orange) and penetration
/// depth drawn along the normal (dark red).
/// Contact forces (green) on body a: impulse / step length, scaled to metres.
void drawContacts(const std::vector<ContactManifold>& contacts, bool normals, Real forceScale, Real h) {
    for (const ContactManifold& m : contacts) {
        for (int i = 0; i < m.count; ++i) {
            const ContactPoint& p = m.points[i];
            DrawSphere(toRl(p.position), 0.06f, RED);
            if (normals) drawArrow(p.position, m.normal * 0.5, ORANGE);
            Vec3 half = m.normal * (p.depth / 2);
            DrawLine3D(toRl(p.position - half), toRl(p.position + half), MAROON);
            if (forceScale > 0 && p.normalImpulse > 0)
                drawArrow(p.position, m.normal * (p.normalImpulse / h * forceScale), {0, 160, 60, 255});
        }
    }
}

// ---------------------------------------------------------------- scenes

struct Scene {
    const char* name;
    /// Contact scenes use semi-implicit Euler (the only integrator that resolves
    /// contacts) and show the contact solver settings.
    bool contacts;
    std::function<void(World&)> build;
};

/// Adds a dynamic body at x with orientation q.
RigidBody& place(World& w, const Shape& shape, Real mass, const Vec3& x, const Quat& q = {}) {
    RigidBody& b = w.addBody(shape, mass);
    b.x = x;
    b.q = q;
    b.updateDerived();
    return b;
}

const std::vector<Scene>& scenes() {
    static const std::vector<Scene> list = {
        {"Intermediate axis (Dzhanibekov)", false,
         [](World& w) {
             w.gravity = {};
             RigidBody& b = w.addBody(Shape::box({1.2, 0.6, 0.2}), 1.0);
             b.x = {0, 2, 0};
             b.setAngularVelocity({0.01, 5.0, 0.01});
         }},
        {"Three principal axes", false,
         [](World& w) {
             w.gravity = {};
             const Vec3 spins[3] = {{6, 0.05, 0.05}, {0.05, 6, 0.05}, {0.05, 0.05, 6}};
             for (int i = 0; i < 3; ++i) {
                 RigidBody& b = w.addBody(Shape::box({1.0, 0.5, 0.15}), 1.0);
                 b.x = {(i - 1) * 3.5, 2, 0};
                 b.setAngularVelocity(spins[i]);
             }
         }},
        {"Thrown boxes and balls", false,
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
        {"Collision test (move bodies in the panel)", true,
         [](World& w) {
             w.gravity = {};
             w.addBody(Shape::plane(), 0); // ground
             RigidBody& ramp = w.addBody(Shape::plane(), 0);
             ramp.x = {6, 0, 0};
             ramp.q = Quat::fromAxisAngle({0, 0, 1}, 0.5);
             RigidBody& block = w.addBody(Shape::box({1, 0.5, 1}), 0);
             block.x = {-3, 0.5, 0};
             RigidBody& b = w.addBody(Shape::box({0.5, 0.5, 0.5}), 1);
             b.x = {-3, 1.45, 0.3};
             b.q = Quat::fromAxisAngle({0, 1, 0}, 0.4);
             RigidBody& s = w.addBody(Shape::sphere(0.5), 1);
             s.x = {0, 0.45, 0};
             RigidBody& b2 = w.addBody(Shape::box({0.4, 0.3, 0.6}), 1);
             b2.x = {1.4, 0.6, 0};
             b2.q = Quat::fromAxisAngle({1, 0, 1}, 0.6);
             for (auto& body : w.bodies())
                 body->updateDerived();
         }},
        {"Balls and boxes", true,
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             w.addBody(Shape::plane(), 0);
             RigidBody& block = w.addBody(Shape::box({1.5, 0.4, 1.0}), 0);
             block.x = {-1.5, 0.4, 0};
             block.updateDerived();
             for (int i = 0; i < 12; ++i) {
                 bool box = i % 2 == 0;
                 Shape shape = box ? Shape::box({0.35, 0.25, 0.3}) : Shape::sphere(0.3);
                 Vec3 x{(i % 4 - 1.5) * 1.1, 2.0 + 0.8 * i, (i % 3 - 1) * 0.9};
                 RigidBody& b = place(w, shape, 1.0, x, Quat::fromAxisAngle({1, 2, 3}, 0.7 * i));
                 b.restitution = box ? 0.2 : 0.5;
                 b.setAngularVelocity({0.5, 0.8 * (i % 5 - 2), 0.3});
             }
         }},
        {"Box stack", true,
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             w.addBody(Shape::plane(), 0).restitution = 0;
             for (int i = 0; i < 6; ++i)
                 place(w, Shape::box({0.5, 0.5, 0.5}), 1, {-2, 0.5 + 1.0 * i, 0}).restitution = 0;
             // pyramid of 3 + 2 + 1 boxes
             for (int row = 0; row < 3; ++row)
                 for (int k = 0; k < 3 - row; ++k)
                     place(w, Shape::box({0.4, 0.4, 0.4}), 1,
                           {1.5 + (k - (2 - row) / 2.0) * 0.82, 0.4 + 0.8 * row, 0})
                         .restitution = 0;
         }},
        {"Restitution (e = 0, 0.25, 0.5, 0.75, 1)", true,
         [](World& w) {
             w.gravity = {0, -9.81, 0};
             w.addBody(Shape::plane(), 0).restitution = 0;
             for (int i = 0; i < 5; ++i)
                 place(w, Shape::sphere(0.3), 1, {(i - 2) * 1.0, 2.3, 0}).restitution = 0.25 * i;
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
    bool showContacts = true;
    bool showNormals = false;
    float forceScale = 0.05f; ///< metres per newton for the force arrows (0: off)
    int contactSubsteps = 4;  ///< substeps in contact scenes
    Vec3 sceneGravity{};      ///< the scene's own gravity, for the on/off switch
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

    void loadScene(int i) {
        sceneIndex = i;
        world.clear();
        const Scene& scene = scenes()[i];
        if (scene.contacts) {
            world.integrator = IntegratorType::SemiImplicitEuler;
            world.substeps = contactSubsteps;
        } else {
            world.integrator = freeIntegrator;
            world.substeps = 1; // keep the integrator comparison at the plain step length
        }
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

    void recordHistory() {
        if (world.bodies().empty()) return;
        const RigidBody& b = *world.bodies().front();
        // Angular velocity in body coordinates shows the axis flip clearly.
        Vec3 wb = transpose(b.R()) * b.omega();
        for (int k = 0; k < 3; ++k)
            omegaHistory[k][historyPos] = float(wb[k]);
        energyErrorHistory[historyPos] = float(world.totalEnergy() - energy0);
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

        const bool contactScene = scenes()[sceneIndex].contacts;
        ImGui::BeginDisabled(contactScene);
        if (ImGui::BeginCombo("Integrator", name(world.integrator))) {
            for (IntegratorType type : kAllIntegrators)
                if (ImGui::Selectable(name(type), type == world.integrator)) {
                    freeIntegrator = type;
                    loadScene(sceneIndex); // restart so E - E0 compares like with like
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
                ImGui::PlotLines(labels[k], omegaHistory[k].data(), kHistory, historyPos, nullptr, -8.0f,
                                 8.0f, {0, 50});
        }
        footer();
    }

    void footer() {
        ImGui::Separator();
        ImGui::TextDisabled("Drag: orbit   Wheel: zoom   %d FPS", GetFPS());
        ImGui::End();
    }

    void solverGui() {
        if (!ImGui::CollapsingHeader("Contact solver", ImGuiTreeNodeFlags_DefaultOpen)) return;
        ContactSettings& cs = world.solver.settings;
        if (ImGui::BeginCombo("Solver", name(cs.type))) {
            for (ContactSolverType t : {ContactSolverType::SequentialImpulses, ContactSolverType::BaraffLcp})
                if (ImGui::Selectable(name(t), t == cs.type)) cs.type = t;
            ImGui::EndCombo();
        }
        if (ImGui::SliderInt("Substeps", &contactSubsteps, 1, 16)) world.substeps = contactSubsteps;
        ImGui::BeginDisabled(cs.type != ContactSolverType::SequentialImpulses);
        ImGui::SliderInt("Iterations", &cs.iterations, 1, 50);
        ImGui::Checkbox("Warm start", &cs.warmStart);
        ImGui::EndDisabled();
        float beta = float(cs.baumgarte), slopMm = float(cs.slop * 1000);
        if (ImGui::SliderFloat("Baumgarte", &beta, 0, 1)) cs.baumgarte = beta;
        if (ImGui::SliderFloat("Slop (mm)", &slopMm, 0, 20)) cs.slop = slopMm / 1000;
        ImGui::Checkbox("Resolve contacts", &world.resolveContacts);
        const ContactSolverStats& st = world.solver.stats();
        if (cs.type == ContactSolverType::BaraffLcp)
            ImGui::Text("%d rows, %d pivots%s", st.rows, st.pivots, st.lcpFailed ? "  PIVOT LIMIT HIT" : "");
        else
            ImGui::Text("%d rows", st.rows);
    }

    void contactsGui() {
        if (!ImGui::CollapsingHeader("Contacts", ImGuiTreeNodeFlags_DefaultOpen)) return;
        ImGui::Checkbox("Normals", &showNormals);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::SliderFloat("Force arrows (m/N)", &forceScale, 0, 0.2f);
        int points = 0;
        Real maxDepth = 0;
        for (const ContactManifold& m : world.contacts()) {
            points += m.count;
            for (int i = 0; i < m.count; ++i)
                maxDepth = std::max(maxDepth, m.points[i].depth);
        }
        ImGui::Text("%zu contacting pairs, %d points, max depth %.3f", world.contacts().size(), points,
                    maxDepth);
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

    int bodyIndex(const RigidBody* b) const {
        for (size_t i = 0; i < world.bodies().size(); ++i)
            if (world.bodies()[i].get() == b) return int(i);
        return -1;
    }

    static const char* shapeName(const RigidBody& b) {
        switch (b.shape().type) {
        case Shape::Type::Sphere: return "sphere";
        case Shape::Type::Box: return "box";
        case Shape::Type::Plane: return "plane";
        }
        return "?";
    }

    void editGui() {
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
            const Real k = M_PI / 180;
            Quat r = Quat::fromAxisAngle({0, 0, 1}, editEulerDeg[2] * k) *
                     Quat::fromAxisAngle({0, 1, 0}, editEulerDeg[1] * k) *
                     Quat::fromAxisAngle({1, 0, 0}, editEulerDeg[0] * k);
            b.q = normalize(r * editBaseQ);
            b.updateDerived();
            world.detectCollisions();
        }
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

    const Color palette[] = {{230, 120, 70, 255}, {80, 150, 220, 255},  {120, 190, 100, 255},
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
            if (b->shape().type == Shape::Type::Plane) continue; // drawn below
            if (b->isStatic()) {
                drawBody(*b, {150, 155, 165, 255});
                continue;
            }
            drawBody(*b, palette[i++ % std::size(palette)]);
            if (app.showVectors) {
                drawArrow(b->x, b->omega() * 0.3, {200, 160, 0, 255});
                drawArrow(b->x, b->L * 0.3, MAGENTA);
            }
        }
        // Translucent planes last and without depth writes, so bodies that sink
        // below them stay visible.
        rlDrawRenderBatchActive();
        rlDisableDepthMask();
        for (auto& b : app.world.bodies())
            if (b->shape().type == Shape::Type::Plane) drawBody(*b, {120, 130, 145, 90});
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        if (app.showContacts) {
            // Overlay without depth test: force arrows point into body a and
            // would otherwise be hidden inside it.
            rlDrawRenderBatchActive();
            rlDisableDepthTest();
            drawContacts(app.world.contacts(), app.showNormals, app.forceScale, app.world.lastSubstep());
            rlDrawRenderBatchActive();
            rlEnableDepthTest();
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
