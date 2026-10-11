#include "interaction.h"

#include "draw.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>

namespace ncotb::app {

namespace {

Vec3 toVec(const ::Vector3& v) { return {v.x, v.y, v.z}; }

/// Unit vector from the camera towards its target.
Vec3 viewDirection(const Camera3D& c) { return normalize(toVec(c.target) - toVec(c.position)); }

Real weightOf(const World& world, const RigidBody& b) {
    Real g = length(world.gravity);
    return b.mass() * (g > 0 ? g : 9.81); // in zero gravity, still scale by a normal weight
}

} // namespace

Input Input::read() {
    Input in;
    in.mouse = GetMousePosition();
    in.leftDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    in.leftPressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    in.wheel = GetMouseWheelMove();
    in.shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
    in.option = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);
    in.key1 = IsKeyPressed(KEY_ONE);
    in.key2 = IsKeyPressed(KEY_TWO);
    in.key3 = IsKeyPressed(KEY_THREE);
    in.backspace = IsKeyPressed(KEY_BACKSPACE);
    in.dt = GetFrameTime();
    return in;
}

void Interaction::reset() {
    hovered_ = nullptr;
    grab_ = nullptr; // the joint went with the old world
    mouseOwned_ = false;
    pushFlash_ = 0;
    spawned_.clear();
}

bool Interaction::update(World& world, const Camera3D& camera, const Input& in, bool mouseFree,
                         bool keysFree) {
    removedBodies_ = false;
    pushFlash_ = std::max(0.0f, pushFlash_ - in.dt);
    const Ray ray = GetScreenToWorldRay(in.mouse, camera);
    const Vec3 origin = toVec(ray.position), dir = toVec(ray.direction);

    // Hover: the dynamic body under the cursor (not while dragging).
    std::optional<RayHit> hit;
    if (mouseFree && !grab_) hit = world.raycast(origin, dir);
    hovered_ = grab_ ? &grab_->bodyB() : (hit ? hit->body : nullptr);

    if (mouseFree && hit && !in.shift && in.leftPressed) {
        mouseOwned_ = true;
        if (in.option) {
            // Push: a velocity change pushSpeed along the view ray, at the hit point.
            hit->body->applyImpulse(dir * (hit->body->mass() * pushSpeed), hit->point);
            pushPoint_ = hit->point;
            pushDir_ = dir;
            pushFlash_ = 0.4f;
        } else {
            startGrab(world, *hit, camera);
        }
    }

    if (grab_) {
        // Keep the target on the plane facing the camera at the grab depth;
        // scrolling moves that plane.
        grabDepth_ = std::max(0.5f, grabDepth_ * (1 + in.wheel * 0.05f));
        Real along = dot(dir, viewDirection(camera));
        if (along > 1e-3) grab_->setTarget(origin + dir * (grabDepth_ / along));
        grab_->setStiffness(grabFrequency, grabDampingRatio);
        grab_->setMaxForce(grabMaxForceG * weightOf(world, grab_->bodyB()));
        grab_->setAngularDamping(grabAngularDamping);
    }
    if (mouseOwned_ && !in.leftDown) {
        mouseOwned_ = false;
        releaseGrab(world);
    }

    if (keysFree) {
        if (in.key1) spawn(world, origin, dir, 1);
        if (in.key2) spawn(world, origin, dir, 2);
        if (in.key3) spawn(world, origin, dir, 3);
        if (in.backspace && hovered_) remove(world, *hovered_);
    }
    return mouseOwned_;
}

void Interaction::startGrab(World& world, const RayHit& hit, const Camera3D& camera) {
    grabDepth_ = float(dot(hit.point - toVec(camera.position), viewDirection(camera)));
    grab_ = &world.addMouseJoint(*hit.body, hit.point, grabFrequency, grabDampingRatio,
                                 grabMaxForceG * weightOf(world, *hit.body));
    grab_->setAngularDamping(grabAngularDamping);
}

void Interaction::releaseGrab(World& world) {
    if (grab_) world.removeJoint(*grab_);
    grab_ = nullptr;
}

void Interaction::spawn(World& world, const Vec3& origin, const Vec3& dir, int key) {
    const Real s = spawnSize;
    RigidBody* body = nullptr;
    if (key == 3) {
        // Throw a ball from just in front of the camera towards the cursor.
        body = &world.addBody(Shape::sphere(s), spawnMass, origin + dir * (2 * s + 0.5));
        body->setLinearVelocity(dir * Real(throwSpeed));
    } else {
        // Drop: 2 m above whatever is under the cursor, static bodies included.
        auto hit = world.raycast(origin, dir, INFINITY, true);
        if (!hit) return;
        Shape shape = key == 1 ? Shape::box({s, s, s}) : Shape::sphere(s);
        body = &world.addBody(shape, spawnMass, hit->point + Vec3{0, 2 + s, 0},
                              Quat::fromAxisAngle({1, 2, 3}, 0.5 * double(spawned_.size())));
    }
    spawned_.push_back(body);
}

void Interaction::remove(World& world, RigidBody& body) {
    if (grab_ && &grab_->bodyB() == &body) releaseGrab(world); // the joint goes with the body anyway
    std::erase(spawned_, &body);
    if (hovered_ == &body) hovered_ = nullptr;
    world.removeBody(body);
    removedBodies_ = true;
}

void Interaction::draw(const World& world) const {
    (void)world;
    if (hovered_ && !grab_) drawOutline(*hovered_, {255, 200, 0, 255});
    if (grab_) {
        // Spring from the grabbed point to the cursor, green to red with the force.
        Real f = grab_->maxForce() > 0 ? std::min(grab_->force() / grab_->maxForce(), Real(1)) : 0;
        Color c{(unsigned char)(255 * f), (unsigned char)(200 * (1 - f)), 40, 255};
        drawOutline(grab_->bodyB(), c);
        DrawLine3D(toRl(grab_->anchorB()), toRl(grab_->target()), c);
        DrawSphere(toRl(grab_->anchorB()), 0.04f, c);
        DrawSphere(toRl(grab_->target()), 0.06f, c);
    }
    if (pushFlash_ > 0) drawArrow(pushPoint_ - pushDir_ * 0.6, pushDir_ * 0.6, {230, 60, 30, 255});
}

void Interaction::gui(World& world) {
    if (!ImGui::CollapsingHeader("Interaction")) return;
    ImGui::TextDisabled("drag a body: grab   Option+click: push");
    ImGui::TextDisabled("1/2: drop box/ball   3: throw   Backspace: delete");
    ImGui::SliderFloat("Grab stiffness (Hz)", &grabFrequency, 0.5f, 20, "%.1f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Grab damping ratio", &grabDampingRatio, 0, 2);
    ImGui::SliderFloat("Grab max force (x weight)", &grabMaxForceG, 1, 100, "%.0f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Calm held body (1/s)", &grabAngularDamping, 0, 30);
    if (grab_) ImGui::Text("Grab force: %.1f N", grab_->force());
    ImGui::SliderFloat("Push (m/s)", &pushSpeed, 0.2f, 20, "%.1f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Throw (m/s)", &throwSpeed, 2, 40);
    ImGui::SliderFloat("Spawn size (m)", &spawnSize, 0.05f, 1);
    ImGui::SliderFloat("Spawn mass (kg)", &spawnMass, 0.1f, 50, "%.1f", ImGuiSliderFlags_Logarithmic);
    ImGui::Text("%zu spawned", spawned_.size());
    ImGui::SameLine();
    if (ImGui::Button("Remove spawned")) {
        while (!spawned_.empty())
            remove(world, *spawned_.back());
    }
}

} // namespace ncotb::app
