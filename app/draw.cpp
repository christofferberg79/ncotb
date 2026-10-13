#include "draw.h"

#include <rlgl.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace ncotb::app {

namespace {
/// Pushes the body's transform (R and x) onto the matrix stack.
void pushTransform(const RigidBody& b) {
    const Mat3& R = b.R();
    // clang-format off
    float m[16] = {float(R.m[0][0]), float(R.m[1][0]), float(R.m[2][0]), 0,
                   float(R.m[0][1]), float(R.m[1][1]), float(R.m[2][1]), 0,
                   float(R.m[0][2]), float(R.m[1][2]), float(R.m[2][2]), 0,
                   float(b.x.x),     float(b.x.y),     float(b.x.z),     1};
    // clang-format on
    rlPushMatrix();
    rlMultMatrixf(m);
}

// ---------------------------------------------------------------- terrain

/// Uploaded terrain meshes, one per heightfield. Holding the heightfield keeps
/// its address from being reused by a new one while the mesh is cached.
struct TerrainMesh {
    std::shared_ptr<const Heightfield> terrain;
    Model model;
};
std::vector<TerrainMesh> terrainMeshes;

/// Terrain colour: grass in the valleys, earth, rock on the heights and on steep
/// slopes, snow on the tops.
Vec3 terrainColor(Real t, Real steep) {
    const Vec3 grass{88, 138, 68}, earth{140, 122, 84}, rock{132, 128, 122}, snow{236, 238, 242};
    auto mix = [](const Vec3& a, const Vec3& b, Real f) {
        return a + (b - a) * std::clamp(f, Real(0), Real(1));
    };
    Vec3 c = t < 0.5 ? mix(grass, earth, (t - 0.25) / 0.25) : mix(earth, rock, (t - 0.55) / 0.2);
    c = mix(c, rock, (steep - 0.25) / 0.25);
    return mix(c, snow, (t - 0.85) / 0.06);
}

/// A mesh with a colour per vertex, shaded on the CPU by a fixed sun (the
/// renderer has no lights). The triangles are the heightfield's own.
Model buildTerrainModel(const Heightfield& hf) {
    const int nx = hf.nx(), nz = hf.nz();
    // Smooth normals at the vertices, from central differences.
    auto vertexNormal = [&](int i, int k) {
        int i0 = std::max(i - 1, 0), i1 = std::min(i + 1, nx - 1);
        int k0 = std::max(k - 1, 0), k1 = std::min(k + 1, nz - 1);
        Real dx = (hf.heightAt(i1, k) - hf.heightAt(i0, k)) / ((i1 - i0) * hf.cellSize());
        Real dz = (hf.heightAt(i, k1) - hf.heightAt(i, k0)) / ((k1 - k0) * hf.cellSize());
        return normalize(Vec3{-dx, 1, -dz});
    };
    const Vec3 sun = normalize(Vec3{0.45, 1, 0.3});
    const Real range = std::max(hf.maxHeight() - hf.minHeight(), Real(1e-6));

    const int cells = (nx - 1) * (nz - 1);
    Mesh mesh{};
    mesh.vertexCount = cells * 6;
    mesh.triangleCount = cells * 2;
    mesh.vertices = static_cast<float*>(MemAlloc(unsigned(mesh.vertexCount * 3 * int(sizeof(float)))));
    mesh.normals = static_cast<float*>(MemAlloc(unsigned(mesh.vertexCount * 3 * int(sizeof(float)))));
    mesh.colors = static_cast<unsigned char*>(MemAlloc(unsigned(mesh.vertexCount) * 4));
    int v = 0;
    auto add = [&](int i, int k) {
        Vec3 p = hf.vertex(i, k), n = vertexNormal(i, k);
        Real shade = 0.5 + 0.5 * std::max(dot(n, sun), Real(0));
        Vec3 c = terrainColor((p.y - hf.minHeight()) / range, 1 - n.y) * shade;
        for (int a = 0; a < 3; ++a) {
            mesh.vertices[3 * v + a] = float(p[a]);
            mesh.normals[3 * v + a] = float(n[a]);
            mesh.colors[4 * v + a] = (unsigned char)std::clamp(c[a], Real(0), Real(255));
        }
        mesh.colors[4 * v + 3] = 255;
        ++v;
    };
    for (int k = 0; k + 1 < nz; ++k) {
        for (int i = 0; i + 1 < nx; ++i) {
            // The same two triangles as Heightfield::triangle(), counter-clockwise from above.
            for (auto [di, dk] : {std::pair{0, 0}, {0, 1}, {1, 1}, {0, 0}, {1, 1}, {1, 0}})
                add(i + di, k + dk);
        }
    }
    UploadMesh(&mesh, false);
    return LoadModelFromMesh(mesh);
}

void drawTerrain(const RigidBody& b) {
    const auto& terrain = b.shape().terrain;
    auto it = std::find_if(terrainMeshes.begin(), terrainMeshes.end(),
                           [&](const TerrainMesh& t) { return t.terrain == terrain; });
    if (it == terrainMeshes.end()) {
        terrainMeshes.push_back({terrain, buildTerrainModel(*terrain)});
        it = terrainMeshes.end() - 1;
    }
    DrawModel(it->model, {0, 0, 0}, 1, WHITE);
    // Faint grid lines every 5 cells, just above the surface.
    const Heightfield& hf = *terrain;
    const Color line{0, 0, 0, 28};
    auto lifted = [&](int i, int k) { return toRl(hf.vertex(i, k) + Vec3{0, 0.02, 0}); };
    for (int k = 0; k < hf.nz(); k += 5)
        for (int i = 0; i + 1 < hf.nx(); ++i)
            DrawLine3D(lifted(i, k), lifted(i + 1, k), line);
    for (int i = 0; i < hf.nx(); i += 5)
        for (int k = 0; k + 1 < hf.nz(); ++k)
            DrawLine3D(lifted(i, k), lifted(i, k + 1), line);
}
} // namespace

void releaseUnusedTerrainMeshes() {
    std::erase_if(terrainMeshes, [](TerrainMesh& t) {
        if (t.terrain.use_count() > 1) return false; // a body still has it
        UnloadModel(t.model);
        return true;
    });
}

void drawOutline(const RigidBody& b, Color color) {
    pushTransform(b);
    const Shape& s = b.shape();
    if (s.type == Shape::Type::Box) {
        ::Vector3 size = toRl(2 * s.halfExtents);
        for (float grow : {1.02f, 1.04f}) // two shells: a bolder line
            DrawCubeWiresV({0, 0, 0}, {size.x * grow, size.y * grow, size.z * grow}, color);
    } else if (s.type == Shape::Type::Sphere) {
        DrawSphereWires({0, 0, 0}, float(s.radius) * 1.04f, 10, 12, color);
    }
    rlPopMatrix();
}

void drawBody(const RigidBody& b, Color color) {
    pushTransform(b);
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
        // Infinite plane drawn as a 40 x 40 patch in the body's xz-plane.
        const float half = 20;
        DrawPlane({0, -0.001f, 0}, {2 * half, 2 * half}, color);
        for (float t = -half; t <= half; t += 1) {
            DrawLine3D({-half, 0, t}, {half, 0, t}, {0, 0, 0, 40});
            DrawLine3D({t, 0, -half}, {t, 0, half}, {0, 0, 0, 40});
        }
        break;
    }
    case Shape::Type::Heightfield: drawTerrain(b); break;
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
/// Contact forces on body a: normal (green) and friction (blue), as impulse /
/// step length, scaled to metres.
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
            if (forceScale > 0 && lengthSquared(p.tangentImpulse) > 0)
                drawArrow(p.position, p.tangentImpulse * (forceScale / h), {30, 90, 230, 255});
        }
    }
}

/// Joints: rods (dark grey); with details anchors (dark), lines from the
/// bodies' centres to them (grey), hinge axes (blue) and slider axes (purple).
void drawJoints(const World& world, bool details) {
    for (const auto& jp : world.joints()) {
        const Joint& j = *jp;
        Vec3 pa = j.anchorA(), pb = j.anchorB();
        if (dynamic_cast<const DistanceJoint*>(&j)) {
            DrawCylinderEx(toRl(pa), toRl(pb), 0.004f, 0.004f, 6, {70, 72, 80, 255});
            continue;
        }
        if (!details) continue;
        for (const RigidBody* b : {&j.bodyA(), &j.bodyB()})
            if (!b->isStatic()) DrawLine3D(toRl(b->x), toRl(b == &j.bodyA() ? pa : pb), {90, 90, 100, 255});
        DrawSphere(toRl(pa), 0.035f, {40, 40, 50, 255});
        if (auto* h = dynamic_cast<const HingeJoint*>(&j)) {
            Vec3 n = h->axis() * 0.3;
            DrawLine3D(toRl(pa - n), toRl(pa + n), {40, 90, 220, 255});
        } else if (auto* s = dynamic_cast<const SliderJoint*>(&j)) {
            Vec3 n = s->axis();
            Real lo = s->hasLimits() ? s->lowerLimit() : -1, hi = s->hasLimits() ? s->upperLimit() : 1;
            DrawLine3D(toRl(pa + n * lo), toRl(pa + n * hi), {150, 70, 200, 255});
        }
    }
}

void drawTyre(const RigidBody& wheel, const Vec3& axle, Real radius, Real width) {
    const Vec3 c = wheel.x, side = axle * (width / 2);
    const float r = float(radius);
    DrawCylinderEx(toRl(c - side), toRl(c + side), r, r, 28, {38, 38, 42, 255});
    // Rims on both faces, a little proud of the tyre.
    for (Real s : {-1.0, 1.0}) {
        Vec3 face = c + side * (s * 1.02);
        DrawCylinderEx(toRl(c + side * (s * 0.9)), toRl(face), r * 0.5f, r * 0.5f, 16, {150, 155, 165, 255});
    }
    // Tread lines across the tyre and spokes on the rims, turning with the wheel.
    // Their directions are fixed in the wheel's own frame (perpendicular to the
    // axle as the wheel sees it), so they turn with the wheel.
    const Mat3& R = wheel.R();
    const Vec3 la = transpose(R) * axle; // constant while the wheel spins about it
    const Vec3 lu = anyPerpendicular(la), lv = cross(la, lu);
    for (int k = 0; k < 12; ++k) {
        const Real a = k * 2 * M_PI / 12;
        Vec3 d = R * (lu * std::cos(a) + lv * std::sin(a));
        d = normalize(d - axle * dot(d, axle)); // keep it perpendicular to the axle
        Vec3 p = c + d * (radius * 1.01);
        DrawLine3D(toRl(p - side), toRl(p + side), {90, 90, 96, 255});
        if (k % 3 == 0)
            for (Real s : {-1.0, 1.0})
                DrawLine3D(toRl(c + side * (s * 1.03)), toRl(c + side * (s * 1.03) + d * (radius * 0.5)),
                           {60, 60, 66, 255});
    }
}

/// Springs as coils: 10 turns around the line between the anchors.
void drawSprings(const World& world) {
    for (const Spring& sp : world.springs()) {
        Vec3 pa = sp.anchorA(), pb = sp.anchorB(), d = pb - pa;
        Real len = length(d);
        if (len < 1e-6) continue;
        Vec3 n = d / len;
        Vec3 u = anyPerpendicular(n), v = cross(n, u);
        const int segments = 120;
        const Real radius = std::min(0.06, 0.12 * len); // short shocks get slim coils
        Vec3 prev = pa;
        for (int i = 1; i <= segments; ++i) {
            Real t = Real(i) / segments, phi = t * 10 * 2 * M_PI;
            Real r = (i < 4 || i > segments - 4) ? 0 : radius; // straight ends
            Vec3 p = pa + d * t + (u * std::cos(phi) + v * std::sin(phi)) * r;
            DrawLine3D(toRl(prev), toRl(p), {60, 60, 70, 255});
            prev = p;
        }
    }
}

std::uintptr_t packColor(int r, int g, int b) {
    return (std::uintptr_t(r) << 24) | (std::uintptr_t(g) << 16) | (std::uintptr_t(b) << 8) | 0xff;
}

Color colorOf(const RigidBody& b, Color fallback) {
    if (b.userData == 0) return fallback;
    auto c = std::uint32_t(b.userData);
    return {(unsigned char)(c >> 24), (unsigned char)(c >> 16), (unsigned char)(c >> 8), (unsigned char)c};
}

} // namespace ncotb::app
