// Contacts of spheres and boxes with heightfield terrain. Everything is worked
// out in the terrain's frame and turned into world space at the end.

#include "body/rigid_body.h"
#include "collision/collide_internal.h"

#include <algorithm>
#include <cmath>

namespace ncotb::detail {

namespace {

using Triangle = std::array<Vec3, 3>;

/// Closest point on triangle t to p (Ericson, Real-Time Collision Detection 5.1.5).
/// interior is set when it lies inside the face, not on an edge or corner.
Vec3 closestOnTriangle(const Vec3& p, const Triangle& t, bool& interior) {
    const Vec3 &a = t[0], &b = t[1], &c = t[2];
    interior = false;
    Vec3 ab = b - a, ac = c - a, ap = p - a;
    Real d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    Vec3 bp = p - b;
    Real d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    Real vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    Vec3 cp = p - c;
    Real d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    Real vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    Real va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    interior = true;
    Real denom = 1 / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

/// A contact point with its own normal, in the terrain's frame.
struct Candidate {
    Vec3 n;
    Vec3 position;
    Real depth;
};

/// Groups candidates with (nearly) the same normal into manifolds of at most
/// four points, and turns them into world space.
void emit(const std::vector<Candidate>& cands, RigidBody& body, RigidBody& terrain,
          std::vector<ContactManifold>& out) {
    const Mat3& R = terrain.R();
    std::vector<bool> used(cands.size(), false);
    std::vector<ContactPoint> pts;
    for (size_t i = 0; i < cands.size(); ++i) {
        if (used[i]) continue;
        const Vec3 n = cands[i].n;
        pts.clear();
        for (size_t j = i; j < cands.size(); ++j) {
            if (used[j] || dot(cands[j].n, n) < 0.9995) continue; // within about 1.8 degrees
            used[j] = true;
            pts.push_back({terrain.x + R * cands[j].position, cands[j].depth});
        }
        const Vec3 nWorld = R * n;
        reduceToFour(pts, nWorld);
        ContactManifold m;
        m.a = &body;
        m.b = &terrain;
        m.normal = nWorld;
        m.count = int(pts.size());
        std::copy(pts.begin(), pts.end(), m.points);
        out.push_back(m);
    }
}

} // namespace

void sphereTerrain(RigidBody& sphere, RigidBody& terrain, std::vector<ContactManifold>& out, Real margin) {
    const Heightfield& hf = *terrain.shape().terrain;
    const Real r = sphere.shape().radius;
    const Vec3 c = transpose(terrain.R()) * (sphere.x - terrain.x);
    const Real reach = r + margin;
    if (c.y - reach > hf.maxHeight()) return;
    int i0, k0, i1, k1;
    if (!hf.cellRange(c.x - reach, c.z - reach, c.x + reach, c.z + reach, i0, k0, i1, k1)) return;

    std::vector<Candidate> cands;
    if (hf.contains(c.x, c.z) && c.y < hf.height(c.x, c.z)) {
        // The centre is under the surface (fell in, or a huge step): push it
        // straight out of the triangle it is under.
        Vec3 n = hf.normal(c.x, c.z);
        Real depth = r + (hf.height(c.x, c.z) - c.y) * n.y;
        cands.push_back({n, c - n * (r - depth / 2), depth});
        emit(cands, sphere, terrain, out);
        return;
    }

    // The closest point on every nearby triangle.
    struct Near {
        Triangle t;
        Vec3 q;
        Real dist2;
        bool interior;
    };
    std::vector<Near> near;
    for (int k = k0; k <= k1; ++k) {
        for (int i = i0; i <= i1; ++i) {
            for (int half = 0; half < 2; ++half) {
                Near n{hf.triangle(i, k, half), {}, 0, false};
                n.q = closestOnTriangle(c, n.t, n.interior);
                n.dist2 = lengthSquared(c - n.q);
                if (n.dist2 <= reach * reach) near.push_back(n);
            }
        }
    }

    for (size_t a = 0; a < near.size(); ++a) {
        const Near& t = near[a];
        // Internal edges: an edge or corner point only counts if no triangle that
        // shares it comes closer. Otherwise a ball rolling over flat terrain would
        // catch on the edges between its triangles.
        bool keep = true;
        for (size_t b = 0; b < near.size() && keep; ++b) {
            if (b == a) continue;
            bool unused;
            bool shares = lengthSquared(closestOnTriangle(t.q, near[b].t, unused) - t.q) < 1e-18;
            if (!t.interior && shares && near[b].dist2 < t.dist2 - 1e-12) keep = false;
            // The same point found from two triangles (a ridge, a shared corner): once.
            if (b < a && lengthSquared(near[b].q - t.q) < 1e-18) keep = false;
        }
        if (!keep) continue;
        Real dist = std::sqrt(t.dist2);
        Vec3 n = dist > 1e-9 ? (c - t.q) / dist : triangleNormal(t.t);
        Real depth = r - dist;
        cands.push_back({n, (t.q + c - n * r) / 2, depth});
    }
    emit(cands, sphere, terrain, out);
}

void boxTerrain(RigidBody& box, RigidBody& terrain, std::vector<ContactManifold>& out, Real margin) {
    const Heightfield& hf = *terrain.shape().terrain;
    const Mat3 Rt = transpose(terrain.R());
    const Vec3 centre = Rt * (box.x - terrain.x);
    const Mat3 Rb = Rt * box.R(); // box axes in the terrain's frame
    const Vec3& he = box.shape().halfExtents;

    std::vector<Candidate> cands;
    Vec3 lo{INFINITY, INFINITY, INFINITY}, hi = -lo;
    for (const Vec3& world : boxCorners(box)) {
        Vec3 p = Rt * (world - terrain.x);
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        // Corners: the gap to the plane of the triangle below.
        if (!hf.contains(p.x, p.z)) continue;
        int i, k, half;
        hf.locate(p.x, p.z, i, k, half);
        Triangle t = hf.triangle(i, k, half);
        Vec3 n = triangleNormal(t);
        Real dist = dot(p - t[0], n);
        if (dist < margin) cands.push_back({n, p - n * (dist / 2), -dist});
    }
    if (lo.y - margin > hf.maxHeight()) return;

    // Terrain vertices poking up into the box's bottom (a crate on a sharp
    // peak): push the box up off them. Only the face that looks down the most
    // counts; a vertex level with the bottom just ahead of a sliding box must
    // not push against its front face.
    int axis = 0;
    for (int a = 1; a < 3; ++a)
        if (std::abs(Rb.m[1][a]) > std::abs(Rb.m[1][axis])) axis = a; // box axis closest to vertical
    Vec3 face{};
    face[axis] = Rb.m[1][axis] > 0 ? -1 : 1; // the bottom face, in box coordinates
    const Vec3 f = Rb * face;                // its outward normal (downwards)
    const int u = (axis + 1) % 3, w = (axis + 2) % 3;
    const Real cs = hf.cellSize();
    const int vi0 = std::max(0, int(std::ceil((lo.x + hf.halfSizeX()) / cs)));
    const int vi1 = std::min(hf.nx() - 1, int(std::floor((hi.x + hf.halfSizeX()) / cs)));
    const int vk0 = std::max(0, int(std::ceil((lo.z + hf.halfSizeZ()) / cs)));
    const int vk1 = std::min(hf.nz() - 1, int(std::floor((hi.z + hf.halfSizeZ()) / cs)));
    for (int k = vk0; k <= vk1; ++k) {
        for (int i = vi0; i <= vi1; ++i) {
            const Vec3 v = hf.vertex(i, k);
            if (v.y < lo.y - margin) continue;
            const Vec3 pb = transpose(Rb) * (v - centre);                     // in box coordinates
            if (std::abs(pb[u]) > he[u] || std::abs(pb[w]) > he[w]) continue; // not under the bottom face
            const Real depth = he[axis] + pb[axis] * face[axis] * -1; // height of v above the bottom face
            if (depth < -margin || depth > 2 * he[axis]) continue;
            cands.push_back({-f, v + f * (depth / 2), depth});
        }
    }
    emit(cands, box, terrain, out);
}

} // namespace ncotb::detail
