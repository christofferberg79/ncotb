#include "collision/raycast.h"

#include "body/rigid_body.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace ncotb {

namespace {

std::optional<RayHit> raySphere(RigidBody& b, const Vec3& o, const Vec3& d) {
    const Real r = b.shape().radius;
    Vec3 m = o - b.x;
    Real c = dot(m, m) - r * r;
    if (c <= 0) return RayHit{&b, o, -d, 0}; // starts inside
    Real bb = dot(m, d);
    if (bb > 0) return std::nullopt; // outside and pointing away
    Real disc = bb * bb - c;
    if (disc < 0) return std::nullopt;
    Real t = -bb - std::sqrt(disc);
    Vec3 p = o + d * t;
    return RayHit{&b, p, (p - b.x) / r, t};
}

std::optional<RayHit> rayBox(RigidBody& b, const Vec3& o, const Vec3& d) {
    // Slab test in box coordinates: intersect the three pairs of parallel planes.
    const Vec3& h = b.shape().halfExtents;
    const Mat3 Rt = transpose(b.R());
    Vec3 lo = Rt * (o - b.x), ld = Rt * d;
    Real tEnter = -INFINITY, tExit = INFINITY;
    int axis = -1;
    Real sign = 0;
    for (int k = 0; k < 3; ++k) {
        if (std::abs(ld[k]) < 1e-12) {
            if (std::abs(lo[k]) > h[k]) return std::nullopt; // parallel and outside this slab
            continue;
        }
        Real t1 = (-h[k] - lo[k]) / ld[k], t2 = (h[k] - lo[k]) / ld[k];
        Real faceSign = -1; // t1 is the -h face
        if (t1 > t2) {
            std::swap(t1, t2);
            faceSign = 1;
        }
        if (t1 > tEnter) {
            tEnter = t1;
            axis = k;
            sign = faceSign;
        }
        tExit = std::min(tExit, t2);
        if (tEnter > tExit) return std::nullopt;
    }
    if (tExit < 0) return std::nullopt;           // the box is behind the ray
    if (tEnter <= 0) return RayHit{&b, o, -d, 0}; // starts inside
    Vec3 nLocal{};
    nLocal[axis] = sign;
    return RayHit{&b, o + d * tEnter, b.R() * nLocal, tEnter};
}

std::optional<RayHit> rayPlane(RigidBody& b, const Vec3& o, const Vec3& d) {
    const Vec3 n = b.R().column(1);
    Real height = dot(o - b.x, n);
    if (height <= 0) return RayHit{&b, o, -d, 0}; // starts below: inside the half-space
    Real denom = dot(n, d);
    if (denom >= 0) return std::nullopt; // parallel or pointing away
    Real t = -height / denom;
    return RayHit{&b, o + d * t, n, t};
}

/// Ray against one triangle, from its upper side only (Moller-Trumbore).
std::optional<Real> rayTriangle(const Vec3& o, const Vec3& d, const std::array<Vec3, 3>& t) {
    Vec3 e1 = t[1] - t[0], e2 = t[2] - t[0];
    Vec3 p = cross(d, e2);
    Real det = dot(e1, p);
    if (det < 1e-12) return std::nullopt; // parallel, or hitting the underside
    Real inv = 1 / det;
    Vec3 s = o - t[0];
    Real u = dot(s, p) * inv;
    if (u < 0 || u > 1) return std::nullopt;
    Vec3 q = cross(s, e1);
    Real v = dot(d, q) * inv;
    if (v < 0 || u + v > 1) return std::nullopt;
    Real dist = dot(e2, q) * inv;
    if (dist < 0) return std::nullopt;
    return dist;
}

std::optional<RayHit> rayHeightfield(RigidBody& b, const Vec3& origin, const Vec3& dir) {
    const Heightfield& hf = *b.shape().terrain;
    const Mat3 Rt = transpose(b.R());
    const Vec3 o = Rt * (origin - b.x), d = Rt * dir;
    // Where the ray is over the grid, between the lowest and highest point.
    const Vec3 lo{-hf.halfSizeX(), hf.minHeight(), -hf.halfSizeZ()};
    const Vec3 hi{hf.halfSizeX(), hf.maxHeight(), hf.halfSizeZ()};
    Real t0 = 0, t1 = INFINITY;
    for (int k = 0; k < 3; ++k) {
        if (std::abs(d[k]) < 1e-12) {
            if (o[k] < lo[k] || o[k] > hi[k]) return std::nullopt;
            continue;
        }
        Real a = (lo[k] - o[k]) / d[k], c = (hi[k] - o[k]) / d[k];
        if (a > c) std::swap(a, c);
        t0 = std::max(t0, a);
        t1 = std::min(t1, c);
        if (t0 > t1) return std::nullopt;
    }
    // Walk the cells under the ray in order (2D DDA) and test their triangles.
    const Real cs = hf.cellSize();
    const Vec3 start = o + d * t0;
    int i = std::clamp(int(std::floor((start.x + hf.halfSizeX()) / cs)), 0, hf.nx() - 2);
    int k = std::clamp(int(std::floor((start.z + hf.halfSizeZ()) / cs)), 0, hf.nz() - 2);
    const int stepI = d.x > 0 ? 1 : -1, stepK = d.z > 0 ? 1 : -1;
    // Ray parameter where it crosses the next cell boundary in x and in z.
    auto boundary = [&](int cell, int step, Real half, Real oc, Real dc) {
        if (std::abs(dc) < 1e-12) return Real(INFINITY);
        Real edge = (cell + (step > 0 ? 1 : 0)) * cs - half;
        return (edge - oc) / dc;
    };
    Real nextX = boundary(i, stepI, hf.halfSizeX(), o.x, d.x);
    Real nextZ = boundary(k, stepK, hf.halfSizeZ(), o.z, d.z);
    const Real dtX = std::abs(d.x) < 1e-12 ? INFINITY : cs / std::abs(d.x);
    const Real dtZ = std::abs(d.z) < 1e-12 ? INFINITY : cs / std::abs(d.z);
    while (true) {
        std::optional<Real> best;
        for (int half = 0; half < 2; ++half) {
            auto t = hf.triangle(i, k, half);
            if (auto hit = rayTriangle(o, d, t); hit && (!best || *hit < *best)) best = hit;
        }
        if (best) {
            Vec3 p = o + d * *best;
            int ci, ck, half;
            hf.locate(p.x, p.z, ci, ck, half);
            Vec3 n = triangleNormal(hf.triangle(ci, ck, half));
            return RayHit{&b, b.x + b.R() * p, b.R() * n, *best};
        }
        if (std::min(nextX, nextZ) > t1) return std::nullopt;
        if (nextX < nextZ) {
            i += stepI;
            nextX += dtX;
        } else {
            k += stepK;
            nextZ += dtZ;
        }
        if (i < 0 || i > hf.nx() - 2 || k < 0 || k > hf.nz() - 2) return std::nullopt;
    }
}

} // namespace

std::optional<RayHit> raycast(RigidBody& body, const Vec3& origin, const Vec3& dir) {
    switch (body.shape().type) {
    case Shape::Type::Sphere: return raySphere(body, origin, dir);
    case Shape::Type::Box: return rayBox(body, origin, dir);
    case Shape::Type::Plane: return rayPlane(body, origin, dir);
    case Shape::Type::Heightfield: return rayHeightfield(body, origin, dir);
    }
    return std::nullopt;
}

} // namespace ncotb
