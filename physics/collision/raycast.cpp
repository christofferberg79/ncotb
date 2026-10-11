#include "collision/raycast.h"

#include "body/rigid_body.h"

#include <algorithm>
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

} // namespace

std::optional<RayHit> raycast(RigidBody& body, const Vec3& origin, const Vec3& dir) {
    switch (body.shape().type) {
    case Shape::Type::Sphere: return raySphere(body, origin, dir);
    case Shape::Type::Box: return rayBox(body, origin, dir);
    case Shape::Type::Plane: return rayPlane(body, origin, dir);
    }
    return std::nullopt;
}

} // namespace ncotb
