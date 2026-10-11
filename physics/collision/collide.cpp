#include "collision/collide.h"

#include "body/rigid_body.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

namespace ncotb {

namespace {

constexpr Real kEpsilon = 1e-9;

using Points = std::vector<ContactPoint>;

/// Picks at most four points that keep the contact area as large as possible:
/// the deepest point, the point farthest from it, and the two points that
/// span the largest triangles on either side of the line between them.
void reduceToFour(Points& pts, const Vec3& n) {
    if (pts.size() <= ContactManifold::kMaxPoints) return;

    auto deepest = std::max_element(pts.begin(), pts.end(),
                                    [](const ContactPoint& p, const ContactPoint& q) { return p.depth < q.depth; });
    const Vec3 p0 = deepest->position;

    auto farthest = std::max_element(pts.begin(), pts.end(), [&](const ContactPoint& p, const ContactPoint& q) {
        return lengthSquared(p.position - p0) < lengthSquared(q.position - p0);
    });
    const Vec3 p1 = farthest->position;

    auto area = [&](const ContactPoint& p) { return dot(cross(p1 - p0, p.position - p0), n); };
    auto byArea = [&](const ContactPoint& p, const ContactPoint& q) { return area(p) < area(q); };
    auto most = std::max_element(pts.begin(), pts.end(), byArea);
    auto least = std::min_element(pts.begin(), pts.end(), byArea);

    Points kept{*deepest, *farthest};
    if (area(*most) > kEpsilon) kept.push_back(*most);
    if (area(*least) < -kEpsilon) kept.push_back(*least);
    pts = std::move(kept);
}

bool finish(ContactManifold& m, const Vec3& normal, Points& pts) {
    if (pts.empty()) return false;
    reduceToFour(pts, normal);
    m.normal = normal;
    m.count = int(pts.size());
    std::copy(pts.begin(), pts.end(), m.points);
    return true;
}

bool finish(ContactManifold& m, const Vec3& normal, const ContactPoint& p) {
    Points pts{p};
    return finish(m, normal, pts);
}

// ------------------------------------------------------------------ spheres

bool sphereSphere(const RigidBody& a, const RigidBody& b, ContactManifold& m, Real margin) {
    const Real ra = a.shape().radius, rb = b.shape().radius;
    Vec3 d = a.x - b.x;
    Real dist = length(d);
    Real depth = ra + rb - dist;
    if (-depth > margin) return false;
    Vec3 n = dist > kEpsilon ? d / dist : Vec3{0, 1, 0}; // concentric: any direction
    return finish(m, n, {b.x + n * (rb - depth / 2), depth});
}

bool spherePlane(const RigidBody& sphere, const RigidBody& plane, ContactManifold& m, Real margin) {
    const Vec3 n = plane.R().column(1);
    const Real r = sphere.shape().radius;
    Real dist = dot(sphere.x - plane.x, n); // centre height above the plane
    Real depth = r - dist;
    if (-depth > margin) return false;
    // midway between the sphere's lowest point and its projection on the plane
    return finish(m, n, {sphere.x - n * (dist + r) / 2, depth});
}

bool sphereBox(const RigidBody& sphere, const RigidBody& box, ContactManifold& m, Real margin) {
    const Vec3& h = box.shape().halfExtents;
    const Real r = sphere.shape().radius;
    const Mat3& R = box.R();

    // Work in box coordinates.
    Vec3 c = transpose(R) * (sphere.x - box.x);
    Vec3 closest{std::clamp(c.x, -h.x, h.x), std::clamp(c.y, -h.y, h.y), std::clamp(c.z, -h.z, h.z)};
    Vec3 d = c - closest;
    Real dist = length(d);

    Vec3 nLocal;
    Real depth;
    if (dist > kEpsilon) {
        // centre outside the box: normal from the closest point to the centre
        nLocal = d / dist;
        depth = r - dist;
    } else {
        // centre inside the box: push out through the nearest face
        int axis = 0;
        Real best = h.x - std::abs(c.x);
        for (int k = 1; k < 3; ++k) {
            Real gap = h[k] - std::abs(c[k]);
            if (gap < best) {
                best = gap;
                axis = k;
            }
        }
        nLocal = {};
        nLocal[axis] = c[axis] >= 0 ? 1 : -1;
        closest = c;
        closest[axis] = nLocal[axis] * h[axis];
        depth = r + best;
    }
    if (-depth > margin) return false;

    Vec3 n = R * nLocal;
    Vec3 onBox = box.x + R * closest;
    Vec3 onSphere = sphere.x - n * r;
    return finish(m, n, {(onBox + onSphere) / 2, depth});
}

// ------------------------------------------------------------------ boxes

/// World-space corners of a box.
std::array<Vec3, 8> boxCorners(const RigidBody& box) {
    std::array<Vec3, 8> v;
    const Vec3& h = box.shape().halfExtents;
    for (int i = 0; i < 8; ++i) {
        Vec3 local{(i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z};
        v[i] = box.x + box.R() * local;
    }
    return v;
}

bool boxPlane(const RigidBody& box, const RigidBody& plane, ContactManifold& m, Real margin) {
    const Vec3 n = plane.R().column(1);
    Points pts;
    for (const Vec3& v : boxCorners(box)) {
        Real dist = dot(v - plane.x, n);
        if (dist < margin) pts.push_back({v - n * (dist / 2), -dist});
    }
    return finish(m, n, pts);
}

/// Sutherland-Hodgman: keeps the part of the polygon where dot(p, normal) <= offset.
std::vector<Vec3> clip(const std::vector<Vec3>& poly, const Vec3& normal, Real offset) {
    std::vector<Vec3> out;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vec3& p = poly[i];
        const Vec3& q = poly[(i + 1) % poly.size()];
        Real dp = dot(p, normal) - offset, dq = dot(q, normal) - offset;
        if (dp <= 0) out.push_back(p);
        if ((dp < 0) != (dq < 0) && dp != dq) out.push_back(p + (q - p) * (dp / (dp - dq)));
    }
    return out;
}

/// Face contact: clip the incident box's face against the side planes of the
/// reference box's face whose outward normal is refNormal.
void faceContact(const RigidBody& ref, int refAxis, const Vec3& refNormal, const RigidBody& inc, Real margin,
                 Points& pts) {
    const Vec3& hr = ref.shape().halfExtents;
    const Vec3& hi = inc.shape().halfExtents;
    const Vec3 refCenter = ref.x + refNormal * hr[refAxis];

    // Incident face: the face of inc whose normal is most anti-parallel to refNormal.
    int incAxis = 0;
    Real best = -1;
    for (int k = 0; k < 3; ++k) {
        Real d = std::abs(dot(inc.R().column(k), refNormal));
        if (d > best) {
            best = d;
            incAxis = k;
        }
    }
    const Vec3 axis = inc.R().column(incAxis);
    const Real sign = dot(axis, refNormal) > 0 ? -1 : 1;
    const Vec3 incCenter = inc.x + axis * (sign * hi[incAxis]);
    const int u = (incAxis + 1) % 3, v = (incAxis + 2) % 3;
    const Vec3 du = inc.R().column(u) * hi[u], dv = inc.R().column(v) * hi[v];
    std::vector<Vec3> poly{incCenter + du + dv, incCenter - du + dv, incCenter - du - dv, incCenter + du - dv};

    // Clip against the four side planes of the reference face.
    for (int k = 0; k < 3 && !poly.empty(); ++k) {
        if (k == refAxis) continue;
        Vec3 side = ref.R().column(k);
        poly = clip(poly, side, dot(ref.x, side) + hr[k]);
        if (!poly.empty()) poly = clip(poly, -side, -dot(ref.x, side) + hr[k]);
    }

    for (const Vec3& p : poly) {
        Real dist = dot(p - refCenter, refNormal);
        if (dist < margin) pts.push_back({p - refNormal * (dist / 2), -dist});
    }
}

/// Closest points between segments p + s d (|s| <= hp) and q + t e (|t| <= hq).
std::pair<Vec3, Vec3> closestOnSegments(const Vec3& p, const Vec3& d, Real hp, const Vec3& q, const Vec3& e,
                                        Real hq) {
    Vec3 r = p - q;
    Real de = dot(d, e), dr = dot(d, r), er = dot(e, r);
    Real denom = 1 - de * de; // d and e are unit vectors
    Real s = denom > kEpsilon ? std::clamp((de * er - dr) / denom, -hp, hp) : 0;
    Real t = std::clamp(er + de * s, -hq, hq);
    s = std::clamp(de * t - dr, -hp, hp);
    return {p + d * s, q + e * t};
}

/// Box-box by the separating axis test over the 15 candidate axes, then face
/// clipping (face axes) or the closest points of the two edges (edge axes).
bool boxBox(const RigidBody& a, const RigidBody& b, ContactManifold& m, Real margin) {
    const Vec3& ha = a.shape().halfExtents;
    const Vec3& hb = b.shape().halfExtents;
    const Vec3 A[3] = {a.R().column(0), a.R().column(1), a.R().column(2)};
    const Vec3 B[3] = {b.R().column(0), b.R().column(1), b.R().column(2)};
    const Vec3 T = a.x - b.x;

    // Separation along unit axis L (negative when overlapping).
    auto separation = [&](const Vec3& L) {
        Real ra = 0, rb = 0;
        for (int k = 0; k < 3; ++k) {
            ra += ha[k] * std::abs(dot(A[k], L));
            rb += hb[k] * std::abs(dot(B[k], L));
        }
        return std::abs(dot(T, L)) - (ra + rb);
    };

    struct Axis {
        Real sep = -INFINITY;
        Vec3 L{};
        int i = -1, j = -1; // face of a (i), face of b (j), or edge pair (i, j)
    };
    Axis faceA, faceB, edge;
    for (int k = 0; k < 3; ++k) {
        if (Real s = separation(A[k]); s > faceA.sep) faceA = {s, A[k], k, -1};
        if (Real s = separation(B[k]); s > faceB.sep) faceB = {s, B[k], -1, k};
    }
    if (faceA.sep > margin || faceB.sep > margin) return false;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3 L = cross(A[i], B[j]);
            Real len = length(L);
            if (len < 1e-6) continue; // parallel edges: covered by the face axes
            L /= len;
            Real s = separation(L);
            if (s > margin) return false;
            if (s > edge.sep) edge = {s, L, i, j};
        }
    }

    // Prefer face contacts (and faces of b) unless another axis is clearly
    // better, so the reference face doesn't flip back and forth.
    auto clearlyBetter = [](Real s, Real than) { return s > 0.95 * than + 0.005; };
    const Axis& face = clearlyBetter(faceA.sep, faceB.sep) ? faceA : faceB;
    const bool useEdge = edge.i >= 0 && clearlyBetter(edge.sep, face.sep);
    const Axis& best = useEdge ? edge : face;
    const Vec3 n = dot(T, best.L) >= 0 ? best.L : -best.L; // from b towards a

    Points pts;
    if (!useEdge) {
        if (best.j >= 0) faceContact(b, best.j, n, a, margin, pts);  // b's face, normal towards a
        else faceContact(a, best.i, -n, b, margin, pts);             // a's face, normal towards b
        return finish(m, n, pts);
    }

    // Edge-edge: the edge of a nearest to b and the edge of b nearest to a.
    Vec3 pa = a.x, pb = b.x;
    for (int k = 0; k < 3; ++k) {
        if (k != edge.i) pa -= A[k] * (ha[k] * (dot(A[k], n) > 0 ? 1 : -1));
        if (k != edge.j) pb += B[k] * (hb[k] * (dot(B[k], n) > 0 ? 1 : -1));
    }
    auto [ca, cb] = closestOnSegments(pa, A[edge.i], ha[edge.i], pb, B[edge.j], hb[edge.j]);
    return finish(m, n, {(ca + cb) / 2, -edge.sep});
}

int rank(Shape::Type t) {
    switch (t) {
    case Shape::Type::Sphere: return 0;
    case Shape::Type::Box: return 1;
    case Shape::Type::Plane: return 2;
    }
    return 3;
}

} // namespace

bool collide(RigidBody& a, RigidBody& b, ContactManifold& m, Real margin) {
    // Order the pair so that the "simpler" shape comes first; flip the normal back afterwards.
    const bool swapped = rank(a.shape().type) > rank(b.shape().type);
    const RigidBody& s = swapped ? b : a;
    const RigidBody& t = swapped ? a : b;

    m = ContactManifold{};
    bool hit = false;
    using T = Shape::Type;
    switch (s.shape().type) {
    case T::Sphere:
        if (t.shape().type == T::Sphere) hit = sphereSphere(s, t, m, margin);
        else if (t.shape().type == T::Box) hit = sphereBox(s, t, m, margin);
        else hit = spherePlane(s, t, m, margin);
        break;
    case T::Box:
        if (t.shape().type == T::Box) hit = boxBox(s, t, m, margin);
        else hit = boxPlane(s, t, m, margin);
        break;
    case T::Plane: break; // plane-plane
    }
    if (!hit) return false;

    m.a = &a;
    m.b = &b;
    if (swapped) m.normal = -m.normal;
    return true;
}

} // namespace ncotb
