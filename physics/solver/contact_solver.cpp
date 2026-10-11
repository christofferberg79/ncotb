#include "solver/contact_solver.h"

#include "body/rigid_body.h"
#include "solver/lcp.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ncotb {

namespace {
constexpr Real kNoBounce = -std::numeric_limits<Real>::infinity();

/// 1 / (J M^-1 J^T) for an impulse along unit direction d at offsets ra, rb.
Real effectiveMassInverse(const RigidBody& a, const RigidBody& b, const Vec3& ra, const Vec3& rb,
                          const Vec3& d) {
    Vec3 raxd = cross(ra, d), rbxd = cross(rb, d);
    Real K = a.invMass() + b.invMass() + dot(raxd, a.Iinv() * raxd) + dot(rbxd, b.Iinv() * rbxd);
    return K > 0 ? 1 / K : 0;
}

/// A tangent perpendicular to n that depends only on n, so it is the same every
/// step for a resting contact.
Vec3 tangentOf(const Vec3& n) {
    Vec3 axis = std::abs(n.x) < 0.57 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    return normalize(cross(n, axis));
}
} // namespace

const char* name(ContactSolverType type) {
    switch (type) {
    case ContactSolverType::SequentialImpulses: return "Sequential impulses";
    case ContactSolverType::BaraffLcp: return "Baraff LCP (pivoting)";
    }
    return "?";
}

Vec3 ContactSolver::relativeVelocity(const Row& r) const {
    Vec3 va = r.a->v() + cross(r.a->omega(), r.ra);
    Vec3 vb = r.b->v() + cross(r.b->omega(), r.rb);
    return va - vb;
}

void ContactSolver::applyRowImpulse(const Row& r, const Vec3& J) const {
    r.a->applyImpulseAtOffset(J, r.ra);
    r.b->applyImpulseAtOffset(-J, r.rb);
}

bool ContactSolver::useLcp() const {
    return settings.type == ContactSolverType::BaraffLcp && !stats_.lcpTooLarge;
}

void ContactSolver::prepare(std::vector<ContactManifold>& contacts, const std::vector<Joint*>& joints, Real h) {
    rows_.clear();
    stats_ = {};
    h_ = h;
    joints_ = joints;
    const JointSettings js{settings.baumgarte, settings.jointMaxCorrection};
    for (Joint* j : joints_)
        j->prepare(h, js);
    for (ContactManifold& m : contacts) {
        RigidBody& a = *m.a;
        RigidBody& b = *m.b;
        const Real e = std::max(a.restitution, b.restitution);
        const Real mu = settings.friction ? std::sqrt(a.friction * b.friction) : 0;
        for (int i = 0; i < m.count; ++i) {
            ContactPoint& cp = m.points[i];
            Row r{};
            r.a = &a;
            r.b = &b;
            r.n = m.normal;
            r.t1 = tangentOf(r.n);
            r.t2 = cross(r.n, r.t1);
            r.mu = mu;
            r.ra = r.ra0 = cp.position - a.x;
            r.rb = r.rb0 = cp.position - b.x;
            r.xa0 = a.x;
            r.xb0 = b.x;
            r.qa0 = a.q;
            r.qb0 = b.q;
            r.depth0 = cp.depth;
            r.invK = effectiveMassInverse(a, b, r.ra, r.rb, r.n);
            r.invKt1 = effectiveMassInverse(a, b, r.ra, r.rb, r.t1);
            r.invKt2 = effectiveMassInverse(a, b, r.ra, r.rb, r.t2);
            // Warm start friction from last step's impulse, projected on the
            // (possibly slightly different) tangents.
            r.lt1 = dot(cp.tangentImpulse, r.t1);
            r.lt2 = dot(cp.tangentImpulse, r.t2);
            // Bounce on the impact velocity, before this step's forces (gravity)
            // were added; otherwise the bounce gains g h in speed every time.
            Real vn =
                dot(r.n, a.pointVelocityAtStepStart(cp.position) - b.pointVelocityAtStepStart(cp.position));
            r.restitutionTarget = vn < -settings.restitutionThreshold ? -e * vn : kNoBounce;
            r.impulse = &cp.normalImpulse;
            r.tangentImpulse = &cp.tangentImpulse;
            rows_.push_back(r);
        }
    }
    stats_.rows = int(rows_.size());
    stats_.lcpTooLarge = settings.type == ContactSolverType::BaraffLcp && stats_.rows > settings.lcpMaxRows;
}

void ContactSolver::solveVelocities() {
    for (Row& r : rows_) {
        if (r.depth0 < 0) {
            r.target = r.depth0 / h_; // still apart: may close the gap, not pass it
        } else {
            Real push = settings.baumgarte / h_ * std::max(r.depth0 - settings.slop, Real(0));
            r.target = std::min(push, settings.maxPushVelocity);
        }
    }
    for (Joint* j : joints_)
        j->warmStart();
    if (!useLcp()) {
        for (Row& r : rows_) {
            if (!settings.warmStart) {
                *r.impulse = 0;
                r.lt1 = r.lt2 = 0;
            }
            applyRowImpulse(r, r.n * *r.impulse + r.t1 * r.lt1 + r.t2 * r.lt2);
        }
        sequentialImpulses(true, true, true);
    } else {
        for (Row& r : rows_) {
            *r.impulse = 0; // the LCP gives the whole impulse
            r.lt1 = r.lt2 = 0;
        }
        baraffWithFriction(true);
    }
    storeFriction();
}

Real ContactSolver::updateAnchors(Row& r) const {
    // How far the two contact points moved, with the offsets turned along with
    // the bodies. The lever arms ra, rb themselves stay as they were: a sphere's
    // contact point stays at the bottom while the sphere turns, and a rotated
    // lever arm would make the normal impulse brake the rolling.
    Vec3 movedA = (r.a->x + rotate(r.a->q * conjugate(r.qa0), r.ra0)) - (r.xa0 + r.ra0);
    Vec3 movedB = (r.b->x + rotate(r.b->q * conjugate(r.qb0), r.rb0)) - (r.xb0 + r.rb0);
    r.invK = effectiveMassInverse(*r.a, *r.b, r.ra, r.rb, r.n); // I^-1 has turned
    r.invKt1 = effectiveMassInverse(*r.a, *r.b, r.ra, r.rb, r.t1);
    r.invKt2 = effectiveMassInverse(*r.a, *r.b, r.ra, r.rb, r.t2);
    return r.depth0 - dot(r.n, movedA - movedB);
}

void ContactSolver::relax() {
    if (rows_.empty() && joints_.empty()) return;

    if (settings.relax) {
        // Same constraints at the new positions, without the push-out: v_n >= 0
        // for touching or overlapping contacts, >= gap / h for those still apart.
        for (Row& r : rows_) {
            Real depth = updateAnchors(r);
            r.target = depth < 0 ? depth / h_ : 0;
        }
        if (!useLcp()) {
            sequentialImpulses(true, true, false); // clamping the accumulated impulses lets them shrink again
        } else {
            for (Row& r : rows_) {
                applyRowImpulse(r,
                                -(r.n * *r.impulse + r.t1 * r.lt1 + r.t2 * r.lt2)); // undo, then solve afresh
                *r.impulse = 0;
                r.lt1 = r.lt2 = 0;
            }
            baraffWithFriction(false);
        }
    }

    // Restitution: rows that were hit (impulse > 0) while approaching fast get
    // the bounce target. Doing it after the main solve means speculative
    // contacts, which stop a body just before it touches, still bounce.
    // Friction is left as it is.
    bool bounce = false;
    for (Row& r : rows_) {
        if (r.restitutionTarget != kNoBounce && *r.impulse > 0) {
            r.target = std::max(r.target, r.restitutionTarget);
            bounce = true;
        }
    }
    if (bounce) {
        if (!useLcp())
            sequentialImpulses(false, false, false);
        else
            baraffLcp(false);
    }
    storeFriction();
}

/// Friction for one point: drive the sliding velocity to zero, keeping the
/// accumulated friction impulse inside the disc |f| <= mu * normal impulse.
void ContactSolver::solveFriction(Row& r) const {
    if (r.mu <= 0) return;
    Vec3 v = relativeVelocity(r);
    Real n1 = r.lt1 - dot(r.t1, v) * r.invKt1;
    Real n2 = r.lt2 - dot(r.t2, v) * r.invKt2;
    Real limit = r.mu * *r.impulse;
    Real len = std::hypot(n1, n2);
    if (len > limit) {
        Real k = len > 0 ? limit / len : 0;
        n1 *= k;
        n2 *= k;
    }
    applyRowImpulse(r, r.t1 * (n1 - r.lt1) + r.t2 * (n2 - r.lt2));
    r.lt1 = n1;
    r.lt2 = n2;
}

/// Final clamp onto the Coulomb cone (the normal impulse may have shrunk after
/// friction was last solved), then store the friction impulse.
void ContactSolver::storeFriction() {
    for (Row& r : rows_) {
        Real limit = r.mu * *r.impulse;
        Real len = std::hypot(r.lt1, r.lt2);
        if (len > limit) {
            Real k = len > 0 ? limit / len : 0;
            applyRowImpulse(r, r.t1 * (r.lt1 * k - r.lt1) + r.t2 * (r.lt2 * k - r.lt2));
            r.lt1 *= k;
            r.lt2 *= k;
        }
        *r.tangentImpulse = r.t1 * r.lt1 + r.t2 * r.lt2;
    }
}

/// Projected Gauss-Seidel: solve one row at a time with the others fixed,
/// clamping the accumulated impulse (not the increment). Friction first: the
/// normal rows come last so non-penetration is what holds best.
void ContactSolver::sequentialImpulses(bool withFriction, bool withJoints, bool useBias) {
    for (int it = 0; it < settings.iterations; ++it) {
        if (withJoints)
            for (Joint* j : joints_)
                j->solve(useBias);
        for (Row& r : rows_) {
            if (withFriction) solveFriction(r);
            Real dLambda = (r.target - normalVelocity(r)) * r.invK;
            Real old = *r.impulse;
            *r.impulse = std::max(old + dLambda, Real(0));
            applyRowImpulse(r, r.n * (*r.impulse - old));
        }
    }
}

/// Staggered projections: exact LCP for the normal impulses with friction and
/// joints held fixed, then friction and joint passes with the normal impulses
/// held fixed, repeated. (Joints are equalities, which the pivoting method for
/// inequalities doesn't handle, so they are solved like friction.)
void ContactSolver::baraffWithFriction(bool useBias) {
    const bool friction = settings.friction && !rows_.empty();
    const bool joints = !joints_.empty();
    const int rounds = friction || joints ? std::max(settings.staggers, 1) : 1;
    for (int k = 0; k < rounds; ++k) {
        if (!rows_.empty()) {
            for (Row& r : rows_) { // replace the normal impulses, keep friction
                applyRowImpulse(r, r.n * -*r.impulse);
                *r.impulse = 0;
            }
            baraffLcp(true);
        }
        if (!friction && !joints) break;
        for (int it = 0; it < settings.iterations; ++it) {
            for (Joint* jt : joints_)
                jt->solve(useBias);
            if (friction)
                for (Row& r : rows_)
                    solveFriction(r);
        }
    }
}

/// Builds A = J M^-1 J^T and b = v_n - target from the current velocities and
/// solves the LCP exactly. replace: the result is the whole impulse; otherwise
/// it is added on top of the impulses already applied.
void ContactSolver::baraffLcp(bool replace) {
    const int n = int(rows_.size());
    DenseMatrix A(n);
    std::vector<Real> b(static_cast<size_t>(n)), x;

    // Rows i and j are coupled through every body they share.
    auto coupling = [](const RigidBody* body, Real si, const Vec3& ni, const Vec3& ri, Real sj,
                       const Vec3& nj, const Vec3& rj) {
        return si * sj * (body->invMass() * dot(ni, nj) + dot(cross(ri, ni), body->Iinv() * cross(rj, nj)));
    };
    Real trace = 0;
    for (int i = 0; i < n; ++i) {
        const Row& ri = rows_[size_t(i)];
        for (int j = 0; j < n; ++j) {
            const Row& rj = rows_[size_t(j)];
            Real sum = 0;
            for (int u = 0; u < 2; ++u) {
                const RigidBody* bi = u == 0 ? ri.a : ri.b;
                if (bi->isStatic()) continue;
                Real si = u == 0 ? 1 : -1;
                const Vec3& rOnI = u == 0 ? ri.ra : ri.rb;
                if (bi == rj.a) sum += coupling(bi, si, ri.n, rOnI, 1, rj.n, rj.ra);
                if (bi == rj.b) sum += coupling(bi, si, ri.n, rOnI, -1, rj.n, rj.rb);
            }
            A(i, j) = sum;
        }
        trace += A(i, i);
        b[size_t(i)] = normalVelocity(ri) - ri.target;
    }
    // A tiny regularization makes A positive definite even with redundant
    // contacts (4 coplanar points on a box face give a singular A).
    const Real eps = 1e-9 * trace / n;
    for (int i = 0; i < n; ++i)
        A(i, i) += eps;

    LcpResult res = solveLcpBaraff(A, b, x);
    stats_.pivots += res.pivots;
    stats_.lcpFailed |= !res.solved;
    for (int i = 0; i < n; ++i) {
        const Row& r = rows_[size_t(i)];
        if (replace) *r.impulse = 0;
        *r.impulse += x[size_t(i)];
        applyRowImpulse(r, r.n * x[size_t(i)]);
    }
}

} // namespace ncotb
