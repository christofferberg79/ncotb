#include "solver/contact_solver.h"

#include "body/rigid_body.h"
#include "solver/lcp.h"

#include <algorithm>
#include <limits>

namespace ncotb {

namespace {
constexpr Real kNoBounce = -std::numeric_limits<Real>::infinity();

Real effectiveMassInverse(const RigidBody& a, const RigidBody& b, const Vec3& ra, const Vec3& rb,
                          const Vec3& n) {
    Vec3 raxn = cross(ra, n), rbxn = cross(rb, n);
    Real K = a.invMass() + b.invMass() + dot(raxn, a.Iinv() * raxn) + dot(rbxn, b.Iinv() * rbxn);
    return K > 0 ? 1 / K : 0;
}
} // namespace

const char* name(ContactSolverType type) {
    switch (type) {
    case ContactSolverType::SequentialImpulses: return "Sequential impulses";
    case ContactSolverType::BaraffLcp: return "Baraff LCP (pivoting)";
    }
    return "?";
}

Real ContactSolver::normalVelocity(const Row& r) const {
    Vec3 va = r.a->v() + cross(r.a->omega(), r.ra);
    Vec3 vb = r.b->v() + cross(r.b->omega(), r.rb);
    return dot(r.n, va - vb);
}

void ContactSolver::applyRowImpulse(const Row& r, Real dLambda) const {
    r.a->applyImpulseAtOffset(r.n * dLambda, r.ra);
    r.b->applyImpulseAtOffset(-r.n * dLambda, r.rb);
}

void ContactSolver::prepare(std::vector<ContactManifold>& contacts, Real h) {
    rows_.clear();
    stats_ = {};
    h_ = h;
    for (ContactManifold& m : contacts) {
        RigidBody& a = *m.a;
        RigidBody& b = *m.b;
        const Real e = std::max(a.restitution, b.restitution);
        for (int i = 0; i < m.count; ++i) {
            ContactPoint& cp = m.points[i];
            Row r{};
            r.a = &a;
            r.b = &b;
            r.n = m.normal;
            r.ra = r.ra0 = cp.position - a.x;
            r.rb = r.rb0 = cp.position - b.x;
            r.xa0 = a.x;
            r.xb0 = b.x;
            r.qa0 = a.q;
            r.qb0 = b.q;
            r.depth0 = cp.depth;
            r.invK = effectiveMassInverse(a, b, r.ra, r.rb, r.n);
            // Bounce on the impact velocity, before this step's forces (gravity)
            // were added; otherwise the bounce gains g h in speed every time.
            Real vn =
                dot(r.n, a.pointVelocityAtStepStart(cp.position) - b.pointVelocityAtStepStart(cp.position));
            r.restitutionTarget = vn < -settings.restitutionThreshold ? -e * vn : kNoBounce;
            r.impulse = &cp.normalImpulse;
            rows_.push_back(r);
        }
    }
    stats_.rows = int(rows_.size());
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
    if (settings.type == ContactSolverType::SequentialImpulses) {
        for (const Row& r : rows_) {
            if (!settings.warmStart) *r.impulse = 0;
            applyRowImpulse(r, *r.impulse);
        }
        sequentialImpulses();
    } else {
        for (const Row& r : rows_)
            *r.impulse = 0; // the LCP gives the whole impulse
        baraffLcp(true);
    }
}

Real ContactSolver::updateAnchors(Row& r) const {
    r.ra = rotate(r.a->q * conjugate(r.qa0), r.ra0);
    r.rb = rotate(r.b->q * conjugate(r.qb0), r.rb0);
    Vec3 movedA = (r.a->x + r.ra) - (r.xa0 + r.ra0);
    Vec3 movedB = (r.b->x + r.rb) - (r.xb0 + r.rb0);
    r.invK = effectiveMassInverse(*r.a, *r.b, r.ra, r.rb, r.n);
    return r.depth0 - dot(r.n, movedA - movedB);
}

void ContactSolver::relax() {
    if (rows_.empty()) return;

    if (settings.relax) {
        // Same constraints at the new positions, without the push-out: v_n >= 0
        // for touching or overlapping contacts, >= gap / h for those still apart.
        for (Row& r : rows_) {
            Real depth = updateAnchors(r);
            r.target = depth < 0 ? depth / h_ : 0;
        }
        if (settings.type == ContactSolverType::SequentialImpulses) {
            sequentialImpulses(); // clamping the accumulated impulse lets it shrink again
        } else {
            for (const Row& r : rows_) {
                applyRowImpulse(r, -*r.impulse); // undo, then solve afresh
                *r.impulse = 0;
            }
            baraffLcp(true);
        }
    }

    // Restitution: rows that were hit (impulse > 0) while approaching fast get
    // the bounce target. Doing it after the main solve means speculative
    // contacts, which stop a body just before it touches, still bounce.
    bool bounce = false;
    for (Row& r : rows_) {
        if (r.restitutionTarget != kNoBounce && *r.impulse > 0) {
            r.target = std::max(r.target, r.restitutionTarget);
            bounce = true;
        }
    }
    if (!bounce) return;
    if (settings.type == ContactSolverType::SequentialImpulses)
        sequentialImpulses();
    else
        baraffLcp(false);
}

/// Projected Gauss-Seidel: solve one row at a time with the others fixed,
/// clamping the accumulated impulse (not the increment) to >= 0.
void ContactSolver::sequentialImpulses() {
    for (int it = 0; it < settings.iterations; ++it) {
        for (const Row& r : rows_) {
            Real dLambda = (r.target - normalVelocity(r)) * r.invK;
            Real old = *r.impulse;
            *r.impulse = std::max(old + dLambda, Real(0));
            applyRowImpulse(r, *r.impulse - old);
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
        applyRowImpulse(r, x[size_t(i)]);
    }
}

} // namespace ncotb
