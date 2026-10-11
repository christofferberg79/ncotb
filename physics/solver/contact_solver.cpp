#include "solver/contact_solver.h"

#include "body/rigid_body.h"
#include "solver/lcp.h"

#include <algorithm>
#include <limits>

namespace ncotb {

const char* name(ContactSolverType type) {
    switch (type) {
    case ContactSolverType::SequentialImpulses: return "Sequential impulses";
    case ContactSolverType::BaraffLcp: return "Baraff LCP (pivoting)";
    }
    return "?";
}

Real ContactSolver::normalVelocity(const Row& r) const {
    return dot(r.n, r.a->pointVelocity(r.p) - r.b->pointVelocity(r.p));
}

void ContactSolver::applyRowImpulse(const Row& r, Real dLambda) const {
    r.a->applyImpulse(r.n * dLambda, r.p);
    r.b->applyImpulse(-r.n * dLambda, r.p);
}

void ContactSolver::solve(std::vector<ContactManifold>& contacts, Real h) {
    rows_.clear();
    stats_ = {};
    const Real noBounce = -std::numeric_limits<Real>::infinity();

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
            r.p = cp.position;
            r.raxn = cross(cp.position - a.x, m.normal);
            r.rbxn = cross(cp.position - b.x, m.normal);
            Real K =
                a.invMass() + b.invMass() + dot(r.raxn, a.Iinv() * r.raxn) + dot(r.rbxn, b.Iinv() * r.rbxn);
            r.invK = K > 0 ? 1 / K : 0;
            r.bias = cp.depth < 0 ? cp.depth / h // still apart: may approach by the gap
                                  : settings.baumgarte / h * std::max(cp.depth - settings.slop, Real(0));
            // Bounce on the impact velocity, before this step's forces (gravity)
            // were added; otherwise the bounce gains g h in speed every time.
            Real vn = dot(r.n, a.pointVelocityAtStepStart(r.p) - b.pointVelocityAtStepStart(r.p));
            r.restitutionTarget = vn < -settings.restitutionThreshold ? -e * vn : noBounce;
            r.impulse = &cp.normalImpulse;
            rows_.push_back(r);
        }
    }
    stats_.rows = int(rows_.size());
    if (rows_.empty()) return;

    if (settings.type == ContactSolverType::SequentialImpulses) {
        if (settings.warmStart) {
            for (const Row& r : rows_)
                applyRowImpulse(r, *r.impulse);
        } else {
            for (const Row& r : rows_)
                *r.impulse = 0;
        }
        sequentialImpulses(false);
    } else {
        baraffLcp(false);
    }

    // Restitution: rows that were hit (impulse > 0) while approaching fast get
    // the bounce target. Doing it after the main solve means speculative
    // contacts, which stop a body just before it touches, still bounce.
    bool bounce = false;
    for (Row& r : rows_) {
        if (r.restitutionTarget != noBounce && *r.impulse > 0)
            bounce = true;
        else
            r.restitutionTarget = noBounce;
    }
    if (!bounce) return;
    if (settings.type == ContactSolverType::SequentialImpulses)
        sequentialImpulses(true);
    else
        baraffLcp(true);
}

/// Projected Gauss-Seidel: solve one row at a time with the others fixed,
/// clamping the accumulated impulse (not the increment) to >= 0.
void ContactSolver::sequentialImpulses(bool restitution) {
    for (int it = 0; it < settings.iterations; ++it) {
        for (const Row& r : rows_) {
            Real target = restitution ? std::max(r.bias, r.restitutionTarget) : r.bias;
            Real dLambda = (target - normalVelocity(r)) * r.invK;
            Real old = *r.impulse;
            *r.impulse = std::max(old + dLambda, Real(0));
            applyRowImpulse(r, *r.impulse - old);
        }
    }
}

/// Builds A = J M^-1 J^T and b = v_n - target from the current velocities, solves
/// the LCP exactly, and adds the resulting impulses on top of those already applied.
void ContactSolver::baraffLcp(bool restitution) {
    const int n = int(rows_.size());
    DenseMatrix A(n);
    std::vector<Real> b(static_cast<size_t>(n)), x;

    // Rows i and j are coupled through every body they share.
    auto coupling = [](const RigidBody* body, Real si, const Vec3& ni, const Vec3& rxni, Real sj,
                       const Vec3& nj, const Vec3& rxnj) {
        return si * sj * (body->invMass() * dot(ni, nj) + dot(rxni, body->Iinv() * rxnj));
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
                const Vec3& rxni = u == 0 ? ri.raxn : ri.rbxn;
                if (bi == rj.a) sum += coupling(bi, si, ri.n, rxni, 1, rj.n, rj.raxn);
                if (bi == rj.b) sum += coupling(bi, si, ri.n, rxni, -1, rj.n, rj.rbxn);
            }
            A(i, j) = sum;
        }
        trace += A(i, i);
        Real target = restitution ? std::max(ri.bias, ri.restitutionTarget) : ri.bias;
        b[size_t(i)] = normalVelocity(ri) - target;
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
        if (!restitution) *r.impulse = 0; // the LCP gives the whole impulse; no warm start
        *r.impulse += x[size_t(i)];
        applyRowImpulse(r, x[size_t(i)]);
    }
}

} // namespace ncotb
