#pragma once

#include "collision/collide.h"

#include <vector>

namespace ncotb {

class RigidBody;

enum class ContactSolverType {
    SequentialImpulses, ///< projected Gauss-Seidel with warm starting (Catto)
    BaraffLcp,          ///< exact LCP solve with Baraff's pivoting method
};

const char* name(ContactSolverType type);

struct ContactSettings {
    ContactSolverType type = ContactSolverType::SequentialImpulses;
    int iterations = 10;             ///< sequential impulses: passes over all contacts
    bool warmStart = true;           ///< sequential impulses: start from last step's impulses
    Real baumgarte = 0.2;            ///< fraction of the penetration removed per step
    Real slop = 0.005;               ///< penetration (m) left alone, to avoid jitter
    Real restitutionThreshold = 0.5; ///< no bounce below this approach speed (m/s)
};

struct ContactSolverStats {
    int rows = 0;           ///< contact points solved
    int pivots = 0;         ///< Baraff LCP: pivots used
    bool lcpFailed = false; ///< Baraff LCP: pivot limit hit (result is approximate)
};

/// Velocity-level contact solver. Each contact point is a constraint row
///
///     v_n = n . (v_a + w_a x r_a - v_b - w_b x r_b) >= target,   impulse >= 0,
///
/// complementary (an impulse only where the constraint is active). The target
/// is 0 for touching contacts, pushes out penetration beyond the slop
/// (Baumgarte), and allows approach up to gap / h for contacts that are still
/// apart (speculative contacts). Restitution is a second pass on the rows that
/// were hit while approaching faster than the threshold.
class ContactSolver {
public:
    ContactSettings settings;

    /// Changes the bodies' velocities (P, L) so the contacts hold after a step of h.
    /// Stores each point's accumulated impulse in ContactPoint::normalImpulse.
    void solve(std::vector<ContactManifold>& contacts, Real h);

    const ContactSolverStats& stats() const { return stats_; }

private:
    struct Row {
        RigidBody* a;
        RigidBody* b;
        Vec3 n, p;              ///< normal (b to a) and contact point
        Vec3 raxn, rbxn;        ///< r x n for each body
        Real invK;              ///< 1 / (J M^-1 J^T): effective mass along n
        Real bias;              ///< non-penetration target velocity
        Real restitutionTarget; ///< -e * initial v_n, or -inf if no bounce
        Real* impulse;          ///< accumulated impulse, in the manifold
    };

    Real normalVelocity(const Row& r) const;
    void applyRowImpulse(const Row& r, Real dLambda) const;
    void sequentialImpulses(bool restitution);
    void baraffLcp(bool restitution);

    std::vector<Row> rows_;
    ContactSolverStats stats_;
};

} // namespace ncotb
