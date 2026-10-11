#pragma once

#include "collision/collide.h"
#include "math/quat.h"

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
    int iterations = 10;             ///< sequential impulses: passes over all contacts (per phase)
    bool warmStart = true;           ///< sequential impulses: start from last step's impulses
    Real baumgarte = 0.2;            ///< fraction of the penetration removed per step
    Real slop = 0.005;               ///< penetration (m) left alone, to avoid jitter
    Real maxPushVelocity = 1.0;      ///< cap on the push-out speed (m/s)
    bool relax = true;               ///< remove the push-out velocity after moving (soft step)
    Real restitutionThreshold = 0.5; ///< no bounce below this approach speed (m/s)
};

struct ContactSolverStats {
    int rows = 0;           ///< contact points solved
    int pivots = 0;         ///< Baraff LCP: pivots used
    bool lcpFailed = false; ///< Baraff LCP: no solution found within the pivot limit
};

/// Velocity-level contact solver. Each contact point is a constraint row
///
///     v_n = n . (v_a + w_a x r_a - v_b - w_b x r_b) >= target,   impulse >= 0,
///
/// complementary (an impulse only where the constraint is active).
///
/// A step has three phases (the "soft step" of Box2D v3):
///   1. solveVelocities(): target 0 for touching contacts, a push-out
///      (Baumgarte, capped) for penetration beyond the slop, and gap / h for
///      contacts still apart (speculative contacts).
///   2. the caller moves the bodies with these velocities, which removes overlap;
///   3. relax(): solve again without the push-out, so the push-out moves the
///      bodies but doesn't stay as velocity; then the restitution pass for rows
///      hit while approaching faster than the threshold.
class ContactSolver {
public:
    ContactSettings settings;

    /// Builds one row per contact point. Impulses are stored in ContactPoint::normalImpulse.
    void prepare(std::vector<ContactManifold>& contacts, Real h);
    void solveVelocities();
    void relax();

    const ContactSolverStats& stats() const { return stats_; }

private:
    struct Row {
        RigidBody* a;
        RigidBody* b;
        Vec3 n;        ///< normal, b to a
        Vec3 ra, rb;   ///< contact point relative to each centre of mass
        Vec3 ra0, rb0; ///< the same at prepare(), and the poses then:
        Vec3 xa0, xb0; ///<   to measure how far the bodies moved
        Quat qa0, qb0;
        Real depth0;            ///< penetration at prepare()
        Real invK;              ///< 1 / (J M^-1 J^T): effective mass along n
        Real target;            ///< target v_n of the current phase
        Real restitutionTarget; ///< -e * impact v_n, or -inf if no bounce
        Real* impulse;          ///< accumulated impulse, in the manifold
    };

    Real normalVelocity(const Row& r) const;
    void applyRowImpulse(const Row& r, Real dLambda) const;
    /// Moves ra, rb with the bodies (after they moved) and returns the current depth.
    Real updateAnchors(Row& r) const;
    void sequentialImpulses();
    void baraffLcp(bool replace);

    std::vector<Row> rows_;
    Real h_ = 0;
    ContactSolverStats stats_;
};

} // namespace ncotb
