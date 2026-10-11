#pragma once

#include "collision/collide.h"
#include "joints/joint.h"
#include "math/quat.h"

#include <vector>

namespace ncotb {

class RigidBody;

enum class SolverType {
    SequentialImpulses, ///< projected Gauss-Seidel with warm starting (Catto)
    BaraffLcp,          ///< exact LCP solve with Baraff's pivoting method
};

const char* name(SolverType type);

struct SolverSettings {
    SolverType type = SolverType::SequentialImpulses;
    int iterations = 10;             ///< sequential impulses: passes over all contacts (per phase)
    bool warmStart = true;           ///< sequential impulses: start from last step's impulses
    Real baumgarte = 0.2;            ///< fraction of the penetration removed per step
    Real slop = 0.005;               ///< penetration (m) left alone, to avoid jitter
    Real maxPushVelocity = 1.0;      ///< cap on the push-out speed (m/s)
    bool relax = true;               ///< remove the push-out velocity after moving (soft step)
    Real restitutionThreshold = 0.5; ///< no bounce below this approach speed (m/s)
    bool friction = true;            ///< Coulomb friction (two tangent rows per point)
    int staggers = 4;                ///< Baraff LCP with friction: normal/friction alternations
    /// Baraff LCP: above this many contact points the step uses sequential impulses
    /// instead. Each pivot solves a dense system, so cost grows like rows^4.
    int lcpMaxRows = 300;
    Real jointMaxCorrection = 2.0; ///< joints: cap on the position-correction velocity
};

struct SolverStats {
    int rows = 0;             ///< contact points solved
    int pivots = 0;           ///< Baraff LCP: pivots used
    bool lcpFailed = false;   ///< Baraff LCP: no solution found within the pivot limit
    bool lcpTooLarge = false; ///< Baraff LCP: too many rows, sequential impulses were used
};

/// Velocity-level constraint solver for contacts and joints. Each contact point is a
/// constraint row
///
///     v_n = n . (v_a + w_a x r_a - v_b - w_b x r_b) >= target,   impulse >= 0,
///
/// complementary (an impulse only where the constraint is active), plus two
/// friction rows along tangents t1, t2 that try to make the sliding velocity
/// zero, with the friction impulse limited to |f| <= mu * normal impulse
/// (a circular Coulomb cone).
///
/// Sequential impulses solve friction and normal rows together, one at a time.
/// The Baraff LCP handles normals only, so with friction it alternates: exact
/// LCP for the normals with friction fixed, then friction with the normals
/// fixed ("staggered projections", Kaufman et al. 2008).
///
/// A step has three phases (the "soft step" of Box2D v3):
///   1. solveVelocities(): target 0 for touching contacts, a push-out
///      (Baumgarte, capped) for penetration beyond the slop, and gap / h for
///      contacts still apart (speculative contacts).
///   2. the caller moves the bodies with these velocities, which removes overlap;
///   3. relax(): solve again without the push-out, so the push-out moves the
///      bodies but doesn't stay as velocity; then the restitution pass for rows
///      hit while approaching faster than the threshold.
class ConstraintSolver {
public:
    SolverSettings settings;

    /// Builds one row per contact point (impulses are stored in the contact
    /// points) and prepares the joints, which are solved in the same passes.
    void prepare(std::vector<ContactManifold>& contacts, const std::vector<Joint*>& joints, Real h);
    void solveVelocities();
    void relax();

    const SolverStats& stats() const { return stats_; }

private:
    struct Row {
        RigidBody* a;
        RigidBody* b;
        Vec3 n;        ///< normal, b to a
        Vec3 t1, t2;   ///< tangents: n, t1, t2 orthonormal
        Real mu;       ///< combined friction coefficient
        Vec3 ra, rb;   ///< contact point relative to each centre of mass
        Vec3 ra0, rb0; ///< the same at prepare(), and the poses then:
        Vec3 xa0, xb0; ///<   to measure how far the bodies moved
        Quat qa0, qb0;
        Real depth0;            ///< penetration at prepare()
        Real invK;              ///< 1 / (J M^-1 J^T): effective mass along n
        Real invKt1, invKt2;    ///< the same along t1 and t2
        Real lt1, lt2;          ///< accumulated friction impulse along t1, t2
        Real target;            ///< target v_n of the current phase
        Real restitutionTarget; ///< -e * impact v_n, or -inf if no bounce
        Real* impulse;          ///< accumulated normal impulse, in the manifold
        Vec3* tangentImpulse;   ///< friction impulse vector, in the manifold
    };

    Vec3 relativeVelocity(const Row& r) const;
    Real normalVelocity(const Row& r) const { return dot(r.n, relativeVelocity(r)); }
    void applyRowImpulse(const Row& r, const Vec3& J) const;
    /// Returns the current depth after the bodies moved (and updates the effective masses).
    Real updateAnchors(Row& r) const;
    void solveFriction(Row& r) const;
    void storeFriction();
    /// The solver actually used this step (Baraff LCP falls back when too large).
    bool useLcp() const;
    /// One phase of Gauss-Seidel passes: joints first, then each contact point's
    /// friction and normal rows. useBias: joints also correct position errors.
    void sequentialImpulses(bool withFriction, bool withJoints, bool useBias);
    void baraffLcp(bool replace);
    /// LCP for the normals, alternating with friction and joint passes.
    void baraffWithFriction(bool useBias);

    std::vector<Row> rows_;
    std::vector<Joint*> joints_;
    Real h_ = 0;
    SolverStats stats_;
};

} // namespace ncotb
