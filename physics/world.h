#pragma once

#include "body/rigid_body.h"
#include "collision/collide.h"
#include "collision/raycast.h"
#include "forces/spring.h"
#include "integration/integrator.h"
#include "joints/joint.h"
#include "solver/constraint_solver.h"

#include <memory>
#include <vector>

namespace ncotb {

/// Owns the rigid bodies and advances them in time with fixed steps.
///
/// As an OdeSystem the state is, per body, Baraff's 13 values (x, q, P, L).
class World : public OdeSystem {
public:
    static constexpr int kStateSize = 13;

    Vec3 gravity{0, -9.81, 0};
    IntegratorType integrator = IntegratorType::SemiImplicitEuler;

    RigidBody& addBody(const Shape& shape, Real mass);
    /// Adds a body at position x with orientation q (at rest).
    RigidBody& addBody(const Shape& shape, Real mass, const Vec3& x, const Quat& q = {});
    /// Removes all bodies, joints and springs; settings (gravity, integrator) are kept.
    void clear() {
        joints_.clear();
        springs_.clear();
        contacts_.clear();
        cache_.clear();
        bodies_.clear();
        time_ = 0;
    }

    /// A static body at the origin to attach joints and springs to the world.
    /// It takes part in no collisions and isn't in bodies().
    RigidBody& ground() { return ground_; }

    // Joints: anchor (and axis) in world space, at the bodies' current poses.
    BallJoint& addBallJoint(RigidBody& a, RigidBody& b, const Vec3& anchor);
    HingeJoint& addHinge(RigidBody& a, RigidBody& b, const Vec3& anchor, const Vec3& axis);
    SliderJoint& addSlider(RigidBody& a, RigidBody& b, const Vec3& anchor, const Vec3& axis);
    /// A rigid rod from anchorA on a to anchorB on b (ball joints at both ends).
    DistanceJoint& addDistanceJoint(RigidBody& a, RigidBody& b, const Vec3& anchorA, const Vec3& anchorB);
    /// A soft "mouse" joint pulling grabPoint on b towards a target (see MouseJoint).
    MouseJoint& addMouseJoint(RigidBody& b, const Vec3& grabPoint, Real frequency, Real dampingRatio,
                              Real maxForce);
    void removeJoint(const Joint& joint);
    const std::vector<std::unique_ptr<Joint>>& joints() const { return joints_; }

    /// Removes a body, and with it every joint, spring and contact that refers to it.
    void removeBody(const RigidBody& body);

    /// Nearest body hit by the ray origin + t dir (t in [0, maxDistance]); static
    /// bodies only if includeStatic. dir needn't be normalized.
    std::optional<RayHit> raycast(const Vec3& origin, const Vec3& dir, Real maxDistance = INFINITY,
                                  bool includeStatic = false);

    /// Spring with damper between world points pa (on a) and pb (on b), at the
    /// bodies' current poses. restLength < 0: the current distance.
    Spring& addSpring(RigidBody& a, const Vec3& pa, RigidBody& b, const Vec3& pb, Real stiffness,
                      Real damping, Real restLength = -1);
    const std::vector<Spring>& springs() const { return springs_; }
    /// The spring between a and b (in that order), or nullptr. Valid until springs are added or removed.
    Spring* findSpring(const RigidBody& a, const RigidBody& b);

    /// Advance the simulation by one fixed step h.
    ///
    /// With semi-implicit Euler, contacts and joints are resolved: each of the
    /// substeps integrates forces (gravity, springs, forces added to the bodies
    /// before the step) into velocities, detects contacts, solves the contact
    /// and joint impulses and then moves the bodies with the new velocities. The
    /// other integrators move bodies freely under the forces, springs included
    /// (contacts are detected for display, joints are ignored).
    void step(Real h);

    /// Constraint solver for contacts and joints, and its settings (type, iterations, ...).
    ConstraintSolver solver;
    /// Substeps per step (semi-implicit Euler with contacts).
    int substeps = 1;
    /// false: contacts are only detected, bodies pass through each other.
    bool resolveContacts = true;
    /// Length of the last (sub)step; contact force = impulse / lastSubstep().
    Real lastSubstep() const { return lastSubstep_; }

    /// Brute-force contact detection over all pairs with at least one dynamic body,
    /// except pairs connected by a joint (a joint decides how they meet) and pairs
    /// in the same collision group.
    /// In each manifold a is dynamic (b may be static).
    ///
    /// With a step length h > 0 the margin grows with the speed of the pair, so a
    /// fast body sees what it would hit within the step: a contact point is kept
    /// if its gap is at most contactMargin + h * (closing speed at that point).
    /// The solver lets it close the gap but not pass it, so it can't tunnel.
    void detectCollisions(Real h = 0);
    /// Contacts found by the last detectCollisions() (or step()).
    const std::vector<ContactManifold>& contacts() const { return contacts_; }
    /// Contacts are also reported for shapes this close but not touching (plus the
    /// distance they close within a step, see detectCollisions). The solver uses
    /// them as speculative contacts, so fast bodies stop at the surface.
    Real contactMargin = 0.02;

    const std::vector<std::unique_ptr<RigidBody>>& bodies() const { return bodies_; }
    Real time() const { return time_; }

    /// Kinetic plus gravitational potential energy of all bodies.
    Real totalEnergy() const;

    // OdeSystem
    int dimension() const override { return int(bodies_.size()) * kStateSize; }
    void getState(Real* y) const override;
    void setState(const Real* y) override;
    void derivative(Real t, Real* dydt) override;

private:
    /// Accumulates gravity and spring forces on all bodies.
    void applyForces(Real t);
    void stepSemiImplicitEuler(Real h);
    static void integratePosition(RigidBody& b, Real h);
    /// Copies impulses from the previous step's contacts to matching new ones.
    void warmStartFromCache();
    void updateCache();

    struct CachedPoint {
        const RigidBody* a;
        const RigidBody* b;
        Vec3 localA; ///< contact point in a's body coordinates
        Real impulse;
        Vec3 tangentImpulse;
    };

    RigidBody ground_{Shape::sphere(0), 0};
    std::vector<std::unique_ptr<RigidBody>> bodies_;
    std::vector<std::unique_ptr<Joint>> joints_;
    std::vector<Spring> springs_;
    std::vector<Vec3> userForce_, userTorque_; ///< forces added before step(), per body
    std::vector<ContactManifold> contacts_;
    std::vector<CachedPoint> cache_;
    std::vector<Real> speed_;                   ///< per body, for the speculative margin
    std::vector<ContactManifold> pairContacts_; ///< scratch: the contacts of one pair
    Real lastSubstep_ = 0;
    Integrator generic_;
    Real time_ = 0;
};

} // namespace ncotb
