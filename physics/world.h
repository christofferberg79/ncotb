#pragma once

#include "body/rigid_body.h"
#include "collision/collide.h"
#include "integration/integrator.h"
#include "solver/contact_solver.h"

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
    /// Removes all bodies; settings (gravity, integrator) are kept.
    void clear() {
        bodies_.clear();
        contacts_.clear();
        cache_.clear();
        time_ = 0;
    }

    /// Advance the simulation by one fixed step h.
    ///
    /// With semi-implicit Euler, contacts are resolved: each of the substeps
    /// integrates forces into velocities, detects contacts, solves the contact
    /// impulses and then moves the bodies with the new velocities. The other
    /// integrators only move bodies freely (contacts are detected for display).
    void step(Real h);

    /// Contact solver and its settings (solver type, iterations, Baumgarte, ...).
    ContactSolver solver;
    /// Substeps per step (semi-implicit Euler with contacts).
    int substeps = 1;
    /// false: contacts are only detected, bodies pass through each other.
    bool resolveContacts = true;
    /// Length of the last (sub)step; contact force = impulse / lastSubstep().
    Real lastSubstep() const { return lastSubstep_; }

    /// Brute-force contact detection over all pairs with at least one dynamic body.
    /// In each manifold a is dynamic (b may be static).
    void detectCollisions();
    /// Contacts found by the last detectCollisions() (or step()).
    const std::vector<ContactManifold>& contacts() const { return contacts_; }
    /// Contacts are also reported for shapes this close but not touching. The
    /// solver uses them as speculative contacts, so fast bodies stop at the surface.
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
    /// Accumulates external forces and torques (gravity for now) on all bodies.
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
    };

    std::vector<std::unique_ptr<RigidBody>> bodies_;
    std::vector<ContactManifold> contacts_;
    std::vector<CachedPoint> cache_;
    Real lastSubstep_ = 0;
    Integrator generic_;
    Real time_ = 0;
};

} // namespace ncotb
