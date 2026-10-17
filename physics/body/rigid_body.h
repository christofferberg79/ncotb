#pragma once

#include "body/shape.h"
#include "math/quat.h"

#include <cstdint>

namespace ncotb {

/// A rigid body following Baraff's notation (Rigid Body Simulation I, section 2.9).
///
/// State variables:  x (position), q (orientation), P (linear momentum), L (angular momentum).
/// Derived:          v = P/M, R = R(q), I^-1 = R Ibody^-1 R^T, omega = I^-1 L.
class RigidBody {
public:
    /// mass <= 0 creates a static (immovable) body.
    RigidBody(const Shape& shape, Real mass);

    // --- state ---
    Vec3 x{};
    Quat q{};
    Vec3 P{};
    Vec3 L{};

    /// Coefficient of restitution (0 = no bounce, 1 = elastic). A contact uses the
    /// larger value of the two bodies.
    Real restitution = 0.3;
    /// Coulomb friction coefficient. A contact uses sqrt(mu_a mu_b).
    Real friction = 0.5;

    /// Bodies in the same non-zero group don't collide with each other (the parts
    /// of a car, say). 0: collides with everything.
    int collisionGroup = 0;

    /// Sleeping (see World::sleeping): a body at rest long enough is left out of
    /// the step until something touches or pushes it.
    bool asleep = false;
    Real sleepTime = 0; ///< how long it has been nearly still (s)
    void wake() {
        asleep = false;
        sleepTime = 0;
    }
    /// A dynamic body that takes part in the step.
    bool isAwake() const { return !isStatic() && !asleep; }

    /// Free for the application to use (the app stores a colour); ignored by the physics.
    std::uintptr_t userData = 0;

    // --- force/torque accumulators (cleared after each step) ---
    Vec3 force{};
    Vec3 torque{};

    void addForce(const Vec3& f) { force += f; }
    /// Force f applied at world-space point p contributes torque (p - x) x f.
    void addForceAtPoint(const Vec3& f, const Vec3& p) {
        force += f;
        torque += cross(p - x, f);
    }
    void clearForces() {
        force = {};
        torque = {};
    }

    /// Apply impulse J at world-space point p: P += J, L += (p - x) x J. Updates v
    /// and omega directly (I^-1 is unchanged, so this matches updateDerived()).
    void applyImpulse(const Vec3& J, const Vec3& p) { applyImpulseAtOffset(J, p - x); }
    /// Same, with the point given relative to the centre of mass (r = p - x).
    void applyImpulseAtOffset(const Vec3& J, const Vec3& r) {
        if (isStatic()) return;
        Vec3 dL = cross(r, J);
        P += J;
        L += dL;
        v_ += J * invMass_;
        omega_ += Iinv_ * dL;
    }

    /// Apply an angular impulse (a torque times a time step): L += J.
    void applyAngularImpulse(const Vec3& J) {
        if (isStatic()) return;
        L += J;
        omega_ += Iinv_ * J;
    }

    /// Recompute R, I^-1, v and omega from the state variables.
    void updateDerived();

    void setLinearVelocity(const Vec3& vel) {
        P = vel * mass_;
        updateDerived();
        wake();
    }
    /// Sets L = I omega so that the body spins with the given angular velocity.
    void setAngularVelocity(const Vec3& w);

    bool isStatic() const { return invMass_ == 0; }
    Real mass() const { return mass_; }
    Real invMass() const { return invMass_; }
    const Shape& shape() const { return shape_; }
    const Mat3& Ibody() const { return Ibody_; }

    const Mat3& R() const { return R_; }
    const Mat3& Iinv() const { return Iinv_; }
    Mat3 I() const { return R_ * Ibody_ * transpose(R_); }
    const Vec3& v() const { return v_; }
    const Vec3& omega() const { return omega_; }

    /// Angular velocity the body would have with orientation q and its current L.
    Vec3 omegaAt(const Quat& orientation) const;

    /// Velocity of the world-space point p attached to the body: v + omega x (p - x).
    Vec3 pointVelocity(const Vec3& p) const { return v_ + cross(omega_, p - x); }

    /// Remembers v and omega before this step's forces are applied. Restitution
    /// uses these: the impact velocity, not the one including this step's gravity.
    void markStepStart() {
        vStart_ = v_;
        omegaStart_ = omega_;
    }
    Vec3 pointVelocityAtStepStart(const Vec3& p) const { return vStart_ + cross(omegaStart_, p - x); }

    Real kineticEnergy() const;

private:
    Shape shape_;
    Real mass_;
    Real invMass_;
    Mat3 Ibody_;
    Mat3 IbodyInv_;

    Mat3 R_ = Mat3::identity();
    Mat3 Iinv_;
    Vec3 v_{};
    Vec3 omega_{};
    Vec3 vStart_{};
    Vec3 omegaStart_{};
};

} // namespace ncotb
