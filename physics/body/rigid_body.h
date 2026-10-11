#pragma once

#include "body/shape.h"
#include "math/quat.h"

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

    /// Recompute R, I^-1, v and omega from the state variables.
    void updateDerived();

    void setLinearVelocity(const Vec3& vel) {
        P = vel * mass_;
        updateDerived();
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
};

} // namespace ncotb
