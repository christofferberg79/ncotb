#pragma once

#include "math/quat.h"

namespace ncotb {

class RigidBody;
struct SnapshotAccess; // saves and restores the private state (io/snapshot.cpp)

/// Settings shared by all joints (set by the World from the contact settings).
struct JointSettings {
    Real baumgarte = 0.2;     ///< fraction of the position error corrected per step
    Real maxCorrection = 2.0; ///< cap on the correction velocity (m/s, rad/s)
};

/// A joint between bodies a and b, solved as velocity-level constraint rows by
/// sequential impulses (Catto's approach). b may be a static body, e.g. the
/// World's ground anchor.
///
/// Each step: prepare() computes the lever arms, effective masses and the
/// position error; warmStart() reapplies last step's impulses; solve() is one
/// Gauss-Seidel pass, with useBias = true while also correcting the position
/// error and false in the relax phase.
class Joint {
public:
    Joint(RigidBody& a, RigidBody& b) : a_(&a), b_(&b) {}
    virtual ~Joint() = default;

    RigidBody& bodyA() const { return *a_; }
    RigidBody& bodyB() const { return *b_; }

    virtual void prepare(Real h, const JointSettings& settings) = 0;
    virtual void warmStart() = 0;
    virtual void solve(bool useBias) = 0;

    /// Anchor points in world space (for drawing and for measuring the error).
    virtual Vec3 anchorA() const = 0;
    virtual Vec3 anchorB() const = 0;
    /// How far the joint is from holding (m): the gap between the anchors.
    virtual Real error() const { return length(anchorB() - anchorA()); }

protected:
    /// Changing a motor or limits must wake sleeping bodies (World::sleeping).
    void wakeBodies();

    RigidBody* a_;
    RigidBody* b_;
    JointSettings settings_{}; ///< from the last prepare()
};

/// Ball and socket: anchor points on a and b coincide (3 rows, solved as one block).
class BallJoint : public Joint {
    friend struct SnapshotAccess;

public:
    /// anchor in world space; the bodies' current poses define the local anchors.
    BallJoint(RigidBody& a, RigidBody& b, const Vec3& anchor);

    void prepare(Real h, const JointSettings& settings) override;
    void warmStart() override;
    void solve(bool useBias) override;
    Vec3 anchorA() const override;
    Vec3 anchorB() const override;
    /// Impulse on b from the last step (a gets the opposite); force = impulse / h.
    const Vec3& impulse() const { return impulse_; }

protected:
    Vec3 localA_, localB_; ///< anchors in body coordinates
    Vec3 rA_, rB_;         ///< lever arms in world space (fixed during a step)
    Mat3 invK_;            ///< inverse of the 3x3 effective mass matrix
    Vec3 bias_;            ///< correction velocity for the position error
    Vec3 impulse_{};       ///< accumulated impulse on b (a gets the opposite)
};

/// Hinge (revolute): a ball joint plus two angular rows that keep the hinge
/// axes of a and b aligned. Optional angle limits and a motor about the axis.
class HingeJoint : public BallJoint {
    friend struct SnapshotAccess;

public:
    /// anchor and axis in world space, at the bodies' current poses (angle 0).
    HingeJoint(RigidBody& a, RigidBody& b, const Vec3& anchor, const Vec3& axis);

    void setLimits(Real lower, Real upper); ///< radians, lower <= 0 <= upper
    void disableLimits() {
        if (limits_) wakeBodies();
        limits_ = false;
    }
    /// Drive the relative angular velocity (b relative to a about the axis)
    /// towards speed, with at most maxTorque.
    void setMotor(Real speed, Real maxTorque);
    void disableMotor() {
        if (motor_) wakeBodies();
        motor_ = false;
    }

    /// Angle of b relative to a about the axis (radians, 0 at creation).
    Real angle() const;
    Vec3 axis() const; ///< world-space hinge axis (on a)

    void prepare(Real h, const JointSettings& settings) override;
    void warmStart() override;
    void solve(bool useBias) override;

private:
    Vec3 localAxisA_, localAxisB_; ///< hinge axis in each body
    Vec3 localRefA_, localRefB_;   ///< perpendicular reference vectors, for the angle
    Vec3 axis_, t1_, t2_;          ///< world axis and two perpendicular directions
    Real invKAngular_[2][2]{};     ///< inverse 2x2 effective mass of the alignment rows
    Real alignBias_[2]{};
    Real alignImpulse_[2]{};
    Real invKAxis_ = 0; ///< effective mass about the axis (limit and motor rows)
    bool limits_ = false, motor_ = false;
    Real lower_ = 0, upper_ = 0;
    Real motorSpeed_ = 0, maxMotorTorque_ = 0;
    Real angle_ = 0, h_ = 0;
    Real lowerImpulse_ = 0, upperImpulse_ = 0, motorImpulse_ = 0;
};

/// Slider (prismatic): b may only translate along an axis fixed in a; the
/// relative rotation is locked. Optional limits and a motor along the axis.
class SliderJoint : public Joint {
    friend struct SnapshotAccess;

public:
    /// anchor and axis in world space, at the bodies' current poses (position 0).
    SliderJoint(RigidBody& a, RigidBody& b, const Vec3& anchor, const Vec3& axis);

    void setLimits(Real lower, Real upper); ///< metres along the axis
    void disableLimits() {
        if (limits_) wakeBodies();
        limits_ = false;
    }
    void setMotor(Real speed, Real maxForce);
    void disableMotor() {
        if (motor_) wakeBodies();
        motor_ = false;
    }

    Real position() const; ///< displacement of b along the axis (0 at creation)
    bool hasLimits() const { return limits_; }
    Real lowerLimit() const { return lower_; }
    Real upperLimit() const { return upper_; }
    Real motorSpeed() const { return motorSpeed_; }
    Real maxMotorForce() const { return maxMotorForce_; }
    Vec3 axis() const;

    void prepare(Real h, const JointSettings& settings) override;
    void warmStart() override;
    void solve(bool useBias) override;
    Vec3 anchorA() const override;
    Vec3 anchorB() const override;

private:
    /// A linear row along direction n: J_a = (-n, -(r_a + d) x n), J_b = (n, r_b x n).
    struct LinearRow {
        Vec3 n, aAngular, bAngular;
        Real invK = 0, bias = 0, impulse = 0;
    };
    Real rowVelocity(const LinearRow& row) const;
    void applyRow(const LinearRow& row, Real lambda) const;
    LinearRow makeRow(const Vec3& n, Real error, Real h, const JointSettings& settings) const;

    Vec3 localA_, localB_, localAxis_;
    Quat relativeQ0_; ///< q_a^-1 q_b at creation: the locked relative rotation
    Vec3 rA_, rB_, d_, axis_;
    LinearRow perp_[2];
    LinearRow axial_; ///< along the axis: limits and motor share its effective mass
    Mat3 invKAngular_;
    Vec3 angularBias_{}, angularImpulse_{};
    bool limits_ = false, motor_ = false;
    Real lower_ = 0, upper_ = 0, motorSpeed_ = 0, maxMotorForce_ = 0;
    Real position_ = 0, h_ = 0;
    Real lowerImpulse_ = 0, upperImpulse_ = 0, motorImpulse_ = 0;
};

/// A rigid rod between a point on a and a point on b, with ball joints at both
/// ends: their distance stays fixed (1 row). Like a car's camber link or tie rod.
class DistanceJoint : public Joint {
    friend struct SnapshotAccess;

public:
    /// Anchors in world space, on the bodies at their current poses; the rod's
    /// length is their current distance.
    DistanceJoint(RigidBody& a, RigidBody& b, const Vec3& anchorA, const Vec3& anchorB);

    Real length() const { return length_; }

    void prepare(Real h, const JointSettings& settings) override;
    void warmStart() override;
    void solve(bool useBias) override;
    Vec3 anchorA() const override;
    Vec3 anchorB() const override;
    Real error() const override;

private:
    Vec3 localA_, localB_;
    Real length_;
    Vec3 rA_{}, rB_{}, n_{}; ///< lever arms and the rod's direction (a to b) this step
    Real invK_ = 0, bias_ = 0;
    Real impulse_ = 0; ///< along n on b (a gets the opposite)
};

/// Pulls a point on body b towards a moving world target, like a stiff spring
/// with a damper (Catto's "mouse joint"): a soft point constraint with
///
///     stiffness k = m (2 pi f)^2,   damping c = 2 m zeta (2 pi f)
///
/// for b's mass m, frequency f and damping ratio zeta, and the force capped at
/// maxForce, so a drag can't fling a body or tear it out of other joints.
/// a is the static anchor (World::ground()).
///
/// The spring only damps the motion of the grabbed point; the body would keep
/// swinging around it like an undamped pendulum. For comfortable dragging the
/// held body also gets angular damping (an interaction choice, not physics:
/// like extra air resistance while held; 0 turns it off).
class MouseJoint : public Joint {
    friend struct SnapshotAccess;

public:
    /// grabPoint in world space, on b at its current pose; the target starts there.
    MouseJoint(RigidBody& ground, RigidBody& b, const Vec3& grabPoint, Real frequency, Real dampingRatio,
               Real maxForce);

    void setTarget(const Vec3& target) { target_ = target; }
    const Vec3& target() const { return target_; }
    void setStiffness(Real frequency, Real dampingRatio) {
        frequency_ = frequency;
        dampingRatio_ = dampingRatio;
    }
    void setMaxForce(Real maxForce) { maxForce_ = maxForce; }
    void setAngularDamping(Real perSecond) { angularDamping_ = perSecond; }
    Real maxForce() const { return maxForce_; }
    /// Force from the last step (N).
    Real force() const { return h_ > 0 ? length(impulse_) / h_ : 0; }

    void prepare(Real h, const JointSettings& settings) override;
    void warmStart() override;
    void solve(bool useBias) override;
    Vec3 anchorA() const override { return target_; }
    Vec3 anchorB() const override;

private:
    Vec3 localB_, target_;
    Real frequency_, dampingRatio_, maxForce_;
    Real angularDamping_ = 10; ///< 1/s: settles a box jerked 2.6 m in ~3 s (3/s: still swinging)
    Vec3 rB_{}, bias_{};
    Mat3 invK_;
    Real gamma_ = 0, h_ = 0;
    Vec3 impulse_{};
};

} // namespace ncotb
