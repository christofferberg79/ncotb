#include "joints/joint.h"

#include "body/rigid_body.h"

#include <algorithm>
#include <cmath>

namespace ncotb {

namespace {

/// Correction velocity beta/h * error, capped in magnitude.
Real correction(Real error, Real h, const JointSettings& s) {
    return std::clamp(s.baumgarte / h * error, -s.maxCorrection, s.maxCorrection);
}

Vec3 correction(const Vec3& error, Real h, const JointSettings& s) {
    Vec3 v = error * (s.baumgarte / h);
    Real len = length(v);
    return len > s.maxCorrection ? v * (s.maxCorrection / len) : v;
}

Vec3 velocityAt(const RigidBody& b, const Vec3& r) { return b.v() + cross(b.omega(), r); }

Real safeInverse(Real k) { return k > 0 ? 1 / k : 0; }

/// Accumulates impulse into acc, clamped to [lo, hi]; returns the applied change.
Real clampAccumulate(Real& acc, Real delta, Real lo, Real hi) {
    Real old = acc;
    acc = std::clamp(acc + delta, lo, hi);
    return acc - old;
}

/// Mass and inertia as the position projection sees them: a sleeping (or
/// static) body doesn't move, as if infinitely heavy.
Real invMassOf(const RigidBody& b) { return b.isAwake() ? b.invMass() : 0; }
Mat3 invInertiaOf(const RigidBody& b) { return b.isAwake() ? b.Iinv() : Mat3{}; }

/// True if a motor is switched on or its settings change noticeably (a servo
/// re-sending almost the same speed every step mustn't keep its bodies awake).
bool motorChanged(bool on, Real oldSpeed, Real oldMax, Real speed, Real max) {
    return !on || std::abs(speed - oldSpeed) > 1e-3 || max != oldMax;
}

} // namespace

void Joint::wakeBodies() {
    a_->wake();
    b_->wake();
}

// ------------------------------------------------------------------ ball joint

BallJoint::BallJoint(RigidBody& a, RigidBody& b, const Vec3& anchor) : Joint(a, b) {
    localA_ = transpose(a.R()) * (anchor - a.x);
    localB_ = transpose(b.R()) * (anchor - b.x);
}

Vec3 BallJoint::anchorA() const { return a_->x + a_->R() * localA_; }
Vec3 BallJoint::anchorB() const { return b_->x + b_->R() * localB_; }

void BallJoint::prepare(Real h, const JointSettings& settings) {
    settings_ = settings;
    const RigidBody& a = *a_;
    const RigidBody& b = *b_;
    rA_ = a.R() * localA_;
    rB_ = b.R() * localB_;
    // K = (1/m_a + 1/m_b) I - [r_a]x I_a^-1 [r_a]x - [r_b]x I_b^-1 [r_b]x
    Mat3 sa = Mat3::star(rA_), sb = Mat3::star(rB_);
    Mat3 K = Mat3::identity() * (a.invMass() + b.invMass()) - sa * a.Iinv() * sa - sb * b.Iinv() * sb;
    invK_ = inverse(K);
    bias_ = correction((b.x + rB_) - (a.x + rA_), h, settings);
}

void BallJoint::warmStart() {
    b_->applyImpulseAtOffset(impulse_, rB_);
    a_->applyImpulseAtOffset(-impulse_, rA_);
}

void BallJoint::solve(bool useBias) {
    // C = p_b - p_a = 0; drive dC/dt to -bias (or 0 when relaxing).
    Vec3 cdot = velocityAt(*b_, rB_) - velocityAt(*a_, rA_);
    Vec3 lambda = invK_ * ((useBias ? -bias_ : Vec3{}) - cdot);
    impulse_ += lambda;
    b_->applyImpulseAtOffset(lambda, rB_);
    a_->applyImpulseAtOffset(-lambda, rA_);
}

void BallJoint::solvePosition(Real beta) {
    RigidBody& a = *a_;
    RigidBody& b = *b_;
    if (!a.isAwake() && !b.isAwake()) return;
    const Vec3 rA = a.R() * localA_, rB = b.R() * localB_;
    const Vec3 C = (b.x + rB) - (a.x + rA);
    const Mat3 sa = Mat3::star(rA), sb = Mat3::star(rB);
    const Mat3 K = Mat3::identity() * (invMassOf(a) + invMassOf(b)) - sa * invInertiaOf(a) * sa -
                   sb * invInertiaOf(b) * sb;
    const Vec3 P = inverse(K) * (C * -beta);
    b.applyPositionImpulse(P, rB);
    a.applyPositionImpulse(-P, rA);
}

// ------------------------------------------------------------------ distance (rod)

DistanceJoint::DistanceJoint(RigidBody& a, RigidBody& b, const Vec3& anchorA, const Vec3& anchorB)
    : Joint(a, b),
      length_(ncotb::length(anchorB - anchorA)) {
    localA_ = transpose(a.R()) * (anchorA - a.x);
    localB_ = transpose(b.R()) * (anchorB - b.x);
}

Vec3 DistanceJoint::anchorA() const { return a_->x + a_->R() * localA_; }
Vec3 DistanceJoint::anchorB() const { return b_->x + b_->R() * localB_; }
Real DistanceJoint::error() const { return std::abs(ncotb::length(anchorB() - anchorA()) - length_); }

void DistanceJoint::prepare(Real h, const JointSettings& settings) {
    settings_ = settings;
    const RigidBody& a = *a_;
    const RigidBody& b = *b_;
    rA_ = a.R() * localA_;
    rB_ = b.R() * localB_;
    Vec3 d = (b.x + rB_) - (a.x + rA_);
    Real len = ncotb::length(d);
    n_ = len > 1e-9 ? d / len : Vec3{1, 0, 0};
    // K = 1/m_a + 1/m_b + (r_a x n) . I_a^-1 (r_a x n) + (r_b x n) . I_b^-1 (r_b x n)
    Vec3 ca = cross(rA_, n_), cb = cross(rB_, n_);
    invK_ = safeInverse(a.invMass() + b.invMass() + dot(ca, a.Iinv() * ca) + dot(cb, b.Iinv() * cb));
    bias_ = correction(len - length_, h, settings);
}

void DistanceJoint::solvePosition(Real beta) {
    RigidBody& a = *a_;
    RigidBody& b = *b_;
    if (!a.isAwake() && !b.isAwake()) return;
    const Vec3 rA = a.R() * localA_, rB = b.R() * localB_;
    const Vec3 d = (b.x + rB) - (a.x + rA);
    const Real len = ncotb::length(d);
    if (len < 1e-9) return;
    const Vec3 n = d / len;
    const Vec3 ca = cross(rA, n), cb = cross(rB, n);
    const Real K =
        invMassOf(a) + invMassOf(b) + dot(ca, invInertiaOf(a) * ca) + dot(cb, invInertiaOf(b) * cb);
    const Real lambda = -beta * (len - length_) * safeInverse(K);
    b.applyPositionImpulse(n * lambda, rB);
    a.applyPositionImpulse(n * -lambda, rA);
}

void DistanceJoint::warmStart() {
    b_->applyImpulseAtOffset(n_ * impulse_, rB_);
    a_->applyImpulseAtOffset(-n_ * impulse_, rA_);
}

void DistanceJoint::solve(bool useBias) {
    // C = |p_b - p_a| - L = 0; dC/dt = n . (v_b - v_a) driven to -bias.
    Real cdot = dot(n_, velocityAt(*b_, rB_) - velocityAt(*a_, rA_));
    Real lambda = ((useBias ? -bias_ : 0) - cdot) * invK_;
    impulse_ += lambda;
    b_->applyImpulseAtOffset(n_ * lambda, rB_);
    a_->applyImpulseAtOffset(-n_ * lambda, rA_);
}

// ------------------------------------------------------------------ hinge

HingeJoint::HingeJoint(RigidBody& a, RigidBody& b, const Vec3& anchor, const Vec3& axis)
    : BallJoint(a, b, anchor) {
    Vec3 n = normalize(axis);
    Vec3 ref = anyPerpendicular(n);
    localAxisA_ = transpose(a.R()) * n;
    localAxisB_ = transpose(b.R()) * n;
    localRefA_ = transpose(a.R()) * ref;
    localRefB_ = transpose(b.R()) * ref;
}

void HingeJoint::setLimits(Real lower, Real upper) {
    if (!limits_ || std::min(lower, upper) != lower_ || std::max(lower, upper) != upper_) wakeBodies();
    limits_ = true;
    lower_ = std::min(lower, upper);
    upper_ = std::max(lower, upper);
}

void HingeJoint::setMotor(Real speed, Real maxTorque) {
    maxTorque = std::max(maxTorque, Real(0));
    if (motorChanged(motor_, motorSpeed_, maxMotorTorque_, speed, maxTorque)) wakeBodies();
    motor_ = true;
    motorSpeed_ = speed;
    maxMotorTorque_ = maxTorque;
}

Vec3 HingeJoint::axis() const { return normalize(a_->R() * localAxisA_); }

Real HingeJoint::angle() const {
    Vec3 n = axis();
    Vec3 refA = a_->R() * localRefA_;
    Vec3 refB = b_->R() * localRefB_;
    return std::atan2(dot(cross(refA, refB), n), dot(refA, refB));
}

void HingeJoint::prepare(Real h, const JointSettings& settings) {
    BallJoint::prepare(h, settings);
    h_ = h;
    axis_ = axis();
    t1_ = anyPerpendicular(axis_);
    t2_ = cross(axis_, t1_);
    const Mat3 I = a_->Iinv() + b_->Iinv();

    // Alignment rows: the relative angular velocity has no component along t1, t2.
    Real k00 = dot(t1_, I * t1_), k01 = dot(t1_, I * t2_), k11 = dot(t2_, I * t2_);
    Real det = k00 * k11 - k01 * k01;
    Real inv = det > 0 ? 1 / det : 0;
    invKAngular_[0][0] = k11 * inv;
    invKAngular_[0][1] = invKAngular_[1][0] = -k01 * inv;
    invKAngular_[1][1] = k00 * inv;
    // Misalignment: b's axis must turn by -(axis_a x axis_b) to line up again.
    Vec3 e = cross(axis_, normalize(b_->R() * localAxisB_));
    alignBias_[0] = correction(dot(e, t1_), h, settings);
    alignBias_[1] = correction(dot(e, t2_), h, settings);

    invKAxis_ = safeInverse(dot(axis_, I * axis_));
    angle_ = angle();
    if (!limits_) lowerImpulse_ = upperImpulse_ = 0;
    if (!motor_) motorImpulse_ = 0;
}

void HingeJoint::warmStart() {
    BallJoint::warmStart();
    Vec3 J = t1_ * alignImpulse_[0] + t2_ * alignImpulse_[1] +
             axis_ * (motorImpulse_ + lowerImpulse_ - upperImpulse_);
    b_->applyAngularImpulse(J);
    a_->applyAngularImpulse(-J);
}

void HingeJoint::solve(bool useBias) {
    auto relativeSpin = [&] { return dot(b_->omega() - a_->omega(), axis_); };
    auto applyAxis = [&](Real lambda) {
        b_->applyAngularImpulse(axis_ * lambda);
        a_->applyAngularImpulse(axis_ * -lambda);
    };

    if (motor_) {
        Real maxImpulse = maxMotorTorque_ * h_;
        applyAxis(clampAccumulate(motorImpulse_, (motorSpeed_ - relativeSpin()) * invKAxis_, -maxImpulse,
                                  maxImpulse));
    }

    if (limits_) {
        // Like speculative contacts: while inside the range the joint may move
        // towards a limit by the remaining angle; past it, push back.
        auto target = [&](Real C) { return C > 0 ? -C / h_ : (useBias ? -correction(C, h_, settings_) : 0); };
        Real lowerTarget = target(angle_ - lower_);
        applyAxis(clampAccumulate(lowerImpulse_, (lowerTarget - relativeSpin()) * invKAxis_, 0, INFINITY));
        Real upperTarget = target(upper_ - angle_);
        applyAxis(-clampAccumulate(upperImpulse_, (upperTarget + relativeSpin()) * invKAxis_, 0, INFINITY));
    }

    // Alignment (2x2 block).
    Vec3 w = b_->omega() - a_->omega();
    Real rhs0 = (useBias ? -alignBias_[0] : 0) - dot(w, t1_);
    Real rhs1 = (useBias ? -alignBias_[1] : 0) - dot(w, t2_);
    Real l0 = invKAngular_[0][0] * rhs0 + invKAngular_[0][1] * rhs1;
    Real l1 = invKAngular_[1][0] * rhs0 + invKAngular_[1][1] * rhs1;
    alignImpulse_[0] += l0;
    alignImpulse_[1] += l1;
    Vec3 J = t1_ * l0 + t2_ * l1;
    b_->applyAngularImpulse(J);
    a_->applyAngularImpulse(-J);

    BallJoint::solve(useBias);
}

void HingeJoint::solvePosition(Real beta) {
    RigidBody& a = *a_;
    RigidBody& b = *b_;
    if (!a.isAwake() && !b.isAwake()) return;
    // Alignment: turn the axes back into line (the 2x2 block across the axis) ...
    const Vec3 axisA = normalize(a.R() * localAxisA_), axisB = normalize(b.R() * localAxisB_);
    const Vec3 e = cross(axisA, axisB);
    const Vec3 t1 = anyPerpendicular(axisA), t2 = cross(axisA, t1);
    const Mat3 I = invInertiaOf(a) + invInertiaOf(b);
    const Real k00 = dot(t1, I * t1), k01 = dot(t1, I * t2), k11 = dot(t2, I * t2);
    const Real det = k00 * k11 - k01 * k01;
    if (det > 0) {
        const Real e0 = -beta * dot(e, t1), e1 = -beta * dot(e, t2);
        const Real l0 = (k11 * e0 - k01 * e1) / det, l1 = (k00 * e1 - k01 * e0) / det;
        const Vec3 J = t1 * l0 + t2 * l1;
        b.applyAngularPositionImpulse(J);
        a.applyAngularPositionImpulse(-J);
    }
    // ... back inside the limits, if it went past them ...
    if (limits_) {
        const Real angle = this->angle();
        const Real excess = angle < lower_ ? angle - lower_ : (angle > upper_ ? angle - upper_ : 0);
        const Vec3 axis = this->axis();
        const Real k = dot(axis, (invInertiaOf(a) + invInertiaOf(b)) * axis);
        if (excess != 0 && k > 0) {
            const Vec3 J = axis * (-beta * excess / k);
            b.applyAngularPositionImpulse(J);
            a.applyAngularPositionImpulse(-J);
        }
    }
    // ... then bring the anchors together.
    BallJoint::solvePosition(beta);
}

// ------------------------------------------------------------------ slider

SliderJoint::SliderJoint(RigidBody& a, RigidBody& b, const Vec3& anchor, const Vec3& axis) : Joint(a, b) {
    localA_ = transpose(a.R()) * (anchor - a.x);
    localB_ = transpose(b.R()) * (anchor - b.x);
    localAxis_ = transpose(a.R()) * normalize(axis);
    relativeQ0_ = conjugate(a.q) * b.q;
}

void SliderJoint::setLimits(Real lower, Real upper) {
    if (!limits_ || std::min(lower, upper) != lower_ || std::max(lower, upper) != upper_) wakeBodies();
    limits_ = true;
    lower_ = std::min(lower, upper);
    upper_ = std::max(lower, upper);
}

void SliderJoint::setMotor(Real speed, Real maxForce) {
    maxForce = std::max(maxForce, Real(0));
    if (motorChanged(motor_, motorSpeed_, maxMotorForce_, speed, maxForce)) wakeBodies();
    motor_ = true;
    motorSpeed_ = speed;
    maxMotorForce_ = maxForce;
}

Vec3 SliderJoint::axis() const { return normalize(a_->R() * localAxis_); }
Vec3 SliderJoint::anchorA() const { return a_->x + a_->R() * localA_; }
Vec3 SliderJoint::anchorB() const { return b_->x + b_->R() * localB_; }
Real SliderJoint::position() const { return dot(anchorB() - anchorA(), axis()); }

SliderJoint::LinearRow SliderJoint::makeRow(const Vec3& n, Real error, Real h,
                                            const JointSettings& settings) const {
    LinearRow row;
    row.n = n;
    row.aAngular = cross(rA_ + d_, n);
    row.bAngular = cross(rB_, n);
    row.invK = safeInverse(a_->invMass() + b_->invMass() + dot(row.aAngular, a_->Iinv() * row.aAngular) +
                           dot(row.bAngular, b_->Iinv() * row.bAngular));
    row.bias = correction(error, h, settings);
    return row;
}

Real SliderJoint::rowVelocity(const LinearRow& row) const {
    return dot(row.n, b_->v()) + dot(row.bAngular, b_->omega()) - dot(row.n, a_->v()) -
           dot(row.aAngular, a_->omega());
}

void SliderJoint::applyRow(const LinearRow& row, Real lambda) const {
    b_->applyImpulseAtOffset(row.n * lambda, rB_);
    a_->applyImpulseAtOffset(row.n * -lambda, rA_ + d_);
}

void SliderJoint::prepare(Real h, const JointSettings& settings) {
    settings_ = settings;
    h_ = h;
    rA_ = a_->R() * localA_;
    rB_ = b_->R() * localB_;
    d_ = (b_->x + rB_) - (a_->x + rA_);
    axis_ = axis();
    Vec3 n1 = anyPerpendicular(axis_), n2 = cross(axis_, n1);

    Real keep[2] = {perp_[0].impulse, perp_[1].impulse};
    perp_[0] = makeRow(n1, dot(d_, n1), h, settings);
    perp_[1] = makeRow(n2, dot(d_, n2), h, settings);
    perp_[0].impulse = keep[0];
    perp_[1].impulse = keep[1];
    axial_ = makeRow(axis_, 0, h, settings);
    position_ = dot(d_, axis_);

    // Locked rotation: q_b should equal q_a * relativeQ0. The error rotation
    // (world space) is q_b (q_a relativeQ0)^-1, as a small-angle vector.
    Quat err = b_->q * conjugate(a_->q * relativeQ0_);
    Vec3 e = err.v * (err.s >= 0 ? 2 : -2);
    angularBias_ = correction(e, h, settings);
    invKAngular_ = inverse(a_->Iinv() + b_->Iinv());

    if (!limits_) lowerImpulse_ = upperImpulse_ = 0;
    if (!motor_) motorImpulse_ = 0;
}

void SliderJoint::solvePosition(Real beta) {
    RigidBody& a = *a_;
    RigidBody& b = *b_;
    if (!a.isAwake() && !b.isAwake()) return;
    // The locked rotation: turn b back by the error rotation (world space).
    const Quat err = b.q * conjugate(a.q * relativeQ0_);
    const Vec3 e = err.v * (err.s >= 0 ? 2 : -2);
    const Mat3 I = invInertiaOf(a) + invInertiaOf(b);
    if (std::abs(determinant(I)) > 0) {
        const Vec3 J = inverse(I) * (e * -beta);
        b.applyAngularPositionImpulse(J);
        a.applyAngularPositionImpulse(-J);
    }
    // No offset across the axis (two rows, one after the other), and back
    // inside the limits along it.
    const Vec3 axis = this->axis();
    const Vec3 n1 = anyPerpendicular(axis), n2 = cross(axis, n1);
    auto row = [&](const Vec3& n, Real error) {
        const Vec3 rA = a.R() * localA_, rB = b.R() * localB_;
        const Vec3 d = (b.x + rB) - (a.x + rA);
        const Vec3 ca = cross(rA + d, n), cb = cross(rB, n);
        const Real K =
            invMassOf(a) + invMassOf(b) + dot(ca, invInertiaOf(a) * ca) + dot(cb, invInertiaOf(b) * cb);
        const Real lambda = -beta * error * safeInverse(K);
        b.applyPositionImpulse(n * lambda, rB);
        a.applyPositionImpulse(n * -lambda, rA + d);
    };
    auto offset = [&] { return (b.x + b.R() * localB_) - (a.x + a.R() * localA_); };
    row(n1, dot(offset(), n1));
    row(n2, dot(offset(), n2));
    if (limits_) {
        const Real p = dot(offset(), axis);
        const Real excess = p < lower_ ? p - lower_ : (p > upper_ ? p - upper_ : 0);
        if (excess != 0) row(axis, excess);
    }
}

void SliderJoint::warmStart() {
    b_->applyAngularImpulse(angularImpulse_);
    a_->applyAngularImpulse(-angularImpulse_);
    applyRow(perp_[0], perp_[0].impulse);
    applyRow(perp_[1], perp_[1].impulse);
    applyRow(axial_, motorImpulse_ + lowerImpulse_ - upperImpulse_);
}

void SliderJoint::solve(bool useBias) {
    if (motor_) {
        Real maxImpulse = maxMotorForce_ * h_;
        Real delta = (motorSpeed_ - rowVelocity(axial_)) * axial_.invK;
        applyRow(axial_, clampAccumulate(motorImpulse_, delta, -maxImpulse, maxImpulse));
    }
    if (limits_) {
        auto target = [&](Real C) { return C > 0 ? -C / h_ : (useBias ? -correction(C, h_, settings_) : 0); };
        Real lowerTarget = target(position_ - lower_);
        applyRow(axial_, clampAccumulate(lowerImpulse_, (lowerTarget - rowVelocity(axial_)) * axial_.invK, 0,
                                         INFINITY));
        Real upperTarget = target(upper_ - position_);
        applyRow(axial_, -clampAccumulate(upperImpulse_, (upperTarget + rowVelocity(axial_)) * axial_.invK, 0,
                                          INFINITY));
    }

    // Locked rotation (3x3 block).
    Vec3 w = b_->omega() - a_->omega();
    Vec3 J = invKAngular_ * ((useBias ? -angularBias_ : Vec3{}) - w);
    angularImpulse_ += J;
    b_->applyAngularImpulse(J);
    a_->applyAngularImpulse(-J);

    // No motion perpendicular to the axis.
    for (LinearRow& row : perp_) {
        Real lambda = ((useBias ? -row.bias : 0) - rowVelocity(row)) * row.invK;
        row.impulse += lambda;
        applyRow(row, lambda);
    }
}

// ------------------------------------------------------------------ mouse joint

MouseJoint::MouseJoint(RigidBody& ground, RigidBody& b, const Vec3& grabPoint, Real frequency,
                       Real dampingRatio, Real maxForce)
    : Joint(ground, b),
      localB_(transpose(b.R()) * (grabPoint - b.x)),
      target_(grabPoint),
      frequency_(frequency),
      dampingRatio_(dampingRatio),
      maxForce_(maxForce) {}

Vec3 MouseJoint::anchorB() const { return b_->x + b_->R() * localB_; }

void MouseJoint::prepare(Real h, const JointSettings& settings) {
    settings_ = settings;
    h_ = h;
    const RigidBody& b = *b_;
    rB_ = b.R() * localB_;

    // Soft constraint (Catto, "Soft Constraints", GDC 2011): a spring k and damper
    // c become a constraint with compliance gamma and position feedback beta.
    const Real m = b.mass() > 0 ? b.mass() : 1;
    const Real omega = 2 * M_PI * frequency_;
    const Real k = m * omega * omega, c = 2 * m * dampingRatio_ * omega;
    const Real hk = h * k, denom = c + hk;
    gamma_ = denom > 0 ? 1 / (h * denom) : 0;
    const Real beta = denom > 0 ? hk / denom : 0;

    // K = 1/m I - [r]x I^-1 [r]x, softened by gamma.
    Mat3 s = Mat3::star(rB_);
    Mat3 K = Mat3::identity() * (b.invMass() + gamma_) - s * b.Iinv() * s;
    invK_ = inverse(K);
    bias_ = ((b.x + rB_) - target_) * (beta / h);

    // Angular damping of the held body: omega *= 1 / (1 + h c), as an impulse.
    if (angularDamping_ > 0 && !b_->isStatic()) {
        Vec3 L = b.I() * b.omega();
        b_->applyAngularImpulse(L * -(1 - 1 / (1 + h * angularDamping_)));
    }
}

void MouseJoint::warmStart() { b_->applyImpulseAtOffset(impulse_, rB_); }

void MouseJoint::solve(bool /*useBias*/) {
    // A spring stays a spring in the relax phase too: its pull is a real force,
    // not a position correction, so the bias is always used.
    Vec3 cdot = b_->v() + cross(b_->omega(), rB_);
    Vec3 lambda = invK_ * -(cdot + bias_ + impulse_ * gamma_);
    Vec3 old = impulse_;
    impulse_ += lambda;
    Real maxImpulse = maxForce_ * h_;
    if (Real len = length(impulse_); len > maxImpulse) impulse_ *= maxImpulse / len;
    b_->applyImpulseAtOffset(impulse_ - old, rB_);
}

} // namespace ncotb
