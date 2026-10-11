#include "body/rigid_body.h"

namespace ncotb {

RigidBody::RigidBody(const Shape& shape, Real mass)
    : shape_(shape),
      mass_(mass > 0 ? mass : 0),
      invMass_(mass > 0 ? 1 / mass : 0),
      Ibody_(mass > 0 ? shape.inertia(mass) : Mat3{}),
      IbodyInv_(mass > 0 ? inverse(Ibody_) : Mat3{}) {
    updateDerived();
}

void RigidBody::updateDerived() {
    R_ = toMat3(q);
    Iinv_ = R_ * IbodyInv_ * transpose(R_);
    v_ = P * invMass_;
    omega_ = Iinv_ * L;
}

void RigidBody::setAngularVelocity(const Vec3& w) {
    R_ = toMat3(q);
    L = I() * w;
    updateDerived();
}

Vec3 RigidBody::omegaAt(const Quat& orientation) const {
    Mat3 R = toMat3(orientation);
    return R * (IbodyInv_ * (transpose(R) * L));
}

Real RigidBody::kineticEnergy() const {
    return Real(0.5) * (dot(P, v_) + dot(L, omega_));
}

} // namespace ncotb
