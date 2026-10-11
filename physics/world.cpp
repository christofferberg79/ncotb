#include "world.h"

namespace ncotb {

RigidBody& World::addBody(const Shape& shape, Real mass) {
    bodies_.push_back(std::make_unique<RigidBody>(shape, mass));
    return *bodies_.back();
}

void World::step(Real h) {
    for (auto& body : bodies_) {
        RigidBody& b = *body;
        if (b.isStatic()) {
            b.clearForces();
            continue;
        }

        // Semi-implicit (symplectic) Euler: update momenta first, then positions
        // using the new velocities. This is what contact solvers will hook into
        // later (between the velocity and position updates).
        b.addForce(gravity * b.mass());
        b.P += b.force * h;
        b.L += b.torque * h;
        b.updateDerived();

        b.x += b.v() * h;

        // Orientation. Baraff's q' = 1/2 [0, omega] q integrated with plain Euler
        // gains energy quickly for tumbling bodies, since omega = I^-1(q) L changes
        // with q during the step. Instead rotate by the exponential map using
        // omega evaluated at the midpoint orientation (2nd order, stays unit).
        Quat qHalf = Quat::fromRotationVector(b.omega() * (h / 2)) * b.q;
        Vec3 omegaHalf = b.omegaAt(qHalf);
        b.q = normalize(Quat::fromRotationVector(omegaHalf * h) * b.q);
        b.updateDerived();

        b.clearForces();
    }
    time_ += h;
}

} // namespace ncotb
