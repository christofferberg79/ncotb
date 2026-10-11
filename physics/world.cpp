#include "world.h"

#include <algorithm>

namespace ncotb {

RigidBody& World::addBody(const Shape& shape, Real mass) {
    bodies_.push_back(std::make_unique<RigidBody>(shape, mass));
    return *bodies_.back();
}

void World::step(Real h) {
    if (integrator == IntegratorType::SemiImplicitEuler) {
        stepSemiImplicitEuler(h);
    } else {
        generic_.step(integrator, *this, time_, h);
        detectCollisions(); // for display only
        lastSubstep_ = h;
    }
    for (auto& b : bodies_)
        b->clearForces();
    time_ += h;
}

void World::detectCollisions() {
    contacts_.clear();
    ContactManifold m;
    for (size_t i = 0; i < bodies_.size(); ++i) {
        for (size_t j = i + 1; j < bodies_.size(); ++j) {
            RigidBody& a = *bodies_[i];
            RigidBody& b = *bodies_[j];
            if (a.isStatic() && b.isStatic()) continue;
            // Keep the dynamic body as a, so normals point away from static geometry.
            bool hit = a.isStatic() ? collide(b, a, m, contactMargin) : collide(a, b, m, contactMargin);
            if (hit) contacts_.push_back(m);
        }
    }
}

void World::applyForces(Real /*t*/) {
    for (auto& b : bodies_)
        if (!b->isStatic()) b->addForce(gravity * b->mass());
}

void World::stepSemiImplicitEuler(Real h) {
    applyForces(time_); // forces are held constant over the substeps
    const int n = std::max(substeps, 1);
    const Real hs = h / n;
    lastSubstep_ = hs;
    for (int sub = 0; sub < n; ++sub) {
        // Semi-implicit (symplectic) Euler: update momenta first, ...
        for (auto& body : bodies_) {
            RigidBody& b = *body;
            b.markStepStart();
            if (b.isStatic()) continue;
            b.P += b.force * hs;
            b.L += b.torque * hs;
            b.updateDerived();
        }

        // ... let the contact solver correct the velocities ...
        detectCollisions();
        if (resolveContacts) {
            warmStartFromCache();
            solver.solve(contacts_, hs);
            updateCache();
        }

        // ... then move the bodies with the new velocities.
        for (auto& body : bodies_)
            if (!body->isStatic()) integratePosition(*body, hs);
    }
}

void World::integratePosition(RigidBody& b, Real h) {
    b.x += b.v() * h;

    // Orientation. Baraff's q' = 1/2 [0, omega] q integrated with plain Euler
    // gains energy quickly for tumbling bodies, since omega = I^-1(q) L changes
    // with q during the step. Instead rotate by the exponential map using
    // omega evaluated at the midpoint orientation (2nd order, stays unit).
    Quat qHalf = Quat::fromRotationVector(b.omega() * (h / 2)) * b.q;
    Vec3 omegaHalf = b.omegaAt(qHalf);
    b.q = normalize(Quat::fromRotationVector(omegaHalf * h) * b.q);
    b.updateDerived();
}

void World::warmStartFromCache() {
    // A new point continues an old one if it is on the same pair and within 2 cm
    // of where the old point was, measured on body a.
    const Real tolerance2 = 0.02 * 0.02;
    for (ContactManifold& m : contacts_) {
        for (int i = 0; i < m.count; ++i) {
            ContactPoint& p = m.points[i];
            Vec3 local = transpose(m.a->R()) * (p.position - m.a->x);
            Real best = tolerance2;
            for (const CachedPoint& c : cache_) {
                if (c.a != m.a || c.b != m.b) continue;
                Real d2 = lengthSquared(c.localA - local);
                if (d2 < best) {
                    best = d2;
                    p.normalImpulse = c.impulse;
                }
            }
        }
    }
}

void World::updateCache() {
    cache_.clear();
    for (const ContactManifold& m : contacts_)
        for (int i = 0; i < m.count; ++i)
            cache_.push_back(
                {m.a, m.b, transpose(m.a->R()) * (m.points[i].position - m.a->x), m.points[i].normalImpulse});
}

Real World::totalEnergy() const {
    Real e = 0;
    for (auto& b : bodies_)
        e += b->kineticEnergy() - b->mass() * dot(gravity, b->x);
    return e;
}

void World::getState(Real* y) const {
    for (auto& b : bodies_) {
        for (int k = 0; k < 3; ++k) {
            y[k] = b->x[k];
            y[7 + k] = b->P[k];
            y[10 + k] = b->L[k];
        }
        y[3] = b->q.s;
        y[4] = b->q.v.x;
        y[5] = b->q.v.y;
        y[6] = b->q.v.z;
        y += kStateSize;
    }
}

void World::setState(const Real* y) {
    for (auto& b : bodies_) {
        if (!b->isStatic()) {
            b->x = {y[0], y[1], y[2]};
            // The integrators treat q as a plain 4-vector, so it drifts off unit
            // length; R(q) and I^-1 need a unit quaternion.
            b->q = normalize(Quat{y[3], {y[4], y[5], y[6]}});
            b->P = {y[7], y[8], y[9]};
            b->L = {y[10], y[11], y[12]};
            b->updateDerived();
        }
        y += kStateSize;
    }
}

void World::derivative(Real t, Real* dydt) {
    for (auto& b : bodies_)
        b->clearForces();
    applyForces(t);
    for (auto& body : bodies_) {
        const RigidBody& b = *body;
        if (b.isStatic()) {
            std::fill(dydt, dydt + kStateSize, Real(0));
        } else {
            // d/dt (x, q, P, L) = (v, 1/2 [0, omega] q, F, tau)
            Quat qdot = (Quat{0, b.omega()} * b.q) * Real(0.5);
            for (int k = 0; k < 3; ++k) {
                dydt[k] = b.v()[k];
                dydt[7 + k] = b.force[k];
                dydt[10 + k] = b.torque[k];
            }
            dydt[3] = qdot.s;
            dydt[4] = qdot.v.x;
            dydt[5] = qdot.v.y;
            dydt[6] = qdot.v.z;
        }
        dydt += kStateSize;
    }
}

} // namespace ncotb
