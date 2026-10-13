#include "world.h"

#include <algorithm>

namespace ncotb {

RigidBody& World::addBody(const Shape& shape, Real mass) {
    bodies_.push_back(std::make_unique<RigidBody>(shape, mass));
    return *bodies_.back();
}

BallJoint& World::addBallJoint(RigidBody& a, RigidBody& b, const Vec3& anchor) {
    joints_.push_back(std::make_unique<BallJoint>(a, b, anchor));
    return static_cast<BallJoint&>(*joints_.back());
}

HingeJoint& World::addHinge(RigidBody& a, RigidBody& b, const Vec3& anchor, const Vec3& axis) {
    joints_.push_back(std::make_unique<HingeJoint>(a, b, anchor, axis));
    return static_cast<HingeJoint&>(*joints_.back());
}

SliderJoint& World::addSlider(RigidBody& a, RigidBody& b, const Vec3& anchor, const Vec3& axis) {
    joints_.push_back(std::make_unique<SliderJoint>(a, b, anchor, axis));
    return static_cast<SliderJoint&>(*joints_.back());
}

MouseJoint& World::addMouseJoint(RigidBody& b, const Vec3& grabPoint, Real frequency, Real dampingRatio,
                                 Real maxForce) {
    joints_.push_back(std::make_unique<MouseJoint>(ground_, b, grabPoint, frequency, dampingRatio, maxForce));
    return static_cast<MouseJoint&>(*joints_.back());
}

void World::removeJoint(const Joint& joint) {
    std::erase_if(joints_, [&](const auto& j) { return j.get() == &joint; });
}

void World::removeBody(const RigidBody& body) {
    std::erase_if(joints_, [&](const auto& j) { return &j->bodyA() == &body || &j->bodyB() == &body; });
    std::erase_if(springs_, [&](const Spring& s) { return s.a == &body || s.b == &body; });
    std::erase_if(contacts_, [&](const ContactManifold& m) { return m.a == &body || m.b == &body; });
    std::erase_if(cache_, [&](const CachedPoint& c) { return c.a == &body || c.b == &body; });
    std::erase_if(bodies_, [&](const auto& b) { return b.get() == &body; });
}

std::optional<RayHit> World::raycast(const Vec3& origin, const Vec3& dir, Real maxDistance,
                                     bool includeStatic) {
    const Vec3 d = normalize(dir);
    std::optional<RayHit> best;
    for (auto& body : bodies_) {
        if (body->isStatic() && !includeStatic) continue;
        // Quick rejection: the ray passes farther from the centre than the bounding radius.
        Real r = body->shape().boundingRadius();
        if (std::isfinite(r)) {
            Vec3 m = body->x - origin;
            Real along = std::max(dot(m, d), Real(0));
            if (lengthSquared(m - d * along) > r * r) continue;
        }
        auto hit = ncotb::raycast(*body, origin, d);
        if (hit && hit->distance <= maxDistance && (!best || hit->distance < best->distance)) best = hit;
    }
    return best;
}

Spring& World::addSpring(RigidBody& a, const Vec3& pa, RigidBody& b, const Vec3& pb, Real stiffness,
                         Real damping, Real restLength) {
    Spring s{&a,
             &b,
             transpose(a.R()) * (pa - a.x),
             transpose(b.R()) * (pb - b.x),
             restLength >= 0 ? restLength : length(pb - pa),
             stiffness,
             damping};
    springs_.push_back(s);
    return springs_.back();
}

RigidBody& World::addBody(const Shape& shape, Real mass, const Vec3& x, const Quat& q) {
    RigidBody& b = addBody(shape, mass);
    b.x = x;
    b.q = q;
    b.updateDerived();
    return b;
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

namespace {

/// Upper bound on the speed of any surface point of b: |v| + |omega| r (a sphere
/// turning in place doesn't move its surface).
Real speedBound(const RigidBody& b) {
    if (b.isStatic()) return 0; // also avoids 0 * infinity for planes
    Real turning = b.shape().type == Shape::Type::Sphere ? 0 : length(b.omega()) * b.shape().boundingRadius();
    return length(b.v()) + turning;
}

/// Moves copy to where body will be after time t at its current velocities.
void predict(RigidBody& copy, const RigidBody& body, Real t) {
    copy.x = body.x + body.v() * t;
    copy.q = normalize(Quat::fromRotationVector(body.omega() * t) * body.q);
    copy.updateDerived();
}

/// Drops speculative points that won't be reached within h: those whose gap is
/// larger than margin + h * (closing speed at the point along the normal).
void keepReachable(ContactManifold& m, Real margin, Real h) {
    int kept = 0;
    for (int i = 0; i < m.count; ++i) {
        const ContactPoint& p = m.points[i];
        Real closing = -dot(m.normal, m.a->pointVelocity(p.position) - m.b->pointVelocity(p.position));
        if (-p.depth <= margin + h * std::max(closing, Real(0))) m.points[kept++] = p;
    }
    m.count = kept;
}

/// Speculative contacts for a pair that is farther apart than margin now but may
/// meet within h; speed bounds how fast their gap can shrink. ms holds their
/// contacts at the current positions and is replaced.
///
/// The normal of a distant contact can be misleading: far from a box, the
/// nearest feature of a ball passing beside it is an edge, and a contact on that
/// edge would stop the ball although it never gets there. So find the time of
/// impact first, by conservative advancement (move both along their velocities
/// by as much as the gap allows, until it is closed or the step is over), and
/// take the contacts there, moved back to the current positions.
void speculate(RigidBody& a, RigidBody& b, std::vector<ContactManifold>& ms, Real margin, Real h,
               Real speed) {
    ms.clear();
    RigidBody a2 = a, b2 = b;
    Real t = 0;
    for (int i = 0;; ++i) {
        Real reach = margin + (h - t) * speed; // farther than this can't be reached
        Real d = distance(a2, b2, reach);
        if (d <= margin || i == 20) break; // met (or very nearly; take it as met)
        t += d / speed;
        if (t >= h) return;
        predict(a2, a, t);
        predict(b2, b, t);
    }
    collide(a2, b2, ms, margin + (h - t) * speed);
    for (ContactManifold& m : ms) {
        // Back to now: the gap is wider by what closes until t.
        for (int i = 0; i < m.count; ++i) {
            ContactPoint& p = m.points[i];
            p.position = a.x + a.R() * (transpose(a2.R()) * (p.position - a2.x));
            Real closing = -dot(m.normal, a.pointVelocity(p.position) - b.pointVelocity(p.position));
            p.depth -= t * closing;
        }
        m.a = &a;
        m.b = &b;
        keepReachable(m, margin, h);
    }
}

} // namespace

void World::detectCollisions(Real h) {
    contacts_.clear();
    auto joined = [&](const RigidBody* a, const RigidBody* b) {
        for (const auto& j : joints_)
            if ((&j->bodyA() == a && &j->bodyB() == b) || (&j->bodyA() == b && &j->bodyB() == a)) return true;
        return false;
    };
    speed_.resize(bodies_.size());
    for (size_t i = 0; i < bodies_.size(); ++i)
        speed_[i] = h > 0 ? speedBound(*bodies_[i]) : 0;
    std::vector<ContactManifold>& ms = pairContacts_;
    for (size_t i = 0; i < bodies_.size(); ++i) {
        for (size_t j = i + 1; j < bodies_.size(); ++j) {
            RigidBody& a = *bodies_[i];
            RigidBody& b = *bodies_[j];
            if (a.isStatic() && b.isStatic()) continue;
            // How far the pair can close within the step, at most.
            const Real margin = contactMargin + h * (speed_[i] + speed_[j]);
            // Cheap rejection with bounding spheres before the exact test.
            Real reach = a.shape().boundingRadius() + b.shape().boundingRadius() + margin;
            if (lengthSquared(a.x - b.x) > reach * reach) continue;
            if (!joints_.empty() && joined(&a, &b)) continue;
            // Keep the dynamic body as a, so normals point away from static geometry.
            RigidBody& dyn = a.isStatic() ? b : a;
            RigidBody& other = a.isStatic() ? a : b;
            ms.clear();
            collide(dyn, other, ms, margin);
            if (ms.empty()) continue;
            if (margin > contactMargin) {
                // Fast enough to reach farther than contactMargin within the step.
                Real gap = INFINITY;
                for (const ContactManifold& m : ms)
                    for (int k = 0; k < m.count; ++k)
                        gap = std::min(gap, -m.points[k].depth);
                if (gap <= contactMargin) {
                    for (ContactManifold& m : ms)
                        keepReachable(m, contactMargin, h); // already touching: keep what it closes on
                } else {
                    speculate(dyn, other, ms, contactMargin, h, speed_[i] + speed_[j]);
                }
            }
            for (const ContactManifold& m : ms)
                if (m.count > 0) contacts_.push_back(m);
        }
    }
}

void World::applyForces(Real /*t*/) {
    for (auto& b : bodies_)
        if (!b->isStatic()) b->addForce(gravity * b->mass());
    for (const Spring& s : springs_)
        s.applyForces();
}

void World::stepSemiImplicitEuler(Real h) {
    // Forces added to the bodies before the step are held over the substeps;
    // gravity and springs are evaluated again in every substep.
    userForce_.resize(bodies_.size());
    userTorque_.resize(bodies_.size());
    for (size_t i = 0; i < bodies_.size(); ++i) {
        userForce_[i] = bodies_[i]->force;
        userTorque_[i] = bodies_[i]->torque;
    }
    const int n = std::max(substeps, 1);
    const Real hs = h / n;
    lastSubstep_ = hs;
    std::vector<ContactManifold> noContacts;
    std::vector<Joint*> joints;
    joints.reserve(joints_.size());
    for (auto& j : joints_)
        joints.push_back(j.get());

    for (int sub = 0; sub < n; ++sub) {
        for (size_t i = 0; i < bodies_.size(); ++i) {
            bodies_[i]->force = userForce_[i];
            bodies_[i]->torque = userTorque_[i];
        }
        applyForces(time_ + sub * hs);

        // Semi-implicit (symplectic) Euler: update momenta first, ...
        for (auto& body : bodies_) {
            RigidBody& b = *body;
            b.markStepStart();
            if (b.isStatic()) continue;
            b.P += b.force * hs;
            b.L += b.torque * hs;
            b.updateDerived();
        }

        // ... let the solver correct the velocities (contacts and joints) ...
        detectCollisions(hs);
        if (resolveContacts) warmStartFromCache();
        solver.prepare(resolveContacts ? contacts_ : noContacts, joints, hs);
        solver.solveVelocities();

        // ... then move the bodies with the new velocities ...
        for (auto& body : bodies_)
            if (!body->isStatic()) integratePosition(*body, hs);

        // ... and remove the correction velocity again, then bounce.
        solver.relax();
        if (resolveContacts) updateCache();
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
                    p.tangentImpulse = c.tangentImpulse;
                }
            }
        }
    }
}

void World::updateCache() {
    cache_.clear();
    for (const ContactManifold& m : contacts_)
        for (int i = 0; i < m.count; ++i)
            cache_.push_back({m.a, m.b, transpose(m.a->R()) * (m.points[i].position - m.a->x),
                              m.points[i].normalImpulse, m.points[i].tangentImpulse});
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
