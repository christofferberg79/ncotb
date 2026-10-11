#pragma once

#include "body/rigid_body.h"

namespace ncotb {

/// A linear spring with a damper between two points on bodies a and b (b may be
/// the World's static ground anchor):
///
///     f = (k (|d| - L0) + c (v_rel . d^)) d^,   d = p_b - p_a,
///
/// pulling a towards b when stretched (and b towards a). A force, not a
/// constraint, so it works with every integrator.
struct Spring {
    RigidBody* a;
    RigidBody* b;
    Vec3 localA, localB; ///< attachment points in body coordinates
    Real restLength;     ///< L0 (m)
    Real stiffness;      ///< k (N/m)
    Real damping;        ///< c (N s/m)

    Vec3 anchorA() const { return a->x + a->R() * localA; }
    Vec3 anchorB() const { return b->x + b->R() * localB; }

    void applyForces() const {
        Vec3 pa = anchorA(), pb = anchorB();
        Vec3 d = pb - pa;
        Real len = length(d);
        if (len < 1e-9) return;
        Vec3 dir = d / len;
        Real stretchRate = dot(b->pointVelocity(pb) - a->pointVelocity(pa), dir);
        Vec3 f = dir * (stiffness * (len - restLength) + damping * stretchRate);
        a->addForceAtPoint(f, pa);
        b->addForceAtPoint(-f, pb);
    }
};

} // namespace ncotb
