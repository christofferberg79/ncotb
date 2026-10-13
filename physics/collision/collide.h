#pragma once

#include "math/vec3.h"

namespace ncotb {

class RigidBody;

/// One point of contact between two bodies.
struct ContactPoint {
    Vec3 position; ///< world space, midway between the two surfaces
    Real depth;    ///< penetration depth; negative means separated (within the margin)
    /// Accumulated normal impulse from the contact solver (N s); also used to
    /// warm start the next step.
    Real normalImpulse = 0;
    /// Accumulated friction impulse on body a (world space, perpendicular to the
    /// normal); also warm starts the next step.
    Vec3 tangentImpulse{};
};

/// The contact between two bodies: one normal and up to four points (enough to
/// support a box resting on a face).
struct ContactManifold {
    static constexpr int kMaxPoints = 4;

    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    /// Unit normal pointing from b towards a (Baraff's convention).
    Vec3 normal{};
    int count = 0;
    ContactPoint points[kMaxPoints]{};
};

/// Tests a and b for contact. Returns true and fills m if the shapes overlap or
/// are closer than margin. Supports every pair of sphere, box and plane except
/// plane-plane.
bool collide(RigidBody& a, RigidBody& b, ContactManifold& m, Real margin = 0);

/// Distance between the shapes of a and b (negative when they overlap), or a
/// lower bound on it: it is exact except for box-box, where it is the largest
/// separation along the 15 separating axes. Never more than the true distance,
/// so a body can safely move this far towards the other (conservative advancement).
Real distance(RigidBody& a, RigidBody& b);

} // namespace ncotb
