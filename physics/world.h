#pragma once

#include "body/rigid_body.h"

#include <memory>
#include <vector>

namespace ncotb {

/// Owns the rigid bodies and advances them in time with fixed steps.
class World {
public:
    Vec3 gravity{0, -9.81, 0};

    RigidBody& addBody(const Shape& shape, Real mass);
    void clear() { bodies_.clear(); time_ = 0; }

    /// Advance the simulation by one fixed step h.
    void step(Real h);

    const std::vector<std::unique_ptr<RigidBody>>& bodies() const { return bodies_; }
    Real time() const { return time_; }

private:
    std::vector<std::unique_ptr<RigidBody>> bodies_;
    Real time_ = 0;
};

} // namespace ncotb
