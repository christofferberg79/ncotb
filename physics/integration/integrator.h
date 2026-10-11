#pragma once

#include "math/vec3.h"

#include <vector>

namespace ncotb {

/// A system of first-order ODEs dy/dt = f(t, y) (Baraff, Differential Equation Basics).
class OdeSystem {
public:
    virtual ~OdeSystem() = default;
    virtual int dimension() const = 0;
    virtual void getState(Real* y) const = 0;
    virtual void setState(const Real* y) = 0;
    /// Writes dy/dt for the state most recently passed to setState().
    virtual void derivative(Real t, Real* dydt) = 0;
};

enum class IntegratorType { Euler, SemiImplicitEuler, Midpoint, Heun, RK4 };

inline constexpr IntegratorType kAllIntegrators[] = {IntegratorType::Euler, IntegratorType::SemiImplicitEuler,
                                                     IntegratorType::Midpoint, IntegratorType::Heun,
                                                     IntegratorType::RK4};

const char* name(IntegratorType type);

/// Generic explicit integrators for an OdeSystem. Semi-implicit Euler needs to know which
/// part of the state is position and which is velocity, so it lives in World instead.
class Integrator {
public:
    /// Advance the system from t to t + h. type must not be SemiImplicitEuler.
    void step(IntegratorType type, OdeSystem& system, Real t, Real h);

private:
    void euler(OdeSystem& system, Real t, Real h);
    void midpoint(OdeSystem& system, Real t, Real h);
    void heun(OdeSystem& system, Real t, Real h);
    void rk4(OdeSystem& system, Real t, Real h);

    // scratch buffers, kept between steps to avoid reallocation
    std::vector<Real> y_, tmp_, k1_, k2_, k3_, k4_;
};

} // namespace ncotb
