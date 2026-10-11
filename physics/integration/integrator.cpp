#include "integration/integrator.h"

#include <cassert>

namespace ncotb {

const char* name(IntegratorType type) {
    switch (type) {
    case IntegratorType::Euler: return "Euler (explicit)";
    case IntegratorType::SemiImplicitEuler: return "Semi-implicit Euler";
    case IntegratorType::Midpoint: return "Midpoint";
    case IntegratorType::Heun: return "Heun";
    case IntegratorType::RK4: return "Runge-Kutta 4";
    }
    return "?";
}

void Integrator::step(IntegratorType type, OdeSystem& system, Real t, Real h) {
    const size_t n = size_t(system.dimension());
    for (auto* v : {&y_, &tmp_, &k1_, &k2_, &k3_, &k4_}) v->resize(n);
    system.getState(y_.data());

    switch (type) {
    case IntegratorType::Euler: euler(system, t, h); break;
    case IntegratorType::Midpoint: midpoint(system, t, h); break;
    case IntegratorType::Heun: heun(system, t, h); break;
    case IntegratorType::RK4: rk4(system, t, h); break;
    case IntegratorType::SemiImplicitEuler: assert(!"handled by World"); break;
    }
}

// y(t+h) = y + h f(t, y)
void Integrator::euler(OdeSystem& system, Real t, Real h) {
    system.derivative(t, k1_.data());
    for (size_t i = 0; i < y_.size(); ++i) y_[i] += h * k1_[i];
    system.setState(y_.data());
}

// y(t+h) = y + h f(t + h/2, y + h/2 f(t, y))
void Integrator::midpoint(OdeSystem& system, Real t, Real h) {
    system.derivative(t, k1_.data());
    for (size_t i = 0; i < y_.size(); ++i) tmp_[i] = y_[i] + h / 2 * k1_[i];
    system.setState(tmp_.data());
    system.derivative(t + h / 2, k2_.data());
    for (size_t i = 0; i < y_.size(); ++i) y_[i] += h * k2_[i];
    system.setState(y_.data());
}

// y(t+h) = y + h/2 (f(t, y) + f(t + h, y + h f(t, y)))
void Integrator::heun(OdeSystem& system, Real t, Real h) {
    system.derivative(t, k1_.data());
    for (size_t i = 0; i < y_.size(); ++i) tmp_[i] = y_[i] + h * k1_[i];
    system.setState(tmp_.data());
    system.derivative(t + h, k2_.data());
    for (size_t i = 0; i < y_.size(); ++i) y_[i] += h / 2 * (k1_[i] + k2_[i]);
    system.setState(y_.data());
}

// y(t+h) = y + h/6 (k1 + 2 k2 + 2 k3 + k4)
//   k1 = f(t, y)              k2 = f(t + h/2, y + h/2 k1)
//   k3 = f(t + h/2, y + h/2 k2)  k4 = f(t + h, y + h k3)
void Integrator::rk4(OdeSystem& system, Real t, Real h) {
    const size_t n = y_.size();
    system.derivative(t, k1_.data());

    for (size_t i = 0; i < n; ++i) tmp_[i] = y_[i] + h / 2 * k1_[i];
    system.setState(tmp_.data());
    system.derivative(t + h / 2, k2_.data());

    for (size_t i = 0; i < n; ++i) tmp_[i] = y_[i] + h / 2 * k2_[i];
    system.setState(tmp_.data());
    system.derivative(t + h / 2, k3_.data());

    for (size_t i = 0; i < n; ++i) tmp_[i] = y_[i] + h * k3_[i];
    system.setState(tmp_.data());
    system.derivative(t + h, k4_.data());

    for (size_t i = 0; i < n; ++i) y_[i] += h / 6 * (k1_[i] + 2 * k2_[i] + 2 * k3_[i] + k4_[i]);
    system.setState(y_.data());
}

} // namespace ncotb
