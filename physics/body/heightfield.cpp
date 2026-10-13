#include "body/heightfield.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <random>
#include <utility>

namespace ncotb {

Heightfield::Heightfield(int nx, int nz, Real cellSize, std::vector<Real> heights)
    : nx_(nx),
      nz_(nz),
      cell_(cellSize),
      halfX_((nx - 1) * cellSize / 2),
      halfZ_((nz - 1) * cellSize / 2),
      h_(std::move(heights)) {
    assert(nx >= 2 && nz >= 2 && h_.size() == size_t(nx) * size_t(nz));
    auto [lo, hi] = std::minmax_element(h_.begin(), h_.end());
    minH_ = *lo;
    maxH_ = *hi;
}

Heightfield Heightfield::fromFunction(int cellsX, int cellsZ, Real cellSize,
                                      const std::function<Real(Real, Real)>& f) {
    const int nx = cellsX + 1, nz = cellsZ + 1;
    const Real hx = cellsX * cellSize / 2, hz = cellsZ * cellSize / 2;
    std::vector<Real> h(size_t(nx) * size_t(nz));
    for (int k = 0; k < nz; ++k)
        for (int i = 0; i < nx; ++i)
            h[size_t(k) * size_t(nx) + size_t(i)] = f(i * cellSize - hx, k * cellSize - hz);
    return {nx, nz, cellSize, std::move(h)};
}

Heightfield Heightfield::hills(int cells, Real cellSize, Real amplitude, Real wavelength,
                               std::uint32_t seed) {
    // Value noise: random values on a coarse lattice, blended smoothly, summed
    // over octaves of halving wavelength and amplitude.
    constexpr int kLattice = 64; // repeats after kLattice * wavelength
    std::mt19937 rng(seed);
    std::uniform_real_distribution<Real> u(-1, 1);
    std::vector<Real> lattice(size_t(kLattice) * kLattice);
    for (Real& v : lattice)
        v = u(rng);
    auto at = [&](int i, int k) {
        auto wrap = [](int n) { return ((n % kLattice) + kLattice) % kLattice; };
        return lattice[size_t(wrap(k)) * kLattice + size_t(wrap(i))];
    };
    auto noise = [&](Real x, Real z) {
        Real fx = std::floor(x), fz = std::floor(z);
        int i = int(fx), k = int(fz);
        Real s = x - fx, t = z - fz;
        s = s * s * (3 - 2 * s); // smoothstep: no kinks at lattice lines
        t = t * t * (3 - 2 * t);
        Real a = at(i, k) + (at(i + 1, k) - at(i, k)) * s;
        Real b = at(i, k + 1) + (at(i + 1, k + 1) - at(i, k + 1)) * s;
        return a + (b - a) * t;
    };
    return fromFunction(cells, cells, cellSize, [&](Real x, Real z) {
        Real h = 0, amp = amplitude / 2, freq = 1 / wavelength;
        for (int octave = 0; octave < 4; ++octave) {
            // Offset each octave so lattice points don't line up.
            h += amp * noise(x * freq + 17.3 * octave, z * freq - 9.1 * octave);
            amp *= 0.4;
            freq *= 2;
        }
        return h;
    });
}

bool Heightfield::contains(Real x, Real z) const {
    return x >= -halfX_ && x <= halfX_ && z >= -halfZ_ && z <= halfZ_;
}

void Heightfield::locate(Real x, Real z, int& i, int& k, int& half) const {
    Real fx = (x + halfX_) / cell_, fz = (z + halfZ_) / cell_;
    i = std::clamp(int(std::floor(fx)), 0, nx_ - 2);
    k = std::clamp(int(std::floor(fz)), 0, nz_ - 2);
    half = (fz - k) >= (fx - i) ? 0 : 1; // 0: the triangle above the diagonal (towards +z)
}

std::array<Vec3, 3> Heightfield::triangle(int i, int k, int half) const {
    if (half == 0) return {vertex(i, k), vertex(i, k + 1), vertex(i + 1, k + 1)};
    return {vertex(i, k), vertex(i + 1, k + 1), vertex(i + 1, k)};
}

Real Heightfield::height(Real x, Real z) const {
    int i, k, half;
    locate(x, z, i, k, half);
    auto t = triangle(i, k, half);
    Vec3 n = triangleNormal(t);
    // On the plane through the triangle: n . (p - t0) = 0.
    return t[0].y - (n.x * (x - t[0].x) + n.z * (z - t[0].z)) / n.y;
}

Vec3 Heightfield::normal(Real x, Real z) const {
    int i, k, half;
    locate(x, z, i, k, half);
    return triangleNormal(triangle(i, k, half));
}

bool Heightfield::cellRange(Real x0, Real z0, Real x1, Real z1, int& i0, int& k0, int& i1, int& k1) const {
    if (x1 < -halfX_ || x0 > halfX_ || z1 < -halfZ_ || z0 > halfZ_) return false;
    i0 = std::clamp(int(std::floor((x0 + halfX_) / cell_)), 0, nx_ - 2);
    i1 = std::clamp(int(std::floor((x1 + halfX_) / cell_)), 0, nx_ - 2);
    k0 = std::clamp(int(std::floor((z0 + halfZ_) / cell_)), 0, nz_ - 2);
    k1 = std::clamp(int(std::floor((z1 + halfZ_) / cell_)), 0, nz_ - 2);
    return true;
}

} // namespace ncotb
