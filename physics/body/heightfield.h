#pragma once

#include "math/vec3.h"

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace ncotb {

/// Terrain: a grid of heights over the xz-plane of its body, centred on the body
/// position. nx x nz vertices, cellSize apart. Each cell is split into two
/// triangles along its diagonal from (i, k) to (i + 1, k + 1).
/// All coordinates here are in the body's frame (y up).
class Heightfield {
public:
    /// heights[k * nx + i] is the height of vertex (i, k).
    Heightfield(int nx, int nz, Real cellSize, std::vector<Real> heights);

    /// cellsX x cellsZ cells with heights from f(x, z).
    static Heightfield fromFunction(int cellsX, int cellsZ, Real cellSize,
                                    const std::function<Real(Real, Real)>& f);
    /// Smooth random hills (value noise over a few octaves), the same for the same
    /// seed. amplitude is roughly the height of the largest hills; their width is
    /// about wavelength.
    static Heightfield hills(int cells, Real cellSize, Real amplitude, Real wavelength, std::uint32_t seed);

    int nx() const { return nx_; }
    int nz() const { return nz_; }
    Real cellSize() const { return cell_; }
    Real halfSizeX() const { return halfX_; }
    Real halfSizeZ() const { return halfZ_; }
    Real minHeight() const { return minH_; }
    Real maxHeight() const { return maxH_; }

    Real heightAt(int i, int k) const { return h_[size_t(k) * size_t(nx_) + size_t(i)]; }
    Vec3 vertex(int i, int k) const { return {i * cell_ - halfX_, heightAt(i, k), k * cell_ - halfZ_}; }

    /// True if (x, z) is over the grid.
    bool contains(Real x, Real z) const;
    /// Height and upward unit normal of the triangle under (x, z), which must be over the grid.
    Real height(Real x, Real z) const;
    Vec3 normal(Real x, Real z) const;

    /// Triangle `half` (0 or 1) of cell (i, k), counter-clockwise seen from above.
    std::array<Vec3, 3> triangle(int i, int k, int half) const;
    /// The cell and triangle under (x, z) (over the grid).
    void locate(Real x, Real z, int& i, int& k, int& half) const;
    /// Cells overlapping the rectangle [x0, x1] x [z0, z1], clamped to the grid.
    /// False if there are none.
    bool cellRange(Real x0, Real z0, Real x1, Real z1, int& i0, int& k0, int& i1, int& k1) const;

private:
    int nx_, nz_;
    Real cell_, halfX_, halfZ_;
    Real minH_ = 0, maxH_ = 0;
    std::vector<Real> h_;
};

/// Upward unit normal of a triangle given counter-clockwise seen from above.
inline Vec3 triangleNormal(const std::array<Vec3, 3>& t) {
    return normalize(cross(t[1] - t[0], t[2] - t[0]));
}

} // namespace ncotb
