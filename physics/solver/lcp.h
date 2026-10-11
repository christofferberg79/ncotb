#pragma once

#include "math/vec3.h"

#include <vector>

namespace ncotb {

/// Dense square matrix, row-major.
class DenseMatrix {
public:
    explicit DenseMatrix(int n = 0) : n_(n), a_(size_t(n) * size_t(n), 0) {}
    int size() const { return n_; }
    Real& operator()(int i, int j) { return a_[size_t(i) * size_t(n_) + size_t(j)]; }
    Real operator()(int i, int j) const { return a_[size_t(i) * size_t(n_) + size_t(j)]; }

private:
    int n_;
    std::vector<Real> a_;
};

struct LcpResult {
    bool solved = false; ///< false if the pivot limit was hit or no step was possible
    int pivots = 0;
};

/// Solves the linear complementarity problem
///
///     w = A x + b,   x >= 0,   w >= 0,   x_i w_i = 0
///
/// with the pivoting method of Baraff, "Fast Contact Force Computation for
/// Nonpenetrating Rigid Bodies" (SIGGRAPH 1994), for symmetric positive
/// (semi)definite A. Unlike the old cids version it never loops forever: it
/// stops after maxPivots pivots (default 10 n + 100) and reports failure.
/// On return x holds the solution, or the best iterate found (x >= 0).
LcpResult solveLcpBaraff(const DenseMatrix& A, const std::vector<Real>& b, std::vector<Real>& x,
                         int maxPivots = -1);

} // namespace ncotb
