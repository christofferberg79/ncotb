#include "solver/lcp.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ncotb {

namespace {

enum class Set { None, Clamped, NotClamped };

/// Solves M y = r by Gaussian elimination with partial pivoting. M is small
/// (one row per clamped contact). Near-zero pivots are skipped, giving y = 0 in
/// that component, so a singular M (redundant contacts) never fails.
std::vector<Real> solveDense(std::vector<Real> M, std::vector<Real> r, int n, Real tiny) {
    auto at = [&](int i, int j) -> Real& { return M[size_t(i) * size_t(n) + size_t(j)]; };
    std::vector<int> pivotCol(size_t(n), -1);
    for (int col = 0, row = 0; col < n && row < n; ++col) {
        int best = row;
        for (int i = row + 1; i < n; ++i)
            if (std::abs(at(i, col)) > std::abs(at(best, col))) best = i;
        if (std::abs(at(best, col)) <= tiny) continue;
        for (int j = 0; j < n; ++j)
            std::swap(at(row, j), at(best, j));
        std::swap(r[size_t(row)], r[size_t(best)]);
        for (int i = row + 1; i < n; ++i) {
            Real f = at(i, col) / at(row, col);
            if (f == 0) continue;
            for (int j = col; j < n; ++j)
                at(i, j) -= f * at(row, j);
            r[size_t(i)] -= f * r[size_t(row)];
        }
        pivotCol[size_t(row)] = col;
        ++row;
    }
    std::vector<Real> y(size_t(n), 0);
    for (int row = n - 1; row >= 0; --row) {
        int col = pivotCol[size_t(row)];
        if (col < 0) continue;
        Real s = r[size_t(row)];
        for (int j = col + 1; j < n; ++j)
            s -= at(row, j) * y[size_t(j)];
        y[size_t(col)] = s / at(row, col);
    }
    return y;
}

} // namespace

LcpResult solveLcpBaraff(const DenseMatrix& A, const std::vector<Real>& b, std::vector<Real>& x,
                         int maxPivots) {
    const int n = A.size();
    if (maxPivots < 0) maxPivots = 10 * n + 100;
    LcpResult result;

    // Tolerances relative to the problem scale, not absolute numbers (the old
    // code used 0.01, which zeroed meaningful values for light bodies).
    Real scale = 0;
    for (int i = 0; i < n; ++i)
        scale = std::max(scale, std::abs(A(i, i)));
    if (scale == 0) scale = 1;
    const Real tiny = 1e-12 * scale;

    x.assign(size_t(n), 0);
    std::vector<Real> w(b), dx(static_cast<size_t>(n)), dw(static_cast<size_t>(n));
    std::vector<Set> set(size_t(n), Set::None);

    auto finish = [&](bool solved) {
        for (Real& xi : x)
            xi = std::max(xi, Real(0)); // round-off only: the steps keep x >= 0
        result.solved = solved;
        return result;
    };

    auto wTolerance = [&](int i) { return 1e-10 * (std::abs(b[size_t(i)]) + scale * 1e-3); };

    // dx: x_d = 1, clamped components keep w_C = 0 (A_CC dx_C = -A_Cd), the rest 0.
    auto direction = [&](int d) {
        std::vector<int> C;
        for (int i = 0; i < n; ++i)
            if (set[size_t(i)] == Set::Clamped) C.push_back(i);
        const int m = int(C.size());
        std::vector<Real> M(size_t(m) * size_t(m)), r(static_cast<size_t>(m));
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < m; ++j)
                M[size_t(i) * size_t(m) + size_t(j)] = A(C[size_t(i)], C[size_t(j)]);
            r[size_t(i)] = -A(C[size_t(i)], d);
        }
        std::vector<Real> y = solveDense(std::move(M), std::move(r), m, tiny);
        std::fill(dx.begin(), dx.end(), 0);
        dx[size_t(d)] = 1;
        for (int i = 0; i < m; ++i)
            dx[size_t(C[size_t(i)])] = y[size_t(i)];
        for (int i = 0; i < n; ++i) {
            Real s = 0;
            for (int j = 0; j < n; ++j)
                s += A(i, j) * dx[size_t(j)];
            dw[size_t(i)] = s;
        }
    };

    // Largest step s along (dx, dw) that keeps x_C >= 0 and w_NC >= 0, stopping
    // when w_d reaches 0. Returns the blocking index, or -1 if nothing blocks.
    auto maxStep = [&](int d, Real& s) {
        s = std::numeric_limits<Real>::infinity();
        int j = -1;
        if (dw[size_t(d)] > tiny) {
            s = -w[size_t(d)] / dw[size_t(d)];
            j = d;
        }
        for (int i = 0; i < n; ++i) {
            if (set[size_t(i)] == Set::Clamped && dx[size_t(i)] < -1e-12) {
                Real t = -x[size_t(i)] / dx[size_t(i)];
                if (t < s) {
                    s = t;
                    j = i;
                }
            } else if (set[size_t(i)] == Set::NotClamped && dw[size_t(i)] < -tiny) {
                Real t = -w[size_t(i)] / dw[size_t(i)];
                if (t < s) {
                    s = t;
                    j = i;
                }
            }
        }
        s = std::max(s, Real(0));
        return j;
    };

    while (true) {
        // Pick the most negative w_d among the indices not yet handled.
        int d = -1;
        for (int i = 0; i < n; ++i)
            if (set[size_t(i)] == Set::None && w[size_t(i)] < -wTolerance(i) &&
                (d < 0 || w[size_t(i)] < w[size_t(d)]))
                d = i;
        if (d < 0) break;

        // drive-to-zero(d)
        while (true) {
            if (result.pivots++ >= maxPivots) return finish(false); // give up instead of freezing
            direction(d);
            Real s;
            int j = maxStep(d, s);
            if (j < 0) return finish(false); // unbounded direction: A is not positive semidefinite here
            for (int i = 0; i < n; ++i) {
                x[size_t(i)] += s * dx[size_t(i)];
                w[size_t(i)] += s * dw[size_t(i)];
            }
            if (j == d) {
                set[size_t(d)] = Set::Clamped;
                w[size_t(d)] = 0;
                break;
            }
            if (set[size_t(j)] == Set::Clamped) {
                set[size_t(j)] = Set::NotClamped;
                x[size_t(j)] = 0;
            } else {
                set[size_t(j)] = Set::Clamped;
                w[size_t(j)] = 0;
            }
        }
        // Indices with w >= 0 that were never touched stay at x = 0 (not clamped).
        for (int i = 0; i < n; ++i)
            if (set[size_t(i)] == Set::None && w[size_t(i)] >= -wTolerance(i))
                set[size_t(i)] = Set::NotClamped;
    }
    return finish(true);
}

} // namespace ncotb
