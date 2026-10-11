# Design notes

## Why the old contact solver froze

Reading `cids/src/linalg/linearcomplementarysolver.cpp` and `rigidbodysystem.cpp`:

1. `driveToZero()` loops `while(true)` with no iteration cap; degenerate pivots (step `s = 0`)
   can cycle forever. This is the most likely cause of the freeze.
2. `maxstep()` can return `j = -1` with `s = inf` when `dy_d <= 0` (happens with redundant
   contacts), and then `m_clamped[-1]` is undefined behaviour.
3. Redundant contacts (four coplanar points of a box resting on a plane) make `A_CC` singular.
   The failed solve result (`valid`) is ignored.
4. The absolute tolerance `0.01` used to zero `x, y, dx, dy` depends on the scale of masses
   and forces, so it can zero genuinely meaningful values.
5. The collision/impulse handling in `Integrator::safeStep` is commented out, so nothing
   corrects penetration drift, and the LCP gets inconsistent input.
6. RK4 runs collision detection and the LCP inside every derivative evaluation. The contact
   forces are discontinuous, which defeats the higher-order integrator.

## New approach

Velocity-level time stepping, as in Box2D/Bullet ("Sequential Impulses", E. Catto):
forces → velocities → solve constraints as impulses (fixed iteration count, so it can't
freeze) → positions. Friction, joints and contacts become constraint rows of one kind.
Baraff's acceleration-level LCP will be added later as an optional solver for comparison.

## Integration

Semi-implicit Euler for x and P, L. For orientation, the naive `q += h/2 [0,ω] q` gains
energy fast for tumbling bodies (3.5× in 10 s in a test), because ω = I⁻¹(q) L changes
during the step. We instead rotate with the exponential map using ω at the midpoint
orientation, which keeps energy within 0.1% over the same test.
