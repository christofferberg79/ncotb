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

## Choosing an integrator

The app lets you pick Euler, Semi-implicit Euler, Midpoint, Heun or Runge-Kutta 4
(`physics/integration/integrator.*`; `World` is an `OdeSystem` with Baraff's 13 values
per body: x, q, P, L).

Energy error after 2 s of "Thrown boxes and balls" at 240 steps/s:

| Integrator          | E − E₀       | Why |
|---------------------|--------------|-----|
| Euler (explicit)    | +2.4 J       | moves with the start-of-step velocity: bodies end up too high, +m·g²·h/2 per second |
| Semi-implicit Euler | −2.4 J       | moves with the end-of-step velocity: bodies end up too low, −m·g²·h/2 per second |
| Midpoint, Heun      | ~10⁻⁶ J      | 2nd order: exact for a parabola, small error from the tumbling |
| Runge-Kutta 4       | ~10⁻¹⁰ J     | 4th order |

Convergence orders (halving h divides the error by 2, 4, 4 and 16) are checked in
`tests/test_integrators.cpp`.

So why is semi-implicit Euler still the default? Contacts and joints are not smooth: impulses
make velocities jump in the middle of a step, which breaks RK4's assumptions. The old cids code
ran collision detection and the contact LCP inside each of RK4's four derivative evaluations,
which is part of why it froze. Semi-implicit Euler lets the constraint solver sit between the
velocity update and the position update, so a resting box gets v = 0 *before* it moves. It is
also symplectic, so for springs and pendulums the energy error oscillates instead of drifting.
The modern way to gain accuracy with contacts is substepping. How the selector behaves in
contact scenes will be decided at milestone 3.
