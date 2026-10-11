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

## Collision detection (milestone 2)

`physics/collision/collide.*`. A `ContactManifold` holds one unit normal, pointing from
body b towards body a (Baraff's convention), and up to 4 points. Each point sits midway
between the two surfaces and has a penetration depth, which is negative when the shapes are
apart but closer than the margin. `World::detectCollisions()` checks every pair brute force
and keeps the dynamic body as a, so normals point away from static geometry.

| Pair          | Method |
|---------------|--------|
| sphere–sphere | centre distance |
| sphere–plane  | centre height above the plane |
| sphere–box    | closest point on the box, in box coordinates; centre inside → nearest face |
| box–plane     | the 8 corners below the plane, reduced to 4 |
| box–box       | separating axis test over the 15 axes (3 + 3 faces, 9 edge×edge) |

Box–box picks the axis with the least penetration. Face axes are preferred unless an edge
axis is clearly better (`s > 0.95·s_face + 0.005`), so the contact type doesn't flicker
between steps. For a face contact the most anti-parallel face of the other box is clipped
against the reference face's 4 side planes (Sutherland–Hodgman), and the points below the
reference face are kept. An edge–edge contact gives one point, from the closest points
between the two edges.

When there are more than 4 points they are reduced to: the deepest, the one farthest from it,
and the two that span the largest triangles on either side of that line. This keeps the
contact area, which matters for stable resting contact.

## Contact response (milestone 3)

`physics/solver/`. Each (sub)step of semi-implicit Euler:

1. forces → velocities (`P += F h`, `L += τ h`)
2. detect contacts at the current positions (2 cm margin)
3. solve the contact impulses, which changes `P` and `L`
4. velocities → positions

Each contact point is one constraint row at the velocity level:

    v_n = n · (v_a + ω_a × r_a − v_b − ω_b × r_b) ≥ target,   λ ≥ 0,   λ (v_n − target) = 0

Two solvers find the impulses λ:

- **Sequential impulses** (Catto): projected Gauss–Seidel. It solves one row at a time and
  clamps the *accumulated* impulse to ≥ 0. Warm starting from the previous step's impulses
  matches points within 2 cm on body a. It has a fixed cost and can never freeze.
- **Baraff LCP**: builds A = J M⁻¹ Jᵀ (rows are coupled through shared bodies) and
  b = v_n − target, then solves the LCP exactly with the pivoting method from "Fast Contact
  Force Computation". Fixes compared to the old cids code:
  - a pivot limit (10n + 100);
  - a check for "no blocking index" instead of indexing with −1;
  - tolerances relative to the matrix scale instead of an absolute 0.01;
  - a 10⁻⁹ regularization, so redundant contacts (4 coplanar points) don't make A singular;
  - a linear solve that tolerates singular sub-matrices.

  It's tested on 300 random problems, including singular ones, and with a box resting on a
  face.

The target velocity:
- **touching** (0 ≤ depth ≤ slop): 0
- **penetrating** beyond the slop: Baumgarte push-out, β/h · (depth − slop), β = 0.2,
  capped at 1 m/s
- **apart but within the margin** (depth < 0): depth/h, so the body may close the gap but not
  pass it ("speculative contacts", which prevent overshoot and jitter)
- **restitution**: a second pass, only on rows that got an impulse while approaching faster
  than 0.5 m/s, with target −e·v_n. This works with speculative contacts too.

Restitution uses v_n from *before* the step's gravity was added. Using the velocity after
gravity made an elastic ball bounce 1.8% higher at 240 Hz, because it reversed v + g·h
instead of v. Now it returns to its drop height exactly.

**Substeps beat iterations.** In a frictionless stack of 5 boxes, an unconverged Gauss–Seidel
solve spreads the impulse unevenly over a face's 4 points. The tiny net torque tilts the boxes
by about 10⁻⁴ rad, and they slide sideways (nothing resists that until friction arrives).
Drift after 5 s:

| Iterations × substeps | 10 × 1 | 40 × 1 | 10 × 4 | 80 × 1 |
|---|---|---|---|---|
| top box drift | 12 mm | 1.4 mm | 0.7 mm | 0.03 mm |

The exact LCP solver doesn't drift at all. The app uses 4 substeps in contact scenes.

### Push-out without launching (soft step)

Plain Baumgarte gives overlapping bodies a separation *velocity*, and that velocity stays
after the overlap is gone. A sphere started 5 cm into the floor flew off at 8.6 m/s with
4 substeps, because the push-out scales with 1/h. Now each (sub)step has three phases, as
in Box2D v3's "soft step":

1. solve the velocities *with* the push-out (capped at 1 m/s)
2. move the bodies, which removes some overlap
3. **relax**: solve again at the new positions *without* the push-out (target 0, or gap/h
   for contacts still apart), then the restitution pass

The push-out now moves bodies but leaves no velocity behind. The same sphere leaves the floor
at 0 m/s, rising 4.2 mm per step until it rests on the surface. During relax the contact
offsets r_a, r_b move with the bodies, and the depth is updated by how far the bodies moved.

**Overlapping starts are still best avoided.** Some have no valid answer at all. For
example, a 1 m ball between walls 95 cm apart (tested):
- The exact LCP keeps the ball centred.
- Sequential impulses solve one wall after the other, so the last one wins and the ball
  shifts 2 cm (until that side's overlap is down to the slop).

Both stay calm, with no jitter or launch. A stack started 3 cm into itself settles at the
right heights. With sequential impulses a little push-out speed (≤ 0.3 m/s) survives a
few steps, because 10 relax iterations don't fully converge in a coupled stack.

**Known limitations:** no friction yet, so spheres roll and slide forever; no continuous
collision detection, so a fast body can pass through a thin box.
