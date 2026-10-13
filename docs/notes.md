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

**Known limitation:** no continuous collision detection, so a fast body can pass through a
thin box.

## Friction (milestone 4)

Each contact point gets two friction rows along tangents t1, t2. The tangents depend only on
the normal, so a resting contact keeps the same ones every step. The friction rows try to
make the sliding velocity zero, with the friction impulse limited to the disc
|f| ≤ μ·λ_n (a circular Coulomb cone). The coefficient is per body, combined as
√(μ_a μ_b), as in Box2D.

- **Sequential impulses:** within each pass, a point's friction is solved first and its
  normal row last, so non-penetration is what holds best. The friction impulse is
  warm-started too. It is stored as a world-space vector and projected onto the new tangents.
- **Baraff LCP:** Baraff's pivoting method handles the normal impulses only. With friction
  the solver alternates, 4 rounds by default: exact LCP for the normals with friction held
  fixed, then friction passes with the normals held fixed ("staggered projections",
  Kaufman et al. 2008).
- After the last pass, friction is clamped onto the cone once more, because the normal
  impulse may have shrunk after friction was last solved.
- The restitution pass leaves friction alone. The relax pass includes it.

**A bug found by the rolling test.** The relax pass used to turn the contact offsets
r_a, r_b along with the bodies. A sphere's contact point stays at the bottom while the
sphere turns, so the turned offset gave the normal impulse a lever arm. That made a
braking torque, like rolling resistance: the ball rolled downhill 8% too slowly. Now the
turned offsets are only used to measure how the gap changed, as in Box2D. The lever arms
stay as they were at the start of the step.

Results (both solvers, 240 Hz, 4 substeps):

| Test | Measured | Theory |
|---|---|---|
| box on a 30° ramp, μ = 0.2 | a = 3.2059 m/s² | g(sin θ − μ cos θ) = 3.2059 |
| box on a 20° ramp, μ = 0.5 | stays put | tan 20° = 0.36 < μ |
| ball rolling down 20° | a = 2.3966 m/s² | 5/7·g·sin θ = 2.3966 |
| ball skidding at 4 m/s, no spin | rolls at 2.8571 m/s | 5/7·v₀ = 2.8571 |
| box sliding at 3 m/s, μ = 0.4 | stops after 1.1452 m | v₀²/(2μg) = 1.1468 |
| 5-box stack, 10 iterations × 1 substep | drift < 1 mm in 5 s | (12 mm without friction) |

There's no rolling resistance, so a ball on flat ground rolls forever, which is correct for
rigid bodies.

## Cannon vs brick wall (stress scene)

147 bricks in running bond and a 30 kg cannonball at 25 m/s. The wall alone has about
1600 contact points, because bricks touch their neighbours on every side. Release build,
240 steps/s:

- **Collision detection** takes about 0.3 ms per substep. A bounding-sphere test now skips
  far-apart pairs before the exact test; before, every one of the ~11,000 pairs went
  through the full box–box test.
- **Sequential impulses** take about 1.2 ms per substep. 2 substeps keep the scene real-time
  (3.4 ms per 4.2 ms step); 4 would not (6.0 ms). The scene sets its own default.
- **Baraff LCP** builds a dense matrix, and each pivot solves a dense system, so the cost
  grows roughly with the 4th power of the number of contact points. That's fine for a few
  dozen points, but hopeless for 1600: it would hang the app. Above `lcpMaxRows` (300) a step
  uses sequential impulses instead, and the panel says so.

The cannon is drawn only and isn't a physics body. Scenes can now set their own substeps,
camera, overlays, an action (here "Fire", key F, run automatically at t = 1 s) and extra
drawing.

## Tunnelling: speculative contacts that grow with speed

With a fixed 2 cm contact margin, a body moving farther than the margin plus the
obstacle's thickness in one step could skip over it. At 30 steps/s with 2 substeps the
25 m/s cannonball moves 42 cm per substep, and it passed straight through the 30 cm wall.

**The fix** (`World::detectCollisions(h)`):

- **Wider margin for fast pairs.** Each body gets a speed bound, |v| + |ω|·r (r is the
  bounding radius; spheres spinning in place don't move their surface, so 0 for them).
  A pair's margin is 2 cm + h · (bound a + bound b), which is everything they could close
  within the substep. The bounding-sphere rejection grows by the same amount. Slow pairs
  are as before.
- **Points they actually reach.** A point is kept only if its gap is at most
  2 cm + h · (closing speed at that point along the normal). Contacts moving sideways,
  like a box sliding along the ground, don't get extra points.
- **Time of impact for distant pairs.** Far from a box, the nearest feature of a ball that
  is passing *beside* the box is an edge, with a normal that points almost straight at the
  ball. A contact on that edge stops the ball although it would never get there: a "ghost
  collision", the classic weakness of speculative contacts. So for a pair farther apart
  than 2 cm, copies of the bodies are moved along their velocities by conservative
  advancement: move forward by distance / (speed bound), which can't overshoot, and
  repeat until the gap is under 2 cm (a hit) or the substep is over (no contact). The
  contact is taken from where they meet and moved back to the current positions, with
  the gap widened by what closes until then. `distance()` in collide.cpp gives the
  distance needed for this. It is exact except for box–box, where it is the largest
  separation over the 15 SAT axes. That is a lower bound, which is what conservative
  advancement needs.
- The solver didn't change: a contact with gap g may close at most g/h, so the body
  arrives at the surface by the end of the step. The restitution pass then bounces it
  as usual.

**Cost.** For the brick wall and cannon in a release build, the time per step doesn't
change measurably: about 5 ms while settling, about 4 ms after the shot, at 30 to
240 steps/s. Only pairs with a fast body pay anything. A worst case: 200 bodies at
30 m/s in a closed room at 30 steps/s. Before the fix, 138 of them escaped through the
walls (0.1 ms per step). After it, none escape (0.6–0.9 ms per step, partly because they
stay in the room and keep colliding).

**Limits.** The sweep follows straight lines at the current velocity, without gravity
or rotation along the path. The solver also adds velocity during the step that the
detection didn't know about, for example a brick hit by the ball. A body accelerated
like that within one step can still overlap the next body by up to v·h. The contact
then pushes it out gently, so it doesn't tunnel unless that is more than the next
body's thickness.

## Springs, dampers and joints (milestone 5)

**Springs** (`physics/forces/spring.h`) are forces, not constraints:
f = (k(|d| − L₀) + c·(v_rel·d̂))·d̂ between two attachment points. They are re-evaluated in
every substep, and work with every integrator, including RK4 in the free-motion scenes.
Tested: equilibrium stretch m·g/k, period 2π√(m/k) with both semi-implicit Euler and RK4,
and no overshoot with critical damping c = 2√(km).

**Joints** (`physics/joints/joint.*`) are velocity-level constraint rows, solved by
sequential impulses in the same passes as the contacts, joints first in each pass.
They get the same soft step: Baumgarte position correction (capped at 2 m/s) while solving
velocities, then a relax pass without it.
- **Ball joint**: anchor points coincide. 3 rows solved together as one 3×3 block with
  K = (1/m_a + 1/m_b)·I − [r_a]× I_a⁻¹ [r_a]× − [r_b]× I_b⁻¹ [r_b]×.
- **Hinge**: a ball joint, plus 2 angular rows that keep the axes aligned (a 2×2 block),
  plus optional angle limits and a motor about the axis. The limits work like speculative
  contacts: the joint may approach a limit by the remaining angle, and is pushed back past
  it. The motor drives the relative spin toward a target speed with at most the maximum torque.
- **Slider**: relative rotation locked (3×3 block), no motion across the axis (2 rows), and
  optional limits and motor along the axis.
- `World::ground()` is a static anchor for attaching joints and springs to the world. Bodies
  connected by a joint don't collide with each other (like Box2D's default
  `collideConnected = false`): a chain's links touch at their joints.
- **With the Baraff LCP:** joints are equalities, and the pivoting method handles
  inequalities. So in LCP mode joints are solved like friction, in the passes between the
  exact LCP solves for the contact normals.
- The free-motion integrators (Euler, RK4, ...) ignore joints, as they ignore contacts.

Tested:
- pendulum periods within 0.3% (a ball on a ball joint, and a bar on a hinge as a physical
  pendulum, including the finite-amplitude correction);
- a hinge removing off-axis spin;
- a chain's top joint carrying the whole weight of 10 links;
- a door stopping at its 90° limit;
- a motor's spin-up rate τ/I and its target speed;
- a slider following its axis, stopping at its limit, and driven back up by its motor;
- a hinged flap resting on a box (joints and contacts together).

A test that was wrong taught something: spinning a door about its *centre* when the hinge is
at its edge gives 3·I_cm/I_hinge = 0.75 rad/s after the first step, not 3 rad/s. Angular
momentum about the hinge is conserved.

**Two lessons from the demo scenes:**
- **Newton's cradle with 1 mm gaps doesn't work.** The gaps are smaller than the 2 cm contact
  margin, so in the step of the first impact all five balls are in (speculative) contact.
  The solver resolves them together as one inelastic chain: the last ball gets 1.6 instead
  of 2.3 m/s and the others keep moving. Real cradles work because the collisions happen
  one after another. With 3 cm gaps each impact is a separate two-ball collision: the last
  ball gets 98% of the ideal speed and the others nearly stop (tested).
- **A box standing on four compression springs tips over.** A spring only pushes along its
  own line and has no sideways stiffness, so a tilted box makes the springs buckle. Real
  suspensions guide their springs. The demo crate hangs from its springs instead and settles
  exactly at the predicted height.

## Interaction (milestone 6)

**Ray casting** (`physics/collision/raycast.*`): sphere (a quadratic equation), box (the slab
test in box coordinates: intersect the three pairs of parallel face planes) and plane (a
half-space). A ray that starts inside a body hits it at distance 0. `World::raycast` returns
the nearest hit, skipping static bodies unless asked, with a bounding-sphere rejection first.

**Grabbing** uses a mouse joint (`MouseJoint`, Catto): a *soft* point constraint between a
point on the body and the cursor's target. A spring k and damper c,

    k = m (2π f)²,   c = 2 m ζ (2π f)    (f = 5 Hz, ζ = 0.7, m = the body's mass)

become a constraint with compliance γ = 1/(h(c + hk)) and position feedback β = hk/(c + hk),
solved like the other joints:

    λ = −(K + γI)⁻¹ (v_point + β/h · C + γ·λ_accumulated)

Two safety measures:
- **The force is capped** at 30 × the body's weight, so a drag can't fling a body or tear it
  out of its joints. Tested: with the cap at half the weight, a held box sinks at g/2.
- **The held body gets angular damping** (10/s). This is an interaction choice, not physics:
  the spring only damps the grabbed *point*, so a body would keep swinging around it like a
  pendulum without air. With 3/s a box jerked 2.6 m still swung after 5 s, because damping
  the spin about the centre only works through I_cm / I_pivot ≈ 40% of it. 10/s settles it in
  about 3 s.

The target moves on the plane through the grabbed point that faces the camera; scrolling
moves that plane nearer or farther.

**Pushing** applies J = m · Δv along the view ray at the hit point. Hitting a ball off-centre
along a downward ray puts part of it into spin and into the ground: a 3 m/s push gave a ball
2.0 m/s.

**Removing a body** (`World::removeBody`) also removes every joint, spring, contact and
warm-start entry that refers to it. Otherwise they'd hold dangling pointers (checked under ASan).

The app reads raylib's input into an `Input` struct once per frame, so the interaction code can
also be driven by a script. A scripted run under ASan exercises grabbing, pushing, spawning,
deleting and "remove spawned".

## Terrain (milestone 7)

`Heightfield` is a grid of heights in the body's xz-plane, centred on the body. Each cell is
split into two triangles along the same diagonal, and everything (collision, ray casts and
the drawn mesh) uses those exact triangles. `Heightfield::hills()` makes smooth random hills
from value noise: random values on a coarse lattice, blended with smoothstep, over four
octaves. A seed gives the same hills every time. Terrain bodies are static, like planes.

**More than one normal per pair.** A box lying across a ridge touches triangles that face
different ways, but a `ContactManifold` has a single normal. `collide()` now has a version
that appends a list of manifolds. Terrain pairs group their points by normal (within about
2°) into manifolds of at most four points; every other pair still gives at most one. The
speculative sweep from the tunnelling fix works on these lists too. Warm starting is keyed by
the contact point on body a, so it doesn't care how the points are grouped.

**Sphere–terrain.** The closest point on each triangle near the sphere. If the sphere's
centre is under the surface (it fell in, or took a huge step), it is pushed straight out
through the triangle it is under.

- *Internal edges:* a ball rolling across flat terrain also finds the shared edge of the
  next triangle as a closest point, with a normal tilted back towards it. As a speculative
  contact that edge catches the ball, the classic "internal edge" bump. So an edge or
  corner point only counts if no triangle sharing it is closer: on flat ground the face
  under the ball always is. On a real ridge both triangles have the same closest point on
  the shared edge, and it is kept once. A test checks that a ball rolling over many edges
  and diagonals keeps its speed to 0.1% and its height to 1 mm.

**Box–terrain.**

- *Corners:* each corner's gap to the plane of the triangle under it.
- *Terrain vertices poking up into the box's bottom face:* so that a board resting on four
  spikes stays up on them instead of sinking until its corners touch the ground. Only the
  face that looks down the most is tested. At first all faces were, and a terrain vertex
  level with the bottom of a sliding box, just ahead of it, pushed back on its front face
  and tripped it up.
- *Not handled:* box edges against terrain edges (a box edge resting across a sharp
  ridge), and corners beside a steep wall that rises next to them. Neither matters on
  smooth hills. Over sharp ridges the speculative distance for boxes can be slightly too
  large, so a fast box can overlap a ridge a little and is then pushed out gently.

**Ray casts** walk the cells under the ray in order (a 2D DDA) and test their two triangles,
from above only. Grab, drop and hover therefore work on the terrain.

**Found on the way: rolling spheres sank into a phantom gap.** The relax pass recomputes each
contact's depth after the position update by moving the contact point with each body,
turned with its rotation. A sphere's surface doesn't move when it turns about its centre.
But a rotated contact point on a rolling ball rises off the ground by r (1 − cos ωh), so the
relax pass let the ball fall into that gap at about v²h / 2r. At 6 m/s and 1 substep that is
0.3 m/s. At 20 m/s it would trip the 0.5 m/s bounce threshold, and the ball (or a car wheel)
hopped 13 cm. Now only the centre's motion counts for spheres. A regression test rolls a
ball at 20 m/s on the plane.

**Drawing.** The mesh is uploaded once per heightfield and cached. The renderer has no
lights, so colours are shaded on the CPU from smooth vertex normals and a fixed sun: grass
low down, earth, rock on the heights and steep slopes, snow on the tops. Faint lines every
5 m help to read the slopes. A cached mesh is freed once no body uses its heightfield.

**Cost.** 50 bodies on 80 × 80 m of hills: 0.48 ms per step with 4 substeps, against 0.39 ms on
the plane (release build).

## The RC truck (milestone 8)

An RC monster truck at about 1:10 scale (0.5 m long, 3 kg), built only from rigid bodies,
joints and springs (`physics/vehicle/car.*`), so driving it exercises the whole engine. Per
corner:

- a **lower arm**, hinged to the chassis along the car, and hinged to an **upright** at its
  outer end;
- a **camber link** (a rod, the new `DistanceJoint`) from the chassis to the top of the
  upright, parallel to the arm and as long, so the upright moves up and down without tilting;
- a **shock** (spring + damper) from a tower on the chassis to the lower arm;
- at the front a **steering knuckle**, hinged to the upright about a vertical kingpin; its
  hinge motor is the steering servo;
- the **wheel**, hinged to the knuckle (front) or upright (rear) about the axle. Wheels are
  spheres for collision, drawn as tyres.

All parts share a collision group (new: `RigidBody::collisionGroup`), so they don't collide
with each other.

**Tuning by frequency.** The shocks are set from a ride frequency (2.5 Hz) and a damping
ratio (0.6), not raw stiffness. The rest length is found by virtual work so that the truck
stands level with the arms horizontal: per radian the arm turns, every force (the tyre's
load, the weight of the arm, upright, knuckle and wheel, and the shock) does work, and the
shock's must cancel the rest. The lever ratio turns the wheel rate into the spring rate.

**Driving.**

- *Motor:* each wheel gets torque T_max · throttle · (1 − ω/ω_max) minus a small drag. The
  reaction goes to the chassis, where the motor sits. Equal torque on every wheel acts like
  open differentials. Top speed is where the motor's torque meets its drag: 10.4 m/s
  (37 km/h).
- *Brake:* the axle hinges' motors with target speed 0 and limited torque. That's an implicit
  constraint, so a brake can't overshoot and spin the wheel backwards.
- *Steering:* each knuckle's servo drives its angle to a target, with Ackermann geometry: both
  front wheels point at one turning centre on the line of the rear axle, so the inner one
  turns more.

The app adds WASD and Space, easing the steering in and out, and a chase camera that follows
the chassis and swings round behind it.

### What went wrong on the way

1. **Single swing arms "jacked".** The first version had one arm per wheel, hinged at hub
   height. The tyre touches the ground a wheel radius below the hinge, so a sideways force on
   the tyre turns the arm just as the load does. With rigid bodies and sticking friction,
   how the weight splits between the normal force and that sideways grip is undetermined,
   and the solver drifted to the friction limit. At full throttle the rear tyres spent their
   whole grip pushing the wheels apart (±8 N sideways at μN ≈ 8.4 N) and spun, and the truck
   pulled a wheelie. The fix is the classic one: a parallelogram (double wishbone), along
   whose arms a sideways tyre force simply pulls without loading the spring. A test drives
   at full throttle and checks it goes straight and reaches top speed.
2. **Redundant constraints slowed the solver down.** The first double wishbone used a
   hinged upper arm: four hinges around a closed loop, five constraints each, many of them
   doing the same thing. Parked, the truck kept rocking at 0.2–0.3 rad/s unless the solver
   ran 200 iterations. With the upper arm replaced by a rod (one constraint), each corner has
   exactly one way to move and no redundancy, and 50 iterations settle it. The scene uses
   60 (0.33 ms per step with 4 substeps). Below about 50, a self-sustaining rocking remains.
   Mass ratios turned out not to matter: heavier wheels or a lighter chassis changed little.
3. **Not it: coupled friction directions.** Solving each contact's two friction directions
   as one 2×2 block looked promising for free-spinning wheels, but it changed nothing for the
   truck. It also made sliding friction wrong: the block's impulse isn't opposite to the
   slip, so a sliding box got 3% too little friction. Reverted.

### Limits

- **Tyres are rigid spheres.** They grip the ground at a point, with no flex. Sideways
  forces between the left and right tyres can stay locked in (seen at 2–7 N) and eat into
  the grip available near its limit. Real tyres flex and "roll out" such forces. A tyre
  model (sideways stiffness, slip angle, relaxation as it rolls) would fix that. Sideways,
  the spheres also collide wider than the drawn tyres.
- **Stiff for the solver.** 16 bodies and 22 joints in tight loops need 60 iterations. A
  direct solver for the joint tree (Featherstone) would be the next step if that becomes a
  problem.

**Tests** (`tests/test_vehicle.cpp`):

- at rest: level, arms horizontal, still, joints closed;
- after a drop: settles, joint drift under 5 mm;
- full throttle: straight, reaches the computed top speed;
- braking: stops within the friction-limited distance;
- steering: Ackermann angles and the turning circle (within 15%);
- 10 s of loops over rough terrain: in one piece, no wheel in the ground;
- deterministic.
