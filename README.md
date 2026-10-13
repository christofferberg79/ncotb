# ncotb — rigid body simulation

A from-scratch rebuild of the old `cids` project (Baraff's SIGGRAPH '97 notes), in C++20 with
raylib + Dear ImGui. See [docs/notes.md](docs/notes.md) for design notes.

## Build & run

```sh
brew install cmake ninja          # once
cmake --preset release            # first configure downloads raylib, imgui, rlImGui, doctest
cmake --build --preset release
ctest --preset release            # unit tests + formatting check
./build/app/ncotb
```

Presets (`CMakePresets.json`):

| Preset    | Directory      | Use |
|-----------|----------------|-----|
| `debug`   | `build-debug/` | debugging (VS Code launch configs, clangd) |
| `release` | `build/`       | running at full speed (RelWithDebInfo) |
| `asan`    | `build-asan/`  | Debug + AddressSanitizer/UBSan for our code |

Our code (physics, app, tests) builds with `-Wall -Wextra -Wpedantic -Wshadow -Werror`, so a
warning fails the build (`-DNCOTB_WERROR=OFF` to relax that locally). `ctest` also runs a `format`
test that fails if a file differs from `.clang-format` (fix with the "format all" task).

Controls:

| Input | Action |
|---|---|
| drag | orbit |
| Shift + drag, or right-drag (two-finger click + drag) | pan |
| scroll (two fingers) | zoom |
| arrow keys (Shift: faster), Q / E | move the view's centre over the ground, down / up |
| C | reset the view (R restarts the scene and keeps the view) |
| **click-drag on a body** | **grab it** (a soft spring to the cursor; scroll: nearer / farther) |
| **Option + click on a body** | **push it** along the view |
| **1 / 2 / 3** | **drop a box / drop a ball / throw a ball** at the cursor |
| **Backspace** | **delete the body under the cursor** |
| Space / S / R / F | pause / single step / reset scene / scene action |

### VS Code

Recommended extensions: clangd, CodeLLDB, CMake Tools (see `.vscode/extensions.json`).
⇧⌘B builds the debug preset; the Run and Debug panel has configs for the app and the tests
(also filtered by test name, and under ASan/UBSan). clangd uses Apple's `/usr/bin/clangd`,
formats on save with `.clang-format`, and shows the clang-tidy checks in `.clang-tidy`.

## Layout

```
physics/   simulation library (no graphics dependency)
  math/    Vec3, Mat3, Quat
  body/    Shape, RigidBody
  integration/  Euler, Midpoint, Heun, RK4 for any OdeSystem
  collision/    sphere/box/plane contact manifolds, ray casts
  solver/       contact solvers with friction: sequential impulses, Baraff LCP
  joints/       ball, hinge (limits, motor), slider (limits, motor), mouse (soft)
  forces/       springs with dampers
  world.*  World: owns bodies, fixed-step integration
app/       raylib + ImGui front-end, scenes
tests/     doctest unit tests
docs/      notes
```

## Milestones

- [x] 0. Skeleton: CMake, raylib, ImGui, doctest, orbit camera
- [x] 1. Free rigid body (x, q, P, L), inertia, gravity, Dzhanibekov effect
  - selectable integrators: Euler, Semi-implicit Euler, Midpoint, Heun, RK4
- [x] 2. Collision detection: sphere/plane/box, contact manifolds
- [x] 3. Contacts: collisions + resting contact (Sequential Impulses / PGS, plus Baraff's LCP for comparison)
- [x] 4. Friction
- [x] 5. Springs, dampers, joints (ball, hinge, slider)
- [x] 6. Interaction: mouse picking/dragging, impulses, spawning
- [x] 7. Heightfield terrain
- [ ] 8. Drivable vehicle
