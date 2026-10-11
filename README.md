# ncotb — rigid body simulation

A from-scratch rebuild of the old `cids` project (Baraff's SIGGRAPH '97 notes), in C++20 with
raylib + Dear ImGui. See [docs/notes.md](docs/notes.md) for design notes.

## Build & run

```sh
brew install cmake ninja          # once
cmake --preset release            # first configure downloads raylib, imgui, rlImGui, doctest
cmake --build --preset release
./build/tests/physics_tests       # or: ctest --preset release
./build/app/ncotb
```

Presets (`CMakePresets.json`):

| Preset    | Directory      | Use |
|-----------|----------------|-----|
| `debug`   | `build-debug/` | debugging (VS Code launch configs, clangd) |
| `release` | `build/`       | running at full speed (RelWithDebInfo) |
| `asan`    | `build-asan/`  | Debug + AddressSanitizer/UBSan for our code |

Controls: drag to orbit, wheel to zoom, Space pause, S single step, R reset.

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
  collision/    sphere/box/plane contact manifolds
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
- [ ] 3. Contacts: collisions + resting contact (Sequential Impulses / PGS, plus Baraff's LCP for comparison)
- [ ] 4. Friction
- [ ] 5. Springs, dampers, joints (ball, hinge, slider)
- [ ] 6. Interaction: mouse picking/dragging, impulses, spawning
- [ ] 7. Heightfield terrain
- [ ] 8. Drivable vehicle
