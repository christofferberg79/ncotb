# ncotb — rigid body simulation

A from-scratch rebuild of the old `cids` project (Baraff's SIGGRAPH '97 notes), in C++20 with
raylib + Dear ImGui. See [docs/notes.md](docs/notes.md) for design notes.

## Build & run

```sh
brew install cmake ninja          # once
cmake -B build -G Ninja           # first configure downloads raylib, imgui, rlImGui, doctest
cmake --build build
./build/tests/physics_tests       # or: ctest --test-dir build
./build/app/ncotb
```

Controls: drag to orbit, wheel to zoom, Space pause, S single step, R reset.

## Layout

```
physics/   simulation library (no graphics dependency)
  math/    Vec3, Mat3, Quat
  body/    Shape, RigidBody
  integration/  Euler, Midpoint, Heun, RK4 for any OdeSystem
  world.*  World: owns bodies, fixed-step integration
app/       raylib + ImGui front-end, scenes
tests/     doctest unit tests
docs/      notes
```

## Milestones

- [x] 0. Skeleton: CMake, raylib, ImGui, doctest, orbit camera
- [x] 1. Free rigid body (x, q, P, L), inertia, gravity, Dzhanibekov effect
  - selectable integrators: Euler, Semi-implicit Euler, Midpoint, Heun, RK4
- [ ] 2. Collision detection: sphere/plane/box, contact manifolds
- [ ] 3. Contacts: collisions + resting contact (Sequential Impulses / PGS, plus Baraff's LCP for comparison)
- [ ] 4. Friction
- [ ] 5. Springs, dampers, joints (ball, hinge, slider)
- [ ] 6. Interaction: mouse picking/dragging, impulses, spawning
- [ ] 7. Heightfield terrain
- [ ] 8. Drivable vehicle
