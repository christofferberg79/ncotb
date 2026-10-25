# ncotb — rigid body simulation

A from-scratch rebuild of the old `cids` project (Baraff's SIGGRAPH '97 notes), in C++20 with
raylib + Dear ImGui. See [docs/notes.md](docs/notes.md) for design notes.

## Download

Ready-to-run builds are on the [Releases](https://github.com/christofferberg79/ncotb/releases)
page; nothing else needs to be installed.

- **macOS** (11 or later, Apple Silicon and Intel): open `ncotb-…-macos-universal.dmg` and drag
  ncotb to Applications. The app isn't notarised by Apple, so the first launch is blocked: open
  System Settings → Privacy & Security and click "Open Anyway" (or run
  `xattr -dr com.apple.quarantine /Applications/ncotb.app`). Snapshots are saved to
  `~/Library/Application Support/ncotb`.
- **Windows** (10 or later, x64): unzip `ncotb-…-windows-x64.zip` and run `ncotb.exe`. If
  SmartScreen says "Windows protected your PC", click "More info" → "Run anyway". Snapshots are
  saved to `snapshots\` next to the exe.

## Build & run

Needs CMake ≥ 3.25, Ninja, a C++20 clang, and git (the first configure downloads raylib, imgui,
rlImGui and doctest).

### macOS

```sh
xcode-select --install            # once: Apple clang, clang-format, clangd, lldb
brew install cmake ninja          # once
cmake --preset release
cmake --build --preset release
ctest --preset release            # unit tests + formatting check
./build/app/ncotb
```

### Windows

The toolchain is [LLVM-MinGW](https://github.com/mstorsjo/llvm-mingw): clang, libc++,
clang-format, clangd and lldb in one package, so it works like the macOS setup. Install it once
from PowerShell, then open a new terminal so PATH is updated:

```powershell
winget install -e --id MartinStorsjo.LLVM-MinGW.UCRT
winget install -e --id Kitware.CMake
winget install -e --id Ninja-build.Ninja
```

If winget hangs on "Extracting archive…", have it extract with `tar` instead of the (very slow)
shell API: run `winget settings` and add
`"installBehavior": { "archiveExtractionMethod": "tar" }`.

Then, in PowerShell from the repository root:

```powershell
cmake --preset release
cmake --build --preset release
ctest --preset release            # unit tests + formatting check
.\build\app\ncotb.exe
```

The executables link the C++ runtime statically, so `ncotb.exe` runs without the toolchain on
PATH (except the `asan` preset's, which need the sanitizer runtime DLL from LLVM-MinGW's `bin`).
Run it from the repository root, since snapshots are saved to `snapshots/` in the working
directory. Visual Studio's MSVC isn't supported: the presets select `clang`/`clang++`, and the
warning flags are GCC/clang style.

### Presets

Presets (`CMakePresets.json`):

| Preset    | Directory      | Use |
|-----------|----------------|-----|
| `debug`   | `build-debug/` | debugging (VS Code launch configs, clangd) |
| `release` | `build/`       | running at full speed (RelWithDebInfo) |
| `asan`    | `build-asan/`  | Debug + AddressSanitizer/UBSan for our code |
| `package` | `build-package/` | release packages: `cpack --preset package` after building |

The `package` preset builds a universal (arm64 + x86_64) `ncotb.app`, ad-hoc signed, in a `.dmg`
on macOS, and a `.zip` with a console-less `ncotb.exe` on Windows.

### Releases (CI)

`.github/workflows/build.yml` builds, tests and packages on macOS and Windows for every pushed
branch (the packages are attached to the run as artifacts). Pushing a
version tag also publishes a GitHub release with both packages:

```sh
git tag v0.1.0
git push origin v0.1.0
```

Our code (physics, app, tests) builds with `-Wall -Wextra -Wpedantic -Wshadow -Werror`, so a
warning fails the build (`-DNCOTB_WERROR=OFF` to relax that locally). `ctest` also runs a `format`
test that fails if a file differs from `.clang-format` (fix with the "format all" task).

Controls (on Windows, Option is Alt and Cmd is Ctrl):

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
| Cmd + S | save a snapshot of the complete state to `snapshots/` (load it in the panel's "Snapshots" section) |

In the cannon scene:

| Input | Action |
|---|---|
| W / S | barrel up / down |
| A / D | turn left / right |
| Z / X | less / more powder (muzzle speed) |
| Shift | fine aiming |
| F or Space | fire (a dashed arc shows where the ball will go) |
| P | pause |

The "RC truck track" scene is an oval of dirt in a walled arena: whoops, a tabletop and a small
double on the first straight, banked turns, and on the other straight a big double and a loop.
Boost pads (the yellow-to-red arrows) give full throttle and a faster, stronger motor
(`CarSpec::boostSpeed`, `boostTorque`) for 1.2 s after leaving them: enough to clear the big
double and to get round the loop. The loop is entered at an angle (it shifts sideways so the exit
clears the entry): line up with the second pad's arrows, and the kerbs and the yellow guide rib
do the rest.

In the RC truck scenes:

| Input | Action |
|---|---|
| W / S | throttle / reverse (S brakes first while rolling forward, W while rolling back) |
| A / D | steer left / right |
| Space | brake |
| T | put the truck back on its wheels |
| V | chase camera on / off (orbiting by hand pauses it for 2 s) |
| P | pause (Space is the brake here) |

### VS Code

Recommended extensions: clangd, CodeLLDB, CMake Tools (see `.vscode/extensions.json`).
⇧⌘B (Ctrl+Shift+B on Windows) builds the debug preset; the Run and Debug panel has configs for
the app and the tests (also filtered by test name, and under ASan/UBSan). The tasks and launch
configs have Windows variants (PowerShell commands, `.exe` paths). clangd is the one on PATH
(Apple's `/usr/bin/clangd` on macOS, LLVM-MinGW's on Windows); it formats on save with
`.clang-format` and shows the clang-tidy checks in `.clang-tidy`.

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
- [x] 8. Drivable vehicle: an RC monster truck with double-wishbone suspension
- [x] 9. A track for it: jumps, banked turns, boost pads and a loop
