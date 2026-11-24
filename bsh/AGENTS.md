# AGENTS.md

This file provides guidance to AI coding agents (Claude Code, Codex, Cursor, etc.) when working with code in this repository.

## Project

BSH (Behavioral Spherical Harmonics): a GPU boid/flocking simulator with OpenGL rendering. Long-range agent–group and obstacle interactions are represented by projecting directional information into order-3 (9-coefficient) spherical harmonics. The original 2015 implementation is OpenCL (`cl/`); the current work is a SYCL port (`sycl/`) that is also used for the journal paper (`docs/methods_boids.tex`, figure scripts in `../scripts/`).

This directory (`bsh/`) is the CMake project root. The git repo root is one level up.

## Build and run

```bash
# SYCL backend (default target boidSimSYCL); needs Intel oneAPI icpx.
# CMake auto-finds icpx in /opt/intel/oneapi/compiler/latest/bin, or: source /opt/intel/oneapi/setvars.sh
cmake -S . -B build
cmake --build build --target boidSimSYCL -j

# OpenCL backend only (plain g++, needs an OpenCL SDK)
cmake -S . -B build-cl -DBUILD_OPENCL=ON -DBUILD_SYCL=OFF
```

Run the program from inside `build/`. `common.h` loads shaders and kernels through the relative paths `../shaders/` and `../cl/`:

```bash
cd build && ./boidSimSYCL [scene] [-sh dirac|vmf|exact] [-kappa k] [-timing t]
                          [-accuracy t] [-out file.csv] [-seed s] [-hspread kh]
                          [-snapshot f1 f2 ...] [-snapshot-dir d] [-window w h] [-camera ex ey ez cx cy cz]
```

- `-timing t` runs headless-style benchmarking: it does warm-up steps, then `t` fixed-dt steps, prints mean/min/max per phase (project/grid/interact/upload), and quits. Use it to measure performance.
- `-accuracy t` (scenes 8 and 9 only) runs `t` fixed-dt steps with a fixed seed. Each step it compares the Dirac and vMF SH steering against a brute-force reference on the same state, then prints a per-lane table, appends one CSV row per lane to `-out` (default `accuracy.csv`; the header is written only into a new file), and quits. `-sh exact` drives these scenes with the reference itself. `-seed` fixes the placement and `-hspread` adds vMF noise to the initial headings.
- `-snapshot f1 f2 ...` writes `scene<N>_snap_<FFFF>.png` (no overlay text) after the given frames and quits. Frame 0 is the initial state. Runs use a fixed dt and seed, so the images are reproducible. `-window` sets the image size and `-camera` a fixed viewpoint. `../scripts/make_snapshots.py` reproduces the paper snapshots, and `../scripts/eval_scene9_accuracy.py` the scene 9 accuracy table.
- Runtime keys: `1`–`0` switch scene, `r` restart, `+`/`-` double/halve the boid count, `t` cycle the initial placement, `p` cycle the SH projection (Dirac → vMF, then the exact reference in scenes 8 and 9), `[`/`]` halve/double the vMF κ, `v`/`g`/`s`/`l` toggle the box, ground, skybox, and acceleration-grid overlay.

There is no test suite or linter. Verify a change by building it and running the affected scene, or by using `-timing`.

IDE/IntelliSense errors on `sycl/*` files (for example, unresolved `sycl::` names) are false positives. Only the icpx build output counts.

## Architecture

**Two backends, one shared app layer.** Each CMake option builds its own executable from `SHARED_SOURCES` (Simulation, gfx, renderables, overlay) plus backend sources. The SYCL target defines `USE_SYCL`, which removes every OpenCL include and the `CLHelper` dependency through `#ifdef`s in `common.h`, `Simulation.h/.cpp`. The SYCL target builds as C++17 and the OpenCL target as C++11.

**Scenes.** `Simulation` (a singleton) owns the current `BoidModel*`, the renderables, and the input handling. Each scene is one row in `sceneTable` in `Simulation.cpp` (weights, world grid size, camera preset, whether columns or tunnel are shown) plus one case in `createModel()`. **The scene IDs are the `BOID_*` constants in `SimParam.h`, but the SYCL build reuses those IDs for different scenes**, selected with `#ifdef USE_SYCL` in both places:

| Key | OpenCL scene | SYCL scene / class |
|---|---|---|
| 1 | Simple | `BoidModelSimpleSYCL` (brute force O(N²)) |
| 2 | Grid | `BoidGridBase` (uniform grid) |
| 3 | SH | `BoidGroupAvoidance` (two groups, no SH) |
| 4 | Grid 2D | `BoidGroupAvoidanceSH` |
| 5 | SH 2D | `Boid4GroupsAvoidanceSH` |
| 6 | SH Way1 | `BoidThreeColumnsSH` (obstacle SH) |
| 7 | SH Way2 | `BoidTunnelSH` |
| 8 | SH Obstacle | `BoidCompositionSH` (SH accuracy: four groups composition, Gaunt products) |
| 9 | SH Obstacle + Way | `BoidPlateLanesSH` (SH accuracy: plate lanes, obstacle term) |
| 0 | SH Tunnel | (OpenCL only) |

`scenes_notes.txt` describes the original OpenCL scenes and their known issues.

**SYCL class hierarchy.** Each level adds one term to the per-agent steering:
`BoidModel` (abstract interface, `BoidModel.h`) → `BoidModelSimpleView` (GL VBO/VAO rendering) → `BoidModelSimpleSYCL` (USM device buffers, the shared device helpers `wallRepulsion`, flocking maths, `shEval3`/`shProject`) → `BoidGridBase` (hash → sort → cell ranges → reorder, 27-cell neighbour search; the cell size comes from `GRID_ACCEL_CELL_SIZE` / interaction radius, not from the `GRID_SIZE_*` macros) → `BoidGroupAvoidance` → `BoidGroupAvoidanceSH` (per-group SH projection, N groups) → `Boid4GroupsAvoidanceSH`, and `BoidThreeColumnsSH` (precomputed obstacle SH field) → `BoidTunnelSH`. The accuracy scenes branch off as `BoidGroupAvoidanceSH` → `BoidCompositionSH` and `BoidThreeColumnsSH` → `BoidPlateLanesSH`.

Each step copies the device results back into GL VBOs on the host. There is no GL interop. Per-phase timings are reported through `getPhaseTimes()`.

**SH projection policies** (`sycl/SHProjection.h`): `SHDirac` and `SHVonMisesFisher` are template parameters of the projection kernels, so each policy compiles to its own kernel with no runtime branch. A runtime `SHMode` chooses which instantiation runs. The vMF band factors are computed on the host in double precision. In the code the concentration is called `k`, because `kappa` means obstacle strength in the methods text. `SHMode::Exact` is not a projection: it selects the brute-force reference (the exact, untruncated vMF field with the same `k`), which only the accuracy scenes implement. Other scenes fall back to Dirac.

**Accuracy scenes** (8, 9). Scene 8 reuses the four-group placement of scene 5. The red group steers toward the maximum of a composite field, P = max(0, w·goal) · F̂_orange · (1 − F̂_blue), where each F̂ is a group heading field normalised to [0, 1]. In SH form, P is built with Gaunt products: the 9×9×9 triple-product tensor is integrated exactly at construction, in the `shEval3` basis. The other groups only seek their goals. Scene 9 stacks 7 independent lanes along y (geometry in `AccuracyLanes.h`), each with one group facing a thin plate at a set incidence angle; `PlateLanes` is a shared renderable that also generates the obstacle points. Goal seeking keeps the groups on course, and the plate points' 1/r² weight is tapered to zero at `PLATE_RANGE`. Each step both models evaluate Dirac, vMF and the brute-force reference on the same state, and `sycl/AccuracyProbe` accumulates the statistics on the host.

**Parameters.** All compile-time tuning lives in `SimParam.h` (weights, grid sizes, boid counts) and `BoidModel.h` (placements, SH projection defaults, camera presets). `simParams_t` is passed by value into kernels. In the OpenCL backend it is also duplicated by hand in each `.cl` file, as are `LOCAL_SIZE_LIMIT` and `bitonic_sort.cl`, and those copies must be kept in sync manually.

**Initial placements** (`createData` in `Simulation.cpp`, cycled with `t`) set positions, velocities, goals and per-group colors. The group models depend on boids of the same group being stored contiguously.
