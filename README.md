Behavioral Spherical Harmonics
==============
Behavioral spherical harmonic (BSH) is a novel approach to efficiently and compactly represent the directional-dependent behavior of agents. BSH is based on spherical harmonics to project the directional information of a group of multiple agents to a vector of few coefficients; thus, BSH drastically reduces the complexity of the directional evaluation, as it requires only few agent-group interactions instead of multiple agent-agent ones. 
The BSH model can efficiently model intricate behaviors such as long-range collision avoidance, reaching interactive performance, and avoiding agent congestion on challenging multi-groups scenarios. 
Furthermore, we demonstrate how both the innate parallelism and the compact coefficient representation of the BSH model are well suited for GPU architectures.


- Project's web page: http://bcosenza.github.io/bsh
- Source code:        https://github.com/bcosenza/bsh
- Author's home:      http://www.cosenza.eu

Backends
--------
The simulation is implemented in **SYCL** (`bsh/sycl/`), which is the default and actively maintained backend.

The original OpenCL implementation (`bsh/cl/`) from the 2015 paper is **deprecated**. It is still in the repository and can be built for reference and comparison, but it is no longer developed, and new features (such as the von Mises-Fisher SH projection and the `-timing` mode) are available only in the SYCL backend.

Building (SYCL)
---------------
Requirements:
- Intel oneAPI DPC++/C++ compiler (`icpx`)
- CMake >= 3.16
- OpenGL, GLEW, freeglut and FreeType. On Linux these come from the system packages. On Windows they come from vcpkg, using the bundled `bsh/vcpkg.json` manifest.
- GLM is used if it is installed. Otherwise CMake downloads it.

```bash
cd bsh
source /opt/intel/oneapi/setvars.sh      # optional: CMake also finds icpx in the default oneAPI install
cmake -S . -B build
cmake --build build --target boidSimSYCL -j
```

Running
-------
Run the executable from inside `build/`, because shaders are loaded through relative paths:

```bash
cd build
./boidSimSYCL [scene] [-sh dirac|vmf] [-kappa k] [-timing t]
```

| Option | Meaning |
|---|---|
| `scene`, `-scene n` | scene to start in (keys `1`–`7`) |
| `-sh dirac\|vmf` | SH projection kernel: exact Dirac or von Mises-Fisher smoothed |
| `-kappa k` | von Mises-Fisher concentration |
| `-timing t` | benchmark: run `t` fixed-time-step steps after a warm-up, print mean/min/max time per step and per phase, then quit |

Scenes in the SYCL backend:

| Key | Scene |
|---|---|
| `1` | Simple (brute-force flocking) |
| `2` | Grid (uniform-grid accelerated flocking) |
| `3` | Two groups avoidance |
| `4` | Two groups avoidance with SH |
| `5` | Four groups avoidance with SH |
| `6` | Three columns avoidance with SH |
| `7` | Tunnel SH |

Runtime keys: `r` restart, `+`/`-` double/halve the number of boids, `t` cycle through the initial placements, `p` switch the SH projection, `[`/`]` halve/double the vMF concentration, `v`/`g`/`s`/`l` toggle the world box, ground, skybox and acceleration grid, `q`/`Esc` quit.

Building the deprecated OpenCL backend
--------------------------------------
The OpenCL backend builds with any C++ compiler (g++ by default) and needs an OpenCL SDK:

```bash
cd bsh
cmake -S . -B build -DBUILD_OPENCL=ON -DBUILD_SYCL=OFF
cmake --build build --target boidSim -j
cd build && ./boidSim
```

Leave out `-DBUILD_SYCL=OFF` to build both executables. That requires `icpx`.

Citation
--------
If you use this code for research purpose, please cite the paper:

```
@inproceedings{Cosenza15,
  author    = {Biagio Cosenza},
  title     = {{Behavioral Spherical Harmonics for Long-Range Agents' Interaction}},
  booktitle = {Euro-Par 2015: Parallel Processing Workshops - PADABS},
  pages     = {392--404},
  year      = {2015}  
}
```

License
-------
The Behavioural Spherical Harmonics (BSH) code is licensed under BSD 3 (or "BSD Simplified").
For full license terms please see the LICENSE file distributed with this source code.
