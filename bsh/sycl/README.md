# SYCL backend (planned)

This folder will hold the SYCL reimplementations of the boid models that
currently live, in their OpenCL form, under [`../cl`](../cl).

The intended layout mirrors `cl/`: a SYCL device/queue helper (the SYCL
counterpart of `CLHelper`) plus one `.cpp` per model
(`BoidModelSimple`, `BoidModelGrid*`, `BoidModelSH*`, ...).

Prerequisite: `BoidModel.h` (kept at the repo root as the shared base class)
still embeds `cl::` types. Before adding SYCL implementations here, that header
needs a backend-agnostic abstraction so both `cl/` and `sycl/` can implement
against it.
