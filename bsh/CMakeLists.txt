cmake_minimum_required(VERSION 3.16)

# --- backend selection ---
# Each backend produces its own executable; pick one or both. The default is the
# SYCL backend. The SYCL backend needs the Intel oneAPI compiler (icpx); the
# OpenCL backend builds with any C++ compiler (g++ by default).
#   cmake -DBUILD_OPENCL=ON -DBUILD_SYCL=OFF ..   # OpenCL only, plain g++
#   cmake -DBUILD_OPENCL=ON ..                    # both backends (needs icpx)
# Declared before project() so a SYCL build can auto-select icpx below, before
# the C++ language is enabled and the compiler is locked in.
option(BUILD_SYCL   "Build the SYCL backend (boidSimSYCL)" ON)
option(BUILD_OPENCL "Build the OpenCL backend (boidSim)"   OFF)

if(NOT BUILD_OPENCL AND NOT BUILD_SYCL)
    message(FATAL_ERROR
        "Nothing to build: enable at least one of BUILD_OPENCL or BUILD_SYCL.")
endif()

# For a SYCL build, if the user has not pinned a compiler, locate icpx on the
# system (PATH via `source setvars.sh`, or a default oneAPI install) and use it.
# An explicit -DCMAKE_CXX_COMPILER=... always wins, since the guard skips then.
if(BUILD_SYCL AND NOT CMAKE_CXX_COMPILER)
    find_program(ICPX_EXECUTABLE
        NAMES icpx
        PATHS /opt/intel/oneapi/compiler/latest/bin
              /opt/intel/oneapi/compiler/latest/linux/bin
        DOC "Intel oneAPI C++/SYCL compiler (icpx)")
    if(ICPX_EXECUTABLE)
        set(CMAKE_CXX_COMPILER "${ICPX_EXECUTABLE}" CACHE FILEPATH
            "C++ compiler" FORCE)
        message(STATUS "SYCL backend: using Intel oneAPI compiler ${ICPX_EXECUTABLE}")
    endif()
endif()

project(BoidSim CXX)

# Dependencies come from the system on Linux (find_package over the distro
# packages) and from vcpkg on Windows (the bundled vcpkg.json manifest).

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# The SYCL backend requires icpx; if auto-detection above did not find it (and
# the user did not point at it), fail with an actionable hint.
if(BUILD_SYCL AND NOT CMAKE_CXX_COMPILER_ID MATCHES "IntelLLVM")
    message(FATAL_ERROR
        "BUILD_SYCL requires the Intel oneAPI compiler (icpx), which was not "
        "found. Source the oneAPI environment (e.g. `source "
        "/opt/intel/oneapi/setvars.sh`) so icpx is on PATH, or pass "
        "-DCMAKE_CXX_COMPILER=icpx.")
endif()

# Built with the usual out-of-source flow (mkdir build && cd build && cmake ..),
# so the executable lands in build/. common.h reaches cl/ and shaders/ via
# the ../ relative paths, which resolve from build/ to the repo root.

# --- dependencies ---
# Shared by both backends.
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)      # freeglut on both platforms
find_package(Freetype REQUIRED)

# Only the OpenCL backend needs an OpenCL SDK, so a SYCL-only configure does not
# require one to be installed.
if(BUILD_OPENCL)
    find_package(OpenCL REQUIRED)
endif()

# GLM is header only: use an installed copy if present, otherwise fetch a
# current release. Either way the target is glm::glm.
find_package(glm CONFIG QUIET)
if(NOT glm_FOUND)
    include(FetchContent)
    FetchContent_Declare(glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 1.0.1)
    FetchContent_MakeAvailable(glm)
endif()

# --- sources ---
# Backend-agnostic app/render layer, shared by both the OpenCL and SYCL builds.
set(SHARED_SOURCES
    Simulation.cpp
    gfx.cpp
    BoidModelSimpleView.cpp
    LogFile.cpp
    Shader.cpp
    Shader_utils.cpp
    OverlayText.cpp
    WorldBox.cpp
    Grid.cpp
    WorldGround.cpp
    Skybox.cpp
    Column.cpp
    Tunnel.cpp)

if(WIN32)
    list(APPEND SHARED_SOURCES main_win.cpp)
else()
    list(APPEND SHARED_SOURCES main_linux.cpp)
endif()

# OpenCL backend: host code that invokes the kernels (see cl/).
set(OPENCL_SOURCES
    cl/CLHelper.cpp
    cl/BoidModelSimple.cpp
    cl/BoidModelGridBase.cpp
    cl/BoidModelGrid.cpp
    cl/BoidModelGrid_2D.cpp
    cl/BoidModelSH.cpp
    cl/BoidModelSH_2D.cpp
    cl/boidModelSHWay1.cpp
    cl/BoidModelSHWay2.cpp
    cl/BoidModelSHCombined.cpp
    cl/boidModelSHObstacle.cpp
    cl/BoidModelSHObstacleTunnel.cpp)

# SYCL backend: Simple (brute-force) and Grid-accelerated models (see sycl/).
set(SYCL_SOURCES
    sycl/BoidModelSimpleSYCL.cpp
    sycl/BoidGridBase.cpp
    sycl/BoidGroupAvoidance.cpp
    sycl/BoidGroupAvoidanceSH.cpp)

# Copy the runtime assets a target opens by its own filename (textures, fonts)
# next to its executable. cl/ and shaders/ stay at the repo root and are reached
# via the ../ relative paths in common.h.
function(copy_runtime_assets target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/textures $<TARGET_FILE_DIR:${target}>/textures
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/FreeSans.otf ${CMAKE_SOURCE_DIR}/FreeSans.ttf
            $<TARGET_FILE_DIR:${target}>/)
endfunction()

# --- OpenCL executable ---
if(BUILD_OPENCL)
    add_executable(boidSim ${SHARED_SOURCES} ${OPENCL_SOURCES})

    # project headers, plus compat/ for the CL/cl.hpp shim that redirects the
    # removed deprecated cl.hpp to the modern opencl.hpp (used on both platforms)
    target_include_directories(boidSim PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/cl
        ${CMAKE_SOURCE_DIR}/compat)

    target_link_libraries(boidSim PRIVATE
        OpenGL::GL
        GLEW::GLEW
        GLUT::GLUT
        OpenCL::OpenCL
        Freetype::Freetype
        glm::glm)

    if(NOT MSVC)
        target_compile_options(boidSim PRIVATE -Wall -Wno-deprecated-declarations)
    endif()

    # When boidSim is built with the Intel compiler (e.g. a combined
    # -DBUILD_SYCL=ON configure), silence the vendored stb_image.h narrowing that
    # only clang enforces. The g++ build never sees this flag, so it stays unchanged.
    if(CMAKE_CXX_COMPILER_ID MATCHES "IntelLLVM")
        target_compile_options(boidSim PRIVATE -Wno-c++11-narrowing)
    endif()

    copy_runtime_assets(boidSim)
endif()

# --- SYCL executable (first port: Simple model only) ---
# Requires the Intel oneAPI compiler (enforced above); enable with -DBUILD_SYCL=ON.
if(BUILD_SYCL)
    add_executable(boidSimSYCL ${SHARED_SOURCES} ${SYCL_SOURCES})

    # DPC++ requires C++17 (the OpenCL build stays on the project-wide C++11)
    set_target_properties(boidSimSYCL PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

    # -Wno-c++11-narrowing: the vendored stb_image.h trips clang's stricter
    # narrowing check that the g++ OpenCL build does not enforce.
    target_compile_options(boidSimSYCL PRIVATE -fsycl -Wno-c++11-narrowing)
    target_link_options   (boidSimSYCL PRIVATE -fsycl)
    target_compile_definitions(boidSimSYCL PRIVATE USE_SYCL)

    # No OpenCL: USE_SYCL drops the <CL/cl.hpp> include and the CLHelper coupling.
    target_include_directories(boidSimSYCL PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/sycl
        ${CMAKE_SOURCE_DIR}/compat)

    target_link_libraries(boidSimSYCL PRIVATE
        OpenGL::GL
        GLEW::GLEW
        GLUT::GLUT
        Freetype::Freetype
        glm::glm)

    copy_runtime_assets(boidSimSYCL)
endif()
