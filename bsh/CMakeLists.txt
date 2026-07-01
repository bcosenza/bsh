cmake_minimum_required(VERSION 3.16)
project(BoidSim CXX)

# Single build description for both platforms. Dependencies come from the
# system on Linux (find_package over the distro packages) and from vcpkg on
# Windows (the bundled vcpkg.json manifest); nothing is checked into the repo.

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Built with the usual out-of-source flow (mkdir build && cd build && cmake ..),
# so the executable lands in build/. common.h reaches cl/ and shaders/ via
# the ../ relative paths, which resolve from build/ to the repo root.

# --- dependencies ---
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)      # freeglut on both platforms
find_package(OpenCL REQUIRED)
find_package(Freetype REQUIRED)

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
    LogFile.cpp
    Shader.cpp
    Shader_utils.cpp
    OverlayText.cpp
    WorldBox.cpp
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

# SYCL backend: first port, only the Simple model (see sycl/).
set(SYCL_SOURCES
    sycl/BoidModelSimpleSYCL.cpp)

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

# --- OpenCL executable (default build) ---
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

# When boidSim is also built with the Intel compiler (dual OpenCL+SYCL configure),
# silence the vendored stb_image.h narrowing that only clang enforces. The g++
# build never sees this flag, so it stays unchanged.
if(CMAKE_CXX_COMPILER_ID MATCHES "IntelLLVM")
    target_compile_options(boidSim PRIVATE -Wno-c++11-narrowing)
endif()

copy_runtime_assets(boidSim)

# --- SYCL executable (first port: Simple model only) ---
# Added only when the project is configured with the Intel oneAPI compiler
# (icpx / IntelLLVM), e.g. cmake -DCMAKE_CXX_COMPILER=icpx .. ; a normal g++
# configure builds only boidSim and is completely unaffected.
if(CMAKE_CXX_COMPILER_ID MATCHES "IntelLLVM")
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
