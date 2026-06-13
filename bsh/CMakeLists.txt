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

# The executable runs with ../kernels and ../shaders as its relative asset
# paths (see common.h), so place it in <repo>/bin like the old Makefile did.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

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
set(SOURCES
    Simulation.cpp
    gfx.cpp
    CLHelper.cpp
    LogFile.cpp
    Shader.cpp
    Shader_utils.cpp
    OverlayText.cpp
    WorldBox.cpp
    WorldGround.cpp
    Skybox.cpp
    Column.cpp
    Tunnel.cpp
    BoidModelSimple.cpp
    BoidModelGridBase.cpp
    BoidModelGrid.cpp
    BoidModelGrid_2D.cpp
    BoidModelSH.cpp
    BoidModelSH_2D.cpp
    boidModelSHWay1.cpp
    BoidModelSHWay2.cpp
    BoidModelSHCombined.cpp
    boidModelSHObstacle.cpp
    BoidModelSHObstacleTunnel.cpp)

if(WIN32)
    list(APPEND SOURCES main_win.cpp)
else()
    list(APPEND SOURCES main_linux.cpp)
endif()

add_executable(boidSim ${SOURCES})

# project headers, plus compat/ for the CL/cl.hpp shim that redirects the
# removed deprecated cl.hpp to the modern opencl.hpp (used on both platforms)
target_include_directories(boidSim PRIVATE
    ${CMAKE_SOURCE_DIR}
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

# --- runtime assets next to the executable (kernels/ and shaders/ are reached
# --- via the ../ relative paths in common.h and stay at the repo root) ---
add_custom_command(TARGET boidSim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/textures $<TARGET_FILE_DIR:boidSim>/textures
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/FreeSans.otf ${CMAKE_SOURCE_DIR}/FreeSans.ttf
        $<TARGET_FILE_DIR:boidSim>/)
