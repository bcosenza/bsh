// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#pragma once

// Platform compatibility: on Linux this provides the Windows-isms the code
// uses (TRUE/FALSE, GetTickCount64) and pins the OpenCL C++ bindings version;
// on Windows the equivalents come from <windows.h> via GL/glew.h.
#ifndef _WIN32
#include "linux_compat.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <ctime>
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>

//OpenGL include
#include <GL/glew.h>
#include <GL/freeglut.h>

//OpenCL include (skipped for the SYCL build, which has no OpenCL dependency)
//The legacy <CL/cl.hpp> is no longer shipped by OpenCL SDKs, so include the modern
//C++ bindings <CL/opencl.hpp> directly, configured to match: exceptions on, OpenCL 1.2
#ifndef USE_SYCL
#define CL_USE_DEPRECATED_OPENCL_2_0_APIS
#define CL_HPP_ENABLE_EXCEPTIONS
#ifndef CL_HPP_TARGET_OPENCL_VERSION
#define CL_HPP_TARGET_OPENCL_VERSION 120
#endif
#ifndef CL_HPP_MINIMUM_OPENCL_VERSION
#define CL_HPP_MINIMUM_OPENCL_VERSION 100
#endif
#include <CL/opencl.hpp>
#endif

//OpenGL matrix/vector manipulation header lib
//GLM_ENABLE_EXPERIMENTAL is required by gtx/ headers since GLM 0.9.9
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>

const std::string kernel_path = "../cl/";
const std::string shader_path = "../shaders/";
