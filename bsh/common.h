// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
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

#define __CL_ENABLE_EXCEPTIONS

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

//OpenCL include
#define CL_USE_DEPRECATED_OPENCL_2_0_APIS
#include <CL/cl.hpp>

//OpenGL matrix/vector manipulation header lib
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>
#include <gtx/rotate_vector.hpp>

const std::string kernel_path = "../kernels/";
const std::string shader_path = "../shaders/";
