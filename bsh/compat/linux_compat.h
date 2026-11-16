#pragma once
// Pre-set OpenCL C++ bindings version before opencl.hpp is included
#ifndef CL_HPP_TARGET_OPENCL_VERSION
#  define CL_HPP_TARGET_OPENCL_VERSION 120
#endif
#ifndef CL_HPP_MINIMUM_OPENCL_VERSION
#  define CL_HPP_MINIMUM_OPENCL_VERSION 100
#endif

// Boolean constants (Windows BOOL macros)
#ifndef TRUE
#  define TRUE 1
#endif
#ifndef FALSE
#  define FALSE 0
#endif

// Windows timing function — returns milliseconds since an arbitrary epoch
#include <time.h>
inline unsigned long long GetTickCount64() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000ULL
         + (unsigned long long)ts.tv_nsec / 1000000ULL;
}
