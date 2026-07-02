// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDMODELSIMPLESYCL_H_
#define _BOIDMODELSIMPLESYCL_H_

#include "common.h"
#include "BoidModelSimpleView.h"
#include "vectorTypes.h"
#include <sycl/sycl.hpp>

/*
	SYCL port of BoidModelSimple (the naive O(n^2) flocking model).

	Backend-agnostic sibling of the OpenCL cl/BoidModelSimple: derives from the
	same BoidModel interface, so gfx.cpp drives it unchanged. Boid state lives in
	device memory allocated with sycl::malloc_device; each step runs a SYCL
	parallel_for port of the boidKernel and copies the result back into the GL
	VBOs that are rendered (mirroring the no-GL-interop path of the OpenCL model).
*/
class BoidModelSimpleSYCL : public BoidModelSimpleView
{
public:
	BoidModelSimpleSYCL(std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	~BoidModelSimpleSYCL();

	// backend-specific parts of the BoidModel interface (the rest come from the view)
	void simulate(float dt);
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();

private:
	// Pointer to simulation time description strings
	std::vector<const char*> simTimeDisc;
	// Simulation time as string for overlay text
	std::string stringSimTime;
	// last measured step time in milliseconds
	long lastSimTimeMs;

	// SYCL device state. Boid state is stored as sycl::float4 in device memory
	// (same 16-byte layout as Vec4, so host<->device transfers are plain memcpy).
	sycl::queue queue;
	sycl::float4* d_pos;
	sycl::float4* d_pos_out;
	sycl::float4* d_vel;
	sycl::float4* d_vel_out;

	// host staging buffer used to upload results into the GL VBOs
	std::vector<Vec4> hostBuf;
};

#endif // _BOIDMODELSIMPLESYCL_H_
