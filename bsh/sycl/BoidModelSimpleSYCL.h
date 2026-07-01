// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDMODELSIMPLESYCL_H_
#define _BOIDMODELSIMPLESYCL_H_

#include "common.h"
#include "BoidModel.h"
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
class BoidModelSimpleSYCL : public BoidModel
{
public:
	BoidModelSimpleSYCL(std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	~BoidModelSimpleSYCL();

	// BoidModel interface
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	// Renderable interface
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

private:
	/* Create the two VAO/VBO sets (in/out ping-pong) and the boidTri shader */
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel);

	// helper is used to switch between input and output position buffer
	int helper = 0;
	GLuint pos_vbo[1];
	GLuint pos_vao[1];
	GLuint pos_out_vbo[1];
	GLuint pos_out_vao[1];

	GLuint vel_vbo[1];
	GLuint vel_out_vbo[1];

	// number of boids
	int num;

	// Pointer to simulation time description strings
	std::vector<const char*> simTimeDisc;
	// Simulation time as string for overlay text
	std::string stringSimTime;
	// last measured step time in milliseconds
	long lastSimTimeMs;

	// Attributes used in the shader
	std::vector<std::string> attribName;
	Shader* shader;

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
