// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidModelSimpleSYCL.h"
#include "vectorTypes.h"
#include <chrono>

// distinct alias: vector_types.h already defines an (operator-less) float4
using sf4 = sycl::float4;

// same neighbourhood bounding-box factor as boidModelSimple_kernel_v2.cl
#define boundingBoxFactor 2

BoidModelSimpleSYCL::BoidModelSimpleSYCL(std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP)
{
	simTimeDisc = std::vector<const char*>(5);
	simTimeDisc[0] = "Boid Model Simple";
	simTimeDisc[1] = "SYCL Simulation Times:";
	simTimeDisc[2] = "";
	simTimeDisc[3] = "";
	simTimeDisc[4] = "";

	simParams = *simP;
	num = simParams.numBodies;
	lastSimTimeMs = 0;
	hostBuf.resize(num);

	createVboBindShader(pos, vel);

	//make sure OpenGL is finished before we proceed
	glFinish();

	// device buffer allocation
	size_t bytes = num * sizeof(sf4);
	d_pos     = sycl::malloc_device<sf4>(num, queue);
	d_pos_out = sycl::malloc_device<sf4>(num, queue);
	d_vel     = sycl::malloc_device<sf4>(num, queue);
	d_vel_out = sycl::malloc_device<sf4>(num, queue);

	queue.memcpy(d_pos, pos.data(), bytes);
	queue.memcpy(d_vel, vel.data(), bytes);
	queue.wait();
}

BoidModelSimpleSYCL::~BoidModelSimpleSYCL(){
	queue.wait();
	sycl::free(d_pos, queue);
	sycl::free(d_pos_out, queue);
	sycl::free(d_vel, queue);
	sycl::free(d_vel_out, queue);

	destroyGLResources();
}

void BoidModelSimpleSYCL::simulate(float dt){
	size_t array_size = num * sizeof(Vec4);
	const int n = num;
	const simParams_t sp = simParams;	// captured by value into the kernel

	bool pingPong = ((helper++ % 2) == 0);

	// select input/output device buffers (ping-pong, as the OpenCL model swaps
	// its kernel arguments)
	sf4* posIn  = pingPong ? d_pos     : d_pos_out;
	sf4* posOut = pingPong ? d_pos_out : d_pos;
	sf4* velIn  = pingPong ? d_vel     : d_vel_out;
	sf4* velOut = pingPong ? d_vel_out : d_vel;

	auto t0 = std::chrono::high_resolution_clock::now();

	queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		const int id = (int)idx[0];

		sf4 p = posIn[id];
		sf4 v = velIn[id];

		sf4 cohesion   = sf4(0.0f);
		sf4 separation = sf4(0.0f);
		sf4 alignment  = sf4(0.0f);
		sf4 velCor     = sf4(0.0f);

		// each rule averages over its own neighbourhood, so it needs its own count
		int cohesionMates  = 0;
		int alignmentMates = 0;

		// compare every boid with every other boid (brute force O(n^2))
		for (int j = 0; j < (int)sp.numBodies; j++){
			if (j == id) continue;

			sf4 otherBoid = posIn[j];
			sf4 dist = otherBoid - p;
			dist.w() = 0.0f;

			float distLength = sycl::length(dist);

			// field-of-view test: ignore neighbours behind the boid (outside its
			// ~45deg forward vision cone), regardless of the rule's radius
			float dotP  = sycl::dot(-v, dist);
			float lenV  = sycl::length(v);
			float angle = dotP / (lenV * distLength);
			// acute angle in degrees; acospi(x)*180 == acos(x)/pi*180
			float deg   = sycl::acos(angle) * (180.0f / (float)M_PI);
			bool inView = (dotP < 0.0f) || (sycl::fabs(deg) > 45.0f);
			if (!inView) continue;

			// Cohesion: steer toward the average position of nearby flockmates
			if (distLength < cohesionRadius){
				cohesion += otherBoid;
				cohesionMates += 1;
			}

			// Alignment: steer toward the average heading of nearby flockmates
			if (distLength < alignmentRadius){
				alignment += velIn[j];
				alignmentMates += 1;
			}

			// Separation: steer away from flockmates that are too close
			if (distLength < separationRadius)
				separation -= dist;
		}

		// turn the accumulated positions/velocities into steering vectors relative
		// to this boid (average neighbour minus own state)
		if (alignmentMates > 0)
			alignment = (alignment / (float)alignmentMates) - v;
		if (cohesionMates > 0)
			cohesion  = (cohesion  / (float)cohesionMates) - p;

		// apply weights and compute new velocity
		v = v * sp.wOwn + (cohesion * sp.wCohesion + alignment * sp.wAlignment + separation * sp.wSeparation);
		v.w() = 0.0f;

		// cap the speed at maxVel (nicer than clamping per component)
		float len = sycl::length(v);
		if (len > sp.maxVel)
			v *= sp.maxVel / len;   // w is already 0, so it stays 0

		// wall repulsion: push the boid back toward the interior once it comes
		// within boundingBoxFactor cells of a world face. This is a pure boundary
		// test, so it works straight from world coordinates.
		const sf4 cellSize = sf4(sp.cellSize.x, sp.cellSize.y, sp.cellSize.z, 0.0f);
		const sf4 margin   = cellSize * (float)boundingBoxFactor;
		const sf4 worldMax = sf4((float)sp.gridSize.x, (float)sp.gridSize.y, (float)sp.gridSize.z, 0.0f) * cellSize;
		const sf4 r        = p - sf4(sp.worldOrigin.x, sp.worldOrigin.y, sp.worldOrigin.z, 0.0f);

		// +maxVelCor near the low faces, -maxVelCor near the high faces (high wins
		// on overlap, as before). maxCor.w() is 0, so velCor.w() stays 0 whatever
		// the w lane of the border masks happens to be.
		const sf4 maxCor = sf4(sp.maxVelCor, sp.maxVelCor, sp.maxVelCor, 0.0f);
		velCor = sycl::select(velCor,  maxCor, r <  margin);
		velCor = sycl::select(velCor, -maxCor, r >= (worldMax - margin));

		// apply border correction and integrate the position
		v += velCor;
		p += v * dt;

		posOut[id] = p;
		velOut[id] = v;
	});
	queue.wait();

	auto t1 = std::chrono::high_resolution_clock::now();
	lastSimTimeMs = (long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

	// copy the freshly computed state back into the GL VBOs that getPosVAO()
	// will render this frame
	GLuint posVboToFill = pingPong ? pos_out_vbo[0] : pos_vbo[0];
	GLuint velVboToFill = pingPong ? vel_out_vbo[0] : vel_vbo[0];

	queue.memcpy(hostBuf.data(), posOut, array_size).wait();
	glBindBuffer(GL_ARRAY_BUFFER, posVboToFill);
	glBufferSubData(GL_ARRAY_BUFFER, 0, array_size, hostBuf.data());

	queue.memcpy(hostBuf.data(), velOut, array_size).wait();
	glBindBuffer(GL_ARRAY_BUFFER, velVboToFill);
	glBufferSubData(GL_ARRAY_BUFFER, 0, array_size, hostBuf.data());
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

long BoidModelSimpleSYCL::getSimulationTime(){
	return lastSimTimeMs;
}

std::vector<const char*> BoidModelSimpleSYCL::getSimTimeDescriptions(){
	std::stringstream strstream;
	strstream.str(std::string());
	strstream << "Simulation time: " << getSimulationTime() << "ms" << "\0";
	stringSimTime = strstream.str();
	simTimeDisc[4] = stringSimTime.c_str();
	return simTimeDisc;
}
