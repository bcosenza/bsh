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

// sf4 (== sycl::float4) and boundingBoxFactor live in BoidModelSimpleSYCL.h so
// the shared wallRepulsion() device helper can use them too.

BoidModelSimpleSYCL::BoidModelSimpleSYCL(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP)
{
	simTimeDisc = std::vector<const char*>(5);
	simTimeDisc[0] = "Boid Model Simple";
	simTimeDisc[1] = "SYCL Simulation Times:";
	simTimeDisc[2] = "";
	simTimeDisc[3] = "";
	simTimeDisc[4] = "";

	simParams = *simP;

	// Override the flocking weights for this model only (leaving SimParam.h and
	// the other models untouched). The default SimParam weights let own-velocity
	// dwarf the three rules; unit weights put cohesion/alignment/separation on the
	// same footing as momentum so their behaviour is actually visible. Note these
	// rules are still un-normalized, so equal weights != equal influence
	// (alignment/separation naturally dominate cohesion) -- normalization TBD.
	simParams.wCohesion   = 1.0f;
	simParams.wAlignment  = 1.0f;
	simParams.wSeparation = 1.0f;
	simParams.wOwn        = 1.0f;

	num = simParams.numBodies;
	lastSimTimeMs = 0;
	hostBuf.resize(num);

	createVboBindShader(pos, vel, color);

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

		// Reynolds steering over the whole flock (brute force O(n^2)), then the
		// shared dynamics (momentum + gain, speed clamp, wall, integrate)
		const sf4 steer = flockingSteer(p, v, posIn, velIn, n, sp);
		advanceBoid(p, v, steer, sp, dt);

		posOut[id] = p;
		velOut[id] = v;
	});
	queue.wait();

	auto t1 = std::chrono::high_resolution_clock::now();
	lastSimTimeMs = (long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

	// copy the freshly computed state back into the GL VBOs that getPosVAO()
	// will render this frame
	uploadToVBO(posOut, pingPong ? pos_out_vbo[0] : pos_vbo[0]);
	uploadToVBO(velOut, pingPong ? vel_out_vbo[0] : vel_vbo[0]);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void BoidModelSimpleSYCL::uploadToVBO(sf4* deviceBuf, GLuint vbo){
	size_t bytes = num * sizeof(Vec4);
	queue.memcpy(hostBuf.data(), deviceBuf, bytes).wait();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, hostBuf.data());
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
