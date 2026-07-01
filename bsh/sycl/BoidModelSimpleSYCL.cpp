// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
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

/*
	Small GL helper: create a VBO, upload data and leave it bound (mirrors the
	behaviour of CLHelper::createVBO, but without any OpenCL dependency).
*/
static GLuint makeVBO(const void* data, size_t dataSize){
	GLuint id;
	glGenBuffers(1, &id);
	glBindBuffer(GL_ARRAY_BUFFER, id);
	glBufferData(GL_ARRAY_BUFFER, dataSize, data, GL_DYNAMIC_DRAW);
	return id;
}

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

	// Vec4 and sycl::sf4 share the same 16-byte layout, so the initial host
	// state can be copied straight into the device buffers.
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

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, pos_vbo);
	glDeleteBuffers(1, pos_out_vbo);

	glBindVertexArray(0);
	glDeleteVertexArrays(1, pos_vao);
	glDeleteVertexArrays(1, pos_out_vao);

	delete shader;
}

GLuint BoidModelSimpleSYCL::getPosVAO(){
	if (helper % 2 == 0)
		return pos_out_vao[0];
	else
		return pos_vao[0];
}

GLuint BoidModelSimpleSYCL::getPosVBO(){
	if (helper % 2 == 0)
		return pos_out_vbo[0];
	else
		return pos_vbo[0];
}

GLuint BoidModelSimpleSYCL::getVelVBO(){
	if (helper % 2 == 0)
		return vel_out_vbo[0];
	else
		return vel_vbo[0];
}

int BoidModelSimpleSYCL::getNumBoid(){
	return num;
}

void BoidModelSimpleSYCL::createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel){

	std::vector<Vec4> newDataColor(num);
	for (int i = 0; i < num; i++)
		newDataColor[i] = BOID_COLOR;

	GLuint id[1];
	size_t array_size = num * sizeof(Vec4);

	shader = new Shader("boidTri.v.glsl", "boidTri.f.glsl", "boidTri.g.glsl");
	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetAttribLocation(shader->id(), "color");
	GLint velLoc = glGetAttribLocation(shader->id(), "vel3d");

	//------VBO 1--------- (in)
	glGenVertexArrays(1, &pos_vao[0]);
	glBindVertexArray(pos_vao[0]);

	pos_vbo[0] = makeVBO(&pos[0], array_size);
	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(vertLoc);

	vel_vbo[0] = makeVBO(&vel[0], array_size);
	glVertexAttribPointer(velLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(velLoc);

	glGenBuffers(1, &id[0]);
	glBindBuffer(GL_ARRAY_BUFFER, id[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* num, &newDataColor[0], GL_STATIC_DRAW);
	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0);
	glBindVertexArray(0);

	//------VBO 2--------- (out)
	glGenVertexArrays(1, &pos_out_vao[0]);
	glBindVertexArray(pos_out_vao[0]);

	pos_out_vbo[0] = makeVBO(&pos[0], array_size);
	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(vertLoc);

	vel_out_vbo[0] = makeVBO(&vel[0], array_size);
	glVertexAttribPointer(velLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(velLoc);

	glGenBuffers(1, &id[0]);
	glBindBuffer(GL_ARRAY_BUFFER, id[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* num, &newDataColor[0], GL_STATIC_DRAW);
	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0);
	glBindVertexArray(0);
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

		int numFlockMates = 0;

		// compare every boid with every other boid (brute force O(n^2))
		for (int j = 0; j < (int)sp.numBodies; j++){
			if (j == id) continue;

			sf4 otherBoid = posIn[j];
			sf4 dist = otherBoid - p;
			dist.w() = 0.0f;

			float distLength = sycl::length(dist);
			if (distLength < 5.0f){
				float dotP = sycl::dot(-v, dist);
				float lenV = sycl::length(v);
				float angle = dotP / (lenV * distLength);

				// acute angle in degrees; acospi(x)*180 == acos(x)/pi*180
				float deg = sycl::acos(angle) * (180.0f / (float)M_PI);
				if ((dotP < 0.0f) || (sycl::fabs(deg) > 45.0f)){
					cohesion  += otherBoid;
					alignment += velIn[j];
					numFlockMates += 1;

					if (distLength < 2.5f)
						separation -= dist;
				}
			}
		}

		if (numFlockMates > 0){
			alignment = (alignment / (float)numFlockMates) - v;
			cohesion  = (cohesion  / (float)numFlockMates) - p;
		}

		// apply weights and compute new velocity
		v = v * sp.wOwn + (cohesion * sp.wCohesion + alignment * sp.wAlignment + separation * sp.wSeparation);
		v.w() = 0.0f;

		// cap the speed at maxVel (nicer than clamping per component)
		float len = sycl::length(v);
		if (len > sp.maxVel){
			v.x() = (v.x() / len) * sp.maxVel;
			v.y() = (v.y() / len) * sp.maxVel;
			v.z() = (v.z() / len) * sp.maxVel;
		}

		// grid position, to detect border cells (getGridPos in the kernel)
		int gx = (int)sycl::floor((p.x() - sp.worldOrigin.x) / sp.cellSize.x);
		int gy = (int)sycl::floor((p.y() - sp.worldOrigin.y) / sp.cellSize.y);
		int gz = (int)sycl::floor((p.z() - sp.worldOrigin.z) / sp.cellSize.z);
		gx = sycl::clamp(gx, 0, (int)sp.gridSize.x - 1);
		gy = sycl::clamp(gy, 0, (int)sp.gridSize.y - 1);
		gz = sycl::clamp(gz, 0, (int)sp.gridSize.z - 1);

		if (gx < boundingBoxFactor)                              velCor.x() =  sp.maxVelCor;
		if (gx >= (int)sp.gridSize.x - boundingBoxFactor)        velCor.x() = -sp.maxVelCor;
		if (gy < boundingBoxFactor)                              velCor.y() =  sp.maxVelCor;
		if (gy >= (int)sp.gridSize.y - boundingBoxFactor)        velCor.y() = -sp.maxVelCor;
		if (gz < boundingBoxFactor)                              velCor.z() =  sp.maxVelCor;
		if (gz >= (int)sp.gridSize.z - boundingBoxFactor)        velCor.z() = -sp.maxVelCor;

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

void BoidModelSimpleSYCL::render(){
	shader->bind();
	glBindVertexArray(getPosVAO());
	glDrawArrays(GL_POINTS, 0, num);
	glBindVertexArray(0);
	shader->unbind();
}

long BoidModelSimpleSYCL::getSimulationTime(){
	return lastSimTimeMs;
}

void BoidModelSimpleSYCL::bindShader(){
	shader->bind();
}

void BoidModelSimpleSYCL::unbindShader(){
	shader->unbind();
}

Shader* BoidModelSimpleSYCL::getShader(){
	return shader;
}

std::vector<const char*> BoidModelSimpleSYCL::getSimTimeDescriptions(){
	std::stringstream strstream;
	strstream.str(std::string());
	strstream << "Simulation time: " << getSimulationTime() << "ms" << "\0";
	stringSimTime = strstream.str();
	simTimeDisc[4] = stringSimTime.c_str();
	return simTimeDisc;
}

void BoidModelSimpleSYCL::getFollowedBoid(unsigned int* boidIndex, Vec4* pos, Vec4* vel){
	Vec4 v;
	GLuint vbo = getVelVBO();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glGetBufferSubData(GL_ARRAY_BUFFER, sizeof(Vec4)* *boidIndex, sizeof(Vec4), &v);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	(*vel).set(v.x, v.y, v.z, 0.0);

	Vec4 p;
	vbo = getPosVBO();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glGetBufferSubData(GL_ARRAY_BUFFER, sizeof(Vec4)* *boidIndex, sizeof(Vec4), &p);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	(*pos).set(p.x, p.y, p.z, 0.0);
}
