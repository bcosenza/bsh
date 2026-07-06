// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidGroupAvoidanceSH.h"
#include "vectorTypes.h"
#include <cmath>
#include <chrono>

BoidGroupAvoidanceSH::BoidGroupAvoidanceSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP)
	: BoidGroupAvoidance(pos, vel, color, simP)
{
	simTimeDisc[0] = "Two groups avoidance with SH";

	// one order-3 SH coefficient set per group, stored on the device
	d_groupSH = sycl::malloc_device<float>(2 * SH_COEFFS, queue);
	queue.wait();
}

BoidGroupAvoidanceSH::~BoidGroupAvoidanceSH(){
	queue.wait();
	sycl::free(d_groupSH, queue);
	// ~BoidGridBase / ~BoidModelSimpleSYCL free the rest
}

void BoidGroupAvoidanceSH::simulate(float dt){
	const int         n    = num;
	const int         half = n / 2;                 // group 0 = [0,half), group 1 = [half,n)
	const simParams_t sp   = simParams;

	const float cs   = cellSize;
	const int   dimX = gridDimX, dimY = gridDimY, dimZ = gridDimZ;
	const sf4   origin = gridOrigin;

	auto t0 = std::chrono::high_resolution_clock::now();

	// --- SH step 1: project each boid's heading into its group's SH basis ---
	// d_vel is in original boid order (index i => group i<half), so this is a
	// trivial two-bucket reduction. Done on the host for simplicity (only two
	// groups); the averaged coefficients are then uploaded to the device.
	queue.memcpy(hostBuf.data(), d_vel, (size_t)n * sizeof(Vec4)).wait();

	float sh[2 * SH_COEFFS];
	for (int k = 0; k < 2 * SH_COEFFS; k++) sh[k] = 0.0f;
	const int count[2] = { half, n - half };

	for (int i = 0; i < n; i++){
		const int g = (i < half) ? 0 : 1;
		float x = hostBuf[i].x, y = hostBuf[i].y, z = hostBuf[i].z;
		float len = sqrtf(x * x + y * y + z * z);
		if (len > 1e-6f){ x /= len; y /= len; z /= len; } else { x = y = z = 0.0f; }

		float b[8];
		shEval3(x, y, z, b);
		float* c = sh + g * SH_COEFFS;
		c[0] += SH_BAND0;
		for (int k = 0; k < 8; k++) c[1 + k] += b[k];
	}
	// average per group so the reconstructed field is ~O(1) regardless of size
	for (int g = 0; g < 2; g++)
		if (count[g] > 0)
			for (int k = 0; k < SH_COEFFS; k++)
				sh[g * SH_COEFFS + k] /= (float)count[g];

	queue.memcpy(d_groupSH, sh, sizeof(sh)).wait();

	// --- build the uniform grid (inherited, on-device) ---
	buildGrid();

	// --- local flocking interaction + SH step 2 (cross-group reconstruction) ---
	sf4* sortedPos = d_pos_out;
	sf4* sortedVel = d_vel_out;
	sf4* outPos    = d_pos;
	sf4* outVel    = d_vel;
	int* cellStart = d_cellStart;
	int* cellEnd   = d_cellEnd;
	unsigned int* index = d_index;
	float* groupSH = d_groupSH;
	const int   halfLocal  = half;
	const float shStrength = shAvoidStrength;

	queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		const int k = (int)idx[0];
		sf4 p = sortedPos[k];
		sf4 v = sortedVel[k];

		// local flocking over the 27 neighbouring cells -> steering vector
		FlockAccum acc = gridAccumulate(p, v, sortedPos, sortedVel,
		                                cellStart, cellEnd, cs, dimX, dimY, dimZ, origin);
		sf4 steer = finalizeSteer(acc, p, v, sp);

		// SH step 2: evaluate the OTHER group's SH field in this agent's direction
		// and reconstruct a velocity contribution
		const int orig  = (int)index[k];
		const int group = (orig < halfLocal) ? 0 : 1;
		const float* otherSH = groupSH + (1 - group) * SH_COEFFS;
		sf4 shSteer = shAvoidanceSteer(v, otherSH, shStrength);

		// combine local + SH steering, then integrate (shared dynamics)
		advanceBoid(p, v, steer + shSteer, sp, dt);

		outPos[orig] = p;
		outVel[orig] = v;
	});
	queue.wait();

	auto t1 = std::chrono::high_resolution_clock::now();
	lastSimTimeMs = (long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

	// draw: upload into the ping-pong VBO the base's getPosVAO() renders this frame
	bool pingPong = ((helper++ % 2) == 0);
	uploadToVBO(outPos, pingPong ? pos_out_vbo[0] : pos_vbo[0]);
	uploadToVBO(outVel, pingPong ? vel_out_vbo[0] : vel_vbo[0]);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
