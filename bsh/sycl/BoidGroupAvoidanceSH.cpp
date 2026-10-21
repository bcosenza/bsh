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
	: BoidGroupAvoidanceSH(pos, vel, color, simP, 2)   // two groups
{
	simTimeDisc[0] = "Two groups avoidance with SH";
}

BoidGroupAvoidanceSH::BoidGroupAvoidanceSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP, int groups)
	: BoidGroupAvoidance(pos, vel, color, simP), numGroups(groups)
{
	// one order-3 SH coefficient set per group, stored on the device
	d_groupSH = sycl::malloc_device<float>(numGroups * SH_COEFFS, queue);
	queue.wait();
}

BoidGroupAvoidanceSH::~BoidGroupAvoidanceSH(){
	queue.wait();
	sycl::free(d_groupSH, queue);
	// ~BoidGridBase / ~BoidModelSimpleSYCL free the rest
}

void BoidGroupAvoidanceSH::projectGroupSH(){
	const int  n  = num;
	const int  ng = numGroups;
	const sf4* vel     = d_vel;       // current state, original boid order
	float*     groupSH = d_groupSH;

	queue.fill(groupSH, 0.0f, (size_t)ng * SH_COEFFS);

	const size_t WG     = 128;
	const size_t global = ((size_t)n + WG - 1) / WG * WG;

	queue.parallel_for(sycl::nd_range<1>(global, WG), [=](sycl::nd_item<1> it){
		const int i = (int)it.get_global_id(0);

		// this boid's contribution, pre-divided by its group size so the totals are
		// group averages (keeps the reconstructed field ~O(1) whatever the size)
		float c[SH_COEFFS];
		for (int k = 0; k < SH_COEFFS; k++) c[k] = 0.0f;
		int g = -1;                                  // padding items contribute nothing
		if (i < n){
			g = groupOf(i, n, ng);
			const sf4 dir = safeNormalize(vel[i]);
			float b[8];
			shEval3(dir.x(), dir.y(), dir.z(), b);
			const float inv = 1.0f / (float)groupSize(g, n, ng);
			c[0] = SH_BAND0 * inv;
			for (int k = 0; k < 8; k++) c[1 + k] = b[k] * inv;
		}

		// reduce within the work-group, then one atomic add per coefficient (a
		// work-group may straddle a group boundary, hence the loop over groups)
		const auto wg = it.get_group();
		for (int gg = 0; gg < ng; gg++){
			for (int k = 0; k < SH_COEFFS; k++){
				const float s = sycl::reduce_over_group(wg, (g == gg) ? c[k] : 0.0f, sycl::plus<float>());
				if (it.get_local_id(0) == 0 && s != 0.0f){
					sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,
					                 sycl::access::address_space::global_space> total(groupSH[gg * SH_COEFFS + k]);
					total.fetch_add(s);
				}
			}
		}
	});
	// no wait: the in-order queue runs the grid build / interaction after this
}

void BoidGroupAvoidanceSH::simulate(float dt){
	const int         n    = num;
	const int         ng   = numGroups;             // boids split into ng contiguous index buckets
	const simParams_t sp   = simParams;

	const float cs   = cellSize;
	const int   dimX = gridDimX, dimY = gridDimY, dimZ = gridDimZ;
	const sf4   origin = gridOrigin;

	auto t0 = std::chrono::high_resolution_clock::now();

	// --- SH step 1: project each boid's heading into its group's SH basis (device) ---
	projectGroupSH();

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
	const int   ngLocal    = ng;
	const float shStrength = shAvoidStrength;

	queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		const int k = (int)idx[0];
		sf4 p = sortedPos[k];
		sf4 v = sortedVel[k];

		// local flocking over the 27 neighbouring cells -> steering vector
		FlockAccum acc = gridAccumulate(p, v, sortedPos, sortedVel,
		                                cellStart, cellEnd, cs, dimX, dimY, dimZ, origin);
		sf4 steer = finalizeSteer(acc, p, v, sp);

		// SH step 2: evaluate every OTHER group's SH field in this agent's direction
		// and reconstruct a summed velocity contribution
		const int orig  = (int)index[k];
		const sf4 shSteer = groupSHSteer(v, groupOf(orig, n, ngLocal), ngLocal, groupSH, shStrength);

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
