// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidThreeColumnsSH.h"
#include "vectorTypes.h"
#include <cmath>
#include <chrono>

BoidThreeColumnsSH::BoidThreeColumnsSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
                                       std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end,
                                       std::vector<Vec4> posObst)
	: BoidGridBase(pos, vel, color, simP)
{
	simTimeDisc[0] = "Three columns avoidance with SH";
	obstStrength = 3.0f;
	numObst = (int)posObst.size();

	d_obstacleSH = sycl::malloc_device<float>((size_t)numObst * OBST_STRIDE, queue);
	d_posObst    = sycl::malloc_device<sf4>(numObst, queue);

	// --- project each obstacle point's repulsion vectors into a vector-valued SH
	// field, once at setup (the geometry is static). For every corner vector c of
	// the point, accumulate SHEval3(dir(c)) weighted by each component of c. ---
	std::vector<float> sh((size_t)numObst * OBST_STRIDE, 0.0f);
	for (int i = 0; i < numObst; i++){
		float* c = sh.data() + (size_t)i * OBST_STRIDE;
		for (unsigned int j = start[i]; j < end[i]; j++){
			const Vec4& vv = cor[j];
			float len = std::sqrt(vv.x * vv.x + vv.y * vv.y + vv.z * vv.z);
			if (len < 1e-6f) continue;
			float b[8];
			shEval3(vv.x / len, vv.y / len, vv.z / len, b);
			for (int k = 0; k < 8; k++){
				c[0  + k] += b[k] * vv.x;
				c[8  + k] += b[k] * vv.y;
				c[16 + k] += b[k] * vv.z;
			}
			c[24] += SH_BAND0 * vv.x;
			c[25] += SH_BAND0 * vv.y;
			c[26] += SH_BAND0 * vv.z;
		}
	}

	queue.memcpy(d_obstacleSH, sh.data(), (size_t)numObst * OBST_STRIDE * sizeof(float));
	queue.memcpy(d_posObst, posObst.data(), (size_t)numObst * sizeof(Vec4));   // Vec4 == sf4 layout
	queue.wait();
}

BoidThreeColumnsSH::~BoidThreeColumnsSH(){
	queue.wait();
	sycl::free(d_obstacleSH, queue);
	sycl::free(d_posObst, queue);
	// ~BoidGridBase / ~BoidModelSimpleSYCL free the rest
}

void BoidThreeColumnsSH::simulate(float dt){
	const int         n  = num;
	const simParams_t sp = simParams;

	const float cs   = cellSize;
	const int   dimX = gridDimX, dimY = gridDimY, dimZ = gridDimZ;
	const sf4   origin = gridOrigin;

	auto t0 = std::chrono::high_resolution_clock::now();

	// build the uniform grid (inherited, on-device)
	buildGrid();

	sf4* sortedPos = d_pos_out;
	sf4* sortedVel = d_vel_out;
	sf4* outPos    = d_pos;
	sf4* outVel    = d_vel;
	int* cellStart = d_cellStart;
	int* cellEnd   = d_cellEnd;
	unsigned int* index = d_index;
	float* obstacleSH = d_obstacleSH;
	sf4*   posObst    = d_posObst;
	const int   nObst  = numObst;
	const float obstF  = obstStrength;

	queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		const int k = (int)idx[0];
		sf4 p = sortedPos[k];
		sf4 v = sortedVel[k];

		// local flocking over the 27 neighbouring cells -> steering vector
		FlockAccum acc = gridAccumulate(p, v, sortedPos, sortedVel,
		                                cellStart, cellEnd, cs, dimX, dimY, dimZ, origin);
		sf4 steer = finalizeSteer(acc, p, v, sp);

		// obstacle-SH avoidance: sum the repulsion reconstructed from every obstacle
		// point, attenuated by 1/distance^2 (as in the OpenCL useSH obstacle loop)
		sf4 obst = sf4(0.0f);
		for (int i = 0; i < nObst; i++){
			sf4 d = p - posObst[i];          // from the obstacle point toward the boid
			d.w() = 0.0f;
			float dist2 = d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
			if (dist2 < 1e-3f) continue;
			float dist = sycl::sqrt(dist2);
			sf4 dir = d / dist;
			obst += reconstructObstacle(dir, obstacleSH + i * OBST_STRIDE) * (1.0f / dist2);
		}

		// combine local + obstacle steering, then integrate (shared dynamics)
		advanceBoid(p, v, steer + obst * obstF, sp, dt);

		const int orig = (int)index[k];
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
