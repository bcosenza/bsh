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
	: BoidThreeColumnsSH(pos, vel, color, simP, cor, start, end, posObst, 4)   // four groups
{
}

BoidThreeColumnsSH::BoidThreeColumnsSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
                                       std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end,
                                       std::vector<Vec4> posObst, int groups)
	: BoidGroupAvoidanceSH(pos, vel, color, simP, groups)   // grid + per-group SH
{
	simTimeDisc[0] = "Three columns avoidance with SH";
	obstStrength = 3.0f;
	d_goal = nullptr;      // goal seeking off unless enableGoalSeeking() is called
	pathWeight = 0.0f;
	numObst = (int)posObst.size();

	d_obstacleSH    = sycl::malloc_device<float>((size_t)numObst * OBST_STRIDE, queue);
	d_obstacleSHvmf = sycl::malloc_device<float>((size_t)numObst * OBST_STRIDE, queue);
	d_posObst       = sycl::malloc_device<sf4>(numObst, queue);

	obstCor = cor;
	obstStart = start;
	obstEnd = end;

	// project the static obstacle geometry once per kernel
	projectObstacles(SHDirac(), d_obstacleSH);
	projectObstacles(vmfPolicy(), d_obstacleSHvmf);
	vmfProjectedK = vmfK;

	queue.memcpy(d_posObst, posObst.data(), (size_t)numObst * sizeof(Vec4));   // Vec4 == sf4 layout
	queue.wait();
}

// Project each obstacle point's repulsion vectors into a vector-valued SH field:
// for every corner vector c of the point, project dir(c) with policy P, weighted
// by each component of c (separate X/Y/Z coefficient sets).
template <class P>
void BoidThreeColumnsSH::projectObstacles(const P& policy, float* dst){
	std::vector<float> sh((size_t)numObst * OBST_STRIDE, 0.0f);
	for (int i = 0; i < numObst; i++){
		float* c = sh.data() + (size_t)i * OBST_STRIDE;
		for (unsigned int j = obstStart[i]; j < obstEnd[i]; j++){
			const Vec4& vv = obstCor[j];
			float len = std::sqrt(vv.x * vv.x + vv.y * vv.y + vv.z * vv.z);
			if (len < 1e-6f) continue;
			const float comp[3] = { vv.x, vv.y, vv.z };
			for (int a = 0; a < 3; a++){
				float t[SH_COEFFS];
				shProject(vv.x / len, vv.y / len, vv.z / len, comp[a], policy, t);
				for (int k = 0; k < 8; k++) c[a * 8 + k] += t[1 + k];
				c[24 + a] += t[0];
			}
		}
	}
	queue.memcpy(dst, sh.data(), sh.size() * sizeof(float)).wait();
}

void BoidThreeColumnsSH::setSHProjection(int mode, float k){
	BoidGroupAvoidanceSH::setSHProjection(mode, k);
	if (vmfK != vmfProjectedK){
		projectObstacles(vmfPolicy(), d_obstacleSHvmf);
		vmfProjectedK = vmfK;
	}
}

BoidThreeColumnsSH::~BoidThreeColumnsSH(){
	queue.wait();
	sycl::free(d_obstacleSH, queue);
	sycl::free(d_obstacleSHvmf, queue);
	sycl::free(d_posObst, queue);
	if (d_goal) sycl::free(d_goal, queue);
	// ~BoidGroupAvoidanceSH frees the group SH; ~BoidGridBase / ~BoidModelSimpleSYCL the rest
}

void BoidThreeColumnsSH::enableGoalSeeking(const std::vector<Vec4>& goal, float weight){
	if (d_goal == nullptr)
		d_goal = sycl::malloc_device<sf4>(num, queue);
	queue.memcpy(d_goal, goal.data(), (size_t)num * sizeof(Vec4)).wait();   // Vec4 == sf4 layout
	pathWeight = weight;
}

void BoidThreeColumnsSH::simulate(float dt){
	const int         n  = num;
	const simParams_t sp = simParams;

	const float cs   = cellSize;
	const int   dimX = gridDimX, dimY = gridDimY, dimZ = gridDimZ;
	const sf4   origin = gridOrigin;

	auto t0 = std::chrono::high_resolution_clock::now();
	resetPhaseEvents();

	// project each boid's heading into its group's SH basis (inherited, on-device)
	projectGroupSH();

	// build the uniform grid (inherited, on-device)
	buildGrid();

	sf4* sortedPos = d_pos_out;
	sf4* sortedVel = d_vel_out;
	sf4* outPos    = d_pos;
	sf4* outVel    = d_vel;
	int* cellStart = d_cellStart;
	int* cellEnd   = d_cellEnd;
	unsigned int* index = d_index;
	float* obstacleSH = (shMode == SHMode::VMF) ? d_obstacleSHvmf : d_obstacleSH;
	sf4*   posObst    = d_posObst;
	const int   nObst  = numObst;
	const float obstF  = obstStrength;
	float*      groupSH    = d_groupSH;
	const int   ng         = numGroups;
	const float shStrength = shAvoidStrength;
	const sf4*  goal       = d_goal;
	const float wPath      = pathWeight;

	sycl::event ev = queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
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

		// long-range avoidance of the other agent groups (one SH per group)
		const int orig = (int)index[k];
		const sf4 grp  = groupSHSteer(v, groupOf(orig, n, ng), ng, groupSH, shStrength);

		// goal seeking (the OpenCL path term), only where a scene enables goals
		sf4 path = sf4(0.0f);
		if (wPath > 0.0f)
			path = safeNormalize(goal[orig] - p) * wPath;

		// combine local + group + goal + obstacle steering, then integrate (shared dynamics)
		advanceBoid(p, v, steer + grp + path + obst * obstF, sp, dt);

		outPos[orig] = p;
		outVel[orig] = v;
	});
	evInteract.set(ev, ev);
	queue.wait();
	collectPhaseTimes();

	auto t1 = std::chrono::high_resolution_clock::now();
	lastSimTimeMs = (long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

	// draw: upload into the ping-pong VBO the base's getPosVAO() renders this frame
	bool pingPong = ((helper++ % 2) == 0);
	uploadState(outPos, outVel, pingPong);
}
