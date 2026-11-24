// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidCrossingLanesSH.h"
#include "AccuracyLanes.h"
#include "vectorTypes.h"
#include <chrono>

BoidCrossingLanesSH::BoidCrossingLanesSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
                                         std::vector<Vec4> goal)
	: BoidGroupAvoidanceSH(pos, vel, color, simP, 2 * AccLanes::LANES)   // groups 2L, 2L+1 = lane L
{
	simTimeDisc[0] = "SH accuracy: crossing lanes";
	recording = false;

	d_groupSHvmf = sycl::malloc_device<float>(numGroups * SH_COEFFS, queue);
	d_snapPos    = sycl::malloc_device<sf4>(num, queue);
	d_snapHead   = sycl::malloc_device<sf4>(num, queue);
	d_goal       = sycl::malloc_device<sf4>(num, queue);
	queue.memcpy(d_goal, goal.data(), (size_t)num * sizeof(Vec4));   // Vec4 == sf4 layout

	std::vector<int> laneOf(num);
	for (int i = 0; i < num; i++) laneOf[i] = groupOf(i, num, numGroups) / 2;
	probe.init(queue, laneOf,
	           std::vector<float>(AccLanes::CROSS_DEG, AccLanes::CROSS_DEG + AccLanes::LANES),
	           "cross_deg");
	queue.wait();
}

BoidCrossingLanesSH::~BoidCrossingLanesSH(){
	queue.wait();
	sycl::free(d_groupSHvmf, queue);
	sycl::free(d_snapPos, queue);
	sycl::free(d_snapHead, queue);
	sycl::free(d_goal, queue);
	probe.release(queue);
}

void BoidCrossingLanesSH::simulate(float dt){
	const int         n  = num;
	const int         ng = numGroups;
	const simParams_t sp = simParams;

	const float cs   = cellSize;
	const int   dimX = gridDimX, dimY = gridDimY, dimZ = gridDimZ;
	const sf4   origin = gridOrigin;

	// which variants this step needs: the driver, and all three when recording
	const int  mode    = (int)shMode;
	const bool rec     = recording;
	const bool needD   = rec || shMode == SHMode::Dirac;
	const bool needV   = rec || shMode == SHMode::VMF;
	const bool needRef = rec || shMode == SHMode::Exact;

	auto t0 = std::chrono::high_resolution_clock::now();
	resetPhaseEvents();

	// SH step 1 with the kernels needed: Dirac into d_groupSH, vMF into d_groupSHvmf
	if (needD && needV){
		PhaseEvents a = projectGroupSHImpl(SHDirac(), d_groupSH);
		PhaseEvents b = projectGroupSHImpl(vmfPolicy(), d_groupSHvmf);
		evProject.set(a.first, b.last);
	}
	else if (needD) evProject = projectGroupSHImpl(SHDirac(), d_groupSH);
	else if (needV) evProject = projectGroupSHImpl(vmfPolicy(), d_groupSHvmf);

	// the reference reads the partner's positions/headings of this step (original order)
	sf4* snapPos  = d_snapPos;
	sf4* snapHead = d_snapHead;
	if (needRef){
		const sf4* pos = d_pos;
		const sf4* vel = d_vel;
		queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
			const int i = (int)idx[0];
			snapPos[i]  = pos[i];
			snapHead[i] = safeNormalize(vel[i]);
		});
	}

	buildGrid();

	sf4* sortedPos = d_pos_out;
	sf4* sortedVel = d_vel_out;
	sf4* outPos    = d_pos;
	sf4* outVel    = d_vel;
	int* cellStart = d_cellStart;
	int* cellEnd   = d_cellEnd;
	unsigned int* index = d_index;
	const float* shD = d_groupSH;
	const float* shV = d_groupSHvmf;
	float*      recBuf  = probe.device();
	const float lambda  = shAvoidStrength;
	const float kRef    = vmfK;
	const float normRef = vmfDensityNorm(vmfK);
	const float sepR    = separationRadius;
	const int   REC     = AccuracyProbe::REC;
	const sf4*  goal    = d_goal;
	const float wPath   = AccLanes::PATH_WEIGHT;

	sycl::event ev = queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		const int k = (int)idx[0];
		sf4 p = sortedPos[k];
		sf4 v = sortedVel[k];

		// local flocking over the 27 neighbouring cells -> steering vector
		FlockAccum acc = gridAccumulate(p, v, sortedPos, sortedVel,
		                                cellStart, cellEnd, cs, dimX, dimY, dimZ, origin);
		const sf4 steer = finalizeSteer(acc, p, v, sp);

		// the agent only feels its lane partner (group 2L <-> 2L+1)
		const int orig    = (int)index[k];
		const int partner = groupOf(orig, n, ng) ^ 1;
		const sf4 h = safeNormalize(v);

		float fD = 0.0f, fV = 0.0f, fR = 0.0f;
		sf4 sD = sf4(0.0f), sV = sf4(0.0f), sR = sf4(0.0f);
		if (needD){
			const float* c = shD + partner * SH_COEFFS;
			fD = shFieldValue(h, c);
			sD = sidewaysSteer(h, fD, shDominantHeading(c), lambda);
		}
		if (needV){
			const float* c = shV + partner * SH_COEFFS;
			fV = shFieldValue(h, c);
			sV = sidewaysSteer(h, fV, shDominantHeading(c), lambda);
		}

		// brute-force reference over the partner group, plus the behaviour metrics
		float minDist = 1e30f, contacts = 0.0f;
		if (needRef){
			const int lo = groupStart(partner, n, ng);
			const int hi = groupStart(partner + 1, n, ng);
			float fsum = 0.0f;
			sf4 msum = sf4(0.0f);
			for (int j = lo; j < hi; j++){
				const sf4 hj = snapHead[j];
				fsum += normRef * sycl::exp(kRef * (sycl::dot(h, hj) - 1.0f));
				msum += hj;
				sf4 d = snapPos[j] - p;
				d.w() = 0.0f;
				const float dist = sycl::length(d);
				minDist = sycl::fmin(minDist, dist);
				if (dist < sepR) contacts += 1.0f;
			}
			fR = (hi > lo) ? fsum / (float)(hi - lo) : 0.0f;
			sR = sidewaysSteer(h, fR, safeNormalize(msum), lambda);
		}

		const sf4 grp = (mode == (int)SHMode::Exact) ? sR : (mode == (int)SHMode::VMF) ? sV : sD;
		// goal seeking keeps the group on its lane course
		const sf4 path = safeNormalize(goal[orig] - p) * wPath;
		advanceBoid(p, v, steer + grp + path, sp, dt);

		outPos[orig] = p;
		outVel[orig] = v;

		if (rec){
			float* r = recBuf + (size_t)orig * REC;
			r[AccuracyProbe::S_REF] = sR.x(); r[AccuracyProbe::S_REF + 1] = sR.y(); r[AccuracyProbe::S_REF + 2] = sR.z();
			r[AccuracyProbe::S_D]   = sD.x(); r[AccuracyProbe::S_D + 1]   = sD.y(); r[AccuracyProbe::S_D + 2]   = sD.z();
			r[AccuracyProbe::S_V]   = sV.x(); r[AccuracyProbe::S_V + 1]   = sV.y(); r[AccuracyProbe::S_V + 2]   = sV.z();
			r[AccuracyProbe::AUX_REF] = fR;
			r[AccuracyProbe::AUX_D]   = fD;
			r[AccuracyProbe::AUX_V]   = fV;
			r[AccuracyProbe::B_DIST]  = minDist;
			r[AccuracyProbe::B_EVENT] = 0.5f * contacts;   // each pair is seen from both sides
		}
	});
	evInteract.set(ev, ev);
	queue.wait();
	collectPhaseTimes();
	if (rec) probe.collect(queue);

	auto t1 = std::chrono::high_resolution_clock::now();
	lastSimTimeMs = (long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

	bool pingPong = ((helper++ % 2) == 0);
	uploadState(outPos, outVel, pingPong);
}

static const char* driverName(SHMode m){
	return (m == SHMode::Exact) ? "exact" : (m == SHMode::VMF) ? "vmf" : "dirac";
}

std::string BoidCrossingLanesSH::getAccuracyCSV(const std::string& extraHeader, const std::string& extraValues, bool header){
	return probe.csv(extraHeader, extraValues, driverName(shMode), vmfK, header);
}

std::string BoidCrossingLanesSH::getAccuracySummary(){
	std::stringstream t;
	t << "accuracy: crossing lanes (group SH), driver " << driverName(shMode) << ", k " << vmfK;
	return probe.summary(t.str(), "min dist", "contacts");
}
