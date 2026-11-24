// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidPlateLanesSH.h"
#include "AccuracyLanes.h"
#include "vectorTypes.h"
#include <chrono>

BoidPlateLanesSH::BoidPlateLanesSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
                                   std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end,
                                   std::vector<Vec4> posObst, int pointsPerLane_,
                                   const std::vector<Vec4>& plateCentres, Vec4 half, std::vector<Vec4> goal)
	// one group per lane; the obstacle SH engine projects the plates with both kernels
	: BoidThreeColumnsSH(pos, vel, color, simP, cor, start, end, posObst, AccLanes::LANES)
{
	simTimeDisc[0] = "SH accuracy: plate lanes";
	obstStrength = AccLanes::PLATE_STRENGTH;
	enableGoalSeeking(goal, AccLanes::PLATE_PATH_WEIGHT);
	recording = false;
	pointsPerLane = pointsPerLane_;
	plateHalf = sf4(half.x, half.y, half.z, 0.0f);

	d_cor          = sycl::malloc_device<sf4>(cor.size(), queue);
	d_corStart     = sycl::malloc_device<unsigned int>(start.size(), queue);
	d_corEnd       = sycl::malloc_device<unsigned int>(end.size(), queue);
	d_plateCentre  = sycl::malloc_device<sf4>(plateCentres.size(), queue);
	queue.memcpy(d_cor, cor.data(), cor.size() * sizeof(Vec4));   // Vec4 == sf4 layout
	queue.memcpy(d_corStart, start.data(), start.size() * sizeof(unsigned int));
	queue.memcpy(d_corEnd, end.data(), end.size() * sizeof(unsigned int));
	queue.memcpy(d_plateCentre, plateCentres.data(), plateCentres.size() * sizeof(Vec4));

	std::vector<int> laneOf(num);
	for (int i = 0; i < num; i++) laneOf[i] = groupOf(i, num, numGroups);
	probe.init(queue, laneOf,
	           std::vector<float>(AccLanes::PLATE_DEG, AccLanes::PLATE_DEG + AccLanes::LANES),
	           "incidence_deg");
	queue.wait();
}

BoidPlateLanesSH::~BoidPlateLanesSH(){
	queue.wait();
	sycl::free(d_cor, queue);
	sycl::free(d_corStart, queue);
	sycl::free(d_corEnd, queue);
	sycl::free(d_plateCentre, queue);
	probe.release(queue);
}

void BoidPlateLanesSH::simulate(float dt){
	const int         n  = num;
	const int         ng = numGroups;
	const simParams_t sp = simParams;

	const float cs   = cellSize;
	const int   dimX = gridDimX, dimY = gridDimY, dimZ = gridDimZ;
	const sf4   origin = gridOrigin;

	const int  mode    = (int)shMode;
	const bool rec     = recording;
	const bool needD   = rec || shMode == SHMode::Dirac;
	const bool needV   = rec || shMode == SHMode::VMF;
	const bool needRef = rec || shMode == SHMode::Exact;

	auto t0 = std::chrono::high_resolution_clock::now();
	resetPhaseEvents();

	// no group SH in this scene: grid, then the interaction
	buildGrid();

	sf4* sortedPos = d_pos_out;
	sf4* sortedVel = d_vel_out;
	sf4* outPos    = d_pos;
	sf4* outVel    = d_vel;
	int* cellStart = d_cellStart;
	int* cellEnd   = d_cellEnd;
	unsigned int* index = d_index;
	const float* shD = d_obstacleSH;
	const float* shV = d_obstacleSHvmf;
	const sf4*   posObst = d_posObst;
	const sf4*   corv    = d_cor;
	const unsigned int* cStart = d_corStart;
	const unsigned int* cEnd   = d_corEnd;
	const sf4*   plateC  = d_plateCentre;
	const sf4    half    = plateHalf;
	const int    ppl     = pointsPerLane;
	const float  obstF   = obstStrength;
	const float  kRef    = vmfK;
	const float  refScale = (float)M_PI * vmfDensityNorm(vmfK);   // pi * K normalisation
	const float  range2   = AccLanes::PLATE_RANGE * AccLanes::PLATE_RANGE;
	const sf4*   goal     = d_goal;
	const float  wPath    = pathWeight;
	float*       recBuf  = probe.device();
	const int    REC     = AccuracyProbe::REC;

	sycl::event ev = queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		const int k = (int)idx[0];
		sf4 p = sortedPos[k];
		sf4 v = sortedVel[k];

		FlockAccum acc = gridAccumulate(p, v, sortedPos, sortedVel,
		                                cellStart, cellEnd, cs, dimX, dimY, dimZ, origin);
		const sf4 steer = finalizeSteer(acc, p, v, sp);

		// the agent only feels its own lane's plate
		const int orig = (int)index[k];
		const int lane = groupOf(orig, n, ng);
		sf4 gD = sf4(0.0f), gV = sf4(0.0f), gR = sf4(0.0f);
		for (int i = lane * ppl; i < (lane + 1) * ppl; i++){
			sf4 d = p - posObst[i];          // from the obstacle point toward the boid
			d.w() = 0.0f;
			const float dist2 = d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
			if (dist2 < 1e-3f || dist2 >= range2) continue;
			// 1/r^2 falloff, smoothly tapered to zero at the plate range
			const float taper = 1.0f - dist2 / range2;
			const float w = taper * taper / dist2;
			const sf4 dir = d / sycl::sqrt(dist2);
			if (needD) gD += reconstructObstacle(dir, shD + i * OBST_STRIDE) * w;
			if (needV) gV += reconstructObstacle(dir, shV + i * OBST_STRIDE) * w;
			if (needRef){
				// exact vMF field of this point's vectors: pi * sum_m c_m K(dir . c_m/|c_m|)
				sf4 g = sf4(0.0f);
				for (unsigned int m = cStart[i]; m < cEnd[i]; m++){
					const sf4 c = corv[m];
					const float len = sycl::length(c);
					if (len < 1e-6f) continue;
					g += c * sycl::exp(kRef * (sycl::dot(dir, c) / len - 1.0f));
				}
				gR += g * (refScale * w);
			}
		}
		const sf4 sD = gD * obstF, sV = gV * obstF, sR = gR * obstF;

		const sf4 obst = (mode == (int)SHMode::Exact) ? sR : (mode == (int)SHMode::VMF) ? sV : sD;
		// goal seeking toward the point behind the plate
		const sf4 path = safeNormalize(goal[orig] - p) * wPath;
		const sf4 pOld = p;
		advanceBoid(p, v, steer + obst + path, sp, dt);

		outPos[orig] = p;
		outVel[orig] = v;

		if (rec){
			const sf4 c = plateC[lane];
			// outward direction on the agent's side of the plate (along the normal)
			const float side = (pOld.x() >= c.x()) ? 1.0f : -1.0f;
			// signed distance to the plate box after the move (negative inside)
			const float qx = sycl::fabs(p.x() - c.x()) - half.x();
			const float qy = sycl::fabs(p.y() - c.y()) - half.y();
			const float qz = sycl::fabs(p.z() - c.z()) - half.z();
			const float ox = sycl::fmax(qx, 0.0f), oy = sycl::fmax(qy, 0.0f), oz = sycl::fmax(qz, 0.0f);
			const float clearance = sycl::sqrt(ox * ox + oy * oy + oz * oz)
			                      + sycl::fmin(sycl::fmax(qx, sycl::fmax(qy, qz)), 0.0f);
			// penetration: the step crossed the mid-plane within the plate's extent
			float pen = 0.0f;
			const float a = pOld.x() - c.x(), b = p.x() - c.x();
			if (a * b < 0.0f){
				const float t = a / (a - b);
				const sf4 x = pOld + (p - pOld) * t;
				if (sycl::fabs(x.y() - c.y()) <= half.y() && sycl::fabs(x.z() - c.z()) <= half.z())
					pen = 1.0f;
			}

			float* r = recBuf + (size_t)orig * REC;
			r[AccuracyProbe::S_REF] = sR.x(); r[AccuracyProbe::S_REF + 1] = sR.y(); r[AccuracyProbe::S_REF + 2] = sR.z();
			r[AccuracyProbe::S_D]   = sD.x(); r[AccuracyProbe::S_D + 1]   = sD.y(); r[AccuracyProbe::S_D + 2]   = sD.z();
			r[AccuracyProbe::S_V]   = sV.x(); r[AccuracyProbe::S_V + 1]   = sV.y(); r[AccuracyProbe::S_V + 2]   = sV.z();
			r[AccuracyProbe::AUX_REF] = sR.x() * side;
			r[AccuracyProbe::AUX_D]   = sD.x() * side;
			r[AccuracyProbe::AUX_V]   = sV.x() * side;
			r[AccuracyProbe::B_DIST]  = clearance;
			r[AccuracyProbe::B_EVENT] = pen;
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

std::string BoidPlateLanesSH::getAccuracyCSV(const std::string& extraHeader, const std::string& extraValues, bool header){
	return probe.csv(extraHeader, extraValues, driverName(shMode), vmfK, header);
}

std::string BoidPlateLanesSH::getAccuracySummary(){
	std::stringstream t;
	t << "accuracy: plate lanes (obstacle SH), driver " << driverName(shMode) << ", k " << vmfK
	  << ", strength " << obstStrength;
	return probe.summary(t.str(), "clearance", "penetr.");
}
