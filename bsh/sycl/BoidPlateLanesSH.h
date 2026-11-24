// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDPLATELANESSH_H_
#define _BOIDPLATELANESSH_H_

#include "BoidThreeColumnsSH.h"
#include "AccuracyProbe.h"

/*
	SH accuracy scene 9: plate lanes, for the obstacle SH term.

	AccLanes::LANES independent lanes, each with one group approaching a thin plate
	(PlateLanes) at the lane's incidence angle. The obstacle SH engine of
	BoidThreeColumnsSH is reused unchanged (both kernels' coefficient sets, vMF
	re-projection when k changes); there is no group SH term. Each agent only
	feels its own lane's plate, so lanes are independent, and each plate point's
	1/r^2 weight is tapered to zero at AccLanes::PLATE_RANGE so the plate acts
	locally (see AccuracyLanes.h). Each agent seeks a goal behind its plate
	(AccLanes::PLATE_PATH_WEIGHT), so the plate field has to hold the group off.

	The obstacle term can be driven by the Dirac SH, the vMF SH or the brute-force
	reference (SHMode::Exact): the exact vMF field of each point's repulsion
	vectors, pi * sum_m c_m K(d . c_m / |c_m|) with K the normalised vMF density of
	the same k, with the same 1/r^2 falloff and strength. Thin plates expose the
	back lobe of the truncated expansion: a face's points, seen through the plate,
	can pull an agent toward it.

	With accuracy recording on, each step evaluates all three on the same state and
	accumulates per-lane errors, the outward component of each push (negative =
	toward the plate), the minimum clearance to the plate (negative inside it) and
	penetrations (crossings of the plate's mid-plane within its extent).
*/
class BoidPlateLanesSH : public BoidThreeColumnsSH
{
public:
	// cor/start/end/posObst from PlateLanes::getObstacleForce; lane L owns obstacle
	// points [L * pointsPerLane, (L+1) * pointsPerLane); one plate per lane at
	// plateCentres[L] with half extents plateHalf; goal: per-boid goal behind the plate
	BoidPlateLanesSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
	                 std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end,
	                 std::vector<Vec4> posObst, int pointsPerLane,
	                 const std::vector<Vec4>& plateCentres, Vec4 plateHalf, std::vector<Vec4> goal);
	virtual ~BoidPlateLanesSH();

	void simulate(float dt);

	bool supportsExactReference() { return true; }
	bool hasAccuracyMetrics() { return true; }
	void setAccuracyRecording(bool on) { recording = on; }
	std::string getAccuracyCSV(const std::string& extraHeader, const std::string& extraValues, bool header);
	std::string getAccuracySummary();

protected:
	int pointsPerLane;
	sf4* d_cor;                  // all repulsion vectors (for the reference)
	unsigned int* d_corStart;    // per obstacle point: its vectors [start, end)
	unsigned int* d_corEnd;
	sf4* d_plateCentre;          // per lane
	sf4  plateHalf;

	AccuracyProbe probe;
	bool recording;
};

#endif // _BOIDPLATELANESSH_H_
