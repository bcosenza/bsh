// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDCROSSINGLANESSH_H_
#define _BOIDCROSSINGLANESSH_H_

#include "BoidGroupAvoidanceSH.h"
#include "AccuracyProbe.h"

/*
	SH accuracy scene 8: crossing lanes, for the group SH term.

	AccLanes::LANES independent lanes (AccuracyLanes.h), each with two groups whose
	headings meet at the lane's crossing angle (0 = parallel control ... 180 =
	head-on). Groups are 2L and 2L+1; each group only feels its own lane partner's
	field, so a lane isolates one encounter angle.

	Every step projects the group SH with BOTH kernels (Dirac and vMF). The group
	term can be driven by either SH variant or by the brute-force reference
	(SHMode::Exact): the exact vMF density of the partner's headings,
	  f_ref(h) = 1/|o| sum_j K(h . v_j),  K = normalised vMF with the same k,
	and the partner's exact mean heading, fed through the same sideways-push
	formula (sidewaysSteer). With k_ref = k the vMF SH differs from it only by the
	order-3 truncation; Dirac by truncation plus the kernel.

	With accuracy recording on, each step also evaluates all three variants on the
	same state and accumulates per-lane errors (AccuracyProbe), plus the minimum
	distance between partner groups and their close contacts (< separation radius).
	Goal seeking (AccLanes::PATH_WEIGHT) keeps each group on its lane course.
*/
class BoidCrossingLanesSH : public BoidGroupAvoidanceSH
{
public:
	// goal: per-boid goal on the far side of the lane centre (keeps the course)
	BoidCrossingLanesSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
	                    std::vector<Vec4> goal);
	virtual ~BoidCrossingLanesSH();

	void simulate(float dt);

	bool supportsExactReference() { return true; }
	bool hasAccuracyMetrics() { return true; }
	void setAccuracyRecording(bool on) { recording = on; }
	std::string getAccuracyCSV(const std::string& extraHeader, const std::string& extraValues, bool header);
	std::string getAccuracySummary();

protected:
	float* d_groupSHvmf;   // vMF group coefficients (d_groupSH holds the Dirac ones)
	sf4*   d_snapPos;      // positions and unit headings of this step, original order,
	sf4*   d_snapHead;     // read by the reference (the interaction overwrites d_pos/d_vel)
	sf4*   d_goal;         // per-boid goal, original order (AccLanes::PATH_WEIGHT)

	AccuracyProbe probe;
	bool recording;
};

#endif // _BOIDCROSSINGLANESSH_H_
