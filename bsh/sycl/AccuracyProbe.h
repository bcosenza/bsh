// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _ACCURACYPROBE_H_
#define _ACCURACYPROBE_H_

#include <sycl/sycl.hpp>
#include <string>
#include <vector>

/*
	Per-lane accuracy statistics of the SH accuracy scenes (8, 9).

	Each step the model's interaction kernel writes one record of REC floats per
	agent (original boid order) into device(): the steering term under study as
	computed by the brute-force reference (S_REF), the Dirac SH (S_D) and the vMF
	SH (S_V), all on the same state; a scalar per variant (AUX_*: the group field
	value f in scene 8, the outward component of the obstacle push in scene 9);
	and two behaviour values (B_DIST: distance to the partner group / clearance to
	the plate, B_EVENT: close contacts / penetrations this step). collect() copies
	the records to the host and accumulates them per lane.

	Errors are reported against the reference, per lane:
	  nRMSE      = RMS |s - s_ref| / RMS |s_ref|
	  fit nRMSE  = the same after the best scalar gain g = <s, s_ref> / <s, s>, i.e.
	               the error that a retuned strength could not remove (shape error)
	  angle      = mean angle between s and s_ref where both are non-zero
	  missed / spurious = share of agent-steps where only the reference / only the
	               SH steering is non-zero
	  neg        = share of agent-steps with a negative AUX value (scene 9: push
	               toward the plate)
	  field nRMSE = whole-field error over the sampled directions (scene 8:
	               the composite preference field), and the share of directions
	               where the SH field is negative
	B_DIST is reported as its minimum and its mean over the run.
*/
class AccuracyProbe
{
public:
	enum : int { S_REF = 0, S_D = 3, S_V = 6, AUX_REF = 9, AUX_D = 10, AUX_V = 11,
	             B_DIST = 12, B_EVENT = 13,
	             // optional whole-field comparison (scene 8): per agent, the sum over the
	             // sampled directions of (P_sh - P_ref)^2 and P_ref^2, and the share of
	             // directions where P_sh < 0. Zero when a scene does not use it.
	             FIELD_SE_D = 14, FIELD_SE_V = 15, FIELD_SS_REF = 16, FIELD_NEG_D = 17, FIELD_NEG_V = 18,
	             REC = 19 };

	AccuracyProbe() : d_rec(nullptr), steps(0) {}

	// laneOfAgent[i]: lane of boid i (-1: not recorded); laneValue[L]: the lane's
	// value (e.g. an angle in degrees), reported under the column name laneLabel
	void init(sycl::queue& q, const std::vector<int>& laneOfAgent,
	          const std::vector<float>& laneValue, const char* laneLabel);
	void release(sycl::queue& q);

	float* device() const { return d_rec; }

	// copy the last step's records (the queue must have drained) and accumulate them
	void collect(sycl::queue& q);

	// CSV rows (one per lane): extraValues, lane, angle, driver, k, then the statistics
	std::string csv(const std::string& extraHeader, const std::string& extraValues,
	                const std::string& driver, float k, bool header) const;
	// human-readable table of the main columns
	std::string summary(const std::string& title, const std::string& distLabel,
	                    const std::string& eventLabel) const;

private:
	// accumulated error of one SH variant (Dirac or vMF) against the reference
	struct Err {
		double dot = 0, ss = 0, se = 0, maxe = 0;   // <s,s_ref>, <s,s>, sum |s-s_ref|^2, max |s-s_ref|
		double ang = 0, angN = 0;                    // sum of angles (deg), count
		double missed = 0, spurious = 0;             // counts
		double auxErr = 0, neg = 0;                  // sum |aux - aux_ref|, count aux < 0
	};
	struct Lane {
		double n = 0;                                // agent-steps
		double ssRef = 0, auxRef = 0, negRef = 0;
		Err d, v;
		double minDist = 1e30, sumDist = 0, events = 0;
		double fieldSsRef = 0, fieldSeD = 0, fieldSeV = 0, fieldNegD = 0, fieldNegV = 0;
	};

	float* d_rec;
	long steps;
	std::vector<int> laneOf;
	std::vector<float> laneVal;
	std::string label;
	std::vector<float> host;
	std::vector<Lane> lanes;

	static void accumulateErr(Err& e, const float* s, const float* r, float aux, float auxRef);
};

#endif // _ACCURACYPROBE_H_
