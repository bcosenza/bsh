// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDGROUPAVOIDANCESH_H_
#define _BOIDGROUPAVOIDANCESH_H_

#include "BoidGroupAvoidance.h"

/*
	Two-groups model with long-range group avoidance via spherical harmonics (SH).

	On top of BoidGroupAvoidance (grid-accelerated local flocking, two groups) it
	keeps, per group, one order-3 SH basis that accumulates the heading directions
	of all the group's boids. The coefficients live on the device (malloc_device).

	Each simulate() adds two SH steps around the usual grid interaction:
	  (1) project every boid's velocity direction into its group's SH basis;
	  (2) after the local flocking interaction, evaluate the OTHER group's SH field
	      in the agent's heading direction and reconstruct a velocity contribution
	      (added to the local steering before integration).

	SH math ported from the OpenCL SHEval3 (P.-P. Sloan, order-3 real SH: band 0 is
	the constant 0.2820947917738781, bands 1-2 give the 8 remaining coefficients).
*/
class BoidGroupAvoidanceSH : public BoidGroupAvoidance
{
public:
	BoidGroupAvoidanceSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP);
	virtual ~BoidGroupAvoidanceSH();

	void simulate(float dt);   // adds the SH project + reconstruct steps

protected:
	// N-group variant used by derived models (e.g. the 4-group version). Boids are
	// split into 'groups' contiguous index buckets, matching the coloured groups of
	// the init placements.
	BoidGroupAvoidanceSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP, int groups);

	static constexpr int   SH_COEFFS = 9;                         // order-3 SH: band0 + 8
	// overall strength (and sign) of the cross-group SH steering. This is the main
	// knob for the avoidance behaviour -- tune it (and its sign) by eye.
	static constexpr float shAvoidStrength = 3.0f;

	int numGroups;                // number of interacting groups (2, 4, ...)

	// per-group accumulated (averaged) SH coefficients on device:
	// group g, coefficient k at d_groupSH[g * SH_COEFFS + k]. Index 0 is band 0.
	// (shEval3 and SH_BAND0 are inherited from BoidModelSimpleSYCL.)
	float* d_groupSH;

	// Reconstruct the other group's SH field in the agent's heading direction and
	// turn it into a steering vector (device code). otherSH points at that group's
	// SH_COEFFS coefficients. The scalar 'val' is the reconstructed field value
	// (how strongly the other group heads along the agent's direction); it scales a
	// push along the other group's dominant heading (recovered from its band-1
	// coefficients). Sign/strength of 'strength' select attraction vs avoidance.
	static sf4 shAvoidanceSteer(const sf4& v, const float* otherSH, float strength)
	{
		sf4 dir = safeNormalize(v);
		float b[8];
		shEval3(dir.x(), dir.y(), dir.z(), b);

		float val = otherSH[0] * SH_BAND0;
		for (int k = 0; k < 8; k++)
			val += otherSH[1 + k] * b[k];

		// dominant heading of the other group from its band-1 (l=1) coefficients:
		// out[0]=-a*y, out[1]=a*z, out[2]=-a*x  =>  dir ~ (-c[3], -c[1], c[2])
		sf4 meanDir = safeNormalize(sf4(-otherSH[3], -otherSH[1], otherSH[2], 0.0f));
		return meanDir * (val * strength);
	}
};

#endif // _BOIDGROUPAVOIDANCESH_H_
