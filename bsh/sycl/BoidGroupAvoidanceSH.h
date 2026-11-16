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

	// SH step 1, entirely on the device: project every boid's heading (read from
	// d_vel, original boid order) into its group's averaged SH coefficients in
	// d_groupSH, with the projection kernel selected by shMode (Dirac or vMF).
	// Each work-group reduces its boids' contributions, then adds them to the
	// group totals with one atomic per coefficient. Derived models call it at the
	// start of their simulate().
	void projectGroupSH();
	// the projection kernel for one policy P (SHDirac / SHVonMisesFisher)
	template <class P> void projectGroupSHImpl(const P& policy);

public:
	// "SH: Dirac" or "SH: vMF (k=...)", for the overlay
	const char* getSHProjectionName();

protected:
	std::string shProjectionName;

	// group of boid i: boids are split into ng contiguous index buckets
	static int groupOf(int i, int n, int ng)
	{
		const int g = (int)(((long long)i * ng) / n);
		return (g < ng) ? g : ng - 1;
	}
	// number of boids in group g (boids i with ceil(g*n/ng) <= i < ceil((g+1)*n/ng))
	static int groupSize(int g, int n, int ng)
	{
		const long long lo = ((long long)g * n + ng - 1) / ng;
		const long long hi = ((long long)(g + 1) * n + ng - 1) / ng;
		return (int)(hi - lo);
	}

	// SH step 2 for one agent (device code): the summed steering reconstructed from
	// every group except the agent's own.
	static sf4 groupSHSteer(const sf4& v, int group, int ng, const float* groupSH, float strength)
	{
		sf4 s = sf4(0.0f);
		for (int og = 0; og < ng; og++){
			if (og == group) continue;
			s += shAvoidanceSteer(v, groupSH + og * SH_COEFFS, strength);
		}
		return s;
	}

	// weight of the "keep right" tie-break in shAvoidanceSteer: when two flows meet
	// exactly head-on there is no lateral component to steer away from, so each
	// agent turns to its own right; the groups then pass on opposite sides.
	static constexpr float SH_KEEP_RIGHT = 0.25f;

	// Reconstruct the other group's SH field in the agent's heading direction and
	// turn it into a sideways steering vector (device code). otherSH points at that
	// group's SH_COEFFS coefficients.
	//  - 'val' is the reconstructed field value: how strongly the other group heads
	//    along the agent's direction (negative ringing is clamped to zero);
	//  - the other group's dominant heading m is recovered from its band-1 coeffs;
	//  - 'oncoming' = (1 - m.h)/2 is 1 for a head-on flow and 0 for one moving the
	//    same way, so parallel groups do not push each other.
	// The push is perpendicular to the agent's heading h: away from the lateral part
	// of the other flow, plus a small keep-right bias for the head-on case. Pushing
	// along m instead would make head-on groups reverse.
	static sf4 shAvoidanceSteer(const sf4& v, const float* otherSH, float strength)
	{
		const sf4 h = safeNormalize(v);
		float b[8];
		shEval3(h.x(), h.y(), h.z(), b);

		float val = otherSH[0] * SH_BAND0;
		for (int k = 0; k < 8; k++)
			val += otherSH[1 + k] * b[k];

		// dominant heading of the other group from its band-1 (l=1) coefficients:
		// out[0]=-a*y, out[1]=a*z, out[2]=-a*x  =>  dir ~ (-c[3], -c[1], c[2])
		const sf4 m = safeNormalize(sf4(-otherSH[3], -otherSH[1], otherSH[2], 0.0f));

		const float mh       = sycl::dot(m, h);
		const float oncoming = 0.5f * (1.0f - mh);
		const sf4   lateral  = m - h * mh;                       // other flow, across our heading
		const sf4   right    = sf4(-h.z(), 0.0f, h.x(), 0.0f);   // h x up (up = +y)
		const sf4   side     = safeNormalize(right * SH_KEEP_RIGHT - lateral);

		return side * (strength * sycl::fmax(val, 0.0f) * oncoming);
	}
};

#endif // _BOIDGROUPAVOIDANCESH_H_
