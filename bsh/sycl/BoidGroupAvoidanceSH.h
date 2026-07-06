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
	static constexpr int   SH_COEFFS = 9;                         // order-3 SH: band0 + 8
	static constexpr float SH_BAND0  = 0.2820947917738781f;       // constant band-0 basis value
	// overall strength (and sign) of the cross-group SH steering. This is the main
	// knob for the avoidance behaviour -- tune it (and its sign) by eye.
	static constexpr float shAvoidStrength = 3.0f;

	// per-group accumulated (averaged) SH coefficients on device:
	// group g, coefficient k at d_groupSH[g * SH_COEFFS + k]. Index 0 is band 0.
	float* d_groupSH;

	// Order-3 real SH basis of a unit direction (x,y,z); fills the 8 non-constant
	// coefficients (band 0 is the constant SH_BAND0). Plain arithmetic, so it runs
	// on both host (projection) and device (reconstruction).
	static void shEval3(float x, float y, float z, float out[8])
	{
		const float fZ2 = z * z;
		out[1] = 0.4886025119029199f * z;
		out[5] = 0.9461746957575601f * fZ2 - 0.3153915652525201f;
		const float fC0 = x, fS0 = y;
		const float fTmpA = -0.48860251190292f;
		out[2] = fTmpA * fC0;
		out[0] = fTmpA * fS0;
		const float fTmpB = -1.092548430592079f * z;
		out[6] = fTmpB * fC0;
		out[4] = fTmpB * fS0;
		const float fC1 = x * fC0 - y * fS0;
		const float fS1 = x * fS0 + y * fC0;
		const float fTmpC = 0.5462742152960395f;
		out[7] = fTmpC * fC1;
		out[3] = fTmpC * fS1;
	}

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
