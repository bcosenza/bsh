// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDTHREECOLUMNSSH_H_
#define _BOIDTHREECOLUMNSSH_H_

#include "BoidGridBase.h"

/*
	Static obstacle avoidance against three columns, via spherical harmonics -- the
	SYCL port of the OpenCL BoidModelSHObstacle scenario.

	The three columns' surfaces are discretised (host side, by the Column class)
	into a list of obstacle points, each carrying a bundle of outward repulsion
	vectors. At construction those are projected once into a per-point, vector-
	valued order-3 SH field (separate X/Y/Z coefficient sets), stored on the device.

	Each simulate() runs the inherited grid flocking, then -- per boid -- sums the
	repulsion reconstructed from every obstacle point (attenuated by 1/distance^2)
	and adds it to the steering before integrating. Reuses BoidGridBase (grid,
	flocking, wall, rendering) and the shared shEval3 / SH_BAND0 basis.
*/
class BoidThreeColumnsSH : public BoidGridBase
{
public:
	// cor/start/end/posObst come from Column::getObstacleForce (via
	// Simulation::columnObstacleData), exactly as the OpenCL obstacle model.
	BoidThreeColumnsSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
	                   std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end,
	                   std::vector<Vec4> posObst);
	virtual ~BoidThreeColumnsSH();

	void simulate(float dt);   // grid flocking + obstacle-SH avoidance

protected:
	// overall strength of the obstacle repulsion (matches FACTOR_OBST in the OpenCL
	// kernel). Main tuning knob for how hard boids are pushed off the obstacle;
	// set in the constructor so derived scenes (e.g. the tunnel) can override it.
	float obstStrength;

	// per obstacle point: 27 floats = shX[8], shY[8], shZ[8], c0x, c0y, c0z
	static constexpr int OBST_STRIDE = 27;

	int   numObst;        // number of obstacle points
	float* d_obstacleSH;  // numObst * OBST_STRIDE, projected once at construction
	sf4*   d_posObst;     // world-space anchor of each obstacle point

	// Reconstruct the vector-valued obstacle SH field at unit direction dir (from
	// the obstacle point toward the boid). c points at that point's OBST_STRIDE
	// coefficients. Uses the standard SH product band weights (l=1 -> 2pi/3,
	// l=2 -> pi/4, band 0 constant). Returns the (un-attenuated) repulsion vector.
	static sf4 reconstructObstacle(const sf4& dir, const float* c)
	{
		float b[8];
		shEval3(dir.x(), dir.y(), dir.z(), b);

		const float w1 = (float)M_PI * (2.0f / 3.0f);   // l = 1
		const float w2 = (float)M_PI * (1.0f / 4.0f);   // l = 2

		float sx = c[24] * SH_BAND0 * (float)M_PI;
		float sy = c[25] * SH_BAND0 * (float)M_PI;
		float sz = c[26] * SH_BAND0 * (float)M_PI;
		for (int k = 0; k < 8; k++){
			const float w = (k < 3) ? w1 : w2;
			sx += b[k] * c[0  + k] * w;
			sy += b[k] * c[8  + k] * w;
			sz += b[k] * c[16 + k] * w;
		}
		return sf4(sx, sy, sz, 0.0f);
	}
};

#endif // _BOIDTHREECOLUMNSSH_H_
