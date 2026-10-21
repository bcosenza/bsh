// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDTUNNELSH_H_
#define _BOIDTUNNELSH_H_

#include "BoidThreeColumnsSH.h"

/*
	Tunnel scenario -- a one-way funnel through a wall with openings (the SYCL port
	of the OpenCL BoidModelSHObstacleTunnel). It reuses the obstacle-SH engine of
	BoidThreeColumnsSH unchanged (grid flocking + per-obstacle-point vector-valued
	SH repulsion projected once and reconstructed per agent with 1/distance^2
	falloff). Only the obstacle geometry (from Tunnel::getObstacleForce) and the
	repulsion strength differ; all of simulate() is inherited.
*/
class BoidTunnelSH : public BoidThreeColumnsSH
{
public:
	// cor/start/end/posObst come from Tunnel::getObstacleForce; goal holds each
	// boid's target on the far side of the wall (set by the two-groups placement).
	BoidTunnelSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
	             std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end,
	             std::vector<Vec4> posObst, std::vector<Vec4> goal);
	virtual ~BoidTunnelSH();
};

#endif // _BOIDTUNNELSH_H_
