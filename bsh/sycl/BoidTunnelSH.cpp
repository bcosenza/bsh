// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidTunnelSH.h"

BoidTunnelSH::BoidTunnelSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
                           std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end,
                           std::vector<Vec4> posObst, std::vector<Vec4> goal)
	// reuse the obstacle-SH engine as-is (it projects whatever obstacle geometry it is
	// given), with two agent groups to match the scene's two-groups placement
	: BoidThreeColumnsSH(pos, vel, color, simP, cor, start, end, posObst, 2)
{
	simTimeDisc[0] = "Tunnel SH";
	// the funnel needs a stronger push than the free-standing columns (the OpenCL
	// tunnel uses FACTOR_OBST 30 vs 21 for the columns) -- tune by eye.
	obstStrength = 4.5f;
	// what drives the boids through the wall: each one seeks its goal on the far
	// side (weight as WEIGHT_GOAL_SH_OBSTACLE in the OpenCL tunnel); the obstacle SH
	// then steers them into one of the two openings
	enableGoalSeeking(goal, 2.0f);
}

BoidTunnelSH::~BoidTunnelSH(){
	// ~BoidThreeColumnsSH frees the obstacle SH buffers; the grid/base free the rest
}
