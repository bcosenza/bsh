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
                           std::vector<Vec4> posObst)
	// reuse the obstacle-SH engine as-is (it projects whatever obstacle geometry it is given)
	: BoidThreeColumnsSH(pos, vel, color, simP, cor, start, end, posObst)
{
	simTimeDisc[0] = "Tunnel SH";
	// the funnel needs a stronger push than the free-standing columns (the OpenCL
	// tunnel uses FACTOR_OBST 30 vs 21 for the columns) -- tune by eye.
	obstStrength = 4.5f;
}

BoidTunnelSH::~BoidTunnelSH(){
	// ~BoidThreeColumnsSH frees the obstacle SH buffers; the grid/base free the rest
}
