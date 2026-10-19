// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "Boid4GroupsAvoidanceSH.h"

Boid4GroupsAvoidanceSH::Boid4GroupsAvoidanceSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP)
	: BoidGroupAvoidanceSH(pos, vel, color, simP, 4)   // four groups; everything else inherited
{
	simTimeDisc[0] = "Four groups avoidance with SH";
}

Boid4GroupsAvoidanceSH::~Boid4GroupsAvoidanceSH(){
	// base destructors free the SH buffer, grid buffers and the rest
}
