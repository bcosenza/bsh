// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidGroupAvoidance.h"

BoidGroupAvoidance::BoidGroupAvoidance(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP)
	: BoidGridBase(pos, vel, color, simP)   // full grid model; only the label differs
{
	simTimeDisc[0] = "Two groups avoidance";
}

BoidGroupAvoidance::~BoidGroupAvoidance(){
	// ~BoidGridBase frees the grid buffers; ~BoidModelSimpleSYCL the rest
}
