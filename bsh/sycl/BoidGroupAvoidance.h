// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDGROUPAVOIDANCE_H_
#define _BOIDGROUPAVOIDANCE_H_

#include "BoidGridBase.h"

/*
	Grid-accelerated SYCL model set up for two interacting groups. It reuses all
	of BoidGridBase (uniform-grid neighbour search, flocking, wall repulsion,
	rendering) and only relabels the model; its scene defaults to the two-groups
	initial placement. A dedicated inter-group avoidance term can be added here
	later by overriding simulate() while still calling the shared grid build.
*/
class BoidGroupAvoidance : public BoidGridBase
{
public:
	BoidGroupAvoidance(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP);
	virtual ~BoidGroupAvoidance();
};

#endif // _BOIDGROUPAVOIDANCE_H_
