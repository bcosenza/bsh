// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOID4GROUPSAVOIDANCESH_H_
#define _BOID4GROUPSAVOIDANCESH_H_

#include "BoidGroupAvoidanceSH.h"

/*
	Four-group variant of BoidGroupAvoidanceSH. Identical machinery -- one order-3
	SH basis per group, projected each step and reconstructed from every other
	group -- but with four groups instead of two. All the work is inherited; this
	class only selects numGroups = 4 (via the base's N-group constructor) and the
	label. Pair it with the four-groups init placement.
*/
class Boid4GroupsAvoidanceSH : public BoidGroupAvoidanceSH
{
public:
	Boid4GroupsAvoidanceSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP);
	virtual ~Boid4GroupsAvoidanceSH();
};

#endif // _BOID4GROUPSAVOIDANCESH_H_
