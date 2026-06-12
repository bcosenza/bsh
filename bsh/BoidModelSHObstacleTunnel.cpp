#include "stdafx.h"
#include "BoidModel.h"

BoidModelSHObstacleTunnel::BoidModelSHObstacleTunnel(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
	std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP, std::vector<Vec4> cor,
	std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst)
	: BoidModelSHCombined(clHlpr, pos, vel, goal, color, simP, cor, start, end, posObst,
		"boidModelSHObstacleTunnel_kernel_v1.cl", "Boid Model SH Tunnel")
{
}
