#include "common.h"
#include "BoidModel.h"

//place all boids on the fixed Y plane of the 2D world
static std::vector<Vec4> flattenedPos(std::vector<Vec4> pos, float y){
	for (size_t i = 0; i < pos.size(); i++)
		pos[i].y = y;
	return pos;
}

static std::vector<Vec4> flattenedVel(std::vector<Vec4> vel){
	for (size_t i = 0; i < vel.size(); i++)
		vel[i].y = 0.0f;
	return vel;
}

BoidModelGrid_2D::BoidModelGrid_2D(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP)
	: BoidModelGrid(clHlpr, flattenedPos(pos, CELL_SIZE_Y / 4), flattenedVel(vel), simP,
		"boidModelGrid_2D_kernel_v2.cl", "Boid Model Grid 2D")
{
}
