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

BoidModelSH_2D::BoidModelSH_2D(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP)
	: BoidModelSH(clHlpr, flattenedPos(pos, CELL_SIZE_Y / 2), flattenedVel(vel), simP,
		"boidModelSH_2D_kernel_v1.cl", "Boid Model SH 2D")
{
	yAxisFixed = CELL_SIZE_Y / 2;
}

/* the 2D useSH kernel takes the fixed Y height before dt */
void BoidModelSH_2D::setUseSHFinalArgs(float dt){
	err = kernel_useSH.setArg(9, yAxisFixed);
	err = kernel_useSH.setArg(10, dt);
}
