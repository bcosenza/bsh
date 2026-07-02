// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDMODEL_H_
#define _BOIDMODEL_H_

#include "common.h"
#include "SimParam.h"
#include "vector_types.h"
#include "vectorTypes.h"
#include "Shader.h"
#include "Renderable.h"

/*
	Simulation parameters used in OpenCL kernels
*/
typedef struct simParams_t{
	uint3 gridSize;				// number of cells per axis
	unsigned int numCells;		// pre calculated number of cells
	float3 worldOrigin;			// origin of the world in object space (currently not used)
	float3 cellSize;			// size of cells

	unsigned int numBodies;		// number of boids used in the simulation
	unsigned int localSize;

	float wSeparation;			// weight of separation in the simulation
	float wAlignment;			// weight of alignment in the simulation
	float wCohesion;			// weight of cohesion in the simulation
	float wOwn;					// weight of own velocity in simulation
	float wPath;

	float maxVel;				// maximum velocity
	float maxVelCor;			// maximum correction velocity

} simParams_t;

/*
	Virtual base class for boids, implements interface Renderable.
*/
class BoidModel : public Renderable
{
public:
	simParams_t simParams;

	virtual ~BoidModel() {};

	/* Execute all simulation steps for the boid model
	dt - delta time */
	virtual void simulate(float dt) = 0;

	/* Returns the index of Vertex Buffer Object of positions */
	virtual GLuint getPosVBO() = 0;

	/* Returns index of Vertex Buffer Object of velocities */
	virtual GLuint getVelVBO() = 0;

	/* Returns index of Vertex Array Object */
	virtual GLuint getPosVAO() = 0;

	/* Returns number of boids */
	virtual int getNumBoid() = 0;

	/* Returns execution time of the simulate kernel */
	virtual long getSimulationTime() = 0;

	/* Returns std::vector with pointers to text which is used to display text in the interface */
	virtual std::vector<const char*> getSimTimeDescriptions() = 0;

	/* Get position data of a specific Boid, boidIndex passed in may be changed to new Index after reordering */
	virtual void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel) = 0;

protected:
	BoidModel() {};
};
#endif // _BOIDMODEL_H_
