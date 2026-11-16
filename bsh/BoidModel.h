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
	Duration of the phases of the last simulate() call, in microseconds. GPU phases
	come from device profiling events; upload is the host-side device->GL copy.
	A phase a model does not have stays 0.
*/
struct PhaseTimes {
	double projectUs  = 0.0;	// SH projection (group SH)
	double gridUs     = 0.0;	// uniform-grid build (hash, sort, ranges, reorder)
	double interactUs = 0.0;	// interaction + integration kernel
	double uploadUs   = 0.0;	// device -> host -> GL VBO upload
};

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

	/* SH projection kernel (SH_PROJECTION_*) and vMF concentration k. Models without
	SH ignore it and report an empty name. */
	virtual void setSHProjection(int mode, float k) {}
	virtual int getSHProjection() { return SH_PROJECTION_DIRAC; }
	virtual const char* getSHProjectionName() { return ""; }

	/* Phase durations of the last simulate() call (zero if not measured) */
	virtual PhaseTimes getPhaseTimes() { return PhaseTimes(); }

	/* Hook for the accuracy study: compare the SH steering of the current state
	against a brute-force reference. Not implemented yet; meant for specific scenes. */
	virtual void evaluateAgainstReference() {}

protected:
	BoidModel() {};
};
#endif // _BOIDMODEL_H_
