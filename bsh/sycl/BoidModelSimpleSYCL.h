// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDMODELSIMPLESYCL_H_
#define _BOIDMODELSIMPLESYCL_H_

#include "common.h"
#include "BoidModelSimpleView.h"
#include "vectorTypes.h"
#include <sycl/sycl.hpp>

// distinct alias: vector_types.h already defines an (operator-less) float4
using sf4 = sycl::float4;

/*
	BoidModelSimple is a naive O(n^2) flocking model implemented in SYCL
	Boids data is stored in device memory allocated with sycl::malloc_device; each step runs a SYCL
	parallel_for and copies the result back into the GL VBOs that are rendered (no-GL-interop path).
*/
class BoidModelSimpleSYCL : public BoidModelSimpleView
{
public:
	BoidModelSimpleSYCL(std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	~BoidModelSimpleSYCL();

	// backend-specific parts of the BoidModel interface (the rest come from the view)
	void simulate(float dt);
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();

protected:
	// neighbourhood bounding-box factor (matches boidModelSimple_kernel_v2.cl):
	// the border repulsion shell is this many cells thick on each world face.
	static constexpr int boundingBoxFactor = 2;

	// Repulsive world boundary (device code, shared by SYCL boid models).
	// Returns a correction velocity that pushes a boid at world position p back
	// toward the interior when it lies within boundingBoxFactor cells of a world
	// face, and zero in the interior. It is a soft wall: the boid is nudged, not
	// clamped or wrapped. The returned w component is always 0, so it can be added
	// straight onto a velocity without disturbing its w.
	static sf4 wallRepulsion(const sf4& p, const simParams_t& sp)
	{
		const sf4 cellSize = sf4(sp.cellSize.x, sp.cellSize.y, sp.cellSize.z, 0.0f);
		const sf4 margin   = cellSize * (float)boundingBoxFactor;
		const sf4 worldMax = sf4((float)sp.gridSize.x, (float)sp.gridSize.y, (float)sp.gridSize.z, 0.0f) * cellSize;
		const sf4 r        = p - sf4(sp.worldOrigin.x, sp.worldOrigin.y, sp.worldOrigin.z, 0.0f);

		// +maxVelCor near the low faces, -maxVelCor near the high faces (high wins
		// on overlap). maxCor.w() is 0, so the result's w stays 0 whatever the w
		// lane of the border masks happens to be.
		const sf4 maxCor = sf4(sp.maxVelCor, sp.maxVelCor, sp.maxVelCor, 0.0f);
		sf4 velCor = sf4(0.0f);
		velCor = sycl::select(velCor,  maxCor, r <  margin);
		velCor = sycl::select(velCor, -maxCor, r >= (worldMax - margin));
		return velCor;
	}

private:
	// perception radii of the three Reynolds steering rules (world units). A boid
	// only reacts to flockmates that fall inside the corresponding radius.
	static constexpr float cohesionRadius   = 5.0f;   // how far it looks to steer toward the flock centre
	static constexpr float alignmentRadius  = 5.0f;   // how far it looks to match neighbours' heading
	static constexpr float separationRadius = 2.5f;   // tighter zone within which it pushes away

	// pointer to description strings (will be used for simulation time)
	std::vector<const char*> simTimeDisc;
	// Simulation time as string for overlay text
	std::string stringSimTime;
	// last measured step time in milliseconds
	long lastSimTimeMs;

	// boid state is stored as sycl::float4 in device memory
	sycl::queue queue;
	sycl::float4* d_pos;
	sycl::float4* d_pos_out;
	sycl::float4* d_vel;
	sycl::float4* d_vel_out;

	// host staging buffer used to upload results into the GL VBOs
	std::vector<Vec4> hostBuf;
};

#endif // _BOIDMODELSIMPLESYCL_H_
