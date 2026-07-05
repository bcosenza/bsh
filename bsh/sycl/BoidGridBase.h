// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDGRIDBASE_H_
#define _BOIDGRIDBASE_H_

#include "BoidModelSimpleSYCL.h"

/*
	Grid-accelerated SYCL flocking model.

	Runs exactly the same Reynolds model as BoidModelSimpleSYCL -- it reuses that
	class's interaction maths (accumulateNeighbour / finalizeSteer), dynamics
	(advanceBoid), wall repulsion and VBO rendering -- but replaces the O(n^2)
	neighbour search with a uniform grid, so each boid only inspects the 27
	neighbouring cells.

	Each step:
	  1. compute every boid's grid cell position,
	  2. compute its cell hash and (hash, index) pair,
	  3. sort the pairs by hash,
	  4. reorder position/velocity by the sorted index and record per-cell
	     [start, end) ranges,
	  5. compute pairwise interaction as in the simple model, but only over the
	     27 neighbouring cells,
	  6. draw (same VBO upload as the simple model).

	The grid cell size equals the largest interaction radius (maxRadius), so a
	boid's whole neighbourhood is always covered by the 3x3x3 cell block. The grid
	resolution is derived here from the world extent -- it is a parameter of this
	class and does NOT use the grid fields in SimParam.h.

	Designed as a base class: models that need the same grid (e.g. a future
	spatial-hash SH model) can derive from it and override the interaction while
	reusing buildGrid().
*/
class BoidGridBase : public BoidModelSimpleSYCL
{
public:
	BoidGridBase(std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	virtual ~BoidGridBase();

	// grid-accelerated step (overrides the brute-force simulate of the base)
	void simulate(float dt);

protected:
	// --- uniform grid parameters (independent of SimParam.h's grid fields) ---
	float cellSize;                 // == maxRadius, so 27 cells cover a neighbourhood
	int   gridDimX, gridDimY, gridDimZ;
	int   numCells;
	sf4   gridOrigin;               // world-space origin (from simParams.worldOrigin)

	// --- device grid buffers (malloc_device) ---
	int*          d_cellStart;      // first sorted-boid index in each cell (-1 if empty)
	int*          d_cellEnd;        // one-past-last sorted-boid index in each cell
	unsigned int* d_hash;           // cell hash per boid (sort key)
	unsigned int* d_index;          // boid index (sort payload); sorted order after the sort

	// world coordinate -> clamped cell index along one axis. Plain integer truncation
	// (works identically on host and device); truncation and floor differ only for
	// negative coordinates, which both clamp to cell 0 anyway.
	static int gridCoord(float w, float origin, float cs, int dim)
	{
		int c = (int)((w - origin) / cs);
		if (c < 0)        c = 0;
		if (c > dim - 1)  c = dim - 1;
		return c;
	}
	// (gx, gy, gz) -> linear cell index
	static int cellIndex(int gx, int gy, int gz, int dimX, int dimY)
	{
		return (gz * dimY + gy) * dimX + gx;
	}

	// Build the grid for the current state in d_pos/d_vel (steps 1-4): hash on the
	// host, sort the (hash, index) pairs, fill the per-cell [start, end) ranges and
	// reorder position/velocity into d_pos_out/d_vel_out (sorted order).
	void buildGrid();
};

#endif // _BOIDGRIDBASE_H_
