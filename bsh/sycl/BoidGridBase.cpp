// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidGridBase.h"
#include "vectorTypes.h"
#include <oneapi/dpl/execution>
#include <oneapi/dpl/algorithm>
#include <cmath>
#include <chrono>

BoidGridBase::BoidGridBase(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP)
	: BoidModelSimpleSYCL(pos, vel, color, simP)   // sets up device state, VBOs (with colors) and weights
{
	// relabel the overlay (the base constructor set "Boid Model Simple")
	simTimeDisc[0] = "Boid Model Grid";

	// cells are the largest interaction radius wide, so the 3x3x3 cell block
	// always covers a boid's whole neighbourhood
	cellSize = maxRadius;

	// the world extent is taken from SimParam's world (gridSize * cellSize); the
	// grid resolution here is our own, derived from the cell size above
	float3 worldSize = make_float3(
		simParams.gridSize.x * simParams.cellSize.x,
		simParams.gridSize.y * simParams.cellSize.y,
		simParams.gridSize.z * simParams.cellSize.z);

	gridDimX = (int)std::ceil(worldSize.x / cellSize);
	gridDimY = (int)std::ceil(worldSize.y / cellSize);
	gridDimZ = (int)std::ceil(worldSize.z / cellSize);
	if (gridDimX < 1) gridDimX = 1;
	if (gridDimY < 1) gridDimY = 1;
	if (gridDimZ < 1) gridDimZ = 1;
	numCells = gridDimX * gridDimY * gridDimZ;

	gridOrigin = sf4(simParams.worldOrigin.x, simParams.worldOrigin.y, simParams.worldOrigin.z, 0.0f);

	// device grid buffers
	d_cellStart = sycl::malloc_device<int>(numCells, queue);
	d_cellEnd   = sycl::malloc_device<int>(numCells, queue);
	d_hash      = sycl::malloc_device<unsigned int>(num, queue);
	d_index     = sycl::malloc_device<unsigned int>(num, queue);

	queue.wait();
}

BoidGridBase::~BoidGridBase(){
	queue.wait();
	sycl::free(d_cellStart, queue);
	sycl::free(d_cellEnd, queue);
	sycl::free(d_hash, queue);
	sycl::free(d_index, queue);
	// ~BoidModelSimpleSYCL frees d_pos/... and the GL resources
}

void BoidGridBase::buildGrid(){
	const int   n   = num;
	const float cs  = cellSize;
	const int   dimX = gridDimX, dimY = gridDimY, dimZ = gridDimZ;
	const sf4   origin = gridOrigin;

	sf4*          pos   = d_pos;
	unsigned int* hash  = d_hash;
	unsigned int* index = d_index;
	int*          cellStart = d_cellStart;
	int*          cellEnd   = d_cellEnd;

	// steps 1-2: cell hash + identity index for every boid (device)
	queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		int i = (int)idx[0];
		sf4 p = pos[i];
		int gx = gridCoord(p.x(), origin.x(), cs, dimX);
		int gy = gridCoord(p.y(), origin.y(), cs, dimY);
		int gz = gridCoord(p.z(), origin.z(), cs, dimZ);
		hash[i]  = (unsigned int)cellIndex(gx, gy, gz, dimX, dimY);
		index[i] = (unsigned int)i;
	});

	// step 3: radix sort the (hash, index) pairs by hash, entirely on device.
	// The in-order queue guarantees the hash kernel above has finished first.
	oneapi::dpl::sort_by_key(oneapi::dpl::execution::make_device_policy(queue),
	                         hash, hash + n, index);

	// step 4a: reset cell ranges (-1 == empty), then derive [start, end) from the
	// now-sorted hashes: a run of equal hashes is one cell.
	queue.fill(cellStart, -1, (size_t)numCells);
	queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		int k = (int)idx[0];
		unsigned int c = hash[k];
		if (k == 0 || c != hash[k - 1]){
			cellStart[c] = k;                 // first boid of this cell
			if (k > 0) cellEnd[hash[k - 1]] = k;   // end of the previous cell
		}
		if (k == n - 1) cellEnd[c] = n;       // end of the last cell
	});

	// step 4b: reorder position/velocity into d_pos_out/d_vel_out (sorted order)
	sf4* src  = d_pos;   sf4* dst  = d_pos_out;
	sf4* srcV = d_vel;   sf4* dstV = d_vel_out;
	queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		int k = (int)idx[0];
		int i = (int)index[k];
		dst[k]  = src[i];
		dstV[k] = srcV[i];
	});
	queue.wait();
}

void BoidGridBase::simulate(float dt){
	const int          n  = num;
	const simParams_t  sp = simParams;

	// grid parameters as kernel-capturable locals (no 'this' capture in kernels)
	const float cs   = cellSize;
	const int   dimX = gridDimX, dimY = gridDimY, dimZ = gridDimZ;
	const sf4   origin = gridOrigin;

	auto t0 = std::chrono::high_resolution_clock::now();

	// steps 1-4: hash, sort, cell ranges, reorder -> sorted state in d_pos_out/d_vel_out
	buildGrid();

	sf4* sortedPos = d_pos_out;   // input: sorted current state
	sf4* sortedVel = d_vel_out;
	sf4* outPos    = d_pos;       // output: new state, scattered back to original order
	sf4* outVel    = d_vel;
	int* cellStart = d_cellStart;
	int* cellEnd   = d_cellEnd;
	unsigned int* index = d_index;   // sorted position k -> original boid index

	// step 5: pairwise interaction, but only over the 27 neighbouring cells
	queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		const int k = (int)idx[0];
		sf4 p = sortedPos[k];
		sf4 v = sortedVel[k];

		// accumulate the three rules over the 3x3x3 block of cells around this boid
		FlockAccum acc = gridAccumulate(p, v, sortedPos, sortedVel,
		                                cellStart, cellEnd, cs, dimX, dimY, dimZ, origin);

		// same steering + dynamics as the simple model
		const sf4 steer = finalizeSteer(acc, p, v, sp);
		advanceBoid(p, v, steer, sp, dt);

		// scatter back to the boid's original index so the rendered VBO (and thus
		// the static per-boid color VBO) stays in a stable order across frames
		const int orig = (int)index[k];
		outPos[orig] = p;
		outVel[orig] = v;
	});
	queue.wait();

	auto t1 = std::chrono::high_resolution_clock::now();
	lastSimTimeMs = (long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

	// step 6: draw -- upload the new state into the ping-pong VBO the base's
	// getPosVAO() renders this frame (reuses the base's uploadToVBO)
	bool pingPong = ((helper++ % 2) == 0);
	uploadToVBO(outPos, pingPong ? pos_out_vbo[0] : pos_vbo[0]);
	uploadToVBO(outVel, pingPong ? vel_out_vbo[0] : vel_vbo[0]);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
