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
	BoidModelSimpleSYCL(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP);
	~BoidModelSimpleSYCL();

	// backend-specific parts of the BoidModel interface (the rest come from the view)
	void simulate(float dt);
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();

protected:
	// neighbourhood bounding-box factor (matches boidModelSimple_kernel_v2.cl):
	// the border repulsion shell is this many cells thick on each world face.
	static constexpr int boundingBoxFactor = 2;

	// Unit direction of u (w ignored), or the zero vector if u is ~0. Returning
	// zero keeps an inactive rule from injecting a bogus direction.
	static sf4 safeNormalize(sf4 u)
	{
		u.w() = 0.0f;
		float len = sycl::length(u);
		return (len > 1e-6f) ? (u / len) : sf4(0.0f);
	}

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

	// --- Reynolds flocking, split so the brute-force and grid neighbour searches
	// share identical interaction maths (device code, shared by SYCL boid models) ---

	// running per-boid accumulation of the three rules over its neighbourhood
	struct FlockAccum {
		sf4 cohesion   = sf4(0.0f);
		sf4 separation = sf4(0.0f);
		sf4 alignment  = sf4(0.0f);
		int cohesionMates  = 0;   // cohesion & alignment each average over their
		int alignmentMates = 0;   // own neighbourhood, so each keeps its own count
	};

	// Fold one neighbour (position op, velocity ov) into acc, for the boid at p
	// with velocity v. Applies the forward field-of-view test and the three
	// per-rule radii; self and coincident neighbours (distance ~0) are ignored.
	static void accumulateNeighbour(FlockAccum& acc, const sf4& p, const sf4& v,
	                                const sf4& op, const sf4& ov)
	{
		sf4 dist = op - p;
		dist.w() = 0.0f;
		float distLength = sycl::length(dist);
		if (distLength < 1e-6f) return;   // self / coincident

		// field-of-view: ignore neighbours in the ~45deg rear blind cone
		float dotP  = sycl::dot(-v, dist);
		float lenV  = sycl::length(v);
		float angle = dotP / (lenV * distLength);
		float deg   = sycl::acos(angle) * (180.0f / (float)M_PI);
		bool inView = (dotP < 0.0f) || (sycl::fabs(deg) > 45.0f);
		if (!inView) return;

		// Cohesion: average position of nearby flockmates
		if (distLength < cohesionRadius){ acc.cohesion += op; acc.cohesionMates += 1; }
		// Alignment: average heading of nearby flockmates
		if (distLength < alignmentRadius){ acc.alignment += ov; acc.alignmentMates += 1; }
		// Separation: push away, weighted by 1/distance (closest push hardest)
		if (distLength < separationRadius) acc.separation -= dist / (distLength * distLength);
	}

	// Turn an accumulated neighbourhood into the weighted, unit-normalized steering
	// vector. Normalizing makes the weights true relative importances, independent
	// of radii / world scale; caller applies momentum and gain via advanceBoid().
	static sf4 finalizeSteer(FlockAccum acc, const sf4& p, const sf4& v, const simParams_t& sp)
	{
		// average neighbour minus own state
		if (acc.alignmentMates > 0) acc.alignment = (acc.alignment / (float)acc.alignmentMates) - v;
		if (acc.cohesionMates  > 0) acc.cohesion  = (acc.cohesion  / (float)acc.cohesionMates)  - p;

		const sf4 cohesionDir   = safeNormalize(acc.cohesion);
		const sf4 alignmentDir  = safeNormalize(acc.alignment);
		const sf4 separationDir = safeNormalize(acc.separation);

		return cohesionDir   * sp.wCohesion
		     + alignmentDir  * sp.wAlignment
		     + separationDir * sp.wSeparation;
	}

	// Brute-force O(n^2) steering: fold in every other boid. Grid models replace
	// this loop with a 27-cell search but reuse accumulateNeighbour/finalizeSteer.
	static sf4 flockingSteer(const sf4& p, const sf4& v,
	                         const sf4* pos, const sf4* vel,
	                         int n, const simParams_t& sp)
	{
		FlockAccum acc;
		for (int j = 0; j < n; j++)
			accumulateNeighbour(acc, p, v, pos[j], vel[j]);
		return finalizeSteer(acc, p, v, sp);
	}

	// Advance one boid: apply momentum + steering gain, clamp the speed into
	// [minSpeed, maxVel], add the repulsive-wall correction and integrate the
	// position (explicit Euler). Shared by all SYCL boid models so the dynamics
	// stay identical however the steering was gathered.
	static void advanceBoid(sf4& p, sf4& v, const sf4& steer, const simParams_t& sp, float dt)
	{
		v = v * sp.wOwn + steer * steerStrength;
		v.w() = 0.0f;

		// clamp speed into [minSpeed, maxVel]: cap the top, and hold a floor so a
		// settled cluster keeps cruising instead of damping to a standstill
		float len = sycl::length(v);
		float minSpeed = minSpeedFactor * sp.maxVel;
		if (len > sp.maxVel)                    v *= sp.maxVel / len;
		else if (len < minSpeed && len > 1e-6f) v *= minSpeed / len;

		// keep the boid inside the world, then integrate
		v += wallRepulsion(p, sp);
		p += v * dt;
	}

protected:
	// perception radii of the three Reynolds steering rules (world units). A boid
	// only reacts to flockmates that fall inside the corresponding radius.
	// Scaled to the scene (8192 boids in the 1200-unit world => ~33-unit mean
	// spacing) and ordered as concentric Couzin zones: separation < alignment <
	// cohesion (zone of repulsion < orientation < attraction). Because the rules
	// are un-normalized, the radii also set relative strength: cohesion force
	// grows ~linearly with its radius (so it is the largest, to counter its
	// otherwise weak pull) while separation grows ~radius^4 (so it is the
	// smallest, to stop it dominating) -- see notes in simulate().
	static constexpr float cohesionRadius   = 80.0f;  // outermost: steer toward the flock centre
	static constexpr float alignmentRadius  = 50.0f;  // mid zone: match neighbours' heading
	static constexpr float separationRadius = 13.0f;  // innermost personal space: push apart

	// largest interaction radius: a boid can influence another only within this
	// distance, so a uniform grid with cells this size only ever needs to search
	// the 27 neighbouring cells. Used by grid-accelerated derived models.
	static constexpr float maxRadius = cohesionRadius;

	// overall steering gain (world units / step). The three rules are normalized
	// to unit directions, so this single knob sets how hard a boid turns; the
	// per-rule weights then only set the balance between the rules. Kept well
	// below maxVel so heading changes gradually instead of snapping each frame.
	static constexpr float steerStrength = 1.0f;

	// minimum cruising speed, as a fraction of maxVel. Alignment pulls a boid's
	// velocity toward its neighbours' average, which inside a settled cluster
	// tends to ~0 and would otherwise let the flock freeze in place; this floor
	// keeps boids milling. Kept above the max per-step steering nudge so velocity
	// can never be driven through zero (which would lose the heading direction).
	static constexpr float minSpeedFactor = 0.5f;

	// pointer to description strings (will be used for simulation time)
	std::vector<const char*> simTimeDisc;
	// Simulation time as string for overlay text
	std::string stringSimTime;
	// last measured step time in milliseconds
	long lastSimTimeMs;

	// boid state is stored as sycl::float4 in device memory. The queue is in-order
	// so multi-kernel pipelines (e.g. the grid build) serialize without explicit
	// event chaining -- USM allocations carry no automatic dependencies.
	sycl::queue queue{sycl::property::queue::in_order()};
	sycl::float4* d_pos;
	sycl::float4* d_pos_out;
	sycl::float4* d_vel;
	sycl::float4* d_vel_out;

	// host staging buffer used to upload results into the GL VBOs
	std::vector<Vec4> hostBuf;

	// copy one device state buffer (num float4s) into hostBuf and upload it into
	// the given GL VBO for rendering. Blocks on the copy before the upload.
	void uploadToVBO(sycl::float4* deviceBuf, GLuint vbo);
};

#endif // _BOIDMODELSIMPLESYCL_H_
