// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _ACCURACYLANES_H_
#define _ACCURACYLANES_H_

/*
	Lane geometry of the SH accuracy scene 9 (plate lanes), shared by the initial
	placement (Simulation), the plate obstacles (PlateLanes) and the model's report.

	The scene stacks LANES independent experiments along y, LANE_SPACING apart
	(more than the cohesion radius plus two group radii, so local flocking does
	not couple lanes). Each lane is centred at (world x / 2, laneY(L), world z / 2);
	one group per lane starts APPROACH away from the centre and approaches a thin
	plate (normal +-x) at incidence angle PLATE_DEG[L] from the normal.
*/
namespace AccLanes {
	constexpr int   LANES        = 7;
	constexpr float LANE_Y0      = 120.0f;   // y of lane 0's centre (world units)
	constexpr float LANE_SPACING = 160.0f;   // y distance between lane centres
	constexpr float APPROACH     = 200.0f;   // start distance of a group from the lane centre
	// Goal seeking keeps each group on its lane course: without it the flocking
	// rules re-orient a dense group within ~150 steps and the incidence angle is
	// lost. Each agent seeks the point APPROACH behind the plate on its line (as
	// the tunnel's goals behind the wall), with weight PLATE_PATH_WEIGHT below. The
	// term is the same whatever drives the SH term.

	// incidence angle from the plate normal, per lane (degrees)
	constexpr float PLATE_DEG[LANES] = { 0.0f, 15.0f, 30.0f, 45.0f, 60.0f, 70.0f, 80.0f };

	// scene 9 plate: half extents (x = half thickness, y = half height, z = half
	// width), obstacle-point spacing on each face, and repulsion vector length
	// (as the columns' 5-unit corner vectors)
	constexpr float PLATE_HALF_X = 2.5f;
	constexpr float PLATE_HALF_Y = 50.0f;
	constexpr float PLATE_HALF_Z = 80.0f;
	constexpr float PLATE_POINT_SPACING = 10.0f;
	constexpr float PLATE_VECTOR_LEN = 5.0f;
	// range of each plate point: its 1/r^2 weight is tapered by (1 - (r/R)^2)^2 and
	// is zero beyond R. Without it the 374 points of a plate add up to a far field
	// (0.05 per step at 150 units, which integrates into an early turn), so groups
	// would never reach the plate. Applied alike to the SH and reference fields.
	constexpr float PLATE_RANGE = 40.0f;
	// obstacle strength (kappa) and goal weight of the plate scene, chosen so the
	// exact reference just holds the groups off: ~0.5 per step at 20 units, ~3.5 at
	// 5 units (exact field, normal incidence) against a 0.5 pull toward the goal
	// behind the plate. With path 0.5 the reference lets a handful of agents through
	// at near-normal incidence and none from 30 degrees on; a stronger pull (or a
	// weaker plate) saturates the penetration count for every driver.
	constexpr float PLATE_STRENGTH = 10.0f;
	constexpr float PLATE_PATH_WEIGHT = 0.5f;

	inline float laneY(int lane) { return LANE_Y0 + lane * LANE_SPACING; }
}

#endif // _ACCURACYLANES_H_
