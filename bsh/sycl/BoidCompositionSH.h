// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDCOMPOSITIONSH_H_
#define _BOIDCOMPOSITIONSH_H_

#include "BoidGroupAvoidanceSH.h"
#include "AccuracyProbe.h"

/*
	SH accuracy scene 8: behaviour composition with Gaunt products, on the four-groups
	layout of scene 5 (red from -z, blue from +z, orange from -x, purple from +x, all
	converging on the centre).

	The red group steers by a composite preference field over directions w,

	    P(w) = Cone(w) * F_orange(w) * (1 - F_blue(w)),

	"keep toward my goal, move with orange, but not where blue heads":
	  Cone      = max(0, w . u), u the unit direction to the agent's goal (a clamped
	              cosine, projected in closed form: band factors pi, 2pi/3, pi/4);
	  F_g       = the heading field of group g (its per-group SH), divided by its
	              maximum over the sampled directions so it lies in [0, 1];
	  1 - F     = the complement (1 has SH coefficient sqrt(4 pi) in band 0).
	In the SH domain each product is a Gaunt product (the triple-product tensor
	G_ijk = int Y_i Y_j Y_k, order 3 x order 3 truncated back to order 3), evaluated
	as G = F_orange x (1 - F_blue) once per step and P = Cone x G per agent. The
	agent steers toward the maximum of P over NUM_DIRS fixed directions w*:
	    s = W_COMPOSITE (w* - h).
	The other three groups only seek their goals, so the composite is the only SH
	term in the scene and the drivers differ in red's steering alone.

	Drivers: Dirac SH, vMF SH, or the brute-force reference (SHMode::Exact): the
	same product evaluated pointwise on the exact (untruncated) factors -- the exact
	vMF heading densities of the same k, the exact clamped cosine -- with no SH and
	no truncation. With accuracy recording on, every step evaluates all three on the
	same state for every red agent and records the chosen direction w* of each (the
	probe's steering slots; the steering is affine in w*), the whole-field error of
	P over the sampled directions, the share of directions with a spurious negative
	lobe (P < -NEG_LOBE, the fields being normalised to [0, 1]), the reference
	preference of the direction each variant chooses (AUX), red's angle to orange's
	mean heading (B_DIST) and whether red heads toward its goal (B_EVENT).
*/
class BoidCompositionSH : public BoidGroupAvoidanceSH
{
public:
	// goal: per-boid goal (four-groups placement: the opposite group's start)
	BoidCompositionSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
	                  std::vector<Vec4> goal);
	virtual ~BoidCompositionSH();

	void simulate(float dt);

	bool supportsExactReference() { return true; }
	bool hasAccuracyMetrics() { return true; }
	void setAccuracyRecording(bool on) { recording = on; }
	std::string getAccuracyCSV(const std::string& extraHeader, const std::string& extraValues, bool header);
	std::string getAccuracySummary();

protected:
	// groups of the four-groups placement (contiguous buckets of groupOf)
	static constexpr int RED = 0, BLUE = 1, ORANGE = 2;
	static constexpr int GROUPS = 4;
	static constexpr int FOLLOW = ORANGE, AVOID = BLUE;
	static constexpr int NUM_DIRS = 1024;         // directions P is maximised / compared over (~6 deg apart)
	static constexpr float NEG_LOBE = 0.05f;      // P below -NEG_LOBE counts as a spurious negative lobe
	static constexpr float W_COMPOSITE = 2.0f;    // gain of red's composite steering
	static constexpr float W_PATH = 2.0f;         // goal seeking of the other groups (as the tunnel)

	float* d_groupSHvmf;    // vMF group coefficients (d_groupSH holds the Dirac ones)
	sf4*   d_goal;          // per-boid goal, original order
	float* d_gaunt;         // 9 x 9 x 9 Gaunt tensor, G[i*81 + j*9 + k]
	sf4*   d_dirs;          // NUM_DIRS unit directions (Fibonacci sphere)
	float* d_dirY;          // their 9 SH basis values, dirY[d*9 + k]
	float* d_refTable;      // exact factor values at the directions: [follow | avoid] x NUM_DIRS
	float* d_compSH;        // G = F_follow x (1 - F_avoid): Dirac [0..8], vMF [9..17]
	float* d_refG;          // exact G at the directions (NUM_DIRS)

	std::vector<float> gaunt, dirY;
	std::vector<sf4> dirs;

	AccuracyProbe probe;
	bool recording;

	// host: SH product truncated to order 3, c = a x b
	void gauntProduct(const float* a, const float* b, float* c) const;
	// host: normalise a group field by its maximum over the directions, take the
	// complement of the avoided one and form their product
	void composeFactors(const float* follow, const float* avoid, float* out) const;
};

#endif // _BOIDCOMPOSITIONSH_H_
