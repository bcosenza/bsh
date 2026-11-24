// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidCompositionSH.h"
#include "vectorTypes.h"
#include <chrono>
#include <cmath>

// Gauss-Legendre nodes / weights on [-1, 1] (Newton on P_n)
static void gaussLegendre(int n, std::vector<double>& x, std::vector<double>& w){
	x.resize(n); w.resize(n);
	for (int i = 0; i < n; i++){
		double t = std::cos(M_PI * (i + 0.75) / (n + 0.5)), dp = 1.0;
		for (int it = 0; it < 100; it++){
			double p0 = 1.0, p1 = t;
			for (int k = 2; k <= n; k++){ const double p2 = ((2 * k - 1) * t * p1 - (k - 1) * p0) / k; p0 = p1; p1 = p2; }
			dp = n * (t * p1 - p0) / (t * t - 1.0);
			const double dt = p1 / dp;
			t -= dt;
			if (std::fabs(dt) < 1e-15) break;
		}
		x[i] = t;
		w[i] = 2.0 / ((1.0 - t * t) * dp * dp);
	}
}

BoidCompositionSH::BoidCompositionSH(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color, simParams_t* simP,
                                     std::vector<Vec4> goal)
	: BoidGroupAvoidanceSH(pos, vel, color, simP, GROUPS)
{
	simTimeDisc[0] = "SH accuracy: four groups composition";
	recording = false;

	// --- Gaunt tensor G_ijk = int Y_i Y_j Y_k over the sphere, in the basis of
	// shEval3 (so conventions match). The integrand is a polynomial of degree 6:
	// 8 Gauss-Legendre nodes in cos(theta) x 16 uniform azimuths integrate it exactly.
	gaunt.assign(SH_COEFFS * SH_COEFFS * SH_COEFFS, 0.0f);
	{
		std::vector<double> gx, gw;
		gaussLegendre(8, gx, gw);
		const int nPhi = 16;
		std::vector<double> G(gaunt.size(), 0.0);
		for (int a = 0; a < 8; a++){
			const double z = gx[a], r = std::sqrt(std::max(0.0, 1.0 - z * z));
			for (int b = 0; b < nPhi; b++){
				const double phi = 2.0 * M_PI * b / nPhi;
				float y8[8];
				shEval3((float)(r * std::cos(phi)), (float)(r * std::sin(phi)), (float)z, y8);
				double Y[SH_COEFFS];
				Y[0] = SH_BAND0;
				for (int k = 0; k < 8; k++) Y[1 + k] = y8[k];
				const double wq = gw[a] * 2.0 * M_PI / nPhi;
				for (int i = 0; i < SH_COEFFS; i++)
					for (int j = 0; j < SH_COEFFS; j++)
						for (int k = 0; k < SH_COEFFS; k++)
							G[(i * SH_COEFFS + j) * SH_COEFFS + k] += wq * Y[i] * Y[j] * Y[k];
			}
		}
		for (size_t i = 0; i < G.size(); i++) gaunt[i] = (std::fabs(G[i]) < 1e-7) ? 0.0f : (float)G[i];
	}

	// --- NUM_DIRS directions on a Fibonacci sphere, and their basis values
	dirs.resize(NUM_DIRS);
	dirY.resize((size_t)NUM_DIRS * SH_COEFFS);
	for (int d = 0; d < NUM_DIRS; d++){
		const double z = 1.0 - (2.0 * d + 1.0) / NUM_DIRS, r = std::sqrt(1.0 - z * z);
		const double phi = d * M_PI * (3.0 - std::sqrt(5.0));
		dirs[d] = sf4((float)(r * std::cos(phi)), (float)z, (float)(r * std::sin(phi)), 0.0f);   // any frame: uniform
		float y8[8];
		shEval3(dirs[d].x(), dirs[d].y(), dirs[d].z(), y8);
		dirY[(size_t)d * SH_COEFFS] = SH_BAND0;
		for (int k = 0; k < 8; k++) dirY[(size_t)d * SH_COEFFS + 1 + k] = y8[k];
	}

	d_groupSHvmf = sycl::malloc_device<float>(numGroups * SH_COEFFS, queue);
	d_goal       = sycl::malloc_device<sf4>(num, queue);
	d_gaunt      = sycl::malloc_device<float>(gaunt.size(), queue);
	d_dirs       = sycl::malloc_device<sf4>(NUM_DIRS, queue);
	d_dirY       = sycl::malloc_device<float>(dirY.size(), queue);
	d_refTable   = sycl::malloc_device<float>(2 * NUM_DIRS, queue);
	d_compSH     = sycl::malloc_device<float>(2 * SH_COEFFS, queue);
	d_refG       = sycl::malloc_device<float>(NUM_DIRS, queue);
	queue.memcpy(d_goal, goal.data(), (size_t)num * sizeof(Vec4));   // Vec4 == sf4 layout
	queue.memcpy(d_gaunt, gaunt.data(), gaunt.size() * sizeof(float));
	queue.memcpy(d_dirs, dirs.data(), dirs.size() * sizeof(sf4));
	queue.memcpy(d_dirY, dirY.data(), dirY.size() * sizeof(float));
	queue.fill(d_compSH, 0.0f, 2 * SH_COEFFS);
	queue.fill(d_refG, 0.0f, NUM_DIRS);

	// only the red group is under study
	std::vector<int> laneOf(num);
	for (int i = 0; i < num; i++) laneOf[i] = (groupOf(i, num, numGroups) == RED) ? 0 : -1;
	probe.init(queue, laneOf, std::vector<float>(1, 0.0f), "group");
	queue.wait();
}

BoidCompositionSH::~BoidCompositionSH(){
	queue.wait();
	sycl::free(d_groupSHvmf, queue);
	sycl::free(d_goal, queue);
	sycl::free(d_gaunt, queue);
	sycl::free(d_dirs, queue);
	sycl::free(d_dirY, queue);
	sycl::free(d_refTable, queue);
	sycl::free(d_compSH, queue);
	sycl::free(d_refG, queue);
	probe.release(queue);
}

void BoidCompositionSH::gauntProduct(const float* a, const float* b, float* c) const {
	for (int k = 0; k < SH_COEFFS; k++){
		double s = 0.0;
		for (int i = 0; i < SH_COEFFS; i++)
			for (int j = 0; j < SH_COEFFS; j++)
				s += (double)gaunt[(i * SH_COEFFS + j) * SH_COEFFS + k] * a[i] * b[j];
		c[k] = (float)s;
	}
}

void BoidCompositionSH::composeFactors(const float* follow, const float* avoid, float* out) const {
	// maximum of an SH field over the sampled directions
	auto fieldMax = [this](const float* c){
		float m = -1e30f;
		for (int d = 0; d < NUM_DIRS; d++){
			float v = 0.0f;
			for (int k = 0; k < SH_COEFFS; k++) v += c[k] * dirY[(size_t)d * SH_COEFFS + k];
			m = std::max(m, v);
		}
		return m;
	};
	float f[SH_COEFFS], a[SH_COEFFS];
	const float mf = fieldMax(follow), ma = fieldMax(avoid);
	for (int k = 0; k < SH_COEFFS; k++){
		f[k] = (mf > 1e-12f) ? follow[k] / mf : 0.0f;
		a[k] = (ma > 1e-12f) ? -avoid[k] / ma : 0.0f;   // complement: 1 - F
	}
	a[0] += (float)std::sqrt(4.0 * M_PI);                 // the constant 1 is sqrt(4 pi) Y_0
	gauntProduct(f, a, out);
}

void BoidCompositionSH::simulate(float dt){
	const int         n  = num;
	const int         ng = numGroups;
	const simParams_t sp = simParams;

	const float cs   = cellSize;
	const int   dimX = gridDimX, dimY = gridDimY, dimZ = gridDimZ;
	const sf4   origin = gridOrigin;

	const int  mode    = (int)shMode;
	const bool rec     = recording;
	const bool needD   = rec || shMode == SHMode::Dirac;
	const bool needV   = rec || shMode == SHMode::VMF;
	const bool needRef = rec || shMode == SHMode::Exact;

	auto t0 = std::chrono::high_resolution_clock::now();
	resetPhaseEvents();

	// --- the group heading fields with the kernels needed
	if (needD && needV){
		PhaseEvents a = projectGroupSHImpl(SHDirac(), d_groupSH);
		PhaseEvents b = projectGroupSHImpl(vmfPolicy(), d_groupSHvmf);
		evProject.set(a.first, b.last);
	}
	else if (needD) evProject = projectGroupSHImpl(SHDirac(), d_groupSH);
	else if (needV) evProject = projectGroupSHImpl(vmfPolicy(), d_groupSHvmf);

	// --- exact factor values at the directions: unnormalised vMF heading densities
	// of the followed and the avoided group (normalised by their maximum below)
	if (needRef){
		const sf4* vel  = d_vel;
		const sf4* dirs_ = d_dirs;
		float* table = d_refTable;
		const float kRef = vmfK;
		const int loF = groupStart(FOLLOW, n, ng), hiF = groupStart(FOLLOW + 1, n, ng);
		const int loA = groupStart(AVOID, n, ng),  hiA = groupStart(AVOID + 1, n, ng);
		queue.parallel_for(sycl::range<1>(2 * NUM_DIRS), [=](sycl::id<1> idx){
			const int t = (int)idx[0];
			const int which = t / NUM_DIRS, d = t % NUM_DIRS;
			const int lo = which ? loA : loF, hi = which ? hiA : hiF;
			const sf4 w = dirs_[d];
			float s = 0.0f;
			for (int j = lo; j < hi; j++)
				s += sycl::exp(kRef * (sycl::dot(w, safeNormalize(vel[j])) - 1.0f));
			table[t] = s;
		});
	}
	queue.wait();

	// --- host: compose the factors. SH: G = F_follow x (1 - F_avoid) per kernel;
	// reference: the same product pointwise on the exact factors
	{
		std::vector<float> c(SH_COEFFS * GROUPS);
		float comp[2 * SH_COEFFS] = { 0 };
		if (needD){
			queue.memcpy(c.data(), d_groupSH, c.size() * sizeof(float)).wait();
			composeFactors(&c[FOLLOW * SH_COEFFS], &c[AVOID * SH_COEFFS], comp);
		}
		if (needV){
			queue.memcpy(c.data(), d_groupSHvmf, c.size() * sizeof(float)).wait();
			composeFactors(&c[FOLLOW * SH_COEFFS], &c[AVOID * SH_COEFFS], comp + SH_COEFFS);
		}
		queue.memcpy(d_compSH, comp, sizeof(comp));
		if (needRef){
			std::vector<float> t(2 * NUM_DIRS), g(NUM_DIRS);
			queue.memcpy(t.data(), d_refTable, t.size() * sizeof(float)).wait();
			float mf = 0.0f, ma = 0.0f;
			for (int d = 0; d < NUM_DIRS; d++){ mf = std::max(mf, t[d]); ma = std::max(ma, t[NUM_DIRS + d]); }
			for (int d = 0; d < NUM_DIRS; d++){
				const float f = (mf > 0.0f) ? t[d] / mf : 0.0f;
				const float a = (ma > 0.0f) ? t[NUM_DIRS + d] / ma : 0.0f;
				g[d] = f * (1.0f - a);
			}
			queue.memcpy(d_refG, g.data(), g.size() * sizeof(float)).wait();   // g is local
		}
		queue.wait();
	}

	buildGrid();

	sf4* sortedPos = d_pos_out;
	sf4* sortedVel = d_vel_out;
	sf4* outPos    = d_pos;
	sf4* outVel    = d_vel;
	int* cellStart = d_cellStart;
	int* cellEnd   = d_cellEnd;
	unsigned int* index = d_index;
	const sf4*   goal   = d_goal;
	const float* G      = d_gaunt;
	const sf4*   dirs_  = d_dirs;
	const float* dY     = d_dirY;
	const float* comp   = d_compSH;
	const float* refG   = d_refG;
	const float* shD    = d_groupSH;
	float*       recBuf = probe.device();
	const int    REC    = AccuracyProbe::REC;
	const float  w1 = (float)M_PI * (2.0f / 3.0f), w2 = (float)M_PI * 0.25f;   // clamped cosine bands

	sycl::event ev = queue.parallel_for(sycl::range<1>(n), [=](sycl::id<1> idx){
		const int k = (int)idx[0];
		sf4 p = sortedPos[k];
		sf4 v = sortedVel[k];

		FlockAccum acc = gridAccumulate(p, v, sortedPos, sortedVel,
		                                cellStart, cellEnd, cs, dimX, dimY, dimZ, origin);
		const sf4 steer = finalizeSteer(acc, p, v, sp);

		const int orig = (int)index[k];
		const sf4 u = safeNormalize(goal[orig] - p);   // toward the goal
		sf4 extra;
		if (groupOf(orig, n, ng) != RED){
			extra = u * W_PATH;                        // the other groups just seek their goals
		}
		else {
			const sf4 h = safeNormalize(v);

			// Cone = max(0, w . u) in SH: band factors pi, 2pi/3, pi/4 times Y(u)
			float b[8];
			shEval3(u.x(), u.y(), u.z(), b);
			float cone[SH_COEFFS];
			cone[0] = (float)M_PI * SH_BAND0;
			for (int q = 0; q < 3; q++) cone[1 + q] = w1 * b[q];
			for (int q = 3; q < 8; q++) cone[1 + q] = w2 * b[q];

			// P = Cone x G (Gaunt product, truncated to order 3) for the kernels needed
			float PD[SH_COEFFS], PV[SH_COEFFS];
			for (int q = 0; q < SH_COEFFS; q++){
				float sd = 0.0f, sv = 0.0f;
				for (int i = 0; i < SH_COEFFS; i++)
					for (int j = 0; j < SH_COEFFS; j++){
						const float gij = G[(i * SH_COEFFS + j) * SH_COEFFS + q] * cone[i];
						sd += gij * comp[j];
						sv += gij * comp[SH_COEFFS + j];
					}
				PD[q] = needD ? sd : 0.0f;
				PV[q] = needV ? sv : 0.0f;
			}

			// maximise P over the directions; compare with the reference pointwise
			int bestD = 0, bestV = 0, bestR = 0;
			float maxD = -1e30f, maxV = -1e30f, maxR = -1e30f;
			float seD = 0.0f, seV = 0.0f, ssR = 0.0f, negD = 0.0f, negV = 0.0f;
			for (int d = 0; d < NUM_DIRS; d++){
				const float* y = dY + d * SH_COEFFS;
				float pd = 0.0f, pv = 0.0f;
				for (int q = 0; q < SH_COEFFS; q++){ pd += PD[q] * y[q]; pv += PV[q] * y[q]; }
				const float pr = needRef ? sycl::fmax(sycl::dot(dirs_[d], u), 0.0f) * refG[d] : 0.0f;
				if (pd > maxD){ maxD = pd; bestD = d; }
				if (pv > maxV){ maxV = pv; bestV = d; }
				if (pr > maxR){ maxR = pr; bestR = d; }
				seD += (pd - pr) * (pd - pr);
				seV += (pv - pr) * (pv - pr);
				ssR += pr * pr;
				negD += (pd < -NEG_LOBE) ? 1.0f : 0.0f;
				negV += (pv < -NEG_LOBE) ? 1.0f : 0.0f;
			}

			// steer toward the preferred direction
			const sf4 sD = needD   ? (dirs_[bestD] - h) * W_COMPOSITE : sf4(0.0f);
			const sf4 sV = needV   ? (dirs_[bestV] - h) * W_COMPOSITE : sf4(0.0f);
			const sf4 sR = needRef ? (dirs_[bestR] - h) * W_COMPOSITE : sf4(0.0f);
			extra = (mode == (int)SHMode::Exact) ? sR : (mode == (int)SHMode::VMF) ? sV : sD;

			if (rec){
				// reference preference of each variant's chosen direction
				auto prefRef = [&](int d){ return sycl::fmax(sycl::dot(dirs_[d], u), 0.0f) * refG[d]; };
				// red's alignment with orange's mean heading (band 1, kernel independent)
				const sf4 mF = shDominantHeading(shD + FOLLOW * SH_COEFFS);
				const float cosF = sycl::fmin(sycl::fmax(sycl::dot(h, mF), -1.0f), 1.0f);

				// the chosen directions w* (the steering W (w* - h) is affine in w*, so
				// its error is W times theirs; their angle is the direction error)
				const sf4 wD = dirs_[bestD], wV = dirs_[bestV], wR = dirs_[bestR];
				float* r = recBuf + (size_t)orig * REC;
				r[AccuracyProbe::S_REF] = wR.x(); r[AccuracyProbe::S_REF + 1] = wR.y(); r[AccuracyProbe::S_REF + 2] = wR.z();
				r[AccuracyProbe::S_D]   = wD.x(); r[AccuracyProbe::S_D + 1]   = wD.y(); r[AccuracyProbe::S_D + 2]   = wD.z();
				r[AccuracyProbe::S_V]   = wV.x(); r[AccuracyProbe::S_V + 1]   = wV.y(); r[AccuracyProbe::S_V + 2]   = wV.z();
				r[AccuracyProbe::AUX_REF] = prefRef(bestR);
				r[AccuracyProbe::AUX_D]   = prefRef(bestD);
				r[AccuracyProbe::AUX_V]   = prefRef(bestV);
				r[AccuracyProbe::B_DIST]  = sycl::acos(cosF) * (180.0f / (float)M_PI);
				r[AccuracyProbe::B_EVENT] = (sycl::dot(h, u) > 0.0f) ? 1.0f : 0.0f;
				r[AccuracyProbe::FIELD_SE_D]   = seD;
				r[AccuracyProbe::FIELD_SE_V]   = seV;
				r[AccuracyProbe::FIELD_SS_REF] = ssR;
				r[AccuracyProbe::FIELD_NEG_D]  = negD / NUM_DIRS;
				r[AccuracyProbe::FIELD_NEG_V]  = negV / NUM_DIRS;
			}
		}

		advanceBoid(p, v, steer + extra, sp, dt);
		outPos[orig] = p;
		outVel[orig] = v;
	});
	evInteract.set(ev, ev);
	queue.wait();
	collectPhaseTimes();
	if (rec) probe.collect(queue);

	auto t1 = std::chrono::high_resolution_clock::now();
	lastSimTimeMs = (long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

	bool pingPong = ((helper++ % 2) == 0);
	uploadState(outPos, outVel, pingPong);
}

static const char* driverName(SHMode m){
	return (m == SHMode::Exact) ? "exact" : (m == SHMode::VMF) ? "vmf" : "dirac";
}

std::string BoidCompositionSH::getAccuracyCSV(const std::string& extraHeader, const std::string& extraValues, bool header){
	return probe.csv(extraHeader, extraValues, driverName(shMode), vmfK, header);
}

std::string BoidCompositionSH::getAccuracySummary(){
	std::stringstream t;
	t << "accuracy: four groups composition (Gaunt products), driver " << driverName(shMode) << ", k " << vmfK;
	return probe.summary(t.str(), "ang orange", "on course");
}
