// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "AccuracyProbe.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

// a steering vector shorter than this counts as "no steering" (missed / spurious)
static const double ACTIVE_EPS = 1e-4;

void AccuracyProbe::init(sycl::queue& q, const std::vector<int>& laneOfAgent,
                         const std::vector<float>& laneValue, const char* laneLabel){
	laneOf = laneOfAgent;
	laneVal = laneValue;
	label = laneLabel;
	lanes.assign(laneValue.size(), Lane());
	host.resize(laneOf.size() * REC);
	steps = 0;
	d_rec = sycl::malloc_device<float>(host.size(), q);
	q.fill(d_rec, 0.0f, host.size()).wait();
}

void AccuracyProbe::release(sycl::queue& q){
	if (d_rec) sycl::free(d_rec, q);
	d_rec = nullptr;
}

void AccuracyProbe::accumulateErr(Err& e, const float* s, const float* r, float aux, float auxRef){
	const double sr = s[0] * r[0] + s[1] * r[1] + s[2] * r[2];
	const double ss = s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
	const double rr = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
	const double dx = s[0] - r[0], dy = s[1] - r[1], dz = s[2] - r[2];
	const double e2 = dx * dx + dy * dy + dz * dz;
	e.dot += sr;
	e.ss += ss;
	e.se += e2;
	e.maxe = std::max(e.maxe, std::sqrt(e2));

	const double ls = std::sqrt(ss), lr = std::sqrt(rr);
	const bool sOn = ls > ACTIVE_EPS, rOn = lr > ACTIVE_EPS;
	if (sOn && rOn){
		const double c = std::max(-1.0, std::min(1.0, sr / (ls * lr)));
		e.ang += std::acos(c) * 180.0 / M_PI;
		e.angN += 1;
	}
	else if (rOn) e.missed += 1;
	else if (sOn) e.spurious += 1;

	e.auxErr += std::fabs((double)aux - auxRef);
	if (aux < 0.0f) e.neg += 1;
}

void AccuracyProbe::collect(sycl::queue& q){
	q.memcpy(host.data(), d_rec, host.size() * sizeof(float)).wait();
	steps++;
	const int n = (int)laneOf.size();
	for (int i = 0; i < n; i++){
		const float* r = host.data() + (size_t)i * REC;
		Lane& L = lanes[laneOf[i]];
		L.n += 1;
		L.ssRef += r[S_REF] * r[S_REF] + r[S_REF + 1] * r[S_REF + 1] + r[S_REF + 2] * r[S_REF + 2];
		L.auxRef += r[AUX_REF];
		if (r[AUX_REF] < 0.0f) L.negRef += 1;
		accumulateErr(L.d, r + S_D, r + S_REF, r[AUX_D], r[AUX_REF]);
		accumulateErr(L.v, r + S_V, r + S_REF, r[AUX_V], r[AUX_REF]);
		L.minDist = std::min(L.minDist, (double)r[B_DIST]);
		L.events += r[B_EVENT];
	}
}

// RMS error relative to the reference RMS, before and after the best scalar gain
static void errorStats(double n, double ssRef, double dot, double ss, double se,
                       double& nrmse, double& gain, double& nrmseFit){
	const double refRms = (n > 0) ? std::sqrt(ssRef / n) : 0.0;
	const double den = (refRms > 0) ? refRms : 1.0;
	nrmse = (n > 0) ? std::sqrt(se / n) / den : 0.0;
	gain = (ss > 0) ? dot / ss : 0.0;
	const double resid = (ss > 0) ? std::max(0.0, ssRef - dot * dot / ss) : ssRef;
	nrmseFit = (n > 0) ? std::sqrt(resid / n) / den : 0.0;
}

std::string AccuracyProbe::csv(const std::string& extraHeader, const std::string& extraValues,
                               const std::string& driver, float k, bool header) const {
	std::ostringstream s;
	s.precision(10);
	if (header){
		if (!extraHeader.empty()) s << extraHeader << ",";
		s << "lane," << label << ",driver,k,steps,agent_steps,ref_rms"
		     ",D_nrmse,V_nrmse,D_gain,V_gain,D_nrmse_fit,V_nrmse_fit,D_maxerr,V_maxerr"
		     ",D_angle_deg,V_angle_deg,D_missed,V_missed,D_spurious,V_spurious"
		     ",aux_ref_mean,D_aux_mae,V_aux_mae,ref_neg,D_neg,V_neg,min_dist,events"
		     // raw sums, so runs and lanes can be pooled exactly
		     ",ref_ss,D_dot,D_ss,D_se,D_angsum,D_angN,V_dot,V_ss,V_se,V_angsum,V_angN\n";
	}
	for (size_t l = 0; l < lanes.size(); l++){
		const Lane& L = lanes[l];
		const double n = (L.n > 0) ? L.n : 1.0;
		double dN, dG, dF, vN, vG, vF;
		errorStats(L.n, L.ssRef, L.d.dot, L.d.ss, L.d.se, dN, dG, dF);
		errorStats(L.n, L.ssRef, L.v.dot, L.v.ss, L.v.se, vN, vG, vF);
		if (!extraValues.empty()) s << extraValues << ",";
		s << l << "," << laneVal[l] << "," << driver << "," << k << "," << steps << "," << (long)L.n
		  << "," << std::sqrt(L.ssRef / n)
		  << "," << dN << "," << vN << "," << dG << "," << vG << "," << dF << "," << vF
		  << "," << L.d.maxe << "," << L.v.maxe
		  << "," << (L.d.angN > 0 ? L.d.ang / L.d.angN : 0.0) << "," << (L.v.angN > 0 ? L.v.ang / L.v.angN : 0.0)
		  << "," << L.d.missed / n << "," << L.v.missed / n << "," << L.d.spurious / n << "," << L.v.spurious / n
		  << "," << L.auxRef / n << "," << L.d.auxErr / n << "," << L.v.auxErr / n
		  << "," << L.negRef / n << "," << L.d.neg / n << "," << L.v.neg / n
		  << "," << L.minDist << "," << L.events
		  << "," << L.ssRef << "," << L.d.dot << "," << L.d.ss << "," << L.d.se << "," << L.d.ang << "," << L.d.angN
		  << "," << L.v.dot << "," << L.v.ss << "," << L.v.se << "," << L.v.ang << "," << L.v.angN << "\n";
	}
	return s.str();
}

std::string AccuracyProbe::summary(const std::string& title, const std::string& distLabel,
                                   const std::string& eventLabel) const {
	std::ostringstream s;
	s << title << ", " << steps << " steps\n";
	char line[256];
	snprintf(line, sizeof(line), "  %4s %6s %9s | %7s %7s | %7s %7s | %6s %6s | %6s %6s | %9s %8s\n",
	         "lane", label.c_str(), "ref rms", "nRMSE D", "nRMSE V", "fit D", "fit V",
	         "gain D", "gain V", "ang D", "ang V", distLabel.c_str(), eventLabel.c_str());
	s << line;
	for (size_t l = 0; l < lanes.size(); l++){
		const Lane& L = lanes[l];
		const double n = (L.n > 0) ? L.n : 1.0;
		double dN, dG, dF, vN, vG, vF;
		errorStats(L.n, L.ssRef, L.d.dot, L.d.ss, L.d.se, dN, dG, dF);
		errorStats(L.n, L.ssRef, L.v.dot, L.v.ss, L.v.se, vN, vG, vF);
		snprintf(line, sizeof(line), "  %4zu %6.0f %9.4f | %7.3f %7.3f | %7.3f %7.3f | %6.2f %6.2f | %6.1f %6.1f | %9.2f %8.0f\n",
		         l, laneVal[l], std::sqrt(L.ssRef / n), dN, vN, dF, vF, dG, vG,
		         L.d.angN > 0 ? L.d.ang / L.d.angN : 0.0, L.v.angN > 0 ? L.v.ang / L.v.angN : 0.0,
		         L.minDist, L.events);
		s << line;
	}
	s << "  nRMSE: RMS error / RMS reference; fit: after the best scalar gain; ang: mean angle\n"
	     "  to the reference (deg); D = Dirac SH, V = vMF SH, both on the same states.\n";
	return s.str();
}
