// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _SHPROJECTION_H_
#define _SHPROJECTION_H_

#include "SimParam.h"
#include <cmath>

/*
	SH projection policies, used as template parameters of the SH projection kernels
	(BoidModelSimpleSYCL::shProject and the kernels built on it).

	Projecting a direction d into order-3 SH means taking the coefficients of some
	kernel centred on d. Both policies below are zonal kernels, so they only rescale
	each band l of the Dirac coefficients Y_lm(d) by a factor a_l (Funk-Hecke):

	  SHDirac          a_l = 1                    the delta function itself (exact
	                                              point projection, max ringing)
	  SHVonMisesFisher a_l = I_{l+1/2}(k) / I_{1/2}(k)
	                                              the von Mises-Fisher density
	                                              exp(k d.w), a smooth lobe of
	                                              concentration k

	For the first three bands the vMF ratio has the closed form
	  a_0 = 1,  a_1 = coth(k) - 1/k,  a_2 = 1 - 3 a_1 / k.
	k -> infinity gives back the Dirac (a_l -> 1), k -> 0 the uniform density
	(a_l -> 0 for l > 0). The concentration is called k (not kappa) in the code,
	since kappa is the obstacle strength in the methods text.
*/

// runtime selector; values match SH_PROJECTION_* in SimParam.h
enum class SHMode : int { Dirac = SH_PROJECTION_DIRAC, VMF = SH_PROJECTION_VMF };

// Dirac projection: coefficients are used as they are
struct SHDirac
{
	void apply(float* /*c[9]*/) const {}
};

// von Mises-Fisher projection: scales band 1 (c[1..3]) by a1 and band 2 (c[4..8]) by
// a2. Trivially copyable, so it can be captured by value into a SYCL kernel.
struct SHVonMisesFisher
{
	float a1, a2;

	void apply(float* c) const
	{
		for (int k = 1; k < 4; k++) c[k] *= a1;
		for (int k = 4; k < 9; k++) c[k] *= a2;
	}

	// band factors for concentration k (host code, in double: a_2 cancels badly in
	// float for small k). Below k = 1e-2 the Taylor series of the closed forms
	// replace them: a_1 = k/3 - k^3/45, a_2 = k^2/15 - 2k^4/315.
	static SHVonMisesFisher fromConcentration(float kf)
	{
		const double k = (kf > 0.0f) ? (double)kf : 0.0;
		double a1, a2;
		if (k < 1e-2){
			a1 = k / 3.0 - k * k * k / 45.0;
			a2 = k * k / 15.0 - 2.0 * k * k * k * k / 315.0;
		} else {
			a1 = 1.0 / std::tanh(k) - 1.0 / k;
			a2 = 1.0 - 3.0 * a1 / k;
		}
		return SHVonMisesFisher{ (float)a1, (float)a2 };
	}
};

#endif // _SHPROJECTION_H_
