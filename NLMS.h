#pragma once
/*
MATLAB code for my NLMS

	x = x;
	xx = zeros(M,1);
	w1 = zeros(M,1);
	y = zeros(Ns,1);
	e = zeros(Ns,1);

	for n = 1:Ns
		xx = [xx(2:M);x(n)];
		y(n) = w1' * xx;
		k = mu/(a + xx'*xx);
		e(n) = d(n) - y(n);
		w1 = w1 + k * e(n) * xx;
		w(:,n) = w1;
	end
*/

#include <vector>
#include <string.h>
#include "config.h"
#include "arm_math.h"
#include <liquid/liquid.h>

namespace Adaptive {
	class NLMS
	{
		arm_lms_norm_instance_f32 lmsNorm_instance;
		arm_fir_instance_f32 antyaliasingFIR_instance;
		
		int NumOfTaps;	//filter size
		float mu;	//step size
		float a;	//R factor

		// Signal vectors
		float y[FRAMES_PER_BUFFER] = { 0 };			// Output data
		float e[FRAMES_PER_BUFFER] = { 0 };			// Error data
		float w1[NUM_OF_TAPS] = { 0 };		// filter coeff.
		float xx[NUM_OF_TAPS] = { 0 };		// vector to apply filter to
		float k = { 0 };

		float32_t lmsStateF32[NUM_OF_TAPS + FRAMES_PER_BUFFER / 2];
		float32_t lmsNormCoeff_f32[NUM_OF_TAPS];

		float32_t antyaliasingFIRStateF32[60 + FRAMES_PER_BUFFER / 2];
		float32_t antyaliasingFilterCoeff[60];

		float32_t errOutput[FRAMES_PER_BUFFER / 2];
		float32_t Out[FRAMES_PER_BUFFER / 2];
		float32_t AntyAliasOut[FRAMES_PER_BUFFER / 2];

		int numTaps;
		float energy;
		float x0;

	public:
		NLMS();
		NLMS(int, float, float);
		~NLMS();

		float* processNLMS(float *d, float *x);		
		float* getOutputSignal();
		float* getErrorSignal();

	private:		
		void pushBack(float *a, double x);
	};
};
