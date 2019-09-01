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

namespace Adaptive {
	class NLMS
	{
		int NumOfTaps;	//filter size
		float mu;	//step size
		float a;	//R factor

		// Signal vectors
		float y[FRAMES_PER_BUFFER] = { 0 };			// Output data
		float e[FRAMES_PER_BUFFER] = { 0 };			// Error data
		float w1[NUM_OF_TAPS] = { 0 };		// filter coeff.
		float xx[NUM_OF_TAPS] = { 0 };		// vector to apply filter to
		float k = { 0 };

		float pState[NUM_OF_TAPS + FRAMES_PER_BUFFER] = { 0 };
		float pCoeffs[NUM_OF_TAPS] = { 0.1f };

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

		void arm_lms_norm_init_f32(int numTaps, float mu, int blockSize);
		void arm_lms_norm_f32(float * pSrc, float * pRef, float * pOut, float * pErr, int blockSize);

	private:
		void pushBack(float *a, double x);
	};
};
