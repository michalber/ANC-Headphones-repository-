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
//#include "config.h"
#include "ARM_NLMS.h"
//#include <liquid/liquid.h>

#define FRAMES_PER_BUFFER 1024
#define NUM_OF_TAPS 250
#define MU (float)0.001
#define BUFFER_SIZE FRAMES_PER_BUFFER * 2

namespace Adaptive {
	class NLMS
	{
		arm_lms_norm_instance_f32 lmsNorm_instance;
		
		int NumOfTaps;	//filter size
		float mu;	//step size
		float a;	//R factor

		int bufferSize = 0;

		// Signal vectors
		float y[FRAMES_PER_BUFFER] = { 0 };			// Output data
		float e[FRAMES_PER_BUFFER] = { 0 };			// Error data
		float w1[NUM_OF_TAPS] = { 0 };		// filter coeff.
		float xx[NUM_OF_TAPS] = { 0 };		// vector to apply filter to
		float k = { 0 };

		float lmsStateF32[NUM_OF_TAPS + FRAMES_PER_BUFFER] = { 0.1 };
		float lmsNormCoeff_f32[NUM_OF_TAPS] = { 0.1 };

		float antyaliasingFIRStateF32[60 + FRAMES_PER_BUFFER / 2];
		float antyaliasingFilterCoeff[60];

		float errOutput[FRAMES_PER_BUFFER] = { 0 };
		float Out[FRAMES_PER_BUFFER] = { 0 };
		float AntyAliasOut[FRAMES_PER_BUFFER] = { 0 };

		int numTaps;
		float energy;
		float x0;

	public:
		NLMS(void);
		NLMS(int size);
		NLMS(int x, float y, float z);
		~NLMS();

		inline float* processNLMS(float *d, float *x, float *outL, float *outR);		
		inline float* getOutputSignal();
		inline float* getErrorSignal();
		inline float* getCoeff();

	private:		
		void pushBack(float *a, float x);
	};
};

namespace Adaptive {

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Default constructor of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/
	NLMS::NLMS() : NumOfTaps(100), mu(0.5), a(0.0001)
	{
		//liquid_firdes_kaiser(NUM_OF_TAPS, 0.5, 60.0f, 0.0f, lmsNormCoeff_f32);
		arm_lms_norm_init_f32(&lmsNorm_instance, NUM_OF_TAPS, lmsNormCoeff_f32, lmsStateF32, MU, FRAMES_PER_BUFFER);

		//liquid_firdes_kaiser(NUM_OF_TAPS, 0.5, 60.0f, 0.0f, antyaliasingFilterCoeff);
		//arm_fir_init_f32(&antyaliasingFIR_instance, 60, (float *)antyaliasingFilterCoeff, antyaliasingFIRStateF32, FRAMES_PER_BUFFER);
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Default constructor of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/
	NLMS::NLMS(int buffSize) : bufferSize(buffSize)
	{
		//liquid_firdes_kaiser(NUM_OF_TAPS, 0.5, 60.0f, 0.0f, lmsNormCoeff_f32);
		arm_lms_norm_init_f32(&lmsNorm_instance, NUM_OF_TAPS, lmsNormCoeff_f32, lmsStateF32, MU, bufferSize);

		//liquid_firdes_kaiser(NUM_OF_TAPS, 0.5, 60.0f, 0.0f, antyaliasingFilterCoeff);
		//arm_fir_init_f32(&antyaliasingFIR_instance, 60, (float *)antyaliasingFilterCoeff, antyaliasingFIRStateF32, FRAMES_PER_BUFFER);
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Parametrized constructor of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param filterSize
		@param stepSize
		@param RFactor
	*/
	NLMS::NLMS(int filterSize, float stepSize, float RFactor) : NumOfTaps(filterSize), mu(stepSize), a(RFactor)
	{
		//liquid_firdes_kaiser(NUM_OF_TAPS, 0.5, 60.0f, 0.0f, lmsNormCoeff_f32);
		arm_lms_norm_init_f32(&lmsNorm_instance, NUM_OF_TAPS, lmsNormCoeff_f32, lmsStateF32, MU, FRAMES_PER_BUFFER / 2);

		//liquid_firdes_kaiser(NUM_OF_TAPS, 0.5, 60.0f, 0.0f, antyaliasingFilterCoeff);
		//arm_fir_init_f32(&antyaliasingFIR_instance, 60, (float *)antyaliasingFilterCoeff, antyaliasingFIRStateF32, FRAMES_PER_BUFFER);
	}

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Destructor of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/
	NLMS::~NLMS()
	{
	}

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief processNLMS method of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param vector<float> d - destiny sound input data
		@param vector<float> x - noise sound input data
	*/

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
	inline float * NLMS::getOutputSignal()
	{
		return &Out[0];
	}
	inline float * NLMS::getErrorSignal()
	{
		return &errOutput[0];
	}
	inline float* NLMS::getCoeff() {
		return lmsNormCoeff_f32;
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief processNLMS method of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param float d - destiny sound input data
		@param float x - noise sound input data
	*/
	inline float* NLMS::processNLMS(float *d, float *x, float *outL, float *outR) {
		//static int i = 0;
		//static int j = 0;
		//static int k = 0;
		//static float temp;

		//temp = 0;
		//memset(&y[0], 0, NUM_OF_TAPS * sizeof(float));

		//for (i = 0; i < FRAMES_PER_BUFFER; i++) {
		//	pushBack(&xx[0], x[i]);

		//	// filter input data
		//	for (j = 0; j < NUM_OF_TAPS; j++) {
		//		y[i] += w1[j] * xx[j];
		//		temp += xx[j] * xx[j];
		//	}
		//	
		//	k = mu / (a + temp);

		//	// calculate error data
		//	e[i] = d[i] - y[i];

		//	// update filter coeff.
		//	for (k = 0; k < NUM_OF_TAPS; k++) {
		//		w1[k] += (k * e[i] * xx[k]);
		//	}
		//}		
		//return &y[0];

		//static float tempX[BUFFER_SIZE / 2] = { 0 };
		//static float tempD[BUFFER_SIZE / 2] = { 0 };

		//for (int i = 0; i < BUFFER_SIZE / 2; i++) {
		//	tempD[i] = d[2 * i];
		//	tempX[i] = d[2 * i + 1];
		//}

		arm_lms_norm_f32(
			&lmsNorm_instance,			/* LMSNorm instance */
			x,							/* Input signal */
			d,							/* Reference Signal */
			Out,						/* Converged Signal */
			errOutput,					/* Error Signal, this will become small as the signal converges */
			bufferSize);				/* BlockSize */

//		memcpy(outL, Out, bufferSize * sizeof(float));
//		memcpy(outR, Out, bufferSize * sizeof(float));

//		arm_fir_f32(&antyaliasingFIR_instance, Out, AntyAliasOut, FRAMES_PER_BUFFER);

		return &Out[0];
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief pushBack method of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param float &a - reference to array to push data to
		@param float x - pushed data
	*/
	void NLMS::pushBack(float *a, float x)
	{		
		//memmove(&a[0],&a[1], (NUM_OF_TAPS - 1) * sizeof(float));
		//a[NUM_OF_TAPS] = x;
		for (int i = 0; i < NUM_OF_TAPS - 1; i++) {
			a[i] = a[i + 1];
		}
		a[NUM_OF_TAPS - 1] = x;
	}
}