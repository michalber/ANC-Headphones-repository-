#include "../ANC-Headphones-repository-/NLMS.h"

namespace Adaptive {

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Default constructor of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/
	NLMS::NLMS() : NumOfTaps(100), mu(0.5), a(0.0001)
	{

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
	float * NLMS::getOutputSignal()
	{
		return &y[0];
	}
	float * NLMS::getErrorSignal()
	{
		return &e[0];
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief processNLMS method of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param float d - destiny sound input data
		@param float x - noise sound input data
	*/
	float* NLMS::processNLMS(float *d, float *x) {
		static int i = 0;
		static int j = 0;
		static int k = 0;
		static float temp;

		temp = 0;
		memset(&y[0], 0, NUM_OF_TAPS * sizeof(float));

		for (i = 0; i < FRAMES_PER_BUFFER; i++) {
			pushBack(&xx[0], x[i]);

			// filter input data
			for (j = 0; j < NUM_OF_TAPS; j++) {
				y[i] += w1[j] * xx[j];
				temp += xx[j] * xx[j];
			}

			k = mu / (a + temp);

			// calculate error data
			e[i] = d[i] - y[i];

			// update filter coeff.
			for (k = 0; k < NUM_OF_TAPS; k++) {
				w1[k] += (k * e[i] * xx[k]);
			}
		}
		return &y[0];
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief pushBack method of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param float &a - reference to array to push data to
		@param float x - pushed data
	*/
	void NLMS::pushBack(float *a, double x)
	{
		//memmove(&a[0],&a[1], (NUM_OF_TAPS - 1) * sizeof(float));
		//a[NUM_OF_TAPS] = x;
		for (int i = 0; i < NUM_OF_TAPS - 1; i++) {
			a[i] = a[i + 1];
		}
		a[NUM_OF_TAPS - 1] = x;
	}



	void NLMS::arm_lms_norm_init_f32(
		int _numTaps,
		float _mu,
		int _blockSize)
	{
		/* Assign filter taps */
		numTaps = _numTaps;

		/* Assign coefficient pointer */
		//pCoeffs = _pCoeffs;

		/* Clear state buffer and size is always blockSize + numTaps - 1 */
		memset(&pState[0], 0, (numTaps + (_blockSize - 1u)) * sizeof(float));
		memset(&pCoeffs[0], 1, (numTaps) * sizeof(float));

		/* Assign state pointer */
		//pState = _pState;

		/* Assign Step size value */
		mu = _mu;

		/* Initialise Energy to zero */
		energy = 0.0f;

		/* Initialise x0 to zero */
		x0 = 0.0f;
	}

	void NLMS::arm_lms_norm_f32(float * pSrc, float * pRef, float * pOut, float * pErr, int blockSize)
	{
		float *_pState = &pState[0];                 /* State pointer */
		float *_pCoeffs = &pCoeffs[0];               /* Coefficient pointer */
		float *pStateCurnt;                        /* Points to the current sample of the state */
		float *px = NULL;                            /* Temporary pointers for state and coefficient buffers */
		float *pb = NULL;
		//float mu = S->mu;                          /* Adaptive factor */
		//int numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
		int tapCnt, blkCnt;                       /* Loop counters */
		float energy;                              /* Energy of the input */
		float sum, e, d;                           /* accumulator, error, reference data sample */
		float w, x0, in;                           /* weight factor, temporary variable to hold input sample and state */

		blkCnt = blockSize;						/* Loop over blockSize number of values */

		/* Initializations of error,  difference, Coefficient update */
		e = 0.0f;
		d = 0.0f;
		w = 0.0f;

		//energy = S->energy;
		//x0 = S->x0;

		/* S->pState points to buffer which contains previous frame (numTaps - 1) samples */
		/* pStateCurnt points to the location where the new input data should be written */
		//pStateCurnt = &(S->pState[(numTaps - 1u)]);
		pStateCurnt = &(_pState[(numTaps - 1u)]);


		/* Run the below code for Cortex-M0 */

		while (blkCnt > 0u)
		{
			/* Copy the new input sample into the state buffer */
			*pStateCurnt++ = *pSrc;

			/* Initialize pCoeffs pointer */
			pb = _pCoeffs;

			/* Initialize pState pointer */
			px = _pState;

			/* Read the sample from input buffer */
			in = *pSrc++;

			/* Update the energy calculation */
			energy -= x0 * x0;
			energy += in * in;

			/* Set the accumulator to zero */
			sum = 0.0f;

			/* Loop over numTaps number of values */
			tapCnt = numTaps;

			while (tapCnt > 0u)
			{
				/* Perform the multiply-accumulate */
				sum += (*px++) * (*pb++);

				/* Decrement the loop counter */
				tapCnt--;
			}

			/* The result in the accumulator is stored in the destination buffer. */
			*pOut++ = sum;

			/* Compute and store error */
			d = (float)(*pRef++);
			e = d - sum;
			*pErr++ = e;

			/* Calculation of Weighting factor for updating filter coefficients */
			/* epsilon value 0.000000119209289f */
			w = (e * mu) / (energy + 0.000000119209289f);

			/* Initialize pState pointer */
			px = _pState;

			/* Initialize pCcoeffs pointer */
			pb = _pCoeffs;

			/* Loop over numTaps number of values */
			tapCnt = numTaps;

			while (tapCnt > 0u)
			{
				/* Perform the multiply-accumulate */
				*pb += w * (*px++);
				pb++;

				/* Decrement the loop counter */
				tapCnt--;
			}

			x0 = *_pState;

			/* Advance state pointer by 1 for the next sample */
			_pState = _pState + 1;

			/* Decrement the loop counter */
			blkCnt--;
		}

		//energy = energy;
		//x0 = x0;

		/* Processing is complete. Now copy the last numTaps - 1 samples to the
		   satrt of the state buffer. This prepares the state buffer for the
		   next function call. */

		   /* Points to the start of the pState buffer */
		//pStateCurnt = pState;

		/* Copy (numTaps - 1u) samples  */
		tapCnt = (numTaps - 1u);

		/* Copy the remaining q31_t data */
		while (tapCnt > 0u)
		{
			*pStateCurnt++ = *_pState++;

			/* Decrement the loop counter */
			tapCnt--;
		}
	}
}