/*
  ==============================================================================

    ARM_NLMS.h
    Created: 13 Sep 2019 12:13:21pm
    Author:  michu

  ==============================================================================
*/

#pragma once

#include <stdio.h>
#include <string.h>
#include <chrono>


/**
  * @brief Instance structure for the floating-point normalized LMS filter.
  */

typedef struct
{
	int  numTaps;    /**< number of coefficients in the filter. */
	float *pState;    /**< points to the state variable array. The array is of length numTaps+blockSize-1. */
	float *pCoeffs;   /**< points to the coefficient array. The array is of length numTaps. */
	float mu;        /**< step size that control filter coefficient updates. */
	float energy;    /**< saves previous frame energy. */
	float x0;        /**< saves previous input sample. */
} arm_lms_norm_instance_f32;

/**
 * @brief Processing function for floating-point normalized LMS filter.
 * @param[in] *S points to an instance of the floating-point normalized LMS filter structure.
 * @param[in] *pSrc points to the block of input data.
 * @param[in] *pRef points to the block of reference data.
 * @param[out] *pOut points to the block of output data.
 * @param[out] *pErr points to the block of error data.
 * @param[in] blockSize number of samples to process.
 * @return none.
 */

inline void arm_lms_norm_f32(
	arm_lms_norm_instance_f32 * S,
	float * pSrc,
	float * pRef,
	float * pOut,
	float * pErr,
	int blockSize);

/**
 * @brief Initialization function for floating-point normalized LMS filter.
 * @param[in] *S points to an instance of the floating-point LMS filter structure.
 * @param[in] numTaps  number of filter coefficients.
 * @param[in] *pCoeffs points to coefficient buffer.
 * @param[in] *pState points to state buffer.
 * @param[in] mu step size that controls filter coefficient updates.
 * @param[in] blockSize number of samples to process.
 * @return none.
 */

void arm_lms_norm_init_f32(
	arm_lms_norm_instance_f32 * S,
	int numTaps,
	float * pCoeffs,
	float * pState,
	float mu,
	int blockSize);


/*-----------------------------------------------------------------------------
* Copyright (C) 2010 ARM Limited. All rights reserved.
*
* $Date:        29. November 2010
* $Revision: 	V1.0.3
*
* Project: 	    CMSIS DSP Library
* Title:        arm_lms_norm_init_f32.c
*
* Description:  Floating-point NLMS filter initialization function.
*
* Target Processor: Cortex-M4/Cortex-M3
*
* Version 1.0.3 2010/11/29
*    Re-organized the CMSIS folders and updated documentation.
*
* Version 1.0.2 2010/11/11
*    Documentation updated.
*
* Version 1.0.1 2010/10/05
*    Production release and review comments incorporated.
*
* Version 1.0.0 2010/09/20
*    Production release and review comments incorporated
*
* Version 0.0.7  2010/06/10
*    Misra-C changes done
* ---------------------------------------------------------------------------*/

/**
 * @ingroup groupFilters
 */

 /**
  * @addtogroup LMS_NORM
  * @{
  */

  /**
   * @brief Initialization function for floating-point normalized LMS filter.
   * @param[in] *S points to an instance of the floating-point LMS filter structure.
   * @param[in] numTaps  number of filter coefficients.
   * @param[in] *pCoeffs points to coefficient buffer.
   * @param[in] *pState points to state buffer.
   * @param[in] mu step size that controls filter coefficient updates.
   * @param[in] blockSize number of samples to process.
   * @return none.
   *
 * \par Description:
 * <code>pCoeffs</code> points to the array of filter coefficients stored in time reversed order:
 * <pre>
 *    {b[numTaps-1], b[numTaps-2], b[N-2], ..., b[1], b[0]}
 * </pre>
 * The initial filter coefficients serve as a starting point for the adaptive filter.
 * <code>pState</code> points to an array of length <code>numTaps+blockSize-1</code> samples,
 * where <code>blockSize</code> is the number of input samples processed by each call to <code>arm_lms_norm_f32()</code>.
 */

void arm_lms_norm_init_f32(
	arm_lms_norm_instance_f32 * S,
	int numTaps,
	float * pCoeffs,
	float * pState,
	float mu,
	int blockSize)
{
	/* Assign filter taps */
	S->numTaps = numTaps;

	/* Assign coefficient pointer */
	S->pCoeffs = pCoeffs;

	/* Clear state buffer and size is always blockSize + numTaps - 1 */
	memset(pState, 0, (numTaps + (blockSize - 1u)) * sizeof(float));

	/* Assign state pointer */
	S->pState = pState;

	/* Assign Step size value */
	S->mu = mu;

	/* Initialise Energy to zero */
	S->energy = 0.0f;

	/* Initialise x0 to zero */
	S->x0 = 0.0f;

}

/**
 * @} end of LMS_NORM group
 */
 /* ----------------------------------------------------------------------
 * Copyright (C) 2010 ARM Limited. All rights reserved.
 *
 * $Date:        29. November 2010
 * $Revision: 	V1.0.3
 *
 * Project: 	    CMSIS DSP Library
 * Title:	    arm_lms_norm_f32.c
 *
 * Description:	Processing function for the floating-point Normalised LMS.
 *
 * Target Processor: Cortex-M4/Cortex-M3
 *
 * Version 1.0.3 2010/11/29
 *    Re-organized the CMSIS folders and updated documentation.
 *
 * Version 1.0.2 2010/11/11
 *    Documentation updated.
 *
 * Version 1.0.1 2010/10/05
 *    Production release and review comments incorporated.
 *
 * Version 1.0.0 2010/09/20
 *    Production release and review comments incorporated
 *
 * Version 0.0.7  2010/06/10
 *    Misra-C changes done
 * -------------------------------------------------------------------- */

 /**
  * @ingroup groupFilters
  */

  /**
   * @defgroup LMS_NORM Normalized LMS Filters
   *
   * This set of functions implements a commonly used adaptive filter.
   * It is related to the Least Mean Square (LMS) adaptive filter and includes an additional normalization
   * factor which increases the adaptation rate of the filter.
   * The CMSIS DSP Library contains normalized LMS filter functions that operate on Q15, Q31, and floating-point data types.
   *
   * A normalized least mean square (NLMS) filter consists of two components as shown below.
   * The first component is a standard transversal or FIR filter.
   * The second component is a coefficient update mechanism.
   * The NLMS filter has two input signals.
   * The "input" feeds the FIR filter while the "reference input" corresponds to the desired output of the FIR filter.
   * That is, the FIR filter coefficients are updated so that the output of the FIR filter matches the reference input.
   * The filter coefficient update mechanism is based on the difference between the FIR filter output and the reference input.
   * This "error signal" tends towards zero as the filter adapts.
   * The NLMS processing functions accept the input and reference input signals and generate the filter output and error signal.
   * \image html LMS.gif "Internal structure of the NLMS adaptive filter"
   *
   * The functions operate on blocks of data and each call to the function processes
   * <code>blockSize</code> samples through the filter.
   * <code>pSrc</code> points to input signal, <code>pRef</code> points to reference signal,
   * <code>pOut</code> points to output signal and <code>pErr</code> points to error signal.
   * All arrays contain <code>blockSize</code> values.
   *
   * The API functions operate on a block-by-block basis.
   * Internally, the filter coefficients <code>b[n]</code> are updated on a sample-by-sample basis.
   * The convergence of the LMS filter is slower compared to the normalized LMS algorithm.
   *
   * \par Algorithm:
   * The output signal <code>y[n]</code> is computed by a standard FIR filter:
   * <pre>
   *     y[n] = b[0] * x[n] + b[1] * x[n-1] + b[2] * x[n-2] + ...+ b[numTaps-1] * x[n-numTaps+1]
   * </pre>
   *
   * \par
   * The error signal equals the difference between the reference signal <code>d[n]</code> and the filter output:
   * <pre>
   *     e[n] = d[n] - y[n].
   * </pre>
   *
   * \par
   * After each sample of the error signal is computed the instanteous energy of the filter state variables is calculated:
   * <pre>
   *    E = x[n]^2 + x[n-1]^2 + ... + x[n-numTaps+1]^2.
   * </pre>
   * The filter coefficients <code>b[k]</code> are then updated on a sample-by-sample basis:
   * <pre>
   *     b[k] = b[k] + e[n] * (mu/E) * x[n-k],  for k=0, 1, ..., numTaps-1
   * </pre>
   * where <code>mu</code> is the step size and controls the rate of coefficient convergence.
   *\par
   * In the APIs, <code>pCoeffs</code> points to a coefficient array of size <code>numTaps</code>.
   * Coefficients are stored in time reversed order.
   * \par
   * <pre>
   *    {b[numTaps-1], b[numTaps-2], b[N-2], ..., b[1], b[0]}
   * </pre>
   * \par
   * <code>pState</code> points to a state array of size <code>numTaps + blockSize - 1</code>.
   * Samples in the state buffer are stored in the order:
   * \par
   * <pre>
   *    {x[n-numTaps+1], x[n-numTaps], x[n-numTaps-1], x[n-numTaps-2]....x[0], x[1], ..., x[blockSize-1]}
   * </pre>
   * \par
   * Note that the length of the state buffer exceeds the length of the coefficient array by <code>blockSize-1</code> samples.
   * The increased state buffer length allows circular addressing, which is traditionally used in FIR filters,
   * to be avoided and yields a significant speed improvement.
   * The state variables are updated after each block of data is processed.
   * \par Instance Structure
   * The coefficients and state variables for a filter are stored together in an instance data structure.
   * A separate instance structure must be defined for each filter and
   * coefficient and state arrays cannot be shared among instances.
   * There are separate instance structure declarations for each of the 3 supported data types.
   *
   * \par Initialization Functions
   * There is also an associated initialization function for each data type.
   * The initialization function performs the following operations:
   * - Sets the values of the internal structure fields.
   * - Zeros out the values in the state buffer.
   * \par
   * Instance structure cannot be placed into a const data section and it is recommended to use the initialization function.
   * \par Fixed-Point Behavior:
   * Care must be taken when using the Q15 and Q31 versions of the normalised LMS filter.
   * The following issues must be considered:
   * - Scaling of coefficients
   * - Overflow and saturation
   *
   * \par Scaling of Coefficients:
   * Filter coefficients are represented as fractional values and
   * coefficients are restricted to lie in the range <code>[-1 +1)</code>.
   * The fixed-point functions have an additional scaling parameter <code>postShift</code>.
   * At the output of the filter's accumulator is a shift register which shifts the result by <code>postShift</code> bits.
   * This essentially scales the filter coefficients by <code>2^postShift</code> and
   * allows the filter coefficients to exceed the range <code>[+1 -1)</code>.
   * The value of <code>postShift</code> is set by the user based on the expected gain through the system being modeled.
   *
   * \par Overflow and Saturation:
   * Overflow and saturation behavior of the fixed-point Q15 and Q31 versions are
   * described separately as part of the function specific documentation below.
   */


   /**
	* @addtogroup LMS_NORM
	* @{
	*/


	/**
	 * @brief Processing function for floating-point normalized LMS filter.
	 * @param[in] *S points to an instance of the floating-point normalized LMS filter structure.
	 * @param[in] *pSrc points to the block of input data.
	 * @param[in] *pRef points to the block of reference data.
	 * @param[out] *pOut points to the block of output data.
	 * @param[out] *pErr points to the block of error data.
	 * @param[in] blockSize number of samples to process.
	 * @return none.
	 */

inline void arm_lms_norm_f32(
	arm_lms_norm_instance_f32 * S,
	float * pSrc,
	float * pRef,
	float * pOut,
	float * pErr,
	int blockSize)
{
	float *pState = S->pState;                 /* State pointer */
	float *pCoeffs = S->pCoeffs;               /* Coefficient pointer */
	float *pStateCurnt;                        /* Points to the current sample of the state */
	float *px, *pb;                            /* Temporary pointers for state and coefficient buffers */
	float mu = S->mu;                          /* Adaptive factor */
	int numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
	int tapCnt, blkCnt;                       /* Loop counters */
	float energy;                              /* Energy of the input */
	float sum, e, d;                           /* accumulator, error, reference data sample */
	float w, x0, in;                           /* weight factor, temporary variable to hold input sample and state */

	/* Initializations of error,  difference, Coefficient update */
	e = 0.0f;
	d = 0.0f;
	w = 0.0f;

	energy = S->energy;
	x0 = S->x0;

	/* S->pState points to buffer which contains previous frame (numTaps - 1) samples */
	/* pStateCurnt points to the location where the new input data should be written */
	pStateCurnt = &(S->pState[(numTaps - 1u)]);

	blkCnt = blockSize;

	while (blkCnt > 0u)
	{
		/* Copy the new input sample into the state buffer */
		*pStateCurnt++ = *pSrc;

		/* Initialize pState pointer */
		px = pState;

		/* Initialize coeff pointer */
		pb = (pCoeffs);

		/* Read the sample from input buffer */
		in = *pSrc++;

		/* Update the energy calculation */
		energy -= x0 * x0;
		energy += in * in;

		/* Set the accumulator to zero */
		sum = 0.0f;

		/* Loop unrolling.  Process 4 taps at a time. */
		tapCnt = numTaps >> 2;

		while (tapCnt > 0u)
		{
			/* Perform the multiply-accumulate */
			sum += (*px++) * (*pb++);
			sum += (*px++) * (*pb++);
			sum += (*px++) * (*pb++);
			sum += (*px++) * (*pb++);

			/* Decrement the loop counter */
			tapCnt--;
		}

		/* If the filter length is not a multiple of 4, compute the remaining filter taps */
		tapCnt = numTaps % 0x4u;

		while (tapCnt > 0u)
		{
			/* Perform the multiply-accumulate */
			sum += (*px++) * (*pb++);

			/* Decrement the loop counter */
			tapCnt--;
		}

		/* The result in the accumulator, store in the destination buffer. */
		*pOut++ = -sum;

		/* Compute and store error */
		d = (float)(*pRef++);
		e = d - sum;
		*pErr++ = e;

		/* Calculation of Weighting factor for updating filter coefficients */
		/* epsilon value 0.000000119209289f */
		w = (e * mu) / (energy + 0.000000119209289f);

		/* Initialize pState pointer */
		px = pState;

		/* Initialize coeff pointer */
		pb = (pCoeffs);

		/* Loop unrolling.  Process 4 taps at a time. */
		tapCnt = numTaps >> 2;

		/* Update filter coefficients */
		while (tapCnt > 0u)
		{
			/* Perform the multiply-accumulate */
			*pb += w * (*px++);
			pb++;

			*pb += w * (*px++);
			pb++;

			*pb += w * (*px++);
			pb++;

			*pb += w * (*px++);
			pb++;


			/* Decrement the loop counter */
			tapCnt--;
		}

		/* If the filter length is not a multiple of 4, compute the remaining filter taps */
		tapCnt = numTaps % 0x4u;

		while (tapCnt > 0u)
		{
			/* Perform the multiply-accumulate */
			*pb += w * (*px++);
			pb++;

			/* Decrement the loop counter */
			tapCnt--;
		}

		x0 = *pState;

		/* Advance state pointer by 1 for the next sample */
		pState = pState + 1;

		/* Decrement the loop counter */
		blkCnt--;
	}

	S->energy = energy;
	S->x0 = x0;

	/* Processing is complete. Now copy the last numTaps - 1 samples to the
	   satrt of the state buffer. This prepares the state buffer for the
	   next function call. */

	   /* Points to the start of the pState buffer */
	pStateCurnt = S->pState;

	/* Loop unrolling for (numTaps - 1u)/4 samples copy */
	tapCnt = (numTaps - 1u) >> 2u;

	/* copy data */
	while (tapCnt > 0u)
	{
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;

		/* Decrement the loop counter */
		tapCnt--;
	}

	/* Calculate remaining number of copies */
	tapCnt = (numTaps - 1u) % 0x4u;

	/* Copy the remaining q31_t data */
	while (tapCnt > 0u)
	{
		*pStateCurnt++ = *pState++;

		/* Decrement the loop counter */
		tapCnt--;
	}
}

/**
   * @} end of LMS_NORM group
   */



   /**
	* @brief  Caluclation of SNR
	* @param  float* 	Pointer to the reference buffer
	* @param  float*	Pointer to the test buffer
	* @param  uint32_t	total number of samples
	* @return float	SNR
	* The function Caluclates signal to noise ratio for the reference output
	* and test output
	*/

float arm_snr_f32(float *pRef, float *pTest, uint32_t buffSize)
{
	float EnergySignal = 0.0, EnergyError = 0.0;
	uint32_t i;
	float SNR;
	int temp;
	int *test;

	for (i = 0; i < buffSize; i++)
	{
		/* Checking for a NAN value in pRef array */
		test = (int *)(&pRef[i]);
		temp = *test;

		if (temp == 0x7FC00000)
		{
			return(0);
		}

		/* Checking for a NAN value in pTest array */
		test = (int *)(&pTest[i]);
		temp = *test;

		if (temp == 0x7FC00000)
		{
			return(0);
		}
		EnergySignal += pRef[i] * pRef[i];
		EnergyError += (pRef[i] - pTest[i]) * (pRef[i] - pTest[i]);
	}

	/* Checking for a NAN value in EnergyError */
	test = (int *)(&EnergyError);
	temp = *test;

	if (temp == 0x7FC00000)
	{
		return(0);
	}


	SNR = 10 * log10(EnergySignal / EnergyError);

	return (SNR);

}