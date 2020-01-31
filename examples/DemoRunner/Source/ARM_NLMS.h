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
//#include <omp.h>

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

typedef struct
{
	int numTaps;    /**< number of coefficients in the filter. */
	float *pState;   /**< points to the state variable array. The array is of length numTaps+blockSize-1. */
	float *pCoeffs;  /**< points to the coefficient array. The array is of length numTaps. */
	float mu;        /**< step size that controls filter coefficient updates. */
} arm_lms_instance_f32;

typedef struct
{
	int numTaps;     /**< number of filter coefficients in the filter. */
	float *pState;    /**< points to the state variable array. The array is of length numTaps+blockSize-1. */
	const float *pCoeffs;   /**< points to the coefficient array. The array is of length numTaps. */
} arm_fir_instance_f32;

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

 void arm_lms_norm_f32(
	arm_lms_norm_instance_f32 * S,
	const float * pSrc,
	float * pRef,
	float * pOut,
	float * pErr,
	int blockSize);

 void arm_lms_f32(
	arm_lms_instance_f32 * S,
	const float * pSrc,
	float * pRef,
	float * pOut,
	float * pErr,
	int blockSize);

 void arm_lms_norm_anc(
	arm_lms_norm_instance_f32 * S,
	const float * pSrc,
	float * pErrIn,
	float * pOut,
	float * pErr,
	int blockSize);

 void arm_lms_anc(
	arm_lms_instance_f32 * S,
	const float * pSrc,
	float * pErrIn,
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

void arm_lms_init_f32(
	arm_lms_instance_f32 * S,
	int numTaps,
	float * pCoeffs,
	float * pState,
	float mu,
	int blockSize);

void arm_fir_init_f32(
	arm_fir_instance_f32 * S,
	int numTaps,
	const float* pCoeffs,
	float * pState,
	int blockSize)
{
	/* Assign filter taps */
	S->numTaps = numTaps;

	/* Assign coefficient pointer */
	S->pCoeffs = pCoeffs;

	/* Clear state buffer. The size is always (blockSize + numTaps - 1) */
	memset(pState, 0, (numTaps + (blockSize - 1U)) * sizeof(float));

	/* Assign state pointer */
	S->pState = pState;
}
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
void arm_fir_f32(
	const arm_fir_instance_f32 * S,
	const float * pSrc,
	float * pDst,
	int blockSize)
{
	float *pState = S->pState;                 /* State pointer */
	const float *pCoeffs = S->pCoeffs;               /* Coefficient pointer */
	float *pStateCurnt;                        /* Points to the current sample of the state */
	float *px;                                 /* Temporary pointer for state buffer */
	const float *pb;                                 /* Temporary pointer for coefficient buffer */
	float acc0;                                /* Accumulator */
	int numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
	int i, tapCnt, blkCnt;                    /* Loop counters */


	float acc1, acc2, acc3, acc4, acc5, acc6, acc7;     /* Accumulators */
	float x0, x1, x2, x3, x4, x5, x6, x7;               /* Temporary variables to hold state values */
	float c0;                                           /* Temporary variable to hold coefficient value */

  /* S->pState points to state array which contains previous frame (numTaps - 1) samples */
  /* pStateCurnt points to the location where the new input data should be written */
	pStateCurnt = &(S->pState[(numTaps - 1U)]);

	/* Loop unrolling: Compute 8 output values simultaneously.
	 * The variables acc0 ... acc7 hold output values that are being computed:
	 *
	 *    acc0 =  b[numTaps-1] * x[n-numTaps-1] + b[numTaps-2] * x[n-numTaps-2] + b[numTaps-3] * x[n-numTaps-3] +...+ b[0] * x[0]
	 *    acc1 =  b[numTaps-1] * x[n-numTaps]   + b[numTaps-2] * x[n-numTaps-1] + b[numTaps-3] * x[n-numTaps-2] +...+ b[0] * x[1]
	 *    acc2 =  b[numTaps-1] * x[n-numTaps+1] + b[numTaps-2] * x[n-numTaps]   + b[numTaps-3] * x[n-numTaps-1] +...+ b[0] * x[2]
	 *    acc3 =  b[numTaps-1] * x[n-numTaps+2] + b[numTaps-2] * x[n-numTaps+1] + b[numTaps-3] * x[n-numTaps]   +...+ b[0] * x[3]
	 */

	blkCnt = blockSize >> 3U;

	while (blkCnt > 0U)
	{
		/* Copy 4 new input samples into the state buffer. */
		*pStateCurnt++ = *pSrc++;
		*pStateCurnt++ = *pSrc++;
		*pStateCurnt++ = *pSrc++;
		*pStateCurnt++ = *pSrc++;

		/* Set all accumulators to zero */
		acc0 = 0.0f;
		acc1 = 0.0f;
		acc2 = 0.0f;
		acc3 = 0.0f;
		acc4 = 0.0f;
		acc5 = 0.0f;
		acc6 = 0.0f;
		acc7 = 0.0f;

		/* Initialize state pointer */
		px = pState;

		/* Initialize coefficient pointer */
		pb = pCoeffs;

		/* This is separated from the others to avoid
		 * a call to __aeabi_memmove which would be slower
		 */
		*pStateCurnt++ = *pSrc++;
		*pStateCurnt++ = *pSrc++;
		*pStateCurnt++ = *pSrc++;
		*pStateCurnt++ = *pSrc++;

		/* Read the first 7 samples from the state buffer:  x[n-numTaps], x[n-numTaps-1], x[n-numTaps-2] */
		x0 = *px++;
		x1 = *px++;
		x2 = *px++;
		x3 = *px++;
		x4 = *px++;
		x5 = *px++;
		x6 = *px++;

		/* Loop unrolling: process 8 taps at a time. */
		tapCnt = numTaps >> 3U;

		while (tapCnt > 0U)
		{
			/* Read the b[numTaps-1] coefficient */
			c0 = *(pb++);

			/* Read x[n-numTaps-3] sample */
			x7 = *(px++);

			/* acc0 +=  b[numTaps-1] * x[n-numTaps] */
			acc0 += x0 * c0;

			/* acc1 +=  b[numTaps-1] * x[n-numTaps-1] */
			acc1 += x1 * c0;

			/* acc2 +=  b[numTaps-1] * x[n-numTaps-2] */
			acc2 += x2 * c0;

			/* acc3 +=  b[numTaps-1] * x[n-numTaps-3] */
			acc3 += x3 * c0;

			/* acc4 +=  b[numTaps-1] * x[n-numTaps-4] */
			acc4 += x4 * c0;

			/* acc1 +=  b[numTaps-1] * x[n-numTaps-5] */
			acc5 += x5 * c0;

			/* acc2 +=  b[numTaps-1] * x[n-numTaps-6] */
			acc6 += x6 * c0;

			/* acc3 +=  b[numTaps-1] * x[n-numTaps-7] */
			acc7 += x7 * c0;

			/* Read the b[numTaps-2] coefficient */
			c0 = *(pb++);

			/* Read x[n-numTaps-4] sample */
			x0 = *(px++);

			/* Perform the multiply-accumulate */
			acc0 += x1 * c0;
			acc1 += x2 * c0;
			acc2 += x3 * c0;
			acc3 += x4 * c0;
			acc4 += x5 * c0;
			acc5 += x6 * c0;
			acc6 += x7 * c0;
			acc7 += x0 * c0;

			/* Read the b[numTaps-3] coefficient */
			c0 = *(pb++);

			/* Read x[n-numTaps-5] sample */
			x1 = *(px++);

			/* Perform the multiply-accumulates */
			acc0 += x2 * c0;
			acc1 += x3 * c0;
			acc2 += x4 * c0;
			acc3 += x5 * c0;
			acc4 += x6 * c0;
			acc5 += x7 * c0;
			acc6 += x0 * c0;
			acc7 += x1 * c0;

			/* Read the b[numTaps-4] coefficient */
			c0 = *(pb++);

			/* Read x[n-numTaps-6] sample */
			x2 = *(px++);

			/* Perform the multiply-accumulates */
			acc0 += x3 * c0;
			acc1 += x4 * c0;
			acc2 += x5 * c0;
			acc3 += x6 * c0;
			acc4 += x7 * c0;
			acc5 += x0 * c0;
			acc6 += x1 * c0;
			acc7 += x2 * c0;

			/* Read the b[numTaps-4] coefficient */
			c0 = *(pb++);

			/* Read x[n-numTaps-6] sample */
			x3 = *(px++);
			/* Perform the multiply-accumulates */
			acc0 += x4 * c0;
			acc1 += x5 * c0;
			acc2 += x6 * c0;
			acc3 += x7 * c0;
			acc4 += x0 * c0;
			acc5 += x1 * c0;
			acc6 += x2 * c0;
			acc7 += x3 * c0;

			/* Read the b[numTaps-4] coefficient */
			c0 = *(pb++);

			/* Read x[n-numTaps-6] sample */
			x4 = *(px++);

			/* Perform the multiply-accumulates */
			acc0 += x5 * c0;
			acc1 += x6 * c0;
			acc2 += x7 * c0;
			acc3 += x0 * c0;
			acc4 += x1 * c0;
			acc5 += x2 * c0;
			acc6 += x3 * c0;
			acc7 += x4 * c0;

			/* Read the b[numTaps-4] coefficient */
			c0 = *(pb++);

			/* Read x[n-numTaps-6] sample */
			x5 = *(px++);

			/* Perform the multiply-accumulates */
			acc0 += x6 * c0;
			acc1 += x7 * c0;
			acc2 += x0 * c0;
			acc3 += x1 * c0;
			acc4 += x2 * c0;
			acc5 += x3 * c0;
			acc6 += x4 * c0;
			acc7 += x5 * c0;

			/* Read the b[numTaps-4] coefficient */
			c0 = *(pb++);

			/* Read x[n-numTaps-6] sample */
			x6 = *(px++);

			/* Perform the multiply-accumulates */
			acc0 += x7 * c0;
			acc1 += x0 * c0;
			acc2 += x1 * c0;
			acc3 += x2 * c0;
			acc4 += x3 * c0;
			acc5 += x4 * c0;
			acc6 += x5 * c0;
			acc7 += x6 * c0;

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Loop unrolling: Compute remaining outputs */
		tapCnt = numTaps % 0x8U;

		while (tapCnt > 0U)
		{
			/* Read coefficients */
			c0 = *(pb++);

			/* Fetch 1 state variable */
			x7 = *(px++);

			/* Perform the multiply-accumulates */
			acc0 += x0 * c0;
			acc1 += x1 * c0;
			acc2 += x2 * c0;
			acc3 += x3 * c0;
			acc4 += x4 * c0;
			acc5 += x5 * c0;
			acc6 += x6 * c0;
			acc7 += x7 * c0;

			/* Reuse the present sample states for next sample */
			x0 = x1;
			x1 = x2;
			x2 = x3;
			x3 = x4;
			x4 = x5;
			x5 = x6;
			x6 = x7;

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Advance the state pointer by 8 to process the next group of 8 samples */
		pState = pState + 8;

		/* The results in the 8 accumulators, store in the destination buffer. */
		*pDst++ = acc0;
		*pDst++ = acc1;
		*pDst++ = acc2;
		*pDst++ = acc3;
		*pDst++ = acc4;
		*pDst++ = acc5;
		*pDst++ = acc6;
		*pDst++ = acc7;


		/* Decrement loop counter */
		blkCnt--;
	}

	/* Loop unrolling: Compute remaining output samples */
	blkCnt = blockSize % 0x8U;

	while (blkCnt > 0U)
	{
		/* Copy one sample at a time into state buffer */
		*pStateCurnt++ = *pSrc++;

		/* Set the accumulator to zero */
		acc0 = 0.0f;

		/* Initialize state pointer */
		px = pState;

		/* Initialize Coefficient pointer */
		pb = pCoeffs;

		i = numTaps;

		/* Perform the multiply-accumulates */
		while (i > 0U)
		{
			/* acc =  b[numTaps-1] * x[n-numTaps-1] + b[numTaps-2] * x[n-numTaps-2] + b[numTaps-3] * x[n-numTaps-3] +...+ b[0] * x[0] */
			acc0 += *px++ * *pb++;

			i--;
		}

		/* Store result in destination buffer. */
		*pDst++ = acc0;

		/* Advance state pointer by 1 for the next sample */
		pState = pState + 1U;

		/* Decrement loop counter */
		blkCnt--;
	}

	/* Processing is complete.
	   Now copy the last numTaps - 1 samples to the start of the state buffer.
	   This prepares the state buffer for the next function call. */

	   /* Points to the start of the state buffer */
	pStateCurnt = S->pState;

	/* Loop unrolling: Compute 4 taps at a time */
	tapCnt = (numTaps - 1U) >> 2U;

	/* Copy data */
	while (tapCnt > 0U)
	{
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;

		/* Decrement loop counter */
		tapCnt--;
	}

	/* Calculate remaining number of copies */
	tapCnt = (numTaps - 1U) % 0x4U;


	/* Copy remaining data */
	while (tapCnt > 0U)
	{
		*pStateCurnt++ = *pState++;

		/* Decrement loop counter */
		tapCnt--;
	}
}

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

void arm_lms_init_f32(
	arm_lms_instance_f32 * S,
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

	/* Clear state buffer and size is always blockSize + numTaps */
	memset(pState, 0, (numTaps + (blockSize - 1)) * sizeof(float));

	/* Assign state pointer */
	S->pState = pState;

	/* Assign Step size value */
	S->mu = mu;
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

 void arm_lms_norm_f32(
	arm_lms_norm_instance_f32 * S,
	const float * pSrc,
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
		*pOut++ = sum;

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
 void arm_lms_f32(
	arm_lms_instance_f32 * S,
	const float * pSrc,
	float * pRef,
	float * pOut,
	float * pErr,
	int blockSize)
{
	volatile float *pState = S->pState;                 /* State pointer */
	volatile float *pCoeffs = S->pCoeffs;               /* Coefficient pointer */
	volatile float *pStateCurnt;                        /* Points to the current sample of the state */
	volatile float *px, *pb;                            /* Temporary pointers for state and coefficient buffers */
	volatile float mu = S->mu;                          /* Adaptive factor */
	volatile float acc, e;                              /* Accumulator, error */
	volatile float w;                                   /* Weight factor */
	volatile int numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
	volatile int tapCnt, blkCnt;                       /* Loop counters */

/* Initializations of error,  difference, Coefficient update */
	e = 0.0f;
	w = 0.0f;

	/* S->pState points to state array which contains previous frame (numTaps - 1) samples */
	/* pStateCurnt points to the location where the new input data should be written */
	pStateCurnt = &(S->pState[(numTaps - 1U)]);

	/* initialise loop count */
	blkCnt = blockSize;

	while (blkCnt > 0U)
	{
		/* Copy the new input sample into the state buffer */
		*pStateCurnt++ = *pSrc++;

		/* Initialize pState pointer */
		px = pState;

		/* Initialize coefficient pointer */
		pb = pCoeffs;

		/* Set the accumulator to zero */
		acc = 0.0f;

		/* Loop unrolling: Compute 4 taps at a time. */
		tapCnt = numTaps >> 2U;

		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			acc += (*px++) * (*pb++);

			acc += (*px++) * (*pb++);

			acc += (*px++) * (*pb++);

			acc += (*px++) * (*pb++);

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Loop unrolling: Compute remaining taps */
		tapCnt = numTaps % 0x4U;


		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			acc += (*px++) * (*pb++);

			/* Decrement the loop counter */
			tapCnt--;
		}

		/* Store the result from accumulator into the destination buffer. */
		*pOut++ = -acc;

		/* Compute and store error */
		e = (float)*pRef++ - acc;
		*pErr++ = e;

		/* Calculation of Weighting factor for updating filter coefficients */
		w = e * mu;

		/* Initialize pState pointer */
		/* Advance state pointer by 1 for the next sample */
		px = pState++;

		/* Initialize coefficient pointer */
		pb = pCoeffs;

		/* Loop unrolling: Compute 4 taps at a time. */
		tapCnt = numTaps >> 2U;

		/* Update filter coefficients */
		while (tapCnt > 0U)
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

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Loop unrolling: Compute remaining taps */
		tapCnt = numTaps % 0x4U;


		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			*pb += w * (*px++);
			pb++;

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Decrement loop counter */
		blkCnt--;
	}

	/* Processing is complete.
	   Now copy the last numTaps - 1 samples to the start of the state buffer.
	   This prepares the state buffer for the next function call. */

	   /* Points to the start of the pState buffer */
	pStateCurnt = S->pState;

	/* copy data */

  /* Loop unrolling: Compute 4 taps at a time. */
	tapCnt = (numTaps - 1U) >> 2U;

	while (tapCnt > 0U)
	{
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;

		/* Decrement loop counter */
		tapCnt--;
	}

	/* Loop unrolling: Compute remaining taps */
	tapCnt = (numTaps - 1U) % 0x4U;

	while (tapCnt > 0U)
	{
		*pStateCurnt++ = *pState++;

		/* Decrement loop counter */
		tapCnt--;
	}

}
/**
   * @} end of LMS_NORM group
   */

 void arm_lms_norm_anc(
	arm_lms_norm_instance_f32 * S,
	const float * pSrc,
	float * pErrIn,
	float * pOut,
	float * pErr,
	int blockSize)
{
	float *pState = S->pState;                 /* State pointer */
	float *pCoeffs = S->pCoeffs;               /* Coefficient pointer */
	float *pStateCurnt;                        /* Points to the current sample of the state */
	float *px, *pb;                            /* Temporary pointers for state and coefficient buffers */
	float mu = S->mu;                          /* Adaptive factor */
	float acc, e;                              /* Accumulator, error */
	float w;                                   /* Weight factor */
	int numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
	int tapCnt, blkCnt;                       /* Loop counters */
	float energy;                              /* Energy of the input */
	float x0, in;                              /* Temporary variable to hold input sample and state */

/* Initializations of error,  difference, Coefficient update */
	e = 0.0f;
	w = 0.0f;

	energy = S->energy;
	x0 = S->x0;

	/* S->pState points to buffer which contains previous frame (numTaps - 1) samples */
	/* pStateCurnt points to the location where the new input data should be written */
	pStateCurnt = &(S->pState[(numTaps - 1U)]);

	/* initialise loop count */
	blkCnt = blockSize;

	while (blkCnt > 0U)
	{
		/* Copy the new input sample into the state buffer */
		*pStateCurnt++ = *pSrc;

		/* Initialize pState pointer */
		px = pState;

		/* Initialize coefficient pointer */
		pb = pCoeffs;

		/* Read the sample from input buffer */
		in = *pSrc++;

		/* Update the energy calculation */
		energy -= x0 * x0;
		energy += in * in;

		/* Set the accumulator to zero */
		acc = 0.0f;

		/* Loop unrolling: Compute 4 taps at a time. */
		tapCnt = numTaps >> 2U;

		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			acc += (*px++) * (*pb++);

			acc += (*px++) * (*pb++);

			acc += (*px++) * (*pb++);

			acc += (*px++) * (*pb++);

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Loop unrolling: Compute remaining taps */
		tapCnt = numTaps % 0x4U;

		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			acc += (*px++) * (*pb++);

			/* Decrement the loop counter */
			tapCnt--;
		}

		/* Store the result from accumulator into the destination buffer. */
		*pOut++ = -acc;

		/* Compute and store error */
		e = -(float)*pErrIn++;
		*pErr++ = e;

		/* Calculation of Weighting factor for updating filter coefficients */
		/* epsilon value 0.000000119209289f */
		w = (e * mu) / (energy + 0.000000119209289f);

		/* Initialize pState pointer */
		px = pState;

		/* Initialize coefficient pointer */
		pb = pCoeffs;

		/* Loop unrolling: Compute 4 taps at a time. */
		tapCnt = numTaps >> 2U;

		/* Update filter coefficients */
		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Loop unrolling: Compute remaining taps */
		tapCnt = numTaps % 0x4U;

		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			/* Decrement loop counter */
			tapCnt--;
		}

		x0 = *pState;

		/* Advance state pointer by 1 for the next sample */
		pState = pState + 1;

		/* Decrement loop counter */
		blkCnt--;
	}

	/* Save energy and x0 values for the next frame */
	S->energy = energy;
	S->x0 = x0;

	/* Processing is complete.
	   Now copy the last numTaps - 1 samples to the start of the state buffer.
	   This prepares the state buffer for the next function call. */

	   /* Points to the start of the pState buffer */
	pStateCurnt = S->pState;

	/* copy data */
  /* Loop unrolling: Compute 4 taps at a time. */
	tapCnt = (numTaps - 1U) >> 2U;

	while (tapCnt > 0U)
	{
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;

		/* Decrement loop counter */
		tapCnt--;
	}

	/* Loop unrolling: Compute remaining taps */
	tapCnt = (numTaps - 1U) % 0x4U;

	while (tapCnt > 0U)
	{
		*pStateCurnt++ = *pState++;

		/* Decrement loop counter */
		tapCnt--;
	}

}

 void arm_lms_anc(
	arm_lms_instance_f32 * S,
	const float * pSrc,
	float * pErrIn,
	float * pOut,
	float * pErr,
	int blockSize)
{
	volatile float *pState = S->pState;                 /* State pointer */
	volatile float *pCoeffs = S->pCoeffs;               /* Coefficient pointer */
	volatile float *pStateCurnt;                        /* Points to the current sample of the state */
	volatile float *px, *pb;                            /* Temporary pointers for state and coefficient buffers */
	volatile float mu = S->mu;                          /* Adaptive factor */
	volatile float acc, e;                              /* Accumulator, error */
	volatile float w;                                   /* Weight factor */
	volatile int numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
	volatile int tapCnt, blkCnt;                       /* Loop counters */

/* Initializations of error,  difference, Coefficient update */
	e = 0.0f;
	w = 0.0f;

	/* S->pState points to state array which contains previous frame (numTaps - 1) samples */
	/* pStateCurnt points to the location where the new input data should be written */
	pStateCurnt = &(S->pState[(numTaps - 1U)]);

	/* initialise loop count */
	blkCnt = blockSize;

	while (blkCnt > 0U)
	{
		/* Copy the new input sample into the state buffer */
		*pStateCurnt++ = *pSrc++;

		/* Initialize pState pointer */
		px = pState;

		/* Initialize coefficient pointer */
		pb = pCoeffs;

		/* Set the accumulator to zero */
		acc = 0.0f;

		
		/* Loop unrolling: Compute 4 taps at a time. */
		tapCnt = numTaps >> 2U;

		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			acc += (*px++) * (*pb++);

			acc += (*px++) * (*pb++);

			acc += (*px++) * (*pb++);

			acc += (*px++) * (*pb++);

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Loop unrolling: Compute remaining taps */
		tapCnt = numTaps % 0x4U;


		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			acc += (*px++) * (*pb++);

			/* Decrement the loop counter */
			tapCnt--;
		}
		
		/* Store the result from accumulator into the destination buffer. */
		*pOut++ = acc;

		/* Compute and store error */
		e = (float)*pErrIn++;
		*pErr++ = e;

		/* Calculation of Weighting factor for updating filter coefficients */
		w = e * mu;

		/* Initialize pState pointer */
		/* Advance state pointer by 1 for the next sample */
		px = pState++;

		/* Initialize coefficient pointer */
		pb = pCoeffs;

		/* Loop unrolling: Compute 4 taps at a time. */
		tapCnt = numTaps >> 2U;

		/* Update filter coefficients */
		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Loop unrolling: Compute remaining taps */
		tapCnt = numTaps % 0x4U;


		while (tapCnt > 0U)
		{
			/* Perform the multiply-accumulate */
			*pb = (*pb) * 0.999f + w * (*px++);
			pb++;

			/* Decrement loop counter */
			tapCnt--;
		}

		/* Decrement loop counter */
		blkCnt--;
	}

	/* Processing is complete.
	   Now copy the last numTaps - 1 samples to the start of the state buffer.
	   This prepares the state buffer for the next function call. */

	   /* Points to the start of the pState buffer */
	pStateCurnt = S->pState;

	/* copy data */

  /* Loop unrolling: Compute 4 taps at a time. */
	tapCnt = (numTaps - 1U) >> 2U;

	while (tapCnt > 0U)
	{
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;
		*pStateCurnt++ = *pState++;

		/* Decrement loop counter */
		tapCnt--;
	}

	/* Loop unrolling: Compute remaining taps */
	tapCnt = (numTaps - 1U) % 0x4U;

	while (tapCnt > 0U)
	{
		*pStateCurnt++ = *pState++;

		/* Decrement loop counter */
		tapCnt--;
	}

}

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


void arm_dot_prod_f32(
	const float * pSrcA,
	const float * pSrcB,
	int blockSize,
	float * result)
{
	int blkCnt;                               /* Loop counter */
	float sum = 0.0f;                          /* Temporary return variable */

  /* Loop unrolling: Compute 4 outputs at a time */
	blkCnt = blockSize >> 2U;

	/* First part of the processing with loop unrolling. Compute 4 outputs at a time.
	 ** a second loop below computes the remaining 1 to 3 samples. */
	while (blkCnt > 0U)
	{
		/* C = A[0]* B[0] + A[1]* B[1] + A[2]* B[2] + .....+ A[blockSize-1]* B[blockSize-1] */

		/* Calculate dot product and store result in a temporary buffer. */
		sum += (*pSrcA++) * (*pSrcB++);

		sum += (*pSrcA++) * (*pSrcB++);

		sum += (*pSrcA++) * (*pSrcB++);

		sum += (*pSrcA++) * (*pSrcB++);

		/* Decrement loop counter */
		blkCnt--;
	}

	/* Loop unrolling: Compute remaining outputs */
	blkCnt = blockSize % 0x4U;


	while (blkCnt > 0U)
	{
		/* C = A[0]* B[0] + A[1]* B[1] + A[2]* B[2] + .....+ A[blockSize-1]* B[blockSize-1] */

		/* Calculate dot product and store result in a temporary buffer. */
		sum += (*pSrcA++) * (*pSrcB++);

		/* Decrement loop counter */
		blkCnt--;
	}

	/* Store result in destination buffer */
	*result = sum;
}

void arm_add_f32(
	const float * pSrcA,
	const float * pSrcB,
	float * pDst,
	int blockSize)
{
	int blkCnt;                               /* Loop counter */

  /* Loop unrolling: Compute 4 outputs at a time */
	blkCnt = blockSize >> 2U;

	while (blkCnt > 0U)
	{
		/* C = A + B */

		/* Add and store result in destination buffer. */
		*pDst++ = (*pSrcA++) + (*pSrcB++);
		*pDst++ = (*pSrcA++) + (*pSrcB++);
		*pDst++ = (*pSrcA++) + (*pSrcB++);
		*pDst++ = (*pSrcA++) + (*pSrcB++);

		/* Decrement loop counter */
		blkCnt--;
	}

	/* Loop unrolling: Compute remaining outputs */
	blkCnt = blockSize % 0x4U;

	while (blkCnt > 0U)
	{
		/* C = A + B */

		/* Add and store result in destination buffer. */
		*pDst++ = (*pSrcA++) + (*pSrcB++);

		/* Decrement loop counter */
		blkCnt--;
	}
}

void arm_scale_f32(
	const float *pSrc,
	float scale,
	float *pDst,
	int blockSize)
{
	int blkCnt;                               /* Loop counter */

  /* Loop unrolling: Compute 4 outputs at a time */
	blkCnt = blockSize >> 2U;

	while (blkCnt > 0U)
	{
		/* C = A * scale */

		/* Scale input and store result in destination buffer. */
		*pDst++ = (*pSrc++) * scale;

		*pDst++ = (*pSrc++) * scale;

		*pDst++ = (*pSrc++) * scale;

		*pDst++ = (*pSrc++) * scale;

		/* Decrement loop counter */
		blkCnt--;
	}

	/* Loop unrolling: Compute remaining outputs */
	blkCnt = blockSize % 0x4U;

	while (blkCnt > 0U)
	{
		/* C = A * scale */

		/* Scale input and store result in destination buffer. */
		*pDst++ = (*pSrc++) * scale;

		/* Decrement loop counter */
		blkCnt--;
	}

}






#include <array>
#include <limits>
#include <vector>
#include <fstream>
#include <assert.h>
#include "constants.h"


template<int filter_length>
class FIRFilter {
public:
	typedef std::array<float, filter_length> samples_array;
	typedef std::array<float, filter_length> filter_coeffs_array;

	FIRFilter() : _filter_coefficients{ {0.0f} }, _samples_buffer{ {0.0} }, filterSize{filter_length} {}

	FIRFilter(int _filterSize) : _filter_coefficients{ {0.0f} }, _samples_buffer{ {0.0} }, filterSize{ _filterSize } {}

	FIRFilter(int _filterSize, filter_coeffs_array coefficients) : _filter_coefficients(coefficients), filterSize{ _filterSize },
		_samples_buffer{ 0.0 } {}

	FIRFilter(filter_coeffs_array coefficients) : _filter_coefficients(coefficients), _samples_buffer{ 0.0 } {}

	float fir_step(float new_sample) {
		float new_val = 0;
		// Shift sample_buffer (FIFO style)
		for (long unsigned int i = filterSize - 1; i >= 1; --i) {
			_samples_buffer[i] = _samples_buffer[i - 1];
		}
		_samples_buffer[0] = new_sample;
		// Multiply and accumulate
		for (long unsigned int i = 0; i < filterSize; ++i) {
			new_val += _samples_buffer[i] * _filter_coefficients[i];
		}
		return new_val;
	}

	filter_coeffs_array get_coefficients() {
		return _filter_coefficients;
	}

	void set_coefficients(filter_coeffs_array new_coefficients) {
		_filter_coefficients = new_coefficients;
	}

	void reset_sample_buffer() {
		_samples_buffer = { 0 };
	}

private:
	int filterSize;
	filter_coeffs_array _filter_coefficients;
	samples_array _samples_buffer;
};




void dc_removal(float *samples_bufferE, float *samples_bufferR, float *out1, float *out2, long unsigned int buffer_length) {
	static float last_error_sample = 0.0f;
	static float last_ref_sample = 0.0f;

	static FIRFilter<ANTYALIAS_FILTER_LENGTH> aa_filter_error(ANTYALIAS_FILTER_COEFFS);
	static FIRFilter<ANTYALIAS_FILTER_LENGTH> aa_filter_ref(ANTYALIAS_FILTER_COEFFS);

	for (unsigned long i = 1; i < buffer_length; i ++) {
		float error_sample = samples_bufferE[i];
		float reference_sample = samples_bufferR[i - 1];
		// error samples filtering
		float new_err = error_sample + DC_REMOVAL_ALPHA * last_error_sample;
		float out_err = new_err - last_error_sample;
		last_error_sample = new_err;
		out_err = aa_filter_error.fir_step(out_err);
		out1[i] = out_err;
		// reference samples filtering
		float new_ref = reference_sample + DC_REMOVAL_ALPHA * last_ref_sample;
		float out_ref = new_ref - last_ref_sample;
		last_ref_sample = out_ref;
		out_ref = aa_filter_ref.fir_step(out_ref);
		out2[i - 1] = out_ref;
	}
}

template<int filter_length>
class NLMSFilter {
public:

	typedef std::array<float, filter_length> samples_array;
	typedef std::array<float, filter_length> filter_coeffs_array;
	FIRFilter<filter_length> fir_filter;

	NLMSFilter() : mu{ 0.5 }, _nlms_coefficients{ {0} }, _samples_buffer{ {0} }, filterSize{ filter_length } 
	{
		fir_filter = FIRFilter<filter_length>(filterSize);
		fir_filter.set_coefficients(_nlms_coefficients);
	}

	NLMSFilter(float mu, int filterSize) : mu{ mu }, _nlms_coefficients{ {0} }, _samples_buffer{ {0} }, filterSize(filterSize)
	{
		fir_filter = FIRFilter<filter_length>(filterSize);
		fir_filter.set_coefficients(_nlms_coefficients);
	}

	NLMSFilter(float mu, filter_coeffs_array initial_filter) : mu{ mu }, filterSize(filter_length), _nlms_coefficients{0.0}, _samples_buffer{ {0.0} } 
	{
		fir_filter = FIRFilter<filter_length>(filterSize);
		fir_filter.set_coefficients(initial_filter);
	}

	void nlms_step(float *x_reference_sample, float *error_sample, float *outputBuffer, int blockSize) {
		for (int j = 0; j < blockSize; j++)
		{
			// Shift samples buffer
			for (long int i = filterSize - 1; i >= 1; --i) {
				_samples_buffer[i] = _samples_buffer[i - 1];
			}
			_samples_buffer[0] = x_reference_sample[j];
			// Update filter coefficients			
			nlms_filter_update((mu) *(error_sample[j]));
			//x0 = _samples_buffer.at(filterSize - 1);
			// Perform filtering step, to generate new y correction sample
			*(outputBuffer++) = fir_filter.fir_step(x_reference_sample[j]);
		}
	}

	void nlms_filter_update(float update_step) {
		filter_coeffs_array filter_coeffs = fir_filter.get_coefficients();
		for (int i = 0; i < filterSize; ++i) {
			filter_coeffs.at(i) += _samples_buffer.at(i) * update_step;
		}
		fir_filter.set_coefficients(filter_coeffs);
	}

private:
	float mu;         /**< step size that control filter coefficient updates. */
	int filterSize;
	filter_coeffs_array _nlms_coefficients;
	samples_array _samples_buffer;
};



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
//#include <omp.h>

namespace Adaptive {
	class NLMS
	{
		int NumOfTaps;	//filter size
		float mu;	//step size
		float a;	//R factor

		int bufferSize = 0;

		// Signal vectors
		float y[FRAMES_PER_BUFFER] = { 0 };			// Output data
		float w1[NUM_OF_TAPS] = { 0 };		// filter coeff.
		float xx[NUM_OF_TAPS] = { 0 };		// vector to apply filter to
		float err[FRAMES_PER_BUFFER] = { 0 };
		float k = { 0 };

		float errOutput[FRAMES_PER_BUFFER] = { 0 };
		float Out[FRAMES_PER_BUFFER] = { 0 };
		float AntyAliasOut[FRAMES_PER_BUFFER] = { 0 };


	public:
		NLMS(void);
		NLMS(int size);
		NLMS(int x, float y, float z);
		~NLMS();

		inline void processNLMS(float *x, float *e, float *out, int blockSize);
		inline const float* getCoeff();

	private:
		inline void pushBack(float *a, float x);
	};

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Default constructor of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/

	NLMS::NLMS() : NumOfTaps(100), mu(0.5f), a(0.0001f)
	{
		memset(y, 0.0f, FRAMES_PER_BUFFER);
		memset(xx, 0.0f, NumOfTaps);
		memset(w1, 0.0f, NumOfTaps);
		for (int i = 0; i < NumOfTaps; i++)
		{
			w1[i] = -1.0f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0f)));
		}
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
		memset(y, 0, FRAMES_PER_BUFFER);
		memset(xx, 0, NumOfTaps);
		memset(w1, 0, NumOfTaps);
		for (int i = 0; i < NumOfTaps; i++)
		{
			w1[i] = -1.0f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0f)));
		}
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

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief processNLMS method of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param float d - destiny sound input data
		@param float x - noise sound input data
	*/
	void NLMS::processNLMS(float *x, float *d, float *out, int blockSize)
	{
		int i = 0;
		int j = 0;
		int l = 0;
		float temp;
#pragma omp parallel sections
		{
#pragma omp section
			{
				memset(&out[0], 0, blockSize * sizeof(float));
				memset(&y[0], 0, blockSize * sizeof(float));
				memset(&err[0], 0, blockSize * sizeof(float));
			}
#pragma omp section
			{
				for (i = 0; i < blockSize; i++) {
					pushBack(&xx[0], x[i]);
					//xx[NumOfTaps - 1] = x[i];
					xx[0] = x[i];

					// filter input data
#pragma omp parallel for schedule(static, 4) reduction(+:y) reduction(+:temp)
					for (j = 0, temp = 0; j < NumOfTaps; j+=4) {
						y[i] += w1[j] * xx[j];
						temp += xx[j] * xx[j];

						y[i] += w1[j + 1] * xx[j + 1];
						temp += xx[j + 1] * xx[j + 1];

						y[i] += w1[j + 1] * xx[j + 1];
						temp += xx[j + 1] * xx[j + 1];

						y[i] += w1[j + 1] * xx[j + 1];
						temp += xx[j + 1] * xx[j + 1];
					}

					out[i] = -y[i];

					err[i] = d[i] - y[i];

					k = mu / (a + temp);

					// update filter coeff.
#pragma omp parallel for schedule(static, 4) reduction(+:w1)
					for (l = 0; l < NumOfTaps; l+=4) {
						w1[l] += (k * err[i] * xx[l]);
						w1[l + 1] += (k * err[i] * xx[l + 1]);
						w1[l + 2] += (k * err[i] * xx[l + 2]);
						w1[l + 3] += (k * err[i] * xx[l + 3]);
					}
					//for (l = 0; l < NumOfTaps; l += 4) {
					//	w1[l] += (k * (-d[i]) * xx[l]);
					//	w1[l + 1] += (k * (-d[i]) * xx[l + 1]);
					//	w1[l + 2] += (k * (-d[i]) * xx[l + 2]);
					//	w1[l + 3] += (k * (-d[i]) * xx[l + 3]);
					//}
				}
			}
		}
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

		for (int i = NumOfTaps - 1; i > 0; --i) {
			a[i] = a[i - 1];
		}
	}

	const float* NLMS::getCoeff()
	{
		return &w1[0];
	}
}




namespace Adaptive {
	class FbLMS
	{
		int NumOfTaps;	//filter size		
		
		//float mu = 0.0001;  // Step Size
		float w[NUM_OF_TAPS] = { 0 }; // Secondary Path	

		float out;  // Filter Coefficient
		float c[NUM_OF_TAPS];
		float muedf[NUM_OF_TAPS];
		float yQueue[NUM_OF_TAPS];
		float dQueue[NUM_OF_TAPS];
		float dfQueue[NUM_OF_TAPS];

		float y;  // Speaker Output
		float yw; // Speaker Output Signal After Secondary Acoustic Path
		float xw;
		float d;  // Estimated Reference Signal
		float e;
		float mue;  // mu*e
		float df; // Filtered Reference Signal


		// FOR PREDICTING SEC-PATH
		float r;
		//float d;  // Error Mic Input
		float mu; // Step Size
		float muTrain = 0.0000005f; // Step Size
		float x[NUM_OF_TAPS];  // Generated Noise Queue
		//float w[NUM_OF_TAPS];  // Secondary Path Coefficient
		float muEX[NUM_OF_TAPS]; // mu*e*x Queue
		//float y;  // Estimated Error Mic Input
		//float e;  // e = d - y
		//float mue;  // mu*e


	public:
		FbLMS(void);
		FbLMS(int size);
		FbLMS(int x, float y);
		~FbLMS();

		void processFbLMS(float *error, float *output, int blockSize);
		void predictSecPath(float *error, float *output, int blockSize);
		void prepareForANC();
		inline const float* getCoeff();
	};

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Default constructor of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/

	FbLMS::FbLMS() : NumOfTaps(100), mu(0.5)
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
	FbLMS::FbLMS(int filterSize, float stepSize) : NumOfTaps(filterSize), mu(stepSize)
	{
		memset(c, 0.00001f, sizeof(float)*NUM_OF_TAPS);
		memset(muedf, 0.00001f, sizeof(float)*NUM_OF_TAPS);
		memset(yQueue, 0.00001f, sizeof(float)*NUM_OF_TAPS);
		memset(dQueue, 0.00001f, sizeof(float)*NUM_OF_TAPS);
		memset(dfQueue, 0.00001f, sizeof(float)*NUM_OF_TAPS);
		memset(x, 0.00001f, sizeof(float)*NUM_OF_TAPS);
		memset(muEX, 0.00001f, sizeof(float)*NUM_OF_TAPS);
	}

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Destructor of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/
	FbLMS::~FbLMS()
	{
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief processNLMS method of NLMS class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param float d - destiny sound input data
		@param float x - noise sound input data
	*/
	void FbLMS::processFbLMS(float *error, float *output, int blockSize)
	{
		for (int blockCount = 0; blockCount < blockSize; blockCount++)
		{
			arm_dot_prod_f32(w, yQueue, NumOfTaps, &yw);			

			d = e + yw;

			int i = NumOfTaps;
			while (i > 1)
			{
				dQueue[i - 1] = dQueue[i - 2];
				i--;
			}
			dQueue[0] = d;

			arm_dot_prod_f32(dQueue, c, NumOfTaps, &y);

			out = -y;
			//analogWrite(speakerPin, out);
			output[blockCount] = out;

			i = NumOfTaps;
			while (i > 1)
			{
				yQueue[i - 1] = yQueue[i - 2];
				i--;
			}
			yQueue[0] = y;

			arm_dot_prod_f32(yQueue, w, NumOfTaps, &yw);

			//adc->adc0->analogRead(micPin);

			e = error[blockCount] - yw;

			arm_dot_prod_f32(dQueue, w, NumOfTaps, &df);

			i = NumOfTaps;
			while (i > 1)
			{
				dfQueue[i - 1] = dfQueue[i - 2];
				i--;
			}

			dfQueue[0] = df;

			mue = mu * e;

			arm_scale_f32(dfQueue, mue, muedf, NumOfTaps);

			arm_add_f32(c, muedf, c, NumOfTaps);
		}
	}

	void FbLMS::predictSecPath(float * error, float * output, int blockSize)
	{
		for (int blockCount = 0; blockCount < blockSize; blockCount++)
		{
			// Shift elements in Queue to the right and add latest element at the 0 position
			int k = NumOfTaps;
			while (k > 1)
			{
				x[k - 1] = x[k - 2];
				k--;
			}
			r = -.6f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (1.2f)));
			x[0] = r;
			// Write signal to the speaker
			//analogWrite(speakerPin, r);
			output[blockCount] = r;

			// Dot product
			arm_dot_prod_f32(x, w, NumOfTaps, &y);

			//adc->adc0->analogRead(micPin);

			d = error[blockCount];

			e = d - y;

			mue = muTrain * e;

			arm_scale_f32(x, mue, muEX, NumOfTaps);

			arm_add_f32(w, muEX, w, NumOfTaps);
		}
	}
	
	void FbLMS::prepareForANC()
	{
		float out = 0;

		float y = 0;  // Speaker Output
		float yw = 0; // Speaker Output Signal After Secondary Acoustic Path
		float d = 0;  // Estimated Reference Signal
		float e = 0;
		float mue = 0;  // mu*e
		float df = 0;
	}
	//--------------------------------------------------------------------------------------------------------------------

	const float* FbLMS::getCoeff()
	{
		return &c[0];
	}
}
