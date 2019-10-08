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
#include "arm_math.h"
#include "arm_const_structs.h"

   /**
	* @brief  Caluclation of SNR
	* @param  float32_t* 	Pointer to the reference buffer
	* @param  float32_t*	Pointer to the test buffer
	* @param  uint32_t	total number of samples
	* @return float32_t	SNR
	* The function Caluclates signal to noise ratio for the reference output
	* and test output
	*/

float32_t arm_snr_f32(float32_t *pRef, float32_t *pTest, uint32_t buffSize)
{
	float32_t EnergySignal = 0.0, EnergyError = 0.0;
	uint32_t i;
	float32_t SNR;
	uint32_t temp;
	uint32_t *test;

	for (i = 0; i < buffSize; i++)
	{
		/* Checking for a NAN value in pRef array */
		test = (uint32_t *)(&pRef[i]);
		temp = *test;

		if (temp == 0x7FC00000)
		{
			return(0);
		}

		/* Checking for a NAN value in pTest array */
		test = (uint32_t *)(&pTest[i]);
		temp = *test;

		if (temp == 0x7FC00000)
		{
			return(0);
		}
		EnergySignal += pRef[i] * pRef[i];
		EnergyError += (pRef[i] - pTest[i]) * (pRef[i] - pTest[i]);
	}
	/* Checking for a NAN value in EnergyError */
	test = (uint32_t *)(&EnergyError);
	temp = *test;

	if (temp == 0x7FC00000)
	{
		return(0);
	}
	SNR = 10 * log10(EnergySignal / EnergyError);
	return (SNR);
}


#define M 32

typedef struct {
	float32_t alpha;
	float32_t gamma;
	float32_t d[M];
	float32_t D[2 * M];
	float32_t e[M];
	float32_t E[2 * M];
	float32_t fai[2 * M];
	float32_t P[2 * M];
	float32_t U[2 * M];
	float32_t uu[2 * M];
	float32_t W[2 * M];		
	float32_t Y[2 * M];
	float32_t y[M];		
	float32_t temp[2 * M];
	arm_rfft_fast_instance_f32 arm_fft;
}freq_dom_nlms_instance;

void fd_nlms_init(freq_dom_nlms_instance* instance)
{
	arm_rfft_32_fast_init_f32(&instance->arm_fft);
}

void freq_dom_nlms_process(freq_dom_nlms_instance* instance, float32_t* x, float32_t* d, float32_t* out, float32_t* err, uint32_t blockSize) {

	uint32_t _m = M;
	arm_rfft_fast_instance_f32 *_arm_fft = &(instance->arm_fft);
	float32_t _alpha		= instance->alpha;
	float32_t _gamma		= instance->gamma;
	float32_t *_d			= instance->d;	
	float32_t *_D			= instance->D;	
	float32_t *_e			= instance->e;	
	float32_t *_E			= instance->E;	
	float32_t *_fai			= instance->fai;
	float32_t *_P			= instance->P;	
	float32_t *_U			= instance->U;	
	float32_t *_uu			= instance->uu;
	float32_t *_W			= instance->W;	
	float32_t *_Y			= instance->Y;	
	float32_t *_y			= instance->y;	
	float32_t *_temp		= instance->temp;

	for (uint32_t i = 1; i <= blockSize / _m; i++) {
//		d = desired(k*M - M + 1:k * M).';
//		uu(1:M) = uu(M + 1:end);
//		uu(M + 1:end) = u(k*M - M + 1:k * M);
//		U = fft(uu); % Fourier transform of input signal block		

		memcpy(_d, d + i * _m - _m + 1, _m + 1);		
		memcpy(_uu, _uu + _m + 1, _m * sizeof(float));
		memcpy(_uu + _m + 1, x + (i*_m) - _m + 1, (_m + 1) * sizeof(float));		
		arm_rfft_fast_f32(_arm_fft,_uu, _U, 0);

//======================================================================================================================================
//		Y = U.*W; % apply filter
		arm_mult_f32(_U, _W, _Y, 2 * M);
		
//		y = ifft(Y);
		arm_rfft_fast_f32(_arm_fft, _Y, _y, 1);		
//		y = y(M + 1:2 * M); % output
		memcpy(_y, _y, (M - 1) * sizeof(float));

//======================================================================================================================================
//		e = d - y; % error
		arm_sub_f32(_d, _y, _e, M);
//		E = fft(e);
		arm_rfft_fast_f32(_arm_fft, _e, _E, 0);
//		P = gamma.*P + (1 - gamma).*abs(U). ^ 2;
//		D = 1. / P;
//======================================================================================================================================
//		fai = ifft(D.*conj(U).*E);
		arm_mult_f32(_D, _U, _temp, 2 * M);
		arm_mult_f32(_temp, _E, _temp, 2 * M);
		arm_rfft_fast_f32(_arm_fft, _temp, _fai, 1);
//		fai = fai.*[ones(M, 1); zeros(M, 1)];
//		W = W + alpha * fft(fai);
		arm_rfft_fast_f32(_arm_fft, _fai, _temp, 0);
		arm_scale_f32(_temp, _alpha, _temp, 2 * M);
		arm_add_f32(_W, _temp, _W, 2 * M);
//======================================================================================================================================
//		error(k*M - M + 1:k * M) = e;
//		out(k*M - M + 1:k * M) = y;
		memcpy(out + i * M - M + 1, _y, (M + 1) * sizeof(float));
	}
}