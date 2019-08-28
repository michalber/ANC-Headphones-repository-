#include <iostream>
#include <chrono>
#include "ANC_System.h"
#include "arm_math.h"
#include <stdint.h>
#include <inttypes.h>

#include "AudioStream.h"
#include "NLMS.h"
#include "AudioStream.h"
#include <liquid/liquid.h>
#include <math.h>

using namespace std;

#if !PRINT_AUDIO_DEV_INFO
#if INTERNAL_TEST == RUN_ANC
int main(void)
{
	init_ANC_System();
	getchar();
	destroy_ANC_System();
	return 0;
}
#elif INTERNAL_TEST == GENERATE_AUDIO
int main(void)
{
//	init_ANC_System();

	int size = 2646000;

//	std::ofstream fout("Results1.txt", std::ios::binary);
	std::ofstream fout("Out_240_009.raw", std::ios::binary);

	AS::AudioStream Noise;
	AS::AudioStream Error;

	if (!Noise.openFile(std::string(DATA_PATH) + "ch4x.raw"))
		exit(0);
	if (!Error.openFile(std::string(DATA_PATH) + "ch04d.raw"))
		exit(0);
//	if (!Signal.openFile(std::string(DATA_PATH) + "signal.raw"))
//		exit(0);

	//arm_fir_instance_f32 HPF_instance;
	//float32_t HPFfirStateF32[60 + FRAMES_PER_BUFFER];
	//float32_t HPFFilter[60];
	//liquid_firdes_kaiser(NUM_OF_TAPS, 0.5, 60.0f, 0.0f, HPFFilter);
	//arm_fir_init_f32(&HPF_instance, 60, (float32_t *)HPFFilter, HPFfirStateF32, FRAMES_PER_BUFFER);

	// 10 x NLMS Filter,MU size, Pass, SNR
	float OutResults[2000][4] = { 0 };
	int MIN_FILTER_SIZE = 300;
	int MAX_FILTER_SIZE = 600;
	int FILTER_STEP = (MAX_FILTER_SIZE - MIN_FILTER_SIZE) / 20;
	float MIN_MU_SIZE = 0.00001f;
	float MAX_MU_SIZE = 0.01f;
	float MU_STEP = (MAX_MU_SIZE - MIN_MU_SIZE) / 10;
	
	float SNR_Noise = 0;
	float SNR_Signal = 0;
	for (int i = 0; i < FRAMES_PER_BUFFER * 82687; i++) {
		SNR_Noise += (Noise.data[i] * Noise.data[i]);
	}

	int num = 0; 
//	for (int j = MIN_FILTER_SIZE; j <= MAX_FILTER_SIZE; j += FILTER_STEP) {
//		for (float k = MIN_MU_SIZE; k <= MAX_MU_SIZE; k += MU_STEP) {
			
//			cout << j << " " << k << endl;
			
			//OutResults[num][0] = j;
			//OutResults[num][1] = k;
			//float SNR = 0;
			//SNR_Signal = 0;

//			arm_fir_instance_f32 antyaliasingFIR_instance;
//			float32_t antyaliasingFIRStateF32[60 + FRAMES_PER_BUFFER];
//			float32_t antyaliasingFilter[60];
//			liquid_firdes_kaiser(j, 0.5, 60.0f, 0.0f, antyaliasingFilter);
//			arm_fir_init_f32(&antyaliasingFIR_instance, 60, (float32_t *)antyaliasingFilter, antyaliasingFIRStateF32, FRAMES_PER_BUFFER);

			float32_t *p = Noise.data;
			float32_t *d = Error.data;
//			float32_t lmsStateF32[j + FRAMES_PER_BUFFER];
			float32_t lmsStateF32[NUM_OF_TAPS + FRAMES_PER_BUFFER];
//			float32_t lmsNormCoeff_f32[j];
			float32_t lmsNormCoeff_f32[NUM_OF_TAPS];
//			liquid_firdes_kaiser(j, 0.5, 60.0f, 0.0f, lmsNormCoeff_f32);
			liquid_firdes_kaiser(NUM_OF_TAPS, 0.5, 60.0f, 0.0f, lmsNormCoeff_f32);

			float32_t errOutput[FRAMES_PER_BUFFER];
			float32_t Out[FRAMES_PER_BUFFER];
			float cosTamError[FRAMES_PER_BUFFER] = { 0 };
			float cosTamOut[FRAMES_PER_BUFFER] = { 0 };

			uint32_t i;
			arm_status status;
			uint32_t index;
			float32_t minValue;

			arm_lms_norm_instance_f32 lmsNorm_instance;
//			arm_lms_norm_init_f32(&lmsNorm_instance, j, lmsNormCoeff_f32, lmsStateF32, k, FRAMES_PER_BUFFER);
			arm_lms_norm_init_f32(&lmsNorm_instance, NUM_OF_TAPS, lmsNormCoeff_f32, lmsStateF32, MU, FRAMES_PER_BUFFER);

			//Adaptive::NLMS nlms;
			//nlms.arm_lms_norm_init_f32(NUM_OF_TAPS, MU, FRAMES_PER_BUFFER);			

			for (int i = 0; i < 82687; i++) {

auto start = std::chrono::system_clock::now();
				//nlms.arm_lms_norm_f32(p, d, cosTamOut, cosTamError, FRAMES_PER_BUFFER);	

				arm_lms_norm_f32(&lmsNorm_instance, /* LMSNorm instance */
					p,                         /* Input signal */
					d,                         /* Reference Signal */
					cosTamOut,                         /* Converged Signal */
					cosTamError,                    /* Error Signal, this will become small as the signal converges */
					FRAMES_PER_BUFFER);                    /* BlockSize */

				//arm_fir_f32(&antyaliasingFIR_instance, cosTamOut, cosTamOut, FRAMES_PER_BUFFER);

auto end = std::chrono::system_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

				//if (duration.count() / FRAMES_PER_BUFFER > 20) {
				//	OutResults[num][2] = 0;
				//	break;
				//}
				//else {
				//	OutResults[num][2] = 1;
				//}

				for (int j = 0; j < FRAMES_PER_BUFFER; j++) {
					fout.write(reinterpret_cast<const char*>(&cosTamOut[j]), sizeof(float));
				}

				//for (int l = 0; l < FRAMES_PER_BUFFER; l++) {
				//	SNR_Signal += ((*(d + l) - cosTamOut[l]) * (*(d + l) - cosTamOut[l]));
				//}
				p += FRAMES_PER_BUFFER;
				d += FRAMES_PER_BUFFER;
			}

			//SNR = (SNR_Signal) / (SNR_Noise);
			//OutResults[num][3] = 10*log10(SNR);		
			//num++;

//			cout << duration.count() << endl;
			fout.close();

//		}
//	}

//	cout << endl;
//	for (int i = 0; i < num ; i++) {
//		cout << "Filter len: " << OutResults[i][0] << ", ";
//		cout << "Mu: " << OutResults[i][1] << ", ";
//		cout << "Pass? " << OutResults[i][2] << ", ";
//		cout << "SNR: " << OutResults[i][3];
//		cout << endl;
//
//		
//		fout << "Filter len: " << OutResults[i][0] << ", ";
//		fout << "Mu: " << OutResults[i][1] << ", ";
//		fout << "Pass? " << OutResults[i][2] << ", ";
//		fout << "SNR: " << OutResults[i][3];
//		fout << endl;
//	}
//	fout.close();

	getchar();
	return 0;
}

#elif INTERNAL_TEST == RUN_SNR_TESTS

int main(void)
{
	//	init_ANC_System();

	int size = 2646000;

	//	std::ofstream fout("Results1.txt", std::ios::binary);
	std::ofstream fout("Results.txt", std::ios::binary);

	AS::AudioStream Noise;
	AS::AudioStream Error;

	if (!Noise.openFile(std::string(DATA_PATH) + "ch4x.raw"))
		exit(0);
	if (!Error.openFile(std::string(DATA_PATH) + "ch04d.raw"))
		exit(0);
	//	if (!Signal.openFile(std::string(DATA_PATH) + "signal.raw"))
	//		exit(0);

		//arm_fir_instance_f32 HPF_instance;
		//float32_t HPFfirStateF32[60 + FRAMES_PER_BUFFER];
		//float32_t HPFFilter[60];
		//liquid_firdes_kaiser(NUM_OF_TAPS, 0.5, 60.0f, 0.0f, HPFFilter);
		//arm_fir_init_f32(&HPF_instance, 60, (float32_t *)HPFFilter, HPFfirStateF32, FRAMES_PER_BUFFER);

		// 10 x NLMS Filter,MU size, Pass, SNR
	float OutResults[2000][4] = { 0 };
	int MIN_FILTER_SIZE = 100;
	int MAX_FILTER_SIZE = 300;
	int FILTER_STEP = (MAX_FILTER_SIZE - MIN_FILTER_SIZE) / 20;
	float MIN_MU_SIZE = 0.00001f;
	float MAX_MU_SIZE = 0.01f;
	float MU_STEP = (MAX_MU_SIZE - MIN_MU_SIZE) / 10;

	float SNR_Noise = 0;
	float SNR_Signal = 0;
	for (int i = 0; i < FRAMES_PER_BUFFER * 82687; i++) {
		SNR_Noise += (Noise.data[i] * Noise.data[i]);
	}

	int num = 0;
	for (int j = MIN_FILTER_SIZE; j <= MAX_FILTER_SIZE; j += FILTER_STEP) {
		for (float k = MIN_MU_SIZE; k <= MAX_MU_SIZE; k += MU_STEP) {

			cout << j << " " << k << endl;

			OutResults[num][0] = j;
			OutResults[num][1] = k;
			float SNR = 0;
			SNR_Signal = 0;

			//arm_fir_instance_f32 antyaliasingFIR_instance;
			//float32_t antyaliasingFIRStateF32[60 + FRAMES_PER_BUFFER];
			//float32_t antyaliasingFilter[60];
			//liquid_firdes_kaiser(j, 0.5, 60.0f, 0.0f, antyaliasingFilter);
			//arm_fir_init_f32(&antyaliasingFIR_instance, 60, (float32_t *)antyaliasingFilter, antyaliasingFIRStateF32, FRAMES_PER_BUFFER);

			float32_t *p = Noise.data;
			float32_t *d = Error.data;
			float32_t lmsStateF32[j + FRAMES_PER_BUFFER];
			float32_t lmsNormCoeff_f32[j];
			liquid_firdes_kaiser(j, 0.5, 60.0f, 0.0f, lmsNormCoeff_f32);

			float32_t errOutput[FRAMES_PER_BUFFER];
			float32_t Out[FRAMES_PER_BUFFER];
			float cosTamError[FRAMES_PER_BUFFER] = { 0 };
			float cosTamOut[FRAMES_PER_BUFFER] = { 0 };

			uint32_t i;
			arm_status status;
			uint32_t index;
			float32_t minValue;

			arm_lms_norm_instance_f32 lmsNorm_instance;
			arm_lms_norm_init_f32(&lmsNorm_instance, j, lmsNormCoeff_f32, lmsStateF32, k, FRAMES_PER_BUFFER);

			for (int i = 0; i < 82687; i++) {

				auto start = std::chrono::system_clock::now();

				arm_lms_norm_f32(&lmsNorm_instance, /* LMSNorm instance */
					p,                         /* Input signal */
					d,                         /* Reference Signal */
					cosTamOut,                         /* Converged Signal */
					cosTamError,                    /* Error Signal, this will become small as the signal converges */
					FRAMES_PER_BUFFER);                    /* BlockSize */

				//arm_fir_f32(&antyaliasingFIR_instance, cosTamOut, cosTamOut, FRAMES_PER_BUFFER);

				auto end = std::chrono::system_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

				if (duration.count() / FRAMES_PER_BUFFER > 20) {
					OutResults[num][2] = 0;
					break;
				}
				else {
					OutResults[num][2] = 1;
				}

				for (int l = 0; l < FRAMES_PER_BUFFER; l++) {
					SNR_Signal += ((*(d + l) - cosTamOut[l]) * (*(d + l) - cosTamOut[l]));
				}
				p += FRAMES_PER_BUFFER;
				d += FRAMES_PER_BUFFER;
			}

			SNR = (SNR_Signal) / (SNR_Noise);
			OutResults[num][3] = 10 * log10(SNR);
			num++;
		}
	}

		cout << endl;
		for (int i = 0; i < num ; i++) {
			cout << "Filter len: " << OutResults[i][0] << ", ";
			cout << "Mu: " << OutResults[i][1] << ", ";
			cout << "Pass? " << OutResults[i][2] << ", ";
			cout << "SNR: " << OutResults[i][3];
			cout << endl;
	
			
			fout << "Filter len: " << OutResults[i][0] << ", ";
			fout << "Mu: " << OutResults[i][1] << ", ";
			fout << "Pass? " << OutResults[i][2] << ", ";
			fout << "SNR: " << OutResults[i][3];
			fout << endl;
		}
		fout.close();

	getchar();
	return 0;
}
#endif
#else

/** @file pa_devs.c
	@ingroup examples_src
	@brief List available devices, including device information.
	@author Phil Burk http://www.softsynth.com

	@note Define PA_USE_ASIO=0 to compile this code on Windows without
		ASIO support.
*/
/*
 * $Id$
 *
 * This program uses the PortAudio Portable Audio Library.
 * For more information see: http://www.portaudio.com
 * Copyright (c) 1999-2000 Ross Bencina and Phil Burk
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

 /*
  * The text above constitutes the entire PortAudio license; however,
  * the PortAudio community also makes the following non-binding requests:
  *
  * Any person wishing to distribute modifications to the Software is
  * requested to send the modifications to the original developer so that
  * they can be incorporated into the canonical version. It is also
  * requested that these non-binding requests be included along with the
  * license above.
  */

#include <stdio.h>
#include <math.h>
#include "portaudio.h"

#ifdef WIN32
#include <windows.h>

#if PA_USE_ASIO
#include "pa_asio.h"
#endif
#endif

  /*******************************************************************/
static void PrintSupportedStandardSampleRates(
	const PaStreamParameters *inputParameters,
	const PaStreamParameters *outputParameters)
{
	static double standardSampleRates[] = {
		8000.0, 9600.0, 11025.0, 12000.0, 16000.0, 22050.0, 24000.0, 32000.0,
		44100.0, 48000.0, 88200.0, 96000.0, 192000.0, -1 /* negative terminated  list */
	};
	int     i, printCount;
	PaError err;

	printCount = 0;
	for (i = 0; standardSampleRates[i] > 0; i++)
	{
		err = Pa_IsFormatSupported(inputParameters, outputParameters, standardSampleRates[i]);
		if (err == paFormatIsSupported)
		{
			if (printCount == 0)
			{
				printf("\t%8.2f", standardSampleRates[i]);
				printCount = 1;
			}
			else if (printCount == 4)
			{
				printf(",\n\t%8.2f", standardSampleRates[i]);
				printCount = 1;
			}
			else
			{
				printf(", %8.2f", standardSampleRates[i]);
				++printCount;
			}
		}
	}
	if (!printCount)
		printf("None\n");
	else
		printf("\n");
}

/*******************************************************************/
int main(void);
int main(void)
{
	int     i, numDevices, defaultDisplayed;
	const   PaDeviceInfo *deviceInfo;
	PaStreamParameters inputParameters, outputParameters;
	PaError err;


	err = Pa_Initialize();
	if (err != paNoError)
	{
		printf("ERROR: Pa_Initialize returned 0x%x\n", err);
		goto error;
	}

	printf("PortAudio version: 0x%08X\n", Pa_GetVersion());
	printf("Version text: '%s'\n", Pa_GetVersionInfo()->versionText);

	numDevices = Pa_GetDeviceCount();
	if (numDevices < 0)
	{
		printf("ERROR: Pa_GetDeviceCount returned 0x%x\n", numDevices);
		err = numDevices;
		goto error;
	}

	printf("Number of devices = %d\n", numDevices);
	for (i = 0; i < numDevices; i++)
	{
		deviceInfo = Pa_GetDeviceInfo(i);
		printf("--------------------------------------- device #%d\n", i);

		/* Mark global and API specific default devices */
		defaultDisplayed = 0;
		if (i == Pa_GetDefaultInputDevice())
		{
			printf("[ Default Input");
			defaultDisplayed = 1;
		}
		else if (i == Pa_GetHostApiInfo(deviceInfo->hostApi)->defaultInputDevice)
		{
			const PaHostApiInfo *hostInfo = Pa_GetHostApiInfo(deviceInfo->hostApi);
			printf("[ Default %s Input", hostInfo->name);
			defaultDisplayed = 1;
		}

		if (i == Pa_GetDefaultOutputDevice())
		{
			printf((defaultDisplayed ? "," : "["));
			printf(" Default Output");
			defaultDisplayed = 1;
		}
		else if (i == Pa_GetHostApiInfo(deviceInfo->hostApi)->defaultOutputDevice)
		{
			const PaHostApiInfo *hostInfo = Pa_GetHostApiInfo(deviceInfo->hostApi);
			printf((defaultDisplayed ? "," : "["));
			printf(" Default %s Output", hostInfo->name);
			defaultDisplayed = 1;
		}

		if (defaultDisplayed)
			printf(" ]\n");

		/* print device info fields */
#ifdef WIN32
		{   /* Use wide char on windows, so we can show UTF-8 encoded device names */
			wchar_t wideName[MAX_PATH];
			MultiByteToWideChar(CP_UTF8, 0, deviceInfo->name, -1, wideName, MAX_PATH - 1);
			wprintf(L"Name                        = %s\n", wideName);
		}
#else
		printf("Name                        = %s\n", deviceInfo->name);
#endif
		printf("Host API                    = %s\n", Pa_GetHostApiInfo(deviceInfo->hostApi)->name);
		printf("Max inputs = %d", deviceInfo->maxInputChannels);
		printf(", Max outputs = %d\n", deviceInfo->maxOutputChannels);

		printf("Default low input latency   = %8.4f\n", deviceInfo->defaultLowInputLatency);
		printf("Default low output latency  = %8.4f\n", deviceInfo->defaultLowOutputLatency);
		printf("Default high input latency  = %8.4f\n", deviceInfo->defaultHighInputLatency);
		printf("Default high output latency = %8.4f\n", deviceInfo->defaultHighOutputLatency);

#ifdef WIN32
#if PA_USE_ASIO
		/* ASIO specific latency information */
		if (Pa_GetHostApiInfo(deviceInfo->hostApi)->type == paASIO) {
			long minLatency, maxLatency, preferredLatency, granularity;

			err = PaAsio_GetAvailableLatencyValues(i,
				&minLatency, &maxLatency, &preferredLatency, &granularity);

			printf("ASIO minimum buffer size    = %ld\n", minLatency);
			printf("ASIO maximum buffer size    = %ld\n", maxLatency);
			printf("ASIO preferred buffer size  = %ld\n", preferredLatency);

			if (granularity == -1)
				printf("ASIO buffer granularity     = power of 2\n");
			else
				printf("ASIO buffer granularity     = %ld\n", granularity);
		}
#endif /* PA_USE_ASIO */
#endif /* WIN32 */

		printf("Default sample rate         = %8.2f\n", deviceInfo->defaultSampleRate);

		/* poll for standard sample rates */
		inputParameters.device = i;
		inputParameters.channelCount = deviceInfo->maxInputChannels;
		inputParameters.sampleFormat = paInt16;
		inputParameters.suggestedLatency = 0; /* ignored by Pa_IsFormatSupported() */
		inputParameters.hostApiSpecificStreamInfo = NULL;

		outputParameters.device = i;
		outputParameters.channelCount = deviceInfo->maxOutputChannels;
		outputParameters.sampleFormat = paInt16;
		outputParameters.suggestedLatency = 0; /* ignored by Pa_IsFormatSupported() */
		outputParameters.hostApiSpecificStreamInfo = NULL;

		if (inputParameters.channelCount > 0)
		{
			printf("Supported standard sample rates\n for half-duplex 16 bit %d channel input = \n",
				inputParameters.channelCount);
			PrintSupportedStandardSampleRates(&inputParameters, NULL);
		}

		if (outputParameters.channelCount > 0)
		{
			printf("Supported standard sample rates\n for half-duplex 16 bit %d channel output = \n",
				outputParameters.channelCount);
			PrintSupportedStandardSampleRates(NULL, &outputParameters);
		}

		if (inputParameters.channelCount > 0 && outputParameters.channelCount > 0)
		{
			printf("Supported standard sample rates\n for full-duplex 16 bit %d channel input, %d channel output = \n",
				inputParameters.channelCount, outputParameters.channelCount);
			PrintSupportedStandardSampleRates(&inputParameters, &outputParameters);
		}
	}

	Pa_Terminate();

	printf("----------------------------------------------\n");
	getchar();
	return 0;

error:
	Pa_Terminate();
	fprintf(stderr, "Error number: %d\n", err);
	fprintf(stderr, "Error message: %s\n", Pa_GetErrorText(err));
	return err;
}
#endif