#pragma once
#ifndef _AUDIO_INTERFACE_H_
#define _AUDIO_INTERFACE_H_

#include <atomic>
#include <vector>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <iostream>
#include <string>
#include <string.h>

#include <portaudio.h>
#include <liquid/liquid.h>
#include <pa_linux_alsa.h>
#include "config.h"
#include "FullDoubleBuffer.h"

namespace AI {
	class AudioInterface
	{
		PaStream *stream;

		cbufferf *externalPlayBuffer;
		cbufferf *externalNoiseRecordBuffer;
		cbufferf *externalErrorRecordBuffer;

		AudioBuffers::FullDoubleBuffer *externalDoubleBuffer;

		float *externalMicsBuffer;		
		float *outBuffer;

		bool *extNewData;
		char message[20];

	public:
		AudioInterface();
		~AudioInterface();

		bool open(PaDeviceIndex);
		bool close();
		bool start();
		bool stop();
		bool getNextVecFlag();

		void setUpBuffer(cbufferf *Output, cbufferf *Noise, cbufferf *Error, AudioBuffers::FullDoubleBuffer *extDoubleBuffer, float* extInNoiseBuffer, float* extOutBuffer, bool *extNewData);


	private:
		int paCallbackMethod(const void *, void *, unsigned long, const PaStreamCallbackTimeInfo*, const PaStreamCallbackFlags);
		static int paCallback(const void *, void *, unsigned long, const PaStreamCallbackTimeInfo*, const PaStreamCallbackFlags, void *);
		void paStreamFinishedMethod();
		static void paStreamFinished(void*);
	};
}

#endif // !_AUDIO_INTERFACE_H_