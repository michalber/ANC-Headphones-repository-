#pragma once
#ifndef _AUDIO_INTERFACE_H_
#define _AUDIO_INTERFACE_H_

#include <atomic>
#include "portaudio.h"
#include "config.h"
#include "ring_buffer.h"
#include "AudioStream.h"


#ifndef M_PI
#define M_PI  (3.14159265)
#endif //M_PI


namespace AI {
	class AudioInterface
	{
		PaStream *stream;
		RingBuffer::RingBuffer<float> *OutputBuffer = NULL;
		std::atomic<int> *dataReaded = NULL;
		double *buffer;
		int left_phase;
		int right_phase;
		char message[20];

	public:
		AudioInterface();
		~AudioInterface();

		bool open(PaDeviceIndex);
		bool close();
		bool start();
		bool stop();
		void setUpBuffer(RingBuffer::RingBuffer<float> *, std::atomic<int> *);

	private:
		int paCallbackMethod(const void *, void *,unsigned long, const PaStreamCallbackTimeInfo*,const PaStreamCallbackFlags);
		static int paCallback(const void *, void *, unsigned long, const PaStreamCallbackTimeInfo*, const PaStreamCallbackFlags, void *);
		void paStreamFinishedMethod();
		static void paStreamFinished(void*);
	};
}

#endif // !_AUDIO_INTERFACE_H_