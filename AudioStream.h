#pragma once
#ifndef _AUDIO_STREAM_H_
#define _AUDIO_STREAM_H_

#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include "ring_buffer.h"

#ifndef DEBUG
#define DEBUG 0
#endif // DEBUG

#ifndef SAMPLE_RATE
#define SAMPLE_RATE   (44100)
#endif //SAMPLE_RATE

#ifndef FRAMES_PER_BUFFER
#define FRAMES_PER_BUFFER  (2048)
#endif //FRAMES_PER_BUFFER

#ifndef TIME_DEBUG
#define TIME_DEBUG 0
#endif // TIME_DEBUG

namespace AS {

	class AudioStream
	{
		std::ifstream fin3;
		RingBuffer::RingBuffer<float> *musicStream;
		size_t num_elements;
	public:
		std::vector<float> data3;

	public:
		AudioStream();
		AudioStream(std::string);
		~AudioStream();
		void openFile(std::string);
		void setUpBuffer(RingBuffer::RingBuffer<float> *);
		void updateBuffer();
	};
}

#endif // !_AUDIO_STREAM_H_