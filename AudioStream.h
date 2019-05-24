#pragma once
#ifndef _AUDIO_STREAM_H_
#define _AUDIO_STREAM_H_

#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include "config.h"
#include "ring_buffer.h"


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