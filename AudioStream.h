#pragma once
#ifndef _AUDIO_STREAM_H_
#define _AUDIO_STREAM_H_

#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <boost/circular_buffer.hpp>
#include "config.h"
#include "ring_buffer.h"


namespace AS {

	class AudioStream
	{
		std::ifstream fin3;
		//RingBuffer::RingBuffer<float> *musicStream;
		boost::circular_buffer<float> *musicStream;
		unsigned long num_elements;
	public:
		std::vector<float> data3;

	public:
		AudioStream();
		AudioStream(std::string);
		~AudioStream();
		void openFile(std::string);
		void setUpBuffer(RingBuffer::RingBuffer<float> *);
		void setUpBuffer(boost::circular_buffer<float> *);
		void updateBuffer();
	};
}

#endif // !_AUDIO_STREAM_H_