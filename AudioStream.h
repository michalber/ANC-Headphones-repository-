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
		std::ifstream filein;		
		boost::circular_buffer<float> *musicStream;		
		size_t num_elements;
		unsigned long temp = 0;
	public:
		std::vector<float> data;

	public:
		AudioStream();
		AudioStream(std::string);
		~AudioStream();
		void openFile(std::string);		
		void setUpBuffer(boost::circular_buffer<float> *);
		void updateBuffer(int size = FRAMES_PER_BUFFER);
		void updateBufferOnce();
	};
}

#endif // !_AUDIO_STREAM_H_