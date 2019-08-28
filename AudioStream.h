#pragma once
#ifndef _AUDIO_STREAM_H_
#define _AUDIO_STREAM_H_

#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <liquid/liquid.h>
#include <stdio.h>
#include <string.h>


#include "config.h"


namespace AS {

	class AudioStream
	{
		std::ifstream inputFile;
		cbufferf* externalBuffer;
		float *extBuffer;
		//size_t numof_elements = 0;
		unsigned long temp = 0;
	public:
		float *data = NULL;
		size_t numof_elements = 0;

	public:
		AudioStream();
		AudioStream(std::string);
		~AudioStream();
		bool openFile(std::string);
		void setUpBuffer(cbufferf * _buffer, float* exBuffer);
		void updateBuffer(int size = FRAMES_PER_BUFFER);
		void updateBufferOnce();
	};
}

#endif // !_AUDIO_STREAM_H_