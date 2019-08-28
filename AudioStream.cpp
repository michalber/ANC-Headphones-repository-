#include "AudioStream.h"

namespace AS {
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Default constructor of AudioStream class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/
	AudioStream::AudioStream()
	{
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Parametrized constructor of AudioStream class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param std::string path - path to file with audio data
	*/
	AudioStream::AudioStream(std::string path)
	{
		openFile(path);
	}
	//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief Destructor of AudioStream class
			@author	Micha� Berdzik
			@version 0.0.1 19-04-2019
		*/
	AudioStream::~AudioStream()
	{
	}

	bool AudioStream::openFile(std::string path)
	{
		inputFile = std::ifstream(path, std::ios::binary | std::ios::in);
		if (!inputFile)
		{
			std::cout << " Error, Couldn't find the file" << "\n";
			return false;
		}
		else {

			inputFile.seekg(0, std::ios::end);
			numof_elements = inputFile.tellg() / sizeof(float);
			inputFile.seekg(0, std::ios::beg);

			data = new float[numof_elements];
			inputFile.read(reinterpret_cast<char*>(&data[0]), numof_elements * sizeof(float));
			return true;
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief Function to assign pointer to outer buffer
			@author	Micha� Berdzik
			@version 0.0.1 19-04-2019
		*/
	void AudioStream::setUpBuffer(cbufferf * _buffer, float * exBuffer)
	{
		externalBuffer = _buffer;
		extBuffer = exBuffer;
	}
	//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief Function to load new frame of data to buffer
			@author	Micha� Berdzik
			@version 0.0.1 19-04-2019
		*/
	void AudioStream::updateBuffer(int size)
	{		
		if (temp + FRAMES_PER_BUFFER <= numof_elements) {
			//for (int i = 0; i < size; i++) {
			//	if (cbufferf_space_available(*externalBuffer) > 0)
			//		cbufferf_push(*externalBuffer, data[temp++]);
			//	else
			//		break;
			//	//temp += size;			
			//}
				
			//memmove(extBuffer, &extBuffer[size], (BUFFER_SIZE - size) * sizeof(float));
			//memcpy(&extBuffer[BUFFER_SIZE - size], &data[temp], size * sizeof(float));
			//temp += size;
			for (int i = 0; i < FRAMES_PER_BUFFER; i++) {
				extBuffer[BUFFER_SIZE - FRAMES_PER_BUFFER + i] = data[temp++];
			}
		}
		else {
			temp = 0;			
		}
	}

	void AudioStream::updateBufferOnce()
	{
		if (temp + 1 < numof_elements) {
			//if (cbufferf_space_available(*externalBuffer) > 0) {
			//	cbufferf_push(*externalBuffer, data[temp++]);
			//}
			memmove(&extBuffer[0], &extBuffer[1], (BUFFER_SIZE - 1) * sizeof(float));
			memcpy(&extBuffer[BUFFER_SIZE - 1], &data[temp++], 1 * sizeof(float));
		}
		else {
			temp = 0;
		}
	}
}