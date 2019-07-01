#include "../ANC-Headphones-repository-/AudioStream.h"

namespace AS {
	//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief Default constructor of AudioStream class
			@author	Micha³ Berdzik
			@version 0.0.1 19-04-2019
		*/
	AudioStream::AudioStream()
	{
	}
	//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief Parametrized constructor of AudioStream class
			@author	Micha³ Berdzik
			@version 0.0.1 19-04-2019
		*/
	AudioStream::AudioStream(std::string path)
	{
		filein = std::ifstream(path, std::ios::binary | std::ios::in);
		if (!filein)
		{
			std::cout << " Error, Couldn't find the file" << "\n";
		}
		else {

			filein.seekg(0, std::ios::end);
			num_elements = filein.tellg() / sizeof(float);
			filein.seekg(0, std::ios::beg);

			data = std::vector<float>(num_elements);
			filein.read(reinterpret_cast<char*>(&data[0]), num_elements * sizeof(float));
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief Destructor of AudioStream class
			@author	Micha³ Berdzik
			@version 0.0.1 19-04-2019
		*/
	AudioStream::~AudioStream()
	{
	}

	void AudioStream::openFile(std::string path)
	{
		filein = std::ifstream(path, std::ios::binary | std::ios::in);
		if (!filein)
		{
			std::cout << " Error, Couldn't find the file" << "\n";
		}
		else {

			filein.seekg(0, std::ios::end);
			num_elements = filein.tellg() / sizeof(float);
			filein.seekg(0, std::ios::beg);

			data = std::vector<float>(num_elements);
			filein.read(reinterpret_cast<char*>(&data[0]), num_elements * sizeof(float));
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief Function to assign pointer to outer buffer
			@author	Micha³ Berdzik
			@version 0.0.1 19-04-2019
		*/ 
	void AudioStream::setUpBuffer(boost::circular_buffer<float>* x)
	{
		musicStream = x;
	}
	//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief Function to load new frame of data to buffer
			@author	Micha³ Berdzik
			@version 0.0.1 19-04-2019
		*/
	void AudioStream::updateBuffer(int size)
	{
		unsigned long i = 0;		
		if (temp < data.size()) {
			for (i = temp; i < temp + size; i++)
			{
				if(!musicStream->full())
					musicStream->push_front(data[i]);
				else break;
			}
			temp = i;
		}
		else {
			temp = 0;
			i = 0;
		}		
	}
	void AudioStream::updateBufferOnce()
	{
		if (temp < data.size()) {			
			if (!musicStream->full())
				musicStream->push_front(data[temp++]);							
		}
		else {
			temp = 0;			
		}
	}
}