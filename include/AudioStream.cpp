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
		fin3 = std::ifstream(path, std::ios::binary | std::ios::in);
		if (!fin3)
		{
			std::cout << " Error, Couldn't find the file" << "\n";
		}
		else {

			fin3.seekg(0, std::ios::end);
			num_elements = fin3.tellg() / sizeof(float);
			fin3.seekg(0, std::ios::beg);

			data3 = std::vector<float>(num_elements);
			fin3.read(reinterpret_cast<char*>(&data3[0]), num_elements * sizeof(float));
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
		fin3 = std::ifstream(path, std::ios::binary | std::ios::in);
		if (!fin3)
		{
			std::cout << " Error, Couldn't find the file" << "\n";
		}
		else {

			fin3.seekg(0, std::ios::end);
			num_elements = fin3.tellg() / sizeof(float);
			fin3.seekg(0, std::ios::beg);

			data3 = std::vector<float>(num_elements);
			fin3.read(reinterpret_cast<char*>(&data3[0]), num_elements * sizeof(float));
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
	void AudioStream::updateBuffer()
	{
		unsigned long i = 0;		
		if (temp < data3.size()) {
			for (i = temp; i < temp + FRAMES_PER_BUFFER; i++)
			{
				if(!musicStream->full())
					musicStream->push_front(data3[i]);
				else break;
			}
			temp = i;
		}
		else {
			temp = 0;
			i = 0;
		}		
	}
}