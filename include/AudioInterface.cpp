#include "../ANC-Headphones-repository-/AudioInterface.h"

namespace AI {

	//--------------------------------------------------------------------------------
	/**
			@brief Default constructor of AudioInterface class

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019
		*/
	AudioInterface::AudioInterface() : stream(0)
	{
#if DEBUG
		sprintf_s(message, "AudioInterface Initialized");
#endif
	}

	//--------------------------------------------------------------------------------
	/**
			@brief Destructor of AudioInterface class

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019

			@param
			@param
			@param
			@return
		*/
	AudioInterface::~AudioInterface()
	{
	}

	//--------------------------------------------------------------------------------
	/**
			@brief Initializes the given ring buffer structure.

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019
		*/
	bool AudioInterface::open(PaDeviceIndex index)
	{
		PaStreamParameters outputParameters;

		outputParameters.device = index;
		if (outputParameters.device == paNoDevice) {
			return false;
		}

		const PaDeviceInfo* pInfo = Pa_GetDeviceInfo(index);
		if (pInfo != 0)
		{
			printf("Output device name: '%s'\r", pInfo->name);
		}

		outputParameters.channelCount = 1;       /* mono output */
		//outputParameters.channelCount = 2;       /* stereo output */
		outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
		outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
		outputParameters.hostApiSpecificStreamInfo = NULL;

		PaError err = Pa_OpenStream(
			&stream,
			NULL, /* no input */
			&outputParameters,
			SAMPLE_RATE,

			FRAMES_PER_BUFFER,
			//paFramesPerBufferUnspecified,

			paClipOff,      /* we won't output out of range samples so don't bother clipping them */
			//paDitherOff,
			//paNoFlag,

			&AudioInterface::paCallback,
			this            /* Using 'this' for userData so we can cast to AudioInterface* in paCallback method */
		);

		if (err != paNoError)
		{
			/* Failed to open stream to device !!! */
			return false;
		}

		err = Pa_SetStreamFinishedCallback(stream, &AudioInterface::paStreamFinished);

		if (err != paNoError)
		{
			Pa_CloseStream(stream);
			stream = 0;

			return false;
		}

		return true;
	}

	//--------------------------------------------------------------------------------
	/**
			@brief Function to close Port Audio Stream

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019
			@return bool
		*/
	bool AudioInterface::close()
	{
		if (stream == 0)
			return false;

		PaError err = Pa_CloseStream(stream);
		stream = 0;

		return (err == paNoError);
	}

	//--------------------------------------------------------------------------------
	/**
			@brief Function to start Port Audio Stream

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019
			@return bool
		*/
	bool AudioInterface::start()
	{
		if (stream == 0)
			return false;

		PaError err = Pa_StartStream(stream);

		return (err == paNoError);
	}

	//--------------------------------------------------------------------------------
	/**
			@brief Function to stop Port Audio Stream

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019
			@return bool
		*/
	bool AudioInterface::stop()
	{
		if (stream == 0)
			return false;

		PaError err = Pa_StopStream(stream);

		return (err == paNoError);
	}

	bool AudioInterface::getNextVecFlag()
	{
		return nextVecReady;
	}

	void AudioInterface::setUpBuffer(RingBuffer::RingBuffer<float>* x, std::atomic<int> *y)
	{		
		dataReaded = y;
	}

	void AudioInterface::setUpBuffer(boost::circular_buffer<float>* x)
	{
		OB = x;
	}
	void AudioInterface::setUpQueue(boost::lockfree::spsc_queue<float>* x)
	{
		outQueue = x;
	}

	//--------------------------------------------------------------------------------
	/**
			@brief Own callback to update Port Audio API

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019

			@param const void *inputBuffer - pointer to PortAudio input buffer
			@param void *outputBuffer - pointer to PortAudio output buffer
			@param unsigned long framesPerBuffer - PortAudio frames per buffer
			@param const PaStreamCallbackTimeInfo* timeInfo
			@param PaStreamCallbackFlags statusFlags
			@return int
		*/
	/* The instance callback, where we have access to every method/variable in object of class XYZ */
	int AudioInterface::paCallbackMethod(const void *inputBuffer, void *outputBuffer,
										 unsigned long framesPerBuffer,
										 const PaStreamCallbackTimeInfo* timeInfo,
										 PaStreamCallbackFlags statusFlags) 
	{
		nextVecReady = false;

		float *out = (float*)outputBuffer;
		unsigned long i = 0;

		(void)timeInfo; /* Prevent unused variable warnings. */
		(void)statusFlags;
		(void)inputBuffer;

		static unsigned long temp = 0;

		//std::cout << "Pobieram próbki: " << temp << std::endl;

		if ((OB->size() >= framesPerBuffer)) {
			for (i = 0; i < framesPerBuffer; i++)
			{
				/*
					We just simply add new data to outputBuffer
				*/		
				if (//OB->size() > FRAMES_PER_BUFFER && 
					!OB->empty()) {
					*out++ = OB->back();					
					OB->pop_back();
				}
				else return paComplete;
			}
		}		

			nextVecReady = true;

		return paContinue;
	}

	//--------------------------------------------------------------------------------
	/**
			@brief paCallback function to assign own callback to run on PortAudio API

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019

			@param const void *inputBuffer - pointer to PortAudio input buffer
			@param void *outputBuffer - pointer to PortAudio output buffer
			@param unsigned long framesPerBuffer - PortAudio frames per buffer
			@param const PaStreamCallbackTimeInfo* timeInfo
			@param PaStreamCallbackFlags statusFlags
			@param void *userData
			@return int
		*/
	/* This routine will be called by the PortAudio engine when audio is needed.
	** It may called at interrupt level on some machines so don't do anything
	** that could mess up the system like calling malloc() or free().
	*/
	int AudioInterface::paCallback(const void *inputBuffer, void *outputBuffer,
								   unsigned long framesPerBuffer,
								   const PaStreamCallbackTimeInfo* timeInfo,
								   PaStreamCallbackFlags statusFlags,
								   void *userData)
	{
		/* Here we cast userData to Sine* type so we can call the instance method paCallbackMethod, we can do that since
		   we called Pa_OpenStream with 'this' for userData */

		return ((AudioInterface*)userData)->paCallbackMethod(inputBuffer, outputBuffer,
			framesPerBuffer,
			timeInfo,
			statusFlags);
		return 0;
	}

	//--------------------------------------------------------------------------------
	/**
			@brief Function to print message when DEBUG is on and when Stream finished work

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019

		*/
	void AudioInterface::paStreamFinishedMethod()
	{
#if DEBUG
		printf("Stream Completed: %s\n", message);
#endif
	}

	//--------------------------------------------------------------------------------
	/**
			@brief This routine is called by portaudio when playback is done.

			@author	Micha³ Berdzik
			@version 0.0.1 18-05-2019

	*/
	void AudioInterface::paStreamFinished(void* userData)
	{
		return ((AudioInterface*)userData)->paStreamFinishedMethod();
	}

} //AI