#include "AudioInterface.h"

namespace AI {

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Default constructor of AudioInterface class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/
	AudioInterface::AudioInterface() : stream(0)
	{
#if DEBUG
		sprintf_s(message, "AudioInterface Initialized");
#endif
	}

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Destructor of AudioInterface class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/
	AudioInterface::~AudioInterface()
	{
	}

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief open method of AudioInterface class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param PaDeviceIndex index 
		@return bool - successful opening of audio device
	*/
	bool AudioInterface::open(PaDeviceIndex index)
	{		
		PaAlsa_EnableRealtimeScheduling(&stream, 1);

		PaStreamParameters outputParameters;
		PaStreamParameters inputParameters;

		outputParameters.device = 0;
		if (outputParameters.device == paNoDevice) {
			return false;
		}
		const PaDeviceInfo* pInfo = Pa_GetDeviceInfo(0);
		if (pInfo != 0)
		{
			//sprintf_s("Output device name: '%s'\r", pInfo->name);
			std::cout << "Output device name: " << pInfo->name << std::endl;
		}

		inputParameters.device = 3;
		if (inputParameters.device == paNoDevice) {
			return false;
		}
		pInfo = Pa_GetDeviceInfo(3);
		if (pInfo != 0)
		{
			//sprintf_s("Input device name: '%s'\r", pInfo->name);
			std::cout << "Input device name: " << pInfo->name << std::endl;
		}
		
		//outputParameters.channelCount = 1;       /* mono output */
		outputParameters.channelCount = 2;       /* stereo output */		
		outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */				
		//outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
		outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultHighOutputLatency;
		//outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultHighOutputLatency;
		outputParameters.hostApiSpecificStreamInfo = NULL;
						
		//inputParameters.channelCount = 1;       /* mono input */
		inputParameters.channelCount = 2;       /* stereo input */		
		inputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
		//inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;	
		inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultHighInputLatency;
		//inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultHighInputLatency;
		inputParameters.hostApiSpecificStreamInfo = NULL;

		PaError err = Pa_OpenStream(
			&stream,
			&inputParameters, /* no input */
			&outputParameters,
			SAMPLE_RATE,
			
			NUM_OF_FRAMES,
			//paFramesPerBufferUnspecified,

			paClipOff,      /* we won't output out of range samples so don't bother clipping them */
			//paDitherOff,
			//paNoFlag,

			&AudioInterface::paCallback,
			(void *)this	/* Using 'this' for userData so we can cast to AudioInterface* in paCallback method */
		);		
		
		PaAlsa_EnableRealtimeScheduling(&stream, 1);

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

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief close method of AudioInterface class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@return bool - successful closing of audio stream
	*/
	bool AudioInterface::close()
	{
		if (stream == 0)
			return false;

		PaError err = Pa_CloseStream(stream);
		stream = 0;

		return (err == paNoError);
	}

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief start method of AudioInterface class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@return bool - successful starting of audio stream
	*/
	bool AudioInterface::start()
	{
		if (stream == 0)
			return false;

		PaError err = Pa_StartStream(stream);

		return (err == paNoError);
	}

	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief stop method of AudioInterface class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@return bool - successful stopping of audio stream
	*/
	bool AudioInterface::stop()
	{
		if (stream == 0)
			return false;

		PaError err = Pa_StopStream(stream);

		return (err == paNoError);
	}
	//--------------------------------------------------------------------------------------------------------------------

	/**
		@brief setUpBuffer method of AudioInterface class
		@author	Michal Berdzik
		@version 0.1 05-07-2019
		@param cbufferf *x,*y - pointer to external data buffer
	*/
	void AudioInterface::setUpBuffer(cbufferf* Output, cbufferf* Noise, cbufferf* Error, AudioBuffers::FullDoubleBuffer *extDoubleBuffer, float* extMicsBuffer, float* extOutBuffer, bool *exNewData)	{
		externalPlayBuffer = Output;
		externalNoiseRecordBuffer = Noise;
		externalErrorRecordBuffer = Error;
		externalDoubleBuffer = extDoubleBuffer;
		externalMicsBuffer = extMicsBuffer;
		outBuffer = extOutBuffer;
		extNewData = exNewData;
	}

	//--------------------------------------------------------------------------------
	/**
		@brief Own callback to update Port Audio API

		@author	Michal Berdzik
		@version 0.1 05-07-2019

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
		float *input = (float *)inputBuffer;
		float *out = (float*)outputBuffer;
				
		unsigned long i = 0;

		(void)timeInfo; /* Prevent unused variable warnings. */
		(void)statusFlags;

		static unsigned long temp = 0;


		if (externalDoubleBuffer->FillPop(input, out)) {

			//memmove(externalMicsBuffer, &externalMicsBuffer[2 * FRAMES_PER_BUFFER], (2 * BUFFER_SIZE - 2 * FRAMES_PER_BUFFER) * sizeof(float));
			//memcpy(&externalMicsBuffer[2 * BUFFER_SIZE - 2 * FRAMES_PER_BUFFER], input, 2 * FRAMES_PER_BUFFER * sizeof(float));
			*extNewData = true;
		}		
		//memcpy(out, outBuffer, FRAMES_PER_BUFFER * sizeof(float));

		return paContinue;



		//unsigned int _num_read;
	
		//for (int j = 0; j < framesPerBuffer; j++) {
		//	if (cbufferf_space_available(*externalNoiseRecordBuffer) > 0)
		//		cbufferf_push(*externalNoiseRecordBuffer, *in++);
		//	else
		//		break;
		//}

		//for (i = 0; i < framesPerBuffer; i++) {
		//	//if (cbufferf_max_read(*externalPlayBuffer) > 1)
		//	//{
		//		cbufferf_pop(*externalPlayBuffer, out++);				
		//	//}
		//	//else {
		//	//	return paComplete;
		//	//}			
		//	//if (cbufferf_space_available(*externalNoiseRecordBuffer) > 0)

		//	//if (cbufferf_space_available(*externalErrorRecordBuffer) > 0)
		//		//cbufferf_push(*externalErrorRecordBuffer, *in++);			
		//}
		//return paContinue;

		
		
		/*for (int i = 0; i < 2 * framesPerBuffer; i++) {
			*out++ = (*in++);			
		}
		return paContinue;*/
	}

	//--------------------------------------------------------------------------------
	/**
		@brief paCallback function to assign own callback to run on PortAudio API

		@author	Michal Berdzik
		@version 0.1 05-07-2019

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

		@author	Michal Berdzik
		@version 0.1 05-07-2019

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

		@author	Michal Berdzik
		@version 0.1 05-07-2019
	*/
	void AudioInterface::paStreamFinished(void* userData)
	{
		return ((AudioInterface*)userData)->paStreamFinishedMethod();
	}

} //AI