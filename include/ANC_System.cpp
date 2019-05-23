#include "ANC_System.h"

namespace ANC {
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Default constructor of ANC_System class
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	ANC_System::ANC_System() :Lambda(0.9999), M(40)
	{
		x = arma::vec(FRAMES_PER_BUFFER, arma::fill::zeros);
		d = arma::vec(FRAMES_PER_BUFFER, arma::fill::zeros);

		NoiseInputBuffer.RingBuffer_SetSize(FRAMES_PER_BUFFER);
		ErrorInputBuffer.RingBuffer_SetSize(FRAMES_PER_BUFFER);
		MusicOutputBuffer.RingBuffer_SetSize(FRAMES_PER_BUFFER);

		Music.openFile("Taco.raw");
		Music.setUpBuffer(&MusicOutputBuffer);

		Noise.openFile("NoiseT.raw");
		Noise.setUpBuffer(&NoiseInputBuffer);

		MusicNoise.openFile("Taco+n.raw");
		MusicNoise.setUpBuffer(&ErrorInputBuffer);

		AudioOutput.setUpBuffer(&MusicOutputBuffer, &newMusicSampleAvailable);

		//RLMS_Algorithm.setLambda(Lambda);
		//RLMS_Algorithm.setNumOfTaps(M);

		updateInputBuffers_Func = std::bind(&ANC_System::updateInputBuffers, this);
		updateInputBuffers_Thread = Wrapper::ThreadWrapper(updateInputBuffers_Func);		

		processDataWithRLMS_Func = std::bind(&ANC_System::processDataWithRLMS, this);
		processDataWithRLMS_Thread = Wrapper::ThreadWrapper(processDataWithRLMS_Func);	

		updateOutputBuffer_Func = std::bind(&ANC_System::updateOutputBuffer, this);
		updateOutputBuffer_Thread = Wrapper::ThreadWrapper(updateOutputBuffer_Func);

		drawNLMSData_Func = std::bind(&ANC_System::drawNLMSData, this);
		drawNLMSData_Thread = Wrapper::ThreadWrapper(drawNLMSData_Func);

	/*	updateInputBuffers_Thread .detach();   
		processDataWithRLMS_Thread.detach();
		updateOutputBuffer_Thread .detach();
		drawNLMSData_Thread		  .detach();*/

		if (paInit.result() != paNoError) {
			fprintf(stderr, "An error occured while using the portaudio stream\n");
			fprintf(stderr, "Error number: %d\n", paInit.result());
			fprintf(stderr, "Error message: %s\n", Pa_GetErrorText(paInit.result()));
		}

		if (AudioOutput.open(Pa_GetDefaultOutputDevice())) {
			if (AudioOutput.start()) {
#if DEBUG
				cout << "AudioOutput Started working" << endl;
#endif // DEBUG
			}
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Parametrized constructor of ANC_System class
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	ANC_System::ANC_System(float lambda, int m, int bufferSize) //: Lambda(lambda), M(m), BufferSize(bufferSize)
	{
		/*NoiseInputBuffer.RingBuffer_SetSize(BufferSize);
		ErrorInputBuffer.RingBuffer_SetSize(BufferSize);
		MusicOutputBuffer.RingBuffer_SetSize(BufferSize);

		RLMS_Algorithm.setLambda(Lambda);
		RLMS_Algorithm.setNumOfTaps(M);*/

		//updateInputBuffers_Thread = thread{ &ANC_System::updateInputBuffers_Func,this };
		//processDataWithRLMS_Thread = thread{ &ANC_System::processDataWithRLMS_Func,this };
		//updateOutputBuffer_Thread = thread{ &ANC_System::updateOutputBuffer_Func,this };
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Destructor of ANC_System class
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	ANC_System::~ANC_System()
	{
		StopThreads.store(1);
		AudioOutput.stop();
		AudioOutput.close();		
		/*if(updateInputBuffers_Thread->joinable())
			updateInputBuffers_Thread->join();
		delete updateInputBuffers_Thread;*/
		//if (processDataWithRLMS_Thread.joinable())
		//	processDataWithRLMS_Thread.join();
		//if (updateOutputBuffer_Thread.joinable())
		//	updateOutputBuffer_Thread .join();
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief updateInputBuffers function of ANC_System class to update error and destiny sound buffers.
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	void ANC_System::updateInputBuffers()
	{		
		//std::thread t([&] {
			auto period = std::chrono::milliseconds(5);

			while (!StopThreads) {
				//std::this_thread::sleep_for(period);

				if (!NoiseInputBuffer.RingBuffer_IsFull()) {
					Noise.updateBuffer();
					newErrorSampleAvailable.store(1);
					//d = ErrorInputBuffer.RingBuffer_GetBufferAsVec();
				}
				if (!ErrorInputBuffer.RingBuffer_IsFull()) {
					MusicNoise.updateBuffer();
					//newNoiseSampleAvailable.store(1);
					//x = NoiseInputBuffer.RingBuffer_GetBufferAsVec();
				}
			}
		//});
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief processDataWithRLMS function of ANC_System class to run RLMS function to process input data
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	void ANC_System::processDataWithRLMS()
	{
		//std::thread t([&] {
			auto period = std::chrono::milliseconds(5);

			while (!StopThreads) {
				//std::this_thread::sleep_for(period);

				if (NoiseInputBuffer.RingBuffer_IsFull() &&
					ErrorInputBuffer.RingBuffer_IsFull())
				{
					/*NLMS_Algorithm.updateNLMS(NoiseInputBuffer.RingBuffer_GetBufferAsVec(),
											  ErrorInputBuffer.RingBuffer_GetBufferAsVec());*/	
					//x = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);
					//d = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);									


					//NLMS_Algorithm.updateNLMS(x,d);
												

					/*if (MusicOutputBuffer.RingBuffer_IsEmpty()) {
						MusicOutputBuffer.RingBuffer_PutVecIntoBuffer(NLMS_Algorithm.getErrorVec());
					}*/

					/*NoiseInputBuffer.RingBuffer_Clear();
					ErrorInputBuffer.RingBuffer_Clear();*/

				}				
			}
		//});
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief updateOutputBuffer function of ANC_System class to update output buffer with processed data and music
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	void ANC_System::updateOutputBuffer()
	{
		//std::thread t([&] {
			//auto timePoint = std::chrono::steady_clock::now();
			//auto period = static_cast<std::chrono::seconds>(FRAMES_PER_BUFFER / SAMPLE_RATE * 1000 / 2);
			//const int time = FRAMES_PER_BUFFER / SAMPLE_RATE * 1000 / 2;
			//auto period = std::chrono::milliseconds(time);
			auto period = std::chrono::milliseconds(11);

			while (!StopThreads) {
				std::this_thread::sleep_for(period);

				if (MusicOutputBuffer.RingBuffer_IsEmpty()) {
					Music.updateBuffer();					
					//MusicOutputBuffer.RingBuffer_PutVecIntoBuffer(NLMS_Algorithm.getErrorVec());
					
				}
			}
		//});
	}
	void ANC_System::drawNLMSData()
	{
		while (!StopThreads) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			NLMS_Algorithm.drawData();
		}
	}
} //ANC