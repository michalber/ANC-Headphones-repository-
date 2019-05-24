#include "../ANC-Headphones-repository-/ANC_System.h"

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

		Music.openFile("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\Taco.raw");
		Music.setUpBuffer(&MusicOutputBuffer);

		Noise.openFile("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\NoiseT.raw");
		Noise.setUpBuffer(&NoiseInputBuffer);

		MusicNoise.openFile("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\Taco+n.raw");
		MusicNoise.setUpBuffer(&ErrorInputBuffer);

		AudioOutput.setUpBuffer(&MusicOutputBuffer, &newMusicSampleAvailable);

		//RLMS_Algorithm.setLambda(Lambda);
		//RLMS_Algorithm.setNumOfTaps(M);

		/*updateInputBuffers_Func = std::bind(&ANC_System::updateInputBuffers, this);
		updateInputBuffers_Thread = Wrapper::ThreadWrapper(updateInputBuffers_Func);		

		processDataWithRLMS_Func = std::bind(&ANC_System::processDataWithRLMS, this);
		processDataWithRLMS_Thread = Wrapper::ThreadWrapper(processDataWithRLMS_Func);	

		updateOutputBuffer_Func = std::bind(&ANC_System::updateOutputBuffer, this);
		updateOutputBuffer_Thread = Wrapper::ThreadWrapper(updateOutputBuffer_Func);

#if PLOT_DATA
		drawNLMSData_Func = std::bind(&ANC_System::drawNLMSData, this);
		drawNLMSData_Thread = Wrapper::ThreadWrapper(drawNLMSData_Func);
#endif
		loadNewNoiseVector_Func = std::bind(&ANC_System::loadNewNoiseVector, this);
		loadNewNoiseVector_Thread = Wrapper::ThreadWrapper(loadNewNoiseVector_Func);

		loadNewErrorVector_Func = std::bind(&ANC_System::loadNewErrorVector, this);
		loadNewErrorVector_Thread = Wrapper::ThreadWrapper(loadNewErrorVector_Func);*/

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


		updateInputBuffers();
		//loadNewNoiseVector();
		//loadNewErrorVector();
		//processDataWithRLMS();
		updateOutputBuffer();
		drawNLMSData();	
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
		//Initialize the chrono timepoint & duration objects we'll be
		//using over & over inside our sleep loop
		std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
		std::chrono::system_clock::time_point nextStartTime{ currentStartTime };

		std::chrono::system_clock::duration wakeupError;
		std::thread t([&] {
		while (!StopThreads) {			
			nextStartTime = currentStartTime + std::chrono::milliseconds(1);
			std::this_thread::sleep_until(nextStartTime);

			if (!NoiseInputBuffer.RingBuffer_IsFull()) {
				Noise.updateBuffer();
				newErrorSampleAvailable.store(true);				
				//auto start = std::chrono::system_clock::now();
				//x = NoiseInputBuffer.RingBuffer_GetBufferAsVec();
				//auto stop = std::chrono::system_clock::now();
				//cout << std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count() << endl;
			}
			if (!ErrorInputBuffer.RingBuffer_IsFull()) {
				MusicNoise.updateBuffer();
				newNoiseSampleAvailable.store(true);				
				//d = ErrorInputBuffer.RingBuffer_GetBufferAsVec();					
			}
		}
		});
		t.detach();
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
		std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
		std::chrono::system_clock::time_point nextStartTime{ currentStartTime };
		std::chrono::system_clock::duration wakeupError;

		std::thread t([&] {
		while (!StopThreads) {
			nextStartTime = currentStartTime + std::chrono::milliseconds(1);
			std::this_thread::sleep_until(nextStartTime);

			if (newNoiseVectorAvailable.load() == true &&
				newErrorVectorAvailable.load() == true)
			{
				/*NLMS_Algorithm.updateNLMS(NoiseInputBuffer.RingBuffer_GetBufferAsVec(),
										  ErrorInputBuffer.RingBuffer_GetBufferAsVec());*/
										  //x = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);
										  //d = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);									

				//NLMS_Algorithm.updateNLMS(x, d);


				newNoiseVectorAvailable.store(false);
				newErrorVectorAvailable.store(false);

				/*if (MusicOutputBuffer.RingBuffer_IsEmpty()) {
					MusicOutputBuffer.RingBuffer_PutVecIntoBuffer(NLMS_Algorithm.getErrorVec());
				}*/

				NoiseInputBuffer.RingBuffer_Clear();
				ErrorInputBuffer.RingBuffer_Clear();

			}
		}
		});
		t.detach();
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
		std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
		std::chrono::system_clock::time_point nextStartTime{ currentStartTime };
		std::chrono::system_clock::duration wakeupError;

		std::thread t([&] {
		while (!StopThreads) {
			nextStartTime = currentStartTime + std::chrono::milliseconds(1);
			std::this_thread::sleep_until(nextStartTime);

			if (!MusicOutputBuffer.RingBuffer_IsFull()) {
				Music.updateBuffer();
				//MusicOutputBuffer.RingBuffer_PutVecIntoBuffer(NLMS_Algorithm.getOutVec());

			}
		}
		});
		t.detach();
	}
	void ANC_System::drawNLMSData()
	{
		std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
		std::chrono::system_clock::time_point nextStartTime{ currentStartTime };
		std::chrono::system_clock::duration wakeupError;

		std::thread t([&] {
			while (!StopThreads) {
				nextStartTime = currentStartTime + std::chrono::milliseconds(500);
				std::this_thread::sleep_until(nextStartTime);

				NLMS_Algorithm.drawData();
			}
		});
		t.detach();
	}
	void ANC_System::loadNewNoiseVector()
	{
		std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
		std::chrono::system_clock::time_point nextStartTime{ currentStartTime };
		std::chrono::system_clock::duration wakeupError;

		std::thread t([&] {
			while (!StopThreads) {
				nextStartTime = currentStartTime + std::chrono::milliseconds(1);
				std::this_thread::sleep_until(nextStartTime);

				if (newNoiseSampleAvailable.load() == true) {
					//d = NoiseInputBuffer.RingBuffer_GetBufferAsVec();
					//NoiseInputBuffer.RingBuffer_Clear();
					//d = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);

					newNoiseVectorAvailable.store(true);
					newNoiseSampleAvailable.store(false);
				}
			}
		});
		t.detach();
	}
	void ANC_System::loadNewErrorVector()
	{
		std::chrono::system_clock::time_point currentStartTime{ std::chrono::system_clock::now() };
		std::chrono::system_clock::time_point nextStartTime{ currentStartTime };
		std::chrono::system_clock::duration wakeupError;

		std::thread t([&] {
			while (!StopThreads) {
				nextStartTime = currentStartTime + std::chrono::milliseconds(1);
				std::this_thread::sleep_until(nextStartTime);

				if (newErrorSampleAvailable.load() == true) {
					//x = ErrorInputBuffer.RingBuffer_GetBufferAsVec();
					//ErrorInputBuffer.RingBuffer_Clear();
					//x = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);

					newErrorVectorAvailable.store(true);
					newErrorSampleAvailable.store(false);
				}
			}
		});
		t.detach();
	}
} //ANC