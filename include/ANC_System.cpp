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

		NLMS_Algorithm.setParameters(150, 0.5, 0.001);

		NIB.set_capacity(4 * FRAMES_PER_BUFFER);
		EIB.set_capacity(4 * FRAMES_PER_BUFFER);
		MOB.set_capacity(4 * FRAMES_PER_BUFFER);
		FIN.set_capacity(4 * FRAMES_PER_BUFFER);

		Music.openFile("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\Taco.raw");
		Music.setUpBuffer(&MOB);

		Noise.openFile("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\NoiseT.raw");
		Noise.setUpBuffer(&NIB);

		MusicNoise.openFile("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\Taco+n.raw");
		MusicNoise.setUpBuffer(&EIB);

		FilteredNoise.openFile("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\fnoise.raw");
		FilteredNoise.setUpBuffer(&FIN);

		//AudioOutput.setUpBuffer(&MusicOutputBuffer, &newMusicSampleAvailable);
		AudioOutput.setUpBuffer(&MOB);

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
		updateNoiseBuffer();
		updateErrorBuffer();
		loadNewNoiseVector();
		//loadNewErrorVector();
		processDataWithRLMS();
		updateOutputBuffer();
#if PLOT_DATA
		drawNLMSData();
#endif
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
	void ANC_System::updateNoiseBuffer()
	{
		std::thread t([&] {
			while (!StopThreads) {
				std::this_thread::sleep_for(std::chrono::microseconds(100));
				

				if ((NIB.size() % FRAMES_PER_BUFFER) == 0 &&
					!NIB.full() //&&
					//newErrorSampleAvailable == false
					)
				{										
					Noise.updateBuffer();
					newNoiseSampleAvailable.store(true, std::memory_order_release);
				}				

		
				//if ((FIN.size() % FRAMES_PER_BUFFER) == 0 &&
				//	!FIN.full() //&&
				//	//newNoiseSampleAvailable == false
				//	)
				//{
				//	FilteredNoise.updateBuffer();
				//}
			}
		});
		t.detach();
	}
	void ANC_System::updateErrorBuffer()
	{
		std::thread t([&] {
			while (!StopThreads) {
				std::this_thread::sleep_for(std::chrono::microseconds(100));


				if ((EIB.size() % FRAMES_PER_BUFFER) == 0 &&
					!EIB.full() //&&
					//newNoiseSampleAvailable == false
					)
				{
					MusicNoise.updateBuffer();					
					newErrorSampleAvailable.store(true, std::memory_order_release);
				}

				//if ((FIN.size() % FRAMES_PER_BUFFER) == 0 &&
				//	!FIN.full() //&&
				//	//newNoiseSampleAvailable == false
				//	)
				//{
				//	FilteredNoise.updateBuffer();
				//}
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
		/*
		max time = 4.1ms
		*/
		std::thread t([&] {
			while (!StopThreads) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1));

				//if (newNoiseVectorAvailable.load() == true &&
				//	newErrorVectorAvailable.load() == true)
				//{
				if(newNoiseVectorAvailable.load(std::memory_order_acquire) &&
				   newErrorVectorAvailable.load(std::memory_order_acquire)) {
					/*NLMS_Algorithm.updateNLMS(NoiseInputBuffer.RingBuffer_GetBufferAsVec(),
											  ErrorInputBuffer.RingBuffer_GetBufferAsVec());*/	
					//x = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);
					//d = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);																								

					//NLMS_Algorithm.updateNLMS(x, d);					

					newNoiseVectorAvailable.store(false, std::memory_order_release);
					newErrorVectorAvailable.store(false, std::memory_order_release);
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
		//std::chrono::steady_clock::time_point currentStartTime{ std::chrono::steady_clock::now() };
		//std::chrono::steady_clock::time_point nextStartTime{ currentStartTime };

		std::thread t([&] {	
		//std::async(std::launch::async, [&] {
			while (!StopThreads) {
				//nextStartTime = currentStartTime + std::chrono::microseconds(500);				
				//std::this_thread::sleep_until(nextStartTime);
				std::this_thread::sleep_for(std::chrono::microseconds(1000));				

				if ((MOB.size() % FRAMES_PER_BUFFER) == 0 && !MOB.full()) {
					Music.updateBuffer();
					//putVecIntoCircBuffer(&MOB, NLMS_Algorithm.getOutVec());					
				}
			}
		});
		t.detach();
	}

	void ANC_System::drawNLMSData()
	{
		std::thread t([&] {
			while (!StopThreads) {
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				{
					std::lock_guard<std::mutex> lk(mut);
					NLMS_Algorithm.drawData(x, d, "x", "d");
					//NLMS_Algorithm.drawData();
				}
			}
		});
		t.detach();
	}
	void ANC_System::loadNewNoiseVector()
	{
		std::thread t([&] {
			while (!StopThreads) {
				std::this_thread::sleep_for(std::chrono::microseconds(100));
				{
					std::lock_guard<std::mutex> lk(mut);

					if (newNoiseSampleAvailable.load(std::memory_order_acquire) == true &&
						newNoiseVectorAvailable.load(std::memory_order_acquire) == false) {

						//x = getCircBufferAsVec(&NIB);
						//NoiseInputBuffer.RingBuffer_Clear();
						x = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);

						newNoiseVectorAvailable.store(true, std::memory_order_release);
						newNoiseSampleAvailable.store(false, std::memory_order_release);
					}
					if (newErrorSampleAvailable.load(std::memory_order_acquire) == true &&
						newErrorVectorAvailable.load(std::memory_order_acquire) == false) {

						//d = getCircBufferAsVec(&EIB);
						//ErrorInputBuffer.RingBuffer_Clear();
						d = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);

						newErrorVectorAvailable.store(true, std::memory_order_release);
						newErrorSampleAvailable.store(false, std::memory_order_release);
					}
				}
			}
		});
		t.detach();
	}
	void ANC_System::loadNewErrorVector()
	{
		std::thread t([&] {
			while (!StopThreads) {
				std::this_thread::sleep_for(std::chrono::microseconds(100));

				if (newErrorSampleAvailable.load(std::memory_order_acquire) == true &&
					newErrorVectorAvailable.load(std::memory_order_acquire) == false) {

					//d = getCircBufferAsVec(&EIB);
					//ErrorInputBuffer.RingBuffer_Clear();
					d = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);

					newErrorVectorAvailable.store(true, std::memory_order_release);
					newErrorSampleAvailable.store(false, std::memory_order_release);
				}
			}
		});
		t.detach();
	}


	arma::vec ANC_System::getCircBufferAsVec(boost::circular_buffer<float> *buff)
	{
		/*
		max time: 1ms
		*/
		arma::vec temp(FRAMES_PER_BUFFER, arma::fill::zeros);
		//temp.fill(0);
		
		//if (buff->size() >= FRAMES_PER_BUFFER) {
			for (int i = 0; i < FRAMES_PER_BUFFER; i++) {
				temp(i) = buff->back();
				buff->pop_back();
			}
		//}
		return temp;
	}

	void ANC_System::putVecIntoCircBuffer(boost::circular_buffer<float> *buff, arma::vec in)
	{
		/*
		max time = 2ms
		*/
		for (int i = 0; i < in.size(); i++) {
			buff->push_front(in(i));
		}
	}
} //ANC