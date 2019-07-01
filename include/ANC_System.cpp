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

		//Set up NLMS algo.
		NLMS_Algorithm.setParameters(300, 0.05, 0.001);
		NLMS_Algorithm.setUpBuffer(&MOB);

		//Set up Circ.Buffers size
		NIB.set_capacity(2 * FRAMES_PER_BUFFER);
		EIB.set_capacity(2 * FRAMES_PER_BUFFER);
		MOB.set_capacity(2 * FRAMES_PER_BUFFER);
		NOB.set_capacity(2 * FRAMES_PER_BUFFER);

		//create music data streams and assign them to Circ. Buffers
		Music.openFile(std::string(DATA_PATH) + "Taco.raw");
		Music.setUpBuffer(&MOB);

		Noise.openFile(std::string(DATA_PATH) + "NoiseT.raw");
		Noise.setUpBuffer(&NIB);

		MusicNoise.openFile(std::string(DATA_PATH) + "Taco+n.raw");
		MusicNoise.setUpBuffer(&EIB);

		
		//Assign Buffer to output music stream 
		AudioOutput.setUpBuffer(&MOB);

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

		//Launch threads
		updateNoiseBuffer();
		//updateErrorBuffer();
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
				std::this_thread::sleep_for(std::chrono::microseconds(50));
				

				if ((NIB.size() % FRAMES_PER_BUFFER) == 0 &&
					!NIB.full() &&
					newNoiseSampleAvailable == false
					)
				{										
					Noise.updateBuffer();
					newNoiseSampleAvailable = true;
				}		

				if ((EIB.size() % FRAMES_PER_BUFFER) == 0 &&
					!EIB.full() &&
					newErrorSampleAvailable == false
					)
				{
					MusicNoise.updateBuffer();
					newErrorSampleAvailable = true;
				}
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
					newErrorSampleAvailable = true;
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
		std::thread t([&] {
			while (!StopThreads) {
				//std::this_thread::sleep_for(std::chrono::microseconds(1));

				//std::lock_guard<std::mutex> lk(mut);


				if (newNoiseVectorAvailable == true &&
					newErrorVectorAvailable == true) {
				//if(updateReady == true) {

					//x = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);
					//d = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);																								
					
					//NLMS_Algorithm.updateNLMSFilter(d, x);
					NLMS_Algorithm.updateNLMS(d, x);

					newNoiseVectorAvailable = false;
					newErrorVectorAvailable = false;

					updateReady = false;
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
			while (!StopThreads) {
				//nextStartTime = currentStartTime + std::chrono::microseconds(500);				
				//std::this_thread::sleep_until(nextStartTime);
				std::this_thread::sleep_for(std::chrono::microseconds(50));	

				
				if (AudioOutput.getNextVecFlag()) {
					//if ((MOB.size() % FRAMES_PER_BUFFER) == 0 && !MOB.full()) {
						//Music.updateBuffer();
						//putVecIntoCircBuffer(&MOB, getCircBufferAsVector(&NOB));
						putVecIntoCircBuffer(&MOB, NLMS_Algorithm.getOutVec());
					//}
				}
			}
		});
		t.detach();
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief 
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	void ANC_System::drawNLMSData()
	{
		std::thread t([&] {
			while (!StopThreads) {
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				{
					//std::lock_guard<std::mutex> lk(mut);
					//NLMS_Algorithm.drawData(x, d, "x", "d");
					NLMS_Algorithm.drawData();
				}
			}
		});
		t.detach();
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	void ANC_System::loadNewNoiseVector()
	{
		std::thread t([&] {
			while (!StopThreads) {
				std::this_thread::sleep_for(std::chrono::microseconds(50));

				//std::lock_guard<std::mutex> lk(mut);

				if (newNoiseSampleAvailable == true &&
					newNoiseVectorAvailable == false) {

					x.clear();
					x = getCircBufferAsVec(&NIB);

					//x = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);

					newNoiseVectorAvailable = true;
					newNoiseSampleAvailable = false;
				}

				if (newErrorSampleAvailable == true &&
					newErrorVectorAvailable == false) {

					d.clear();
					d = getCircBufferAsVec(&EIB);

					//d = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);

					newErrorVectorAvailable = true;
					newErrorSampleAvailable = false;
				}

				if (newErrorVectorAvailable &&
					newNoiseVectorAvailable) 
				{
					updateReady = true;
				}
				else updateReady = false;
			}
		});
		t.detach();
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	void ANC_System::loadNewErrorVector()
	{
		std::thread t([&] {
			while (!StopThreads) {
				std::this_thread::sleep_for(std::chrono::microseconds(50));

				if (newErrorSampleAvailable == true &&
					newErrorVectorAvailable == false) {

					//d = getCircBufferAsVec(&EIB);
					//ErrorInputBuffer.RingBuffer_Clear();
					d = arma::vec(FRAMES_PER_BUFFER, arma::fill::randn);

					newErrorVectorAvailable = true;
					newErrorSampleAvailable = false;
				}
			}
		});
		t.detach();
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	arma::vec ANC_System::getCircBufferAsVec(boost::circular_buffer<float> *buff)
	{
		/*
		max time: 1ms
		*/
		arma::vec temp(FRAMES_PER_BUFFER, arma::fill::zeros);		
		
		//if (buff->size() >= FRAMES_PER_BUFFER) {
			for (int i = 0; i < FRAMES_PER_BUFFER; i++) {
				temp(i) = buff->back();				
				buff->pop_back();
			}
		//}
		return temp;
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	std::vector<float> ANC_System::getCircBufferAsVector(boost::circular_buffer<float> *buff)
	{
		/*
		max time: 1ms
		*/
		std::vector<float> temp;

		//if (buff->size() >= FRAMES_PER_BUFFER) {
		for (int i = 0; i < FRAMES_PER_BUFFER; i++) {
			if (buff->full()) break;			
			temp.push_back(buff->back());
			buff->pop_back();
		}
		//}
		return temp;
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	void ANC_System::putVecIntoCircBuffer(boost::circular_buffer<float> *buff, arma::vec in)
	{
		for (int i = 0; i < in.size(); i++) {
			if (buff->full()) break;
			buff->push_front(in(i));
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief
		@author	Micha³ Berdzik
		@version 0.0.1 10-04-2019
		@param
		@retval
	*/
	void ANC_System::putVecIntoCircBuffer(boost::circular_buffer<float> *buff, std::vector<float> in)
	{
		for (int i = 0; i < in.size(); i++) {
			if (buff->full()) break;
			buff->push_front(in[i]);
		}
	}
} //ANC