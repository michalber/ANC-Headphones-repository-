#pragma once
#ifndef _ANC_SYSTEM_H_
#define _ANC_SYSTEM_H_


#include <thread>
#include <future>
#include <atomic>
#include <chrono>
#include <windows.h>
#include <mutex>
#include <condition_variable>
#include <boost/circular_buffer.hpp>

#include "config.h"
#include "ring_buffer.h"
//#include "RLMS.h"
#include "NLMS.h"
#include "ThreadWrapper.h"
#include "AudioInterface.h"
#include "ScopedPaHandler.h"
#include "AudioStream.h"


using std::cout;
using std::endl;

namespace ANC {

	class ANC_System
	{		
		std::mutex mut;

		int M;
		float Lambda;

		arma::vec x;
		arma::vec d;
		
		volatile std::atomic<bool> StopThreads = 0;

		bool newErrorSampleAvailable{ false };
		bool newNoiseSampleAvailable{ false };
		std::atomic<int> newMusicSampleAvailable;
		bool newNoiseVectorAvailable{ false };
		bool newErrorVectorAvailable{ false };
		
		bool updateReady{ false };
				
		Adaptive::NLMS NLMS_Algorithm;

		boost::circular_buffer<float> NIB;	//Noise Input Buffer
		boost::circular_buffer<float> EIB;	//Error Input Buffer
		boost::circular_buffer<float> MOB;	//Music Output Buffer
		boost::circular_buffer<float> NOB;	//NLMS Output Buffer
		boost::lockfree::spsc_queue<float> *outQueue;

		AI::AudioInterface AudioOutput;
		Handler::ScopedPaHandler paInit;

		AS::AudioStream Music;
		AS::AudioStream Noise;
		AS::AudioStream FilteredNoise;
		AS::AudioStream MusicNoise;

	public:	
		ANC_System();
		ANC_System(float,int,int);
		~ANC_System();

	private:		
		void updateNoiseBuffer();
		void updateErrorBuffer();
		void processDataWithRLMS();
		void updateOutputBuffer();	
		void drawNLMSData();
		void loadNewNoiseVector();
		void loadNewErrorVector();

		arma::vec getCircBufferAsVec(boost::circular_buffer<float>*);
		std::vector<float> getCircBufferAsVector(boost::circular_buffer<float>*);
		void putVecIntoCircBuffer(boost::circular_buffer<float>*, arma::vec);
		void putVecIntoCircBuffer(boost::circular_buffer<float>*, std::vector<float>);
	};
}

#endif // !_ANC_SYSTEM_H_