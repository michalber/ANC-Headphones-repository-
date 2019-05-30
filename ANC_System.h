#pragma once
#ifndef _ANC_SYSTEM_H_
#define _ANC_SYSTEM_H_


#include <thread>
#include <future>
#include <atomic>
#include <chrono>
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
		bool t = true;
		bool f = false;

		int M;
		float Lambda;

		arma::vec x;
		arma::vec d;
		
		std::atomic<bool> StopThreads = 0;

		std::atomic_bool newErrorSampleAvailable {false};
		std::atomic_bool newNoiseSampleAvailable {false};
		std::atomic<int> newMusicSampleAvailable;
		std::atomic_bool newNoiseVectorAvailable {false};
		std::atomic_bool newErrorVectorAvailable {false};

		std::atomic<bool> speedUpANCSampling = 0;
				
		//RLMS::RLMS RLMS_Algorithm;
		Adaptive::NLMS NLMS_Algorithm;

		boost::circular_buffer<float> NIB;
		boost::circular_buffer<float> EIB;
		boost::circular_buffer<float> MOB;	
		boost::circular_buffer<float> FIN;

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
		void putVecIntoCircBuffer(boost::circular_buffer<float>*, arma::vec);
	};
}

#endif // !_ANC_SYSTEM_H_