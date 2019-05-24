#pragma once
#ifndef _ANC_SYSTEM_H_
#define _ANC_SYSTEM_H_


#include <thread>
#include <future>
#include <atomic>
#include <chrono>

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
		int M;
		float Lambda;

		const double microsPerClkTic{ 1.0E6 * std::chrono::system_clock::period::num / std::chrono::system_clock::period::den };
		const std::chrono::microseconds intervalPeriodMillis{ 10 };

		arma::vec x;
		arma::vec d;
		
		std::atomic<bool> StopThreads = 0;
		std::atomic<bool> newErrorSampleAvailable = 0;
		std::atomic<bool> newNoiseSampleAvailable = 0;
		std::atomic<int> newMusicSampleAvailable = 0;
		std::atomic<bool> newNoiseVectorAvailable = 0;
		std::atomic<bool> newErrorVectorAvailable = 0;
		std::atomic<bool> speedUpANCSampling = 0;
				
		//RLMS::RLMS RLMS_Algorithm;
		Adaptive::NLMS NLMS_Algorithm;
		RingBuffer::RingBuffer<float> NoiseInputBuffer;
		RingBuffer::RingBuffer<float> ErrorInputBuffer;
		RingBuffer::RingBuffer<float> MusicOutputBuffer;		

		Wrapper::ThreadWrapper updateInputBuffers_Thread;
		Wrapper::ThreadWrapper processDataWithRLMS_Thread;
		Wrapper::ThreadWrapper updateOutputBuffer_Thread;
		Wrapper::ThreadWrapper drawNLMSData_Thread;
		Wrapper::ThreadWrapper loadNewNoiseVector_Thread;
		Wrapper::ThreadWrapper loadNewErrorVector_Thread;

		std::function<void()> updateInputBuffers_Func;
		std::function<void()> processDataWithRLMS_Func;
		std::function<void()> updateOutputBuffer_Func;
		std::function<void()> drawNLMSData_Func;
		std::function<void()> loadNewNoiseVector_Func;
		std::function<void()> loadNewErrorVector_Func;

		AI::AudioInterface AudioOutput;
		Handler::ScopedPaHandler paInit;

		AS::AudioStream Music;
		AS::AudioStream Noise;
		AS::AudioStream MusicNoise;

	public:	
		ANC_System();
		ANC_System(float,int,int);
		~ANC_System();

	private:		
		void updateInputBuffers();
		void processDataWithRLMS();
		void updateOutputBuffer();	
		void drawNLMSData();
		void loadNewNoiseVector();
		void loadNewErrorVector();
	};
}

#endif // !_ANC_SYSTEM_H_