#pragma once
#ifndef _ANC_SYSTEM_H_
#define _ANC_SYSTEM_H_


#include <thread>
#include <atomic>
#include <chrono>

#include "ring_buffer.h"
//#include "RLMS.h"
#include "NLMS.h"
#include "ThreadWrapper.h"
#include "AudioInterface.h"
#include "ScopedPaHandler.h"
#include "AudioStream.h"

#ifndef DEBUG
#define DEBUG 0
#endif // DEBUG

#ifndef SAMPLE_RATE
#define SAMPLE_RATE   (44100)
#endif //SAMPLE_RATE

#ifndef FRAMES_PER_BUFFER
#define FRAMES_PER_BUFFER  (2048)
#endif //FRAMES_PER_BUFFER

#ifndef TIME_DEBUG
#define TIME_DEBUG 0
#endif // TIME_DEBUG

#if TIME_DEBUG
#endif

using std::cout;
using std::endl;

namespace ANC {

	class ANC_System
	{		
		int M;
		float Lambda;

		arma::vec x;
		arma::vec d;
		
		std::atomic<bool> StopThreads = 0;
		std::atomic<bool> newErrorSampleAvailable = 0;
		std::atomic<bool> newNoiseSampleAvailable = 0;
		std::atomic<int> newMusicSampleAvailable = 0;
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

		std::function<void()> updateInputBuffers_Func;
		std::function<void()> processDataWithRLMS_Func;
		std::function<void()> updateOutputBuffer_Func;
		std::function<void()> drawNLMSData_Func;

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
	};
}

#endif // !_ANC_SYSTEM_H_