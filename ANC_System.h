#pragma once
#ifndef _ANC_SYSTEM_H_
#define _ANC_SYSTEM_H_

/*
	- GET SAMPLE FROM I2S MICROPHONES EVERY 1/SAMPLING_FREQ TIME
		- WIRINGPI piThread instead of std::thread + nanosleep/microsleep
	- PROCESS IT WITH NLMS AND ADD IT TO THE BUFFER
	
	- EVERY FRAMES_PER_BUFFER/SAMPLING_FREQ DATA BUFFER IS SEND TO AUDIO DEVICE

*/
#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h>  
#include <pthread.h> 
#include <time.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <liquid/liquid.h>
#include <math.h>

#include <sys/time.h>
#include <sys/timerfd.h>
//#include <boost/circular_buffer.hpp>

#include "config.h"
#include "NLMS.h"
#include "AudioInterface.h"
#include "ScopedPaHandler.h"
#include "AudioStream.h"
#include "FullDoubleBuffer.h"


#define PERIOD_OF_FRAME (int)(floor(FRAMES_PER_BUFFER / SAMPLE_RATE * 1000000))
#define LIMITER(x) (float)(x > 1.0f ? 1.0f : x);
//-------------------------------------------------------------------
using std::cout;
using std::endl;
//-------------------------------------------------------------------
struct periodic_info {
	int timer_fd;
	unsigned long long wakeups_missed;
};

static void wait_period(struct periodic_info *info);
static int make_periodic(unsigned int period, struct periodic_info *info);

//-------------------------------------------------------------------

static std::atomic_bool runTasks{ false };
static bool newData { false };

//Float circular buffers
static cbufferf outputCirBuffer;
static cbufferf NLMSCirBuffer;
static cbufferf inputNoiseDataCirBuffer;
static cbufferf inputErrorDataCirBuffer;
static cbufferf inputMusicDataCirBuffer;

static AudioBuffers::FullDoubleBuffer DoubleBuffers;

static float nlmsError[FRAMES_PER_BUFFER] = { 0 };
static float inNoiseBuffer[2 * BUFFER_SIZE] = { 0 };
static float inErrorBuffer[2 * BUFFER_SIZE] = { 0 };
static float outBuffer[BUFFER_SIZE] = { 0 };
static float NLMSBuffer[BUFFER_SIZE] = { 0 };
static float inMusicDataBuffer[BUFFER_SIZE] = { 0 };
//Filtering classes
static Adaptive::NLMS NLMS_Algorithm;

//Threads
static pthread_t NLMS_thread;
static pthread_t GetMusicData_thread;
static pthread_t GetNoiseData_thread;
static pthread_t GetErrorData_thread;

static int thread_1_count;
static int thread_2_count;
static int thread_3_count;
static int thread_4_count;

//Audio interface 
static AI::AudioInterface AudioOutput;
static Handler::ScopedPaHandler paInit;

//Audio streaming classes
static AS::AudioStream MusicStream;
static AS::AudioStream NoiseStream;
static AS::AudioStream MusicNoiseStream;

//-------------------------------------------------------------------
void processDataNLMSCallback();
void getNewMusicSamplesCallback();
void getNewErrorSamplesCallback();
void getNewNoiseSamplesCallback();
void *drawNLMSDataCallback();


void init_ANC_System();
//ANC_System(int, float, float);
void destroy_ANC_System();

#endif // !_ANC_SYSTEM_H_