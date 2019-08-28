#include "ANC_System.h"

int make_periodic(unsigned int period, struct periodic_info *info)
{
	int ret;
	unsigned int ns;
	unsigned int sec;
	int fd;
	struct itimerspec itval;

	/* Create the timer */
	fd = timerfd_create(CLOCK_REALTIME, 0);
	info->wakeups_missed = 0;
	info->timer_fd = fd;
	if (fd == -1)
		return fd;

	/* Make the timer periodic */
	sec = period / 1000000;
	ns = (period - (sec * 1000000)) * 1000;
	itval.it_interval.tv_sec = sec;
	itval.it_interval.tv_nsec = ns;
	itval.it_value.tv_sec = sec;
	itval.it_value.tv_nsec = ns;
	ret = timerfd_settime(fd, 0, &itval, NULL);
	return ret;
}

void wait_period(struct periodic_info *info)
{
	unsigned long long missed;
	int ret;

	/* Wait for the next timer event. If we have missed any the
	   number is written to "missed" */
	ret = read(info->timer_fd, &missed, sizeof(missed));
	if (ret == -1) {
		perror("read timer");
		return;
	}

	info->wakeups_missed += missed;
}


//--------------------------------------------------------------------------------------------------------------------
/**
	@brief Default constructor of ANC_System class
	@author	Michal Berdzik	
	@version 0.1 05-07-2019
*/
void init_ANC_System()
{
	wiringPiSetupSys();

	//Set up buffers size 
	outputCirBuffer			= cbufferf_create_max(BUFFER_SIZE, FRAMES_PER_BUFFER);
	NLMSCirBuffer			= cbufferf_create_max(BUFFER_SIZE, FRAMES_PER_BUFFER);
	inputNoiseDataCirBuffer = cbufferf_create_max(BUFFER_SIZE, FRAMES_PER_BUFFER);
	inputErrorDataCirBuffer = cbufferf_create_max(BUFFER_SIZE, FRAMES_PER_BUFFER);
	inputMusicDataCirBuffer = cbufferf_create_max(BUFFER_SIZE, FRAMES_PER_BUFFER);	

	DoubleBuffers.Init();

	//Pass buffers to other classes
	MusicStream.setUpBuffer(&inputMusicDataCirBuffer, inMusicDataBuffer);
	NoiseStream.setUpBuffer(&inputNoiseDataCirBuffer, inMusicDataBuffer);
	MusicNoiseStream.setUpBuffer(&inputErrorDataCirBuffer, inMusicDataBuffer);

	//Open music files into streams
	if (!MusicStream.openFile(std::string(DATA_PATH) + "Taco.raw")) {			
		exit(0);
	}
	if (!NoiseStream.openFile(std::string(DATA_PATH) + "NoiseT.raw")) {
		exit(0);
	}
	if (!MusicNoiseStream.openFile(std::string(DATA_PATH) + "Taco+n.raw")) {
		exit(0);
	}

	//Initlialize thread and their functions
	//int x = piThreadCreate(processDataPiThreadCallback);
	//if (x != 0)
	//	printf("it didn't startn");
	
//	NLMS_Algorithm.arm_lms_norm_init_f32(NUM_OF_TAPS, 0.5, FRAMES_PER_BUFFER);

	//Initlialize audio output
	AudioOutput.setUpBuffer(&outputCirBuffer,
							&inputNoiseDataCirBuffer,
							&inputErrorDataCirBuffer,
							&DoubleBuffers,
							&inNoiseBuffer[0],
							&outBuffer[0],
							&newData);

	if(paInit.result() != paNoError) {
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

	//Run threads
	runTasks.store(true);

	//pthread_create(&GetMusicData_thread, NULL, getNewMusicSamplesCallback, NULL);
	//pthread_create(&NLMS_thread, NULL, processDataNLMSCallback, NULL);
	//pthread_create(&GetErrorData_thread, NULL, getNewErrorSamplesCallback, NULL);
	//pthread_create(&GetNoiseData_thread, NULL, getNewNoiseSamplesCallback, NULL);	

//	getNewMusicSamplesCallback();	
//	getNewNoiseSamplesCallback();	
	processDataNLMSCallback();
//	getNewErrorSamplesCallback();
}

//--------------------------------------------------------------------------------------------------------------------
/**
	@brief Destructor of ANC_System class
	@author	Michal Berdzik
	@version 0.1 05-07-2019
*/
void destroy_ANC_System()
{
	runTasks.store(false);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	//pthread_join(GetData_thread, NULL);
	
	cbufferf_destroy(outputCirBuffer);
	cbufferf_destroy(inputNoiseDataCirBuffer);
	cbufferf_destroy(inputErrorDataCirBuffer);
	cbufferf_destroy(NLMSCirBuffer);
}
//--------------------------------------------------------------------------------------------------------------------
/**
	@brief processDataNLMSCallback method of ANC_System class
	@author	Michal Berdzik
	@version 0.1 05-07-2019
*/
void processDataNLMSCallback()
{
	static AudioBuffers::StereoBuffer *tempBuff;
	static float *_noise;
	static float *_error;
	static float _result[BUFFER_SIZE];
	static unsigned int numReaded;
	static int i = 0;

	struct periodic_info info;
	//make_periodic(20, &info);
	std::thread t([&] {
	while (runTasks.load()) {
//		auto start = std::chrono::high_resolution_clock::now();
		//thread_1_count++;
		//wait_period(&info);
		
		//Get samples from input buffers

		if (newData) {
			//memmove(&NLMSBuffer[0], &NLMSBuffer[FRAMES_PER_BUFFER], (BUFFER_SIZE - FRAMES_PER_BUFFER) * sizeof(float));

			//buffer of size BUFFER_SIZE, which contains 2 channels -> one channel's length is BUFFER_SIZE / 2 
			tempBuff = DoubleBuffers.TakeMicBuff();
		
			//NLMS_Algorithm.processNLMS(&inNoiseBuffer[0], &inNoiseBuffer[0]);
			NLMS_Algorithm.processNLMS(tempBuff->Mid, tempBuff->Mid);

			_error = NLMS_Algorithm.getOutputSignal();
			for (int i = 0; i < BUFFER_SIZE / 2; i++) {
				_result[i * 2] = -_error[i];
				_result[i * 2 + 1] = -_error[i];
			}

//			for (int i = 0; i < 2 * FRAMES_PER_BUFFER; i++) {
//				_result[i] = tempBuff->Mid[i];
//			}
//			memcpy(_result, tempBuff->Mid, FRAMES_PER_BUFFER * sizeof(float));
			
		  

			DoubleBuffers.GiveOutBuff(_result, _result);
			//memcpy(&NLMSBuffer[BUFFER_SIZE - FRAMES_PER_BUFFER], NLMS_Algorithm.getOutputSignal(), FRAMES_PER_BUFFER * sizeof(float));
			//NLMS_Algorithm.arm_lms_norm_f32(
			//	&inNoiseBuffer[0],								/* Input signal */
			//	&inErrorBuffer[0],								/* Reference Signal */
			//	&NLMSBuffer[BUFFER_SIZE - FRAMES_PER_BUFFER],   /* Converged Signal */
			//	&nlmsError[0],										/* Error Signal, this will become small as the signal converges */
			//	FRAMES_PER_BUFFER);								/* BlockSize */
			newData = 0;
		}

		//Add negative result to music sample
		//*_result = -(*NLMS_Algorithm.getOutputSignal());
		//cbufferf_push(NLMSCirBuffer, *_result);
				
		
		
//		auto end = std::chrono::high_resolution_clock::now();
//		auto elapsed = end - start;
//		auto timeToWait = std::chrono::microseconds(20);
//		auto timeToWait = std::chrono::microseconds(PERIOD_OF_FRAME) - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
//		
//		if (timeToWait > std::chrono::microseconds::zero()) {
//			//std::this_thread::sleep_for(timeToWait);
//			delayMicroseconds(timeToWait.count());
//			//std::this_thread::sleep_until(start + timeToWait);
//		}
	}
	});
	t.detach();
}
//--------------------------------------------------------------------------------------------------------------------
/**
	@brief getNewSamples method of ANC_System class
	@author	Michal Berdzik
	@version 0.1 05-07-2019
*/
void getNewMusicSamplesCallback()
{
	static float _music = 0;
	static float _result = 0;

	//struct periodic_info info;
	//make_periodic(PERIOD_OF_FRAME, &info);

	std::thread t([&] {
	while (runTasks.load()) {
		auto start = std::chrono::high_resolution_clock::now();
		//thread_2_count++;
		//wait_period(&info);

		//Update buffers
		memmove(&inMusicDataBuffer[0], &inMusicDataBuffer[FRAMES_PER_BUFFER], (BUFFER_SIZE - FRAMES_PER_BUFFER) * sizeof(float));
		MusicStream.updateBuffer(FRAMES_PER_BUFFER);		


		////Get new music sample						
		//unsigned int _numReaded;


		//if (cbufferf_space_available(outputCirBuffer) >= 2) {
		//	//if (cbufferf_max_read(inputNoiseDataCirBuffer) > 2) {
		//	//	cbufferf_pop(inputNoiseDataCirBuffer, &_music);
		//	//}
		//	if (cbufferf_max_read(inputMusicDataCirBuffer) >= 2) {				
		//		for (int i = 0; i < 2; i++) {
		//			cbufferf_pop(inputMusicDataCirBuffer, &_music);

		//			//if (cbufferf_max_read(NLMSCirBuffer) >= 1) {
		//			//	cbufferf_pop(NLMSCirBuffer, _result);
		//			//}
		//			_result = _music;
		//			cbufferf_push(outputCirBuffer, _result);
		//			_music = 0;
		//		}
		//	}
		//}

		//cbufferf_debug_print(inputMusicDataCirBuffer);

		auto end = std::chrono::high_resolution_clock::now();
		auto elapsed = end - start;
		//auto timeToWait = std::chrono::microseconds(20);
		auto timeToWait = std::chrono::microseconds(PERIOD_OF_FRAME) - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
		
		if (timeToWait > std::chrono::microseconds::zero()) {
			//std::this_thread::sleep_for(timeToWait);
			delayMicroseconds(timeToWait.count());
			//std::this_thread::sleep_until(start + timeToWait);
		}
	}
	});
	t.detach();
}

void getNewErrorSamplesCallback()
{
	static float *buff = NULL;
	static float music = 0;
	static unsigned int numReaded = 0;

	struct periodic_info info;
	//make_periodic(20, &info);

	std::thread t([&] {
	while (runTasks.load()) {

		//thread_3_count++;
		//wait_period(&info);
		auto start = std::chrono::high_resolution_clock::now();

		
		//if (cbufferf_space_available(outputCirBuffer) > FRAMES_PER_BUFFER) {
		//	//cbufferf_read(inputMusicDataCirBuffer, FRAMES_PER_BUFFER, &buff, &numReaded);
		//	cbufferf_read(inputNoiseDataCirBuffer, FRAMES_PER_BUFFER, &buff, &numReaded);
		//	cbufferf_write(outputCirBuffer, buff, numReaded);
		//	//cbufferf_release(inputMusicDataCirBuffer, numReaded);
		//	cbufferf_release(inputNoiseDataCirBuffer, numReaded);
		//}

		memmove(&outBuffer[0], &outBuffer[FRAMES_PER_BUFFER], (BUFFER_SIZE - FRAMES_PER_BUFFER) * sizeof(float));
		//memcpy(&outBuffer[BUFFER_SIZE - FRAMES_PER_BUFFER], &inMusicDataBuffer[0], FRAMES_PER_BUFFER * sizeof(float));
		for (int i = 0; i < FRAMES_PER_BUFFER; i++) {			
			outBuffer[BUFFER_SIZE - FRAMES_PER_BUFFER + i] = LIMITER(/*inMusicDataBuffer[i] +*/ /*inMicsBuffer[i] +*/ -NLMSBuffer[i]);
		}


		//cbufferf_debug_print(inputMusicDataCirBuffer);

		auto end = std::chrono::high_resolution_clock::now();
		auto elapsed = end - start;
		//auto timeToWait = std::chrono::microseconds(20);
		auto timeToWait = std::chrono::microseconds(PERIOD_OF_FRAME) - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
		
		if (timeToWait > std::chrono::microseconds::zero()) {
			//std::this_thread::sleep_for(timeToWait);
			delayMicroseconds(timeToWait.count());
			//std::this_thread::sleep_until(start + timeToWait);
		}
	}
	});
	t.detach();
}

void getNewNoiseSamplesCallback()
{
	struct periodic_info info;
	make_periodic(PERIOD_OF_FRAME, &info);

	while (runTasks.load()) {

		//thread_4_count++;
		//wait_period(&info);

		//Update buffers
		NoiseStream.updateBufferOnce();

		//cbufferf_debug_print(inputMusicDataCirBuffer);

		//auto end = std::chrono::high_resolution_clock::now();
		//auto elapsed = end - start;
		////auto timeToWait = std::chrono::microseconds(20);
		//auto timeToWait = std::chrono::microseconds(21) - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
		//
		//if (timeToWait > std::chrono::microseconds::zero()) {
		//	//std::this_thread::sleep_for(timeToWait);
		//	delayMicroseconds(timeToWait.count());
		//	//std::this_thread::sleep_until(start + timeToWait);
		//}
	}
}