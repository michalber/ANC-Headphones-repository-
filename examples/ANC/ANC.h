/*******************************************************************************
 The block below describes the properties of this PIP. A PIP is a short snippet
 of code that can be read by the Projucer and used to generate a JUCE project.

 BEGIN_JUCE_PIP_METADATA

 name:             Active Noise Cancelling
 version:          1.0.0
 vendor:           Michał Berdzik
 website:          
 description:      Perform ANC on collected data

 dependencies:     juce_audio_basics, juce_audio_devices, juce_audio_formats,
                   juce_audio_processors, juce_audio_utils, juce_core,
                   juce_data_structures, juce_events, juce_graphics,
                   juce_gui_basics, juce_gui_extra
 exporters:        vs2017, linux_make

 moduleFlags:      JUCE_STRICT_REFCOUNTEDPOINTER=1

 type:             Component
 mainClass:        ANCInstance

 useLocalCopy:     1

 END_JUCE_PIP_METADATA

*******************************************************************************/

#pragma once

#include "../Assets/DemoUtilities.h"
#include "../Assets/AudioLiveScrollingDisplay.h"
#include "../DemoRunner/Source/SpectrumAnalyser.h"
#include "../DemoRunner/Source/FilterVisualizer.h"

#include "../DemoRunner/Source/ARM_NLMS.h"
#include "../DemoRunner/Source/config.h"
#include "../DemoRunner/Source/pa_ringbuffer.h"

#include <future>
//#include <omp.h>
#include <string.h>

//==============================================================================
/** A simple class that acts as an AudioIODeviceCallback and writes the
	incoming audio data to a WAV file.
*/
class AudioRecorder : public AudioIODeviceCallback
{
public:
	AudioRecorder(AudioThumbnail& thumbnailToUpdate)
		: thumbnail(thumbnailToUpdate)
	{
		backgroundThread.startThread();
	}

	~AudioRecorder() override
	{
		stop();
	}

	//==============================================================================
	void startRecording(const File& file)
	{
		stop();

		if (sampleRate > 0)
		{
			// Create an OutputStream to write to our destination file...
			file.deleteFile();

			if (auto fileStream = std::unique_ptr<FileOutputStream>(file.createOutputStream()))
			{
				// Now create a WAV writer object that writes to our output stream...
				WavAudioFormat wavFormat;

				if (auto writer = wavFormat.createWriterFor(fileStream.get(), sampleRate, 2, 16, {}, 0))
				{
					fileStream.release(); // (passes responsibility for deleting the stream to the writer object that is now using it)

					// Now we'll create one of these helper objects which will act as a FIFO buffer, and will
					// write the data to disk on our background thread.
					threadedWriter.reset(new AudioFormatWriter::ThreadedWriter(writer, backgroundThread, 32768));

					// Reset our recording thumbnail
					thumbnail.reset(writer->getNumChannels(), writer->getSampleRate());
					nextSampleNum = 0;

					// And now, swap over our active writer pointer so that the audio callback will start using it..
					const ScopedLock sl(writerLock);
					activeWriter = threadedWriter.get();
				}
			}
		}
	}

	void stop()
	{
		// First, clear this pointer to stop the audio callback from using our writer object..
		{
			const ScopedLock sl(writerLock);
			activeWriter = nullptr;
		}

		// Now we can delete the writer object. It's done in this order because the deletion could
		// take a little time while remaining data gets flushed to disk, so it's best to avoid blocking
		// the audio callback while this happens.
		threadedWriter.reset();
	}

	bool isRecording() const
	{
		return activeWriter.load() != nullptr;
	}

	//==============================================================================
	void audioDeviceAboutToStart(AudioIODevice* device) override
	{
		sampleRate = device->getCurrentSampleRate();
	}

	void audioDeviceStopped() override
	{
		sampleRate = 0;
	}

	void audioDeviceIOCallback(const float** inputChannelData, int numInputChannels,
		float** outputChannelData, int numOutputChannels,
		int numSamples) override
	{
		(void)numOutputChannels;
		(void)outputChannelData;
		const ScopedLock sl(writerLock);

		if (activeWriter.load() != nullptr && numInputChannels >= thumbnail.getNumChannels())
		{
			activeWriter.load()->write(inputChannelData, numSamples);

			// Create an AudioBuffer to wrap our incoming data, note that this does no allocations or copies, it simply references our input data
			AudioBuffer<float> buffer(const_cast<float**> (inputChannelData), thumbnail.getNumChannels(), numSamples);
			thumbnail.addBlock(nextSampleNum, buffer, 0, numSamples);
			nextSampleNum += numSamples;
		}

	}

private:
	AudioThumbnail& thumbnail;
	TimeSliceThread backgroundThread{ "Audio Recorder Thread" }; // the thread that will write our audio data to disk
	std::unique_ptr<AudioFormatWriter::ThreadedWriter> threadedWriter; // the FIFO used to buffer the incoming data
	double sampleRate = 0.0;
	int64 nextSampleNum = 0;

	CriticalSection writerLock;
	std::atomic<AudioFormatWriter::ThreadedWriter*> activeWriter{ nullptr };
};

//==============================================================================
class RecordingThumbnail : public Component,
	private ChangeListener
{
public:
	RecordingThumbnail()
	{
		formatManager.registerBasicFormats();
		thumbnail.addChangeListener(this);
	}

	~RecordingThumbnail() override
	{
		thumbnail.removeChangeListener(this);
	}

	AudioThumbnail& getAudioThumbnail() { return thumbnail; }

	void setDisplayFullThumbnail(bool displayFull)
	{
		displayFullThumb = displayFull;
		repaint();
	}

	void paint(Graphics& g) override
	{
		g.fillAll(Colours::darkgrey);
		g.setColour(Colours::lightgrey);

		if (thumbnail.getTotalLength() > 0.0)
		{
			auto endTime = displayFullThumb ? thumbnail.getTotalLength()
				: jmax(30.0, thumbnail.getTotalLength());

			auto thumbArea = getLocalBounds();
			thumbnail.drawChannels(g, thumbArea.reduced(2), 0.0, endTime, 1.0f);
		}
		else
		{
			g.setFont(14.0f);
			g.drawFittedText("(No file recorded)", getLocalBounds(), Justification::centred, 2);
		}
	}

private:
	AudioFormatManager formatManager;
	AudioThumbnailCache thumbnailCache{ 10 };
	AudioThumbnail thumbnail{ 512, formatManager, thumbnailCache };

	bool displayFullThumb = false;

	void changeListenerCallback(ChangeBroadcaster* source) override
	{
		if (source == &thumbnail)
			repaint();
	}

	JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(RecordingThumbnail)
};

//==============================================================================
class ANCInstance : public AudioIODeviceCallback,
						private Thread
{
public:
	/***************************************************************************//**
	 * @brief Default constructor of ANCInstance class
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
	ANCInstance() :Thread("NLMS Processing Thread")
    {
		setPriority(realtimeAudioPriority);
	}
	/***************************************************************************//**
	 * @brief Parametrized constructor of ANCInstance class
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
	ANCInstance(int _filterSize, float _muVal) :Thread("NLMS Processing Thread")
	{		
		filterSize = _filterSize;
		muValue = _muVal;
		setPriority(realtimeAudioPriority);
	}
	/***************************************************************************//**
	 * @brief Default destructor of ANCInstance class
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param Reference to graphics module
	 * @return
	 ******************************************************************************/
	~ANCInstance() {
		stopThread(-1);
	}
	/***************************************************************************//**
 * @brief Overriden function of Thread's run() function
 * @author Michał Berdzik
 * @version 1.0 26/09/2019
 * @param
 * @return
 ******************************************************************************/
	void run() override 
	{
		volatile ring_buffer_size_t availableSamples_L = 0;
		volatile ring_buffer_size_t readedSamples_L = 0;
		volatile ring_buffer_size_t availableSamples_R = 0;
		volatile ring_buffer_size_t readedSamples_R = 0;
		volatile ring_buffer_size_t processedSamples = 0;
		volatile float buffer_L[FRAMES_PER_BUFFER * 4];
		volatile float *bufferPtr_L = (float *)buffer_L;

		volatile float buffer_R[FRAMES_PER_BUFFER * 4];
		volatile float *bufferPtr_R = (float *)buffer_R;

		volatile float buffer[FRAMES_PER_BUFFER * 4];
		volatile float *bufferPtr = (float *)buffer;
		volatile int sampleDelay = 0;
		volatile int sampleCount = 0;		

		while (!threadShouldExit())
		{
#pragma omp parallel sections
			{
#pragma omp section
				{
					availableSamples_L = PaUtil_GetRingBufferReadAvailable(&ringBufferIn_L);
				}
				if (availableSamples_L >= numOfSamples)
				{
					//pthread_mutex_lock( &count_mutex );
					readedSamples_L = PaUtil_ReadRingBuffer(&ringBufferIn_L, (float*)bufferPtr_L, availableSamples_L);
					readedSamples_R = PaUtil_ReadRingBuffer(&ringBufferIn_R, (float*)bufferPtr_R, availableSamples_L);
					//do processing here

					//monoIIRHP.processSamples((float*)bufferPtr_L, readedSamples_L);
					//monoIIRHP.processSamples((float*)bufferPtr_R, readedSamples_R);

//					monoIIR.processSamples((float*)bufferPtr_L, readedSamples_L);
//					monoIIR.processSamples((float*)bufferPtr_R, readedSamples_L);

					sampleDelay+=readedSamples_L;
					if (sampleDelay > FRAMES_PER_BUFFER)
					{
						if (sampleCount < (48000 * 20))
						{							
							for (int n = 0; n < readedSamples_L; n++)
							{
								bufferPtr[n] = -.6f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (1.2f)));								
							}
							arm_lms_f32(&lms_instanceSecPath, (const float*)bufferPtr, (float*)bufferPtr_L, Out, errOutput, readedSamples_L);
							sampleCount += readedSamples_L;
						}
						else
						{							
							arm_fir_f32(&fir_instanceSecPath, (const float*)bufferPtr_R, SecPathFirOut, readedSamples_L);
							arm_lms_anc(&lms_instance, (const float*)SecPathFirOut, (float*)bufferPtr_L, ANCFirOut, errOutput, readedSamples_L);
							arm_fir_f32(&fir_instanceANC, (const float*)bufferPtr_R, (float*)bufferPtr, readedSamples_L);							
						}
					}					
// WITH THIS WORK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					//arm_lms_norm_f32(
					//	&lmsNorm_instance,			/* LMSNorm instance */
					//	(float*)bufferPtr_R,							/* Input signal */
					//	(float*)bufferPtr_L,							/* Reference-Error Signal */
					//	bufferPtr,						/* Converged Signal */
					//	errOutput,					/* Error Signal, this will become small as the signal converges */
					//	readedSamples_L);				/* BlockSize */

					//NLMSFilter.processNLMS(
					//	(float*)bufferPtr_R,
					//	(float*)bufferPtr_L,
					//	bufferPtr,
					//	readedSamples_L
					//);
					
					//monoIIR.processSamples((float*)bufferPtr, readedSamples_L);

#pragma omp section
					{
						memcpy(stereoFIR.state->coefficients.begin(), lmsNormCoeff_f32, filterSize * sizeof(float));
						processedSamples = PaUtil_WriteRingBuffer(&ringBufferOut, (float*)bufferPtr, readedSamples_L);
						//pthread_mutex_unlock( &count_mutex );
					}
				}
			}
		}
	}

	/***************************************************************************//**
	 * @brief Function to get current coefficients of NLMS filter
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param 
	 * @return NLMS FIR Coefficients
	 ******************************************************************************/
#if JUCE_USE_SIMD
	dsp::FIR::Coefficients<float>* getCoeffs() {
		return stereoFIR.state.getObject();		
	}
#else 
	dsp::FIR::Coefficients<float>* getCoeffs() {
		return &coeffs;
	}
#endif
	/***************************************************************************//**
	 * @brief DELETE THIS FUNCTION
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
    void beginTest()
    {
        //resultsBox.moveCaretToEnd();
        //resultsBox.insertTextAtCaret (newLine + newLine + "Starting test..." + newLine);
        //resultsBox.moveCaretToEnd();

        playingSampleNum = recordedSampleNum = 0;
        testIsRunning = true;
    }
	/***************************************************************************//**
	 * @brief Function to set output volume level
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param New volume value
	 * @return
	 ******************************************************************************/
	void setVolume(float vol) {
		volume = vol;
	}
	/***************************************************************************//**
	 * @brief Function to return current SNR value
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param 
	 * @return Float SNR value
	 ******************************************************************************/
	float getSNR() {
		return 0;
	}
	/***************************************************************************//**
	 * @brief Function to put new data od FIFO queue
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param New sample value
	 * @param Channel to put sample to
	 * @return
	 ******************************************************************************/
	void pushNextSamplesIntoFifo(float *samples, bool channel, int dataSize) noexcept
	{
		// if the fifo contains enough data, set a flag to say
		// that the next frame should now be rendered..
		if (!channel) {
			if (fifoIndex_L + dataSize >= 88200)
			{
				if (!nextSNRBlockReady_L)
				{					
					nextSNRBlockReady_L = true;
				}
			}
			else {
				//fifo_L[fifoIndex_L++] = sample;
				memcpy(&fifo_L[0] + fifoIndex_L, samples, dataSize * sizeof(float));
				fifoIndex_L += dataSize;
			}
		}
		if (channel) {
			if (fifoIndex_P + dataSize >= 88200)
			{
				if (!nextSNRBlockReady_P)
				{
					nextSNRBlockReady_P = true;
				}			
			}
			else {

				//			fifo_P[fifoIndex_P++] = sample;
				memcpy(&fifo_P[0] + fifoIndex_P, samples, dataSize * sizeof(float));
				fifoIndex_P += dataSize;
			}
		}
		if (nextSNRBlockReady_L && nextSNRBlockReady_P)
		{
			//SNR = arm_snr_f32(fifo_L, fifo_P, 88200);

			zeromem(fifo_L, sizeof(fifo_L));
			zeromem(fifo_P, sizeof(fifo_P));

			fifoIndex_L = 0;
			fifoIndex_P = 0;
			nextSNRBlockReady_L = false;
			nextSNRBlockReady_P = false;
		}
	}
	/***************************************************************************//**
	 * @brief Overriden function to set audio device parameters before it starts
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param Pointer to IO Audio Device
	 * @return
	 ******************************************************************************/
	static unsigned NextPowerOf2(unsigned val)
	{
		val--;
		val = (val >> 1) | val;
		val = (val >> 2) | val;
		val = (val >> 4) | val;
		val = (val >> 8) | val;
		val = (val >> 16) | val;
		return ++val;
	}

    void audioDeviceAboutToStart (AudioIODevice* device) override
    {
		//omp_set_num_threads(3);

		numOfSamples = device->getCurrentBufferSizeSamples();
        testIsRunning = false;
        playingSampleNum = recordedSampleNum = 0;

        sampleRate          = device->getCurrentSampleRate();
        deviceInputLatency  = device->getInputLatencyInSamples();
        deviceOutputLatency = device->getOutputLatencyInSamples();

#if !JUCE_USE_SIMD
		inData.setSize(2, numOfSamples);
		outData.setSize(2,numOfSamples);
		inData.clear();
		outData.clear();

		arm_lms_norm_init_f32(&lmsNorm_instance, filterSize, lmsNormCoeff_f32, lmsStateF32, muValue, numOfSamples);
		coeffs = dsp::FIR::Coefficients<float>((const float*)lmsNormCoeff_f32, filterSize);
		filter = dsp::FIR::Filter<float>(coeffs);

#else
		interleaved = dsp::AudioBlock<dsp::SIMDRegister<float>>(interleavedBlockData, 2, numOfSamples);
		zero		= dsp::AudioBlock<float>(zeroData, dsp::SIMDRegister<float>::size(), numOfSamples); // [6]
		zero.clear();
		
		dsp::ProcessSpec spec;
		spec.numChannels = 2;
		spec.maximumBlockSize = numOfSamples;
		spec.sampleRate = sampleRate;		

		arm_lms_norm_init_f32(&lmsNorm_instance, filterSize, lmsNormCoeff_f32, lmsStateF32, muValue, numOfSamples);
		arm_lms_init_f32(&lms_instance, filterSize, lmsNormCoeff_f32, lmsStateF32, muValue, numOfSamples);
		arm_lms_init_f32(&lms_instanceSecPath, filterSize, SecPathlmsNormCoeff_f32, SecPathlmsStateF32, muValue / 10.0f, numOfSamples);
		arm_lms_norm_init_f32(&lmsNorm_instanceSecPath, filterSize, SecPathlmsNormCoeff_f32, SecPathlmsStateF32, muValue / 10.0f, numOfSamples);
		arm_fir_init_f32(&fir_instanceSecPath, filterSize, SecPathlmsNormCoeff_f32, SecPathFirState, numOfSamples);
		arm_fir_init_f32(&fir_instanceANC, filterSize, lmsNormCoeff_f32, ANCFirState, numOfSamples);

		stereoFIR.state = new dsp::FIR::Coefficients<float>((const float*)lmsNormCoeff_f32, filterSize);
		stereoFIR.prepare(spec);

		stereoIIR.state = dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 14000.0f, 20);
		stereoIIR.prepare(spec);

		monoIIR.setCoefficients((IIRCoefficients::makeLowPass(sampleRate, 2000, 0.7f)));
		monoIIRHP.setCoefficients(IIRCoefficients::makeHighPass(sampleRate, 30, 0.7f));

		NLMSFilter = Adaptive::NLMS(filterSize, muValue, 0.00000001f);
		FbNLMSFilter = Adaptive::FbLMS(filterSize, muValue);



		int numSamples = NextPowerOf2((unsigned)(SAMPLE_RATE * 0.5 * NR_OF_CHANNELS));
		int numBytes = numSamples * sizeof(float);		
		ringBufferDataIn_L = (float *)malloc(numBytes);
		printf("Creating ringBuffIn array \n");
		if (ringBufferDataIn_L == NULL)
		{
			printf("Could not allocate input ring buffer data.\n");
		}
		printf("Initializing ringBuffIn\n");
		if (PaUtil_InitializeRingBuffer(&ringBufferIn_L, sizeof(float), numSamples, ringBufferDataIn_L) < 0)
		{
			printf("Failed to initialize input ring buffer. Size is not power of 2 ??\n");
		}

		ringBufferDataIn_R = (float *)malloc(numBytes);
		printf("Creating ringBuffIn array \n");
		if (ringBufferDataIn_R == NULL)
		{
			printf("Could not allocate input ring buffer data.\n");
		}
		printf("Initializing ringBuffIn\n");
		if (PaUtil_InitializeRingBuffer(&ringBufferIn_R, sizeof(float), numSamples, ringBufferDataIn_R) < 0)
		{
			printf("Failed to initialize input ring buffer. Size is not power of 2 ??\n");
		}

		printf("Creating ringBuffOut array \n");
		ringBufferDataOut = (float *)malloc(numBytes);
		if (ringBufferDataOut == NULL)
		{
			printf("Could not allocate output ring buffer data.\n");
		}
		printf("Initializing ringBuffOut\n");
		if (PaUtil_InitializeRingBuffer(&ringBufferOut, sizeof(float), numSamples, ringBufferDataOut) < 0)
		{
			printf("Failed to initialize output ring buffer. Size is not power of 2 ??\n");
		}

#endif
		startThread();
    }
	/***************************************************************************//**
	 * @brief Overriden function to clean audio device parameters after it stops
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
    void audioDeviceStopped() override 
	{
		sampleRate = 0;
	}
	/***************************************************************************//**
	 * @brief Callback of IO Audio Device
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param Double pointer to input data
	 * @param Number of input channels
	 * @param Double pointer to input data
	 * @param Number of output channels
	 * @param Number of samples
	 * @return
	 ******************************************************************************/

	static ring_buffer_size_t rbs_min(ring_buffer_size_t a, ring_buffer_size_t b)
	{
		return (a < b) ? a : b;
	}

	void dcBlocker(float *in, float *out, int blockSize)
	{
		for (int i = 0; i < blockSize; i++)
		{
			out[i] = in[i] - xm1 + 0.995f * ym1;
			xm1 = in[i];
			ym1 = out[i];
		}
	}

	void audioDeviceIOCallback(const float** inputChannelData, int numInputChannels,
		float** outputChannelData, int numOutputChannels, int numSamples) override
	{
		static volatile ring_buffer_size_t availableSamples = 0;
		static volatile ring_buffer_size_t readedSamples = 0;
		static volatile ring_buffer_size_t processedSamples = 0;
		static volatile float buffer[FRAMES_PER_BUFFER * 4];
		float *bufferPtr = (float *)buffer;

		const ScopedLock s1(lock);

		(void)numInputChannels;
		(void)numOutputChannels;

//=========================================================================================================================================================	
#pragma omp parallel sections
		{
//			dcBlocker((float*)inputChannelData[0], (float*)inputChannelData[0], numSamples);
//			dcBlocker((float*)inputChannelData[1], (float*)inputChannelData[1], numSamples);
//#if JUCE_LINUX
//			FloatVectorOperations::multiply((float*)inputChannelData[0], 50.0f, numSamples);
//			FloatVectorOperations::multiply((float*)inputChannelData[1], 50.0f, numSamples);
//#endif
			const float *rptr_L = (const float *)inputChannelData[0];
			ring_buffer_size_t elementsWriteable_L = PaUtil_GetRingBufferWriteAvailable(&ringBufferIn_L);
			ring_buffer_size_t elementsToWrite_L = rbs_min(elementsWriteable_L, (ring_buffer_size_t)(numSamples));
			PaUtil_WriteRingBuffer(&ringBufferIn_L, rptr_L, elementsToWrite_L);

			const float *rptr_R = (const float *)inputChannelData[1];
			ring_buffer_size_t elementsWriteable_R = PaUtil_GetRingBufferWriteAvailable(&ringBufferIn_R);
			ring_buffer_size_t elementsToWrite_R = rbs_min(elementsWriteable_R, (ring_buffer_size_t)(numSamples));
			PaUtil_WriteRingBuffer(&ringBufferIn_R, rptr_R, elementsToWrite_R);

#pragma omp section
			{
				float *wptr = (float *)outputChannelData[0];
				ring_buffer_size_t elementsToPlay = PaUtil_GetRingBufferReadAvailable(&ringBufferOut);
				ring_buffer_size_t elementsToRead = rbs_min(elementsToPlay, (ring_buffer_size_t)(numSamples));
				readedSamples = PaUtil_ReadRingBuffer(&ringBufferOut, wptr, elementsToRead);
			}
			if (readedSamples < numOfSamples)
			{
#pragma omp parallel for shedule(static, 4)
				for (int i = readedSamples; i < numOfSamples; i++)
				{
					outputChannelData[0][i] = 0;
				}
			}
		}
		
#pragma omp parallel sections 
		{
#pragma omp section
			{
				pushNextSamplesIntoFifo((float*)inputChannelData[0], 0, numSamples);
				pushNextSamplesIntoFifo((float*)inputChannelData[1], 1, numSamples);
			}
			//nlmsFilter.nlms_step((float*)inputChannelData[1], (float*)inputChannelData[0], outputChannelData[0], numSamples);
				//monoIIR.processSamples((float*)inputChannelData[0], numSamples);
				//monoIIR.processSamples((float*)inputChannelData[1], numSamples);

			//arm_lms_norm_f32(
			//	&lmsNorm_instance,			/* LMSNorm instance */
			//	(float*)inputChannelData[1],							/* Input signal */
			//	(float*)inputChannelData[0],							/* Reference-Error Signal */
			//	outputChannelData[0],						/* Converged Signal */
			//	errOutput,					/* Error Signal, this will become small as the signal converges */
			//	numSamples);				/* BlockSize */
			

			//arm_lms_norm_anc(
			//	&lmsNorm_instance,			/* LMSNorm instance */
			//	(const float*)inputChannelData[1],							/* Input signal */
			//	(float*)inputChannelData[0],							/* Reference-Error Signal */
			//	outputChannelData[0],						/* Converged Signal */
			//	errOutput,					/* Error Signal, this will become small as the signal converges */
			//	numSamples);				/* BlockSize */

			//NLMSFilter.processNLMS(
			//	(float*)inputChannelData[1],
			//	(float*)inputChannelData[0],
			//	outputChannelData[0],
			//	numSamples
			//);

#pragma omp section
			{
				//		stereoFIR.state = new dsp::FIR::Coefficients<float>((const float*)lmsNormCoeff_f32, NUM_OF_TAPS);
				//memcpy(stereoFIR.state->coefficients.begin(), lmsNormCoeff_f32, filterSize * sizeof(float));
				//memcpy(stereoFIR.state->coefficients.begin(), NLMSFilter.getCoeff(), filterSize * sizeof(float));
				//memcpy(stereoFIR.state->coefficients.begin(), FbNLMSFilter.getCoeff(), filterSize * sizeof(float));
		}
#pragma omp section
			{
				FloatVectorOperations::multiply((float*)outputChannelData[0], volume, numSamples);				
				//monoIIR.processSamples(outputChannelData[0], numSamples);
				FloatVectorOperations::copy((float*)outputChannelData[1], (float*)outputChannelData[0], numSamples);
			}
			//=========================================================================================================================================================
//		auto end = std::chrono::high_resolution_clock::now();
//		auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		}
	}
	
	
	/***************************************************************************//**
	 * @brief Function to process new pack of input data 
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param Pointer to array of noise audio
	 * @param Pointer to array of destiny audio
	 * @return
	 ******************************************************************************/
	void processSamples(float *x, float *d) {

		arm_lms_norm_f32(
			&lmsNorm_instance,			/* LMSNorm instance */
			x,							/* Input signal */
			d,							/* Reference Signal */
			Out,						/* Converged Signal */
			errOutput,					/* Error Signal, this will become small as the signal converges */
			numOfSamples);				/* BlockSize */

#if JUCE_USE_SIMD
//		stereoFIR.state = new dsp::FIR::Coefficients<float>((const float*)lmsNormCoeff_f32, NUM_OF_TAPS);
		memcpy(stereoFIR.state->coefficients.begin(), lmsNormCoeff_f32, filterSize * sizeof(float));
#else
		memcpy(coeffs.coefficients.begin(), lmsNormCoeff_f32, filterSize * sizeof(float));
#endif
	}

private:
	CriticalSection lock;

	int filterSize	= 1;
	float muValue	= 1;

    int playingSampleNum  = 0;
    int recordedSampleNum = -1;
    double sampleRate     = 0.0;
    bool testIsRunning    = false;
    int deviceInputLatency, deviceOutputLatency, numOfSamples;
	float volume = 1.0f;

	arm_lms_norm_instance_f32 lmsNorm_instance;
	arm_lms_instance_f32 lms_instance;
	arm_lms_instance_f32 lms_instanceSecPath;
	arm_lms_norm_instance_f32 lmsNorm_instanceSecPath;
	arm_fir_instance_f32 fir_instanceSecPath;
	arm_fir_instance_f32 fir_instanceANC;

	float y[FRAMES_PER_BUFFER] = { 0.0f };								// Output data
	float e[FRAMES_PER_BUFFER] = { 0.0f };								// Error data
	float Out[FRAMES_PER_BUFFER] = { 0.0f };							// Output data
	float errOutput[FRAMES_PER_BUFFER] = { 0.0f };						// Error data
	float lmsStateF32[NUM_OF_TAPS + FRAMES_PER_BUFFER] = { 0.0f };	// Array for NLMS algorithm
	float lmsNormCoeff_f32[NUM_OF_TAPS] = { 0.0f };					// NLMS Coefficients
	float SecPathlmsStateF32[NUM_OF_TAPS + FRAMES_PER_BUFFER] = { 0.0f };	// Array for NLMS algorithm
	float SecPathlmsNormCoeff_f32[NUM_OF_TAPS] = { 0.0f };					// NLMS Coefficients
	float SecPathFirState[NUM_OF_TAPS] = { 0.0f };
	float SecPathFirOut[FRAMES_PER_BUFFER] = { 0.0f };
	float ANCFirState[NUM_OF_TAPS] = { 0.0f };
	float ANCFirOut[FRAMES_PER_BUFFER] = { 0.0f };
	float zeros[FRAMES_PER_BUFFER] = { 0.0f };
	float xm1 = 0;
	float ym1 = 0;

	Adaptive::NLMS NLMSFilter;
	Adaptive::FbLMS FbNLMSFilter;

	float fifo_L[88200];
	int fifoIndex_L = 0;
	bool nextSNRBlockReady_L = false;
	
	float fifo_P[88200];
	int fifoIndex_P = 0;
	bool nextSNRBlockReady_P = false;

	PaUtilRingBuffer ringBufferIn_L;
	PaUtilRingBuffer ringBufferIn_R;
	PaUtilRingBuffer ringBufferOut;
	float *ringBufferDataIn_L;
	float *ringBufferDataIn_R;
	float *ringBufferDataOut;

#if JUCE_USE_SIMD
	dsp::AudioBlock<float> inBlock;
	dsp::AudioBlock<float> outBlock;
	dsp::AudioBlock<dsp::SIMDRegister<float>> interleaved;           
	dsp::AudioBlock<float> zero;
	HeapBlock<char> interleavedBlockData, zeroData;        
	HeapBlock<const float*> channelPointers{ dsp::SIMDRegister<float>::size() };
	dsp::ProcessorDuplicator<dsp::FIR::Filter<dsp::SIMDRegister<float>>, dsp::FIR::Coefficients<float>> stereoFIR;
	dsp::ProcessorDuplicator<dsp::IIR::Filter<dsp::SIMDRegister<float>>, dsp::IIR::Coefficients<float>> stereoIIR;
	IIRFilter monoIIR;
	IIRFilter monoIIRHP;
#else
	AudioBuffer<float> inData, outData;
	dsp::FIR::Coefficients<float> coeffs;
	dsp::FIR::Filter<float> filter;
#endif

	JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(ANCInstance)
};

//==============================================================================
class ActiveNoiseCancelling  : public Component,
								private Timer,
								public Slider::Listener
{
public:
	/***************************************************************************//**
	 * @brief Default constructor of ActiveNoiseCancelling class
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
	ActiveNoiseCancelling()
    {
        setOpaque (true);		
		
		startTimerHz(3);

		// Set up Volume Label text box 
		volumeLabel.setText("Volume: ", dontSendNotification);		
		volumeLabel.attachToComponent(&volumeSlider, true);
		volumeLabel.setColour(volumeLabel.textColourId, Colour(255, 255, 255));

		// Set up Volume Slider box 
		volumeSlider.setRange(0, 10);		
		volumeSlider.addListener(this);
		volumeSlider.setValue(1);
		volumeSlider.setTextBoxStyle(Slider::TextBoxLeft, false, 120, volumeSlider.getTextBoxHeight());

		// Set up Filter size Label text box 
		filterSizeLabel.setText("Filter size: ", dontSendNotification);
		filterSizeLabel.attachToComponent(&filterSizeSlider, true);
		filterSizeLabel.setColour(volumeLabel.textColourId, Colour(255, 255, 255));

		// Set up Filter Size Slider box 
		filterSizeSlider.setRange(256.0, 4096.0, 128.0);
		filterSizeSlider.addListener(this);
		filterSizeSlider.setValue(512);
		filterSizeSlider.setTextBoxStyle(Slider::TextBoxLeft, false, 120, volumeSlider.getTextBoxHeight());

		// Set up Filter size Label text box 
		filterMULabel.setText("Filter mu: ", dontSendNotification);
		filterMULabel.attachToComponent(&filterMUSlider, true);
		filterMULabel.setColour(volumeLabel.textColourId, Colour(255, 255, 255));

		// Set up Filter Size Slider box 
		filterMUSlider.setRange(0.00001, 1.0, 0.00001);
		filterMUSlider.addListener(this);
		filterMUSlider.setValue(0.25);
		filterMUSlider.setTextBoxStyle(Slider::TextBoxLeft, false, 120, volumeSlider.getTextBoxHeight());
		
		// Set up FFT Scale Slider box
		FFTScaleSlider.setTextBoxStyle(Slider::TextBoxBelow, false, 30, FFTScaleSlider.getTextBoxHeight());
		FFTScaleSlider.setSliderStyle(Slider::TwoValueVertical);
		FFTScaleSlider.setRange(-99, 0, 1);
		FFTScaleSlider.setTextValueSuffix(" dB");
		FFTScaleSlider.addListener(this);
		FFTScaleSlider.setMinValue(-99);
		FFTScaleSlider.setMaxValue(0);

		// Add components to be visible
		addAndMakeVisible(FFTScaleSlider);
		addAndMakeVisible(volumeLabel);
		addAndMakeVisible(volumeSlider);
		addAndMakeVisible(filterSizeSlider);
		addAndMakeVisible(filterMUSlider);

		// Reset and add main audio processing components
        liveAudioScroller.reset (new LiveScrollingAudioDisplay());
		spectrumAnalyser.reset(new AnalyserComponent());
		addAndMakeVisible(spectrumAnalyser.get());
		addAndMakeVisible (liveAudioScroller.get());


		filterVisualizer.reset(new FilterVisualizer());		
		filterVisualizer->addCoefficients(&elo, Colours::white);
		addAndMakeVisible(filterVisualizer.get());
		
		addAndMakeVisible(SNR_Value);
		SNR_Value.setColour(SNR_Value.backgroundColourId, Colour(39, 50, 56));
		SNR_Value.setColour(SNR_Value.textColourId, Colour(137, 176, 196));
		SNR_Value.setJustificationType(Justification::centred);
		SNR_Value.setEditable(false);
		SNR_Value.setText("Run ANC to see results", dontSendNotification);

        addAndMakeVisible (startTestButton);
        startTestButton.onClick = [this] { startTest(); };

		addAndMakeVisible(startVisualizigData);
		startVisualizigData.onClick = [this] 
		{
			if (isVisualisingRunning) {
				audioDeviceManager.removeAudioCallback(liveAudioScroller.get());
				audioDeviceManager.removeAudioCallback(spectrumAnalyser.get());
				isVisualisingRunning = false;
			}
			else {
				audioDeviceManager.addAudioCallback(liveAudioScroller.get());
				audioDeviceManager.addAudioCallback(spectrumAnalyser.get());
				isVisualisingRunning = true;
			}
		};

       #ifndef JUCE_DEMO_RUNNER
        RuntimePermissions::request (RuntimePermissions::recordAudio,
                                     [this] (bool granted)
                                     {
                                         int numInputChannels = granted ? 2 : 0;
                                         audioDeviceManager.initialise (numInputChannels, 2, nullptr, true, {}, nullptr);
                                     });
       #endif

		addAndMakeVisible(explanationLabel);
		explanationLabel.setFont(Font(15.0f, Font::plain));
		explanationLabel.setJustificationType(Justification::topLeft);
		explanationLabel.setEditable(false, false, false);
		explanationLabel.setColour(TextEditor::textColourId, Colours::black);
		explanationLabel.setColour(TextEditor::backgroundColourId, Colour(0x00000000));

		addAndMakeVisible(recordButton);
		recordButton.setColour(TextButton::buttonColourId, Colour(0xffff5c5c));
		recordButton.setColour(TextButton::textColourOnId, Colours::black);

		recordButton.onClick = [this]
		{
			if (recorder.isRecording())
				stopRecording();
			else
				startRecording();
		};

		addAndMakeVisible(recordingThumbnail);

#ifndef JUCE_DEMO_RUNNER
		RuntimePermissions::request(RuntimePermissions::recordAudio,
			[this](bool granted)
		{
			int numInputChannels = granted ? 2 : 0;
			audioDeviceManager.initialise(numInputChannels, 2, nullptr, true, {}, nullptr);
		});
#endif


//		audioDeviceManager.addAudioCallback(latencyTester.get());
        audioDeviceManager.addAudioCallback (liveAudioScroller.get());
		audioDeviceManager.addAudioCallback(spectrumAnalyser.get());
		audioDeviceManager.addAudioCallback(&recorder);
		
        setSize (500, 800);
    }
	/***************************************************************************//**
	 * @brief Default destructor of ActiveNoiseCancelling class
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
    ~ActiveNoiseCancelling()
    {
		audioDeviceManager.removeAudioCallback(&recorder);
        audioDeviceManager.removeAudioCallback (liveAudioScroller.get());        
		audioDeviceManager.removeAudioCallback(spectrumAnalyser.get());
		audioDeviceManager.removeAudioCallback(ANC.get());

		ANC.reset();
        liveAudioScroller.reset();
		spectrumAnalyser.reset();		
		filterVisualizer.reset();
    }
	/***************************************************************************//**
	 * @brief function of timer callback - specifies what to do when timer is called
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
	void timerCallback() override
	{
		if (ANC.get() != nullptr)
		{
			SNR_Value.setText(String("SNR = ") + String(ANC->getSNR()) + String(" dB"), NotificationType::dontSendNotification);
			elo = ANC->getCoeffs();
		}
	}
	/***************************************************************************//**
	 * @brief Overriden function of Slider's class
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param Pointer to Slider object
	 * @return
	 ******************************************************************************/
	void sliderValueChanged(Slider* slider) override
	{
		if (slider == &volumeSlider) {
			if (ANC.get() != nullptr) {
				ANC->setVolume((float)volumeSlider.getValue());
			}
		}
		else if (slider == &FFTScaleSlider) {
			if (spectrumAnalyser.get() != nullptr) {
				spectrumAnalyser->setScaleValue(FFTScaleSlider.getMinValue(), FFTScaleSlider.getMaxValue());
			}
		}
	}
	/***************************************************************************//**
	 * @brief Function to start ANC
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
    void startTest()
    {
        if (ANC.get() == nullptr)
        {
			ANC.reset (new ANCInstance(filterSizeSlider.getValue(),filterMUSlider.getValue()));
            audioDeviceManager.addAudioCallback (ANC.get());	
			filterSizeSlider.setEnabled(false);
			filterMUSlider.setEnabled(false);
        }

		ANC->beginTest();
    }
	/***************************************************************************//**
	 * @brief Overriden function to draw new data on screen
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param Reference to graphic module
	 * @return
	 ******************************************************************************/
    void paint (Graphics& g) override
    {
        g.fillAll (findColour (ResizableWindow::backgroundColourId));
    }
	/***************************************************************************//**
	 * @brief Overriden function called when application window is resized
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
    void resized() override
    {
        auto b = getLocalBounds().reduced (5);

		volumeSlider.setBounds(b.getX() + 80, b.getY(), b.getWidth() / 2, b.getHeight() / 20);
		startVisualizigData.setBounds(b.getX() + 80 + b.getWidth() / 2, b.getY(), b.getWidth() / 4 - 40, b.getHeight() / 20);
		recordButton.setBounds(b.getX() + 40 + b.getWidth() / 2 + b.getWidth() / 4, b.getY(), b.getWidth() / 4 - 40, b.getHeight() / 20);
		b.removeFromTop(b.getHeight() / 20);
		b.removeFromTop(3);

		filterSizeSlider.setBounds(b.getX() + 80, b.getY(), b.getWidth() / 2 - 80, b.getHeight() / 20);
		filterMUSlider.setBounds(b.getX() + b.getWidth() / 2 + 80, b.getY(), b.getWidth() / 2 - 80, b.getHeight() / 20);
		b.removeFromTop(b.getHeight() / 20);
		b.removeFromTop(3);

        if (liveAudioScroller.get() != nullptr)
        {
            liveAudioScroller->setBounds (b.removeFromTop (b.getHeight() / 8));
            b.removeFromTop (3);
        }

        startTestButton.setBounds (b.removeFromBottom (b.getHeight() / 15));
        b.removeFromBottom (10);		

		SNR_Value.setBounds(b.removeFromBottom(b.getHeight() / 15));
		b.removeFromBottom(10);
        
		FFTScaleSlider.setBounds(b.getX(), b.getY(), 30, b.getHeight() / 2);
		spectrumAnalyser->setBounds(b.getX() + 30, b.getY(), b.getWidth() - 30, b.getHeight() / 2);

		b.removeFromTop(b.getHeight() / 2  + 3);

		filterVisualizer->setBounds(b);
    }

private:
    // if this PIP is running inside the demo runner, we'll use the shared device manager instead
   #ifndef JUCE_DEMO_RUNNER
    AudioDeviceManager audioDeviceManager;
   #else
    AudioDeviceManager& audioDeviceManager { getSharedAudioDeviceManager (2, 2) };	
   #endif

	bool isVisualisingRunning = true;

    std::unique_ptr<ANCInstance> ANC;
    std::unique_ptr<LiveScrollingAudioDisplay> liveAudioScroller;
	std::unique_ptr<AnalyserComponent> spectrumAnalyser;
	std::unique_ptr<FilterVisualizer> filterVisualizer;
	RecordingThumbnail recordingThumbnail;
	AudioRecorder recorder{ recordingThumbnail.getAudioThumbnail() };
	Label explanationLabel{ {}, "This page demonstrates how to record a wave file from the live audio input..\n\n"
								 #if (JUCE_ANDROID || JUCE_IOS)
								  "After you are done with your recording you can share with other apps."
								 #else
								  "Pressing record will start recording a file in your \"Documents\" folder."
								 #endif
	};
	TextButton recordButton{ "Record" };
	File lastRecording;

    TextButton startTestButton  { "Run ANC" };
	TextButton startVisualizigData{ "Run/Stop Charts" };

	Label SNR_Value;
    //TextEditor resultsBox;

	Slider volumeSlider;
	Label volumeLabel;

	Slider filterSizeSlider;
	Label  filterSizeLabel;

	Slider filterMUSlider;
	Label  filterMULabel;

	Slider FFTScaleSlider;

	dsp::FIR::Coefficients<float>::Ptr elo;

	void startRecording()
	{
		if (!RuntimePermissions::isGranted(RuntimePermissions::writeExternalStorage))
		{
			SafePointer<ActiveNoiseCancelling> safeThis(this);

			RuntimePermissions::request(RuntimePermissions::writeExternalStorage,
				[safeThis](bool granted) mutable
			{
				if (granted)
					safeThis->startRecording();
			});
			return;
		}

		auto parentDir = File::getSpecialLocation(File::userDocumentsDirectory);

		lastRecording = parentDir.getNonexistentChildFile("JUCE Demo Audio Recording", ".wav");

		recorder.startRecording(lastRecording);

		recordButton.setButtonText("Stop");
		recordingThumbnail.setDisplayFullThumbnail(false);
	}

	void stopRecording()
	{
		recorder.stop();


		lastRecording = File();
		recordButton.setButtonText("Record");
		recordingThumbnail.setDisplayFullThumbnail(true);
	}

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ActiveNoiseCancelling)
};
