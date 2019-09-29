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

#include <future>

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
		SNR.store(0);
		setPriority(realtimeAudioPriority);
	}
	/***************************************************************************//**
	 * @brief Parametrized constructor of ANCInstance class
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
	ANCInstance(int _filterSize, float _muVal) :Thread("NLMS Processing Thread"), filterSize(_filterSize), muValue(_muVal)
	{

		SNR.store(0);
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
		while (!threadShouldExit())
		{
			wait(-1);
//			auto start = std::chrono::high_resolution_clock::now();
#if JUCE_USE_SIMD
			processSamples((float*)inBlock.getChannelPointer(0), (float*)inBlock.getChannelPointer(1));
#else
			processSamples((float*)inData.getReadPointer(0), (float*)inData.getReadPointer(1));
#endif
//			SNR = arm_snr_f32((float*)inBlock.getChannelPointer(1), (float*)inBlock.getChannelPointer(0), numOfSamples);
//			auto end = std::chrono::high_resolution_clock::now();
//			auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();			
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

        const ScopedLock sl (lock);
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
		return SNR.load();
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
			SNR = arm_snr_f32(fifo_L, fifo_P, 88200);

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
    void audioDeviceAboutToStart (AudioIODevice* device) override
    {
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

		stereoFIR.state = new dsp::FIR::Coefficients<float>((const float*)lmsNormCoeff_f32, filterSize);
		stereoFIR.prepare(spec);

		stereoIIR.state = dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 14000.0f, 20);
		stereoIIR.prepare(spec);
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
	void audioDeviceIOCallback(const float** inputChannelData, int numInputChannels,
		float** outputChannelData, int numOutputChannels, int numSamples) override
	{
		const ScopedLock sl(lock);
//		auto start = std::chrono::high_resolution_clock::now();
#if !JUCE_USE_SIMD

				auto* playBufferL = outData.getReadPointer(0);
				auto* playBufferR = outData.getReadPointer(1);
				auto* recBufferL = inData.getWritePointer(0);
				auto* recBufferR = inData.getWritePointer(1);
		
				for (int i = 0; i < numSamples; ++i)
				{
					if (recordedSampleNum < inData.getNumSamples()) {
						//				for (int j = 0; j < numInputChannels; j++) {
						//					outputChannelData[j][i] = inputChannelData[j][i];					
						//				}
						recBufferL[recordedSampleNum] = inputChannelData[0][i];
						recBufferR[recordedSampleNum] = inputChannelData[1][i];
					}
					++recordedSampleNum;			
		
					auto outputSampleL = (playingSampleNum <= outData.getNumSamples()) ? playBufferL[playingSampleNum] : 0.0f;
					auto outputSampleR = (playingSampleNum <= outData.getNumSamples()) ? playBufferR[playingSampleNum] : 0.0f;
		
					outputChannelData[0][i] = (volume) * (filter.processSample(inputChannelData[0][i]));
					outputChannelData[1][i] = (volume) * (filter.processSample(inputChannelData[1][i]));
		
//					outputChannelData[0][i] = inputChannelData[0][i];
//					outputChannelData[1][i] = inputChannelData[1][i];
		
					++playingSampleNum;
				}		
				notify();
		
				recordedSampleNum = 0;
				playingSampleNum = 0;
#else
		inBlock	 = dsp::AudioBlock<float>((float*const*)inputChannelData, (size_t)numInputChannels, (size_t)numOfSamples);
		outBlock = dsp::AudioBlock<float>((float*const*)outputChannelData, (size_t)numOutputChannels, (size_t)numOfSamples);
		auto* inout = channelPointers.getData();
		auto n = inBlock.getNumSamples();
	
		pushNextSamplesIntoFifo((float*)inputChannelData[0], 0, numSamples);
		pushNextSamplesIntoFifo((float*)inputChannelData[1], 1, numSamples);

		for (size_t ch = 0; ch < dsp::SIMDRegister<float>::size(); ++ch)
			inout[ch] = (ch < numInputChannels ? const_cast<float*> (inBlock.getChannelPointer(ch)) : zero.getChannelPointer(ch));
		
		AudioDataConverters::interleaveSamples(inout, reinterpret_cast<float*> (interleaved.getChannelPointer(0)),
		static_cast<int> (n), static_cast<int> (dsp::SIMDRegister<float>::size()));
				
		stereoFIR.process(dsp::ProcessContextReplacing<dsp::SIMDRegister<float>>(interleaved));		
		
		notify();

//		stereoIIR.process(dsp::ProcessContextReplacing<dsp::SIMDRegister<float>>(interleaved));

		for (size_t ch = 0; ch < inBlock.getNumChannels(); ++ch)
			inout[ch] = outBlock.getChannelPointer(ch);
			
		AudioDataConverters::deinterleaveSamples(reinterpret_cast<float*> (interleaved.getChannelPointer(0)),
			const_cast<float**> (inout),
			static_cast<int> (n), static_cast<int> (dsp::SIMDRegister<float>::size()));

		FloatVectorOperations::copy((float*)inout[1], (float*)inout[0], numOfSamples);

		FloatVectorOperations::multiply((float*)inout[0], volume, numSamples);
		FloatVectorOperations::multiply((float*)inout[1], volume, numSamples);

#endif
//		auto end = std::chrono::high_resolution_clock::now();
//		auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	}
	/***************************************************************************//**
	 * @brief Function to process new pack of input data 
	 * @author Michał Berdzik
	 * @version 1.0 26/09/2019
	 * @param Pointer to array of noise audio
	 * @param Pointer to array of destiny audio
	 * @return
	 ******************************************************************************/
	inline void processSamples(float *x, float *d) {
		const ScopedLock sl(lock);

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

	std::atomic<float> SNR;

	int filterSize	= 1;
	float muValue	= 1;

    int playingSampleNum  = 0;
    int recordedSampleNum = -1;
    double sampleRate     = 0.0;
    bool testIsRunning    = false;
    int deviceInputLatency, deviceOutputLatency, numOfSamples;
	float volume = 1;

	arm_lms_norm_instance_f32 lmsNorm_instance;
	float y[FRAMES_PER_BUFFER] = { 0 };								// Output data
	float e[FRAMES_PER_BUFFER] = { 0 };								// Error data
	float Out[FRAMES_PER_BUFFER] = { 0 };							// Output data
	float errOutput[FRAMES_PER_BUFFER] = { 0 };						// Error data
	float lmsStateF32[NUM_OF_TAPS + FRAMES_PER_BUFFER] = { 0.1 };	// Array for NLMS algorithm
	float lmsNormCoeff_f32[NUM_OF_TAPS] = { 0.1 };					// NLMS Coefficients
	
	float fifo_L[88200];
	int fifoIndex_L = 0;
	bool nextSNRBlockReady_L = false;
	
	float fifo_P[88200];
	int fifoIndex_P = 0;
	bool nextSNRBlockReady_P = false;

#if JUCE_USE_SIMD
	dsp::AudioBlock<float> inBlock;
	dsp::AudioBlock<float> outBlock;
	dsp::AudioBlock<dsp::SIMDRegister<float>> interleaved;           
	dsp::AudioBlock<float> zero;
	HeapBlock<char> interleavedBlockData, zeroData;        
	HeapBlock<const float*> channelPointers{ dsp::SIMDRegister<float>::size() };
	dsp::ProcessorDuplicator<dsp::FIR::Filter<dsp::SIMDRegister<float>>, dsp::FIR::Coefficients<float>> stereoFIR;
	dsp::ProcessorDuplicator<dsp::IIR::Filter<dsp::SIMDRegister<float>>, dsp::IIR::Coefficients<float>> stereoIIR;
#else
	AudioBuffer<float> inData, outData;
	dsp::FIR::Coefficients<float> coeffs;
	dsp::FIR::Filter<float> filter;
#endif

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ANCInstance)
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
		filterSizeSlider.setValue(256);
		filterSizeSlider.setTextBoxStyle(Slider::TextBoxLeft, false, 120, volumeSlider.getTextBoxHeight());

		// Set up Filter size Label text box 
		filterMULabel.setText("Filter mu: ", dontSendNotification);
		filterMULabel.attachToComponent(&filterMUSlider, true);
		filterMULabel.setColour(volumeLabel.textColourId, Colour(255, 255, 255));

		// Set up Filter Size Slider box 
		filterMUSlider.setRange(0.00001, 1.0, 0.00001);
		filterMUSlider.addListener(this);
		filterMUSlider.setValue(0.68584);
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

//		audioDeviceManager.addAudioCallback(latencyTester.get());
        audioDeviceManager.addAudioCallback (liveAudioScroller.get());
		audioDeviceManager.addAudioCallback(spectrumAnalyser.get());
		
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
        audioDeviceManager.removeAudioCallback (liveAudioScroller.get());
        audioDeviceManager.removeAudioCallback (ANC.get());
		audioDeviceManager.removeAudioCallback(spectrumAnalyser.get());
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
				ANC->setVolume(volumeSlider.getValue());
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

		volumeSlider.setBounds(b.getX() + 120, b.getY(), b.getWidth() / 2, b.getHeight() / 20);
		startVisualizigData.setBounds(b.getX() + 120 + b.getWidth() / 2, b.getY(), b.getWidth() / 2 - 120, b.getHeight() / 20);
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

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ActiveNoiseCancelling)
};
