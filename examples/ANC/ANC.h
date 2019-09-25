/*
  ==============================================================================

   This file is part of the JUCE examples.
   Copyright (c) 2017 - ROLI Ltd.

   The code included in this file is provided under the terms of the ISC license
   http://www.isc.org/downloads/software-support-policy/isc-license. Permission
   To use, copy, modify, and/or distribute this software for any purpose with or
   without fee is hereby granted provided that the above copyright notice and
   this permission notice appear in all copies.

   THE SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY, AND ALL WARRANTIES,
   WHETHER EXPRESSED OR IMPLIED, INCLUDING MERCHANTABILITY AND FITNESS FOR
   PURPOSE, ARE DISCLAIMED.

  ==============================================================================
*/

/*******************************************************************************
 The block below describes the properties of this PIP. A PIP is a short snippet
 of code that can be read by the Projucer and used to generate a JUCE project.

 BEGIN_JUCE_PIP_METADATA

 name:             AudioLatencyDemo
 version:          1.0.0
 vendor:           JUCE
 website:          http://juce.com
 description:      Tests the audio latency of a device.

 dependencies:     juce_audio_basics, juce_audio_devices, juce_audio_formats,
                   juce_audio_processors, juce_audio_utils, juce_core,
                   juce_data_structures, juce_events, juce_graphics,
                   juce_gui_basics, juce_gui_extra
 exporters:        xcode_mac, vs2017, linux_make, androidstudio, xcode_iphone

 moduleFlags:      JUCE_STRICT_REFCOUNTEDPOINTER=1

 type:             Component
 mainClass:        AudioLatencyDemo

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
	ANCInstance() :Thread("NLMS Processing Thread")
    {
		SNR.store(0);
		setPriority(realtimeAudioPriority);
	}
	~ANCInstance() {
		stopThread(-1);
	}

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

    //==============================================================================
#if JUCE_USE_SIMD
	dsp::FIR::Coefficients<float>* getCoeffs() {
		return stereoFIR.state.getObject();		
	}
#else 
	dsp::FIR::Coefficients<float>* getCoeffs() {
		return &coeffs;
	}
#endif
    void beginTest()
    {
        //resultsBox.moveCaretToEnd();
        //resultsBox.insertTextAtCaret (newLine + newLine + "Starting test..." + newLine);
        //resultsBox.moveCaretToEnd();

        const ScopedLock sl (lock);
        playingSampleNum = recordedSampleNum = 0;
        testIsRunning = true;
    }

	void setVolume(float vol) {
		volume = vol;
	}

	float getSNR() {
		return SNR.load();
	}

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
    //==============================================================================
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

		arm_lms_norm_init_f32(&lmsNorm_instance, NUM_OF_TAPS, lmsNormCoeff_f32, lmsStateF32, MU, numOfSamples);
		coeffs = dsp::FIR::Coefficients<float>((const float*)lmsNormCoeff_f32, NUM_OF_TAPS);
		filter = dsp::FIR::Filter<float>(coeffs);

#else
		interleaved = dsp::AudioBlock<dsp::SIMDRegister<float>>(interleavedBlockData, 2, numOfSamples);
		zero		= dsp::AudioBlock<float>(zeroData, dsp::SIMDRegister<float>::size(), numOfSamples); // [6]
		zero.clear();
		
		dsp::ProcessSpec spec;
		spec.numChannels = 2;
		spec.maximumBlockSize = numOfSamples;
		spec.sampleRate = sampleRate;		

		arm_lms_norm_init_f32(&lmsNorm_instance, NUM_OF_TAPS, lmsNormCoeff_f32, lmsStateF32, MU, numOfSamples);

		stereoFIR.state = new dsp::FIR::Coefficients<float>((const float*)lmsNormCoeff_f32, NUM_OF_TAPS);
		stereoFIR.prepare(spec);

		stereoIIR.state = dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 14000.0f, 20);
		stereoIIR.prepare(spec);
#endif
		startThread();
    }

    void audioDeviceStopped() override {}

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

private:
    CriticalSection lock;

	std::atomic<float> SNR;

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
	//==============================================================================

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
		memcpy(stereoFIR.state->coefficients.begin(), lmsNormCoeff_f32, NUM_OF_TAPS * sizeof(float));
#else
		memcpy(coeffs.coefficients.begin(), lmsNormCoeff_f32, NUM_OF_TAPS * sizeof(float));
#endif
	}

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ANCInstance)
};

//==============================================================================
class ActiveNoiseCancelling  : public Component,
								private Timer,
								public Slider::Listener
{
public:
	ActiveNoiseCancelling()
    {
        setOpaque (true);
		
		startTimerHz(3);

		// Set up Volume Label text box 
		volumeLabel.setText("Volume: ", dontSendNotification);		
		volumeLabel.attachToComponent(&volumeSlider, true);
		volumeLabel.setColour(volumeLabel.textColourId, Colour(255, 255, 255));

		// Set up Volume Slider box 
		volumeSlider.setRange(0, 1);		
		volumeSlider.addListener(this);
		volumeSlider.setValue(1);
		volumeSlider.setTextBoxStyle(Slider::TextBoxLeft, false, 120, volumeSlider.getTextBoxHeight());
		
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

	void timerCallback() override
	{
		if (ANC.get() != nullptr)
		{
			SNR_Value.setText(String("SNR = ") + String(ANC->getSNR()) + String(" dB"), NotificationType::dontSendNotification);
			elo = ANC->getCoeffs();
		}
	}

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

    void startTest()
    {
        if (ANC.get() == nullptr)
        {
			ANC.reset (new ANCInstance());
            audioDeviceManager.addAudioCallback (ANC.get());
//			elo = dsp::IIR::Coefficients<float>::makeLowPass(48000, 5000);			
        }

		ANC->beginTest();
    }

    void paint (Graphics& g) override
    {
        g.fillAll (findColour (ResizableWindow::backgroundColourId));
    }

    void resized() override
    {
        auto b = getLocalBounds().reduced (5);

		volumeSlider.setBounds(b.getX() + 120, b.getY(), b.getWidth() / 2, b.getHeight() / 20);
		startVisualizigData.setBounds(b.getX() + 120 + b.getWidth() / 2, b.getY(), b.getWidth() / 2 - 120, b.getHeight() / 20);
		b.removeFromTop(b.getHeight() / 20);
		b.removeFromTop(3);
		

        if (liveAudioScroller.get() != nullptr)
        {
            liveAudioScroller->setBounds (b.removeFromTop (b.getHeight() / 8));
            b.removeFromTop (3);
        }

        startTestButton.setBounds (b.removeFromBottom (b.getHeight() / 10));
        b.removeFromBottom (10);		

		SNR_Value.setBounds(b.removeFromBottom(b.getHeight() / 10));
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

	Slider FFTScaleSlider;

	dsp::FIR::Coefficients<float>::Ptr elo;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ActiveNoiseCancelling)
};
