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

#include "../DemoRunner/Source/NLMS.h"

#include <future>

//==============================================================================
class LoopbackTester : public AudioIODeviceCallback,
						private Thread
{
public:
	LoopbackTester() :Thread("NLMS Processing Thread")
    {
		setPriority(realtimeAudioPriority);
	}
	~LoopbackTester() {
		signalThreadShouldExit();
	}

	void run() override 
	{
		while (!threadShouldExit())
		{
			auto start = std::chrono::high_resolution_clock::now();
			processSamples((float*)inData.getReadPointer(0), (float*)inData.getReadPointer(1));
			auto end = std::chrono::high_resolution_clock::now();
			auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
			wait(-1);
		}
	}

    //==============================================================================
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

    //==============================================================================
    void audioDeviceAboutToStart (AudioIODevice* device) override
    {
		numOfSamples = device->getCurrentBufferSizeSamples();
        testIsRunning = false;
        playingSampleNum = recordedSampleNum = 0;

        sampleRate          = device->getCurrentSampleRate();
        deviceInputLatency  = device->getInputLatencyInSamples();
        deviceOutputLatency = device->getOutputLatencyInSamples();

//		NLMS_Algo = Adaptive::NLMS(numOfSamples);

		inData.setSize(2, numOfSamples);
		outData.setSize(2,numOfSamples);
		inData.clear();
		outData.clear();

		iirCoefficients = dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 440.0f);
		iir.reset(new dsp::IIR::Filter<dsp::SIMDRegister<float>>(iirCoefficients)); // [5]

		interleaved = dsp::AudioBlock<dsp::SIMDRegister<float>>(interleavedBlockData, 2, numOfSamples);
		zero = dsp::AudioBlock<float>(zeroData, dsp::SIMDRegister<float>::size(), numOfSamples); // [6]
		zero.clear();
		
		dsp::ProcessSpec spec;
		spec.numChannels = 2;
		spec.maximumBlockSize = numOfSamples;
		spec.sampleRate = sampleRate;
		iir->prepare(spec); // [7]

		stereoIIR.state = dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, 440.0f);	
		stereoIIR.prepare(spec);


		arm_lms_norm_init_f32(&lmsNorm_instance, 200, lmsNormCoeff_f32, lmsStateF32, 0.001, numOfSamples);
		coeffs = dsp::FIR::Coefficients<float>((const float*)lmsNormCoeff_f32, 200);
		filter = dsp::FIR::Filter<float>(coeffs);
		startThread();
    }

    void audioDeviceStopped() override {}

	void audioDeviceIOCallback(const float** inputChannelData, int numInputChannels,
		float** outputChannelData, int numOutputChannels, int numSamples) override
	{
		const ScopedLock sl(lock);
		auto start = std::chrono::high_resolution_clock::now();

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
		
					outputChannelData[0][i] = (volume / 100.0) * (filter.processSample(inputChannelData[0][i]));
					outputChannelData[1][i] = (volume / 100.0) * (filter.processSample(inputChannelData[1][i]));
		
//					outputChannelData[0][i] = inputChannelData[0][i];
//					outputChannelData[1][i] = inputChannelData[1][i];
		
					++playingSampleNum;
				}		
				notify();
		
				recordedSampleNum = 0;
				playingSampleNum = 0;

//		dsp::AudioBlock<float> inBlock((float*const*)inputChannelData, (size_t)numInputChannels, (size_t)numOfSamples);
//		dsp::AudioBlock<float> outBlock((float*const*)outputChannelData, (size_t)numOutputChannels, (size_t)numOfSamples);
//		auto* inout = channelPointers.getData();
//		auto n = inBlock.getNumSamples();
//
//		for (size_t ch = 0; ch < dsp::SIMDRegister<float>::size(); ++ch) // [10]
//			inout[ch] = (ch < numInputChannels ? const_cast<float*> (inBlock.getChannelPointer(ch)) : zero.getChannelPointer(ch));
//
//		AudioDataConverters::interleaveSamples(inout, reinterpret_cast<float*> (interleaved.getChannelPointer(0)),
//			static_cast<int> (n), static_cast<int> (dsp::SIMDRegister<float>::size())); // [11]
//
//
////		iir->process(dsp::ProcessContextReplacing<dsp::SIMDRegister<float>>(interleaved)); // [12]
//		stereoIIR.process(dsp::ProcessContextReplacing<dsp::SIMDRegister<float>>(interleaved));
//
//		for (size_t ch = 0; ch < inData.getNumChannels(); ++ch) // [13]
//			inout[ch] = outBlock.getChannelPointer(ch);
//
//		AudioDataConverters::deinterleaveSamples(reinterpret_cast<float*> (interleaved.getChannelPointer(0)),
//			const_cast<float**> (inout),
//			static_cast<int> (n), static_cast<int> (dsp::SIMDRegister<float>::size())); // [14]

		auto end = std::chrono::high_resolution_clock::now();
		auto time = end - start;
	}

private:
//    TextEditor& resultsBox;
	Adaptive::NLMS NLMS_Algo;
	AudioBuffer<float> inData, outData;
    CriticalSection lock;

    int playingSampleNum  = 0;
    int recordedSampleNum = -1;
    double sampleRate     = 0.0;
    bool testIsRunning    = false;
    int deviceInputLatency, deviceOutputLatency, numOfSamples;
	float volume;

	arm_lms_norm_instance_f32 lmsNorm_instance;
	float y[FRAMES_PER_BUFFER] = { 0 };			// Output data
	float e[FRAMES_PER_BUFFER] = { 0 };			// Error data
	float Out[FRAMES_PER_BUFFER] = { 0 };
	float errOutput[FRAMES_PER_BUFFER] = { 0 };
	float lmsStateF32[NUM_OF_TAPS + FRAMES_PER_BUFFER] = { 0.1 };
	float lmsNormCoeff_f32[NUM_OF_TAPS] = { 0.1 };

	dsp::FIR::Coefficients<float> coeffs;
	dsp::FIR::Filter<float> filter;



	dsp::IIR::Coefficients<float>::Ptr iirCoefficients;         // [1]
	std::unique_ptr<dsp::IIR::Filter<dsp::SIMDRegister<float>>> iir;
	dsp::AudioBlock<dsp::SIMDRegister<float>> interleaved;           // [2]
	dsp::AudioBlock<float> zero;
	HeapBlock<char> interleavedBlockData, zeroData;        // [3]
	HeapBlock<const float*> channelPointers{ dsp::SIMDRegister<float>::size() };
	dsp::ProcessorDuplicator < dsp::IIR::Filter<dsp::SIMDRegister<float>>, dsp::IIR::Coefficients<float>> stereoIIR;
    //==============================================================================

	inline void processSamples(float *x, float *d) {
		arm_lms_norm_f32(
			&lmsNorm_instance,			/* LMSNorm instance */
			x,							/* Input signal */
			d,							/* Reference Signal */
			Out,						/* Converged Signal */
			errOutput,					/* Error Signal, this will become small as the signal converges */
			numOfSamples);				/* BlockSize */

		memcpy(coeffs.coefficients.begin(), lmsNormCoeff_f32, 200 * sizeof(float));		
	}

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (LoopbackTester)
};

//==============================================================================
class ActiveNoiseCancelling  : public Component,
								public Slider::Listener
{
public:
	ActiveNoiseCancelling()
    {
        setOpaque (true);
	
		volumeLabel.setText("Volume: ", dontSendNotification);		
		volumeLabel.attachToComponent(&volumeSlider, true); // [4]
		volumeLabel.setColour(volumeLabel.textColourId, Colour(255, 255, 255));

		volumeSlider.setRange(0, 100);          // [1]
		volumeSlider.setTextValueSuffix(" %");     // [2]
		volumeSlider.addListener(this);             // [3]
		volumeSlider.setValue(100); // [5]
		volumeSlider.setTextBoxStyle(Slider::TextBoxLeft, false, 160, volumeSlider.getTextBoxHeight());
//		volumeSlider.onValueChange = [this] { latencyTester->setVolume(volumeSlider.getValue()); };

		addAndMakeVisible(volumeLabel);
		addAndMakeVisible(volumeSlider);

        liveAudioScroller.reset (new LiveScrollingAudioDisplay());
		spectrumAnalyser.reset(new AnalyserComponent());
		addAndMakeVisible(spectrumAnalyser.get());
		addAndMakeVisible (liveAudioScroller.get());
		
		addAndMakeVisible(SNR_Value);
		SNR_Value.setColour(SNR_Value.backgroundColourId, Colour(39, 50, 56));
		SNR_Value.setColour(SNR_Value.textColourId, Colour(137, 176, 196));
		SNR_Value.setJustificationType(Justification::centred);
		SNR_Value.setEditable(false);
		SNR_Value.setText("ELO", dontSendNotification);


        //addAndMakeVisible (resultsBox);
        /*resultsBox.setMultiLine (true);
        resultsBox.setReturnKeyStartsNewLine (true);
        resultsBox.setReadOnly (true);
        resultsBox.setScrollbarsShown (true);
        resultsBox.setCaretVisible (false);
        resultsBox.setPopupMenuEnabled (true);

        resultsBox.setColour (TextEditor::outlineColourId, Colour (0x1c000000));
        resultsBox.setColour (TextEditor::shadowColourId,  Colour (0x16000000));

        resultsBox.setText ("Running this test measures the round-trip latency between the audio output and input "
                            "devices you\'ve got selected.\n\n"
                            "It\'ll play a sound, then try to measure the time at which the sound arrives "
                            "back at the audio input. Obviously for this to work you need to have your "
                            "microphone somewhere near your speakers...");*/

        addAndMakeVisible (startTestButton);
        startTestButton.onClick = [this] { startTest(); };

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
        audioDeviceManager.removeAudioCallback (latencyTester    .get());
		audioDeviceManager.removeAudioCallback(spectrumAnalyser.get());
        latencyTester    .reset();
        liveAudioScroller.reset();
		spectrumAnalyser.reset();		
    }

	void sliderValueChanged(Slider* slider) override
	{
		if (slider = &volumeSlider) {
			if (latencyTester.get() != nullptr) {
				latencyTester->setVolume(volumeSlider.getValue());
			}
		}
	}

    void startTest()
    {
        if (latencyTester.get() == nullptr)
        {
            latencyTester.reset (new LoopbackTester());
            audioDeviceManager.addAudioCallback (latencyTester.get());
        }

        latencyTester->beginTest();
    }

    void paint (Graphics& g) override
    {
        g.fillAll (findColour (ResizableWindow::backgroundColourId));
    }

    void resized() override
    {
        auto b = getLocalBounds().reduced (5);

		volumeSlider.setBounds(b.getX() + 120, b.getY(), b.getWidth() - 120, b.getHeight() / 20);
		b.removeFromTop(b.getHeight() / 20);
		b.removeFromTop(3);

        if (liveAudioScroller.get() != nullptr)
        {
            liveAudioScroller->setBounds (b.removeFromTop (b.getHeight() / 3));
            b.removeFromTop (3);
        }

        startTestButton.setBounds (b.removeFromBottom (b.getHeight() / 10));
        b.removeFromBottom (10);

		SNR_Value.setBounds(b.removeFromBottom(b.getHeight() / 10));
		b.removeFromBottom(10);

        //resultsBox.setBounds (b);
		spectrumAnalyser->setBounds(b);
    }

private:
    // if this PIP is running inside the demo runner, we'll use the shared device manager instead
   #ifndef JUCE_DEMO_RUNNER
    AudioDeviceManager audioDeviceManager;
   #else
    AudioDeviceManager& audioDeviceManager { getSharedAudioDeviceManager (2, 2) };	
   #endif

    std::unique_ptr<LoopbackTester> latencyTester;
    std::unique_ptr<LiveScrollingAudioDisplay> liveAudioScroller;
	std::unique_ptr<AnalyserComponent> spectrumAnalyser;

    TextButton startTestButton  { "Test Latency" };
	Label SNR_Value;
    //TextEditor resultsBox;

	Slider volumeSlider;
	Label volumeLabel;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ActiveNoiseCancelling)
};
