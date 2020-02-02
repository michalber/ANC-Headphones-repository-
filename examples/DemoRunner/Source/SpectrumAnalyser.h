/*
  ==============================================================================

   This file is part of the JUCE tutorials.
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

 name:             SpectrumAnalyserTutorial
 version:          2.0.0
 vendor:           JUCE
 website:          http://juce.com
 description:      Displays an FFT spectrum analyser.

 dependencies:     juce_audio_basics, juce_audio_devices, juce_audio_formats,
				   juce_audio_processors, juce_audio_utils, juce_core,
				   juce_data_structures, juce_dsp, juce_events, juce_graphics,
				   juce_gui_basics, juce_gui_extra
 exporters:        xcode_mac, vs2017, linux_make

 type:             Component
 mainClass:        AnalyserComponent

 useLocalCopy:     1

 END_JUCE_PIP_METADATA

*******************************************************************************/


#pragma once

#include "../../../examples/Assets/DemoUtilities.h"

//==============================================================================
class AnalyserComponent : public AudioIODeviceCallback,	
							public Component,
							private Timer
{
public:
	/***************************************************************************//**
	 * @brief Default constructor of AnalyserComponent class
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param Value to set or disable parralerism of filters
	 * @return
	 ******************************************************************************/
	AnalyserComponent() : forwardFFT_L(fftOrder), forwardFFT_P(fftOrder),
							window_L(fftSize, dsp::WindowingFunction<float>::hann), window_P(fftSize, dsp::WindowingFunction<float>::hann)
	{
		setOpaque(true);
		startTimerHz(50);			
	}
	/***************************************************************************//**
	 * @brief Overriden function to set audio device parameters before it starts
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param Pointer to IO Audio Device
	 * @return
	 ******************************************************************************/
	void audioDeviceAboutToStart(AudioIODevice*) override
	{		
	}
	/***************************************************************************//**
	 * @brief Overriden function to clean audio device parameters after it stops
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param 
	 * @return
	 ******************************************************************************/
	void audioDeviceStopped() override
	{	
	}
	/***************************************************************************//**
	 * @brief Callback of IO Audio Device 
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param Double pointer to input data
	 * @param Number of input channels
	 * @param Double pointer to input data
	 * @param Number of output channels
	 * @param Number of samples
	 * @return
	 ******************************************************************************/
	void audioDeviceIOCallback(const float** inputChannelData, int numInputChannels,
		float** outputChannelData, int numOutputChannels,
		int numberOfSamples) override
	{
		(void)numOutputChannels;
		(void)numInputChannels;
		(void)outputChannelData;


		const ScopedLock sl(lock);

		for (int i = 0; i < numberOfSamples; ++i)
		{
//			float inputSample[2] = { 0 };

			pushNextSampleIntoFifo(inputChannelData[0][i], 0);
			pushNextSampleIntoFifo(inputChannelData[1][i], 1);

		}
	}
	/***************************************************************************//**
	 * @brief Function to set scale min and max value
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param Minimal value of scale
	 * @param Maximal value of scale
	 * @return
	 ******************************************************************************/
	void setScaleValue(float valMin, float valMax) {
		scaleValueMin = valMin;
		scaleValueMax = valMax;
	}
	/***************************************************************************//**
	 * @brief Overriden function to draw new data on screen
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param Reference to graphic module
	 * @return
	 ******************************************************************************/
	void paint(Graphics& g) override
	{
		g.fillAll(Colour(39, 50, 56));
		g.setOpacity(1.0f);
		g.setColour(Colour(137, 176, 196));
		drawFrame(g);
	}
	/***************************************************************************//**
	 * @brief function of timer callback - specifies what to do when timer is called
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param
	 * @return
	 ******************************************************************************/
	void timerCallback() override
	{
		if (nextFFTBlockReady_L)
		{
			drawNextFrameOfSpectrum();
			nextFFTBlockReady_L = false;
			nextFFTBlockReady_P = false;
			repaint();
		}
	}
	/***************************************************************************//**
	 * @brief Function to put new data od FIFO queue
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param New sample value
	 * @param Channel to put sample to
	 * @return
	 ******************************************************************************/
	void pushNextSampleIntoFifo(float sample, bool channel) noexcept
	{
		// if the fifo contains enough data, set a flag to say
		// that the next frame should now be rendered..
		if (!channel) {
			if (fifoIndex_L == fftSize)
			{
				if (!nextFFTBlockReady_L)
				{
					zeromem(fftData_L, sizeof(fftData_L));
					memcpy(fftData_L, fifo_L, sizeof(fifo_L));
					nextFFTBlockReady_L = true;
				}

				fifoIndex_L = 0;
			}

			fifo_L[fifoIndex_L++] = sample;
		}
		if(channel) {
			if (fifoIndex_P == fftSize)
			{
				if (!nextFFTBlockReady_P)
				{
					zeromem(fftData_P, sizeof(fftData_P));
					memcpy(fftData_P, fifo_P, sizeof(fifo_P));
					nextFFTBlockReady_P = true;
				}

				fifoIndex_P = 0;
			}

			fifo_P[fifoIndex_P++] = sample;
		}
	}
	/***************************************************************************//**
	 * @brief Function to draw new graph on screen
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param 
	 * @return
	 ******************************************************************************/
	void drawNextFrameOfSpectrum()
	{
		// first apply a windowing function to our data
		window_L.multiplyWithWindowingTable(fftData_L, fftSize);
		window_P.multiplyWithWindowingTable(fftData_P, fftSize);

		// then render our FFT data..
		forwardFFT_L.performFrequencyOnlyForwardTransform(fftData_L);
		forwardFFT_P.performFrequencyOnlyForwardTransform(fftData_P);

		auto mindB = scaleValueMin;
		auto maxdB = scaleValueMax;

		for (int i = 0; i < scopeSize; ++i)
		{
			auto skewedProportionX = 1.0f - std::exp(std::log(1.0f - i / (float)scopeSize) * 0.2f);
			auto fftDataIndex = jlimit(0, fftSize / 2, (int)(skewedProportionX * fftSize / 2));
			auto level = jmap(jlimit(mindB, maxdB, Decibels::gainToDecibels(fftData_L[fftDataIndex])
				- Decibels::gainToDecibels((float)fftSize)),
				mindB, maxdB, 0.0f, 1.0f);

			scopeData_L[i] = level;

			level = jmap(jlimit(mindB, maxdB, Decibels::gainToDecibels(fftData_P[fftDataIndex])
				- Decibels::gainToDecibels((float)fftSize)),
				mindB, maxdB, 0.0f, 1.0f);

			scopeData_P[i] = level;
		}
	}
	/***************************************************************************//**
	 * @brief Function to draw new frame on screen
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param Reference to graphics module
	 * @return
	 ******************************************************************************/
	void drawFrame(Graphics& g)
	{
		auto width = getLocalBounds().getWidth() - 20;
		auto height = getLocalBounds().getHeight();

//		g.setColour(findColour(ResizableWindow::backgroundColourId));
//		for (int i = 0; i < width; i+= width/50)
//		{			
//			g.drawVerticalLine(i, 0, height);
//		}
//		g.setColour(Colours::dimgrey);
//		for (int i = 0; i < width; i += width / 10)
//		{
////			g.drawText(String(i), Rectangle<float>(i - 30, 10, 50, 10), Justification::centred);
//		}

		for (int i = 1; i < scopeSize; ++i)
		{
			g.setColour(Colour(137, 176, 196));
			g.drawLine({ (float)jmap(i - 1, 0, scopeSize - 1, 0, width),
								  jmap(scopeData_L[i - 1], 0.0f, 1.0f, (float)height, 0.0f),
						  (float)jmap(i,     0, scopeSize - 1, 0, width),
								  jmap(scopeData_L[i],     0.0f, 1.0f, (float)height, 0.0f) });

			g.setColour(Colour(204, 238, 255));
			g.drawLine({ (float)jmap(i - 1, 0, scopeSize - 1, 0, width),
					  jmap(scopeData_P[i - 1], 0.0f, 1.0f, (float)height, 0.0f),
			  (float)jmap(i,     0, scopeSize - 1, 0, width),
					  jmap(scopeData_P[i],     0.0f, 1.0f, (float)height, 0.0f) });
		}
	}
	/***************************************************************************//**
	 * @brief Enum specyfies FFT settings
	 * @author Micha³ Berdzik
	 * @version 1.0 26/09/2019
	 * @param Order of FFT
	 * @param Size of FFT
	 * @param Scope size
	 * @return
	 ******************************************************************************/
	enum
	{
		fftOrder = 11,
		fftSize = 1 << fftOrder,
		scopeSize = 1024
	};

private:
	CriticalSection lock;

	float scaleValueMin = -100.0f;
	float scaleValueMax = 0.0f;

	dsp::FFT forwardFFT_L;
	dsp::WindowingFunction<float> window_L;
	float fifo_L[fftSize];
	float fftData_L[2 * fftSize];
	int fifoIndex_L = 0;
	bool nextFFTBlockReady_L = false;
	float scopeData_L[scopeSize];

	dsp::FFT forwardFFT_P;
	dsp::WindowingFunction<float> window_P;
	float fifo_P[fftSize];
	float fftData_P[2 * fftSize];
	int fifoIndex_P = 0;
	bool nextFFTBlockReady_P = false;
	float scopeData_P[scopeSize];


	JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(AnalyserComponent)
};