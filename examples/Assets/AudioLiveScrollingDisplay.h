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

#pragma once


//==============================================================================
/* This component scrolls a continuous waveform showing the audio that's
   coming into whatever audio inputs this object is connected to.
*/
class LiveScrollingAudioDisplay  : public AudioVisualiserComponent,
                                   public AudioIODeviceCallback
{
public:
    LiveScrollingAudioDisplay()  : AudioVisualiserComponent (2)
    {
        setSamplesPerBlock (256);
        setBufferSize (1024);
		setColours(Colour(39, 50, 56), Colour(137, 176, 196));		
    }

    //==============================================================================
    void audioDeviceAboutToStart (AudioIODevice*) override
    {
        clear();
    }

    void audioDeviceStopped() override
    {
        clear();
    }

	void dcBlocker(float *in, float *out, int blockSize)
	{
		static float xm1, ym1;

		for (int i = 0; i < blockSize; i++)
		{
			out[i] = in[i] - xm1 + 0.995f * ym1;
			xm1 = in[i];
			ym1 = out[i];
		}
	}
    void audioDeviceIOCallback (const float** inputChannelData, int numInputChannels,
                                float** outputChannelData, int numOutputChannels,
                                int numberOfSamples) override
    {
		const ScopedLock sl(lock);
		/*dcBlocker((float*)inputChannelData[0], (float*)inputChannelData[0], numberOfSamples);
		dcBlocker((float*)inputChannelData[1], (float*)inputChannelData[1], numberOfSamples);
#if JUCE_LINUX
		FloatVectorOperations::multiply((float*)inputChannelData[0], 100.0f, numberOfSamples);
		FloatVectorOperations::multiply((float*)inputChannelData[1], 100.0f, numberOfSamples);
#endif*/
		for (int i = 0; i < numberOfSamples; ++i)
		{
			float inputSample[2] = { 0 };

			for (int chan = 0; chan < numInputChannels; ++chan) {
				if (const float* inputChannel = inputChannelData[chan])
					inputSample[chan] = inputChannel[i];  // find the sum of all the channels

				//inputSample[chan] *= 5.0f; // boost the level to make it more easily visible.
			}
			pushSample(inputSample, 2);

			// We need to clear the output buffers before returning, in case they're full of junk..
			for (int j = 0; j < numOutputChannels; ++j)
				if (float* outputChannel = outputChannelData[j])
					zeromem(outputChannel, sizeof(float) * (size_t)numberOfSamples);
		}
    }

private:
	CriticalSection lock;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (LiveScrollingAudioDisplay)
};
