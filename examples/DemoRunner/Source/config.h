/*
  ==============================================================================

    config.h
    Created: 24 Sep 2019 12:08:51pm
    Author:  michu

  ==============================================================================
*/

#pragma once

#pragma once

#ifndef DEBUG
#define DEBUG 0
#endif // DEBUG
//-----------------------------------------------------------------------------------------------
#ifndef SAMPLE_RATE
#define SAMPLE_RATE   (44100)
#endif //SAMPLE_RATE
//-----------------------------------------------------------------------------------------------
// At least 2x NUM_OF_FRAMES
#ifndef FRAMES_PER_BUFFER
#define FRAMES_PER_BUFFER  (2048)
#endif //FRAMES_PER_BUFFER
//-----------------------------------------------------------------------------------------------
#ifndef NUM_OF_FRAMES
#define NUM_OF_FRAMES  (FRAMES_PER_BUFFER / 2)
#endif //NUM_OF_FRAMES
//-----------------------------------------------------------------------------------------------
#ifndef BUFFER_SIZE
#define BUFFER_SIZE  (FRAMES_PER_BUFFER)
#endif  //BUFFER_SIZE
//-----------------------------------------------------------------------------------------------
#ifndef NUM_OF_TAPS
#define NUM_OF_TAPS  1536
#endif  //NUM_OF_TAPS
//-----------------------------------------------------------------------------------------------
#ifndef DELTA_ERROR         
#define DELTA_ERROR 0.0000001f
#endif //DELTA_ERROR
//-----------------------------------------------------------------------------------------------
#ifndef DELTA_COEFF         
#define DELTA_COEFF 0.00004f
#endif //DELTA_COEFF
//-----------------------------------------------------------------------------------------------
#ifndef MU                 
#define MU  0.0005f
#endif //MU
//-----------------------------------------------------------------------------------------------
#ifndef TIME_DEBUG
#define TIME_DEBUG 0
#endif // TIME_DEBUG
//-----------------------------------------------------------------------------------------------
#if TIME_DEBUG
#endif
//-----------------------------------------------------------------------------------------------
#ifndef PLOT_DATA
#define PLOT_DATA 0
#endif
//-----------------------------------------------------------------------------------------------
#ifndef DATA_PATH
//#define DATA_PATH "C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\"
#define DATA_PATH "/home/pi/ANC-Headphones-repository-/data/"
#endif
//-----------------------------------------------------------------------------------------------
#ifndef PRINT_AUDIO_DEV_INFO
#define PRINT_AUDIO_DEV_INFO 0
#endif // PRINT_AUDIO_DEV_INFO
//-----------------------------------------------------------------------------------------------
#define RUN_ANC 0
#define GENERATE_AUDIO 1
#define RUN_SNR_TESTS 2

#ifndef INTERNAL_TEST
#define INTERNAL_TEST RUN_ANC
#endif // INTERNAL_TEST
//-----------------------------------------------------------------------------------------------
