#pragma once
#ifndef _CONFIG_H
#define _CONFIG_H
//-----------------------------------------------------------------------------------------------
#ifndef DEBUG
#define DEBUG 0
#endif // DEBUG
//-----------------------------------------------------------------------------------------------
#ifndef SAMPLE_RATE
#define SAMPLE_RATE   (44100)
#endif //SAMPLE_RATE
//-----------------------------------------------------------------------------------------------
#ifndef FRAMES_PER_BUFFER
#define FRAMES_PER_BUFFER  (32)
#endif //FRAMES_PER_BUFFER
//-----------------------------------------------------------------------------------------------
#ifndef BUFFER_SIZE
#define BUFFER_SIZE  (3*FRAMES_PER_BUFFER)
#endif  //BUFFER_SIZE
//-----------------------------------------------------------------------------------------------
#ifndef NUM_OF_TAPS
#define NUM_OF_TAPS  100
#endif  //NUM_OF_TAPS
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
#endif // USE_VECTORS
//-----------------------------------------------------------------------------------------------

#endif // !_CONFIG_H

