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
#define FRAMES_PER_BUFFER  (1024)
#endif //FRAMES_PER_BUFFER
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
#define DATA_PATH "C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\"
#endif
//-----------------------------------------------------------------------------------------------

#endif // !_CONFIG_H

