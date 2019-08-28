#pragma once
#include "config.h"
#include "arm_math.h"

namespace AudioBuffers {

	typedef struct {
		float buffer_M[BUFFER_SIZE];
	}MonoBuffer;

	typedef struct {
		float Mid[BUFFER_SIZE];
		float Side[BUFFER_SIZE];
	}StereoBuffer;

	typedef struct {
		MonoBuffer Buffer_0, Buffer_1;
	}DoubleMonoBuffer;

	typedef struct {
		StereoBuffer Buffer_0, Buffer_1;
	}DoubleStereoBuffer;

	class FullDoubleBuffer
	{
		DoubleStereoBuffer Double_Buffer_Mic;
		DoubleStereoBuffer Double_Buffer_Out;
		volatile int head_0, head_1;
		volatile int select;

	public:
		FullDoubleBuffer();
		~FullDoubleBuffer();

		void Init();
		int FillPop(float *data_mic, float *data_out);
		StereoBuffer *TakeAudioBuff();
		StereoBuffer *TakeMicBuff();
		void GiveOutBuff(float *Mid, float *Side);

	private:
		void MonoBuffer_Init(MonoBuffer *mono_buff);
		void StereoBuffer_Init(StereoBuffer *stereo_buff);
		void DoubleMonoBuffer_Init(DoubleMonoBuffer *doubleMono_buff);
		void DoubleStereoBuffer_Init(DoubleStereoBuffer *double_buff);
		void MonoBuffer_Fill(MonoBuffer *mono_buff, int head, float *data);
		void StereoBuffer_Fill(StereoBuffer *stereo_buff, int head, float *data);
		void StereoBuffer_Pop(StereoBuffer *stereo_buff, int head, float *data);
	};
}
