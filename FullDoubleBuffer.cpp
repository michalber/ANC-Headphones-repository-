#include "FullDoubleBuffer.h"

namespace AudioBuffers {

	FullDoubleBuffer::FullDoubleBuffer()
	{
	}


	FullDoubleBuffer::~FullDoubleBuffer()
	{
	}

	void FullDoubleBuffer::MonoBuffer_Init(MonoBuffer *mono_buff)
	{
		arm_fill_f32(0, mono_buff->buffer_M,  BUFFER_SIZE);		
	}

	void FullDoubleBuffer::StereoBuffer_Init(StereoBuffer *stereo_buff)
	{
		arm_fill_f32(0, stereo_buff->Mid, BUFFER_SIZE);
		arm_fill_f32(0, stereo_buff->Side, BUFFER_SIZE);
	}

	void FullDoubleBuffer::DoubleMonoBuffer_Init(DoubleMonoBuffer *doubleMono_buff)
	{
		MonoBuffer_Init(&doubleMono_buff->Buffer_0);
		MonoBuffer_Init(&doubleMono_buff->Buffer_1);
	}

	void FullDoubleBuffer::DoubleStereoBuffer_Init(DoubleStereoBuffer *double_buff)
	{
		StereoBuffer_Init(&double_buff->Buffer_0);
		StereoBuffer_Init(&double_buff->Buffer_1);
	}

	void FullDoubleBuffer::Init()
	{
		//DoubleStereoBuffer_Init(&fullDouble_buff->Double_Buffer_Audio);
		//DoubleMonoBuffer_Init(&Double_Buffer_Mic);
		DoubleStereoBuffer_Init(&Double_Buffer_Mic);
		DoubleStereoBuffer_Init(&Double_Buffer_Out);

		head_0 = 0;
		head_1 = 0;
		select = 0;
	}

	void FullDoubleBuffer::MonoBuffer_Fill(MonoBuffer *mono_buff, int head, float *data)
	{
		mono_buff->buffer_M[head] = ((float)data[0]);
	}

	void FullDoubleBuffer::StereoBuffer_Fill(StereoBuffer *stereo_buff, int head, float *data)
	{
		//stereo_buff->Mid[head] = (float)(data[0] + data[1]) / 2;
		//stereo_buff->Side[head] = (float)(data[0] - data[1]) / 2;
		stereo_buff->Mid[head] = (float)(data[0]);
		stereo_buff->Side[head] = (float)(0);
	}

	void FullDoubleBuffer::StereoBuffer_Pop(StereoBuffer *stereo_buff, int head, float *data)
	{
		data[0] = (float)(stereo_buff->Mid[head]);
		//data[1] = (float)(stereo_buff->Mid[head]);
	}

	int FullDoubleBuffer::FillPop(float *data_mic, float *data_out)
	{
		if (select == 0)
		{
			for (int i = 0; i < 2 * NUM_OF_FRAMES; i++) {
				StereoBuffer_Fill(&Double_Buffer_Mic.Buffer_0, head_0, &data_mic[i]);
				//MonoBuffer_Fill(&Double_Buffer_Mic.Buffer_0, head_0, data_mic);
				StereoBuffer_Pop(&Double_Buffer_Out.Buffer_0, head_0, &data_out[i]);

				//		data_out[0] = (int16_t)(full_double_buff->Double_Buffer_Mic.Buffer_0.buffer_M[full_double_buff->head_0]);
				//		data_out[1] = (int16_t)(full_double_buff->Double_Buffer_Mic.Buffer_0.buffer_M[full_double_buff->head_0]);

				head_0++;
			}
			if (head_0 >= BUFFER_SIZE)
			{
				head_0 = 0;
				select = 1;
				return 1;
			}
		}
		else
		{
			for (int i = 0; i < 2 * NUM_OF_FRAMES; i++) {
				//		StereoBuffer_Fill(&full_double_buff->Double_Buffer_Audio.Buffer_1, full_double_buff->head_1, data_audio);
				StereoBuffer_Fill(&Double_Buffer_Mic.Buffer_1, head_1, &data_mic[i]);
				//MonoBuffer_Fill(&Double_Buffer_Mic.Buffer_1, head_1, data_mic);
				StereoBuffer_Pop(&Double_Buffer_Out.Buffer_1, head_1, &data_out[i]);

				//		data_out[0] = (int16_t)(full_double_buff->Double_Buffer_Mic.Buffer_1.buffer_M[full_double_buff->head_1]);
				//		data_out[1] = (int16_t)(full_double_buff->Double_Buffer_Mic.Buffer_1.buffer_M[full_double_buff->head_1]);

				head_1++;
			}
			if (head_1 >= BUFFER_SIZE)
			{
				head_1 = 0;
				select = 0;
				return 1;
			}
		}
		return 0;
	}

	//StereoBuffer *FullDoubleBuffer_TakeAudioBuff(FullDoubleBuffer *full_double_buff)
	//{
	////	if(full_double_buff->select == 0) return &full_double_buff->Double_Buffer_Audio.Buffer_1;
	////	else return &full_double_buff->Double_Buffer_Audio.Buffer_0;
	//}

	StereoBuffer * FullDoubleBuffer::TakeMicBuff()
	{
		if (select == 0) return &Double_Buffer_Mic.Buffer_1;
		else return &Double_Buffer_Mic.Buffer_0;
	}

	void FullDoubleBuffer::GiveOutBuff(float *Mid, float *Side)
	{
		if (select == 0)
		{
			arm_copy_f32(Mid, Double_Buffer_Out.Buffer_1.Mid, BUFFER_SIZE);
			//		arm_copy_q15(Side, full_double_buff->Double_Buffer_Out.Buffer_1.Side, Sound_Buffer_Size);
		}
		else
		{
			arm_copy_f32(Mid, Double_Buffer_Out.Buffer_0.Mid, BUFFER_SIZE);
			//		arm_copy_q15(Side, full_double_buff->Double_Buffer_Out.Buffer_0.Side, Sound_Buffer_Size);
		}
	}
}