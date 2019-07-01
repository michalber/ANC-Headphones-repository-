////
////
//////#include "RLMS.h"
//////#include "ring_buffer.h"
////
////
//////#define DEBUG 0
//////#define TIME_DEBUG 0
////
////
//////
//////enum SAMPLING_FREQ {
//////	f44KHz = 44100,
//////	f48KHz = 48000,
//////	f96KHz = 96000,
//////	f192KHz = 192000
//////};
//////
//////const float currentSamplinFreq = f44KHz;
//////
//////Adaptive::RLMS RLMS_Algo(45, 0.9999);
//////RingBuffer::RingBuffer<double> NoiseBuffer(32);
//////RingBuffer::RingBuffer<double> ErrorBuffer(32);
//////RingBuffer::RingBuffer<double> OutputBuffer(32);
//////atomic<bool> stopThreads = 0;
//////
//////void callFromThread() {
//////	while (!stopThreads) {
//////		auto timePoint =
//////			chrono::high_resolution_clock::now() + chrono::nanoseconds(long long(1 / currentSamplinFreq * 1e9));
//////		
//////		OutputBuffer.RingBuffer_Get();
//////		//cout << OutputBuffer.RingBuffer_Get() << endl;
//////		
//////		this_thread::sleep_until(timePoint);
//////	}
//////}
//////
//////void callFromRLMSTask() {
//////	while (!stopThreads) {
//////		auto timePoint =
//////			chrono::high_resolution_clock::now() + chrono::nanoseconds(long long(1 / currentSamplinFreq * 1e9));
//////
//////		OutputBuffer.RingBuffer_Put(
//////			RLMS_Algo.process(NoiseBuffer.RingBuffer_Get(),
//////				ErrorBuffer.RingBuffer_Get(),0));
//////
//////		OutputBuffer.RingBuffer_GetLen();
//////		//cout << "Length of OutBuff is: " << OutputBuffer.RingBuffer_GetLen() << endl;
//////
//////		std::this_thread::sleep_until(timePoint);
//////	}
//////}
//////
//////void CallFromInput() {
//////	while (!stopThreads) {
//////		auto timePoint =
//////			chrono::high_resolution_clock::now() + chrono::nanoseconds(long long(1 / currentSamplinFreq * 1e9));
//////
//////		int i = randn();
//////		if (!NoiseBuffer.RingBuffer_IsFull())
//////			NoiseBuffer.RingBuffer_Put(i);
//////		if (!ErrorBuffer.RingBuffer_IsFull())
//////			ErrorBuffer.RingBuffer_Put(2 * i + 1);
//////
//////		//cout << "Length of ErrorBuff is: " << ErrorBuffer.RingBuffer_GetLen() << endl;
//////		//cout << "Length of NoiseBuff is: " << NoiseBuffer.RingBuffer_GetLen() << endl;
//////
//////		std::this_thread::sleep_until(timePoint);
//////	}
//////}
//////
//////
//////	double tab_d[] = { 1.08934248061136,
//////0.858150672550494,
//////0.979327853383033,
//////1.04549966914263 ,
//////0.988829960017101,
//////0.892354699012372,
//////0.744453930227002,
//////0.867858720477035,
//////0.972020470358927,
//////0.993673450103631,
//////1.06279114166221 ,
//////1.12479772612034 ,
//////0.936883712739629,
//////1.02662291253611 ,
//////0.845448854124272,
//////0.837792188705902,
//////1.14981595734606 ,
//////1.05022730531698 ,
//////0.851698935384489,
//////0.841656311471221 };
//////	double tab_x[] = { 0.498892810948575,
//////0.448138768061242,
//////0.475185400478582,
//////0.490046586417158,
//////0.478176271207648,
//////0.457558719285053,
//////0.425477475717332,
//////0.453940625932368,
//////0.478080715194195,
//////0.484092442367921,
//////0.500518344419215,
//////0.515335850706169,
//////0.476313136529277,
//////0.497732984927178,
//////0.459987709763479,
//////0.460440570678585,
//////0.530164334993513,
//////0.511426165595184,
//////0.470779731532465,
//////0.471252991969380 };
//////
//////int main() {
//////	thread t1(CallFromInput);
//////	thread t2(callFromRLMSTask);
//////	thread t3(callFromThread);
//////
//////
//////	double result = 0;
//////	int i = 0;
//////	for (int j = 0; j < 1; j++) {
//////		for (i = 0; i < 20; i++) {		
//////			/*OutputBuffer.RingBuffer_Put(
//////				RLMS_Algo.process(NoiseBuffer.RingBuffer_Get(),
//////							 ErrorBuffer.RingBuffer_Get()) );*/
//////			//cout << RLMS_Algo.process(tab_d[i], tab_x[i],1) << endl;
//////		}
//////
//////		/*for (i = 0; i < f48KHz * 2; i++) {
//////			auto start = std::chrono::system_clock::now();
//////
//////			OutputBuffer.RingBuffer_Get();
//////
//////			auto end = std::chrono::system_clock::now();
//////			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//////
//////			result += duration.count();
//////		}
//////		cout << "Total:" << result / 1000000 << endl;
//////		cout << "Srednia: " << result / i << endl;
//////		result = 0;*/
//////	}
//////
//////	t1.join();
//////	t2.join();
//////	t3.join();
//////
//////	
//////	getchar();
//////	return 0;
//////}
////


//#include <iostream>
//#include <iomanip>
//#include <fstream>
//#include <vector>
//#include "../ANC-Headphones-repository-/RLMS.h"
//#include "../ANC-Headphones-repository-/NLMS.h"
//
//int main()
//{	
//	RLMS::RLMS RLMS(20, 0.999);
//	Adaptive::NLMS NLMS(128,0.03,0.001);
//
//	std::ofstream fout("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\Out.raw", std::ios::binary);
//	
//	std::ifstream fin1("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\NoiseT.raw", std::ios::binary);
//	if (!fin1)
//	{
//		std::cout << " Error, Couldn't find the file" << "\n";
//		return 0;
//	}
//	std::ifstream fin2("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\Taco+n.raw", std::ios::binary);
//	if (!fin2)
//	{
//		std::cout << " Error, Couldn't find the file" << "\n";
//		return 0;
//	}
//	std::ifstream fin3("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\Taco.raw", std::ios::binary);
//	if (!fin2)
//	{
//		std::cout << " Error, Couldn't find the file" << "\n";
//		return 0;
//	}
//
//	fin1.seekg(0, std::ios::end);
//	const size_t num_elements = fin1.tellg() / sizeof(float);
//	fin1.seekg(0, std::ios::beg);
//
//	std::vector<float> data1(num_elements);
//	std::vector<float> data2(num_elements);
//	std::vector<float> data3(num_elements);
//
//	fin1.read(reinterpret_cast<char*>(&data1[0]), num_elements * sizeof(float));
//	fin2.read(reinterpret_cast<char*>(&data2[0]), num_elements * sizeof(float));
//	fin3.read(reinterpret_cast<char*>(&data3[0]), num_elements * sizeof(float));
//
//	for (size_t i = 0; i < data1.size(); i++)
//	{				
//		//float temp = RLMS.processNLMS(data2[i], data1[i], data3[i]);
//		float temp = NLMS.updateNLMS(data2[i], data1[i], data3[i]);
//		fout.write(reinterpret_cast<const char*>(&temp), sizeof(temp));
//	}
//	fout.close();
//
//	//getchar();
//	return 0;
//}



#include "../ANC-Headphones-repository-/ANC_System.h"

std::vector<float> getCircBufferAsVec(boost::circular_buffer<float> *buff)
{
	/*
	max time: 1ms
	*/	
	std::vector<float> temp(FRAMES_PER_BUFFER);
	temp.clear();
	//if (buff->size() >= FRAMES_PER_BUFFER) {
	for (int i = 0; i < FRAMES_PER_BUFFER; i++) {
		temp.push_back(buff->back());
		buff->pop_back();
	}
	//}
	return temp;
}

void putVecIntoCircBuffer(boost::circular_buffer<float> *buff, std::vector<float> in)
{
	/*
	max time = 2ms
	*/
	for (int i = 0; i < in.size(); i++) {
		buff->push_front(in[i]);
	}
}

#include "../ANC-Headphones-repository-/RLMS.h"

int main() {
	ANC::ANC_System init;

//	std::ofstream fout("C:\\Users\\michu\\source\\repos\\ANC_inz_v0\\ANC_inz_v0\\ANC-Headphones-repository-\\data\\Out.raw", std::ios::binary);
//
//	AI::AudioInterface AudioOutput;
//	Handler::ScopedPaHandler paInit;
//
//	boost::circular_buffer<float> NIB;
//	boost::circular_buffer<float> EIB;
//	boost::circular_buffer<float> MOB;
//
//	AS::AudioStream Music;
//	AS::AudioStream Noise;
//	AS::AudioStream FilteredNoise;
//	AS::AudioStream MusicNoise;
//
//	Adaptive::NLMS NLMS_Algorithm(150, 0.1, 0.001);
//	RLMS::RLMS RLMS(400,1);
//	
//	std::vector<float> x;
//	std::vector<float> d;
//
//	NIB.set_capacity(FRAMES_PER_BUFFER);
//	EIB.set_capacity(FRAMES_PER_BUFFER);
//	MOB.set_capacity(FRAMES_PER_BUFFER);
//
//	Music.openFile(std::string(DATA_PATH) + "Taco.raw");
//	Music.setUpBuffer(&MOB);
//
//	//Noise.openFile(std::string(DATA_PATH) + "NoiseT.raw");
//	Noise.openFile(std::string(DATA_PATH) + "x.raw");
//	Noise.setUpBuffer(&NIB);
//
//	//MusicNoise.openFile(std::string(DATA_PATH) + "Sin+n.raw");
//	MusicNoise.openFile(std::string(DATA_PATH) + "d.raw");
//	MusicNoise.setUpBuffer(&EIB);
//
//	AudioOutput.setUpBuffer(&MOB);
//
//	if (paInit.result() != paNoError) {
//		fprintf(stderr, "An error occured while using the portaudio stream\n");
//		fprintf(stderr, "Error number: %d\n", paInit.result());
//		fprintf(stderr, "Error message: %s\n", Pa_GetErrorText(paInit.result()));
//	}
//
//	if (AudioOutput.open(Pa_GetDefaultOutputDevice())) {
//		if (AudioOutput.start()) {
//#if DEBUG
//			cout << "AudioOutput Started working" << endl;
//#endif // DEBUG
//		}
//	}
//
//
//	auto start = std::chrono::system_clock::now();
//	for (int i = 0; i < 1500; i++) {
//
//		//updateNoiseBuffer();
//		Noise.updateBuffer();
//		//updateErrorBuffer();
//		MusicNoise.updateBuffer();
//		//Music.updateBuffer();
//		//loadNewNoiseVector();
//		x = getCircBufferAsVec(&NIB);
//		d = getCircBufferAsVec(&EIB);
//		//NLMS_Algorithm.updateNLMS(d, x, d);
//		//updateOutputBuffer();
//		//x = NLMS_Algorithm.getErrorVector();
//		//std::vector<float> temp = NLMS_Algorithm.getOutVector();
//		
//		std::vector<float> temp;
//		for (int k = 0; k < FRAMES_PER_BUFFER; k++) {
//			//temp.push_back(RLMS.processNLMS(d[k], x[k], x[k]));		
//			//MOB.push_front(RLMS.processNLMS(d[k], x[k], x[k]));
//			Music.updateBufferOnce();
//		}
//		
//		//for (int j = 0; j < FRAMES_PER_BUFFER; j++)
//		//	fout.write(reinterpret_cast<const char*>(&temp[j]), sizeof(float));
//
//		x.clear();
//		d.clear();
//
//		//drawNLMSData();
//		//NLMS_Algorithm.drawData(x, d, "x", "d");
//		//NLMS_Algorithm.drawData();
//	}
//	auto end = std::chrono::system_clock::now();
//	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//	cout << duration.count() / 1500 << endl;
//	fout.close();

	std::cin.ignore();
	return 0;
}
