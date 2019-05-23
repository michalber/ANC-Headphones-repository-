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
//////#include <iostream>
//////#include <iomanip>
//////#include <fstream>
//////#include <vector>
//////
//////int main()
//////{	
//////	std::ofstream fout("Out.raw", std::ios::binary);
//////	
//////	std::ifstream fin1("Noise.raw", std::ios::binary);
//////	if (!fin1)
//////	{
//////		std::cout << " Error, Couldn't find the file" << "\n";
//////		return 0;
//////	}
//////	std::ifstream fin2("Taco+n.raw", std::ios::binary);
//////	if (!fin2)
//////	{
//////		std::cout << " Error, Couldn't find the file" << "\n";
//////		return 0;
//////	}
//////	std::ifstream fin3("Taco.raw", std::ios::binary);
//////	if (!fin2)
//////	{
//////		std::cout << " Error, Couldn't find the file" << "\n";
//////		return 0;
//////	}
//////
//////	fin1.seekg(0, std::ios::end);
//////	const size_t num_elements = fin1.tellg() / sizeof(double);
//////	fin1.seekg(0, std::ios::beg);
//////
//////	std::vector<double> data1(num_elements);
//////	std::vector<double> data2(num_elements);
//////	std::vector<double> data3(num_elements);
//////
//////	fin1.read(reinterpret_cast<char*>(&data1[0]), num_elements * sizeof(double));
//////	fin2.read(reinterpret_cast<char*>(&data2[0]), num_elements * sizeof(double));
//////	fin3.read(reinterpret_cast<char*>(&data3[0]), num_elements * sizeof(double));
//////
//////	for (size_t i = 0; i < data1.size(); i++)
//////	{				
//////		double temp = data2[i] - RLMS_Algo.process(data2[i], data1[i],data3[i]);
//////		fout.write(reinterpret_cast<const char*>(&temp), sizeof(temp));
//////	}
//////	fout.close();
//////
//////	//getchar();
//////	return 0;
//////}
////
//////
//////#include "ANC_System.h"
//////#include "plot.h"
//////#include <sigpack.h>
//////
//////
//////int main() {
//////	//ANC::ANC_System LocalANCSystem;
//////
//////
//////
//////	//thread updateInputBuffers_Thread(fnc);
//////	//thread processDataWithRLMS_Thread = LocalANCSystem->member2Thread();
//////	//thread updateOutputBuffer_Thread = LocalANCSystem->member3Thread();
//////
//////	//thread thread1([&](ANC_System::ANC_System * ANC_System) { ANC_System->updateInputBuffers_Func(); }, &LocalANCSystem);
//////
//////	//updateInputBuffers_Thread.join();
//////	//processDataWithRLMS_Thread.join();
//////	//updateOutputBuffer_Thread.join();
//////
//////
//////	getchar();
//////	return 0;
//////}
////
////
////#include <chrono>
////#include <portaudio.h>
////#include "ANC_System.h"
////
////using namespace std;
////using namespace sp;
////
////
////
////
////Adaptive::NLMS NLMS_Algo(40);
////
////	// Number of samples
////	int N = 32;
////
////	//Create a FIR filter and adaptive
////	FIR_filt<double, double, double> G;
////
////	// Filter coeffs.
////	arma::vec b = "-0.2 -0.1 0.1 0.3 0.7";
////
////
////	// Signal vectors
////	arma::vec x(N, arma::fill::randn);  // Input sig
////	arma::vec d(N, arma::fill::zeros);  // Output sig
////	arma::vec z(N, arma::fill::randn);  // Measurement noise
////
////
////
////	atomic<bool> newdata = 0;
////	atomic<bool> run = 1;
////
////	void update_f() {
////		for (int n = 0; n < 10000; n++)
////		{
////			auto start = chrono::high_resolution_clock::now();
////			NLMS_Algo.updateNLMS(x, d);			
////			auto stop = chrono::high_resolution_clock::now();
////			cout << double(chrono::duration_cast<chrono::nanoseconds>(stop - start).count()) / 1000 << " ";
////			cout << (double)(chrono::duration_cast<chrono::nanoseconds>(stop - start).count() / 32)/1000 << endl;
////
////			newdata.store(1);
////
////			x = arma::vec(N, arma::fill::randn);
////			z = arma::vec(N, arma::fill::randn);
////			d = G.filter(x) + 0.0001*z;
////
////			this_thread::sleep_for(chrono::milliseconds(10));
////		}	
////		run.store(0);
////	}
////	void update_data() {
////		while (run == 1) {
////			NLMS_Algo.drawData();
////			
////			newdata.store(0);
////			this_thread::sleep_for(chrono::milliseconds(500));
////		}
////	}
////
////
////	int main() {
////
////		Pa_Initialize();
////
////		G.set_coeffs(b);
////		d = G.filter(x) + 0.0001*z;
////
////		thread run(update_f);
////		thread rundata(update_data);
////
////		run.join();
////		rundata.join();
////		std::cin.ignore();
////
////		Pa_Terminate();
////		return 0;
////	}


#include "ANC_System.h"


int main() {
	ANC::ANC_System init;

	std::cin.ignore();
	return 0;
}