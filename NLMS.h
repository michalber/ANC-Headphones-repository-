#pragma once
#ifndef _NLMS_H_
#define _NLMS_H_

//-----------------------------------------------------------------------------------------------
#include <sigpack.h>
#include <mutex>
#include "config.h"
//-----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------
namespace Adaptive {

	class NLMS
	{
		std::mutex mut;
		int pNumOfTaps;
		double stepSize;
		double RFactor;
		// Port Audio FIR filter 
		sp::FIR_filt<double, double, double> AdaptiveFilter;

		// Signal vectors
		arma::vec y;  // Output data
		arma::vec e;  // Error data
		std::vector<float> y_v;  // Output data
		std::vector<float> e_v;  // Error data
		float y_f;
		float e_f;

#if PLOT_DATA
		sp::gplot gpErr;
		sp::gplot gpOut;
#endif

	public:


		NLMS() :pNumOfTaps(60), stepSize(0.5), RFactor(0.001)
		{
			y = arma::vec(FRAMES_PER_BUFFER, arma::fill::zeros);  // Model sig	
			e = arma::vec(FRAMES_PER_BUFFER, arma::fill::zeros);  // Err sig

			AdaptiveFilter.setup_nlms(pNumOfTaps, stepSize, RFactor);

#if PLOT_DATA
			gpErr.window("Error", 10, 10, 700, 500);
			gpOut.window("Out", 700, 10, 700, 500);
			//gp0.ylim(-0.5, 0.5);
#endif
		}
		//--------------------------------------------------------------------------------
		/**
				@brief Default constructor of NLMS class

				@author	Micha³ Berdzik
				@version 0.0.1 18-05-2019

			*/
		NLMS(int Taps, double stepsize, double rfactor) :pNumOfTaps(Taps), stepSize(stepsize), RFactor(rfactor)
		{
			y = arma::vec(FRAMES_PER_BUFFER, arma::fill::zeros);  // Model sig	
			e = arma::vec(FRAMES_PER_BUFFER, arma::fill::zeros);  // Err sig

			AdaptiveFilter.setup_nlms(pNumOfTaps, stepSize, RFactor);			

#if PLOT_DATA
			gpErr.window("Error", 10, 10, 700, 500);
			gpOut.window("Out", 700, 10, 700, 500);
			//gp0.ylim(-0.5, 0.5);
#endif
		}
		//--------------------------------------------------------------------------------
		/**
				@brief Destructor of NLMS class

				@author	Micha³ Berdzik
				@version 0.0.1 18-05-2019

			*/
		~NLMS()
		{
		}

		void setParameters(int Taps, double stepsize, double rfactor)
		{
			y = arma::vec(FRAMES_PER_BUFFER, arma::fill::zeros);  // Model sig	
			e = arma::vec(FRAMES_PER_BUFFER, arma::fill::zeros);  // Err sig

			AdaptiveFilter.setup_nlms(Taps, stepsize, rfactor);
		}
		//--------------------------------------------------------------------------------
		/**
				@brief Function to perform NLMS algorithm on given data x and d

				@author	Micha³ Berdzik
				@version 0.0.1 18-05-2019

				@param arma::vec x  -  signal of readed noise
				@param arma::vec d  -  signal of readed music+noise
			*/
		void updateNLMS(arma::vec d, arma::vec x, arma::vec m)
		{
			std::lock_guard<std::mutex> lk(mut);
			for (int n = 0; n < FRAMES_PER_BUFFER; n++)
			{
				// Apply adaptiv filter
				y(n) = AdaptiveFilter(x(n));

				// Calc error
				//e(n) = (d(n) - m(n)) - y(n);
				e(n) = (d(n)) - y(n);

				// Update filter
				AdaptiveFilter.nlms_adapt(e(n));
			}
		}
		void updateNLMS(std::vector<float> d, std::vector<float> x, std::vector<float> m)
		{
			std::lock_guard<std::mutex> lk(mut);
			y_v.clear();
			e_v.clear();
			for (int n = 0; n < FRAMES_PER_BUFFER; n++)
			{
				// Apply adaptiv filter
				y_v.push_back(AdaptiveFilter(x[n]));

				// Calc error
				//e(n) = (d(n) - m(n)) - y(n);
				e_v.push_back((d[n]) - y_v[n]);

				// Update filter
				AdaptiveFilter.nlms_adapt(e_v[n]);
			}
		}
		double updateNLMS(double d, double x, double m)
		{
			std::lock_guard<std::mutex> lk(mut);
			// Apply adaptiv filter
			y_f = AdaptiveFilter(x);

			// Calc error
			//e_f = (d - m) - y_f;
			e_f = (d) - y_f;

			// Update filter
			AdaptiveFilter.nlms_adapt(e_f);

			return e_f;
		}


		//--------------------------------------------------------------------------------
		/**
				@brief Function to draw error data using Sigpack library

				@author	Micha³ Berdzik
				@version 0.0.1 18-05-2019

			*/
		void drawData()
		{
			std::lock_guard<std::mutex> lk(mut);
#if PLOT_DATA
			gpErr.plot_add(e, "Error");
			gpErr.plot_show();
			gpErr.draw_now();

			gpOut.plot_add(y, "Out");
			gpOut.plot_show();
			gpOut.draw_now();
#endif
		}
		void drawData(arma::vec xx, arma::vec yy, std::string namex = "", std::string namey ="")
		{
			std::lock_guard<std::mutex> lk(mut);
#if PLOT_DATA
			gpErr.plot_add(xx, namex);
			gpErr.plot_show();
			gpErr.draw_now();

			gpOut.plot_add(yy, namey);
			gpOut.plot_show();
			gpOut.draw_now();
#endif
		}

		arma::vec getErrorVec()
		{
			std::lock_guard<std::mutex> lk(mut);
			return e;
		}
		std::vector<float> getErrorVector()
		{
			std::lock_guard<std::mutex> lk(mut);
			return e_v;
		}

		arma::vec getOutVec()
		{
			std::lock_guard<std::mutex> lk(mut);
			return y;
		}
		std::vector<float> getOutVector()
		{
			std::lock_guard<std::mutex> lk(mut);
			return y_v;
		}
		//--------------------------------------------------------------------------------
	};
}
#endif // !_NLMS_H_ 