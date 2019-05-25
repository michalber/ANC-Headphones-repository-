#pragma once
#ifndef _NLMS_H_
#define _NLMS_H_

//-----------------------------------------------------------------------------------------------
#include <sigpack.h>
#include "config.h"
//-----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------
namespace Adaptive {

	class NLMS
	{
		int pNumOfTaps;
		double stepSize;
		double RFactor;
		// Port Audio FIR filter 
		sp::FIR_filt<double, double, double> AdaptiveFilter;

		// Signal vectors
		arma::vec y;  // Output data
		arma::vec e;  // Error data

#if PLOT_DATA
		sp::gplot gpErr;
		sp::gplot gpOut;
#endif

	public:


		NLMS() :pNumOfTaps(60), stepSize(0.001), RFactor(0.001)
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
		//--------------------------------------------------------------------------------
		/**
				@brief Function to perform NLMS algorithm on given data x and d

				@author	Micha³ Berdzik
				@version 0.0.1 18-05-2019

				@param arma::vec x  -  signal of readed noise
				@param arma::vec d  -  signal of readed music+noise
			*/
		void updateNLMS(arma::vec x, arma::vec d)
		{
			for (int n = 0; n < FRAMES_PER_BUFFER; n++)
			{
				// Apply adaptiv filter
				y(n) = AdaptiveFilter(x(n));

				// Calc error
				e(n) = d(n) - y(n);

				// Update filter
				AdaptiveFilter.nlms_adapt(e(n));
			}
		}
		//--------------------------------------------------------------------------------
		/**
				@brief Function to draw error data using Sigpack library

				@author	Micha³ Berdzik
				@version 0.0.1 18-05-2019

			*/
		void drawData()
		{
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
			return e;
		}

		arma::vec getOutVec()
		{
			return y;
		}
		//--------------------------------------------------------------------------------
	};
}
#endif // !_NLMS_H_