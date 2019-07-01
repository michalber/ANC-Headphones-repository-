#pragma once
#ifndef _NLMS_H_
#define _NLMS_H_

//-----------------------------------------------------------------------------------------------
#include <sigpack.h>
#include <mutex>
#include <boost/circular_buffer.hpp>
#include "config.h"
//-----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------
namespace Adaptive {

	class NLMS
	{
		// Port Audio FIR filter 
		sp::FIR_filt<double, double, double> AdaptiveFilter;

		bool processing{ false };
		std::mutex mut;
		int pNumOfTaps;
		float stepSize;
		float RFactor;
		float e;

		// Signal vectors
		arma::vec y;  // Output data
		//arma::vec e;  // Error data
		std::vector<float> y_v;  // Output data
		std::vector<float> e_v;  // Error data
		float y_f;
		float e_f;

		arma::colvec pError;
		arma::colvec pY;		
		
		arma::mat p;
		arma::colvec w1;
		arma::colvec xx;

		arma::rowvec xx_t;
		arma::rowvec w1_t;
		arma::colvec k, l, temp;

		boost::circular_buffer<float> *OutBuff = NULL;

#if PLOT_DATA
		sp::gplot gpErr;
		sp::gplot gpOut;
#endif

	public:


		NLMS() :pNumOfTaps(60), stepSize(0.5), RFactor(0.001)
		{
			AdaptiveFilter.setup_nlms(pNumOfTaps, stepSize, RFactor);

			y = arma::vec(FRAMES_PER_BUFFER);  // Model sig	
			pError = arma::vec(FRAMES_PER_BUFFER);  // Err sig
			y.fill(0);
			pError.fill(0);
			
			w1.set_size(pNumOfTaps, 1);
			w1.zeros();
			xx.set_size(pNumOfTaps, 1);
			xx.zeros();

			xx_t.zeros();
			k.zeros();
			l.zeros();
			temp.zeros();


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
			AdaptiveFilter.setup_nlms(pNumOfTaps, stepSize, RFactor);

			y = arma::vec(FRAMES_PER_BUFFER);  // Model sig	
			pError = arma::vec(FRAMES_PER_BUFFER);  // Err sig
			y.fill(0);
			pError.fill(0);

			w1.set_size(pNumOfTaps, 1);
			w1.zeros();
			xx.set_size(pNumOfTaps, 1);
			xx.zeros();

			xx_t.zeros();
			k.zeros();
			l.zeros();
			temp.zeros();
		

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
			y = arma::vec(FRAMES_PER_BUFFER);  // Model sig	
			pError = arma::vec(FRAMES_PER_BUFFER);  // Err sig
			y.fill(0);
			pError.fill(0);

			pNumOfTaps = Taps;

			w1.set_size(pNumOfTaps, 1);
			w1.zeros();
			xx.set_size(pNumOfTaps, 1);
			xx.zeros();

			xx_t.zeros();
			k.zeros();
			l.zeros();
			temp.zeros();

			stepSize = stepsize;
			RFactor = rfactor;

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
		void updateNLMSFilter(arma::vec d, arma::vec x)
		{
			//std::lock_guard<std::mutex> lk(mut);
			for (int n = 0; n < FRAMES_PER_BUFFER; n++)
			{
				// Apply adaptiv filter
				y(n) = AdaptiveFilter(x(n));

				//Add new data to buffer
				if (!OutBuff->full()) {
					OutBuff->push_front(y(n));
					//OutBuff->push_front(d(n));
				}

				// Calc error				
				pError(n) = (d(n)) - y(n);

				// Update filter
				AdaptiveFilter.nlms_adapt(pError(n));
			}
		}

		void updateNLMS(arma::vec d, arma::vec x)
		{
			/*
			MATLAB code for my NLMS

				x = x;
				xx = zeros(M,1);
				w1 = zeros(M,1);
				y = zeros(Ns,1);
				e = zeros(Ns,1);

				for n = 1:Ns
					xx = [xx(2:M);x(n)];
					y(n) = w1' * xx;
					k = mu/(a + xx'*xx);
					e(n) = d(n) - y(n);
					w1 = w1 + k * e(n) * xx;
					w(:,n) = w1;
				end
			*/
			//std::lock_guard<std::mutex> lk(mut);
			processing = true;

			pY.fill(0);
			pError.fill(0);

			for (int n = 0; n < FRAMES_PER_BUFFER; n++) {

				pushBack(xx, x(n));
				xx_t = trans(xx);
				w1_t = trans(w1);

				//apply adaptive filter to input vector 
				pY = w1_t * xx;
				y(n) = pY(0, 0);

				//update output buffer
				if (!OutBuff->full()) {
					//OutBuff->push_front(pY(0,0));
					//OutBuff->push_front(d(n));
				}

				//update filter weights
				l = xx_t * xx;
				k = stepSize / (RFactor + l(0, 0));
				e = d(n) - pY(0, 0);
				pError(n) = e;
				w1 += (k(0, 0) * e) * xx;
			}

			processing = false;
		}

		void updateNLMS(std::vector<float> d, std::vector<float> x, std::vector<float> m)
		{
			std::lock_guard<std::mutex> lk(mut);
			y_v.clear();
			e_v.clear();
			for (int n = 0; n < FRAMES_PER_BUFFER; n++)
			{
				
			}
		}
		double updateNLMS(double d, double x, double m)
		{
			std::lock_guard<std::mutex> lk(mut);
			

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
			if (!processing) {
				std::lock_guard<std::mutex> lk(mut);
#if PLOT_DATA
				gpErr.plot_add(pError, "Error");
				gpErr.plot_show();
				gpErr.draw_now();

				gpOut.plot_add(y, "Out");
				gpOut.plot_show();
				gpOut.draw_now();
#endif
			}
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
			return pError;
		}
		std::vector<float> getErrorVector()
		{
			return e_v;
		}

		arma::vec getOutVec()
		{
			return y;
		}
		std::vector<float> getOutVector()
		{
			return y_v;
		}
		//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief
			@author	Micha³ Berdzik
			@version 0.0.1 28-04-2019
			@param
			@retval
		*/
		void pushBack(arma::colvec &a, double x)
		{
			std::lock_guard<std::mutex> lk(mut);
			temp = a;
			for (int i = 0; i < pNumOfTaps - 1; i++) {
				a(i, 0) = temp(i + 1, 0);
			}
			a(pNumOfTaps - 1, 0) = x;
		}

		void setUpBuffer(boost::circular_buffer<float>* x)
		{			
			OutBuff = x;
		}
	};
}
#endif // !_NLMS_H_ 