#pragma once

#include "config.h"

#ifndef TIME_DEBUG
	#define TIME_DEBUG 0
	#if TIME_DEBUG
		#include <chrono>
	#endif
#endif // TIME_DEBUG

#include <armadillo>

using namespace arma;

namespace RLMS {
	class RLMS
	{
		double pLambda;
		int pNumOfTaps;
		double plastSample;
		double y, e, mu, a;

		colvec pError;
		colvec pY;
		colvec pFilterParam;

		mat p;
		colvec w1;
		colvec xx;
		rowvec xx_t;
		rowvec w1_t;

		colvec k, l, temp;


	public:
		RLMS();
		RLMS(double);
		RLMS(int);
		RLMS(int, double);
		~RLMS();

		bool setLambda(double);
		bool setNumOfTaps(int);
		double process(double, double, double);
		double processNLMS(double, double, double);
		void clear();

	private:
		void pushFront(colvec&, double);
		void pushBack(colvec&, double);
	};
}
