/*
Copywright - Michał Berdzik, AGH UST 2019, Cracow
github - @michalber
Date: 26.05.2019
*/
#include "RLMS.h"

namespace RLMS {
	//--------------------------------------------------------------------------------------------------------------------
		/**
			@brief Default constructor of RLMS class
			@author	Michał Berdzik
			@version 0.0.1 26-04-2019
			@param
			@retval
		*/
	RLMS::RLMS()
	{
		clear();
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Parametrized constructor of RLMS class
		@author	Michał Berdzik
		@version 0.0.1 26-04-2019
		@param	cNumofTaps -
				cLambda -
		@retval
	*/
	RLMS::RLMS(int cNumOfTaps, double cLambda) : pNumOfTaps(cNumOfTaps), pLambda(cLambda)
	{
		p.set_size(pNumOfTaps, pNumOfTaps);

		w1.set_size(pNumOfTaps, 1);
		w1.zeros();
		xx.set_size(pNumOfTaps, 1);
		xx.zeros();

		xx_t.zeros();
		k.zeros();
		l.zeros();
		temp.zeros();

		p.eye();
		double a = 0.01;
		p *= a;

		//clear();
	}
	//--------------------------------------------------------------------------------------------------------------------
	RLMS::RLMS(double cLambda) : pLambda(cLambda)
	{
		//clear();
	}
	//--------------------------------------------------------------------------------------------------------------------
	RLMS::RLMS(int cNumOfTaps) : pNumOfTaps(cNumOfTaps)
	{
		p.set_size(pNumOfTaps, pNumOfTaps);

		w1.set_size(pNumOfTaps, 1);
		w1.zeros();
		xx.set_size(pNumOfTaps, 1);
		xx.zeros();

		xx_t.zeros();
		k.zeros();
		l.zeros();
		temp.zeros();

		p.eye();
		double a = 0.01;
		p *= a;
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Default destructor of RLMS class
		@author	Michał Berdzik
		@version 0.0.1 26-04-2019
		@param
		@retval
	*/
	RLMS::~RLMS()
	{
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Function to set Lambda value
		@author	Michał Berdzik
		@version 0.0.1 26-04-2019
		@param	lambda -
		@retval	true or false
	*/
	//--------------------------------------------------------------------------------------------------------------------
	bool RLMS::setLambda(double lambda)
	{
		if (pLambda != lambda) {
			pLambda = lambda;
			return true;
		}
		else return false;
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Function to set numOfTaps value
		@author	Michał Berdzik
		@version 0.0.1 26-04-2019
		@param	numofTaps
		@retval	true or false
	*/
	bool RLMS::setNumOfTaps(int numOfTaps)
	{
		if (pNumOfTaps != numOfTaps) {
			pNumOfTaps = numOfTaps;

			p.set_size(pNumOfTaps, pNumOfTaps);

			w1.set_size(pNumOfTaps, 1);
			w1.zeros();
			xx.set_size(pNumOfTaps, 1);
			xx.zeros();

			xx_t.zeros();
			k.zeros();
			l.zeros();
			temp.zeros();

			p.eye();
			double a = 0.01;
			p *= a;

			return true;
		}
		else return false;
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Function to
		@author	Michał Berdzik
		@version 0.0.1 26-04-2019
		@param
		@retval
	*/
	double RLMS::process(volatile double f_d, volatile double f_x, volatile double f_m)
	{
		/*
		f_d - music+noise - mic in headphone cup
				d = music + noise
		f_x - noise - mic outside headphone

		MATLAB code of RLMS function

			I = eye(M);		% matrix M x M
			a = 0.01;
			p = a * I;		% matrix	M x M

			x = x;			% music vector
			w1 = zeros(M,1);	% static coefficients
			y = zeros(Ns, 1);	% music out
			e = zeros(Ns, 1);	% error out
			xx = zeros(M,1);

			for n = 1:Ns		% this is iteration in all music samples so it can be changed to single function on one sample without loop
				xx = [x(n); xx(1:M-1)];
				k = (p * xx) ./ (lamda + xx' * p * xx);
				y(n) = xx'*w1;
				e(n) = d(n) - y(n);
				w1 = w1 + k * e(n);
				p = (p - k * xx' * p) ./ lamda;
				w(:,n) = w1;
			end
		*/


#if TIME_DEBUG
		auto start = std::chrono::system_clock::now();
#endif	

		pushFront(xx, f_x);
		xx_t = trans(xx);

		k = (p * xx);
		l = (pLambda + xx_t * p * xx);
		k /= l(0, 0);

		pY = xx_t * w1;
		pError = f_d - pY(0, 0);

		w1 += k * pError;

		p = (p - k * xx_t * p) / pLambda;

#if DEBUG
		cout << "xx: " << endl;
		xx.print();
		cout << endl;

		cout << "p: " << endl;
		p.print();
		cout << endl;

		cout << "k: " << endl;
		k.print();
		cout << endl;

		cout << "w1: " << endl;
		w1.print();
		cout << endl;

		cout << "y: " << endl;
		pY.print();
		cout << endl;
#endif

#if TIME_DEBUG
		auto end = std::chrono::system_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

		return duration.count();
#else
		pY -= (f_d - f_x);	// new sample is filtered 			
		//pY -= plastSample;	// new sample is filtered 			
		//plastSample = (f_m - pY(0,0));

		plastSample = pY(0, 0);
		return plastSample;
#endif
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief Function to clear all data from used vectors
		@author	Michał Berdzik
		@version 0.0.1 26-04-2019
		@param
		@retval
	*/
	void RLMS::clear()
	{
		/*pError = 0;
		pY = 0;
		pFilterParam = 0;
		w1.clear();
		xx.clear();
		p.fill(0);*/
	}
	//--------------------------------------------------------------------------------------------------------------------
	/**
		@brief
		@author	Michał Berdzik
		@version 0.0.1 28-04-2019
		@param
		@retval
	*/
	void RLMS::pushFront(colvec &a, double x)
	{
		temp = a;
		a(0, 0) = x;
		for (int i = 1; i < pNumOfTaps; i++) {
			a(i, 0) = temp(i - 1, 0);
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
}