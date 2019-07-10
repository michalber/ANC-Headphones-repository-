/*
 * Copyright (c) 2007, 2008, 2009, 2010 Joseph Gaeddert
 * Copyright (c) 2007, 2008, 2009, 2010 Virginia Polytechnic
 *                                      Institute & State University
 *
 * This file is part of liquid.
 *
 * liquid is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * liquid is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with liquid.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "liquid.internal.h"


// Compute group delay for a FIR filter
//  _h      : filter coefficients array [size: _n x 1]
//  _n      : filter length
//  _fc     : frequency at which delay is evaluated (-0.5 < _fc < 0.5)
float fir_group_delay(float * _h,
                      unsigned int _n,
                      float _fc)
{
    // validate input
    if (_n == 0) {
        fprintf(stderr,"error: fir_group_delay(), length must be greater than zero\n");
        exit(1);
    } else if (_fc < -0.5 || _fc > 0.5) {
        fprintf(stderr,"error: fir_group_delay(), _fc must be in [-0.5,0.5]\n");
        exit(1);
    }

    unsigned int i;
    float complex t0=0.0f;
    float complex t1=0.0f;
    for (i=0; i<_n; i++) {
        t0 += _h[i] * cexpf(_Complex_I*2*M_PI*_fc*i) * i;
        t1 += _h[i] * cexpf(_Complex_I*2*M_PI*_fc*i);
    }

    return crealf(t0/t1);
}

// Compute group delay for an IIR filter
//  _b      : filter coefficients array (numerator), [size: _nb x 1]
//  _nb     : filter length (numerator)
//  _a      : filter coefficients array (denominator), [size: _na x 1]
//  _na     : filter length (denominator)
//  _fc     : frequency at which delay is evaluated (-0.5 < _fc < 0.5)
float iir_group_delay(float * _b,
                      unsigned int _nb,
                      float * _a,
                      unsigned int _na,
                      float _fc)
{
    // validate input
    if (_nb == 0) {
        fprintf(stderr,"error: iir_group_delay(), numerator length must be greater than zero\n");
        exit(1);
    } else if (_na == 0) {
        fprintf(stderr,"error: iir_group_delay(), denominator length must be greater than zero\n");
        exit(1);
    } else if (_fc < -0.5 || _fc > 0.5) {
        fprintf(stderr,"error: iir_group_delay(), _fc must be in [-0.5,0.5]\n");
        exit(1);
    }

    // compute c = conv(b,fliplr(a))
    //         c(z) = b(z)*a(1/z)*z^(-_na)
    unsigned int nc = _na + _nb - 1;
    float c[nc];
    unsigned int i,j;
    for (i=0; i<nc; i++)
        c[i] = 0.0;

    for (i=0; i<_na; i++) {
        for (j=0; j<_nb; j++) {
            c[i+j] += conjf(_a[_na-i-1])*_b[j];
        }
    }

    // compute 
    //      sum(c[i] * exp(j 2 pi fc i) * i)
    //      --------------------------------
    //      sum(c[i] * exp(j 2 pi fc i))
    float complex t0=0.0f;
    float complex t1=0.0f;
    for (i=0; i<nc; i++) {
        t0 += c[i] * cexpf(_Complex_I*2*M_PI*_fc*i) * i;
        t1 += c[i] * cexpf(_Complex_I*2*M_PI*_fc*i);
    }

    // prevent divide-by-zero (check magnitude for tolerance range)
    float tol = 1e-5f;
    if (cabsf(t1)<tol)
        return 0.0f;

    // return result, scaled by length of denominator
    return crealf(t0/t1) - (_na - 1);
}

