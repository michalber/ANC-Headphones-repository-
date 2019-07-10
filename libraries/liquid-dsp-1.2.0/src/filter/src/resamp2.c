/*
 * Copyright (c) 2007, 2009, 2010 Joseph Gaeddert
 * Copyright (c) 2007, 2009, 2010 Virginia Polytechnic Institute &
 *                                State University
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

//
// Halfband resampler (interpolator/decimator)
//

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// defined:
//  RESAMP2()       name-mangling macro
//  TO              output data type
//  TC              coefficient data type
//  TI              input data type
//  WINDOW()        window macro
//  DOTPROD()       dotprod macro
//  PRINTVAL()      print macro

struct RESAMP2(_s) {
    TC * h;                 // filter prototype
    unsigned int m;         // primitive filter length
    unsigned int h_len;     // actual filter length: h_len = 4*m+1
    float fc;               // center frequency [-1.0 <= fc <= 1.0]
    float As;               // stop-band attenuation [dB]

    // filter component
    TC * h1;                // filter branch coefficients
    DOTPROD() dp;           // inner dot product object
    unsigned int h1_len;    // filter length (2*m)

    // input buffers
    WINDOW() w0;            // input buffer (even samples)
    WINDOW() w1;            // input buffer (odd samples)

    // halfband filter operation
    unsigned int toggle;
};

// create a resamp2 object
//  _m      :   filter semi-length (effective length: 4*_m+1)
//  _fc     :   center frequency of half-band filter
//  _As     :   stop-band attenuation [dB], _As > 0
RESAMP2() RESAMP2(_create)(unsigned int _m,
                           float _fc,
                           float _As)
{
    // validate input
    if (_m < 2) {
        fprintf(stderr,"error: resamp2_%s_create(), filter semi-length must be at least 2\n", EXTENSION_FULL);
        exit(1);
    }

    RESAMP2() q = (RESAMP2()) malloc(sizeof(struct RESAMP2(_s)));
    q->m  = _m;
    q->fc = _fc;
    q->As = _As;
    if ( q->fc < -0.5f || q->fc > 0.5f ) {
        fprintf(stderr,"error: resamp2_%s_create(), fc (%12.4e) must be in (-1,1)\n", EXTENSION_FULL, q->fc);
        exit(1);
    }

    // change filter length as necessary
    q->h_len = 4*(q->m) + 1;
    q->h = (TC *) malloc((q->h_len)*sizeof(TC));

    q->h1_len = 2*(q->m);
    q->h1 = (TC *) malloc((q->h1_len)*sizeof(TC));

    // design filter prototype
    unsigned int i;
    float t, h1, h2;
    TC h3;
    float beta = kaiser_beta_As(q->As);
    for (i=0; i<q->h_len; i++) {
        t = (float)i - (float)(q->h_len-1)/2.0f;
        h1 = sincf(t/2.0f);
        h2 = kaiser(i,q->h_len,beta,0);
#if TC_COMPLEX == 1
        h3 = cosf(2.0f*M_PI*t*q->fc) + _Complex_I*sinf(2.0f*M_PI*t*q->fc);
#else
        h3 = cosf(2.0f*M_PI*t*q->fc);
#endif
        q->h[i] = h1*h2*h3;
    }

    // resample, alternate sign, [reverse direction]
    unsigned int j=0;
    for (i=1; i<q->h_len; i+=2)
        q->h1[j++] = q->h[q->h_len - i - 1];

    // create dotprod object
    q->dp = DOTPROD(_create)(q->h1, 2*q->m);

    // create window buffers
    q->w0 = WINDOW(_create)(2*(q->m));
    q->w1 = WINDOW(_create)(2*(q->m));

    RESAMP2(_clear)(q);

    return q;
}

// re-create a resamp2 object
//  _q          :   original resamp2 object
//  _m          :   filter semi-length (effective length: 4*_m+1)
//  _fc         :   center frequency of half-band filter
//  _As         :   stop-band attenuation [dB], _As > 0
RESAMP2() RESAMP2(_recreate)(RESAMP2() _q,
                             unsigned int _m,
                             float _fc,
                             float _As)
{
    // only re-design filter if necessary
    if (_m != _q->m) {
        // destroy resampler and re-create from scratch
        RESAMP2(_destroy)(_q);
        _q = RESAMP2(_create)(_m, _fc, _As);

    } else {
        // re-design filter prototype
        unsigned int i;
        float t, h1, h2;
        TC h3;
        float beta = kaiser_beta_As(_q->As);
        for (i=0; i<_q->h_len; i++) {
            t = (float)i - (float)(_q->h_len-1)/2.0f;
            h1 = sincf(t/2.0f);
            h2 = kaiser(i,_q->h_len,beta,0);
#if TC_COMPLEX == 1
            h3 = cosf(2.0f*M_PI*t*_q->fc) + _Complex_I*sinf(2.0f*M_PI*t*_q->fc);
#else
            h3 = cosf(2.0f*M_PI*t*_q->fc);
#endif
            _q->h[i] = h1*h2*h3;
        }

        // resample, alternate sign, [reverse direction]
        unsigned int j=0;
        for (i=1; i<_q->h_len; i+=2)
            _q->h1[j++] = _q->h[_q->h_len - i - 1];

        // create dotprod object
        _q->dp = DOTPROD(_recreate)(_q->dp, _q->h1, 2*_q->m);
    }
    return _q;
}

// destroy a resamp2 object, clearing up all allocated memory
void RESAMP2(_destroy)(RESAMP2() _q)
{
    // destroy dotprod object
    DOTPROD(_destroy)(_q->dp);

    // destroy window buffers
    WINDOW(_destroy)(_q->w0);
    WINDOW(_destroy)(_q->w1);

    // free arrays
    free(_q->h);
    free(_q->h1);

    // free main object memory
    free(_q);
}

// print a resamp2 object's internals
void RESAMP2(_print)(RESAMP2() _q)
{
    printf("fir half-band resampler: [%u taps, fc=%12.8f]\n",
            _q->h_len,
            _q->fc);
    unsigned int i;
    for (i=0; i<_q->h_len; i++) {
        printf("  h(%4u) = ", i+1);
        PRINTVAL_TC(_q->h[i],%12.8f);
        printf(";\n");
    }
    printf("---\n");
    for (i=0; i<_q->h1_len; i++) {
        printf("  h1(%4u) = ", i+1);
        PRINTVAL_TC(_q->h1[i],%12.8f);
        printf(";\n");
    }
}

// clear internal buffer
void RESAMP2(_clear)(RESAMP2() _q)
{
    WINDOW(_clear)(_q->w0);
    WINDOW(_clear)(_q->w1);

    _q->toggle = 0;
}

// execute half-band filter
//  _q      :   resamp2 object
//  _x      :   input sample
//  _y0     :   output sample pointer (low frequency)
//  _y1     :   output sample pointer (high frequency)
void RESAMP2(_filter_execute)(RESAMP2() _q,
                              TI _x,
                              TO * _y0,
                              TO * _y1)
{
    TI * r;     // buffer read pointer
    TO yi;      // delay branch
    TO yq;      // filter branch

    if ( _q->toggle == 0 ) {
        // push sample into upper branch
        WINDOW(_push)(_q->w0, _x);

        // upper branch (delay)
        WINDOW(_index)(_q->w0, _q->m-1, &yi);

        // lower branch (filter)
        WINDOW(_read)(_q->w1, &r);
        DOTPROD(_execute)(_q->dp, r, &yq);
    } else {
        // push sample into lower branch
        WINDOW(_push)(_q->w1, _x);

        // upper branch (delay)
        WINDOW(_index)(_q->w1, _q->m-1, &yi);

        // lower branch (filter)
        WINDOW(_read)(_q->w0, &r);
        DOTPROD(_execute)(_q->dp, r, &yq);
    }

    // toggle flag
    _q->toggle = 1 - _q->toggle;

    // set return values, normalizing gain
    *_y0 = 0.5f*(yi + yq);  // lower band
    *_y1 = 0.5f*(yi - yq);  // upper band
}

// execute analysis half-band filterbank
//  _q      :   resamp2 object
//  _x      :   input array [size: 2 x 1]
//  _y      :   output array [size: 2 x 1]
void RESAMP2(_analyzer_execute)(RESAMP2() _q,
                                TI * _x,
                                TO * _y)
{
    TI * r;     // buffer read pointer
    TO y0;      // delay branch
    TO y1;      // filter branch

    // compute filter branch
    WINDOW(_push)(_q->w1, 0.5*_x[0]);
    WINDOW(_read)(_q->w1, &r);
    DOTPROD(_execute)(_q->dp, r, &y1);

    // compute delay branch
    WINDOW(_push)(_q->w0, 0.5*_x[1]);
    WINDOW(_index)(_q->w0, _q->m-1, &y0);

    // set return value
    _y[0] = y1 + y0;
    _y[1] = y1 - y0;
}

// execute synthesis half-band filterbank
//  _q      :   resamp2 object
//  _x      :   input array [size: 2 x 1]
//  _y      :   output array [size: 2 x 1]
void RESAMP2(_synthesizer_execute)(RESAMP2() _q,
                                   TI * _x,
                                   TO * _y)
{
    TI * r;                 // buffer read pointer
    TI x0 = _x[0] + _x[1];  // delay branch input
    TI x1 = _x[0] - _x[1];  // filter branch input

    // compute delay branch
    WINDOW(_push)(_q->w0, x0);
    WINDOW(_index)(_q->w0, _q->m-1, &_y[0]);

    // compute second branch (filter)
    WINDOW(_push)(_q->w1, x1);
    WINDOW(_read)(_q->w1, &r);
    DOTPROD(_execute)(_q->dp, r, &_y[1]);
}


// execute half-band decimation
//  _q      :   resamp2 object
//  _x      :   input array [size: 2 x 1]
//  _y      :   output sample pointer
void RESAMP2(_decim_execute)(RESAMP2() _q,
                             TI * _x,
                             TO * _y)
{
    TI * r;     // buffer read pointer
    TO y0;      // delay branch
    TO y1;      // filter branch

    // compute filter branch
    WINDOW(_push)(_q->w1, _x[0]);
    WINDOW(_read)(_q->w1, &r);
    DOTPROD(_execute)(_q->dp, r, &y1);

    // compute delay branch
    WINDOW(_push)(_q->w0, _x[1]);
    WINDOW(_index)(_q->w0, _q->m-1, &y0);

    // set return value
    *_y = y0 + y1;
}

// execute half-band interpolation
//  _q      :   resamp2 object
//  _x      :   input sample
//  _y      :   output array [size: 2 x 1]
void RESAMP2(_interp_execute)(RESAMP2() _q,
                              TI   _x,
                              TO * _y)
{
    TI * r;  // buffer read pointer

    // compute delay branch
    WINDOW(_push)(_q->w0, _x);
    WINDOW(_index)(_q->w0, _q->m-1, &_y[0]);

    // compute second branch (filter)
    WINDOW(_push)(_q->w1, _x);
    WINDOW(_read)(_q->w1, &r);
    DOTPROD(_execute)(_q->dp, r, &_y[1]);
}

