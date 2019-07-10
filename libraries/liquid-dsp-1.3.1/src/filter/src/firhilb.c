/*
 * Copyright (c) 2007 - 2015 Joseph Gaeddert
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//
// firhilb.c
//
// finite impulse response (FIR) Hilbert transform
//

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// defined:
//  FIRHILB()       name-mangling macro
//  T               coefficients type
//  WINDOW()        window macro
//  DOTPROD()       dotprod macro
//  PRINTVAL()      print macro

struct FIRHILB(_s) {
    T * h;                  // filter coefficients
    T complex * hc;         // filter coefficients (complex)
    unsigned int h_len;     // length of filter
    float As;               // filter stop-band attenuation [dB]

    unsigned int m;         // filter semi-length, h_len = 4*m+1

    // quadrature filter component
    T * hq;                 // quadrature filter coefficients
    unsigned int hq_len;    // quadrature filter length (2*m)

    // input buffers
    WINDOW() w0;            // input buffer (even samples)
    WINDOW() w1;            // input buffer (odd samples)

    // vector dot product
    DOTPROD() dpq;

    // regular real-to-complex/complex-to-real operation
    unsigned int toggle;
};

// create firhilb object
//  _m      :   filter semi-length (delay: 2*m+1)
//  _As     :   stop-band attenuation [dB]
FIRHILB() FIRHILB(_create)(unsigned int _m,
                           float        _As)
{
    // validate firhilb inputs
    if (_m < 2) {
        fprintf(stderr,"error: firhilb_create(), filter semi-length (m) must be at least 2\n");
        exit(1);
    }

    // allocate memory for main object
    FIRHILB() q = (FIRHILB()) malloc(sizeof(struct FIRHILB(_s)));
    q->m  = _m;         // filter semi-length
    q->As = fabsf(_As); // stop-band attenuation

    // set filter length and allocate memory for coefficients
    q->h_len = 4*(q->m) + 1;
    q->h     = (T *)         malloc((q->h_len)*sizeof(T));
    q->hc    = (T complex *) malloc((q->h_len)*sizeof(T complex));

    // allocate memory for quadrature filter component
    q->hq_len = 2*(q->m);
    q->hq     = (T *) malloc((q->hq_len)*sizeof(T));

    // compute filter coefficients for half-band filter
    liquid_firdes_kaiser(q->h_len, 0.25f, q->As, 0.0f, q->h);

    // alternate sign of non-zero elements
    unsigned int i;
    for (i=0; i<q->h_len; i++) {
        float t = (float)i - (float)(q->h_len-1)/2.0f;
        q->hc[i] = q->h[i] * cexpf(_Complex_I*0.5f*M_PI*t);
        q->h[i]  = cimagf(q->hc[i]);
    }

    // resample, reverse direction
    unsigned int j=0;
    for (i=1; i<q->h_len; i+=2)
        q->hq[j++] = q->h[q->h_len - i - 1];

    // create windows for upper and lower polyphase filter branches
    q->w1 = WINDOW(_create)(2*(q->m));
    q->w0 = WINDOW(_create)(2*(q->m));

    // create internal dot product object
    q->dpq = DOTPROD(_create)(q->hq, q->hq_len);

    // reset internal state and return object
    FIRHILB(_reset)(q);
    return q;
}

// destroy firhilb object
void FIRHILB(_destroy)(FIRHILB() _q)
{
    // destroy window buffers
    WINDOW(_destroy)(_q->w0);
    WINDOW(_destroy)(_q->w1);
    
    // destroy internal dot product object
    DOTPROD(_destroy)(_q->dpq);

    // free coefficients arrays
    free(_q->h);
    free(_q->hc);
    free(_q->hq);

    // free main object memory
    free(_q);
}

// print firhilb object internals
void FIRHILB(_print)(FIRHILB() _q)
{
    printf("fir hilbert transform: [%u]\n", _q->h_len);
    unsigned int i;
    for (i=0; i<_q->h_len; i++) {
        printf("  hc(%4u) = %8.4f + j*%8.4f;\n", i+1, crealf(_q->hc[i]), cimagf(_q->hc[i]));
    }
    printf("---\n");
    for (i=0; i<_q->h_len; i++) {
        printf("  h(%4u) = %8.4f;\n", i+1, _q->h[i]);
    }
    printf("---\n");
    for (i=0; i<_q->hq_len; i++) {
        printf("  hq(%4u) = %8.4f;\n", i+1, _q->hq[i]);
    }
}

// reset firhilb object internal state
void FIRHILB(_reset)(FIRHILB() _q)
{
    // clear window buffers
    WINDOW(_reset)(_q->w0);
    WINDOW(_reset)(_q->w1);

    // reset toggle flag
    _q->toggle = 0;
}

// execute Hilbert transform (real to complex)
//  _q      :   firhilb object
//  _x      :   real-valued input sample
//  _y      :   complex-valued output sample
void FIRHILB(_r2c_execute)(FIRHILB()   _q,
                           T           _x,
                           T complex * _y)
{
    T * r;  // buffer read pointer
    T yi;   // in-phase component
    T yq;   // quadrature component

    if ( _q->toggle == 0 ) {
        // push sample into upper branch
        WINDOW(_push)(_q->w0, _x);

        // upper branch (delay)
        WINDOW(_index)(_q->w0, _q->m-1, &yi);

        // lower branch (filter)
        WINDOW(_read)(_q->w1, &r);
        
        // execute dotprod
        DOTPROD(_execute)(_q->dpq, r, &yq);
    } else {
        // push sample into lower branch
        WINDOW(_push)(_q->w1, _x);

        // upper branch (delay)
        WINDOW(_index)(_q->w1, _q->m-1, &yi);

        // lower branch (filter)
        WINDOW(_read)(_q->w0, &r);

        // execute dotprod
        DOTPROD(_execute)(_q->dpq, r, &yq);
    }

    // toggle flag
    _q->toggle = 1 - _q->toggle;

    // set return value
    *_y = yi + _Complex_I * yq;
}

// execute Hilbert transform (complex to real)
//  _q      :   firhilb object
//  _y      :   complex-valued input sample
//  _x      :   real-valued output sample
void FIRHILB(_c2r_execute)(FIRHILB() _q,
                           T complex _x,
                           T *       _y)
{
    *_y = crealf(_x);
}

// execute Hilbert transform decimator (real to complex)
//  _q      :   firhilb object
//  _x      :   real-valued input array [size: 2 x 1]
//  _y      :   complex-valued output sample
void FIRHILB(_decim_execute)(FIRHILB()   _q,
                             T *         _x,
                             T complex * _y)
{
    T * r;  // buffer read pointer
    T yi;   // in-phase component
    T yq;   // quadrature component

    // compute quadrature component (filter branch)
    WINDOW(_push)(_q->w1, _x[0]);
    WINDOW(_read)(_q->w1, &r);
    DOTPROD(_execute)(_q->dpq, r, &yq);

    WINDOW(_push)(_q->w0, _x[1]);
    WINDOW(_index)(_q->w0, _q->m-1, &yi);

    // set return value
    *_y = yi + _Complex_I * yq;
}

// execute Hilbert transform decimator (real to complex) on
// a block of samples
//  _q      :   Hilbert transform object
//  _x      :   real-valued input array [size: 2*_n x 1]
//  _n      :   number of *output* samples
//  _y      :   complex-valued output array [size: _n x 1]
void FIRHILB(_decim_execute_block)(FIRHILB()    _q,
                                   T *          _x,
                                   unsigned int _n,
                                   T complex *  _y)
{
    unsigned int i;

    for (i=0; i<_n; i++)
        FIRHILB(_decim_execute)(_q, &_x[2*i], &_y[i]);
}

// execute Hilbert transform interpolator (complex to real)
//  _q      :   firhilb object
//  _y      :   complex-valued input sample
//  _x      :   real-valued output array [size: 2 x 1]
void FIRHILB(_interp_execute)(FIRHILB() _q,
                              T complex _x,
                              T *       _y)
{
    T * r;  // buffer read pointer

    // TODO macro for crealf, cimagf?
    
    WINDOW(_push)(_q->w0, cimagf(_x));
    WINDOW(_index)(_q->w0, _q->m-1, &_y[0]);

    // compute second branch (filter)
    WINDOW(_push)(_q->w1, crealf(_x));
    WINDOW(_read)(_q->w1, &r);
    DOTPROD(_execute)(_q->dpq, r, &_y[1]);
}

// execute Hilbert transform interpolator (complex to real)
// on a block of samples
//  _q      :   Hilbert transform object
//  _x      :   complex-valued input array [size: _n x 1]
//  _n      :   number of *input* samples
//  _y      :   real-valued output array [size: 2*_n x 1]
void FIRHILB(_interp_execute_block)(FIRHILB()    _q,
                                    T complex *  _x,
                                    unsigned int _n,
                                    T *          _y)
{
    unsigned int i;

    for (i=0; i<_n; i++)
        FIRHILB(_interp_execute)(_q, _x[i], &_y[2*i]);
}
