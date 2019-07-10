/*
 * Copyright (c) 2007, 2008, 2009, 2010, 2011, 2012 Joseph Gaeddert
 * Copyright (c) 2007, 2008, 2009, 2010, 2011, 2012 Virginia Polytechnic
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

//
// fft_common.c : common utilities specific to precision
//

#include <stdio.h>
#include <stdlib.h>
#include "liquid.internal.h"

struct FFT(plan_s)
{
    // common data
    unsigned int nfft;  // fft size
    TC * x;             // input array pointer (not allocated)
    TC * y;             // output array pointer (not allocated)
    int direction;      // forward/reverse
    int flags;
    liquid_fft_kind kind;
    liquid_fft_method method;

    // 'execute' function pointer
    void (*execute)(FFT(plan));

    // real even/odd DFT parameters (DCT/DST)
    T * xr; // input array (real)
    T * yr; // output array (real)

    // common data structure shared between specific FFT algorithms
    union {
        // DFT
        struct {
            TC * twiddle;               // twiddle factors
        } dft;

        // radix-2 transform data
        struct {
            unsigned int m;             // log2(nfft)
            unsigned int * index_rev;   // reversed indices
            TC * twiddle;               // twiddle factors
        } radix2;

        // recursive mixed-radix transform data:
        //  - compute 'Q' FFTs of size 'P'
        //  - apply twiddle factors
        //  - compute 'P' FFTs of size 'Q'
        //  - transpose result
        struct {
            unsigned int P;     // first FFT size
            unsigned int Q;     // second FFT size
            TC * x;             // input buffer (copied)
            TC * t0;            // temporary buffer (small FFT input)
            TC * t1;            // temporary buffer (small FFT output)
            TC * twiddle;       // twiddle factors
            FFT(plan) fft_P;    // sub-transform of size P
            FFT(plan) fft_Q;    // sub-transform of size Q
        } mixedradix;

        // Rader's algorithm for computing FFTs of prime length
        struct {
            unsigned int * seq; // transformation sequence, size: nfft-1
            TC * R;             // DFT of sequence { exp(-j*2*pi*g^i/nfft }, size: nfft-1
            TC * x_prime;       // sub-transform time-domain buffer
            TC * X_prime;       // sub-transform freq-domain buffer
            FFT(plan) fft;      // sub-FFT of size nfft-1
            FFT(plan) ifft;     // sub-IFFT of size nfft-1
        } rader;

        // Rader's alternat ealgorithm for computing FFTs of prime length
        struct {
            unsigned int nfft_prime;
            unsigned int * seq; // transformation sequence, size: nfft_prime
            TC * R;             // DFT of sequence { exp(-j*2*pi*g^i/nfft }, size: nfft_prime
            TC * x_prime;       // sub-transform time-domain buffer
            TC * X_prime;       // sub-transform freq-domain buffer
            FFT(plan) fft;      // sub-FFT of size nfft_prime
            FFT(plan) ifft;     // sub-IFFT of size nfft_prime
        } rader2;
    } data;
};

// create FFT plan
//  _nfft   :   FFT size
//  _x      :   input array [size: _nfft x 1]
//  _y      :   output array [size: _nfft x 1]
//  _dir    :   fft direction: {FFT_FORWARD, FFT_REVERSE}
//  _method :   fft method
FFT(plan) FFT(_create_plan)(unsigned int _nfft,
                            TC *         _x,
                            TC *         _y,
                            int          _dir,
                            int          _flags)
{
    // determine best method for execution
    // TODO : check flags and allow user override
    liquid_fft_method method = liquid_fft_estimate_method(_nfft);

    // initialize fft based on method
    switch (method) {
    case LIQUID_FFT_METHOD_RADIX2:
        // use radix-2 decimation-in-time method
        return FFT(_create_plan_radix2)(_nfft, _x, _y, _dir, _flags);

    case LIQUID_FFT_METHOD_MIXED_RADIX:
        // use Cooley-Tukey mixed-radix algorithm
        return FFT(_create_plan_mixed_radix)(_nfft, _x, _y, _dir, _flags);

    case LIQUID_FFT_METHOD_RADER:
        // use Rader's algorithm for FFTs of prime length
        return FFT(_create_plan_rader)(_nfft, _x, _y, _dir, _flags);

    case LIQUID_FFT_METHOD_RADER2:
        // use Rader's algorithm for FFTs of prime length
        return FFT(_create_plan_rader2)(_nfft, _x, _y, _dir, _flags);

    case LIQUID_FFT_METHOD_DFT:
        // use slow DFT
        return FFT(_create_plan_dft)(_nfft, _x, _y, _dir, _flags);

    case LIQUID_FFT_METHOD_NONE:
        // no method specified (e.g. real-to-real method)
        break;

    case LIQUID_FFT_METHOD_UNKNOWN:
    default:
        fprintf(stderr,"error: fft_create_plan(), unknown/invalid fft method\n");
        exit(1);
    }

    return NULL;
}

// destroy FFT plan
void FFT(_destroy_plan)(FFT(plan) _q)
{
    switch (_q->method) {
    case LIQUID_FFT_METHOD_DFT:         FFT(_destroy_plan_dft)(_q); break;
    case LIQUID_FFT_METHOD_RADIX2:      FFT(_destroy_plan_radix2)(_q); break;
    case LIQUID_FFT_METHOD_MIXED_RADIX: FFT(_destroy_plan_mixed_radix)(_q); break;
    case LIQUID_FFT_METHOD_RADER:       FFT(_destroy_plan_rader)(_q); break;
    case LIQUID_FFT_METHOD_RADER2:      FFT(_destroy_plan_rader2)(_q); break;
    case LIQUID_FFT_METHOD_NONE:        break;
    case LIQUID_FFT_METHOD_UNKNOWN:
    default:
        fprintf(stderr,"error: fft_destroy_plan(), unknown/invalid fft method\n");
        exit(1);
    }
}

// print FFT plan
void FFT(_print_plan)(FFT(plan) _q)
{
    printf("fft plan [%s], n=%u, ",
            _q->direction == FFT_FORWARD ? "forward" : "reverse",
            _q->nfft);

    switch (_q->method) {
    case LIQUID_FFT_METHOD_DFT:         printf("DFT\n");            break;
    case LIQUID_FFT_METHOD_RADIX2:      printf("Radix-2\n");        break;
    case LIQUID_FFT_METHOD_MIXED_RADIX: printf("Cooley-Tukey\n");   break;
    case LIQUID_FFT_METHOD_RADER:       printf("Rader (Type-I)\n"); break;
    case LIQUID_FFT_METHOD_RADER2:      printf("Rader (Type-II)\n"); break;
    case LIQUID_FFT_METHOD_NONE:        printf("(none)\n");         break;
    case LIQUID_FFT_METHOD_UNKNOWN:     printf("(unknown)\n");      break;
    default:                            printf("(unknown)\n");      break;
    }

    // print recursive plan
    FFT(_print_plan_recursive)(_q, 0);
}

// print FFT plan (recursively)
void FFT(_print_plan_recursive)(FFT(plan)    _q,
                                unsigned int _level)
{
    // print indentation based on recursion level
    unsigned int i;
    for (i=0; i<_level; i++)
        printf("  ");
    printf("%u, ", _q->nfft);

    switch (_q->method) {
    case LIQUID_FFT_METHOD_DFT:
        printf("DFT\n");
        break;

    case LIQUID_FFT_METHOD_RADIX2:
        printf("Radix-2\n");
        break;

    case LIQUID_FFT_METHOD_MIXED_RADIX:
        // two internal transforms
        printf("Cooley-Tukey mixed radix, Q=%u, P=%u\n",
                _q->data.mixedradix.Q,
                _q->data.mixedradix.P);
        FFT(_print_plan_recursive)(_q->data.mixedradix.fft_Q, _level+1);
        FFT(_print_plan_recursive)(_q->data.mixedradix.fft_P, _level+1);
        break;

    case LIQUID_FFT_METHOD_RADER:
        printf("Rader (Type-II), nfft-prime=%u\n", _q->nfft-1);
        FFT(_print_plan_recursive)(_q->data.rader.fft, _level+1);
        break;

    case LIQUID_FFT_METHOD_RADER2:
        printf("Rader (Type-II), nfft-prime=%u\n", _q->data.rader2.nfft_prime);
        FFT(_print_plan_recursive)(_q->data.rader2.fft, _level+1);
        break;

    case LIQUID_FFT_METHOD_NONE:        printf("(none)\n");         break;
    case LIQUID_FFT_METHOD_UNKNOWN:     printf("(unknown)\n");      break;
    default:                            printf("(unknown)\n");      break;
    }
}

// execute fft
void FFT(_execute)(FFT(plan) _q)
{
    // invoke internal function pointer
    _q->execute(_q);
}

// perform n-point FFT allocating plan internally
//  _nfft   :   fft size
//  _x      :   input array [size: _nfft x 1]
//  _y      :   output array [size: _nfft x 1]
//  _dir    :   fft direction: {FFT_FORWARD, FFT_REVERSE}
//  _method :   fft method
void FFT(_run)(unsigned int _nfft,
               TC *         _x,
               TC *         _y,
               int          _dir,
               int          _method)
{
    // create plan
    FFT(plan) plan = FFT(_create_plan)(_nfft, _x, _y, _dir, _method);

    // execute fft
    FFT(_execute)(plan);

    // destroy plan
    FFT(_destroy_plan)(plan);
}

// perform _n-point fft shift
void FFT(_shift)(TC *_x, unsigned int _n)
{
    unsigned int i, n2;
    if (_n%2)
        n2 = (_n-1)/2;
    else
        n2 = _n/2;

    TC tmp;
    for (i=0; i<n2; i++) {
        tmp = _x[i];
        _x[i] = _x[i+n2];
        _x[i+n2] = tmp;
    }
}

