/*
 * Copyright (c) 2011 Joseph Gaeddert
 * Copyright (c) 2011 Virginia Polytechnic Institute & State University
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
// sumproduct.c
//
// floating-point implementation of the sum-product algorithm
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "liquid.internal.h"

#define DEBUG_SUMPRODUCT 0

// phi(x) = -logf( tanhf( x/2 ) )
float sumproduct_phi(float _x) {
    return -logf(tanhf(_x/2.0f + 1e-12));
}

// iterate over the sum-product algorithm:
// returns 1 if parity checks, 0 otherwise
//  _H          :   parity check matrix [size: _m x _n]
//  _m          :   rows
//  _n          :   cols
//  _LLR        :   received signal (soft bits, LLR) [size: _n x 1]
//  _c_hat      :   estimated transmitted signal [size: _n x 1]
//  _max_steps  :   maximum number of steps before bailing
int fec_sumproduct(unsigned char * _H,
                   unsigned int _m,
                   unsigned int _n,
                   float * _LLR,
                   unsigned char * _c_hat,
                   unsigned int _max_steps)
{
    // TODO : validate input
    if (_n == 0 || _m == 0) {
        fprintf(stderr,"error: fec_sumproduct(), matrix dimensions cannot be zero\n");
        exit(1);
    }

    // internal variables
    unsigned int num_iterations = 0;
    float Lq[_m*_n];
    float Lr[_m*_n];
    float Lc[_n];
    float LQ[_n];
    unsigned char parity[_m];
    unsigned int i;
    unsigned int j;
    int parity_pass;
    int continue_running = 1;

    // initialize Lq with log-likelihood values
    for (i=0; i<_n; i++)
        Lc[i] = _LLR[i];
        //Lc[i] = 2.0f * _y[i] / (sigma*sigma);

    for (j=0; j<_m; j++) {
        for (i=0; i<_n; i++) {
            Lq[j*_n+i] = _H[j*_n+i] ? Lc[i] : 0.0f;
        }
    }
#if DEBUG_SUMPRODUCT
    // print Lc
    matrixf_print(Lc,1,_n);
#endif

    // TODO : run multiple iterations
    while (continue_running) {
#if DEBUG_SUMPRODUCT
        //
        printf("\n");
        printf("************* iteration %u ****************\n", num_iterations);
#endif

        // step sum-product algorithm
        parity_pass = fec_sumproduct_step(_H,_m,_n,_c_hat,Lq,Lr,Lc,LQ,parity);

        // update...
        num_iterations++;
        if (parity_pass || num_iterations == _max_steps)
            continue_running = 0;
    }

    return parity_pass;
}

// sum-product algorithm, returns 1 if parity checks, 0 otherwise
//  _H      :   parity check matrix [size: _m x _n]
//  _m      :   rows
//  _n      :   cols
//  _c_hat  :   estimated transmitted signal [size: _n x 1]
//
// internal state arrays
//  _Lq     :   [size: _m x _n]
//  _Lr     :   [size: _m x _n]
//  _Lc     :   [size: _n x 1]
//  _LQ     :   [size: _n x 1]
//  _parity :   _H * _c_hat [size: _m x 1]
int fec_sumproduct_step(unsigned char * _H,
                        unsigned int _m,
                        unsigned int _n,
                        unsigned char * _c_hat,
                        float * _Lq,
                        float * _Lr,
                        float * _Lc,
                        float * _LQ,
                        unsigned char * _parity)
{
    unsigned int i;
    unsigned int j;
    unsigned int ip;
    unsigned int jp;
    float alpha_prod;
    float phi_sum;
    int parity_pass;

    // compute Lr
    for (i=0; i<_n; i++) {
        for (j=0; j<_m; j++) {
            alpha_prod = 1.0f;
            phi_sum    = 0.0f;
            for (ip=0; ip<_n; ip++) {
                if (_H[j*_n+ip]==1 && i != ip) {
                    float alpha = _Lq[j*_n+ip] > 0.0f ? 1.0f : -1.0f;
                    float beta  = fabsf(_Lq[j*_n+ip]);
                    phi_sum += sumproduct_phi(beta);
                    alpha_prod *= alpha;
                }
            }
            _Lr[j*_n+i] = alpha_prod * sumproduct_phi(phi_sum);
        }
    }

#if DEBUG_SUMPRODUCT
    // print Lq
    matrixf_print(_Lq,_m,_n);

    // print Lr
    matrixf_print(_Lr,_m,_n);
#endif

    // compute next iteration of Lq
    for (i=0; i<_n; i++) {
        for (j=0; j<_m; j++) {
            // initialize with LLR
            _Lq[j*_n+i] = _Lc[i];

            for (jp=0; jp<_m; jp++) {
                if (_H[jp*_n+i]==1 && j != jp)
                    _Lq[j*_n+i] += _Lr[jp*_n+i];
            }
        }
    }

#if DEBUG_SUMPRODUCT
    // print Lq
    matrixf_print(_Lq,_m,_n);
#endif

    // compute LQ
    for (i=0; i<_n; i++) {
        _LQ[i] = _Lc[i];  // initialize with LLR value

        for (j=0; j<_m; j++) {
            if (_H[j*_n+i]==1)
                _LQ[i] += _Lr[j*_n+i];
        }
    }

#if DEBUG_SUMPRODUCT
    // print LQ
    matrixf_print(_LQ,1,_n);
#endif

    // compute hard-decoded value
    for (i=0; i<_n; i++)
        _c_hat[i] = _LQ[i] < 0.0f ? 1 : 0;

    // compute parity check: p = H*c_hat
    for (j=0; j<_m; j++) {
        _parity[j] = 0;

        // 
        for (i=0; i<_n; i++)
            _parity[j] += _H[j*_n+i] * _c_hat[i];

        // math is modulo 2
        _parity[j] %= 2;
    }

    // check parity
    parity_pass = 1;
    for (j=0; j<_m; j++) {
        if (_parity[j]) parity_pass = 0;
    }

#if DEBUG_SUMPRODUCT
    // print hard-decision output
    printf("    : c hat = [");
    for (i=0; i<_n; i++)
        printf(" %1u", _c_hat[i]);
    printf(" ],  ");

    // print parity
    printf("parity = [");
    for (j=0; j<_m; j++)
        printf(" %1u", _parity[j]);
    printf(" ],  ");

    printf(" (%s)\n", parity_pass ? "pass" : "FAIL");
#endif

    return parity_pass;
}

