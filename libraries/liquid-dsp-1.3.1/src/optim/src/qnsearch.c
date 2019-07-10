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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "liquid.internal.h"

#define LIQUID_qnsearch_GAMMA_MIN 0.000001

qnsearch qnsearch_create(void * _userdata,
                         float * _v,
                         unsigned int _num_parameters,
                         utility_function _u,
                         int _minmax)
{
    qnsearch q = (qnsearch) malloc( sizeof(struct qnsearch_s) );

    // initialize public values
    q->delta = 1e-6f;   //_delta;
    q->gamma = 1e-3f;   //_gamma;
    q->dgamma = 0.99f;
    q->gamma_hat = q->gamma;

    q->userdata = _userdata;
    q->v = _v;
    q->num_parameters = _num_parameters;
    q->get_utility = _u;
    q->minimize = ( _minmax == LIQUID_OPTIM_MINIMIZE ) ? 1 : 0;

    // initialize internal memory arrays
    q->B        = (float*) calloc( q->num_parameters*q->num_parameters, sizeof(float));
    q->H        = (float*) calloc( q->num_parameters*q->num_parameters, sizeof(float));
    q->p        = (float*) calloc( q->num_parameters, sizeof(float) );
    q->gradient = (float*) calloc( q->num_parameters, sizeof(float) );
    q->gradient0= (float*) calloc( q->num_parameters, sizeof(float) );
    q->v_prime  = (float*) calloc( q->num_parameters, sizeof(float) );
    q->dv       = (float*) calloc( q->num_parameters, sizeof(float) );
    q->utility = q->get_utility(q->userdata, q->v, q->num_parameters);

    qnsearch_reset(q);

    return q;
}

void qnsearch_destroy(qnsearch _q)
{
    free(_q->B);
    free(_q->H);

    free(_q->p);
    free(_q->gradient);
    free(_q->gradient0);
    free(_q->v_prime);
    free(_q->dv);
    free(_q);
}

void qnsearch_print(qnsearch _q)
{
    printf("[%.3f] ", _q->utility);
    unsigned int i;
    for (i=0; i<_q->num_parameters; i++)
        printf("%.3f ", _q->v[i]);
    printf("\n");
}

void qnsearch_reset(qnsearch _q)
{
    _q->gamma_hat = _q->gamma;

    // set B to identity matrix
    unsigned int i,j,n=0;
    for (i=0; i<_q->num_parameters; i++) {
        for (j=0; j<_q->num_parameters; j++) {
            _q->B[n++] = (i==j) ? 1.0f : 0.0f;
        }
    }

    _q->utility = _q->get_utility(_q->userdata, _q->v, _q->num_parameters);
}

void qnsearch_step(qnsearch _q)
{
    unsigned int i;
    unsigned int n = _q->num_parameters;

    // compute normalized gradient vector
    qnsearch_compute_gradient(_q);
    //qnsearch_normalize_gradient(_q);

    // TODO : perform line search to find optimal gamma

    // compute search direction
#if 0
    matrixf_mul(_q->B,        n, n,
                _q->gradient, n, 1,
                _q->p,        n, 1);
    for (i=0; i<_q->num_parameters; i++)
        _q->p[i] = -_q->p[i];
#else
    qnsearch_compute_Hessian(_q);
    matrixf_inv(_q->H, n, n);
    matrixf_mul(_q->H, n, n,
                _q->gradient, n, 1,
                _q->p, n, 1);
#endif

    // compute step vector
    for (i=0; i<_q->num_parameters; i++)
        _q->dv[i] = -_q->gamma_hat * _q->p[i];

    // apply change
    for (i=0; i<_q->num_parameters; i++) {
        _q->v[i] += _q->dv[i];
    }

    // TODO update inverse Hessian approximation

    // store previous gradient
    memmove(_q->gradient0, _q->gradient, (_q->num_parameters)*sizeof(float));

    // update utility
    float u_prime = _q->get_utility(_q->userdata, _q->v, _q->num_parameters);

    if (u_prime > _q->utility) {
        _q->gamma_hat *= 0.99f;
    } else {
        _q->gamma_hat *= 1.001f;
    }

    _q->utility = u_prime;
}

float qnsearch_run(qnsearch _q,
                   unsigned int _max_iterations,
                   float _target_utility)
{
    unsigned int i=0;
    do {
        i++;
        qnsearch_step(_q);
        _q->utility = _q->get_utility(_q->userdata, _q->v, _q->num_parameters);

    } while (
        optim_threshold_switch(_q->utility, _target_utility, _q->minimize) &&
        i < _max_iterations);

    return _q->utility;
}

// 
// internal
//

// compute gradient
void qnsearch_compute_gradient(qnsearch _q)
{
    unsigned int i;
    float f_prime;

    // reset v_prime
    memmove(_q->v_prime, _q->v, (_q->num_parameters)*sizeof(float));

    for (i=0; i<_q->num_parameters; i++) {
        _q->v_prime[i] += _q->delta;
        f_prime = _q->get_utility(_q->userdata, _q->v_prime, _q->num_parameters);
        _q->v_prime[i] -= _q->delta;
        _q->gradient[i] = (f_prime - _q->utility) / _q->delta;
    }
}

// normalize gradient vector to unity
void qnsearch_normalize_gradient(qnsearch _q)
{
    // normalize gradient
    float sig = 0.0f;
    unsigned int i;
    for (i=0; i<_q->num_parameters; i++)
        sig += _q->gradient[i] * _q->gradient[i];

    sig = 1.0f / sqrtf(sig/(float)(_q->num_parameters));

    for (i=0; i<_q->num_parameters; i++)
        _q->gradient[i] *= sig;
}

// compute Hessian
void qnsearch_compute_Hessian(qnsearch _q)
{
    unsigned int i, j;
    unsigned int n = _q->num_parameters;
    float f00, f01, f10, f11;
    float f0, f1, f2;
    float m0, m1;
    float delta = 1e-2f;

    // reset v_prime
    memmove(_q->v_prime, _q->v, (_q->num_parameters)*sizeof(float));


    for (i=0; i<_q->num_parameters; i++) {
        //for (j=0; j<_q->num_parameters; j++) {
        for (j=0; j<=i; j++) {
            if (i==j) {

                _q->v_prime[i] = _q->v[i] - delta;
                f0 = _q->get_utility(_q->userdata, _q->v_prime, _q->num_parameters);

                _q->v_prime[i] = _q->v[i];
                f1 = _q->get_utility(_q->userdata, _q->v_prime, _q->num_parameters);

                _q->v_prime[i] = _q->v[i] + delta;
                f2 = _q->get_utility(_q->userdata, _q->v_prime, _q->num_parameters);
                
                m0 = (f1 - f0) / delta;
                m1 = (f2 - f1) / delta;
                matrix_access(_q->H, n, n, i, j) = (m1 - m0) / delta;

            } else {

                // 0 0
                _q->v_prime[i] = _q->v[i] - delta;
                _q->v_prime[j] = _q->v[j] - delta;
                f00 = _q->get_utility(_q->userdata, _q->v_prime, _q->num_parameters);

                // 0 1
                _q->v_prime[i] = _q->v[i] - delta;
                _q->v_prime[j] = _q->v[j] + delta;
                f01 = _q->get_utility(_q->userdata, _q->v_prime, _q->num_parameters);

                // 1 0
                _q->v_prime[i] = _q->v[i] + delta;
                _q->v_prime[j] = _q->v[j] - delta;
                f10 = _q->get_utility(_q->userdata, _q->v_prime, _q->num_parameters);

                // 1 1
                _q->v_prime[i] = _q->v[i] + delta;
                _q->v_prime[j] = _q->v[j] + delta;
                f11 = _q->get_utility(_q->userdata, _q->v_prime, _q->num_parameters);

                // compute second partial derivative
                m0 = (f01 - f00) / (2.0f*delta);
                m1 = (f11 - f10) / (2.0f*delta);
                matrix_access(_q->H, n, n, i, j) = (m1 - m0) / (2.0f*delta);
                matrix_access(_q->H, n, n, j, i) = (m1 - m0) / (2.0f*delta);
            }
        }
    }
    //matrixf_print(_q->H, n, n);
    //exit(1);
}


