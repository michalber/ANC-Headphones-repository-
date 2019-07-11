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
// Solve linear system of equations using conjugate gradient method
//
// References:
//  [Schewchuk:1994] Jonathon Richard Shewchuk, "An Introduction to
//      the Conjugate Gradient Method Without the Agonizing Pain,"
//      Manuscript, August, 1994.
//

#include <math.h>
#include <string.h>

#include "liquid.internal.h"

#define DEBUG_CGSOLVE 0

// solve linear system of equations using conjugate gradient method
//  _A      :   symmetric positive definite matrix [size: _n x _n]
//  _n      :   system dimension
//  _b      :   equality [size: _n x 1]
//  _x      :   solution estimate [size: _n x 1]
//  _opts   :   options (ignored for now)
void MATRIX(_cgsolve)(T * _A,
                      unsigned int _n,
                      T * _b,
                      T * _x,
                      void * _opts)
{
    // validate input
    if (_n == 0) {
        fprintf(stderr,"error: matrix_cgsolve(), system dimension cannot be zero\n");
        exit(1);
    }

    // options
    unsigned int max_iterations = 4*_n; // maximum number of iterations
    double tol = 1e-6;                  // error tolerance

    unsigned int j;

    // TODO : check options
    //  1. set initial _x0
    //  2. max number of iterations
    //  3. residual tolerance

    // allocate memory for arrays
    T x0[_n], x1[_n];   // iterative vector x (solution estimate)
    T d0[_n], d1[_n];   // iterative vector d
    T r0[_n], r1[_n];   // iterative vector r (step direction)
    T q[_n];            // A * d0
    T Ax1[_n];          // A * x1

    // scalars
    T delta_init;       // b^T * b0
    T delta0;           // r0^T * r0
    T delta1;           // r1^T * r1
    T gamma;            // d0^T * q
    T alpha;
    T beta;
    double res;         // residual
    double res_opt=0.0; // residual of best solution

    // initialize x0 to {0, 0, ... 0}
    for (j=0; j<_n; j++)
        x0[j] = 0.0;

    // d0 = b - A*x0 (assume x0 = {0, 0, 0, ...0})
    for (j=0; j<_n; j++)
        d0[j] = _b[j];

    // r0 = d0
    memmove(r0, d0, _n*sizeof(T));

    // delta_init = b^T * b
    MATRIX(_transpose_mul)(_b, _n, 1, &delta_init);

    // delta0 = r0^T * r0
    MATRIX(_transpose_mul)(r0, _n, 1, &delta0);

    // save best solution
    memmove(_x, x0, _n*sizeof(T));
    unsigned int i=0;   // iteration counter
    while ( (i < max_iterations) && (creal(delta0) > tol*tol*creal(delta_init)) ) {
#if DEBUG_CGSOLVE
        printf("*********** %4u / %4u (max) **************\n", i, max_iterations);
        printf("  comparing %12.4e > %12.4e\n", creal(delta0), tol*tol*creal(delta_init));
#endif

        // q = A*d0
        MATRIX(_mul)(_A, _n, _n,
                     d0, _n,  1,
                     q,  _n,  1);

        // gamma = d0^T * q
        gamma = 0.0;
        for (j=0; j<_n; j++)
            gamma += conj(d0[j]) * q[j];

        // step size: alpha = (r0^T * r0) / (d0^T * A * d0)
        //                  = delta0 / gamma
        alpha = delta0 / gamma;
#if DEBUG_CGSOLVE
        printf("  alpha  = %12.8f\n", crealf(alpha));
        printf("  delta0 = %12.8f\n", crealf(delta0));
#endif

        // update x
        for (j=0; j<_n; j++)
            x1[j] = x0[j] + alpha*d0[j];

#if DEBUG_CGSOLVE
        printf("  x:\n");
        MATRIX(_print)(x1, _n, 1);
#endif

        // update r
        if ( ((i+1)%50) == 0) {
            // peridically re-compute: r = b - A*x1
            MATRIX(_mul)(_A,  _n, _n,
                         x1,  _n,  1,
                         Ax1, _n, 1);
            for (j=0; j<_n; j++)
                r1[j] = _b[j] - Ax1[j];
        } else {
            for (j=0; j<_n; j++)
                r1[j] = r0[j] - alpha*q[j];
        }

        // delta1 = r1^T * r1
        MATRIX(_transpose_mul)(r1, _n, 1, &delta1);

        // update beta
        beta = delta1 / delta0;

        // d1 = r + beta*d0
        for (j=0; j<_n; j++)
            d1[j] = r1[j] + beta*d0[j];

        // compute residual
        res = sqrt( cabs(delta1) / cabs(delta_init) );
        if (i==0 || res < res_opt) {
            // save best solution
            res_opt = res;
            memmove(_x, x1, _n*sizeof(T));
        }
#if DEBUG_CGSOLVE
        printf("  res    = %12.4e\n", res);
#endif

        // copy old x, d, r, delta
        memmove(x0, x1, _n*sizeof(T));
        memmove(d0, d1, _n*sizeof(T));
        memmove(r0, r1, _n*sizeof(T));
        delta0 = delta1;

        // increment counter
        i++;
    }
}
