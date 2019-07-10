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

//
// Matrix inverse method definitions
//

#include "liquid.internal.h"

void MATRIX(_inv)(T * _X, unsigned int _XR, unsigned int _XC)
{
    // ensure lengths are valid
    if (_XR != _XC ) {
        fprintf(stderr, "error: matrix_inv(), invalid dimensions\n");
        exit(1);
    }

    // X:
    //  x11 x12 ... x1n
    //  x21 x22 ... x2n
    //  ...
    //  xn1 xn2 ... xnn

    // allocate temporary memory
    T x[2*_XR*_XC];
    unsigned int xr = _XR;
    unsigned int xc = _XC*2;

    // x:
    //  x11 x12 ... x1n 1   0   ... 0
    //  x21 x22 ... x2n 0   1   ... 0
    //  ...
    //  xn1 xn2 ... xnn 0   0   ... 1
    unsigned int r,c;
    for (r=0; r<_XR; r++) {
        // copy matrix elements
        for (c=0; c<_XC; c++)
            matrix_access(x,xr,xc,r,c) = matrix_access(_X,_XR,_XC,r,c);

        // append identity matrix
        for (c=0; c<_XC; c++)
            matrix_access(x,xr,xc,r,_XC+c) = (r==c) ? 1 : 0;
    }

    // perform Gauss-Jordan elimination on x
    // x:
    //  1   0   ... 0   y11 y12 ... y1n
    //  0   1   ... 0   y21 y22 ... y2n
    //  ...
    //  0   0   ... 1   yn1 yn2 ... ynn
    MATRIX(_gjelim)(x,xr,xc);

    // copy result from right half of x
    for (r=0; r<_XR; r++) {
        for (c=0; c<_XC; c++)
            matrix_access(_X,_XR,_XC,r,c) = matrix_access(x,xr,xc,r,_XC+c);
    }
}

// Gauss-Jordan elmination
void MATRIX(_gjelim)(T * _X, unsigned int _XR, unsigned int _XC)
{
    unsigned int r, c;

    // choose pivot rows based on maximum element along column
    float v;
    float v_max=0.;
    unsigned int r_opt=0;
    unsigned int r_hat;
    for (r=0; r<_XR; r++) {

        // check values along this column and find the maximum
        for (r_hat=r; r_hat<_XR; r_hat++) {
            v = cabsf( matrix_access(_X,_XR,_XC,r_hat,r) );
            // swap rows if necessary
            if (v > v_max || r_hat==r) {
                r_opt = r_hat;
                v_max = v;
            }
        }

        // if the maximum is zero, matrix is singular
        if (v_max == 0.0f) {
            fprintf(stderr,"warning: matrix_gjelim(), matrix singular to machine precision\n");
        }

        // if row does not match column (e.g. maximum value does not
        // lie on the diagonal) swap the rows
        if (r != r_opt) {
            MATRIX(_swaprows)(_X,_XR,_XC,r,r_opt);
        }

        // pivot on the diagonal element
        MATRIX(_pivot)(_X,_XR,_XC,r,r);
    }

    // scale by diagonal
    T g;
    for (r=0; r<_XR; r++) {
        g = 1 / matrix_access(_X,_XR,_XC,r,r);
        for (c=0; c<_XC; c++)
            matrix_access(_X,_XR,_XC,r,c) *= g;
    }
}

// pivot on element _r, _c
void MATRIX(_pivot)(T * _X, unsigned int _XR, unsigned int _XC, unsigned int _r, unsigned int _c)
{
    T v = matrix_access(_X,_XR,_XC,_r,_c);
    if (v==0) {
        fprintf(stderr, "warning: matrix_pivot(), pivoting on zero\n");
        return;
    }
    unsigned int r,c;

    // pivot using back-substitution
    T g;    // multiplier
    for (r=0; r<_XR; r++) {

        // skip over pivot row
        if (r == _r)
            continue;

        // compute multiplier
        g = matrix_access(_X,_XR,_XC,r,_c) / v;

        // back-substitution
        for (c=0; c<_XC; c++) {
            matrix_access(_X,_XR,_XC,r,c) = g*matrix_access(_X,_XR,_XC,_r,c) -
                                              matrix_access(_X,_XR,_XC, r,c);
        }
    }
}

void MATRIX(_swaprows)(T * _X, unsigned int _XR, unsigned int _XC, unsigned int _r1, unsigned int _r2)
{
    unsigned int c;
    T v_tmp;
    for (c=0; c<_XC; c++) {
        v_tmp = matrix_access(_X,_XR,_XC,_r1,c);
        matrix_access(_X,_XR,_XC,_r1,c) = matrix_access(_X,_XR,_XC,_r2,c);
        matrix_access(_X,_XR,_XC,_r2,c) = v_tmp;
    }
}
