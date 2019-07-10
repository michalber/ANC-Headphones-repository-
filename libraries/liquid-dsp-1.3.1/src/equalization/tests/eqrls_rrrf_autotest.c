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

#include "autotest/autotest.h"
#include "liquid.h"

// constant data sequence
const float eqrls_rrrf_autotest_data_sequence[64] = {
    -1.0, -1.0,  1.0, -1.0,  1.0, -1.0,  1.0, -1.0, 
    -1.0,  1.0,  1.0, -1.0, -1.0,  1.0, -1.0,  1.0, 
     1.0, -1.0, -1.0, -1.0,  1.0,  1.0, -1.0,  1.0, 
    -1.0,  1.0,  1.0,  1.0,  1.0,  1.0, -1.0, -1.0,
     1.0,  1.0, -1.0, -1.0,  1.0, -1.0,  1.0, -1.0, 
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 
     1.0, -1.0, -1.0, -1.0,  1.0,  1.0, -1.0,  1.0, 
    -1.0,  1.0,  1.0, -1.0,  1.0, -1.0,  1.0, -1.0
};

// 
// AUTOTEST: channel filter: delta with zero delay
//
void autotest_eqrls_rrrf_01()
{
    float tol=1e-2f;        // error tolerance

    // fixed parameters (do not change)
    unsigned int h_len=4;   // channel filter length
    unsigned int p=6;       // equalizer order
    unsigned int n=64;      // number of symbols to observe

    // bookkeeping variables
    float y[n];         // received data sequence (filtered by channel)
    //float d_hat[n];   // recovered data sequence
    float h[h_len];     // channel filter coefficients
    float w[p];         // equalizer filter coefficients
    unsigned int i;

    // create equalizer
    eqrls_rrrf eq = eqrls_rrrf_create(NULL, p);

    // create channel filter
    h[0] = 1.0f;
    for (i=1; i<h_len; i++)
        h[i] = 0.0f;
    firfilt_rrrf f = firfilt_rrrf_create(h,h_len);

    // data sequence
    float *d = (float*) eqrls_rrrf_autotest_data_sequence;

    // filter data signal through channel
    for (i=0; i<n; i++) {
        firfilt_rrrf_push(f,d[i]);
        firfilt_rrrf_execute(f,&y[i]);
    }

    // initialize weights, train equalizer
    for (i=0; i<p; i++)
        w[i] = 0;
    eqrls_rrrf_train(eq, w, y, d, n);

    // compare filter taps
    CONTEND_DELTA(w[0], 1.0f, tol);
    for (i=1; i<p; i++)
        CONTEND_DELTA(w[i], 0.0f, tol);

    // clean up objects
    firfilt_rrrf_destroy(f);
    eqrls_rrrf_destroy(eq);
}

