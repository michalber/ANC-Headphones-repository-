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

#include <complex.h>
#include "autotest/autotest.h"
#include "liquid.h"

//
// test phase-locked loop
//  _type           :   NCO type (e.g. LIQUID_NCO)
//  _phase_offset   :   initial phase offset
//  _freq_offset    :   initial frequency offset
//  _pll_bandwidth  :   bandwidth of phase-locked loop
//  _num_iterations :   number of iterations to run
//  _tol            :   error tolerance
void nco_crcf_pll_test(int          _type,
                       float        _phase_offset,
                       float        _freq_offset,
                       float        _pll_bandwidth,
                       unsigned int _num_iterations,
                       float        _tol)
{
    // objects
    nco_crcf nco_tx = nco_crcf_create(_type);
    nco_crcf nco_rx = nco_crcf_create(_type);

    // initialize objects
    nco_crcf_set_phase(nco_tx, _phase_offset);
    nco_crcf_set_frequency(nco_tx, _freq_offset);
    nco_crcf_pll_set_bandwidth(nco_rx, _pll_bandwidth);

    // run loop
    unsigned int i;
    float phase_error;
    float complex r, v;
    for (i=0; i<_num_iterations; i++) {
        // received complex signal
        nco_crcf_cexpf(nco_tx,&r);
        nco_crcf_cexpf(nco_rx,&v);

        // error estimation
        phase_error = cargf(r*conjf(v));

        // update pll
        nco_crcf_pll_step(nco_rx, phase_error);

        // update nco objects
        nco_crcf_step(nco_tx);
        nco_crcf_step(nco_rx);
    }

    // ensure phase of oscillators is locked
    float nco_tx_phase = nco_crcf_get_phase(nco_tx);
    float nco_rx_phase = nco_crcf_get_phase(nco_rx);
    CONTEND_DELTA(nco_tx_phase, nco_rx_phase, _tol);

    // ensure frequency of oscillators is locked
    float nco_tx_freq = nco_crcf_get_frequency(nco_tx);
    float nco_rx_freq = nco_crcf_get_frequency(nco_rx);
    CONTEND_DELTA(nco_tx_freq, nco_rx_freq, _tol);

    if (liquid_autotest_verbose) {
        printf("  phase error : %12.4e, frequency error : %12.4e\n",
                cargf( cexpf(_Complex_I*(nco_tx_phase-nco_rx_phase)) ),
                nco_tx_freq-nco_rx_freq);
    }

    // clean it up
    nco_crcf_destroy(nco_tx);
    nco_crcf_destroy(nco_rx);
}

//
// AUTOTEST: test frequency and phase offsets
//
void autotest_vco_crcf_pll_phase()
{
    float tol = 0.01f;

    // test various phase offsets
    nco_crcf_pll_test(LIQUID_NCO, -M_PI/1.1f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO, -M_PI/2.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO, -M_PI/4.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO, -M_PI/8.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  M_PI/8.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  M_PI/4.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  M_PI/2.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  M_PI/1.1f,  0.0f, 0.1f, 256, tol);
    
    // test various frequency offsets
    nco_crcf_pll_test(LIQUID_NCO,  0.0f,      -1.6f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  0.0f,      -0.8f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  0.0f,      -0.4f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  0.0f,      -0.2f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  0.0f,       0.2f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  0.0f,       0.4f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  0.0f,       0.8f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_NCO,  0.0f,       1.6f, 0.1f, 256, tol);
}

//
// AUTOTEST: test frequency and phase offsets
//
void autotest_nco_crcf_pll_phase()
{
    float tol = 1e-4f;

    // test various phase offsets
    nco_crcf_pll_test(LIQUID_VCO, -M_PI/1.1f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO, -M_PI/2.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO, -M_PI/4.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO, -M_PI/8.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  M_PI/8.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  M_PI/4.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  M_PI/2.0f,  0.0f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  M_PI/1.1f,  0.0f, 0.1f, 256, tol);
    
    // test various frequency offsets
    nco_crcf_pll_test(LIQUID_VCO,  0.0f,      -1.6f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  0.0f,      -0.8f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  0.0f,      -0.4f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  0.0f,      -0.2f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  0.0f,       0.2f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  0.0f,       0.4f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  0.0f,       0.8f, 0.1f, 256, tol);
    nco_crcf_pll_test(LIQUID_VCO,  0.0f,       1.6f, 0.1f, 256, tol);
}

