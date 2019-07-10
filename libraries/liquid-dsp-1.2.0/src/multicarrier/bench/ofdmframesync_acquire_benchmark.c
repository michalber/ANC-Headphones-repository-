/*
 * Copyright (c) 2007, 2009, 2010, 2011 Joseph Gaeddert
 * Copyright (c) 2007, 2009, 2010, 2011  Virginia Polytechnic
 *                                 Institute & State University
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
#include <string.h>
#include <math.h>
#include <assert.h>
#include <sys/resource.h>
#include "liquid.h"

#define OFDMFRAMESYNC_ACQUIRE_BENCH_API(M,CP_LEN)   \
(   struct rusage *_start,                          \
    struct rusage *_finish,                         \
    unsigned long int *_num_iterations)             \
{ ofdmframesync_acquire_bench(_start, _finish, _num_iterations, M, CP_LEN); }

// Helper function to keep code base small
void ofdmframesync_acquire_bench(struct rusage *_start,
                                 struct rusage *_finish,
                                 unsigned long int *_num_iterations,
                                 unsigned int _num_subcarriers,
                                 unsigned int _cp_len)
{
    // options
    unsigned int M       = _num_subcarriers;
    unsigned int cp_len  = _cp_len;

    //
    unsigned int num_symbols_S0 = 2;    // number of S0 symbols
    unsigned int num_symbols_S1 = 2;    // number of S0 symbols

    // derived values
    unsigned int num_samples = num_symbols_S0*M +           // short PLCP sequence
                               num_symbols_S1*M + cp_len;   // long PLCP sequence

    // create synthesizer/analyzer objects
    ofdmframegen fg = ofdmframegen_create(M, cp_len, NULL);
    //ofdmframegen_print(fg);

    ofdmframesync fs = ofdmframesync_create(M,cp_len,NULL,NULL,NULL);

    unsigned int i;
    float complex s0[M];            // short PLCP sequence
    float complex s1[M];            // long PLCP sequence
    float complex y[num_samples];   // frame samples

    // generate sequences
    ofdmframegen_write_S0(fg, s0);
    ofdmframegen_write_S1(fg, s1);

    // assemble full frame
    unsigned int n=0;

    // write short sequence(s)
    for (i=0; i<num_symbols_S0; i++) {
        memmove(&y[n], s0, M*sizeof(float complex));
        n += M;
    }

    // write long sequence cyclic prefix
    memmove(&y[n], &s1[M-cp_len], cp_len*sizeof(float complex));
    n += cp_len;

    // write long sequence(s)
    for (i=0; i<num_symbols_S1; i++) {
        memmove(&y[n], s1, M*sizeof(float complex));
        n += M;
    }

    assert(n == num_samples);

    // add noise
    for (i=0; i<num_samples; i++)
        y[i] += 0.02f*randnf()*cexpf(_Complex_I*2*M_PI*randf());

    // start trials
    *_num_iterations /= M*sqrtf(M);
    getrusage(RUSAGE_SELF, _start);
    for (i=0; i<(*_num_iterations); i++) {
        //
        ofdmframesync_execute(fs,y,num_samples);

        //
        ofdmframesync_reset(fs);
    }
    getrusage(RUSAGE_SELF, _finish);
    //*_num_iterations *= 4;

    // destroy objects
    ofdmframegen_destroy(fg);
    ofdmframesync_destroy(fs);
}

//
void benchmark_ofdmframesync_acquire_n64    OFDMFRAMESYNC_ACQUIRE_BENCH_API(64, 8)
void benchmark_ofdmframesync_acquire_n128   OFDMFRAMESYNC_ACQUIRE_BENCH_API(128,16)
void benchmark_ofdmframesync_acquire_n256   OFDMFRAMESYNC_ACQUIRE_BENCH_API(256,32)
void benchmark_ofdmframesync_acquire_n512   OFDMFRAMESYNC_ACQUIRE_BENCH_API(512,64)

