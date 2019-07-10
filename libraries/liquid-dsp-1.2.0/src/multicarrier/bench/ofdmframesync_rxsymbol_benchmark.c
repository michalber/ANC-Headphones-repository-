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

#define OFDMFRAMESYNC_RXSYMBOL_BENCH_API(M,CP_LEN)  \
(   struct rusage *_start,                          \
    struct rusage *_finish,                         \
    unsigned long int *_num_iterations)             \
{ ofdmframesync_rxsymbol_bench(_start, _finish, _num_iterations, M, CP_LEN); }

// Helper function to keep code base small
void ofdmframesync_rxsymbol_bench(struct rusage *_start,
                                 struct rusage *_finish,
                                 unsigned long int *_num_iterations,
                                 unsigned int _num_subcarriers,
                                 unsigned int _cp_len)
{
    // options
    modulation_scheme ms = LIQUID_MODEM_QPSK;
    unsigned int M       = _num_subcarriers;
    unsigned int cp_len  = _cp_len;

    //
    unsigned int num_symbols_S0 = 2;    // number of S0 symbols
    unsigned int num_symbols_S1 = 2;    // number of S0 symbols

    // create synthesizer/analyzer objects
    ofdmframegen fg = ofdmframegen_create(M, cp_len, NULL);
    //ofdmframegen_print(fg);

    modem mod = modem_create(ms);

    ofdmframesync fs = ofdmframesync_create(M,cp_len,NULL,NULL,NULL);

    unsigned int i;
    float complex s0[M];        // short PLCP sequence
    float complex s1[M];        // long PLCP sequence
    float complex X[M];         // channelized symbol
    float complex x[M+cp_len];  // time-domain symbol

    // generate sequences
    ofdmframegen_write_S0(fg, s0);
    ofdmframegen_write_S1(fg, s1);

    // synchronize short sequence(s)
    for (i=0; i<num_symbols_S0; i++)
        ofdmframesync_execute(fs, s0, M);

    // synchronize long sequence cyclic prefix
    ofdmframesync_execute(fs, &s1[M-cp_len], cp_len);

    // write long sequence(s)
    for (i=0; i<num_symbols_S1; i++)
        ofdmframesync_execute(fs, s1, M);

    // modulate data symbols (use same symbol, ignore pilot phase)
    unsigned int s;
    for (i=0; i<M; i++) {
        s = modem_gen_rand_sym(mod);
        modem_modulate(mod,s,&X[i]);
    }

    ofdmframegen_writesymbol(fg, X, x);

    // add noise
    for (i=0; i<M+cp_len; i++)
        x[i] += 0.02f*randnf()*cexpf(_Complex_I*2*M_PI*randf());

    // normalize number of iterations
    *_num_iterations /= M;

    // start trials
    getrusage(RUSAGE_SELF, _start);
    for (i=0; i<(*_num_iterations); i++) {
        //
        ofdmframesync_execute(fs, x, M+cp_len);
        ofdmframesync_execute(fs, x, M+cp_len);
        ofdmframesync_execute(fs, x, M+cp_len);
        ofdmframesync_execute(fs, x, M+cp_len);
    }
    getrusage(RUSAGE_SELF, _finish);
    *_num_iterations *= 4;

    // destroy objects
    ofdmframegen_destroy(fg);
    ofdmframesync_destroy(fs);
    modem_destroy(mod);
}

//
void benchmark_ofdmframesync_rxsymbol_n64   OFDMFRAMESYNC_RXSYMBOL_BENCH_API(64, 8)
void benchmark_ofdmframesync_rxsymbol_n128  OFDMFRAMESYNC_RXSYMBOL_BENCH_API(128,16)
void benchmark_ofdmframesync_rxsymbol_n256  OFDMFRAMESYNC_RXSYMBOL_BENCH_API(256,32)
void benchmark_ofdmframesync_rxsymbol_n512  OFDMFRAMESYNC_RXSYMBOL_BENCH_API(512,64)

