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
// direct digital synthesizer (up/down-converter)
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "liquid.internal.h"

// defined:
//  DDS()           name-mangling macro
//  T               coefficients type
//  RESAMP2()       halfband resampler
//  PRINTVAL()      print macro

struct DDS(_s) {
    // user-defined parameters
    unsigned int num_stages;    // number of halfband stages
    float fc0;                  // high-rate center frequency (-0.5,0.5)
    float bw0;                  // low-rate bandwidth (range?)
    float As0;                  // filter stop-band attenuation [dB]

    // derived values
    unsigned int rate;          // re-sampling rate (2^num_stages)

    // halfband decimation/interpolation stages
    RESAMP2() * halfband_resamp;
    float * fc;                 // filter center frequency
    float * ft;                 // filter transition bandwidth
    float * As;                 // filter stop-band attenuation [dB] array
    unsigned int * h_len;       // filter length

    // internal buffers
    unsigned int buffer_len;
    T * buffer0;
    T * buffer1;

    // low-rate mixing stage
    NCO() ncox;

    // down-converter scaling factor
    float zeta;
};

// create dds object
//  _num_stages     :   number of halfband stages
//  _fc             :   input carrier
//  _bw             :   input signal bandwidth
//  _As             :   stop-band attenuation
DDS() DDS(_create)(unsigned int _num_stages,
                   float _fc,
                   float _bw,
                   float _As)
{
    // create object
    DDS() q = (DDS()) malloc(sizeof(struct DDS(_s)));
    q->num_stages = _num_stages;
    q->rate = 1<<(q->num_stages);
    q->fc0 = _fc;
    q->bw0 = _bw;
    q->As0 = _As;

    // error checking
    if (q->fc0 > 0.5f || q->fc0 < -0.5f) {
        fprintf(stderr,"error: dds_xxxf_create(), frequency %12.4e is out of range [-0.5,0.5]\n", q->fc0);
        exit(1);
    }

    // allocate memory for filter properties
    q->fc    = (float*) malloc((q->num_stages)*sizeof(float));
    q->ft    = (float*) malloc((q->num_stages)*sizeof(float));
    q->As    = (float*) malloc((q->num_stages)*sizeof(float));
    q->h_len = (unsigned int*) malloc((q->num_stages)*sizeof(unsigned int));
    unsigned int i;
    float fc, bw;
    fc = 0.5*(1<<q->num_stages)*q->fc0; // filter center frequency
    bw = q->bw0;                        // signal bandwidth
    // TODO : compute/set filter bandwidths, lengths appropriately
    for (i=0; i<q->num_stages; i++) {
        q->fc[i] = fc;
        while (q->fc[i] >  0.5f) q->fc[i] -= 1.0f;
        while (q->fc[i] < -0.5f) q->fc[i] += 1.0f;

        // compute transition bandwidth
        q->ft[i] = 0.5f*(1.0f - bw);
        if (q->ft[i] > 0.45) q->ft[i] = 0.45f; // set maximum bandwidth
        q->As[i] = q->As0;

        // compute (estimate) required filter length
        //q->h_len[i] = i==0 ? 37 : q->h_len[i-1]*0.7;
        q->h_len[i] = estimate_req_filter_len(q->ft[i], q->As[i]);
        if ((q->h_len[i] % 2) == 0) q->h_len[i]++;

        // ensure h_len[i] is of form 4*m+1
        unsigned int m = (q->h_len[i]-1)/4;
        if (m < 1) m = 1;
        q->h_len[i] = 4*m+1;

        // update carrier, bandwidth parameters
        fc *= 0.5f;
        bw *= 0.5f;
    }

    // allocate memory for buffering
    q->buffer_len = q->rate;
    q->buffer0 = (T*) malloc((q->buffer_len)*sizeof(T));
    q->buffer1 = (T*) malloc((q->buffer_len)*sizeof(T));

    // allocate memory for resampler pointers and create objects
    q->halfband_resamp = (RESAMP2()*) malloc((q->num_stages)*sizeof(RESAMP()*));
    for (i=0; i<q->num_stages; i++) {
        q->halfband_resamp[i] = RESAMP2(_create)(q->h_len[i],
                                                 q->fc[i],
                                                 q->As[i]);
    }

    // set down-converter scaling factor
    q->zeta = 1.0f / ((float)(q->rate));

    // create NCO and set frequency
    q->ncox = NCO(_create)(LIQUID_VCO);
    // TODO : ensure range is in [-pi,pi]
    NCO(_set_frequency)(q->ncox, 2*M_PI*(q->rate)*(q->fc0));

    return q;
}

// destroy dds object, freeing all internally-allocated memory
void DDS(_destroy)(DDS() _q)
{
    // free filter parameter arrays
    free(_q->h_len);
    free(_q->fc);
    free(_q->ft);

    // destroy buffers
    free(_q->buffer0);
    free(_q->buffer1);

    // destroy halfband resampler objects
    unsigned int i;
    for (i=0; i<_q->num_stages; i++)
        RESAMP2(_destroy)(_q->halfband_resamp[i]);
    free(_q->halfband_resamp);

    // destroy NCO object
    NCO(_destroy)(_q->ncox);

    // destroy DDS object
    free(_q);
}

// print dds object internals
void DDS(_print)(DDS() _q)
{
    printf("direct digital synthesizer (dds), rate : %u\n", _q->rate);
    printf("      fc    : %8.5f\n", _q->fc0);
    printf("      bw    : %8.5f\n", _q->bw0);
    printf("      nco/f : %8.4f\n", _q->ncox->d_theta / (2.0f*M_PI));
    printf("      As    : %8.2f [dB]\n", _q->As0);
    printf("    halfband stages (low rate -> high rate) :\n");
    unsigned int i;
    for (i=0; i<_q->num_stages; i++) {
        printf("      [%3u] : fc = %8.5f, ft = %8.5f, %3u taps\n",
                    i,
                    _q->fc[i],
                    _q->ft[i],
                    _q->h_len[i]);
        //RESAMP2(_print)(_q->halfband_resamp[i]);
    }
    printf("    complexity : %12.4f\n",0.0f);
}

// reset dds object internals, clear filters and nco phase
void DDS(_reset)(DDS() _q)
{
    // reset internal filter state variables
    unsigned int i;
    for (i=0; i<_q->num_stages; i++) {
        RESAMP2(_clear)(_q->halfband_resamp[i]);
    }

    NCO(_set_phase)(_q->ncox,0.0f);
}

// execute decimator
//  _q      :   dds object
//  _x      :   input sample array [size: 2^num_stages x 1]
//  _y      :   output sample
void DDS(_decim_execute)(DDS() _q,
                         T * _x,
                         T * _y)
{
    // copy input data
    memmove(_q->buffer0, _x, (_q->rate)*sizeof(T));

    unsigned int k=_q->rate;    // number of inputs for this stage
    unsigned int s;     // stage counter
    unsigned int i;     // input counter
    unsigned int g;     // halfband resampler stage index (reversed)
    T * b0 = NULL;      // input buffer pointer
    T * b1 = NULL;      // output buffer pointer

    // iterate through each stage
    for (s=0; s<_q->num_stages; s++) {
        // length halves with each iteration
        k >>= 1;

        // set buffer pointers
        b0 = s%2 == 0 ? _q->buffer0 : _q->buffer1;
        b1 = s%2 == 1 ? _q->buffer0 : _q->buffer1;

        // execute halfband decimator
        g = _q->num_stages - s - 1;
        for (i=0; i<k; i++)
            RESAMP2(_decim_execute)(_q->halfband_resamp[g], &b0[2*i], &b1[i]);
    }

    // output value
    T y = b1[0];

    // increment NCO
    NCO(_mix_down)(_q->ncox, y, &y);
    NCO(_step)(_q->ncox);

    // set output, normalizing by scaling factor
    *_y = y * _q->zeta;
}

// execute interpolator
//  _q      :   dds object
//  _x      :   input sample
//  _y      :   output sample array [size: 2^num_stages x 1]
void DDS(_interp_execute)(DDS() _q,
                          T _x,
                          T * _y)
{
    // increment NCO
    NCO(_mix_up)(_q->ncox, _x, &_x);
    NCO(_step)(_q->ncox);

    unsigned int s;     // stage counter
    unsigned int i;     // input counter
    unsigned int k=1;   // number of inputs for this stage
    T * b0 = NULL;      // input buffer pointer
    T * b1 = NULL;      // output buffer pointer

    // set initial buffer value
    _q->buffer0[0] = _x;

    // iterate through each stage
    for (s=0; s<_q->num_stages; s++) {

        // set buffer pointers
        b0 = s%2 == 0 ? _q->buffer0 : _q->buffer1;
        b1 = s%2 == 1 ? _q->buffer0 : _q->buffer1;

        // execute halfband interpolator
        for (i=0; i<k; i++)
            RESAMP2(_interp_execute)(_q->halfband_resamp[s], b0[i], &b1[2*i]);
        
        // length doubles with each iteration
        k <<= 1;
    }

    // copy output data
    memmove(_y, b1, (_q->rate)*sizeof(T));
}

