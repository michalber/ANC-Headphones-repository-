/*
 * Copyright (c) 2007, 2009, 2010 Joseph Gaeddert
 * Copyright (c) 2007, 2009, 2010 Virginia Polytechnic Institute & State University
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

#include <assert.h>
#include "autotest/autotest.h"
#include "liquid.h"

//
// AUTOTEST: validate analysis correctness
//
void autotest_firpfbch_crcf_analysis()
{
    float tol = 1e-4f;              // error tolerance
    unsigned int num_channels=4;    // number of channels
    unsigned int p=5;               // filter length (symbols)
    unsigned int num_symbols=12;    // number of symbols

    // derived values
    unsigned int num_samples = num_channels * num_symbols;

    unsigned int i;
    unsigned int j;

    // generate filter
    // NOTE : these coefficients can be random; the purpose of this
    //        exercise is to demonstrate mathematical equivalence
    unsigned int h_len = p*num_channels;
    float h[h_len];
    for (i=0; i<h_len; i++) h[i] = randnf();

    // create filterbank object
    firpfbch_crcf q = firpfbch_crcf_create(LIQUID_ANALYZER, num_channels, p, h);

    // generate filter object
    firfilt_crcf f = firfilt_crcf_create(h, h_len);

    // allocate memory for arrays
    float complex y[num_samples];                   // time-domain input
    float complex Y0[num_symbols][num_channels];    // channelized output
    float complex Y1[num_symbols][num_channels];    // channelized output

    // generate input sequence (complex noise)
    for (i=0; i<num_samples; i++)
        y[i] = randnf() * cexpf(_Complex_I*randf()*2*M_PI);

    // 
    // run analysis filter bank
    //

    for (i=0; i<num_symbols; i++)
        firpfbch_crcf_analyzer_execute(q, &y[i*num_channels], &Y0[i][0]);


    // 
    // run traditional down-converter (inefficient)
    //

    float dphi; // carrier frequency
    unsigned int n=0;
    for (i=0; i<num_channels; i++) {

        // reset filter
        firfilt_crcf_clear(f);

        // set center frequency
        dphi = 2.0f * M_PI * (float)i / (float)num_channels;

        // reset symbol counter
        n=0;

        for (j=0; j<num_samples; j++) {
            // push down-converted sample into filter
            firfilt_crcf_push(f, y[j]*cexpf(-_Complex_I*j*dphi));

            // compute output at the appropriate sample time
            assert(n<num_symbols);
            if ( ((j+1)%num_channels)==0 ) {
                firfilt_crcf_execute(f, &Y1[n][i]);
                n++;
            }
        }
        assert(n==num_symbols);

    }

    // destroy objects
    firfilt_crcf_destroy(f);
    firpfbch_crcf_destroy(q);

    if (liquid_autotest_verbose) {
        // print filterbank channelizer
        printf("\n");
        printf("filterbank channelizer:\n");
        for (i=0; i<num_symbols; i++) {
            printf("%3u: ", i);
            for (j=0; j<num_channels; j++) {
                printf("  %8.5f+j%8.5f, ", crealf(Y0[i][j]), cimagf(Y0[i][j]));
            }
            printf("\n");
        }

        // print traditional channelizer
        printf("\n");
        printf("traditional channelizer:\n");
        for (i=0; i<num_symbols; i++) {
            printf("%3u: ", i);
            for (j=0; j<num_channels; j++) {
                printf("  %8.5f+j%8.5f, ", crealf(Y1[i][j]), cimagf(Y1[i][j]));
            }
            printf("\n");
        }
    }

    // compare results
    for (i=0; i<num_symbols; i++) {
        for (j=0; j<num_channels; j++) {
            CONTEND_DELTA( crealf(Y0[i][j]), crealf(Y1[i][j]), tol );
            CONTEND_DELTA( cimagf(Y0[i][j]), cimagf(Y1[i][j]), tol );
        }
    }

}


