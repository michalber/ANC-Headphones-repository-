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
// math_cexpf_test.c
//
// complex exponent test
//

#include <stdio.h>
#include <stdlib.h>
#include <complex.h>
#include <math.h>

#define sandbox_randf() ((float) rand() / (float) RAND_MAX)

float complex sandbox_cexpf(float complex _z)
{
    float r = expf( crealf(_z) );
    float complex re = cosf( cimagf(_z) );
    float complex im = sinf( cimagf(_z) );

    return r*(re + _Complex_I*im);
}

int main() {
    unsigned int n=32;  // number of tests
    unsigned int d=2;   // number items per line

    // data arrays
    float complex z[n];
    float complex test[n];

    unsigned int i;
    float complex err_max = 0.0f;
    for (i=0; i<n; i++) {
        // generate random complex number
        z[i] = 4.0f*(2.0f*sandbox_randf() - 1.0f) +
               4.0f*(2.0f*sandbox_randf() - 1.0f) * _Complex_I;

        test[i] = cexpf(z[i]);
        float complex expz_hat = sandbox_cexpf(z[i]);

        float complex err = test[i] - expz_hat;

        printf("%3u: z=%6.2f+j%6.2f, exp(z)=%6.2f+j%6.2f (%6.2f+j%6.2f) e=%12.4e\n",
                i,
                crealf(z[i]),       cimagf(z[i]),
                crealf(test[i]),    cimagf(test[i]),
                crealf(expz_hat),   cimagf(expz_hat),
                cabsf(err));

        if ( cabsf(err) > cabsf(err_max) )
            err_max = err;
    }

    printf("maximum error: %12.4e;\n", cabsf(err_max));

    // 
    // print autotest lines
    //

    printf("\n");
    printf("    float complex z[%u] = {\n      ", n);
    for (i=0; i<n; i++) {
        printf("%14.6e+_Complex_I*%14.6e", crealf(z[i]), cimagf(z[i]));

        if ( i == n-1)
            printf(" ");
        else if ( ((i+1)%d)==0 )
            printf(",\n      ");
        else
            printf(", ");
    }
    printf("};\n");

    printf("\n");
    printf("    float complex test[%u] = {\n      ", n);
    for (i=0; i<n; i++) {
        printf("%14.6e+_Complex_I*%14.6e", crealf(test[i]), cimagf(test[i]));

        if ( i == n-1)
            printf(" ");
        else if ( ((i+1)%d)==0 )
            printf(",\n      ");
        else
            printf(", ");
    }
    printf("};\n");

    printf("done.\n");
    return 0;
}

