/*
 * Copyright (c) 2007, 2009 Joseph Gaeddert
 * Copyright (c) 2007, 2009 Virginia Polytechnic Institute & State University
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

#include "autotest/autotest.h"
#include "liquid.h"

//
// AUTOTEST: 
//
void autotest_interp_rrrf_generic()
{
    float h[9] = {
      -0.2762293319046737,
       1.4757679031218007,
       0.1432569489572376,
      -0.2142368750177835,
       1.3471241294836864,
       0.1166010284926269,
       0.0536534505390281,
       0.1412672462812405,
      -0.0991854372394269};

    unsigned int M = 4;     // interp factor
    interp_rrrf q = interp_rrrf_create(M,h,9);

    float x[] = {1.0, -1.0, 1.0, 1.0};
    float y[16];
    float test[16] = {
      -0.2762293319046737,
       1.4757679031218007,
       0.1432569489572376,
      -0.2142368750177835,
       1.6233534613883602,
      -1.3591668746291738,
      -0.0896034984182095,
       0.3555041212990241,
      -1.7225388986277870,
       1.3591668746291738,
       0.0896034984182095,
      -0.3555041212990241,
       1.1700802348184398,
       1.5923689316144276,
       0.1969103994962658,
      -0.0729696287365430};

    float tol = 1e-6;

    unsigned int i;
    for (i=0; i<4; i++)
        interp_rrrf_execute(q, x[i], &y[i*M]);

    for (i=0; i<16; i++) {
        CONTEND_DELTA(y[i], test[i], tol);

        if (liquid_autotest_verbose)
            printf("  y(%u) = %8.4f;\n", i+1, y[i]);
    }

    if (liquid_autotest_verbose)
        interp_rrrf_print(q);

    // destroy interpolator object
    interp_rrrf_destroy(q);
}

void autotest_interp_crcf_generic()
{
    // h = [0, 0.25, 0.5, 0.75, 1.0, 0.75, 0.5, 0.25, 0];
    float h[9] = {
      -0.7393353832652201,
       0.1909821993029451,
      -1.7013834621383086,
      -0.6157406339062349,
       0.5806218191269317,
       0.0576963976148674,
      -1.0958217797368455,
      -0.6379821629743743,
       0.7019489165905530};

    unsigned int M = 4;     // interp factor
    interp_crcf q = interp_crcf_create(M,h,9);

    //  x = [1+j*0.2, -0.2+j*1.3, 0.5+j*0.3, 1.1-j*0.2]
    float complex x[4] = {
      1.0000e+00+  2.0000e-01*_Complex_I, 
     -2.0000e-01+  1.3000e+00*_Complex_I, 
      5.0000e-01+  3.0000e-01*_Complex_I, 
      1.1000e+00+ -2.0000e-01*_Complex_I
    };
        
    float complex y[16];

    // z = [x(1) 0 0 0 x(2) 0 0 0 x(3) 0 0 0 x(4) 0 0 0];
    // test = filter(h,1,z)
    float complex test[16] = {
      -0.7393353832652201 - 0.1478670766530440*_Complex_I,
       0.1909821993029451 + 0.0381964398605890*_Complex_I,
      -1.7013834621383086 - 0.3402766924276617*_Complex_I,
      -0.6157406339062349 - 0.1231481267812470*_Complex_I,
       0.7284888957799757 - 0.8450116344193997*_Complex_I,
       0.0194999577542784 + 0.2598161386168021*_Complex_I,
      -0.7555450873091838 - 2.4309628567271702*_Complex_I,
      -0.5148340361931273 - 0.9280592566729803*_Complex_I,
       0.2161568611325566 + 0.6733975332035558*_Complex_I,
       0.0839518201284991 + 0.1322999766902112*_Complex_I,
      -0.6315273751217851 - 1.9349833522993918*_Complex_I,
      -0.1802738843582426 - 1.0140990020385570*_Complex_I,
      -0.6633477953463869 + 1.2345872139588425*_Complex_I,
       0.2389286180406733 - 0.0208875205761288*_Complex_I,
      -2.4194326982205623 + 0.0115301585066081*_Complex_I,
      -0.9963057787840456 - 0.0682465221110653*_Complex_I };

    float tol = 1e-6;

    unsigned int i;
    for (i=0; i<4; i++)
        interp_crcf_execute(q, x[i], &y[i*M]);

    for (i=0; i<16; i++) {
        CONTEND_DELTA( crealf(y[i]), crealf(test[i]), tol);
        CONTEND_DELTA( cimagf(y[i]), cimagf(test[i]), tol);

        if (liquid_autotest_verbose)
            printf("  y(%u) = %8.4f + j%8.4f;\n", i+1, crealf(y[i]), cimagf(y[i]));
    }

    if (liquid_autotest_verbose)
        interp_crcf_print(q);

    // destroy interpolator object
    interp_crcf_destroy(q);
}

