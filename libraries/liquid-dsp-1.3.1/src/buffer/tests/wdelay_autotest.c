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

//
// AUTOTEST: wdelayf
//
void autotest_wdelayf()
{
    float v;    // reader
    unsigned int i;

    // create wdelay
    // wdelay: 0 0 0 0
    wdelayf w = wdelayf_create(4);

    wdelayf_read(w, &v);
    CONTEND_EQUALITY(v, 0);

    float x0[10]      = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    float y0_test[10] = {0, 0, 0, 0, 1, 2, 3, 4, 5, 6};
    float y0[10];

    for (i=0; i<10; i++) {
        wdelayf_read(w, &y0[i]);
        wdelayf_push(w,  x0[i]);
        //printf("%3u : %6.2f (%6.2f)\n", i, y0[i], y0_test[i]);
    }
    // 7 8 9 10
    CONTEND_SAME_DATA(y0, y0_test, 10*sizeof(float));

    // re-create wdelay object
    // wdelay: 0 0 7 8 9 10
    w = wdelayf_recreate(w,6);

    float x1[10]     = {3, 4, 5, 6, 7, 8, 9, 2, 2, 2};
    float y1_test[10]= {0, 0, 7, 8, 9, 10,3, 4, 5, 6};
    float y1[10];
    for (i=0; i<10; i++) {
        wdelayf_read(w, &y1[i]);
        wdelayf_push(w,  x1[i]);
        //printf("%3u : %6.2f (%6.2f)\n", i, y1[i], y1_test[i]);
    }
    // wdelay: 7 8 9 2 2 2
    CONTEND_SAME_DATA(y1, y1_test, 10*sizeof(float));

    // re-create wdelay object
    // wdelay: 8 9 2 2 2
    w = wdelayf_recreate(w,5);

    float x2[10]     = {1, 1, 1, 1, 1, 1, 1, 2, 3, 4};
    float y2_test[10]= {8, 9, 2, 2, 2, 1, 1, 1, 1, 1};
    float y2[10];

    for (i=0; i<10; i++) {
        wdelayf_read(w, &y2[i]);
        wdelayf_push(w,  x2[i]);
        //printf("%3u : %6.2f (%6.2f)\n", i, y1[i], y1_test[i]);
    }
    // wdelay: 1 1 2 3 4
    CONTEND_SAME_DATA(y2, y2_test, 10*sizeof(float));

    // destroy object
    wdelayf_destroy(w);
}

