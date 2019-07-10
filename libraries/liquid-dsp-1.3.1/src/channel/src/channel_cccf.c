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

// 
// Complex floating-point channel
//

#include <complex.h>
#include "liquid.internal.h"

// naming extensions (useful for print statements)
#define EXTENSION_SHORT "f"
#define EXTENSION_FULL  "cccf"

#define CHANNEL(name)   LIQUID_CONCAT(channel_cccf,name)
#define DOTPROD(name)   LIQUID_CONCAT(dotprod_cccf,name)
#define FIRFILT(name)   LIQUID_CONCAT(firfilt_cccf,name)
#define IIRFILT(name)   LIQUID_CONCAT(iirfilt_rrrf,name)
#define NCO(name)       LIQUID_CONCAT(nco_crcf,name)
#define RESAMP(name)    LIQUID_CONCAT(resamp_crcf,name)
#define TVMPCH(name)    LIQUID_CONCAT(tvmpch_cccf,name)
#define WINDOW(name)    LIQUID_CONCAT(windowcf,name)

#define TO              float complex   // output type
#define TC              float complex   // coefficients type
#define TI              float complex   // input type
#define T               float           // primitive type

#define PRINTVAL_TC(X,F) PRINTVAL_CFLOAT(X,F)

#include "channel.c"
#include "tvmpch.c"

