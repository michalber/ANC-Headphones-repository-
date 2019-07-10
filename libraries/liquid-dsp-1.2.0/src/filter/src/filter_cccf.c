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
// Filter API: complex floating-point
//

#include "liquid.internal.h"

// naming extensions (useful for print statements)
#define EXTENSION_SHORT     "f"
#define EXTENSION_FULL      "cccf"

// 
#define AUTOCORR(name)      LIQUID_CONCAT(autocorr_cccf,name)
#define FIRFILT(name)       LIQUID_CONCAT(firfilt_cccf,name)
#define IIRFILT(name)       LIQUID_CONCAT(iirfilt_cccf,name)
#define IIRFILTSOS(name)    LIQUID_CONCAT(iirfiltsos_cccf,name)
#define FIRPFB(name)        LIQUID_CONCAT(firpfb_cccf,name)
#define INTERP(name)        LIQUID_CONCAT(interp_cccf,name)
#define DECIM(name)         LIQUID_CONCAT(decim_cccf,name)
#define NCO(name)           LIQUID_CONCAT(nco_crcf,name)
#define MSRESAMP(name)      LIQUID_CONCAT(msresamp_cccf,name)
#define RESAMP(name)        LIQUID_CONCAT(resamp_cccf,name)
#define RESAMP2(name)       LIQUID_CONCAT(resamp2_cccf,name)
//#define SYMSYNC(name)       LIQUID_CONCAT(symsync_cccf,name)

#define T                   float complex   // general
#define TO                  float complex   // output
#define TC                  float complex   // coefficients
#define TI                  float complex   // input
#define WINDOW(name)        LIQUID_CONCAT(windowcf,name)
#define DOTPROD(name)       LIQUID_CONCAT(dotprod_cccf,name)
#define POLY(name)          LIQUID_CONCAT(polyf,name)

#define TO_COMPLEX          1
#define TC_COMPLEX          1
#define TI_COMPLEX          1

#define PRINTVAL_TO(X,F)    PRINTVAL_CFLOAT(X,F)
#define PRINTVAL_TC(X,F)    PRINTVAL_CFLOAT(X,F)
#define PRINTVAL_TI(X,F)    PRINTVAL_CFLOAT(X,F)

// source files
#include "autocorr.c"
#include "firfilt.c"
#include "firpfb.c"
#include "iirfilt.c"
#include "iirfiltsos.c"
#include "interp.c"
#include "decim.c"
//#include "qmfb.c"
#include "msresamp.c"
#include "resamp.c"
#include "resamp2.c"
//#include "symsync.c"
