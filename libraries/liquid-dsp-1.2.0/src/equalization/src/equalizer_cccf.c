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
// Floating-point equalizers
//

#include "liquid.internal.h"

// naming extensions (useful for print statements)
#define EXTENSION_SHORT "f"
#define EXTENSION_FULL  "cccf"

#define EQLMS(name)     LIQUID_CONCAT(eqlms_cccf,name)
#define EQRLS(name)     LIQUID_CONCAT(eqrls_cccf,name)

#define DOTPROD(name)   LIQUID_CONCAT(dotprod_cccf,name)
#define WINDOW(name)    LIQUID_CONCAT(windowcf,name)
#define MATRIX(name)    LIQUID_CONCAT(matrixcf,name)

#define T               float complex

#define PRINTVAL(V)     printf("%5.2f+j%5.2f ", crealf(V), cimagf(V));

#include "eqlms.c"
#include "eqrls.c"
