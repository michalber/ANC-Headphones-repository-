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
// autotest fft r2r (real-to-real) data
//

#ifndef __LIQUID_AUTOTEST_FFTR2RDATA8_H__
#define __LIQUID_AUTOTEST_FFTR2RDATA8_H__

// 8-point real even/odd dft data
float fftdata_r2r_n8[] = {
      0.0000000000,
      1.0000000000,
      2.0000000000,
      3.0000000000,
      4.0000000000,
      5.0000000000,
      6.0000000000,
      7.0000000000};

// REDFT00
float fftdata_r2r_REDFT00_n8[] = {
     49.0000000000,
    -20.1956693581,
     -0.0000000000,
     -2.5724165284,
     -0.0000000000,
     -1.2319141135,
     -0.0000000000,
     -1.0000000000};

// REDFT10
float fftdata_r2r_REDFT10_n8[] = {
     56.0000000000,
    -25.7692920908,
      0.0000000000,
     -2.6938192036,
      0.0000000000,
     -0.8036116149,
      0.0000000000,
     -0.2028092910};

// REDFT01
float fftdata_r2r_REDFT01_n8[] = {
     29.1819286410,
    -32.3061136840,
     12.7168729872,
    -10.9904036256,
      5.7286734878,
     -4.9189401648,
      1.8807638636,
     -1.2927815051};

// REDFT11
float fftdata_r2r_REDFT11_n8[] = {
     24.7243981823,
    -31.5148535947,
     13.9257769120,
    -12.7826885392,
      9.1714938314,
     -8.8071984223,
      7.6789810021,
     -7.5857732734};

// RODFT00
float fftdata_r2r_RODFT00_n8[] = {
     39.6989727373,
    -24.7272967751,
     12.1243556530,
    -10.7257823333,
      5.8736974182,
     -5.1961524227,
      2.5477916399,
     -1.5869428264};

// RODFT10
float fftdata_r2r_RODFT10_n8[] = {
     35.8808162684,
    -20.9050074380,
     12.5996671239,
    -11.3137084990,
      8.4188284171,
     -8.6591376023,
      7.1371381075,
     -8.0000000000};

// RODFT01
float fftdata_r2r_RODFT01_n8[] = {
     41.8902640723,
     -9.2302062214,
      0.3792058953,
     -2.4608789465,
      0.0160780480,
     -1.1773622132,
      0.2426629216,
     -0.6033416816};

// RODFT11
float fftdata_r2r_RODFT11_n8[] = {
     46.6916824794,
     -7.4005942194,
      0.9237106918,
     -1.7485238108,
     -0.1159888643,
     -0.8699819349,
     -0.3640003929,
     -0.5519032668};

#endif // __LIQUID_AUTOTEST_FFTR2RDATA8_H__
