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
// modem_common.c
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "liquid.internal.h"

// full modulation type descriptor
const struct modulation_type_s modulation_types[LIQUID_MODEM_NUM_SCHEMES] = {
    // name       fullname        scheme          bps

    // unknown
    {"unknown",   "unkown",       LIQUID_MODEM_UNKNOWN, 0},

    // phase-shift keying
    {"psk2",      "phase-shift keying (2)",   LIQUID_MODEM_PSK2,  1},
    {"psk4",      "phase-shift keying (4)",   LIQUID_MODEM_PSK4,  2},
    {"psk8",      "phase-shift keying (8)",   LIQUID_MODEM_PSK8,  3},
    {"psk16",     "phase-shift keying (16)",  LIQUID_MODEM_PSK16, 4},
    {"psk32",     "phase-shift keying (32)",  LIQUID_MODEM_PSK32, 5},
    {"psk64",     "phase-shift keying (64)",  LIQUID_MODEM_PSK64, 6},
    {"psk128",    "phase-shift keying (128)", LIQUID_MODEM_PSK128, 7},
    {"psk256",    "phase-shift keying (256)", LIQUID_MODEM_PSK256, 8},

    // differential phase-shift keying
    {"dpsk2",     "differential phase-shift keying (2)",   LIQUID_MODEM_DPSK2,  1},
    {"dpsk4",     "differential phase-shift keying (4)",   LIQUID_MODEM_DPSK4,  2},
    {"dpsk8",     "differential phase-shift keying (8)",   LIQUID_MODEM_DPSK8,  3},
    {"dpsk16",    "differential phase-shift keying (16)",  LIQUID_MODEM_DPSK16, 4},
    {"dpsk32",    "differential phase-shift keying (32)",  LIQUID_MODEM_DPSK32, 5},
    {"dpsk64",    "differential phase-shift keying (64)",  LIQUID_MODEM_DPSK64, 6},
    {"dpsk128",   "differential phase-shift keying (128)", LIQUID_MODEM_DPSK128, 7},
    {"dpsk256",   "differential phase-shift keying (256)", LIQUID_MODEM_DPSK256, 8},

    // amplitude-shift keying
    {"ask2",      "amplitude-shift keying (2)",   LIQUID_MODEM_ASK2,  1},
    {"ask4",      "amplitude-shift keying (4)",   LIQUID_MODEM_ASK4,  2},
    {"ask8",      "amplitude-shift keying (8)",   LIQUID_MODEM_ASK8,  3},
    {"ask16",     "amplitude-shift keying (16)",  LIQUID_MODEM_ASK16, 4},
    {"ask32",     "amplitude-shift keying (32)",  LIQUID_MODEM_ASK32, 5},
    {"ask64",     "amplitude-shift keying (64)",  LIQUID_MODEM_ASK64, 6},
    {"ask128",    "amplitude-shift keying (128)", LIQUID_MODEM_ASK128, 7},
    {"ask256",    "amplitude-shift keying (256)", LIQUID_MODEM_ASK256, 8},

    // quadrature amplitude-shift keying
    {"qam4",      "quadrature amplitude-shift keying (4)",   LIQUID_MODEM_QAM4,   2},
    {"qam8",      "quadrature amplitude-shift keying (8)",   LIQUID_MODEM_QAM8,   3},
    {"qam16",     "quadrature amplitude-shift keying (16)",  LIQUID_MODEM_QAM16,  4},
    {"qam32",     "quadrature amplitude-shift keying (32)",  LIQUID_MODEM_QAM32,  5},
    {"qam64",     "quadrature amplitude-shift keying (64)",  LIQUID_MODEM_QAM64,  6},
    {"qam128",    "quadrature amplitude-shift keying (128)", LIQUID_MODEM_QAM128, 7},
    {"qam256",    "quadrature amplitude-shift keying (256)", LIQUID_MODEM_QAM256, 8},

    // amplitude/phase-shift keying
    {"apsk4",     "amplitude/phase-shift keying (4)",   LIQUID_MODEM_APSK4,   2},
    {"apsk8",     "amplitude/phase-shift keying (8)",   LIQUID_MODEM_APSK8,   3},
    {"apsk16",    "amplitude/phase-shift keying (16)",  LIQUID_MODEM_APSK16,  4},
    {"apsk32",    "amplitude/phase-shift keying (32)",  LIQUID_MODEM_APSK32,  5},
    {"apsk64",    "amplitude/phase-shift keying (64)",  LIQUID_MODEM_APSK64,  6},
    {"apsk128",   "amplitude/phase-shift keying (128)", LIQUID_MODEM_APSK128, 7},
    {"apsk256",   "amplitude/phase-shift keying (256)", LIQUID_MODEM_APSK256, 8},

    // specific modem types
    {"bpsk",      "binary phase-shift keying",     LIQUID_MODEM_BPSK,      1},
    {"qpsk",      "quaternary phase-shift keying", LIQUID_MODEM_QPSK,      2},
    {"ook",       "ook (on/off keying)",           LIQUID_MODEM_OOK,       1},
    {"sqam32",    "'square' 32-QAM",               LIQUID_MODEM_SQAM32,    5},
    {"sqam128",   "'square' 128-QAM",              LIQUID_MODEM_SQAM128,   7},
    {"V29",       "V.29",                          LIQUID_MODEM_V29,       4},
    {"arb16opt",  "arb16opt (optimal 16-qam)",     LIQUID_MODEM_ARB16OPT,  4},
    {"arb32opt",  "arb32opt (optimal 32-qam)",     LIQUID_MODEM_ARB32OPT,  5},
    {"arb64opt",  "arb64opt (optimal 64-qam)",     LIQUID_MODEM_ARB64OPT,  6},
    {"arb128opt", "arb128opt (optimal 128-qam)",   LIQUID_MODEM_ARB128OPT, 7},
    {"arb256opt", "arb256opt (optimal 256-qam)",   LIQUID_MODEM_ARB256OPT, 8},
    {"arb64vt",   "arb64vt (64-qam vt logo)",      LIQUID_MODEM_ARB64VT,   6},

    // arbitrary modem type
    {"arb",       "arbitrary constellation",       LIQUID_MODEM_ARB,       0},
};



modulation_scheme liquid_getopt_str2mod(const char * _str)
{
    // compare each string to short name
    unsigned int i;
    for (i=0; i<LIQUID_MODEM_NUM_SCHEMES; i++) {
        if (strcmp(_str,modulation_types[i].name)==0)
            return i;
    }
    fprintf(stderr,"warning: liquid_getopt_str2mod(), unknown/unsupported mod scheme : %s\n", _str);
    return LIQUID_MODEM_UNKNOWN;
}

// Print compact list of existing and available modulation schemes
void liquid_print_modulation_schemes()
{
    unsigned int i;
    unsigned int len = 10;

    // print all available modem schemes
    printf("          ");
    for (i=1; i<LIQUID_MODEM_NUM_SCHEMES; i++) {
        printf("%s", modulation_types[i].name);

        if (i != LIQUID_MODEM_NUM_SCHEMES-1)
            printf(", ");

        len += strlen(modulation_types[i].name);
        if (len > 48 && i != LIQUID_MODEM_NUM_SCHEMES-1) {
            len = 10;
            printf("\n          ");
        }
    }
    printf("\n");
}

// query basic modulation types
int liquid_modem_is_psk(modulation_scheme _ms)
{
    switch (_ms) {
    // Phase-shift keying (PSK)
    case LIQUID_MODEM_PSK2:
    case LIQUID_MODEM_PSK4:
    case LIQUID_MODEM_PSK8:
    case LIQUID_MODEM_PSK16:
    case LIQUID_MODEM_PSK32:
    case LIQUID_MODEM_PSK64:
    case LIQUID_MODEM_PSK128:
    case LIQUID_MODEM_PSK256:
        return 1;
    default:
        return 0;
    }

    return 0;
}

int liquid_modem_is_dpsk(modulation_scheme _ms)
{
    switch (_ms) {
    // Differential phase-shift keying (DPSK)
    case LIQUID_MODEM_DPSK2:
    case LIQUID_MODEM_DPSK4:
    case LIQUID_MODEM_DPSK8:
    case LIQUID_MODEM_DPSK16:
    case LIQUID_MODEM_DPSK32:
    case LIQUID_MODEM_DPSK64:
    case LIQUID_MODEM_DPSK128:
    case LIQUID_MODEM_DPSK256:
        return 1;
    default:
        return 0;
    }

    return 0;
}

int liquid_modem_is_ask(modulation_scheme _ms)
{
    switch (_ms) {
    // amplitude-shift keying (ASK)
    case LIQUID_MODEM_ASK2:
    case LIQUID_MODEM_ASK4:
    case LIQUID_MODEM_ASK8:
    case LIQUID_MODEM_ASK16:
    case LIQUID_MODEM_ASK32:
    case LIQUID_MODEM_ASK64:
    case LIQUID_MODEM_ASK128:
    case LIQUID_MODEM_ASK256:
        return 1;
    default:
        return 0;
    }

    return 0;
}

int liquid_modem_is_qam(modulation_scheme _ms)
{
    switch (_ms) {
    // rectangular quadrature amplitude-shift keying (QAM)
    case LIQUID_MODEM_QAM4:
    case LIQUID_MODEM_QAM8:
    case LIQUID_MODEM_QAM16:
    case LIQUID_MODEM_QAM32:
    case LIQUID_MODEM_QAM64:
    case LIQUID_MODEM_QAM128:
    case LIQUID_MODEM_QAM256:
        return 1;
    default:
        return 0;
    }

    return 0;
}

int liquid_modem_is_apsk(modulation_scheme _ms)
{
    switch (_ms) {
    // amplitude phase-shift keying (APSK)
    case LIQUID_MODEM_APSK4:
    case LIQUID_MODEM_APSK8:
    case LIQUID_MODEM_APSK16:
    case LIQUID_MODEM_APSK32:
    case LIQUID_MODEM_APSK64:
    case LIQUID_MODEM_APSK128:
    case LIQUID_MODEM_APSK256:
        return 1;
    default:
        return 0;
    }

    return 0;
}


// Generate random symbol
unsigned int modem_gen_rand_sym(modem _mod)
{
    return rand() % (_mod->M);
}

// Get modem depth (bits/symbol)
unsigned int modem_get_bps(modem _mod)
{
    return _mod->m;
}

// gray encoding
unsigned int gray_encode(unsigned int symbol_in)
{
    return symbol_in ^ (symbol_in >> 1);
}

// gray decoding
unsigned int gray_decode(unsigned int symbol_in)
{
    unsigned int mask = symbol_in;
    unsigned int symbol_out = symbol_in;
    unsigned int i;

    // Run loop in blocks of 4 to reduce number of comparisons. Running
    // loop more times than MAX_MOD_BITS_PER_SYMBOL will not result in
    // decoding errors.
    for (i=0; i<MAX_MOD_BITS_PER_SYMBOL; i+=4) {
        symbol_out ^= (mask >> 1);
        symbol_out ^= (mask >> 2);
        symbol_out ^= (mask >> 3);
        symbol_out ^= (mask >> 4);
        mask >>= 4;
    }

    return symbol_out;
}

// pack soft bits into symbol
//  _soft_bits  :   soft input bits [size: _bps x 1]
//  _bps        :   bits per symbol
//  _sym_out    :   output symbol, value in [0,2^_bps)
void liquid_pack_soft_bits(unsigned char * _soft_bits,
                           unsigned int _bps,
                           unsigned int * _sym_out)
{
    // validate input
    if (_bps > MAX_MOD_BITS_PER_SYMBOL) {
        fprintf(stderr,"error: liquid_unpack_soft_bits(), bits/symbol exceeds maximum (%u)\n", MAX_MOD_BITS_PER_SYMBOL);
        exit(1);
    }

    unsigned int i;
    unsigned int s=0;
    for (i=0; i<_bps; i++) {
        s <<= 1;
        s |= _soft_bits[i] > LIQUID_SOFTBIT_ERASURE ? 1 : 0;
    }
    *_sym_out = s;
}

// unpack soft bits into symbol
//  _sym_in     :   input symbol, value in [0,2^_bps)
//  _bps        :   bits per symbol
//  _soft_bits  :   soft output bits [size: _bps x 1]
void liquid_unpack_soft_bits(unsigned int _sym_in,
                             unsigned int _bps,
                             unsigned char * _soft_bits)
{
    // validate input
    if (_bps > MAX_MOD_BITS_PER_SYMBOL) {
        fprintf(stderr,"error: liquid_unpack_soft_bits(), bits/symbol exceeds maximum (%u)\n", MAX_MOD_BITS_PER_SYMBOL);
        exit(1);
    }

    unsigned int i;
    for (i=0; i<_bps; i++)
        _soft_bits[i] = ((_sym_in >> (_bps-i-1)) & 0x0001) ? LIQUID_SOFTBIT_1 : LIQUID_SOFTBIT_0;
}


