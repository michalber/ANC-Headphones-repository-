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

// Help function to keep code base small
void modem_test_mod_demod(modulation_scheme _ms)
{
    // generate mod/demod
    modem mod   = modem_create(_ms);
    modem demod = modem_create(_ms);

    // run the test
    unsigned int i, s, M = 1 << modem_get_bps(mod);
    float complex x;
    float e = 0.0f;
    for (i=0; i<M; i++) {
        modem_modulate(mod, i, &x);
        modem_demodulate(demod, x, &s);
        CONTEND_EQUALITY(s, i);

        CONTEND_DELTA( modem_get_demodulator_phase_error(demod), 0.0f, 1e-3f);
        
        CONTEND_DELTA( modem_get_demodulator_evm(demod), 0.0f, 1e-3f);

        e += crealf(x*conjf(x));
    }
    e = sqrtf(e / (float)M);

    CONTEND_DELTA(e,1.0f,1e-3f);

    // clean it up
    modem_destroy(mod);
    modem_destroy(demod);
}

//
// AUTOTESTS: Specific modems
//
void autotest_mod_demod_bpsk()  {   modem_test_mod_demod(LIQUID_MODEM_BPSK);    }
void autotest_mod_demod_qpsk()  {   modem_test_mod_demod(LIQUID_MODEM_QPSK);    }
void autotest_mod_demod_ook()   {   modem_test_mod_demod(LIQUID_MODEM_OOK);     }
void autotest_mod_demod_sqam32(){   modem_test_mod_demod(LIQUID_MODEM_SQAM32);  }
void autotest_mod_demod_sqam128(){  modem_test_mod_demod(LIQUID_MODEM_SQAM128); }

//
// AUTOTESTS: generic ASK
//
void autotest_mod_demod_ask2()  {   modem_test_mod_demod(LIQUID_MODEM_ASK2);   }
void autotest_mod_demod_ask4()  {   modem_test_mod_demod(LIQUID_MODEM_ASK4);   }
void autotest_mod_demod_ask8()  {   modem_test_mod_demod(LIQUID_MODEM_ASK8);   }
void autotest_mod_demod_ask16() {   modem_test_mod_demod(LIQUID_MODEM_ASK16);  }

//
// AUTOTESTS: generic PSK
//
void autotest_mod_demod_psk2()  {   modem_test_mod_demod(LIQUID_MODEM_PSK2);   }
void autotest_mod_demod_psk4()  {   modem_test_mod_demod(LIQUID_MODEM_PSK4);   }
void autotest_mod_demod_psk8()  {   modem_test_mod_demod(LIQUID_MODEM_PSK8);   }
void autotest_mod_demod_psk16() {   modem_test_mod_demod(LIQUID_MODEM_PSK16);  }
void autotest_mod_demod_psk32() {   modem_test_mod_demod(LIQUID_MODEM_PSK32);  }
void autotest_mod_demod_psk64() {   modem_test_mod_demod(LIQUID_MODEM_PSK64);  }

//
// AUTOTESTS: generic differential PSK
//
void autotest_mod_demod_dpsk2()  {  modem_test_mod_demod(LIQUID_MODEM_DPSK2);  }
void autotest_mod_demod_dpsk4()  {  modem_test_mod_demod(LIQUID_MODEM_DPSK4);  }
void autotest_mod_demod_dpsk8()  {  modem_test_mod_demod(LIQUID_MODEM_DPSK8);  }
void autotest_mod_demod_dpsk16() {  modem_test_mod_demod(LIQUID_MODEM_DPSK16); }
void autotest_mod_demod_dpsk32() {  modem_test_mod_demod(LIQUID_MODEM_DPSK32); }
void autotest_mod_demod_dpsk64() {  modem_test_mod_demod(LIQUID_MODEM_DPSK64); }

//
// AUTOTESTS: generic QAM
//
void autotest_mod_demod_qam4()   {  modem_test_mod_demod(LIQUID_MODEM_QAM4);   }
void autotest_mod_demod_qam8()   {  modem_test_mod_demod(LIQUID_MODEM_QAM8);   }
void autotest_mod_demod_qam16()  {  modem_test_mod_demod(LIQUID_MODEM_QAM16);  }
void autotest_mod_demod_qam32()  {  modem_test_mod_demod(LIQUID_MODEM_QAM32);  }
void autotest_mod_demod_qam64()  {  modem_test_mod_demod(LIQUID_MODEM_QAM64);  }
void autotest_mod_demod_qam128() {  modem_test_mod_demod(LIQUID_MODEM_QAM128); }
void autotest_mod_demod_qam256() {  modem_test_mod_demod(LIQUID_MODEM_QAM256); }

//
// AUTOTESTS: generic APSK (maps to specific APSK modems internally)
//
void autotest_mod_demod_apsk4()  {  modem_test_mod_demod(LIQUID_MODEM_APSK4);   }
void autotest_mod_demod_apsk8()  {  modem_test_mod_demod(LIQUID_MODEM_APSK8);   }
void autotest_mod_demod_apsk16() {  modem_test_mod_demod(LIQUID_MODEM_APSK16);  }
void autotest_mod_demod_apsk32() {  modem_test_mod_demod(LIQUID_MODEM_APSK32);  }
void autotest_mod_demod_apsk64() {  modem_test_mod_demod(LIQUID_MODEM_APSK64);  }
void autotest_mod_demod_apsk128(){  modem_test_mod_demod(LIQUID_MODEM_APSK128); }
void autotest_mod_demod_apsk256(){  modem_test_mod_demod(LIQUID_MODEM_APSK256); }

//
// AUTOTESTS: arbitrary modems
//
void autotest_mod_demod_arbV29()    { modem_test_mod_demod(LIQUID_MODEM_V29);       }
void autotest_mod_demod_arb16opt()  { modem_test_mod_demod(LIQUID_MODEM_ARB16OPT);  }
void autotest_mod_demod_arb32opt()  { modem_test_mod_demod(LIQUID_MODEM_ARB32OPT);  }
void autotest_mod_demod_arb64opt()  { modem_test_mod_demod(LIQUID_MODEM_ARB64OPT);  }
void autotest_mod_demod_arb128opt() { modem_test_mod_demod(LIQUID_MODEM_ARB128OPT); }
void autotest_mod_demod_arb256opt() { modem_test_mod_demod(LIQUID_MODEM_ARB256OPT); }
void autotest_mod_demod_arb64vt()   { modem_test_mod_demod(LIQUID_MODEM_ARB64VT);   }

