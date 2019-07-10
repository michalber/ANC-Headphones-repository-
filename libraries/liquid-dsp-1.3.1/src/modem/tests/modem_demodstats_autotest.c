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

#include <stdio.h>

#include "autotest/autotest.h"
#include "liquid.h"

// Helper function to keep code base small
void modem_test_demodstats(modulation_scheme _ms)
{
    // generate mod/demod
    modem mod   = modem_create(_ms);
    modem demod = modem_create(_ms);

    // run the test
    unsigned int i, s, M = 1 << modem_get_bps(mod);
    float complex x;
    float complex x_hat;    // rotated symbol
    float demodstats;
    float phi = 0.01f;

    for (i=0; i<M; i++) {
        // reset modem objects
        modem_reset(mod);
        modem_reset(demod);

        // modulate symbol
        modem_modulate(mod, i, &x);

        // ignore rare condition where modulated symbol is (0,0)
        // (e.g. APSK-8)
        if (cabsf(x) < 1e-3f) continue;

        // add phase offsets
        x_hat = x * cexpf( phi*_Complex_I);

        // demod positive phase signal, and ensure demodulator
        // maps to appropriate symbol
        modem_demodulate(demod, x_hat, &s);
        if (s != i)
            AUTOTEST_WARN("modem_test_demodstats(), output symbol does not match");

        demodstats = modem_get_demodulator_phase_error(demod);
        CONTEND_EXPRESSION(demodstats > 0.0f);
    }

    // repeat with negative phase error
    for (i=0; i<M; i++) {
        // reset modem objects
        modem_reset(mod);
        modem_reset(demod);

        // modulate symbol
        modem_modulate(mod, i, &x);

        // ignore rare condition where modulated symbol is (0,0)
        // (e.g. APSK-8)
        if (cabsf(x) < 1e-3f) continue;

        // add phase offsets
        x_hat = x * cexpf(-phi*_Complex_I);

        // demod positive phase signal, and ensure demodulator
        // maps to appropriate symbol
        modem_demodulate(demod, x_hat, &s);
        if (s != i)
            AUTOTEST_WARN("modem_test_demodstats(), output symbol does not match");

        demodstats = modem_get_demodulator_phase_error(demod);
        CONTEND_EXPRESSION(demodstats < 0.0f);
    }

    // clean up allocated objects up
    modem_destroy(mod);
    modem_destroy(demod);
}

// AUTOTESTS: generic PSK
void autotest_demodstats_psk2()     { modem_test_demodstats(LIQUID_MODEM_PSK2);     }
void autotest_demodstats_psk4()     { modem_test_demodstats(LIQUID_MODEM_PSK4);     }
void autotest_demodstats_psk8()     { modem_test_demodstats(LIQUID_MODEM_PSK8);     }
void autotest_demodstats_psk16()    { modem_test_demodstats(LIQUID_MODEM_PSK16);    }
void autotest_demodstats_psk32()    { modem_test_demodstats(LIQUID_MODEM_PSK32);    }
void autotest_demodstats_psk64()    { modem_test_demodstats(LIQUID_MODEM_PSK64);    }
void autotest_demodstats_psk128()   { modem_test_demodstats(LIQUID_MODEM_PSK128);   }
void autotest_demodstats_psk256()   { modem_test_demodstats(LIQUID_MODEM_PSK256);   }

// AUTOTESTS: generic DPSK
void autotest_demodstats_dpsk2()    { modem_test_demodstats(LIQUID_MODEM_DPSK2);    }
void autotest_demodstats_dpsk4()    { modem_test_demodstats(LIQUID_MODEM_DPSK4);    }
void autotest_demodstats_dpsk8()    { modem_test_demodstats(LIQUID_MODEM_DPSK8);    }
void autotest_demodstats_dpsk16()   { modem_test_demodstats(LIQUID_MODEM_DPSK16);   }
void autotest_demodstats_dpsk32()   { modem_test_demodstats(LIQUID_MODEM_DPSK32);   }
void autotest_demodstats_dpsk64()   { modem_test_demodstats(LIQUID_MODEM_DPSK64);   }
void autotest_demodstats_dpsk128()  { modem_test_demodstats(LIQUID_MODEM_DPSK128);  }
void autotest_demodstats_dpsk256()  { modem_test_demodstats(LIQUID_MODEM_DPSK256);  }

// AUTOTESTS: generic ASK
void autotest_demodstats_ask2()     { modem_test_demodstats(LIQUID_MODEM_ASK2);     }
void autotest_demodstats_ask4()     { modem_test_demodstats(LIQUID_MODEM_ASK4);     }
void autotest_demodstats_ask8()     { modem_test_demodstats(LIQUID_MODEM_ASK8);     }
void autotest_demodstats_ask16()    { modem_test_demodstats(LIQUID_MODEM_ASK16);    }
void autotest_demodstats_ask32()    { modem_test_demodstats(LIQUID_MODEM_ASK32);    }
void autotest_demodstats_ask64()    { modem_test_demodstats(LIQUID_MODEM_ASK64);    }
void autotest_demodstats_ask128()   { modem_test_demodstats(LIQUID_MODEM_ASK128);   }
void autotest_demodstats_ask256()   { modem_test_demodstats(LIQUID_MODEM_ASK256);   }

// AUTOTESTS: generic QAM
void autotest_demodstats_qam4()     { modem_test_demodstats(LIQUID_MODEM_QAM4);     }
void autotest_demodstats_qam8()     { modem_test_demodstats(LIQUID_MODEM_QAM8);     }
void autotest_demodstats_qam16()    { modem_test_demodstats(LIQUID_MODEM_QAM16);    }
void autotest_demodstats_qam32()    { modem_test_demodstats(LIQUID_MODEM_QAM32);    }
void autotest_demodstats_qam64()    { modem_test_demodstats(LIQUID_MODEM_QAM64);    }
void autotest_demodstats_qam128()   { modem_test_demodstats(LIQUID_MODEM_QAM128);   }
void autotest_demodstats_qam256()   { modem_test_demodstats(LIQUID_MODEM_QAM256);   }

// AUTOTESTS: generic APSK (maps to specific APSK modems internally)
void autotest_demodstats_apsk4()    { modem_test_demodstats(LIQUID_MODEM_APSK4);    }
void autotest_demodstats_apsk8()    { modem_test_demodstats(LIQUID_MODEM_APSK8);    }
void autotest_demodstats_apsk16()   { modem_test_demodstats(LIQUID_MODEM_APSK16);   }
void autotest_demodstats_apsk32()   { modem_test_demodstats(LIQUID_MODEM_APSK32);   }
void autotest_demodstats_apsk64()   { modem_test_demodstats(LIQUID_MODEM_APSK64);   }
void autotest_demodstats_apsk128()  { modem_test_demodstats(LIQUID_MODEM_APSK128);  }
void autotest_demodstats_apsk256()  { modem_test_demodstats(LIQUID_MODEM_APSK256);  }

// AUTOTESTS: Specific modems
void autotest_demodstats_bpsk()     { modem_test_demodstats(LIQUID_MODEM_BPSK);     }
void autotest_demodstats_qpsk()     { modem_test_demodstats(LIQUID_MODEM_QPSK);     }
void autotest_demodstats_ook()      { modem_test_demodstats(LIQUID_MODEM_OOK);      }
void autotest_demodstats_sqam32()   { modem_test_demodstats(LIQUID_MODEM_SQAM32);   }
void autotest_demodstats_sqam128()  { modem_test_demodstats(LIQUID_MODEM_SQAM128);  }
void autotest_demodstats_V29()      { modem_test_demodstats(LIQUID_MODEM_V29);      }
void autotest_demodstats_arb16opt() { modem_test_demodstats(LIQUID_MODEM_ARB16OPT); }
void autotest_demodstats_arb32opt() { modem_test_demodstats(LIQUID_MODEM_ARB32OPT); }
void autotest_demodstats_arb64opt() { modem_test_demodstats(LIQUID_MODEM_ARB64OPT); }
void autotest_demodstats_arb128opt(){ modem_test_demodstats(LIQUID_MODEM_ARB128OPT);}
void autotest_demodstats_arb256opt(){ modem_test_demodstats(LIQUID_MODEM_ARB256OPT);}
void autotest_demodstats_arb64vt()  { modem_test_demodstats(LIQUID_MODEM_ARB64VT);  }

