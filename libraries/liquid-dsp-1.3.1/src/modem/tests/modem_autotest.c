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

// AUTOTESTS: generic PSK
void autotest_mod_demod_psk2()      { modem_test_mod_demod(LIQUID_MODEM_PSK2);      }
void autotest_mod_demod_psk4()      { modem_test_mod_demod(LIQUID_MODEM_PSK4);      }
void autotest_mod_demod_psk8()      { modem_test_mod_demod(LIQUID_MODEM_PSK8);      }
void autotest_mod_demod_psk16()     { modem_test_mod_demod(LIQUID_MODEM_PSK16);     }
void autotest_mod_demod_psk32()     { modem_test_mod_demod(LIQUID_MODEM_PSK32);     }
void autotest_mod_demod_psk64()     { modem_test_mod_demod(LIQUID_MODEM_PSK64);     }
void autotest_mod_demod_psk128()    { modem_test_mod_demod(LIQUID_MODEM_PSK128);    }
void autotest_mod_demod_psk256()    { modem_test_mod_demod(LIQUID_MODEM_PSK256);    }

// AUTOTESTS: generic DPSK
void autotest_mod_demod_dpsk2()     { modem_test_mod_demod(LIQUID_MODEM_DPSK2);     }
void autotest_mod_demod_dpsk4()     { modem_test_mod_demod(LIQUID_MODEM_DPSK4);     }
void autotest_mod_demod_dpsk8()     { modem_test_mod_demod(LIQUID_MODEM_DPSK8);     }
void autotest_mod_demod_dpsk16()    { modem_test_mod_demod(LIQUID_MODEM_DPSK16);    }
void autotest_mod_demod_dpsk32()    { modem_test_mod_demod(LIQUID_MODEM_DPSK32);    }
void autotest_mod_demod_dpsk64()    { modem_test_mod_demod(LIQUID_MODEM_DPSK64);    }
void autotest_mod_demod_dpsk128()   { modem_test_mod_demod(LIQUID_MODEM_DPSK128);   }
void autotest_mod_demod_dpsk256()   { modem_test_mod_demod(LIQUID_MODEM_DPSK256);   }

// AUTOTESTS: generic ASK
void autotest_mod_demod_ask2()      { modem_test_mod_demod(LIQUID_MODEM_ASK2);      }
void autotest_mod_demod_ask4()      { modem_test_mod_demod(LIQUID_MODEM_ASK4);      }
void autotest_mod_demod_ask8()      { modem_test_mod_demod(LIQUID_MODEM_ASK8);      }
void autotest_mod_demod_ask16()     { modem_test_mod_demod(LIQUID_MODEM_ASK16);     }
void autotest_mod_demod_ask32()     { modem_test_mod_demod(LIQUID_MODEM_ASK32);     }
void autotest_mod_demod_ask64()     { modem_test_mod_demod(LIQUID_MODEM_ASK64);     }
void autotest_mod_demod_ask128()    { modem_test_mod_demod(LIQUID_MODEM_ASK128);    }
void autotest_mod_demod_ask256()    { modem_test_mod_demod(LIQUID_MODEM_ASK256);    }

// AUTOTESTS: generic QAM
void autotest_mod_demod_qam4()      { modem_test_mod_demod(LIQUID_MODEM_QAM4);      }
void autotest_mod_demod_qam8()      { modem_test_mod_demod(LIQUID_MODEM_QAM8);      }
void autotest_mod_demod_qam16()     { modem_test_mod_demod(LIQUID_MODEM_QAM16);     }
void autotest_mod_demod_qam32()     { modem_test_mod_demod(LIQUID_MODEM_QAM32);     }
void autotest_mod_demod_qam64()     { modem_test_mod_demod(LIQUID_MODEM_QAM64);     }
void autotest_mod_demod_qam128()    { modem_test_mod_demod(LIQUID_MODEM_QAM128);    }
void autotest_mod_demod_qam256()    { modem_test_mod_demod(LIQUID_MODEM_QAM256);    }

// AUTOTESTS: generic APSK (maps to specific APSK modems internally)
void autotest_mod_demod_apsk4()     { modem_test_mod_demod(LIQUID_MODEM_APSK4);     }
void autotest_mod_demod_apsk8()     { modem_test_mod_demod(LIQUID_MODEM_APSK8);     }
void autotest_mod_demod_apsk16()    { modem_test_mod_demod(LIQUID_MODEM_APSK16);    }
void autotest_mod_demod_apsk32()    { modem_test_mod_demod(LIQUID_MODEM_APSK32);    }
void autotest_mod_demod_apsk64()    { modem_test_mod_demod(LIQUID_MODEM_APSK64);    }
void autotest_mod_demod_apsk128()   { modem_test_mod_demod(LIQUID_MODEM_APSK128);   }
void autotest_mod_demod_apsk256()   { modem_test_mod_demod(LIQUID_MODEM_APSK256);   }

// AUTOTESTS: Specific modems
void autotest_mod_demod_bpsk()      { modem_test_mod_demod(LIQUID_MODEM_BPSK);      }
void autotest_mod_demod_qpsk()      { modem_test_mod_demod(LIQUID_MODEM_QPSK);      }
void autotest_mod_demod_ook()       { modem_test_mod_demod(LIQUID_MODEM_OOK);       }
void autotest_mod_demod_sqam32()    { modem_test_mod_demod(LIQUID_MODEM_SQAM32);    }
void autotest_mod_demod_sqam128()   { modem_test_mod_demod(LIQUID_MODEM_SQAM128);   }
void autotest_mod_demod_V29()       { modem_test_mod_demod(LIQUID_MODEM_V29);       }
void autotest_mod_demod_arb16opt()  { modem_test_mod_demod(LIQUID_MODEM_ARB16OPT);  }
void autotest_mod_demod_arb32opt()  { modem_test_mod_demod(LIQUID_MODEM_ARB32OPT);  }
void autotest_mod_demod_arb64opt()  { modem_test_mod_demod(LIQUID_MODEM_ARB64OPT);  }
void autotest_mod_demod_arb128opt() { modem_test_mod_demod(LIQUID_MODEM_ARB128OPT); }
void autotest_mod_demod_arb256opt() { modem_test_mod_demod(LIQUID_MODEM_ARB256OPT); }
void autotest_mod_demod_arb64vt()   { modem_test_mod_demod(LIQUID_MODEM_ARB64VT);   }

