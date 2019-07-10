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
// ofdmflexframesync.c
//
// OFDM frame synchronizer
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "liquid.internal.h"

#define DEBUG_OFDMFLEXFRAMESYNC 0

#define OFDMFLEXFRAME_H_SOFT (0)

// 
// ofdmflexframesync
//

// internal callback
int ofdmflexframesync_internal_callback(float complex * _X,
                                        unsigned char * _p,
                                        unsigned int    _M,
                                        void * _userdata);

// receive header data
void ofdmflexframesync_rxheader(ofdmflexframesync _q,
                                float complex * _X);

// decode header
void ofdmflexframesync_decode_header(ofdmflexframesync _q);

// receive payload data
void ofdmflexframesync_rxpayload(ofdmflexframesync _q,
                                float complex * _X);


struct ofdmflexframesync_s {
    unsigned int M;         // number of subcarriers
    unsigned int cp_len;    // cyclic prefix length
    unsigned int taper_len; // taper length
    unsigned char * p;      // subcarrier allocation (null, pilot, data)

    // constants
    unsigned int M_null;    // number of null subcarriers
    unsigned int M_pilot;   // number of pilot subcarriers
    unsigned int M_data;    // number of data subcarriers
    unsigned int M_S0;      // number of enabled subcarriers in S0
    unsigned int M_S1;      // number of enabled subcarriers in S1

    // header
    modem mod_header;                   // header modulator
    packetizer p_header;                // header packetizer
    unsigned char header[OFDMFLEXFRAME_H_DEC];      // header data (uncoded)
#if OFDMFLEXFRAME_H_SOFT
    unsigned char header_enc[8*OFDMFLEXFRAME_H_ENC];  // header data (encoded, soft bits)
    unsigned char header_mod[OFDMFLEXFRAME_H_BPS*OFDMFLEXFRAME_H_SYM];  // header symbols (soft bits)
#else
    unsigned char header_enc[OFDMFLEXFRAME_H_ENC];  // header data (encoded)
    unsigned char header_mod[OFDMFLEXFRAME_H_SYM];  // header symbols
#endif
    int header_valid;                   // valid header flag

    // header properties
    modulation_scheme ms_payload;       // payload modulation scheme
    unsigned int bps_payload;           // payload modulation depth (bits/symbol)
    unsigned int payload_len;           // payload length (number of bytes)
    crc_scheme check;                   // payload validity check
    fec_scheme fec0;                    // payload FEC (inner)
    fec_scheme fec1;                    // payload FEC (outer)

    // payload
    packetizer p_payload;               // payload packetizer
    modem mod_payload;                  // payload demodulator
    unsigned char * payload_enc;        // payload data (encoded bytes)
    unsigned char * payload_dec;        // payload data (decoded bytes)
    unsigned int payload_enc_len;       // length of encoded payload
    unsigned int payload_mod_len;       // number of payload modem symbols
    int payload_valid;                  // valid payload flag
    float complex * payload_syms;       // received payload symbols

    // callback
    framesync_callback callback;        // user-defined callback function
    void * userdata;                    // user-defined data structure
    framesyncstats_s framestats;        // frame statistic object
    float evm_hat;                      // average error vector magnitude

    // internal synchronizer objects
    ofdmframesync fs;                   // internal OFDM frame synchronizer

    // counters/states
    unsigned int symbol_counter;        // received symbol number
    enum {
        OFDMFLEXFRAMESYNC_STATE_HEADER, // extract header
        OFDMFLEXFRAMESYNC_STATE_PAYLOAD // extract payload symbols
    } state;
    unsigned int header_symbol_index;   // number of header symbols received
    unsigned int payload_symbol_index;  // number of payload symbols received
    unsigned int payload_buffer_index;  // bit-level index of payload (pack array)
};

// create ofdmflexframesync object
//  _M          :   number of subcarriers
//  _cp_len     :   length of cyclic prefix [samples]
//  _taper_len  :   taper length (OFDM symbol overlap)
//  _p          :   subcarrier allocation (PILOT/NULL/DATA) [size: _M x 1]
//  _callback   :   user-defined callback function
//  _userdata   :   user-defined data structure passed to callback
ofdmflexframesync ofdmflexframesync_create(unsigned int       _M,
                                           unsigned int       _cp_len,
                                           unsigned int       _taper_len,
                                           unsigned char *    _p,
                                           framesync_callback _callback,
                                           void *             _userdata)
{
    ofdmflexframesync q = (ofdmflexframesync) malloc(sizeof(struct ofdmflexframesync_s));

    // validate input
    if (_M < 8) {
        fprintf(stderr,"warning: ofdmflexframesync_create(), less than 8 subcarriers\n");
    } else if (_M % 2) {
        fprintf(stderr,"error: ofdmflexframesync_create(), number of subcarriers must be even\n");
        exit(1);
    } else if (_cp_len > _M) {
        fprintf(stderr,"error: ofdmflexframesync_create(), cyclic prefix length cannot exceed number of subcarriers\n");
        exit(1);
    }

    // set internal properties
    q->M         = _M;
    q->cp_len    = _cp_len;
    q->taper_len = _taper_len;
    q->callback  = _callback;
    q->userdata  = _userdata;

    // allocate memory for subcarrier allocation IDs
    q->p = (unsigned char*) malloc((q->M)*sizeof(unsigned char));
    if (_p == NULL) {
        // initialize default subcarrier allocation
        ofdmframe_init_default_sctype(q->M, q->p);
    } else {
        // copy user-defined subcarrier allocation
        memmove(q->p, _p, q->M*sizeof(unsigned char));
    }

    // validate and count subcarrier allocation
    ofdmframe_validate_sctype(q->p, q->M, &q->M_null, &q->M_pilot, &q->M_data);

    // create internal framing object
    q->fs = ofdmframesync_create(_M, _cp_len, _taper_len, _p, ofdmflexframesync_internal_callback, (void*)q);

    // create header objects
    q->mod_header = modem_create(OFDMFLEXFRAME_H_MOD);
    q->p_header   = packetizer_create(OFDMFLEXFRAME_H_DEC,
                                      OFDMFLEXFRAME_H_CRC,
                                      OFDMFLEXFRAME_H_FEC,
                                      LIQUID_FEC_NONE);
    assert(packetizer_get_enc_msg_len(q->p_header)==OFDMFLEXFRAME_H_ENC);

    // frame properties (default values to be overwritten when frame
    // header is received and properly decoded)
    q->ms_payload   = LIQUID_MODEM_QPSK;
    q->bps_payload  = 2;
    q->payload_len  = 1;
    q->check        = LIQUID_CRC_NONE;
    q->fec0         = LIQUID_FEC_NONE;
    q->fec1         = LIQUID_FEC_NONE;

    // create payload objects (initally QPSK, etc but overridden by received properties)
    q->mod_payload = modem_create(q->ms_payload);
    q->p_payload   = packetizer_create(q->payload_len, q->check, q->fec0, q->fec1);
    q->payload_enc_len = packetizer_get_enc_msg_len(q->p_payload);
    q->payload_enc = (unsigned char*) malloc(q->payload_enc_len*sizeof(unsigned char));
    q->payload_dec = (unsigned char*) malloc(q->payload_len*sizeof(unsigned char));
    q->payload_syms = (float complex *) malloc(q->payload_len*sizeof(float complex));
    q->payload_mod_len = 0;

    // reset state
    ofdmflexframesync_reset(q);

    // return object
    return q;
}

void ofdmflexframesync_destroy(ofdmflexframesync _q)
{
    // destroy internal objects
    ofdmframesync_destroy(_q->fs);
    packetizer_destroy(_q->p_header);
    modem_destroy(_q->mod_header);
    packetizer_destroy(_q->p_payload);
    modem_destroy(_q->mod_payload);

    // free internal buffers/arrays
    free(_q->p);
    free(_q->payload_enc);
    free(_q->payload_dec);
    free(_q->payload_syms);

    // free main object memory
    free(_q);
}

void ofdmflexframesync_print(ofdmflexframesync _q)
{
    printf("ofdmflexframesync:\n");
    printf("    num subcarriers     :   %-u\n", _q->M);
    printf("      * NULL            :   %-u\n", _q->M_null);
    printf("      * pilot           :   %-u\n", _q->M_pilot);
    printf("      * data            :   %-u\n", _q->M_data);
    printf("    cyclic prefix len   :   %-u\n", _q->cp_len);
    printf("    taper len           :   %-u\n", _q->taper_len);
}

void ofdmflexframesync_reset(ofdmflexframesync _q)
{
    // reset internal state
    _q->state = OFDMFLEXFRAMESYNC_STATE_HEADER;

    // reset internal counters
    _q->symbol_counter=0;
    _q->header_symbol_index=0;
    _q->payload_symbol_index=0;
    _q->payload_buffer_index=0;
    
    // reset error vector magnitude estimate
    _q->evm_hat = 1e-12f;   // slight offset to ensure no log(0)

    // reset framestats object
    framesyncstats_init_default(&_q->framestats);

    // reset internal OFDM frame synchronizer object
    ofdmframesync_reset(_q->fs);
}

int ofdmflexframesync_is_frame_open(ofdmflexframesync _q)
{
    return ofdmframesync_is_frame_open(_q->fs);
}

// execute synchronizer object on buffer of samples
void ofdmflexframesync_execute(ofdmflexframesync _q,
                               float complex * _x,
                               unsigned int _n)
{
    // push samples through ofdmframesync object
    ofdmframesync_execute(_q->fs, _x, _n);
}

// 
// query methods
//

// received signal strength indication
float ofdmflexframesync_get_rssi(ofdmflexframesync _q)
{
    return ofdmframesync_get_rssi(_q->fs);
}

// received carrier frequency offset
float ofdmflexframesync_get_cfo(ofdmflexframesync _q)
{
    return ofdmframesync_get_cfo(_q->fs);
}

// 
// debugging methods
//

// enable debugging for internal ofdm frame synchronizer
void ofdmflexframesync_debug_enable(ofdmflexframesync _q)
{
    ofdmframesync_debug_enable(_q->fs);
}

// disable debugging for internal ofdm frame synchronizer
void ofdmflexframesync_debug_disable(ofdmflexframesync _q)
{
    ofdmframesync_debug_disable(_q->fs);
}

// print debugging file for internal ofdm frame synchronizer
void ofdmflexframesync_debug_print(ofdmflexframesync _q,
                                   const char *      _filename)
{
    ofdmframesync_debug_print(_q->fs, _filename);
}

//
// internal methods
//

// internal callback
//  _X          :   subcarrier symbols
//  _p          :   subcarrier allocation
//  _M          :   number of subcarriers
//  _userdata   :   user-defined data structure
int ofdmflexframesync_internal_callback(float complex * _X,
                                        unsigned char * _p,
                                        unsigned int    _M,
                                        void * _userdata)
{
#if DEBUG_OFDMFLEXFRAMESYNC
    printf("******* ofdmflexframesync callback invoked!\n");
#endif
    // type-cast userdata as ofdmflexframesync object
    ofdmflexframesync _q = (ofdmflexframesync) _userdata;

    _q->symbol_counter++;

#if DEBUG_OFDMFLEXFRAMESYNC
    printf("received symbol %u\n", _q->symbol_counter);
#endif

    // extract symbols
    switch (_q->state) {
    case OFDMFLEXFRAMESYNC_STATE_HEADER:
        ofdmflexframesync_rxheader(_q, _X);
        break;
    case OFDMFLEXFRAMESYNC_STATE_PAYLOAD:
        ofdmflexframesync_rxpayload(_q, _X);
        break;
    default:
        fprintf(stderr,"error: ofdmflexframesync_internal_callback(), unknown/unsupported internal state\n");
        exit(1);
    }

    // return
    return 0;
}

// receive header data
void ofdmflexframesync_rxheader(ofdmflexframesync _q,
                                float complex * _X)
{
#if DEBUG_OFDMFLEXFRAMESYNC
    printf("  ofdmflexframesync extracting header...\n");
#endif

    // demodulate header symbols
    unsigned int i;
    int sctype;
    for (i=0; i<_q->M; i++) {
        // subcarrier type (PILOT/NULL/DATA)
        sctype = _q->p[i];

        // ignore pilot and null subcarriers
        if (sctype == OFDMFRAME_SCTYPE_DATA) {
            // unload header symbols
            // demodulate header symbol
            unsigned int sym;
#if OFDMFLEXFRAME_H_SOFT
            modem_demodulate_soft(_q->mod_header, _X[i], &sym, &_q->header_mod[OFDMFLEXFRAME_H_BPS*_q->header_symbol_index]);
#else
            modem_demodulate(_q->mod_header, _X[i], &sym);
            _q->header_mod[_q->header_symbol_index] = sym;
#endif
            _q->header_symbol_index++;
            //printf("  extracting symbol %3u / %3u (x = %8.5f + j%8.5f)\n", _q->header_symbol_index, OFDMFLEXFRAME_H_SYM, crealf(_X[i]), cimagf(_X[i]));

            // get demodulator error vector magnitude
            float evm = modem_get_demodulator_evm(_q->mod_header);
            _q->evm_hat += evm*evm;

            // header extracted
            if (_q->header_symbol_index == OFDMFLEXFRAME_H_SYM) {
                // decode header
                ofdmflexframesync_decode_header(_q);
            
                // compute error vector magnitude estimate
                _q->framestats.evm = 10*log10f( _q->evm_hat/OFDMFLEXFRAME_H_SYM );

                // invoke callback if header is invalid
                if (_q->header_valid)
                    _q->state = OFDMFLEXFRAMESYNC_STATE_PAYLOAD;
                else {
                    //printf("**** header invalid!\n");
                    // set framestats internals
                    _q->framestats.rssi             = ofdmframesync_get_rssi(_q->fs);
                    _q->framestats.cfo              = ofdmframesync_get_cfo(_q->fs);
                    _q->framestats.framesyms        = NULL;
                    _q->framestats.num_framesyms    = 0;
                    _q->framestats.mod_scheme       = LIQUID_MODEM_UNKNOWN;
                    _q->framestats.mod_bps          = 0;
                    _q->framestats.check            = LIQUID_CRC_UNKNOWN;
                    _q->framestats.fec0             = LIQUID_FEC_UNKNOWN;
                    _q->framestats.fec1             = LIQUID_FEC_UNKNOWN;

                    // invoke callback method
                    _q->callback(_q->header,
                                 _q->header_valid,
                                 NULL,
                                 0,
                                 0,
                                 _q->framestats,
                                 _q->userdata);

                    ofdmflexframesync_reset(_q);
                }
                break;
            }
        }
    }
}

// decode header
void ofdmflexframesync_decode_header(ofdmflexframesync _q)
{
#if OFDMFLEXFRAME_H_SOFT
#  if 0
    unsigned int i;
    // copy soft bits
    for (i=0; i<8*OFDMFLEXFRAME_H_ENC; i++)
        _q->header_enc[i] = _q->header_mod[i];
#  else
    // TODO: ensure lengths are the same
    memmove(_q->header_enc, _q->header_mod, 8*OFDMFLEXFRAME_H_ENC);
#  endif

    // unscramble header using soft bits
    unscramble_data_soft(_q->header_enc, OFDMFLEXFRAME_H_ENC);

    // run packet decoder
    _q->header_valid = packetizer_decode_soft(_q->p_header, _q->header_enc, _q->header);
#else
    // pack 1-bit header symbols into 8-bit bytes
    unsigned int num_written;
    liquid_repack_bytes(_q->header_mod, OFDMFLEXFRAME_H_BPS, OFDMFLEXFRAME_H_SYM,
                        _q->header_enc, 8,                   OFDMFLEXFRAME_H_ENC,
                        &num_written);
    assert(num_written==OFDMFLEXFRAME_H_ENC);

    // unscramble header
    unscramble_data(_q->header_enc, OFDMFLEXFRAME_H_ENC);

    // run packet decoder
    _q->header_valid = packetizer_decode(_q->p_header, _q->header_enc, _q->header);
#endif

#if 0
    // print header
    printf("header rx (enc) : ");
    for (i=0; i<OFDMFLEXFRAME_H_ENC; i++)
        printf("%.2X ", _q->header_enc[i]);
    printf("\n");

    // print header
    printf("header rx (dec) : ");
    for (i=0; i<OFDMFLEXFRAME_H_DEC; i++)
        printf("%.2X ", _q->header[i]);
    printf("\n");
#endif

#if DEBUG_OFDMFLEXFRAMESYNC
    printf("****** header extracted [%s]\n", _q->header_valid ? "valid" : "INVALID!");
#endif
    if (!_q->header_valid)
        return;

    unsigned int n = OFDMFLEXFRAME_H_USER;

    // first byte is for expansion/version validation
    if (_q->header[n+0] != OFDMFLEXFRAME_PROTOCOL) {
        fprintf(stderr,"warning: ofdmflexframesync_decode_header(), invalid framing version\n");
        _q->header_valid = 0;
    }

    // strip off payload length
    unsigned int payload_len = (_q->header[n+1] << 8) | (_q->header[n+2]);

    // strip off modulation scheme/depth
    unsigned int mod_scheme = _q->header[n+3];
    if (mod_scheme == 0 || mod_scheme >= LIQUID_MODEM_NUM_SCHEMES) {
        fprintf(stderr,"warning: ofdmflexframesync_decode_header(), invalid modulation scheme\n");
        _q->header_valid = 0;
        return;
    }

    // strip off CRC, forward error-correction schemes
    //  CRC     : most-significant 3 bits of [n+4]
    //  fec0    : least-significant 5 bits of [n+4]
    //  fec1    : least-significant 5 bits of [n+5]
    unsigned int check = (_q->header[n+4] >> 5 ) & 0x07;
    unsigned int fec0  = (_q->header[n+4]      ) & 0x1f;
    unsigned int fec1  = (_q->header[n+5]      ) & 0x1f;

    // validate properties
    if (check >= LIQUID_CRC_NUM_SCHEMES) {
        fprintf(stderr,"warning: ofdmflexframesync_decode_header(), decoded CRC exceeds available\n");
        check = LIQUID_CRC_UNKNOWN;
        _q->header_valid = 0;
    }
    if (fec0 >= LIQUID_FEC_NUM_SCHEMES) {
        fprintf(stderr,"warning: ofdmflexframesync_decode_header(), decoded FEC (inner) exceeds available\n");
        fec0 = LIQUID_FEC_UNKNOWN;
        _q->header_valid = 0;
    }
    if (fec1 >= LIQUID_FEC_NUM_SCHEMES) {
        fprintf(stderr,"warning: ofdmflexframesync_decode_header(), decoded FEC (outer) exceeds available\n");
        fec1 = LIQUID_FEC_UNKNOWN;
        _q->header_valid = 0;
    }

    // print results
#if DEBUG_OFDMFLEXFRAMESYNC
    printf("    properties:\n");
    printf("      * mod scheme      :   %s\n", modulation_types[mod_scheme].fullname);
    printf("      * fec (inner)     :   %s\n", fec_scheme_str[fec0][1]);
    printf("      * fec (outer)     :   %s\n", fec_scheme_str[fec1][1]);
    printf("      * CRC scheme      :   %s\n", crc_scheme_str[check][1]);
    printf("      * payload length  :   %u bytes\n", payload_len);
#endif

    // configure payload receiver
    if (_q->header_valid) {
        // configure modem
        if (mod_scheme != _q->ms_payload) {
            // set new properties
            _q->ms_payload  = mod_scheme;
            _q->bps_payload = modulation_types[mod_scheme].bps;

            // recreate modem (destroy/create)
            _q->mod_payload = modem_recreate(_q->mod_payload, _q->ms_payload);
        }

        // set new packetizer properties
        _q->payload_len = payload_len;
        _q->check       = check;
        _q->fec0        = fec0;
        _q->fec1        = fec1;
        
        // recreate packetizer object
        _q->p_payload = packetizer_recreate(_q->p_payload,
                                            _q->payload_len,
                                            _q->check,
                                            _q->fec0,
                                            _q->fec1);

        // re-compute payload encoded message length
        _q->payload_enc_len = packetizer_get_enc_msg_len(_q->p_payload);
#if DEBUG_OFDMFLEXFRAMESYNC
        printf("      * payload encoded :   %u bytes\n", _q->payload_enc_len);
#endif

        // re-allocate buffers accordingly
        _q->payload_enc = (unsigned char*) realloc(_q->payload_enc, _q->payload_enc_len*sizeof(unsigned char));
        _q->payload_dec = (unsigned char*) realloc(_q->payload_dec, _q->payload_len*sizeof(unsigned char));

        // re-compute number of modulated payload symbols
        div_t d = div(8*_q->payload_enc_len, _q->bps_payload);
        _q->payload_mod_len = d.quot + (d.rem ? 1 : 0);
        _q->payload_syms = (float complex*) realloc(_q->payload_syms, _q->payload_mod_len*sizeof(float complex));
#if DEBUG_OFDMFLEXFRAMESYNC
        printf("      * payload mod syms:   %u symbols\n", _q->payload_mod_len);
#endif
    }
}

// receive payload data
void ofdmflexframesync_rxpayload(ofdmflexframesync _q,
                                 float complex * _X)
{
    // demodulate paylod symbols
    unsigned int i;
    int sctype;
    for (i=0; i<_q->M; i++) {
        // subcarrier type (PILOT/NULL/DATA)
        sctype = _q->p[i];

        // ignore pilot and null subcarriers
        if (sctype == OFDMFRAME_SCTYPE_DATA) {
            // unload payload symbols
            unsigned int sym;
            modem_demodulate(_q->mod_payload, _X[i], &sym);

            // store received symbol
            _q->payload_syms[_q->payload_symbol_index] = _X[i];

            // pack decoded symbol into array
            liquid_pack_array(_q->payload_enc,
                              _q->payload_enc_len,
                              _q->payload_buffer_index,
                              _q->bps_payload,
                              sym);

            // increment...
            _q->payload_buffer_index += _q->bps_payload;

            // increment symbol counter
            _q->payload_symbol_index++;

            if (_q->payload_symbol_index == _q->payload_mod_len) {
                // payload extracted

                // decode payload
                _q->payload_valid = packetizer_decode(_q->p_payload, _q->payload_enc, _q->payload_dec);
#if DEBUG_OFDMFLEXFRAMESYNC
                printf("****** payload extracted [%s]\n", _q->payload_valid ? "valid" : "INVALID!");
#endif

                // ignore callback if set to NULL
                if (_q->callback == NULL) {
                    ofdmflexframesync_reset(_q);
                    break;
                }

                // set framestats internals
                _q->framestats.rssi             = ofdmframesync_get_rssi(_q->fs);
                _q->framestats.cfo              = ofdmframesync_get_cfo(_q->fs);
                _q->framestats.framesyms        = _q->payload_syms;
                _q->framestats.num_framesyms    = _q->payload_mod_len;
                _q->framestats.mod_scheme       = _q->ms_payload;
                _q->framestats.mod_bps          = _q->bps_payload;
                _q->framestats.check            = _q->check;
                _q->framestats.fec0             = _q->fec0;
                _q->framestats.fec1             = _q->fec1;

                // invoke callback method
                _q->callback(_q->header,
                             _q->header_valid,
                             _q->payload_dec,
                             _q->payload_len,
                             _q->payload_valid,
                             _q->framestats,
                             _q->userdata);


                // reset object
                ofdmflexframesync_reset(_q);
                break;
            }
        }
    }
}



