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
// windowed delay, defined by macro
//

#include "liquid.internal.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

struct WDELAY(_s) {
    T * v;                      // allocated array pointer
    unsigned int delay;         // length of window
    unsigned int read_index;
};

// create delay buffer object with '_delay' samples
WDELAY() WDELAY(_create)(unsigned int _delay)
{
    // create main object
    WDELAY() q = (WDELAY()) malloc(sizeof(struct WDELAY(_s)));

    // set internal values
    q->delay = _delay;

    // allocte memory
    q->v = (T*) malloc((q->delay)*sizeof(T));
    q->read_index = 0;

    // clear window
    WDELAY(_reset)(q);

    return q;
}

// re-create delay buffer object with '_delay' samples
//  _q      :   old delay buffer object
//  _delay  :   delay for new object
WDELAY() WDELAY(_recreate)(WDELAY()     _q,
                           unsigned int _delay)
{
    // copy internal buffer, re-aligned
    unsigned int ktmp = _q->delay;
    T * vtmp = (T*) malloc(_q->delay * sizeof(T));
    unsigned int i;
    for (i=0; i<_q->delay; i++)
        vtmp[i] = _q->v[ (i + _q->read_index) % _q->delay ];
    
    // destroy object and re-create it
    WDELAY(_destroy)(_q);
    _q = WDELAY(_create)(_delay);

    // push old values
    for (i=0; i<ktmp; i++)
        WDELAY(_push)(_q, vtmp[i]);

    // free temporary array
    free(vtmp);

    // return object
    return _q;
}

// destroy delay buffer object, freeing internal memory
void WDELAY(_destroy)(WDELAY() _q)
{
    // free internal array buffer
    free(_q->v);

    // free main object memory
    free(_q);
}

// print delay buffer object's state to stdout
void WDELAY(_print)(WDELAY() _q)
{
    printf("wdelay [%u elements] :\n", _q->delay);
    unsigned int i, j;
    for (i=0; i<_q->delay; i++) {
        j = (i + _q->read_index) % _q->delay;
        printf("%4u", i);
        BUFFER_PRINT_VALUE(_q->v[j]);
        printf("\n");
    }
}

// clear/reset state of object
void WDELAY(_reset)(WDELAY() _q)
{
    _q->read_index = 0;
    memset(_q->v, 0, (_q->delay)*sizeof(T));
}

// read delayed sample from delay buffer object
//  _q  :   delay buffer object
//  _v  :   value of delayed element
void WDELAY(_read)(WDELAY() _q,
                   T *      _v)
{
    // return value at end of buffer
    *_v = _q->v[_q->read_index];
}

// push new sample into delay buffer object
//  _q  :   delay buffer object
//  _v  :   new value to be added to buffer
void WDELAY(_push)(WDELAY() _q,
                   T        _v)
{
    // append value to end of buffer
    _q->v[_q->read_index] = _v;

    // increment index
    _q->read_index++;

    // wrap around pointer
    _q->read_index %= _q->delay;
}

