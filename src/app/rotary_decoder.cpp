/*
 * Copyright (c) 2016 Immo Software
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "rotary_decoder.h"
#include "board.h"

using namespace slab;

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

const int RotaryDecoder::s_encoderStates[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

RotaryDecoder::RotaryDecoder()
:   m_oldState(0)
{
}

int RotaryDecoder::decode(uint8_t a, uint8_t b)
{
    uint8_t newState = ((b & 1) << 1) | (a & 1);

    m_oldState = ((m_oldState & 3) << 2) | newState;

    return s_encoderStates[m_oldState];

    // First, find the newEncoderState. This'll be a 2-bit value
    // the msb is the state of the B pin. The lsb is the state
    // of the A pin on the encoder.
//     newEncoderState = (digitalRead(bPin)<<1) | (digitalRead(aPin));

    // Now we pair oldEncoderState with new encoder state
    // First we need to shift oldEncoder state left two bits.
    // This'll put the last state in bits 2 and 3.
//     oldEncoderState <<= 2;

    // Mask out everything in oldEncoderState except for the previous state
//     oldEncoderState &= 0xC0;

    // Now add the newEncoderState. oldEncoderState will now be of
    // the form: 0b0000(old B)(old A)(new B)(new A)
//     oldEncoderState |= newEncoderState; // add filteredport value
}

// void encoder_debounce(ar_timer_t * timer, void * arg)
// {
//     int a = g_a;
//     int b = g_b;
// //     printf("encoder: a=%d, b=%d\r\n", a, b);
//
//     int delta = g_decoder.decode(a, b);
//     g_value += delta;
//     if (g_value < 0)
//     {
//         g_value = 0;
//     }
//     else if (g_value > 100)
//     {
//         g_value = 100;
//     }
// }

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
