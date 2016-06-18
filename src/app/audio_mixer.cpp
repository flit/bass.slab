/*
 * Copyright (c) 2015 Immo Software
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

#include "audio_mixer.h"
#include "arm_math.h"
#include <string.h>
#include <assert.h>

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

AudioMixer::AudioMixer()
:   AudioFilter(),
    m_inputCount(0)
{
    memset(m_inputs, 0, sizeof(m_inputs));
}

void AudioMixer::set_input(uint32_t inputNumber, AudioFilter * source, float gain)
{
    assert(inputNumber < m_inputCount);
    m_inputs[inputNumber] = source;
    m_gains[inputNumber] = gain;
}

void AudioMixer::set_gain(uint32_t inputNumber, float gain)
{
    assert(inputNumber < m_inputCount);
    m_gains[inputNumber] = gain;
}

void AudioMixer::process(float * samples, uint32_t count)
{
    assert(m_inputCount > 0 && m_inputs[0]);

    // Process first input directly into output.
    m_inputs[0]->process(samples, count);

    // Apply gain to first input.
    arm_scale_f32(samples, m_gains[0], samples, count);

    // Other inputs go to a temp buffer first.
    assert(m_buffer.get_count() >= count);
    float * buf = m_buffer.get_buffer();
    int i;
    for (i = 1; i < m_inputCount; ++i)
    {
        float igain = m_gains[i];
        m_inputs[i]->process(buf, count);

        int j;
        for (j = 0; j < count; ++j)
        {
            float bufj = buf[j];
            float samplesj = samples[j];
            float tmp = bufj * igain + samplesj;
            samples[j] = tmp;
        }
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
