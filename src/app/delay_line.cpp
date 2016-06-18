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

#include "delay_line.h"
#include "arm_math.h"
#include <string.h>
#include <assert.h>

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

DelayLine::DelayLine()
:   AudioFilter(),
    m_delayLine(0),
    m_maxDelaySamples(0),
    m_delaySamples(0),
    m_readHead(0),
    m_writeHead(0),
    m_feedback(0.0)
{
}

DelayLine::DelayLine(unsigned delaySamples)
:   AudioFilter(),
    m_delayLine(0),
    m_maxDelaySamples(0),
    m_delaySamples(0),
    m_readHead(0),
    m_writeHead(0),
    m_feedback(0.0)
{
	set_maximum_delay_samples(delaySamples);
}

DelayLine::~DelayLine()
{
	if (m_delayLine)
	{
		delete [] m_delayLine;
	}
}

void DelayLine::set_maximum_delay_samples(unsigned delaySamples)
{
	if (m_delayLine)
	{
		delete [] m_delayLine;
	}
	m_delayLine = new float[delaySamples];

	m_maxDelaySamples = delaySamples;
	m_delaySamples = delaySamples;

	// clear delay line and init read and write heads
	reset();
}

void DelayLine::set_maximum_delay_seconds(float delaySeconds)
{
	assert(m_sampleRate != 0.0);

	set_maximum_delay_samples(unsigned(delaySeconds * m_sampleRate));
}

//! Cannot set the delay time if the buffer has not been allocated yet.
//! This is to prevent the \c Tick() method from trying to write to
//! unallocated memory.
void DelayLine::set_delay_samples(unsigned delaySamples)
{
	if (m_delayLine == NULL)
	{
		return;
	}
	if (delaySamples > m_maxDelaySamples)
	{
		delaySamples = m_maxDelaySamples;
	}
	m_delaySamples = delaySamples;
	if (m_readHead >= m_delaySamples)
	{
		m_readHead = 0;
	}
	if (m_writeHead >= m_delaySamples)
	{
		m_writeHead = m_delaySamples-1;
	}
}

void DelayLine::set_delay_seconds(float delaySeconds)
{
	assert(m_sampleRate != 0.0);

	set_delay_samples(unsigned(delaySeconds * m_sampleRate));
}

//! Clears the contents of the delay and moves the read and write heads
//! back to the starting positions.
void DelayLine::reset()
{
	// clear delay line
	memset(m_delayLine, 0, sizeof(float) * m_maxDelaySamples);

	// reset read and write heads
	m_readHead = 0;
	m_writeHead = m_delaySamples - 1;
}

float DelayLine::tick(float inSample)
{
	assert(m_delayLine != NULL);

	// read
	float output = m_delayLine[m_readHead];
	m_readHead = (m_readHead + 1) % m_delaySamples;

	// write and feedback
	m_delayLine[m_writeHead] = inSample + output * m_feedback;
	m_writeHead = (m_writeHead + 1) % m_delaySamples;

	// return
	return inSample * m_dryMix + output * m_wetMix;
}

void DelayLine::_process(float * samples, uint32_t count)
{
	assert(m_delayLine != NULL && samples != 0 && count != 0);

	// load temporary local copies of member variables
	float * delayLine = m_delayLine;
	unsigned readHead = m_readHead;
	unsigned writeHead = m_writeHead;
	unsigned delaySamples = m_delaySamples;
	float feedback = m_feedback;
	float dryMix = m_dryMix;
	float wetMix = m_wetMix;
	float * outputSample = samples;

	while (count--)
	{
		float inSample = *outputSample;

		// read
		float output = delayLine[readHead];
		readHead = (readHead + 1) % delaySamples;

		// write and feedback
		delayLine[writeHead] = inSample + output * feedback;
		writeHead = (writeHead + 1) % delaySamples;

		*outputSample++ = inSample * dryMix + output * wetMix;
	}

	// save back local copies
	m_readHead = readHead;
	m_writeHead = writeHead;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
