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
#if !defined(_DELAY_LINE_H_)
#define _DELAY_LINE_H_

#include "audio_filter.h"
#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

/*!
 * @brief Audio delay line.
 */
class DelayLine : public AudioFilter
{
public:

	//! Default constructor. You must set the maximum delay time in either
	//! samples or seconds before using the delay. Any calls to \c Tick()
	//! before doing this will simply return 0.0.
	DelayLine();

	//! This constructor is equivalent to setting the maximum delay time
	//! using \c SetMaximum_delaySamples().
	DelayLine(unsigned delaySamples);

	//! Disposes of the delay line.
	virtual ~DelayLine();

	void set_maximum_delay_samples(unsigned delaySamples);
	void set_maximum_delay_seconds(float delaySeconds);

	void set_delay_samples(unsigned delaySamples);
	void set_delay_seconds(float delaySeconds);

	void set_feedback(float feedback) { m_feedback = feedback; }

	void set_dry_mix(float mix) { m_dryMix = mix; }
	void set_wet_mix(float mix) { m_wetMix = mix; }

	void reset();

	//! Puts a sample into the delay line and return the delayed output.
	float tick(float inSample);

protected:
	float * m_delayLine;
	unsigned m_maxDelaySamples;
	unsigned m_delaySamples;
	unsigned m_readHead;
	unsigned m_writeHead;
	float m_feedback;
	float m_dryMix;
	float m_wetMix;

    virtual void _process(float * samples, uint32_t count);

};

#endif // _DELAY_LINE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
