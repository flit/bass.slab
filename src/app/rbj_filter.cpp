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

#include "rbj_filter.h"
#include "arm_math.h"
#include <string.h>
#include <assert.h>
#include <math.h>

const float LN2 = 0.6931471805599453f;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

RBJFilter::RBJFilter(FilterType theType)
:   AudioFilter(),
	mType(theType),
	frequency(0),
	bandwidth(0),
	q(0),
	in1(0),
	in2(0),
	out1(0),
	out2(0),
	freqValues(0),
	freqValueCount(0),
	useQ(true)
{
}

void RBJFilter::reset()
{
	in1 = in2 = out1 = out2 = 0.0f;
}

float RBJFilter::tick(float inputSample)
{
	float filteredSample = b0a0*inputSample + b1a0*in1 + b2a0*in2 - a1a0*out1 - a2a0*out2;

	// update sample history
	in2 = in1;
	in1 = inputSample;
	out2 = out1;
	out1 = filteredSample;

	return filteredSample;
}

void RBJFilter::_process(float * samples, uint32_t count)
{
	int n = count;
	float* inCursor = samples;
	float* outCursor = samples;

	float l_in1 = in1;
	float l_in2 = in2;
	float l_out1 = out1;
	float l_out2 = out2;

	float l_b0a0 = b0a0;
	float l_b1a0 = b1a0;
	float l_b2a0 = b2a0;
	float l_a1a0 = a1a0;
	float l_a2a0 = a2a0;

	unsigned n4 = n / 4;
	unsigned r4 = n % 4;

// 	if (freqValues)
// 	{
// 		assert(freqValueCount == count);
//
// 		while (n4--)
// 		{
// 			// update cutoff, but only every 8 samples (cheat, cheat!)
// 			frequency = *freqValues;//++;
// 			recompute_coefficients(); // yuck, inline this sometime? or optimise somehow..
//
// 			l_b0a0 = b0a0;
// 			l_b1a0 = b1a0;
// 			l_b2a0 = b2a0;
// 			l_a1a0 = a1a0;
// 			l_a2a0 = a2a0;
//
// 			freqValues+=4;
//
// 			float inputSample1 = *inCursor++;
// 			float inputSample2 = *inCursor++;
// 			float inputSample3 = *inCursor++;
// 			float inputSample4 = *inCursor++;
//
// 			float filteredSample1 = l_b0a0*inputSample1 + l_b1a0*l_in1 + l_b2a0*l_in2 - l_a1a0*l_out1 - l_a2a0*l_out2;
//
// 			float filteredSample2 = l_b0a0*inputSample2 + l_b1a0*inputSample1 + l_b2a0*l_in1 - l_a1a0*filteredSample1 - l_a2a0*l_out1;
//
// 			float filteredSample3 = l_b0a0*inputSample3 + l_b1a0*inputSample2 + l_b2a0*inputSample1 - l_a1a0*filteredSample2 - l_a2a0*filteredSample1;
//
// 			float filteredSample4 = l_b0a0*inputSample4 + l_b1a0*inputSample3 + l_b2a0*inputSample2 - l_a1a0*filteredSample3 - l_a2a0*filteredSample2;
//
// 			// update sample history
// 			l_in2 = inputSample3;
// 			l_in1 = inputSample4;
// 			l_out2 = filteredSample3;
// 			l_out1 = filteredSample4;
//
// 			*outCursor++ = filteredSample1;
// 			*outCursor++ = filteredSample2;
// 			*outCursor++ = filteredSample3;
// 			*outCursor++ = filteredSample4;
// 		}
//
// 		// update cutoff one last time for the remainder
// 		if (r4)
// 		{
// 			frequency = *freqValues;
// 			recompute_coefficients(); // yuck, inline this sometime? or optimise somehow..
//
// 			l_b0a0 = b0a0;
// 			l_b1a0 = b1a0;
// 			l_b2a0 = b2a0;
// 			l_a1a0 = a1a0;
// 			l_a2a0 = a2a0;
// 		}
//
// 		while (r4--)
// 		{
// 			float inputSample = *inCursor++;
//
// 			float filteredSample = l_b0a0*inputSample + l_b1a0*l_in1 + l_b2a0*l_in2 - l_a1a0*l_out1 - l_a2a0*l_out2;
//
// 			// update sample history
// 			l_in2 = l_in1;
// 			l_in1 = inputSample;
// 			l_out2 = l_out1;
// 			l_out1 = filteredSample;
//
// 			*outCursor++ = filteredSample;
// 		}
// 	}
// 	else
// 	{
		while (n4--)
		{
			float inputSample1 = *inCursor++;
			float inputSample2 = *inCursor++;
			float inputSample3 = *inCursor++;
			float inputSample4 = *inCursor++;

			float filteredSample1 = l_b0a0*inputSample1 + l_b1a0*l_in1 + l_b2a0*l_in2 - l_a1a0*l_out1 - l_a2a0*l_out2;

			float filteredSample2 = l_b0a0*inputSample2 + l_b1a0*inputSample1 + l_b2a0*l_in1 - l_a1a0*filteredSample1 - l_a2a0*l_out1;

			float filteredSample3 = l_b0a0*inputSample3 + l_b1a0*inputSample2 + l_b2a0*inputSample1 - l_a1a0*filteredSample2 - l_a2a0*filteredSample1;

			float filteredSample4 = l_b0a0*inputSample4 + l_b1a0*inputSample3 + l_b2a0*inputSample2 - l_a1a0*filteredSample3 - l_a2a0*filteredSample2;

			// update sample history
			l_in2 = inputSample3;
			l_in1 = inputSample4;
			l_out2 = filteredSample3;
			l_out1 = filteredSample4;

			*outCursor++ = filteredSample1;
			*outCursor++ = filteredSample2;
			*outCursor++ = filteredSample3;
			*outCursor++ = filteredSample4;
		}

		while (r4--)
		{
			float inputSample = *inCursor++;

			float filteredSample = l_b0a0*inputSample + l_b1a0*l_in1 + l_b2a0*l_in2 - l_a1a0*l_out1 - l_a2a0*l_out2;

			// update sample history
			l_in2 = l_in1;
			l_in1 = inputSample;
			l_out2 = l_out1;
			l_out1 = filteredSample;

			*outCursor++ = filteredSample;
		}
// 	}

	// save back out sample history
	in1 = l_in1;
	in2 = l_in2;
	out1 = l_out1;
	out2 = l_out2;

	// reset cutoff samples
// 	freqValues = 0;
}

void RBJFilter::recompute_coefficients()
{
	// compute intermediates
	omega = 2.0f * PI * frequency / get_sample_rate();
	arm_sin_cos_f32(omega, &xsin, &xcos);
// 	xsin = sinf(omega);
// 	xcos = cosf(omega);
	if (useQ)
	{
		alpha = xsin / (2.0f * q);
	}
	else
	{
		alpha = xsin * sinhf( LN2 * 0.5f * bandwidth * omega / xsin );
	}

	switch (mType)
	{
		case kLowPass:
			b0 = (1.0f - xcos) * 0.5f;
			b1 = 1.0f - xcos;
			b2 = (1.0f - xcos) * 0.5f;
			a0 = 1.0f + alpha;
			a1 = -2.0f * xcos;
			a2 = 1.0f - alpha;
			break;

		case kHighPass:
			b0 = (1.0f + xcos) * 0.5f;
			b1 = -(1.0f + xcos);
			b2 = (1.0f + xcos) * 0.5f;
			a0 = 1.0f + alpha;
			a1 = -2.0f * xcos;
			a2 = 1.0f - alpha;
			break;

		case kBandPass:
			b0 = xsin * 0.5f;
			b1 = 0.0f;
			b2 = -xsin * 0.5f;
			a0 = 1.0f + alpha;
			a1 = -2.0f * xcos;
			a2 = 1.0f - alpha;
			break;

		case kNotch:
			b0 =   1.0f;
            b1 =  -2.0f * xcos;
            b2 =   1.0f;
            a0 =   1.0f + alpha;
            a1 =  -2.0f * xcos;
            a2 =   1.0f - alpha;
			break;

		case kAllPass:
			b0 =   1.0f - alpha;
            b1 =  -2.0f * xcos;
            b2 =   1.0f + alpha;
            a0 =   1.0f + alpha;
            a1 =  -2.0f * xcos;
            a2 =   1.0f - alpha;
			break;
	}

	// precompute some divisions
	b0a0 = b0/a0;
	b1a0 = b1/a0;
	b2a0 = b2/a0;
	a1a0 = a1/a0;
	a2a0 = a2/a0;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
