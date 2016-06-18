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

/*	***************************************************************************
 *  Author: Michael Brodsky
 *
 *	Overview:
 *
 *	The reverberator is based on a design proposed by Gardner (1992) that
 *	greatly reduces the metallic characteristics of earlier designs by
 *	Schroeder (1961) and Moorer (1979) while using the same structures present
 *	in those designs.
 *
 *	Gardner’s generalized reverberator design suggested a set of cascaded
 *	allpass filters with a comb feedback loop containing a lowpass filter. Each
 *	of the allpass filters could be cascaded or nested and output taps were
 *	taken between each allpass section:
 *
 *
 *                            .---------------->[+]--------------->[+]-----o (Y)
 *                            |                  ^                  ^
 *                            |                  |                  |
 *                            |                  |                  |
 *                            .                  .                  .
 *                           / \                / \                / \
 *                          /   \              /   \              /   \
 *                         / a0  \            / a1  \            / a2  \
 *                        '-------'          '-------'          '-------'
 *                            |                  |                  |
 *                            |                  |                  |
 *	            .--------.    |    .--------.    |    .--------.    |
 *	            |        |    |    |        |    |    |        |    |
 *	(X) o--[+]--|   AP   |----o----|   AP   |----o----|   AP   |----o
 *	        ^   |        |         |        |         |        |    |
 *	        |   '--------'         '--------'         '--------'    |
 *          |                                                       |
 *          |                                                       |
 *          |                     .         .--------.              |
 *          |               . <   |         |        |              |
 *          '-------------<   g   |---------|   LP   |--------------'
 *                          ' <   |         |        |
 *                                '         '--------'
 *
 *	Gardner developed several designs incorporating allpass, nested allpass
 *	and delay filters with different parameters to simulate various room
 *	sizes. He attributed the sound improvements due to the feedback loop,
 *	which essentially forms a comb filter and introduces a slight delay before
 *	summing the reverberated signal with the input signal. The lowpass filter
 *	simulates air absorption of higher frequencies.
 *
 *	This design extends Gardner's original proposal by substituting each
 *	allpass (AP) stage with two differently-tuned allpass filters, a feedback-
 *	delay and low-pass filter arranged into a `ring' structure similar to
 *	Gardner's `feedback loop'. The unprocessed (dry) input signal is summed
 *	with the scaled output of each stage. Output taps are taken between each
 *	stage, summed and scaled; this becomes the processed (wet) signal which is
 *	summed with the dry input signal at the reveberator's output.
 *
 *
 *	Structure Block Diagrams:
 *
 *	These diagrams represent Multiply-Accumulate (MAC) IIR/FIR filter
 *	structures in the form generally found in the DSP literature. Summing
 *	nodes are indicated by [+] and gains (multiplies) by [x] and labeled
 *	accordingly. Delay lines are indicated by M
 *
 *
 *	Allpass Filter:
 *                                        g
 *                    .---------->[x]------------------------.
 *	                  |                                      |
 *	                  |        ---------                     |
 *	x[n]              |       |         |                    |
 *	o------>(+)-------o------>|    M    |-------o--->[x]--->(+)------>o y[n]
 *	         ^                |         |       |       B
 *	         |                 ---------        |
 *	         |                                  |
 *	         '--------------------[x]<----------'
 *                                       -g
 *
 *	Note: this structure is sometimes referred to as a `Recursive Comb Filter'
 *	in the literature. To ensure filter stability, |g| < 1.
 *
 *	y[n] = gx[n] + Bx[n-M] - gy[n-M]
 *
 *
 *	Feedback Delay Line:
 *
 *                   .------------------------------------------->o y[n]
 *	                 |
 *	                 |        ---------
 *	x[n]             |       |         |
 *	o------>(+)------o------>|    M    |------
 *	         ^               |         |      |
 *	         |                ---------       |
 *	         |                                |
 *	         '--------------------[x]---------'
 *                                       g
 *
 *	y[n] = x[n] + gy[n-M]
 *
 *
 *	Low Pass Filter:
 *
 *	Low pass filters are implemented as DFI IIR Biquad sections, where:
 *
 *	y[n] = b0x[n] + b1x[n-1] + b2x[n-2] - (a1y[n-1] + a2y[n-2])
 *
 *
 *	Reverberator Stage:
 *
 *	Each reverberator `stage' is built from two cascaded allpass filters, a
 *	feedback delay line and lowpass filter, with an output tap and input
 *	summing node:
 *
 *                                                    .--------------->o v
 *	                                                  |
 *	        .----.     .----.     .-----.     .----.  |
 *	x o---->| AP |---->| AP |---->|Delay|---->| LP |--o--[x]--->[+]--->o y
 *	        '----'     '----'     '-----'     '----'        Rt   ^
 *	                                                             |
 *	u o----------------------------------------------------------'
 *
 *	where:
 *
 *		 x: stage input,
 *		 y: stage output,
 *		 u: input (dry) summing node,
 *		 v: output (wet) tap.
 *		Rt: reverb time (see below),
 *
 *
 *	Reverberator Structure:
 *
 *	The implementation uses a total of four (4) stages arranged in a ring
 *	topology, and an output summing node that scales and sums the wet/dry
 *	signal:
 *
 *	      .---------------------------------------------------------------------------------------------------[+]--->o Y
 *	      |                                                                                                    ^
 *	      |                  V0                      V1                     V2                     V3          |
 *        |                 .-------------------->[+]------------------->[+]------------------->[+]------[x]---'
 *	      |                 |                      ^                      ^                      ^          Mix
 *	      |                 |                      |                      |                      |
 *	      |         .----.  |              .----.  |              .----.  |              .----.  |
 *	X o---o-->[+]-->| S0 |--o--[x]-->[+]-->| S1 |--o--[x]-->[+]-->| S2 |--o--[x]-->[+]-->| S3 |--o--[x]---.
 *        |    ^    '----'        Rt  ^    '----'        Rt  ^    '----'        Rt  ^    '----'        Rt |
 *	      |    | U0                   | U1                   | U2                   | U3                  |
 *        |    |                      |                      |                      |                     |
 *	      '----+----------------------o----------------------o----------------------'                     |
 *	           |                                                                                          |
 *	           |                                                                                          |
 *	           '---------------------------------------------<--------------------------------------------'
 *	                                                    Feedback Loop
 *	where:
 *
 *		  X: reverberator input,
 *		  Y: reverberator output,
 *		U,V: correspond to the `u' and `v' stage nodes (see `Reverberator Stage', above),
 *		Mix: wet/dry scaling factor.
 *
 *
 *	Theory of Operation and Implementation:
 *
 *	Natural reverberation is a psychoacoustic effect created by multiple
 *	reflections of sound from surfaces, such as the walls in a room, concert
 *	hall or a cave. Natural reverberation generally follows a sort of
 *	exponential curve, beginning with the original sound, a set of early
 *	reflections, followed by a set of late reflections that decay to silence.
 *	The reverberation time, Rt60, is the time it takes for the reverberated
 *	signal to decay 60 dB below the level of the original signal. The echoes
 *	become thicker and more diffuse in the reverb tail and eventually die away.
 *	In order for an artificial reverberator to sound natural, it must simulate
 *	the impulse response of a reverberant environment, while creating
 *	sufficient echo density without adversely coloring the sound.
 *
 *	The early Schroeder and Moorer designs used cascaded, nested or parallel
 *	combinations of comb and allpass filters to create echoes of varying time
 *	and summed together. One main drawback to the Schroeder and Moorer designs
 *	is that they exhibit a metallic or hollow, “tube-like” sound which is
 *	difficult, if not impossible to remove. This is because the dry signal was
 *	only passed through the structures once and tended to created regularly
 *	spaced echoes resulting in evenly spaced resonances accross the spectrum,
 *	standing waves and other undesirable qualities. It was found
 *	experimentally that these artifacts persisted accross the entire domain of
 *	possible parameters and topologies.
 *
 *	Gardner's design eliminated these artifacts by continually recycling the
 *	signal through the structure and summing the outputs between stages. By
 *	choosing proper parameters, the individual echoes become thicker and almost
 *	randomly spaced as they die away, which is a characteristic of natural
 *	reverberation.
 *
 *	The goal here is to create a natural sounding reverberation effect
 *	suitable for use with electric guitar and to simulate the tonal qualities
 *	of reverberators found in commercial guitar amplifiers. The critical
 *	factors are the delay times introduced by the allpass/delay filters, their
 *	phase response and the lowpass filter cutoff frequencies.
 *
 *	The amount of delay introduced by the AP/Delay sections determines the
 *	`room size' and the Rt scaling factor determines the `decay time' (i.e.
 *	how `live' or reflective the room is. The LP filter cutoff frequencies
 *	determine how much high frequency energy is damped. Application
 *	requirements dictate that all parameters shall be constant with the
 *	exception of the `Mix' parameter - this is because guitar amplifiers
 *	almost always have just a reverb `mix' or `level' knob and nothing else.
 *	The other parameters should be determined by manual tuning (listening
 *	tests).
 *
 *	Because Gardner's general design uses the same structures as the earlier
 *	Schroeder and Moorer designs, many of the parameters can be taken from
 *	those designs and tuned for specific responses. Here, the default settings
 *	from one channel of `Freeverb' (an open-source Schroeder reverberator)
 *	were used as a starting point and manually tuned to achieve the desired
 *	result. This supports the theory that structure topology is as important
 *	a factor as the individual structures themselves when seeking a natural
 *	sounding reverberator.
 *
 *	Allpass and Feedback Delay Line delay times can be chosen somewhat
 *	arbitrarily except that their lengths (M parameter) should be mutually
 *	prime (have no common factors). This prevents undesireable resonances from
 *	appearing in the wet signal.
 *
 *	20141030 - A highpass filter section was added in series with the wet
 *	signal after V3 (see `Reverberator Structure', above). This damped the low
 *	end response, eliminating undesireable characteristics when the amplifier
 *	is operated at high gain (overdrive) settings.
 *
 *	The highpass filter is implemented as a DFI IIR Biquad section with a
 *	cutoff frequency of 150 Hz.
 */

/*	***************************************************************************
 *
 *	Allpass Filter Section Parameters:
 *
 *	There are a total of eight (8) allpass filters, 2 per stage and four (4)
 *	stages, designated as AP0-AP7.
 */

/* Delay sizes (M), in signal flow order: AP0, AP1, ... AP7. */
const unsigned int M1[] =
{
	1557,
	1617,
	1491,
	1422,
	1277,
	1356,
	1188,
	1116
};

/* `g' coefficient. */
const float g1 = 0.92f;

/* `B' coefficient. */
const float B = 1.0f;

/*	***************************************************************************
 *
 *	Feedback Delay Filter Section Parameters:
 *
 *	There are a total of four (4) feedback delay line filters, 1 per stage and
 *	four (4) stages, designated as FD0-FD3.
 */

/* Delay sizes (M), in signal flow order: FD0, FD1, FD2, FD3. */
const unsigned int M2[] =
{
	556,
	441,
	341,
	225
};

/* `g' coefficient. */
const float g2 = 0.2f;

/*	***************************************************************************
 *
 *	Other Parameters:
 */

/* Reverb Time, Rt. */
const float rt = 0.92f;

/* Lowpass filter cutoff frequency in Hertz (Hz). */
const float lpf = 5000.0f;

/* Absolute minimum and maximum `Mix' scaling factors. */
const float mixMin = 0;
const float mixMax = 0.0625f; /* Anything larger saturates the mix. */

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

template <int N>
class Delay
{
public:
    Delay()
    :   m_count(0),
        m_head(0)
    {
        memset(m_delay, 0, sizeof(m_delay));
    }

    float push(float sample)
    {
        float result = m_delay[m_head];
        m_delay[++m_head] = sample;
        return result;
    }

protected:
    float m_delay[N];
    uint32_t m_count;
    uint32_t m_head;

};

/*
 *	Allpass Filter:
 *                                        g
 *                    .---------->[x]------------------------.
 *	                  |                                      |
 *	                  |        ---------                     |
 *	x[n]              |       |         |                    |
 *	o------>(+)-------o------>|    M    |-------o--->[x]--->(+)------>o y[n]
 *	         ^                |         |       |       B
 *	         |                 ---------        |
 *	         |                                  |
 *	         '--------------------[x]<----------'
 *                                       -g
 *
 *	Note: this structure is sometimes referred to as a `Recursive Comb Filter'
 *	in the literature. To ensure filter stability, |g| < 1.
 *
 *	y[n] = gx[n] + Bx[n-M] - gy[n-M]
 */
template <int D>
class Allpass
{
public:
    Allpass()
    :   m_last(0)
    {
    }

    void process(float * inSamples, float * outSamples, uint32_t count)
    {
        float * i = inSamples;
        float * o = outSamples;
        float f = m_last;
        uint32_t n;
        for (n = 0; n < count; ++n)
        {
            float i_n = i[n];
            float m = m_delay.push(i_n);
            o[n] = g1 * i_n + B * m - g1 * m;
        }
        m_last = f;
    }

protected:
    Delay<D> m_delay;
    float m_last;

};

/*
 *	Feedback Delay Line:
 *
 *                   .------------------------------------------->o y[n]
 *	                 |
 *	                 |        ---------
 *	x[n]             |       |         |
 *	o------>(+)------o------>|    M    |------
 *	         ^               |         |      |
 *	         |                ---------       |
 *	         |                                |
 *	         '--------------------[x]---------'
 *                                       g
 *
 *	y[n] = x[n] + gy[n-M]
 */
template <int N>
class FeedbackDelay
{
public:
    FeedbackDelay()
    :   m_last(0)
    {
    }

    void process(float * inSamples, float * outSamples, uint32_t count)
    {
        uint32_t n;
        float f = m_last;
        for (n = 0; n < count; ++n)
        {
            float m = inSamples[n] + f;
            f = m_delay.push(m) * g2;
            outSamples[n] = m;
        }
        m_last = f;
    }

protected:
    Delay<N> m_delay;
    float m_last;
};

Allpass<1557> g_ap0;
Allpass<1617> g_ap1;
Allpass<1491> g_ap2;
Allpass<1422> g_ap3;
Allpass<1277> g_ap4;
Allpass<1356> g_ap5;
Allpass<1188> g_ap6;
Allpass<1116> g_ap7;

ReverbFilter::ReverbFilter(FilterType theType)
:   AudioFilter()
{
}

void ReverbFilter::_process(float * samples, uint32_t count)
{
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
