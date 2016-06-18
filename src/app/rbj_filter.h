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
#if !defined(_RBJ_FILTER_H_)
#define _RBJ_FILTER_H_

#include "audio_filter.h"
#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

/*!
 * This filter class wraps up and makes it easy to use filter from the
 * cookbook by Robert Bristow-Johnson. The single class CRBJFilter provides
 * all types of filters described in his cookbook (well, all those I've
 * implemented so far).
 *
 * The filter type is set either in the constructor or using the
 * \c SetFilterType() member function. If no filter type is set in the
 * constructor, the filter will default to low pass.
 *
 * Because this class is based on \c CAudioUtilityBase, you must set the
 * sample rate using the member function \c SetSampleRate() before the
 *
 */
class RBJFilter : public AudioFilter
{
public:

	typedef enum {
		kLowPass,	//!< low pass filter
		kHighPass,	//!< high pass filter
		kBandPass,	//!< band pass filter
		kNotch,		//!< notch filter
		kAllPass	//!< second-order allpass filter
	} FilterType;

    RBJFilter(FilterType theType = kLowPass);
    virtual ~RBJFilter() {}

	//! Resets the sample history
	void reset();

	//! Changes the filter type. You must call \c RecomputeCoefficients()
	//! for the change to take effect on filtered samples.
	void set_filter_type(FilterType theType) { mType = theType; }
	FilterType get_filter_type() { return mType; } //!< Returns the current filter type.

	//! Sets the filter frequency. This value is in samples per second, and
	//! should not be higher than the Nyquist frequency, or half the
	//! sampling rate.
	void set_frequency(float f) { frequency = f; }
	float get_frequency() { return frequency; } //!< Returns the current cutoff frequency.

	//! Sets the filter bandwidth. The filter can use either bandwidth or
	//! Q but not both. Setting the bandwidth sets a flag indicating to use
	//! the bandwidth instead of Q..
	//! \sa SetQ()
	void set_bandwidth(float b) { bandwidth = b; useQ = false; }
	float get_bandwidth() { return bandwidth; } //!< Returns the current bandwidth.

	//! Sets the filter Q, or resonance. Setting Q sets the flag saying to use
	//! Q instead of bandwidth.
	//! \sa SetBandwidth()
	void set_q(float theQ) { q = theQ;  useQ = true; }
	float get_q() { return q; } //!< Returns the current Q value.

	//! Recalculates coefficients. Does not reset the sample history, so
	//! the filter parameters can be changed in the middle of processing.
	void recompute_coefficients();

	//! Returns the input sample run through the filter.
	//! \param inputSample An input sample.
	//! \result The filtered sample.
	float tick(float inputSample);

	//! Specifies an array containing values for the filter cutoff frequency
	//! to be used during Process(). This must be called before each call
	//! to Process(), because the value array is reset once it is used.
	void set_frequency_values(float* f, uint32_t count) { freqValues = f; freqValueCount = count; }

protected:
	FilterType mType;
	float frequency, bandwidth, q;
	float omega, xsin, xcos, alpha;
	float b0, b1, b2, a0, a1, a2;
	float in1, in2, out1, out2;
	float b0a0, b1a0, b2a0, a1a0, a2a0;
	float *freqValues;
	uint32_t freqValueCount;
	bool useQ;

    virtual void _process(float * samples, uint32_t count);

};

#endif // _RBJ_FILTER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
