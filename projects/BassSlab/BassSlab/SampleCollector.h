//
//  SampleCollector.hpp
//  BassSlab
//
//  Created by Chris Reed on 11/25/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#ifndef SampleCollector_hpp
#define SampleCollector_hpp

#include <stdint.h>
#include "audio_buffer.h"

namespace slab {

/*!
 *
 */
class SampleCollector
{
public:
    SampleCollector();
    ~SampleCollector();

    void init(uint32_t length);

    void clear();
    void append(const AudioBuffer & buf);

    float * get() { return m_samples; }
    const float * get() const { return m_samples; }
    uint32_t get_length() const { return m_length; }
    uint32_t get_count() const { return m_count; }

    float operator [] (uint32_t index);

private:
    float *m_samples;   //!< Array of samples.
    uint32_t m_length;  //!< Total size of m_samples array.
    uint32_t m_first;   //!< Index of first sample in array.
    uint32_t m_count;   //!< Number of samples currently in array.
};

}

#endif /* SampleCollector_hpp */
