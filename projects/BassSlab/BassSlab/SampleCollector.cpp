//
//  SampleCollector.cpp
//  BassSlab
//
//  Created by Chris Reed on 11/25/16.
//  Copyright © 2016 Immo Software. All rights reserved.
//

#include "SampleCollector.h"
#include <algorithm>
#include <string>

using namespace slab;

SampleCollector::SampleCollector()
:   m_samples(NULL),
    m_length(0),
    m_first(0),
    m_count(0)
{
}

SampleCollector::~SampleCollector()
{
    if (m_samples)
    {
        delete [] m_samples;
    }
}

void SampleCollector::init(uint32_t length)
{
    m_samples = new float[length];
    m_length = length;
    m_first = 0;
    m_count = 0;
}

void SampleCollector::clear()
{
    m_first = 0;
    m_count = 0;
}

float SampleCollector::operator [] (uint32_t index)
{
    index = std::min(index, m_count);
    uint32_t offset = (m_first + index) % m_length;
    return m_samples[offset];
}

void SampleCollector::append(const AudioBuffer & buf)
{
    uint32_t count = buf.get_count();
    uint32_t last = (m_first + m_count) % m_length;
    uint32_t firstHalf = std::min(count, m_length - last);
    uint32_t secondHalf = count - firstHalf;

    // Copy first half.
    std::memcpy(m_samples + last, buf.get_buffer(), firstHalf * sizeof(float));

    // Copy wrapped.
    if (secondHalf)
    {
        std::memcpy(m_samples, buf.get_buffer() + firstHalf, secondHalf * sizeof(float));
    }

    uint32_t newLast = (last + count) % m_length;
    if (m_count >= m_length && newLast > m_first)
    {
        m_first = newLast;
    }

    if (m_count < m_length)
    {
        m_count = std::min(m_count + count, m_length);
    }
}

