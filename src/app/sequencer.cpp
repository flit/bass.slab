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

#include "sequencer.h"
#include <string.h>
#include <assert.h>
#include <stdlib.h>

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

Sequencer::Sequencer()
:   m_sampleRate(0.0f),
    m_tempo(0.0f),
    m_samplesPerBeat(0.0f),
    m_firstEvent(NULL),
    m_lastEvent(NULL),
    m_freeEvents(NULL),
    m_sequence(NULL),
    m_sequenceLength(0),
    m_sequenceTime(0),
    m_elapsed(0)
{
    memset(m_events, 0, sizeof(m_events));
}

void Sequencer::init()
{
    // Don't try to calculate anything until all required parameters are set.
    if (m_sampleRate == 0.0 || m_tempo == 0.0 || m_sequence == NULL)
    {
        return;
    }

    m_samplesPerBeat = static_cast<uint32_t>(m_sampleRate * 60.0f / m_tempo);

    m_sequenceLength = (uint32_t)strlen(m_sequence);
    m_sequenceTime = m_sequenceLength * m_samplesPerBeat;

    // Put all events on free list.
    free_all();

    // Queue up the sequence.
    enqueue_sequence(0);
}

void Sequencer::set_sample_rate(float rate)
{
    m_sampleRate = rate;
    init();
}

void Sequencer::set_tempo(float tempo)
{
    m_tempo = tempo;
    init();
}

void Sequencer::set_sequence(const char * seq)
{
    m_sequence = seq;
    init();
}

Sequencer::Event Sequencer::get_next_event(uint32_t count)
{
    uint32_t originalElapsed = m_elapsed;

    m_elapsed += count;
    if (m_elapsed > m_sequenceTime)
    {
        originalElapsed = 0;
        m_elapsed = count;
        enqueue_sequence(count);
    }

    Event result(-1);
    Event * ev = m_firstEvent;
    if (!ev)
    {
        return result;
    }

    if (m_elapsed <= ev->m_timestamp)
    {
        return result;
    }

    result = *ev;
    result.m_timestamp -= originalElapsed;

    pop_event();
    push_free_event(ev);

    return result;
}

void Sequencer::free_all()
{
    m_firstEvent = NULL;
    m_lastEvent = NULL;
    m_elapsed = 0;

    int i;
    m_freeEvents = &m_events[0];
    for (i = 0; i < kMaxEvents - 1; ++i)
    {
        m_events[i].m_next = &m_events[i + 1];
    }
    m_events[i].m_next = NULL;
}

void Sequencer::enqueue_sequence(uint32_t startOffset)
{
    uint32_t timestamp = startOffset;
    int i;
    int32_t note;
    for (i = 0; i < m_sequenceLength; ++i)
    {
        char c = m_sequence[i];
        Event ev(timestamp);

        switch (c)
        {
            case 'x':
                ev.m_event = kTriggerEvent;
                break;
            case 's':
            {
                ev.m_event = kNoteStartEvent;
                note = 0;
                while ((i + 1) < m_sequenceLength)
                {
                    c = m_sequence[i + 1];
                    if (c >= '0' && c <= '9')
                    {
                        note = (note * 10) + (c - '0');
                    }
                    else
                    {
                        break;
                    }
                    ++i;
                }
                ev.m_value = note;
                break;
            }
            case 'p':
                ev.m_event = kNoteStopEvent;
                break;
            default:
                continue;
        }

        append_event(ev);

        timestamp += m_samplesPerBeat;
    }
}

void Sequencer::append_event(const Event & event)
{
    Event * ev = pop_free_event();
    if (ev)
    {
        *ev = event;
        append_event(ev);
    }
}

Sequencer::Event * Sequencer::pop_free_event()
{
    assert(m_freeEvents);
    Event * ev = m_freeEvents;
    m_freeEvents = ev->m_next;
    ev->m_next = NULL;
    return ev;
}

void Sequencer::push_free_event(Event * ev)
{
    ev->m_next = m_freeEvents;
    m_freeEvents = ev;
}

Sequencer::Event * Sequencer::pop_event()
{
    Event * ev = m_firstEvent;
    if (ev)
    {
        m_firstEvent = ev->m_next;
        ev->m_next = NULL;
        if (m_lastEvent == ev)
        {
            m_lastEvent = NULL;
        }
    }
    return ev;
}

void Sequencer::append_event(Event * ev)
{
    if (m_lastEvent)
    {
        m_lastEvent->m_next = ev;
    }
    ev->m_next = NULL;
    m_lastEvent = ev;
    if (!m_firstEvent)
    {
        m_firstEvent = ev;
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
