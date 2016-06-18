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
#if !defined(_SEQUENCER_H_)
#define _SEQUENCER_H_

#include <stdint.h>
#include <stddef.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

/*!
 * @brief Audio event sequencer.
 *
 * Tracks trigger events within a musical sequence. The only events supported are simple triggers
 * and note start and stop. Note numbers or values are not supported.
 *
 * Uses a simple sequence chart in string format. Each character of the string represents
 * one beat. A character corresponding to one of the event types causes that event to fire for that
 * beat. Any other characters are ignored.
 *
 * The character to event table:
 * - 'x' : Simple trigger event
 * - 's' : Note start event
 * - 'p' : Note stop event
 *
 * An example two bar percussion sequence might be: "x---x-x-"
 *
 * For note sequences, it is normal to use a character like '>' or '.' between the start and stop
 * events. Example: '--s>>>>p'
 */
class Sequencer
{
public:

    enum event_type
    {
        kInvalidEvent,
        kTriggerEvent,
        kNoteStartEvent,
        kNoteStopEvent
    };

    /*!
     *
     */
    struct Event
    {
        Event * m_next;
        int32_t m_timestamp;
        event_type m_event;

        Event(int32_t timestamp=0) : m_next(NULL), m_timestamp(timestamp), m_event(kInvalidEvent) {}
    };

    Sequencer();
    ~Sequencer() {}

    void set_sample_rate(float rate) { m_sampleRate = rate; }
    void set_tempo(float tempo) { m_tempo = tempo; }
    void set_sequence(const char * seq) { m_sequence = seq; }

    uint32_t get_samples_per_beat() { return m_samplesPerBeat; }

    void init();

    Event get_next_event(uint32_t count);

protected:

    enum
    {
        kMaxEvents = 32,
    };

    float m_sampleRate;
    float m_tempo;
    uint32_t m_samplesPerBeat;
    Event m_events[kMaxEvents];
    Event * m_firstEvent;
    Event * m_lastEvent;
    Event * m_freeEvents;
    const char * m_sequence; //!< The sequence chart as a string.
    uint32_t m_sequenceLength; //!< Number of chars, and therefore beats, of the sequence.
    uint32_t m_sequenceTime; //!< Length of time in samples of the entire sequence.
    uint32_t m_elapsed;

    void enqueue_sequence(uint32_t startOffset);

    Event * pop_free_event();
    void push_free_event(Event * ev);

    Event * pop_event();
    void append_event(Event * ev);
};


#endif // _SEQUENCER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
