/*
 * Copyright (c) 2015-2016 Chris Reed
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

#include "argon/argon.h"
#include "board.h"
#include "analog_in.h"
#include "audio_output.h"
#include "audio_output_converter.h"
#include "audio_filter.h"
#include "audio_ramp.h"
#include "asr_envelope.h"
#include "sequencer.h"
#include "sequence_reader.h"
#include "audio_mixer.h"
#include "delay_line.h"
#include "rbj_filter.h"
#include "display.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_fxos.h"
#include "arm_math.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

#define OVER_SAMPLE_RATE (384U)
#define BUFFER_SIZE (256)
#define CHANNEL_NUM (2)
#define BUFFER_NUM (2)

template <typename T>
inline T abs(T a)
{
    return (a > 0) ? a : -a;
}

template <typename T>
inline T max3(T a, T b, T c)
{
    T tmp = (a > b) ? a : b;
    return (tmp > c) ? tmp : c;
}

class SineGenerator : public AudioFilter
{
public:
    SineGenerator()
    :   m_delta(0),
        m_sinFreq(0),
        m_phase(0),
        m_env(),
        m_previous(0),
        m_seq(NULL)
    {
    }

    void set_freq(float freq) { m_sinFreq = freq; }
    void set_sequence(Sequencer * seq) { m_seq = seq; }
    void enable_sustain(bool enable) { m_env.enable_sustain(enable); }
    void set_attack(float seconds) { m_env.set_length_in_seconds(ASREnvelope::kAttack, seconds); }
    void set_release(float seconds) { m_env.set_length_in_seconds(ASREnvelope::kRelease, seconds); }
    void init();

    virtual void process(float * samples, uint32_t count);

protected:
    float m_delta;
    float m_sinFreq; // 80 Hz
    float m_phase;
    ASREnvelope m_env;
    float m_previous;
    Sequencer * m_seq;
};

class SquareGenerator : public AudioFilter
{
public:
    SquareGenerator()
    :   m_freq(0),
        m_halfPhaseWidth(0),
        m_phase(0),
        m_env(),
        m_previous(0),
        m_seq(NULL)
    {
    }

    void set_freq(float freq) { m_freq = freq; }
    void set_sequence(Sequencer * seq) { m_seq = seq; }
    void enable_sustain(bool enable) { m_env.enable_sustain(enable); }
    void set_attack(float seconds) { m_env.set_length_in_seconds(ASREnvelope::kAttack, seconds); }
    void set_release(float seconds) { m_env.set_length_in_seconds(ASREnvelope::kRelease, seconds); }
    void init();

    virtual void process(float * samples, uint32_t count);

protected:
    float m_freq; // 80 Hz
    float m_halfPhaseWidth;
    float m_phase;
    ASREnvelope m_env;
    float m_previous;
    Sequencer * m_seq;
};

class RotaryDecoder
{
public:
    RotaryDecoder();

    int decode(uint8_t a, uint8_t b);

private:
    static const int s_encoderStates[];
    uint8_t m_oldState;
};

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void accel_thread(void * arg);
void pots_thread(void * arg);
void encoder_debounce(ar_timer_t * timer, void * arg);
void init_audio_out();
void init_fs();
void init_board();

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

const int RotaryDecoder::s_encoderStates[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

uint32_t g_xtal0Freq = 8000000U;
uint32_t g_xtal32Freq = 32768U;

float g_audioBuf[BUFFER_SIZE];
float g_mixBuf[BUFFER_SIZE];
int16_t g_outBuf[BUFFER_NUM][BUFFER_SIZE * CHANNEL_NUM];

const float kSampleRate = 32000.0f; // 32kHz

AudioOutput g_audioOut;
AudioOutputConverter g_audioOutConverter;
SineGenerator g_kickGen;
Sequencer g_kickSeq;
SineGenerator g_bassGen;
Sequencer g_bassSeq;
// SineGenerator g_tickGen;
// Sequencer g_tickSeq;
AudioMixer g_mixer;
RBJFilter g_filter;
DelayLine g_delay;
i2c_master_handle_t g_i2cHandle;
fxos_handle_t g_fxos;

SequenceInfo * g_firstSequence = 0;
uint32_t g_sequenceCount = 0;

Ar::ThreadWithStack<1024> g_accelThread("accel", accel_thread, 0, 120, kArSuspendThread);
Ar::ThreadWithStack<1024> g_potsThread("pots", pots_thread, 0, 120, kArSuspendThread);

DisplayController g_display;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

RotaryDecoder::RotaryDecoder()
:   m_oldState(0)
{
}

int RotaryDecoder::decode(uint8_t a, uint8_t b)
{
    uint8_t newState = ((b & 1) << 1) | (a & 1);

    m_oldState = ((m_oldState & 3) << 2) | newState;

    return s_encoderStates[m_oldState];

    // First, find the newEncoderState. This'll be a 2-bit value
    // the msb is the state of the B pin. The lsb is the state
    // of the A pin on the encoder.
//     newEncoderState = (digitalRead(bPin)<<1) | (digitalRead(aPin));

    // Now we pair oldEncoderState with new encoder state
    // First we need to shift oldEncoder state left two bits.
    // This'll put the last state in bits 2 and 3.
//     oldEncoderState <<= 2;

    // Mask out everything in oldEncoderState except for the previous state
//     oldEncoderState &= 0xC0;

    // Now add the newEncoderState. oldEncoderState will now be of
    // the form: 0b0000(old B)(old A)(new B)(new A)
//     oldEncoderState |= newEncoderState; // add filteredport value
}

// void encoder_debounce(ar_timer_t * timer, void * arg)
// {
//     int a = g_a;
//     int b = g_b;
// //     printf("encoder: a=%d, b=%d\r\n", a, b);
//
//     int delta = g_decoder.decode(a, b);
//     g_value += delta;
//     if (g_value < 0)
//     {
//         g_value = 0;
//     }
//     else if (g_value > 100)
//     {
//         g_value = 100;
//     }
// }

void my_timer_fn(Ar::Timer * t, void * arg)
{
    GPIO_TogglePinsOutput(PIN_BLUE_LED_GPIO, PIN_BLUE_LED_BIT);
}
Ar::Timer g_myTimer("mine", my_timer_fn, 0, kArPeriodicTimer, 1500);

void SineGenerator::init()
{
    m_delta = 2.0f * PI * (m_sinFreq / m_sampleRate);

    m_env.set_sample_rate(get_sample_rate());
    m_env.set_curve_type(ASREnvelope::kAttack, AudioRamp::kLinear);
    m_env.set_curve_type(ASREnvelope::kRelease, AudioRamp::kLinear);
    m_env.set_peak(1.0f);
}

void SineGenerator::process(float * samples, uint32_t count)
{
    // Check for a trigger.
    Sequencer::Event triggerEvent = m_seq->get_next_event(count);
    int32_t triggerSample = triggerEvent.m_timestamp;
    if (triggerEvent.m_event == Sequencer::kNoteStopEvent)
    {
        m_env.set_release_offset(triggerSample);
        triggerSample = -1;
    }
    bool needsRestartOnZeroCrossing = false;
    float previous = m_previous;
    float * sample = samples;
    int i;
    for (i = 0; i < count; ++i)
    {
        // Detect trigger point.
        if (triggerSample == i)
        {
            needsRestartOnZeroCrossing = true;
        }

        float f = arm_sin_f32(m_phase);
        float v = f * m_env.next();
        *sample++ = v;

        // After triggered, restart on a zero crossing to prevent popping.
        bool zeroCrossing = ((previous >= 0.0f && v <= 0.0f) || (previous <= 0.0f && v >= 0.0f));
        if (needsRestartOnZeroCrossing && zeroCrossing)
        {
            m_env.trigger();
            m_phase = 0.0f;
            needsRestartOnZeroCrossing = false;
        }

        m_phase += m_delta;
        if (m_phase >= 2.0f * PI)
        {
            m_phase = 0.0f;
        }

        previous = v;
    }

    m_previous = previous;
}

void SquareGenerator::init()
{
    m_halfPhaseWidth = m_sampleRate / m_freq / 2.0f;

    m_env.set_sample_rate(get_sample_rate());
    m_env.set_curve_type(ASREnvelope::kAttack, AudioRamp::kLinear);
    m_env.set_curve_type(ASREnvelope::kRelease, AudioRamp::kLinear);
    m_env.set_peak(1.0f);
}

void SquareGenerator::process(float * samples, uint32_t count)
{
    // Check for a trigger.
    Sequencer::Event triggerEvent = m_seq->get_next_event(count);
    int32_t triggerSample = triggerEvent.m_timestamp;
    if (triggerEvent.m_event == Sequencer::kNoteStopEvent)
    {
        m_env.set_release_offset(triggerSample);
        triggerSample = -1;
    }
    bool needsRestartOnZeroCrossing = false;
    float previous = m_previous;
    float * sample = samples;
    int i;
    for (i = 0; i < count; ++i)
    {
        // Detect trigger point.
        if (triggerSample == i)
        {
            needsRestartOnZeroCrossing = true;
        }

        float f = 1.0f;
        if (m_phase > m_halfPhaseWidth)
        {
            f = -1.0f;
        }
        float v = f * m_env.next();
        *sample++ = v;

        // After triggered, restart on a zero crossing to prevent popping.
        bool zeroCrossing = ((previous >= 0.0f && v <= 0.0f) || (previous <= 0.0f && v >= 0.0f));
        if (needsRestartOnZeroCrossing && zeroCrossing)
        {
            m_env.trigger();
            m_phase = 0.0f;
            needsRestartOnZeroCrossing = false;
        }

        m_phase += 1.0f;
        if (m_phase >= m_halfPhaseWidth * 2.0f)
        {
            m_phase = 0.0f;
        }

        previous = v;
    }

    m_previous = previous;
}

// template <typename T, int N>
// class RingBuffer
// {
// public:
//     RingBuffer()
//     :   m_head(0),
//         m_tail(0),
//         m_count(0)
//     {
//         memset(m_buffer, 0, sizeof(m_buffer));
//     }
//
//     void insert(T value)
//     {
//
//     }
//
// protected:
//     T m_buffer[N];
//     uint32_t m_head;
//     uint32_t m_tail;
//     uint32_t m_count;
// };

void accel_thread(void * arg)
{
    memset(&g_fxos, 0, sizeof(g_fxos));
    g_fxos.base = I2C0;
    g_fxos.i2cHandle = &g_i2cHandle;
    g_fxos.xfer.slaveAddress = 0x1c;
    FXOS_Init(&g_fxos);

    const int kHistoryCount = 50;
    float history[kHistoryCount];
    uint32_t head = 0;
    uint32_t count = 0;

    while (1)
    {
        fxos_data_t data;
        status_t status = FXOS_ReadSensorData(&g_fxos, &data);
        if (status == kStatus_Success)
        {
//             printf("acc[x=%6d y=%6d z=%6d] mag[x=%6d y=%6d z=%6d]\r\n",
//                    data.accelX, data.accelY, data.accelZ,
//                    data.magX, data.magY, data.magZ);

            // Get maximum accel in positive or negative.
//             data.accelZ -= 16384; // subtract out gravity (device must be stationary).
            const float kGScale = 16384.0f;
            float xG = float(data.accelX) / kGScale;
            float yG = float(data.accelY) / kGScale;
            float zG = float(data.accelZ) / kGScale;
            zG -= 1.0f;
            float maxAccel = max3(abs(xG), abs(yG), abs(zG));
//             int16_t maxAccel = MAX(abs(data.accelX), MAX(abs(data.accelY), abs(data.accelZ)));
//             int16_t minAccel = MIN(data.accelX, MIN(data.accelY, data.accelZ));
//             if (abs(minAccel) > abs(maxAccel))
//             {
//                 maxAccel = minAccel;
//             }

            history[head] = maxAccel;
            head = (head + 1) % kHistoryCount;
            if (count < kHistoryCount)
            {
                ++count;
            }

            float sum = 0;
            float average;
            uint32_t i = head;
            uint32_t j = 0;
            for (; j < count; ++j)
            {
                sum += history[i];
                i = (i + 1) % kHistoryCount;
            }
            average = sum / float(kHistoryCount);

//             float g = float(average) / 16384.0f;
            float feedback = average; // / 2.0f;
            g_delay.set_feedback(feedback);
        }

        Ar::Thread::sleep(20);
    }
}

void pots_thread(void * arg)
{
    AnalogIn pot1(POT1_ADC, POT1_CHANNEL);
    AnalogIn pot2(POT2_ADC, POT2_CHANNEL);
    pot1.init();
    pot2.init();

    while (1)
    {
        uint32_t value1 = pot1.read();
        uint32_t value2 = pot2.read();
        printf("pot1 = %d; pot2 = %d\n", value1, value2);

        Ar::Thread::sleep(200);
    }
}

void init_audio_out()
{
    // Configure the audio format
    sai_transfer_format_t format;
    format.bitWidth = kSAI_WordWidth16bits;
    format.channel = 0U;
    format.sampleRate_Hz = kSAI_SampleRate32KHz;
    format.masterClockHz = OVER_SAMPLE_RATE * format.sampleRate_Hz;
    format.protocol = kSAI_BusLeftJustified;
    format.stereo = kSAI_Stereo;
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;

    // Configure Sgtl5000 I2C
    uint32_t i2cSourceClock = CLOCK_GetFreq(kCLOCK_BusClk);
    i2c_master_config_t i2cConfig = {0};
    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(I2C0, &i2cConfig, i2cSourceClock);
    I2C_MasterTransferCreateHandle(I2C0, &g_i2cHandle, NULL, NULL);

    g_audioOut.init(&format, I2C0, &g_i2cHandle);
//     g_audioOut.dump_sgtl5000();

    AudioOutput::Buffer buf;
    buf.dataSize = BUFFER_SIZE * CHANNEL_NUM * sizeof(int16_t);
    buf.data = (uint8_t *)&g_outBuf[0][0];
    g_audioOut.add_buffer(&buf);
    buf.data = (uint8_t *)&g_outBuf[1][0];
    g_audioOut.add_buffer(&buf);

    g_audioOut.set_source(&g_audioOutConverter);
    AudioBuffer audioBuf(&g_audioBuf[0], BUFFER_SIZE);
    g_audioOutConverter.set_buffer(audioBuf);
    g_audioOutConverter.set_source(&g_mixer);

    g_kickSeq.set_sample_rate(kSampleRate);
    g_kickSeq.set_tempo(100.0f);
//     g_kickSeq.set_sequence("x---x---x---x-x-x---x---x---x---xx--x--x--xxx-x-");
    g_kickSeq.set_sequence("x---"); //x-x----xx---");
    g_kickSeq.init();

    g_kickGen.set_sample_rate(kSampleRate);
    g_kickGen.set_sequence(&g_kickSeq);
    g_kickGen.set_freq(50.0f);
    g_kickGen.enable_sustain(false);
    g_kickGen.init();
    g_kickGen.set_attack(0.01f);
    g_kickGen.set_release(0.6f);

    g_bassSeq.set_sample_rate(kSampleRate);
    g_bassSeq.set_tempo(100.0f);
    g_bassSeq.set_sequence("--sp"); //"--s>>>p-----s>>>>>>p----");
    g_bassSeq.init();

    g_bassGen.set_sample_rate(kSampleRate);
    g_bassGen.set_sequence(&g_bassSeq);
    g_bassGen.set_freq(40.0f);
    g_bassGen.enable_sustain(true);
    g_bassGen.init();
    g_bassGen.set_attack(0.3f);
    g_bassGen.set_release(1.0f);

//     g_tickSeq.set_sample_rate(kSampleRate);
//     g_tickSeq.set_tempo(100.0f);
//     g_tickSeq.set_sequence("----x-----x-");
//     g_tickSeq.init();
//
//     g_tickGen.set_sample_rate(kSampleRate);
//     g_tickGen.set_sequence(&g_tickSeq);
//     g_tickGen.set_freq(4000.0f);
//     g_tickGen.enable_sustain(false);
//     g_tickGen.init();
//     g_tickGen.set_attack(0.04f);
//     g_tickGen.set_release(0.3f);

    g_filter.set_sample_rate(kSampleRate);
    g_filter.set_frequency(120.0f);
    g_filter.set_q(0.4f);
    g_filter.recompute_coefficients();
    g_filter.set_input(&g_bassGen);

    g_delay.set_sample_rate(kSampleRate);
    g_delay.set_maximum_delay_seconds(0.4f);
    g_delay.set_delay_samples(g_kickSeq.get_samples_per_beat());
    g_delay.set_feedback(0.5f);
    g_delay.set_wet_mix(0.5f);
    g_delay.set_dry_mix(0.8f);
    g_delay.set_input(&g_kickGen);

    AudioBuffer mixBuf(&g_mixBuf[0], BUFFER_SIZE);
    g_mixer.set_buffer(mixBuf);
    g_mixer.set_input_count(2);
    g_mixer.set_input(0, &g_delay, 0.5f);
    g_mixer.set_input(1, &g_bassGen, 0.5f);
//     g_mixer.set_input(2, &g_tickGen, 0.3f);
}

void init_fs()
{
    SequenceReader * reader = new SequenceReader();
    assert(reader);
    reader->init();
    g_sequenceCount = reader->scan_dir("/", &g_firstSequence);

    if (g_sequenceCount == 0)
    {
        printf("No sequences were found on SD card.\n");
        return;
    }

    // Set sequence to the first.
    printf("Setting tempo to %d bpm\n", g_firstSequence->tempo);
    g_kickSeq.set_tempo(g_firstSequence->tempo);
    g_bassSeq.set_tempo(g_firstSequence->tempo);

    printf("Sequence #0: %s\n", g_firstSequence->channels[0]);
    g_kickSeq.set_sequence(g_firstSequence->channels[0]);

    printf("Sequence #1: %s\n", g_firstSequence->channels[1]);
    g_kickSeq.set_sequence(g_firstSequence->channels[1]);
}

int main(void)
{
    printf("Hello...\r\n");

    init_board();
    init_audio_out();
//     init_fs();
    g_display.init();

    g_myTimer.start();
//     g_audioOut.start();
//     g_accelThread.resume();
//     g_potsThread.resume();
    g_display.start();

    ar_kernel_run();

//     Ar::Thread::getCurrent()->suspend();
//     while (1)
//     {
//     }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
