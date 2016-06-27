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
#include "sine_osc.h"
#include "square_osc.h"
#include "sequencer.h"
#include "sequence_reader.h"
#include "audio_mixer.h"
#include "delay_line.h"
#include "rbj_filter.h"
#include "display.h"
#include "rotary_decoder.h"
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

Ar::Thread * g_initThread = NULL;

Ar::ThreadWithStack<1024> g_accelThread("accel", accel_thread, 0, 120, kArSuspendThread);
Ar::ThreadWithStack<1024> g_potsThread("pots", pots_thread, 0, 120, kArSuspendThread);

DisplayController g_display;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void my_timer_fn(Ar::Timer * t, void * arg)
{
    GPIO_TogglePinsOutput(PIN_BLUE_LED_GPIO, PIN_BLUE_LED_BIT);
}
Ar::Timer g_myTimer("mine", my_timer_fn, 0, kArPeriodicTimer, 1500);

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

void init_thread(void * arg)
{
    init_board();
    init_audio_out();
//     init_fs();
    g_display.init();

    g_myTimer.start();
//     g_audioOut.start();
//     g_accelThread.resume();
//     g_potsThread.resume();
    g_display.start();

    delete g_initThread;
}

int main(void)
{
    init_debug_console();
    printf("Hello...\r\n");

    g_initThread = new Ar::Thread("init", init_thread, 0, NULL, 1500, 200, kArStartThread);

    ar_kernel_run();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
