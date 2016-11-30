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
#include "osc.h"
#include "sequencer.h"
#include "sequence_file_reader.h"
#include "audio_mixer.h"
#include "delay_line.h"
#include "rbj_filter.h"
#include "display.h"
#include "rotary_decoder.h"
#include "pin_irq_manager.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_fxos.h"
#include "fsl_i2c.h"
#include "Dialog7212.h"
#include "arm_math.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

using namespace slab;

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
void audio_init_thread(void * arg);
void init_thread(void * arg);

void init_i2c0();
void init_i2c1();
void init_audio_out();
void init_audio_synth();
void init_fs();
void init_board();

void button_handler(PORT_Type * port, uint32_t pin, void * userData);
void rotary_handler(PORT_Type * port, uint32_t pin, void * userData);

void my_timer_fn(Ar::Timer * t, void * arg);
void encoder_debounce(Ar::Timer * timer, void * arg);

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

float g_audioBuf[BUFFER_SIZE];
float g_mixBuf[BUFFER_SIZE];
int16_t g_outBuf[BUFFER_NUM][BUFFER_SIZE * CHANNEL_NUM];

const float kSampleRate = 32000.0f; // 32kHz

AudioOutput g_audioOut;
AudioOutputConverter g_audioOutConverter;
Oscillator g_kickGen;
Sequencer g_kickSeq;
Oscillator g_bassGen;
Sequencer g_bassSeq;
AudioMixer g_mixer;
RBJFilter g_filter;
DelayLine g_delay;
i2c_master_handle_t g_i2cHandle;
i2c_master_handle_t g_i2c1Handle;
fxos_handle_t g_fxos;

SequenceInfo * g_firstSequence = 0;
uint32_t g_sequenceCount = 0;

Ar::Thread * g_audioInitThread = NULL;
Ar::Thread * g_initThread = NULL;

Ar::ThreadWithStack<2048> g_accelThread("accel", accel_thread, 0, 120, kArSuspendThread);
Ar::ThreadWithStack<2048> g_potsThread("pots", pots_thread, 0, 120, kArSuspendThread);

DisplayController g_display;
RotaryDecoder g_rotary;

Ar::Timer g_myTimer("mine", my_timer_fn, 0, kArPeriodicTimer, 1500);
Ar::Timer g_rotaryTimer("mine", encoder_debounce, 0, kArOneShotTimer, 50);

uint32_t g_value = 0;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void my_timer_fn(Ar::Timer * t, void * arg)
{
    GPIO_TogglePinsOutput(PIN_BLUE_LED_GPIO, PIN_BLUE_LED_BIT);
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
    g_fxos.xfer.slaveAddress = BOARD_FXOS_I2C_ADDR;
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
//             printf("feedback = %g\r\n", feedback);
            g_delay.set_feedback(feedback);
        }

        Ar::Thread::sleep(200);
    }
}

void pots_thread(void * arg)
{
    AnalogIn tunePot(TUNE_ADC, TUNE_CHANNEL);
    AnalogIn pot1(POT1_ADC, POT1_CHANNEL);
    AnalogIn pot2(POT2_ADC, POT2_CHANNEL);
    tunePot.init();
    pot1.init();
    pot2.init();

    uint32_t lastTune = ~0;
    uint32_t lastValue1 = ~0;
    uint32_t lastValue2 = ~0;
    while (1)
    {
        uint32_t tuneValue = tunePot.read();
        uint32_t value1 = pot1.read();
        uint32_t value2 = pot2.read();
        if (tuneValue != lastTune || value1 != lastValue1 || value2 != lastValue2)
        {
            lastTune = tuneValue;
        	lastValue1 = value1;
        	lastValue2 = value2;

        	g_display.set_char(0, '0' + (char)(value1 * 10 / 4096));
        	g_display.set_char(1, '0' + (char)(value2 * 10 / 4096));

//         	printf("tune = %d; pot1 = %d; pot2 = %d\r\n", tuneValue, value1, value2);
        }

        Ar::Thread::sleep(200);
    }
}

void button_handler(PORT_Type * port, uint32_t pin, void * userData)
{
    printf("button %d pressed\n", pin);
}

void encoder_debounce(Ar::Timer * timer, void * arg)
{
    int a = GPIO_ReadPinInput(PIN_ENCA_GPIO, PIN_ENCA_BIT);
    int b = GPIO_ReadPinInput(PIN_ENCB_GPIO, PIN_ENCB_BIT);
    printf("encoder: a=%d, b=%d\r\n", a, b);

    int delta = g_rotary.decode(a, b);
    g_value += delta;
    if (g_value < 0)
    {
        g_value = 0;
    }
    else if (g_value > 100)
    {
        g_value = 100;
    }
    printf("g_value = %d\r\n", g_value);
}

void rotary_handler(PORT_Type * port, uint32_t pin, void * userData)
{
    g_rotaryTimer.start();
}

void init_i2c0()
{
    uint32_t i2cSourceClock = CLOCK_GetFreq(kCLOCK_BusClk);
    i2c_master_config_t i2cConfig = {0};
    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(I2C0, &i2cConfig, i2cSourceClock);
    I2C_MasterTransferCreateHandle(I2C0, &g_i2cHandle, NULL, NULL);
}

void init_i2c1()
{
    uint32_t i2cSourceClock = CLOCK_GetFreq(kCLOCK_BusClk);
    i2c_master_config_t i2cConfig = {0};
    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(I2C1, &i2cConfig, i2cSourceClock);
    I2C_MasterTransferCreateHandle(I2C1, &g_i2c1Handle, NULL, NULL);
}

void audio_init_thread(void * arg)
{
    // Configure the audio format
    sai_transfer_format_t format;
    format.bitWidth = kSAI_WordWidth16bits;
    format.channel = 0U;
    format.sampleRate_Hz = kSAI_SampleRate32KHz;
    format.masterClockHz = OVER_SAMPLE_RATE * format.sampleRate_Hz;
    format.protocol = kSAI_BusI2S;
    format.stereo = kSAI_Stereo;
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;

    g_audioOut.init(&format);

    // Configure audio codec
    DA7212_InitCodec(BOARD_CODEC_I2C_BASE);
    DA7212_ChangeFrequency(DA7212_SYS_FS_32K);
    DA7212_ChangeInput(DA7212_Input_MIC1_Dig);

    init_audio_synth();

//     delete g_audioInitThread;
}

void init_audio_out()
{
    g_audioInitThread = new Ar::Thread("auinit", audio_init_thread, 0, NULL, 1500, 200, kArStartThread);
}

void init_audio_synth()
{
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
    g_kickSeq.set_sequence("x---------x---------"); //x-x----xx---");
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
    g_bassSeq.set_sequence("--s>>>>p--------"); //"--s>>>p-----s>>>>>>p----");
    g_bassSeq.init();

    g_bassGen.set_sample_rate(kSampleRate);
    g_bassGen.set_sequence(&g_bassSeq);
    g_bassGen.set_freq(80.0f);
    g_bassGen.enable_sustain(true);
    g_bassGen.init();
    g_bassGen.set_attack(0.3f);
    g_bassGen.set_release(3.0f);

    g_filter.set_sample_rate(kSampleRate);
    g_filter.set_frequency(120.0f);
    g_filter.set_q(0.4f);
    g_filter.recompute_coefficients();
    g_filter.set_input(&g_bassGen);

    g_delay.set_sample_rate(kSampleRate);
    g_delay.set_maximum_delay_seconds(0.4f);
    g_delay.set_delay_samples(g_kickSeq.get_samples_per_beat());
    g_delay.set_feedback(0.7f);
    g_delay.set_wet_mix(0.5f);
    g_delay.set_dry_mix(0.8f);
    g_delay.set_input(&g_kickGen);

    AudioBuffer mixBuf(&g_mixBuf[0], BUFFER_SIZE);
    g_mixer.set_buffer(mixBuf);
    g_mixer.set_input_count(2);
    g_mixer.set_input(0, &g_delay, 0.5f);
    g_mixer.set_input(1, &g_bassGen, 0.34f);
//     g_mixer.set_input(2, &g_tickGen, 0.3f);
}

void init_fs()
{
    SequenceFileReader reader;
    reader.init();
    g_sequenceCount = reader.scan_dir("/", &g_firstSequence);

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

void test_prox()
{
    uint8_t data;
    i2c_master_transfer_t xfer = {0};
    xfer.slaveAddress = 0x39;
    xfer.direction = kI2C_Read;
    xfer.subaddress = 0x80 | 0x12;
    xfer.subaddressSize = 1;
    xfer.data = &data;
    xfer.dataSize = sizeof(data);

    status_t result = I2C_MasterTransferBlocking(BOARD_PROX_I2C_BASE, &xfer);
    printf("result = %d; data = 0x%02x\r\n", result, data);
}

void init_thread(void * arg)
{
    init_board();

    printf("Hello...\r\n");

    init_i2c0();
    init_i2c1();
    init_audio_out();
//     init_fs();
    g_display.init();

    test_prox();

    PinIrqManager::get().connect(PIN_BTN1_PORT, PIN_BTN1_BIT, button_handler, NULL);
    PinIrqManager::get().connect(PIN_BTN2_PORT, PIN_BTN2_BIT, button_handler, NULL);
    PinIrqManager::get().connect(PIN_WAKEUP_PORT, PIN_WAKEUP_BIT, button_handler, NULL);

    PinIrqManager::get().connect(PIN_ENCA_PORT, PIN_ENCA_BIT, rotary_handler, NULL);
    PinIrqManager::get().connect(PIN_ENCB_PORT, PIN_ENCB_BIT, rotary_handler, NULL);

    g_myTimer.start();
    g_audioOut.start();
    g_accelThread.resume();
    g_potsThread.resume();
    g_display.start();

//     delete g_initThread;
}

int main(void)
{
    g_initThread = new Ar::Thread("init", init_thread, 0, NULL, 2500, 60, kArStartThread);

    ar_kernel_run();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
