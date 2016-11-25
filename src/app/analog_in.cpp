/*
 * Copyright (c) 2016 Immo Software
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

#include "analog_in.h"
#include "board.h"

using namespace slab;

bool AnalogIn::s_adcInited[] = {0};

Ar::TypedChannel<uint32_t> g_adc0Result("adc0");
Ar::TypedChannel<uint32_t> g_adc1Result("adc1");

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

AnalogIn::AnalogIn(uint32_t instance, uint32_t channel)
:   m_instance(instance),
    m_channel(channel)
{
    if (instance == 0)
    {
        m_base = ADC0;
    }
    else if (instance == 1)
    {
        m_base = ADC1;
    }
}

void AnalogIn::init()
{
    if (!s_adcInited[m_instance])
    {
        adc16_config_t config;
        ADC16_GetDefaultConfig(&config);
        config.resolution = kADC16_Resolution12or13Bit;
        config.longSampleMode = kADC16_LongSampleCycle24;
        ADC16_Init(m_base, &config);
        ADC16_DoAutoCalibration(m_base);
        ADC16_SetHardwareAverage(m_base, kADC16_HardwareAverageCount16);
        EnableIRQ(m_instance == 0 ? ADC0_IRQn : ADC1_IRQn);

        s_adcInited[m_instance] = true;
    }
}

extern "C" void ADC0_IRQHandler()
{
    uint32_t result = ADC16_GetChannelConversionValue(ADC0, 0);
    g_adc0Result.send(result, kArNoTimeout);
}

extern "C" void ADC1_IRQHandler()
{
    uint32_t result = ADC16_GetChannelConversionValue(ADC1, 0);
    g_adc1Result.send(result, kArNoTimeout);
}

uint32_t AnalogIn::read()
{
    adc16_channel_config_t channelConfig;
    channelConfig.channelNumber = m_channel;
    channelConfig.enableInterruptOnConversionCompleted = true;
    channelConfig.enableDifferentialConversion = false;

    ADC16_SetChannelConfig(m_base, 0, &channelConfig);

    uint32_t result;
    if (m_instance == 0)
    {
        result = g_adc0Result.receive();
    }
    else if (m_instance = 1)
    {
        result = g_adc1Result.receive();
    }
    return result;
}


//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
