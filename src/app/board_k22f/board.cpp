/*
 * Copyright (c) 2016 Chris Reed
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

#include "fsl_port.h"
#include "fsl_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void init_board()
{
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);

    // I2C0 pins
    const port_pin_config_t pinConfig = {
        .pullSelect = kPORT_PullDisable,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .openDrainEnable = kPORT_OpenDrainEnable,
        .driveStrength = kPORT_LowDriveStrength,
        .mux = kPORT_MuxAlt2,
    };
    PORT_SetMultiplePinsConfig(PORTB, (1 << 3)|(1 << 2), &pinConfig);

    // SAI pins
    PORT_SetPinMux(PORTC, 8, kPORT_MuxAlt4);
    PORT_SetPinMux(PORTA, 5, kPORT_MuxAlt6);
    PORT_SetPinMux(PORTA, 12, kPORT_MuxAlt6);
    PORT_SetPinMux(PORTA, 13, kPORT_MuxAlt6);
    PORT_SetPinMux(PORTC, 5, kPORT_MuxAlt4);

    // LED pins
    PORT_SetPinMux(PORTA, 1, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTA, 2, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTD, 5, kPORT_MuxAsGpio);

    const gpio_pin_config_t gpioOut = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0,
    };
    GPIO_PinInit(GPIOA, 1, &gpioOut);
    GPIO_PinInit(GPIOA, 2, &gpioOut);
    GPIO_PinInit(GPIOD, 5, &gpioOut);

    // SPI0 pins for SD card
    // PTC4 = SPI0_PCS0
    // PTD1 = SPI0_SCK
    // PTD2 = SPI0_SOUT
    // PTD3 = SPI0_SIN
    // PTB16 = SD_CARD_DETECT
    PORT_SetPinMux(PORTC, 4, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTD, 1, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTD, 2, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTD, 3, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTB, 16, kPORT_MuxAsGpio);

    const gpio_pin_config_t gpioIn = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0,
    };
    GPIO_PinInit(GPIOB, 16, &gpioIn);

    // A = PTD2
    // B = PTD3
    // red = PTB18 FTM2 CH0
    // blue = PTB19 FTM2 CH1

}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
