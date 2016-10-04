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

#include "board.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

uint32_t g_xtal0Freq = 12000000U;
uint32_t g_xtal32Freq = 32768U;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void init_debug_console()
{
    // Set PTB16 and PTB17
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(PORTB, 16, kPORT_MuxAlt3);
    PORT_SetPinMux(PORTB, 17, kPORT_MuxAlt3);
    DbgConsole_Init((uint32_t)UART0, 115200, DEBUG_CONSOLE_DEVICE_TYPE_UART, CLOCK_GetCoreSysClkFreq());
}

void init_board()
{
    // Disable MPU.
    MPU->CESR = 0;
    SIM->SCGC7 &= ~SIM_SCGC7_MPU_MASK;

    BOARD_BootClockHSRUN();

    init_debug_console();

    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);

    // I2C0 and I2C1 pins
    const port_pin_config_t pinConfig = {
        .pullSelect = kPORT_PullDisable,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .openDrainEnable = kPORT_OpenDrainEnable,
        .driveStrength = kPORT_LowDriveStrength,
        .mux = kPORT_MuxAlt2,
    };
    PORT_SetPinConfig(PIN_I2C0_SDA_PORT, PIN_I2C0_SDA_BIT, &pinConfig);
    PORT_SetPinConfig(PIN_I2C0_SCL_PORT, PIN_I2C0_SCL_BIT, &pinConfig);
    PORT_SetPinConfig(PIN_I2C1_SDA_PORT, PIN_I2C1_SDA_BIT, &pinConfig);
    PORT_SetPinConfig(PIN_I2C1_SCL_PORT, PIN_I2C1_SCL_BIT, &pinConfig);

    // SAI pins
    PORT_SetPinMux(PIN_I2S_MCLK_PORT, PIN_I2S_MCLK_BIT, kPORT_MuxAlt6);
    PORT_SetPinMux(PIN_I2S_TXD_PORT, PIN_I2S_TXD_BIT, kPORT_MuxAlt6);
    PORT_SetPinMux(PIN_I2S_TX_WCLK_PORT, PIN_I2S_TX_WCLK_BIT, kPORT_MuxAlt4);
    PORT_SetPinMux(PIN_I2S_TX_BCLK_PORT, PIN_I2S_TX_BCLK_BIT, kPORT_MuxAlt4);
    PORT_SetPinMux(PIN_I2S_RX_WCLK_PORT, PIN_I2S_RX_WCLK_BIT, kPORT_MuxAlt4);
    PORT_SetPinMux(PIN_I2S_RX_BCLK_PORT, PIN_I2S_RX_BCLK_BIT, kPORT_MuxAlt4);
    PORT_SetPinMux(PIN_I2S_RXD_PORT, PIN_I2S_RXD_BIT, kPORT_MuxAlt4);

    // LED pins
    // PTC9 = Red
    // PTE6 = Green
    // PTA11 = Blue
    PORT_SetPinMux(PIN_RED_LED_PORT, PIN_RED_LED_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_GREEN_LED_PORT, PIN_GREEN_LED_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_BLUE_LED_PORT, PIN_BLUE_LED_BIT, kPORT_MuxAsGpio);

    const gpio_pin_config_t gpioOut1 = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1,
    };
    const gpio_pin_config_t gpioOut0 = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0,
    };
    GPIO_PinInit(PIN_RED_LED_GPIO, PIN_RED_LED_BIT, &gpioOut1);
    GPIO_PinInit(PIN_GREEN_LED_GPIO, PIN_GREEN_LED_BIT, &gpioOut1);
    GPIO_PinInit(PIN_BLUE_LED_GPIO, PIN_BLUE_LED_BIT, &gpioOut1);

    // SPI0 pins
    // PTD0 = SPI0_PCS0
    // PTD1 = SPI0_SCK
    // PTD2 = SPI0_SOUT
    // PTD3 = SPI0_SIN
    PORT_SetPinMux(PORTD, 0, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTD, 1, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTD, 2, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTD, 3, kPORT_MuxAlt2);

    // Shift register pins
    // PTB10 = LATCH
    // PTB11 = nOE
    PORT_SetPinMux(PIN_LATCH_PORT, PIN_LATCH_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_nOE_PORT, PIN_nOE_BIT, kPORT_MuxAsGpio);
    GPIO_PinInit(PIN_LATCH_GPIO, PIN_LATCH_BIT, &gpioOut0);
    GPIO_PinInit(PIN_nOE_GPIO, PIN_nOE_BIT, &gpioOut1);

    // Row select pins
    // PTC2 = ROW_A0
    // PTC3 = ROW_A1
    // PTC4 = ROW_A2
    // PTC4 = ROW_EN
    PORT_SetPinMux(PIN_ROW_A0_PORT, PIN_ROW_A0_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_ROW_A1_PORT, PIN_ROW_A1_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_ROW_A2_PORT, PIN_ROW_A2_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_ROW_EN_PORT, PIN_ROW_EN_BIT, kPORT_MuxAsGpio);

    GPIO_PinInit(PIN_ROW_A0_GPIO, PIN_ROW_A0_BIT, &gpioOut0);
    GPIO_PinInit(PIN_ROW_A1_GPIO, PIN_ROW_A1_BIT, &gpioOut0);
    GPIO_PinInit(PIN_ROW_A2_GPIO, PIN_ROW_A2_BIT, &gpioOut0);
    GPIO_PinInit(PIN_ROW_EN_GPIO, PIN_ROW_EN_BIT, &gpioOut0);

    // Buttons
    PORT_SetPinMux(PIN_BTN1_PORT, PIN_BTN1_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_BTN2_PORT, PIN_BTN2_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_WAKEUP_PORT, PIN_WAKEUP_BIT, kPORT_MuxAsGpio);

    const gpio_pin_config_t gpioIn = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0,
    };
    GPIO_PinInit(PIN_BTN1_GPIO, PIN_BTN1_BIT, &gpioIn);
    GPIO_PinInit(PIN_BTN2_GPIO, PIN_BTN2_BIT, &gpioIn);
    GPIO_PinInit(PIN_WAKEUP_GPIO, PIN_WAKEUP_BIT, &gpioIn);

    PORT_SetPinInterruptConfig(PIN_BTN1_PORT, PIN_BTN1_BIT, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(PIN_BTN2_PORT, PIN_BTN2_BIT, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(PIN_WAKEUP_PORT, PIN_WAKEUP_BIT, kPORT_InterruptFallingEdge);

    PORT_EnablePinsDigitalFilter(PIN_BTN1_PORT, PIN_BTN1_BIT, true);
    PORT_EnablePinsDigitalFilter(PIN_BTN2_PORT, PIN_BTN2_BIT, true);
    PORT_EnablePinsDigitalFilter(PIN_WAKEUP_PORT, PIN_WAKEUP_BIT, true);

    // Rotary encoder
    PORT_SetPinMux(PIN_ENCA_PORT, PIN_ENCA_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_ENCB_PORT, PIN_ENCB_BIT, kPORT_MuxAsGpio);
    GPIO_PinInit(PIN_ENCA_GPIO, PIN_ENCA_BIT, &gpioIn);
    GPIO_PinInit(PIN_ENCB_GPIO, PIN_ENCB_BIT, &gpioIn);
    PORT_SetPinInterruptConfig(PIN_ENCA_PORT, PIN_ENCA_BIT, kPORT_InterruptEitherEdge);
    PORT_SetPinInterruptConfig(PIN_ENCB_PORT, PIN_ENCB_BIT, kPORT_InterruptEitherEdge);
    PORT_EnablePinsDigitalFilter(PIN_ENCA_PORT, PIN_ENCA_BIT, true);
    PORT_EnablePinsDigitalFilter(PIN_ENCB_PORT, PIN_ENCB_BIT, true);

    // Set up digital filtering on button pins.
    port_digital_filter_config_t filterConfig = {
        .digitalFilterWidth = 20,
        .clockSource = kPORT_LpoClock,
    };
    PORT_SetDigitalFilterConfig(PIN_BTN1_PORT, &filterConfig);
    PORT_SetDigitalFilterConfig(PIN_WAKEUP_PORT, &filterConfig);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
