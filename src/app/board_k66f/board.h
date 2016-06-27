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
#if !defined(__BOARD_H__)
#define __BOARD_H__

#include "fsl_device_registers.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

// SDSPI microSD interface
#define BOARD_SDSPI_SPI_BASE SPI0_BASE
#define BOARD_SDSPI_SPI_PCS_NUMBER 0

// Device I2C ports and addresses
#define BOARD_FXOS_I2C_BASE (I2C0)
#define BOARD_FXOS_I2C_ADDR (0x1d)

#define BOARD_GYRO_I2C_BASE (I2C0)
#define BOARD_GYRO_I2C_ADDR (0x21)

#define BOARD_CODEC_I2C_BASE (I2C1)
#define BOARD_CODEC_I2C_ADDR (0x1a)

#define BOARD_PROX_I2C_BASE (I2C1)
#define BOARD_PROX_I2C_ADDR (0x39)

// I2C0 (accelerometer and gyro)

// I2C0 SDA
#define PIN_I2C0_SDA_PORT   (PORTD)
#define PIN_I2C0_SDA_GPIO   (GPIOD)
#define PIN_I2C0_SDA_BIT    (8)
#define PIN_I2C0_SDA        (1 << PIN_I2C0_SDA_BIT)

// I2C0 SCL
#define PIN_I2C0_SCL_PORT   (PORTD)
#define PIN_I2C0_SCL_GPIO   (GPIOD)
#define PIN_I2C0_SCL_BIT    (9)
#define PIN_I2C0_SCL        (1 << PIN_I2C0_SCL_BIT)

// I2C1 (audio codec and proximity sensor)

// I2C1 SDA
#define PIN_I2C1_SDA_PORT   (PORTC)
#define PIN_I2C1_SDA_GPIO   (GPIOC)
#define PIN_I2C1_SDA_BIT    (11)
#define PIN_I2C1_SDA        (1 << PIN_I2C1_SDA_BIT)

// I2C1 SCL
#define PIN_I2C1_SCL_PORT   (PORTC)
#define PIN_I2C1_SCL_GPIO   (GPIOC)
#define PIN_I2C1_SCL_BIT    (10)
#define PIN_I2C1_SCL        (1 << PIN_I2C1_SCL_BIT)

// RGB LED
#define PIN_RED_LED_PORT    (PORTC)
#define PIN_RED_LED_GPIO    (GPIOC)
#define PIN_RED_LED_BIT     (9)
#define PIN_RED_LED         (1 << PIN_RED_LED_BIT)

#define PIN_GREEN_LED_PORT  (PORTE)
#define PIN_GREEN_LED_GPIO  (GPIOE)
#define PIN_GREEN_LED_BIT   (6)
#define PIN_GREEN_LED       (1 << PIN_GREEN_LED_BIT)

#define PIN_BLUE_LED_PORT   (PORTA)
#define PIN_BLUE_LED_GPIO   (GPIOA)
#define PIN_BLUE_LED_BIT    (11)
#define PIN_BLUE_LED        (1 << PIN_BLUE_LED_BIT)

// Display
#define PIN_LATCH_PORT      (PORTB)
#define PIN_LATCH_GPIO      (GPIOB)
#define PIN_LATCH_BIT       (10)
#define PIN_LATCH           (1 << PIN_LATCH_BIT)

#define PIN_nOE_PORT        (PORTB)
#define PIN_nOE_GPIO        (GPIOB)
#define PIN_nOE_BIT         (11)
#define PIN_nOE             (1 << PIN_nOE_BIT)

#define PIN_ROW_A0_PORT     (PORTC)
#define PIN_ROW_A0_GPIO     (GPIOC)
#define PIN_ROW_A0_BIT      (2)
#define PIN_ROW_A0          (1 << PIN_ROW_A0_BIT)

#define PIN_ROW_A1_PORT     (PORTC)
#define PIN_ROW_A1_GPIO     (GPIOC)
#define PIN_ROW_A1_BIT      (3)
#define PIN_ROW_A1          (1 << PIN_ROW_A1_BIT)

#define PIN_ROW_A2_PORT     (PORTC)
#define PIN_ROW_A2_GPIO     (GPIOC)
#define PIN_ROW_A2_BIT      (4)
#define PIN_ROW_A2          (1 << PIN_ROW_A2_BIT)

#define PIN_ROW_EN_PORT     (PORTC)
#define PIN_ROW_EN_GPIO     (GPIOC)
#define PIN_ROW_EN_BIT      (5)
#define PIN_ROW_EN          (1 << PIN_ROW_EN_BIT)

// Proximity Interrupt
#define PIN_INT_PORT        (PORTC)
#define PIN_INT_GPIO        (GPIOC)
#define PIN_INT_BIT         (0)
#define PIN_INT             (1 << PIN_INT_BIT)

// Rotary Encoder
// A = FTM1_QD_PHA
#define PIN_ENCA_PORT       (PORTA)
#define PIN_ENCA_GPIO       (GPIOA)
#define PIN_ENCA_BIT        (9)
#define PIN_ENCA            (1 << PIN_ENCA_BIT)

// B = FTM1_QD_PHB
#define PIN_ENCB_PORT       (PORTA)
#define PIN_ENCB_GPIO       (GPIOA)
#define PIN_ENCB_BIT        (8)
#define PIN_ENCB            (1 << PIN_ENCB_BIT)

// Buttons
#define PIN_BTN1_PORT       (PORTD)
#define PIN_BTN1_GPIO       (GPIOD)
#define PIN_BTN1_BIT        (13)
#define PIN_BTN1            (1 << PIN_BTN1_BIT)

#define PIN_BTN2_PORT       (PORTD)
#define PIN_BTN2_GPIO       (GPIOD)
#define PIN_BTN2_BIT        (12)
#define PIN_BTN2            (1 << PIN_BTN2_BIT)

// Wakeup
#define PIN_WAKEUP_PORT     (PORTA)
#define PIN_WAKEUP_GPIO     (GPIOA)
#define PIN_WAKEUP_BIT      (4)
#define PIN_WAKEUP          (1 << PIN_WAKEUP_BIT)

// SAI
#define PIN_I2S_MCLK_PORT   (PORTC)
#define PIN_I2S_MCLK_GPIO   (GPIOC)
#define PIN_I2S_MCLK_BIT    (6)
#define PIN_I2S_MCLK        (1 << PIN_I2S_MCLK_BIT)

#define PIN_I2S_TX_WCLK_PORT   (PORTE)
#define PIN_I2S_TX_WCLK_GPIO   (GPIOE)
#define PIN_I2S_TX_WCLK_BIT    (11)
#define PIN_I2S_TX_WCLK        (1 << PIN_I2S_TX_WCLK_BIT)

#define PIN_I2S_TX_BCLK_PORT   (PORTE)
#define PIN_I2S_TX_BCLK_GPIO   (GPIOE)
#define PIN_I2S_TX_BCLK_BIT    (12)
#define PIN_I2S_TX_BCLK        (1 << PIN_I2S_TX_BCLK_BIT)

#define PIN_I2S_TXD_PORT   (PORTC)
#define PIN_I2S_TXD_GPIO   (GPIOC)
#define PIN_I2S_TXD_BIT    (1)
#define PIN_I2S_TXD        (1 << PIN_I2S_TXD_BIT)

#define PIN_I2S_RX_WCLK_PORT   (PORTE)
#define PIN_I2S_RX_WCLK_GPIO   (GPIOE)
#define PIN_I2S_RX_WCLK_BIT    (8)
#define PIN_I2S_RX_WCLK        (1 << PIN_I2S_RX_WCLK_BIT)

#define PIN_I2S_RX_BCLK_PORT   (PORTE)
#define PIN_I2S_RX_BCLK_GPIO   (GPIOE)
#define PIN_I2S_RX_BCLK_BIT    (9)
#define PIN_I2S_RX_BCLK        (1 << PIN_I2S_RX_BCLK_BIT)

#define PIN_I2S_RXD_PORT   (PORTE)
#define PIN_I2S_RXD_GPIO   (GPIOE)
#define PIN_I2S_RXD_BIT    (7)
#define PIN_I2S_RXD        (1 << PIN_I2S_RXD_BIT)


// ADC inputs

// Hood POT1 = ADC0_SE17 (PTE24)
#define POT1_ADC            (0)
#define POT1_CHANNEL        (17)

// Hood POT2 = ADC0_SE18 (PTE25)
#define POT2_ADC            (0)
#define POT2_CHANNEL        (18)

// Shield Tempo = ADC1_SE12 (PTB6)
#define TEMPO_ADC           (1)
#define TEMPO_CHANNEL       (12)

// Shield Tune = ADC1_SE13 (PTB7)
#define TUNE_ADC            (1)
#define TUNE_CHANNEL        (13)

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void init_debug_console();
void init_board();

#endif // __BOARD_H__
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
