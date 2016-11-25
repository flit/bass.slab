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

#include "display.h"
#include "board.h"
#include "font_5x7.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_tpm.h"

using namespace slab;

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

DisplayController * DisplayController::s_instance = NULL;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

DisplayController::DisplayController()
{
    memset(m_buffer, 0, sizeof(m_buffer));
    s_instance = this;
}

void DisplayController::init()
{
    // Create semaphore.
    m_transferDoneSem.init("txdone", 0);
    m_nextRowSem.init("nextrow", 0);

    // Init SPI.
    dspi_master_config_t masterConfig;
    masterConfig.ctarConfig.baudRate = 4000000; // 4 MHz
    masterConfig.ctarConfig.bitsPerFrame = 16;
    masterConfig.whichPcs = kDSPI_Pcs1;
    DSPI_MasterGetDefaultConfig(&masterConfig);
    DSPI_MasterInit(SPI0, &masterConfig, CLOCK_GetBusClkFreq());

    DSPI_MasterTransferCreateHandle(SPI0, &m_spiHandle, dspi_master_transfer_callback, this);

    // Init timer.
    CLOCK_SetTpmClock(0x3); // MCGIRCLK = 1MHz
    tpm_config_t tpm;
    TPM_GetDefaultConfig(&tpm);
    TPM_Init(TPM1, &tpm);
    // @ 60 Hz = 16.6 ms/frame = 2.4 ms/row, @ 120 Hz = 1.2 ms/row
    TPM1->MOD = 1200;
    TPM_ClearStatusFlags(TPM1, kTPM_TimeOverflowFlag);
    TPM_EnableInterrupts(TPM1, kTPM_TimeOverflowInterruptEnable);
    NVIC_EnableIRQ(TPM1_IRQn);
}

void DisplayController::start()
{
    memset(m_buffer, 0, sizeof(m_buffer));
    m_buffer[0][0] = 0x0202;
    m_buffer[0][1] = 0x0404;

    m_activeBuffer = 0;
    m_flipBuffers = false;
    m_row = 0;
    set_output_enable(false);
    set_row_enable(false);
    select_row(m_row);

    DSPI_Enable(SPI0, true);
    DSPI_StartTransfer(SPI0);

    TPM_StartTimer(TPM1, kTPM_SystemClock);

    // Send first row.
    transfer_row();
}

//
// 7 rows, 16 bits
//
//   Col   D 1 2 3 4 5
//   Row
//     1     X X X X X
//     2     X X X X X
//     3     X X X X X
//     4     X X X X X
//     5     X X X X X
//     6     X X X X X
//     7   X X X X X X
//
// Bit:  7  6  5  4  3  2  1  0  7  6  5  4  3  2  1  0
// Col:  -  DA 5A 4A 3A 2A 1A -  -  DB 5B 4B 3B 2B 1B -
//
// Row select:
// 3'b000 : Row 7
// 3'b001 : Row 6
// 3'b010 : Row 5
// 3'b011 : Row 4
// 3'b100 : Row 3
// 3'b101 : Row 2
// 3'b110 : Row 1
//
void DisplayController::blit_char(uint32_t charIndex, const uint8_t *data)
{
    uint16_t localBuffer[kRowCount];
    uint32_t bitOffset = (charIndex == 0) ? 8 : 0;
    uint32_t i;

    for (i = 7; i > 0; --i)
    {
        uint16_t row = m_buffer[m_activeBuffer][i - 1];

        uint32_t bits = __RBIT((uint32_t)*data++) >> 23;

        row &= ~(0xff << bitOffset);
        row |= (bits << bitOffset);

        localBuffer[i - 1] = row;
    }

    // Update the next buffer and request buffer flip.
    uint32_t nextBuffer = (m_activeBuffer == 0) ? 1 : 0;
    memcpy(m_buffer[nextBuffer], localBuffer, sizeof(localBuffer));
    m_flipBuffers = true;
}

void DisplayController::set_char(uint32_t charIndex, char c)
{
    const uint8_t * glyphData;
    uint32_t i = 0;
    for (; i < g_font_5x7.count; ++i)
    {
        if (g_font_5x7.index[i] == c)
        {
            glyphData = &g_font_5x7.glyphs[i * FONT5x7_HEIGHT_IN_BYTES];
            blit_char(charIndex, glyphData);
            return;
        }
    }
}

void DisplayController::transfer_row()
{
    // Start transfer.
    dspi_transfer_t transfer;
    transfer.txData = reinterpret_cast<uint8_t *>(&(m_buffer[m_activeBuffer][m_row]));
    transfer.rxData = NULL;
    transfer.dataSize = 2;
    transfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs1;

    DSPI_MasterTransferNonBlocking(SPI0, &m_spiHandle, &transfer);
}

void DisplayController::latch_and_advance()
{
    // Turn off output for previous row.
    set_row_enable(false);

    // Latch row data that was just sent.
    set_latch(true);
    for (int n = 0; n < 15; ++n) // 15 = ~75ns @ 180MHz core
    {
    }
    set_latch(false);

    // Select the row that was just sent.
    select_row(m_row);

    // Turn on new row output.
    set_row_enable(true);

    // Advance row that will be sent next.
    ++m_row;
    if (m_row >= kRowCount)
    {
        m_row = 0;

        // Swap bitmap buffers at end of frame update if requested.
        if (m_flipBuffers)
        {
            uint32_t nextBuffer = (m_activeBuffer == 0) ? 1 : 0;
//             memcpy(m_buffer[m_activeBuffer], m_buffer[nextBuffer], sizeof(localBuffer));
            m_activeBuffer = nextBuffer;
            m_flipBuffers = false;
        }
    }
}

void DisplayController::timer_handler()
{
    latch_and_advance();
    transfer_row();
}

void DisplayController::spi_handler()
{
//     latch_and_advance();
}

void DisplayController::set_latch(bool enable)
{
    GPIO_WritePinOutput(PIN_LATCH_GPIO, PIN_LATCH_BIT, static_cast<uint8_t>(enable));
}

void DisplayController::set_output_enable(bool enable)
{
    GPIO_WritePinOutput(PIN_nOE_GPIO, PIN_nOE_BIT, static_cast<uint8_t>(enable));
}

void DisplayController::set_row_enable(bool enable)
{
    GPIO_WritePinOutput(PIN_ROW_EN_GPIO, PIN_ROW_EN_BIT, static_cast<uint8_t>(enable));
}

void DisplayController::select_row(uint32_t row)
{
    // Clear address bits to 0.
    GPIO_ClearPinsOutput(PIN_ROW_A0_GPIO, PIN_ROW_A0 | PIN_ROW_A1 | PIN_ROW_A2);

    // Set selected address bits to 1.
    uint32_t mask = (row << PIN_ROW_A0_BIT) & (PIN_ROW_A0 | PIN_ROW_A1 | PIN_ROW_A2);
    GPIO_SetPinsOutput(PIN_ROW_A0_GPIO, mask);
}

void DisplayController::dspi_master_transfer_callback(SPI_Type *base,
                                                dspi_master_handle_t *handle,
                                                status_t status,
                                                void *userData)
{
//     DisplayController * _this = reinterpret_cast<DisplayController *>(userData);
//     _this->m_transferDoneSem.put();
//     _this->spi_handler();
}

extern "C" void TPM1_IRQHandler(void)
{
    DisplayController::get().timer_handler();
    TPM_ClearStatusFlags(TPM1, kTPM_TimeOverflowFlag);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
