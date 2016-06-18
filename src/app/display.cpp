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
#include "fsl_clock.h"
#include "fsl_gpio.h"

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

DisplayController::DisplayController()
{
}

void DisplayController::init()
{
    // Create semaphore.
    m_transferDoneSem.init("txdone", 0);

    // Init SPI.
    dspi_master_config_t masterConfig;
    masterConfig.ctarConfig.baudRate = 1000000; // 1 MHz
    masterConfig.ctarConfig.bitsPerFrame = 16;
    masterConfig.whichPcs = kDSPI_Pcs1;
    DSPI_MasterGetDefaultConfig(&masterConfig);
    DSPI_MasterInit(SPI0, &masterConfig, CLOCK_GetBusClkFreq());

    DSPI_MasterTransferCreateHandle(SPI0, &m_spiHandle, dspi_master_transfer_callback, this);

    // Start thread.
    m_thread.init("display", this, &DisplayController::display_thread, 110, kArSuspendThread);
}

void DisplayController::start()
{
    DSPI_Enable(SPI0, true);

    m_thread.resume();
}

void DisplayController::display_thread()
{
    uint16_t counter = 0;
    uint32_t row = 6;
    uint32_t n;

    set_output_enable(true);

    while (1)
    {
        DSPI_StartTransfer(SPI0);

        // Get data for this row.
        uint8_t data[2];
        data[0] = counter & 0xff;
        data[1] = (counter >> 8) & 0xff;
        ++counter;

        // Start transfer.
        dspi_transfer_t transfer;
        transfer.txData = data;
        transfer.rxData = NULL;
        transfer.dataSize = 2;
        transfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs1;

        DSPI_MasterTransferNonBlocking(SPI0, &m_spiHandle, &transfer);

        // Wait until transfer completes.
        m_transferDoneSem.get();

        set_row_enable(false);

        // Latch new row data.
        set_latch(true);
//         Ar::Thread::sleep(1);
        for (n = 0; n < 10000; ++n)
        {
        }
        set_latch(false);

        // Select new row.
        if (row != 0)
        {
            --row;
        }
        else
        {
            row = 6;
        }
        select_row(row);

        set_row_enable(true);

        DSPI_StopTransfer(SPI0);

        Ar::Thread::sleep(10);
    }
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
    // Disable row output while we change the selected row.
//     set_row_enable(false);

    // Clear address bits to 0.
    GPIO_ClearPinsOutput(PIN_ROW_A0_GPIO, PIN_ROW_A0 | PIN_ROW_A1 | PIN_ROW_A2);

    // Set selected address bits to 1.
    uint32_t mask = (row << PIN_ROW_A0_BIT) & (PIN_ROW_A0 | PIN_ROW_A1 | PIN_ROW_A2);
    GPIO_SetPinsOutput(PIN_ROW_A0_GPIO, mask);

    // Re-enable row output.
//     set_row_enable(true);
}

void DisplayController::dspi_master_transfer_callback(SPI_Type *base,
                                                dspi_master_handle_t *handle,
                                                status_t status,
                                                void *userData)
{
    DisplayController * _this = reinterpret_cast<DisplayController *>(userData);
    _this->m_transferDoneSem.put();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
