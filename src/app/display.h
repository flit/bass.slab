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
#if !defined(_DISPLAY_H_)
#define _DISPLAY_H_

#include "argon.h"
#include "fsl_dspi.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Display controller.
 */
class DisplayController
{
public:
    DisplayController();
    ~DisplayController() {}

    static DisplayController & get() { return *s_instance; }

    void init();
    void start();

    void set_char(uint32_t charIndex, char c);
    void blit_char(uint32_t charIndex, const uint8_t *data);

    void timer_handler();
    void spi_handler();

protected:
    static const uint32_t kRowCount = 7;

    static DisplayController * s_instance;
    Ar::Semaphore m_transferDoneSem;
    Ar::Semaphore m_nextRowSem;
    dspi_master_handle_t m_spiHandle;
    uint16_t m_buffer[2][kRowCount];
    uint32_t m_activeBuffer;
    bool m_flipBuffers;
    uint32_t m_row;
    uint8_t m_data[2];

    void transfer_row();
    void latch_and_advance();

    void set_latch(bool enable);
    void set_output_enable(bool enable);
    void set_row_enable(bool enable);
    void select_row(uint32_t row);

    static void dspi_master_transfer_callback(SPI_Type *base,
                                                dspi_master_handle_t *handle,
                                                status_t status,
                                                void *userData);
};

} // namespace slab

#endif // _DISPLAY_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
