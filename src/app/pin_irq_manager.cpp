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

#include "pin_irq_manager.h"
#include "board.h"
#include "fsl_common.h"
#include "fsl_port.h"

using namespace slab;

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

static const PORT_Type * kPortBases[] = PORT_BASE_PTRS;
static const IRQn_Type kPortIrqs[] = PORT_IRQS;

PinIrqManager PinIrqManager::s_manager;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

PinIrqManager::PinIrqManager()
{
    memset(m_pins, 0, sizeof(m_pins));
}

void PinIrqManager::connect(PORT_Type * port, uint32_t pin, pin_callback_t callback, void * userData)
{
    assert(pin < kMaxPinsPerPort);

    uint32_t portIndex = portToIndex(port);

    m_pins[portIndex][pin].callback = callback;
    m_pins[portIndex][pin].userData = userData;

    EnableIRQ(kPortIrqs[portIndex]);
}

void PinIrqManager::disconnect(PORT_Type * port, uint32_t pin)
{
    assert(pin < kMaxPinsPerPort);

    uint32_t portIndex = portToIndex(port);

    m_pins[portIndex][pin].callback = NULL;
    m_pins[portIndex][pin].userData = NULL;
}

void PinIrqManager::handle_irq(PORT_Type * port)
{
    uint32_t portIndex = portToIndex(port);
    uint32_t irqMask = PORT_GetPinsInterruptFlags(port);

    uint32_t pin;
    for (pin = 0; pin < kMaxPinsPerPort; ++pin)
    {
        if (irqMask & (1 << pin))
        {
            PinInfo & info = m_pins[portIndex][pin];
            if (info.callback)
            {
                info.callback(port, pin, info.userData);
            }
        }
    }

    PORT_ClearPinsInterruptFlags(PORTA, irqMask);
}

uint32_t PinIrqManager::portToIndex(PORT_Type * port)
{
    uint32_t i;
    for (i = 0; i < ARRAY_SIZE(kPortBases); ++i)
    {
        if (port == kPortBases[i])
        {
            return i;
        }
    }
    assert(false && "Invalid port base address");
}

extern "C" void PORTA_IRQHandler(void)
{
    PinIrqManager::get().handle_irq(PORTA);
}

extern "C" void PORTB_IRQHandler(void)
{
    PinIrqManager::get().handle_irq(PORTB);
}

extern "C" void PORTC_IRQHandler(void)
{
    PinIrqManager::get().handle_irq(PORTC);
}

extern "C" void PORTD_IRQHandler(void)
{
    PinIrqManager::get().handle_irq(PORTD);
}

extern "C" void PORTE_IRQHandler(void)
{
    PinIrqManager::get().handle_irq(PORTE);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
