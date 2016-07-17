/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
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
#ifndef DA_7212_H_
#define DA_7212_H_

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include "fsl_i2c.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Defines & Macros Section
///////////////////////////////////////////////////////////////////////////////////////////////////

#define DIALOG7212_MASTER (TRUE)

/* Defines whether the codec will work as master or slave */
#if (DIALOG7212_MASTER == TRUE)
#define DIALOG7212_SYS_MCLK (12288000) //	(12000000U) 		// Defines the input clock delivered by the master
#endif

#define DA7212_DAC_MAX_VOL (DA7213_DAC_GAIN_12DB)

#define DA7212_DAC_MIN_VOL (DA7212_DAC_GAIN_MUTE)

#define DA7212_MUTE_ON (1)

#define DA7212_MUTE_OFF (0)

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Typedef Section
///////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum
{
    DA7212_STATUS_BUSY = 0,
    DA7212_STATUS_CONFIGURED,
    DA7212_STATUS_I2C_ERROR,
    DA7212_STATUS_WAIT_TIMER,
    DA7212_STATUS_STARTUP_DONE
} eDA7212Status;

/** Supported frequencies */
typedef enum
{
    DA7212_SYS_FS_8K = 0x0,
    DA7212_SYS_FS_16K,
    DA7212_SYS_FS_32K,
    DA7212_SYS_FS_44_1K,
    DA7212_SYS_FS_48K,
    DA7212_SYS_FS_96K,
    DA7212_SYS_FS_MAX
} eDa7212SysFs;

typedef enum
{
    DA7212_STATUS_BUSY_MASK = 1 << DA7212_STATUS_BUSY,
    DA7212_STATUS_CONFIGURED_MASK = 1 << DA7212_STATUS_CONFIGURED,
    DA7212_STATUS_I2C_ERROR_MASK = 1 << DA7212_STATUS_I2C_ERROR,
    DA7212_STATUS_WAIT_TIMER_MASK = 1 << DA7212_STATUS_WAIT_TIMER,
    DA7212_STATUS_STARTUP_DONE_MASK = 1 << DA7212_STATUS_STARTUP_DONE
} eDA7212StatusMasks;

typedef enum
{
    DA7212_Input_AUX = 0x0,
    DA7212_Input_MIC1_Dig,
    DA7212_Input_MIC1_An,
    DA7212_Input_MIC2,
    DA7212_Input_MAX
} eDA7212InputReg;

typedef enum
{
    DA7212_Output_HP = 0x0,
    DA7212_Output_SP,
    DA7212_Output_MAX
} eDA7212OutputReg;

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function Prototypes Section
///////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

void DA7212_InitCodec(I2C_Type *base);

void DA7212_Driver(void);

uint32_t DA7212_GetStatus(void);

uint8_t DA7212_ChangeFrequency(eDa7212SysFs dwNewFreqValue);

uint8_t DA7212_ChangeHPVolume(uint8_t bDigitalGaindBIndex);

void DA7212_Mute(uint8_t bMuteStatus);

void DA7212_ChangeInput(eDA7212InputReg DA7212_Input);

void DA7212_ChangeOutput(eDA7212OutputReg DA7212_Output);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* DA_7212_H_ */
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
