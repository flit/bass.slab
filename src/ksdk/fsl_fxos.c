/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
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

#include "fsl_fxos.h"

/******************************************************************************
 * Code
 ******************************************************************************/
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
volatile static bool g_completionFlag = false;
volatile static bool g_nakFlag = false;

void FXOS_master_callback(LPI2C_Type *base, lpi2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if (status == kStatus_LPI2C_Nak)
    {
        g_nakFlag = true;
    }
}
#endif

status_t FXOS_Init(fxos_handle_t *fxos_handle)
{
    uint8_t tmp[1] = {0};

    FXOS_ReadReg(fxos_handle, WHO_AM_I_REG, tmp, 1);
    if (tmp[0] != kFXOS_WHO_AM_I_Device_ID)
    {
        return kStatus_Fail;
    }

    /* setup auto sleep with FFMT trigger */
    /* go to standby */
    FXOS_ReadReg(fxos_handle, CTRL_REG1, tmp, 1);

    FXOS_WriteReg(fxos_handle, CTRL_REG1, tmp[0] & (uint8_t)~ACTIVE_MASK);

    /* Read again to make sure we are in standby mode. */
    FXOS_ReadReg(fxos_handle, CTRL_REG1, tmp, 1);
    if ((tmp[0] & ACTIVE_MASK) == ACTIVE_MASK)
    {
        return kStatus_Fail;
    }

    /* Disable the FIFO */
    FXOS_WriteReg(fxos_handle, F_SETUP_REG, F_MODE_DISABLED);

#ifdef LPSLEEP_HIRES
    /* enable auto-sleep, low power in sleep, high res in wake */
    FXOS_WriteReg(fxos_handle, CTRL_REG2, SLPE_MASK | SMOD_LOW_POWER | MOD_HIGH_RES);
#else
    /* enable auto-sleep, low power in sleep, high res in wake */
    FXOS_WriteReg(fxos_handle, CTRL_REG2, MOD_HIGH_RES);

#endif

    /* set up Mag OSR and Hybrid mode using M_CTRL_REG1, use default for Acc */
    FXOS_WriteReg(fxos_handle, M_CTRL_REG1, (M_RST_MASK | M_OSR_MASK | M_HMS_MASK));

    /* Enable hyrid mode auto increment using M_CTRL_REG2 */
    FXOS_WriteReg(fxos_handle, M_CTRL_REG2, (M_HYB_AUTOINC_MASK));

#ifdef EN_FFMT
    /* enable FFMT for motion detect for X and Y axes, latch enable */
    FXOS_WriteReg(fxos_handle, FF_MT_CFG_REG, XEFE_MASK | YEFE_MASK | ELE_MASK | OAE_MASK);
#endif

#ifdef SET_THRESHOLD
    /* set threshold to about 0.25g */
    FXOS_WriteReg(fxos_handle, FT_MT_THS_REG, 0x04);
#endif

#ifdef SET_DEBOUNCE
    /* set debounce to zero */
    FXOS_WriteReg(fxos_handle, FF_MT_COUNT_REG, 0x00);
#endif

#ifdef EN_AUTO_SLEEP
    /* set auto-sleep wait period to 5s (=5/0.64=~8) */
    FXOS_WriteReg(fxos_handle, ASLP_COUNT_REG, 8);
#endif
    /* default set to 4g mode */
    FXOS_WriteReg(fxos_handle, XYZ_DATA_CFG_REG, FULL_SCALE_2G);

#ifdef EN_INTERRUPTS
    /* enable data-ready, auto-sleep and motion detection interrupts */
    /* FXOS1_WriteRegister(CTRL_REG4, INT_EN_DRDY_MASK | INT_EN_ASLP_MASK | INT_EN_FF_MT_MASK); */
    FXOS_WriteReg(fxos_handle, CTRL_REG4, 0x0);

    /* route data-ready interrupts to INT1, others INT2 (default) */
    FXOS_WriteReg(fxos_handle, CTRL_REG5, INT_CFG_DRDY_MASK);

    /* enable ffmt as a wake-up source */
    FXOS_WriteReg(fxos_handle, CTRL_REG3, WAKE_FF_MT_MASK);

    /* finally activate accel_device with ASLP ODR=0.8Hz, ODR=100Hz, FSR=2g */
    FXOS_WriteReg(fxos_handle, CTRL_REG1, HYB_ASLP_RATE_0_8HZ | HYB_DATA_RATE_100HZ | ACTIVE_MASK);

#else
    /* Setup the ODR for 50 Hz and activate the accelerometer */
    FXOS_WriteReg(fxos_handle, CTRL_REG1, (HYB_DATA_RATE_200HZ | ACTIVE_MASK));

#endif

    /* Read Control register again to ensure we are in active mode */
    FXOS_ReadReg(fxos_handle, CTRL_REG1, tmp, 1);

    if ((tmp[0] & ACTIVE_MASK) != ACTIVE_MASK)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t FXOS_ReadSensorData(fxos_handle_t *fxos_handle, fxos_data_t *sensorData)
{
    status_t status = kStatus_Success;
    uint8_t tmp_buff[6] = {0};
//     uint8_t i = 0;

    if (!FXOS_ReadReg(fxos_handle, OUT_X_MSB_REG, tmp_buff, 6) == kStatus_Success)
    {
        status = kStatus_Fail;
    }

//     for (i = 0; i < 6; i++)
//     {
//         ((int8_t *)sensorData)[i] = tmp_buff[i];
//     }

    sensorData->accelX = (tmp_buff[0] << 8) | tmp_buff[1];
    sensorData->accelY = (tmp_buff[2] << 8) | tmp_buff[3];
    sensorData->accelZ = (tmp_buff[4] << 8) | tmp_buff[5];

    if (!FXOS_ReadReg(fxos_handle, M_OUT_X_MSB_REG, tmp_buff, 6) == kStatus_Success)
    {
        status = kStatus_Fail;
    }

//     for (i = 0; i < 6; i++)
//     {
//         ((int8_t *)sensorData)[i + 6] = tmp_buff[i];
//     }

    sensorData->magX = (tmp_buff[0] << 8) | tmp_buff[1];
    sensorData->magY = (tmp_buff[2] << 8) | tmp_buff[3];
    sensorData->magZ = (tmp_buff[4] << 8) | tmp_buff[5];

    return status;
}

status_t FXOS_ReadReg(fxos_handle_t *handle, uint8_t reg, uint8_t *val, uint8_t bytesNumber)
{
    status_t status = kStatus_Success;

    /* Configure I2C xfer */
    handle->xfer.subaddress = (uint32_t)reg;
    handle->xfer.subaddressSize = 1U;
    handle->xfer.data = val;
    handle->xfer.dataSize = bytesNumber;
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    handle->xfer.direction = kLPI2C_Read;
    handle->xfer.flags = kLPI2C_TransferDefaultFlag;
#else
    handle->xfer.direction = kI2C_Read;
    handle->xfer.flags = kI2C_TransferDefaultFlag;
#endif

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    LPI2C_MasterTransferNonBlocking(handle->base, handle->i2cHandle, &handle->xfer);
    /*  wait for transfer completed. */
    while ((!g_nakFlag) && (!g_completionFlag))
    {
    }

    g_nakFlag = false;

    if (g_completionFlag == true)
    {
        g_completionFlag = false;
    }
    else
    {
        status = kStatus_Fail;
    }
#else
    status = I2C_MasterTransferBlocking(handle->base, &handle->xfer);
#endif

    return status;
}

status_t FXOS_WriteReg(fxos_handle_t *handle, uint8_t reg, uint8_t val)
{
    status_t status = kStatus_Success;
    uint8_t buff[1];

    buff[0] = val;
    /* Set I2C xfer structure */
    handle->xfer.subaddress = (uint32_t)reg;
    handle->xfer.subaddressSize = 1U;
    handle->xfer.data = buff;
    handle->xfer.dataSize = 1U;
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    handle->xfer.direction = kLPI2C_Write;
    handle->xfer.flags = kLPI2C_TransferDefaultFlag;
#else
    handle->xfer.direction = kI2C_Write;
    handle->xfer.flags = kI2C_TransferDefaultFlag;
#endif

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    LPI2C_MasterTransferNonBlocking(handle->base, handle->i2cHandle, &handle->xfer);
    /*  wait for transfer completed. */
    while ((!g_nakFlag) && (!g_completionFlag))
    {
    }

    g_nakFlag = false;

    if (g_completionFlag == true)
    {
        g_completionFlag = false;
    }
    else
    {
        status = kStatus_Fail;
    }
#else
    status = I2C_MasterTransferBlocking(handle->base, &handle->xfer);
#endif

    return status;
}
