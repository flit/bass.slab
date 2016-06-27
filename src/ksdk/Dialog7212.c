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
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
// #include "ProjectTypes.h"
// #include "fsl_i2c_master_driver.h"
#include "Dialog7212.h"
// #include "SW_Timer.h"
#include "argon/argon.h"
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Defines & Macros Section
///////////////////////////////////////////////////////////////////////////////////////////////////
#define DA7212_CHECK_STATUS(x)	(DA7212_gDriverStatus&(x))

#define DA7212_CONTROL_BUSY		(DA7212_gDriverStatus&(DA7212_STATUS_BUSY_MASK))

#define DA7212_CONFIGURED		(DA7212_gDriverStatus&(DA7212_STATUS_CONFIGURED_MASK))

#define DA7212_SET_STATUS(x)	(DA7212_gDriverStatus|=(x))

#define DA7212_CLEAR_STATUS(x)	(DA7212_gDriverStatus&=~(x))

#define DA7212_ADDRESS	(0x1A)

#define DA7212_50MS_TIMEOUT	(50)

#define DA7212_CONTROL_BUFFER_SIZE	(2)

#if (DIALOG7212_MASTER == TRUE)
	#define DA7212_INIT_SIZE	(41)
#else	/* DA7212 as slave */
	#define DA7212_INIT_SIZE	(40)
#endif

#define DA7212_INPUT_CHANGE_REGISTERS	(14)

#define DA7212_AUX_REGISTER_SIZE	(33)

#define DA7212_MIC2_REGISTER_SIZE	(33)

#define DA7212_MIC1_REGISTER_SIZE	(24)

#define DA7212_DAC_MUTE_ENABLED		(0xC0)

#define DA7212_DAC_MUTE_DISABLED	(0x80)

#define DA7212_MAX_VOLUME_STEPS 	(15)

#define DA7212_MIN_VOLUME_STEPS 	(0)

#define DA7212_DAC_GAIN_MUTE        (0x07)

#define DA7212_DAC_GAIN_M72DB       (0x17)

#define DA7212_DAC_GAIN_M60DB       (0x1F)

#define DA7212_DAC_GAIN_M54DB       (0x27)

#define DA7212_DAC_GAIN_M48DB       (0x2F)

#define DA7212_DAC_GAIN_M42DB       (0x37)

#define DA7212_DAC_GAIN_M36DB       (0x3F)

#define DA7212_DAC_GAIN_M30DB       (0x47)

#define DA7212_DAC_GAIN_M24DB       (0x4F)

#define DA7212_DAC_GAIN_M18DB       (0x57)

#define DA7212_DAC_GAIN_M12DB       (0x5F)

#define DA7212_DAC_GAIN_M6DB        (0x67)

#define DA7212_DAC_GAIN_0DB         (0x6F)

#define DA7213_DAC_GAIN_6DB         (0x77)

#define DA7213_DAC_GAIN_12DB        (0x7F)

#define DA7212_MAX_VOLUME_STEPS 	(15)

#define DA7212_CHANGE_FREQ_SIZE 	(0x05)

#define I2C1_instance				(0x1)

#define baudRate					(125)

#if DA7212_44_1KHZ_NEED_PLL
	#define DA7212_441_PLL_REGISTERS_TO_WRITE	5
#else
	#define DA7212_441_PLL_REGISTERS_TO_WRITE	2
#endif

#if DA7212_32KHZ_48KHZ_96KHZ_NEED_PLL
	#define DA7212_32KHZ_48KHZ_96KHZ_PLL_REGISTERS_TO_WRITE	5
#else
	#define DA7212_16KHZ_32KHZ_48KHZ_96KHZ_PLL_REGISTERS_TO_WRITE	2
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Typedef Section
///////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum
{
	DA7212_CONTROL_STATE_IDLE = 0,
	DA7212_CONTROL_STATE_WAIT_COMM,
	DA7212_WRITE_BLOCK_STATE,
	DA7212_CONTROL_STATE_WAIT_TIMEOUT,
	DA7212_CONTROL_MAX_STATES
}eDA7212ControlStates;

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Function Prototypes Section
///////////////////////////////////////////////////////////////////////////////////////////////////
static void DA7212_IdleState(void);

static void DA7212_WaitCommState(void);

static void DA7212_WriteBlockState(void);

static void DA7212_TimeoutState(void);

static void DA7212_SWTimerCallback (void);

static void (* const DA7212_StateMachine[]) (void) =
{
		DA7212_IdleState,
		DA7212_WaitCommState,
		DA7212_WriteBlockState,
		DA7212_TimeoutState,
};
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Constants Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Constants Section
///////////////////////////////////////////////////////////////////////////////////////////////////

static const uint8_t DA7212_gInputRegisterSequence[DA7212_Input_MAX][17] =
{
		{
				DIALOG7212_MIXIN_L_SELECT,
				DIALOG7212_MIXIN_R_SELECT,
				DIALOG7212_CP_CTRL,
				DIALOG7212_AUX_L_CTRL,
				DIALOG7212_AUX_R_CTRL,
				DIALOG7212_MIC_1_CTRL,
				DIALOG7212_MIC_2_CTRL,
				DIALOG7212_MIXIN_L_CTRL,
				DIALOG7212_MIXIN_R_CTRL,
				DIALOG7212_ADC_L_CTRL,
				DIALOG7212_GAIN_RAMP_CTRL,
				DIALOG7212_PC_COUNT,
				DIALOG7212_CP_DELAY
		},
		{
				DIALOG7212_MICBIAS_CTRL,
				DIALOG7212_CP_CTRL,
				DIALOG7212_MIXIN_L_SELECT,
				DIALOG7212_MIXIN_R_SELECT,
				DIALOG7212_SYSTEM_MODES_INPUT,
				DIALOG7212_SYSTEM_MODES_OUTPUT,
				DIALOG7212_MIC_2_GAIN,
				DIALOG7212_MIC_2_CTRL,
				DIALOG7212_MIC_1_GAIN,
				DIALOG7212_MIC_1_CTRL
		},
		{
				DIALOG7212_MIXIN_L_SELECT,
				DIALOG7212_MIXIN_R_SELECT,
				DIALOG7212_MIC_1_GAIN,
				DIALOG7212_CP_CTRL,
				DIALOG7212_MIXOUT_L_SELECT,
				DIALOG7212_MIXOUT_R_SELECT,
				DIALOG7212_AUX_R_CTRL,
				DIALOG7212_MICBIAS_CTRL,
				DIALOG7212_MIC_1_CTRL,
				DIALOG7212_MIC_2_CTRL,
				DIALOG7212_MIXIN_L_CTRL,
				DIALOG7212_MIXIN_R_CTRL,
				DIALOG7212_ADC_L_CTRL,
				DIALOG7212_GAIN_RAMP_CTRL,
				DIALOG7212_PC_COUNT,
				DIALOG7212_CP_DELAY
		},
		{
				DIALOG7212_MIXIN_L_SELECT,
				DIALOG7212_MIXIN_R_SELECT,
				DIALOG7212_MIC_2_GAIN,
				DIALOG7212_CP_CTRL,
				DIALOG7212_AUX_R_CTRL,
				DIALOG7212_MICBIAS_CTRL,
				DIALOG7212_MIC_1_CTRL,
				DIALOG7212_MIC_2_CTRL,
				DIALOG7212_MIXIN_L_CTRL,
				DIALOG7212_MIXIN_R_CTRL,
				DIALOG7212_ADC_L_CTRL,
				DIALOG7212_GAIN_RAMP_CTRL,
				DIALOG7212_PC_COUNT,
				DIALOG7212_CP_DELAY
		}
};

static const uint8_t DA7212_gInputRegisterValuesSequence[DA7212_Input_MAX][17] =
{
		{
				0x01,
				0x01,
				0xFD,
				0xB4,
				0xB0,
				0x04,
				0x04,
				0x88,
				0x88,
				0xA0,
				0x02,
				0x02,
				0x95                                                                         //DIALOG7212_MIXOUT_R_SELECT,
		},
		{
				0xA9,                                                                           //DIALOG7212_MICBIAS_CTRL,
				0xF1,                                                                           //DIALOG7212_CP_CTRL,
				0x80,                                                                           //DIALOG7212_MIXIN_L_SELECT,
				0x80,                                                                           //DIALOG7212_MIXIN_R_SELECT,                                                                        //DIALOG7212_MIXOUT_R_SELECT,
				0xFE,                                                                           //DIALOG7212_SYSTEM_MODES_INPUT,
				0xF7,                                                                           //DIALOG7212_SYSTEM_MODES_OUTPUT,
				0x04,                                                                           //DIALOG7212_MIC_2_GAIN,
				0x84,                                                                           //DIALOG7212_MIC_2_CTRL,
				0x01,                                                                           //DIALOG7212_MIC_1_GAIN,
				0x80
		},
		{
				0x02,
				0x04,
				0x03,
				0xFD,
				0x08,
				0x08,
				0x40,
				0x19,
				0x84,
				0x04,
				0x88,
				0x88,
				0xA0,
				0x02,
				0x02,
				0x95
		},
		{
				0x04,
				0x02,
				0x04,
				0xFD,
				0x40,
				0x91,
				0x08,
				0x84,
				0x88,
				0x88,
				0xA0,
				0xA0,
				0xA0,
				0x02,
				0x02,
				0x95
		}
};

static const uint8_t DA7212_gOutputRegisterSequence[DA7212_Output_MAX][4] =
{
		{
				DIALOG7212_CP_CTRL,
				DIALOG7212_LINE_CTRL,
				DIALOG7212_HP_L_CTRL,
				DIALOG7212_HP_R_CTRL
		},
		{
				DIALOG7212_CP_CTRL,
				DIALOG7212_HP_L_CTRL,
				DIALOG7212_HP_R_CTRL,
				DIALOG7212_LINE_CTRL
		}
};

static const uint8_t DA7212_gOutputRegisterValuesSequence[DA7212_Output_MAX][4] =
{
		{
				0xFD,
				0X40,
				(DIALOG7212_HP_L_CTRL_AMP_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_RAMP_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_ZC_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_OE_MASK),
				(DIALOG7212_HP_R_CTRL_AMP_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_RAMP_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_ZC_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_OE_MASK),
		},
		{
				0x3D,
				0x40,
				0x40,
				0xA8
		}
};

static const uint8_t DA7212_gInitRegisterSequence[DA7212_INIT_SIZE]=
{
		DIALOG7212_STATUS1,
		DIALOG7212_PLL_STATUS,
		DIALOG7212_DAC_L_GAIN_STATUS,
		DIALOG7212_DAC_R_GAIN_STATUS,
		DIALOG7212_HP_L_GAIN_STATUS,
		DIALOG7212_HP_R_GAIN_STATUS,
		DIALOG7212_LINE_GAIN_STATUS,
		DIALOG7212_SR,
		DIALOG7212_REFERENCES,
		DIALOG7212_PLL_FRAC_TOP,
		DIALOG7212_PLL_FRAC_BOT,
		DIALOG7212_PLL_INTEGER,
		DIALOG7212_PLL_CTRL,
		DIALOG7212_DAI_CLK_MODE,
		DIALOG7212_DAI_CTRL,
		DIALOG7212_DIG_ROUTING_DAC,
		DIALOG7212_CP_CTRL,
		DIALOG7212_MIXOUT_L_SELECT,
		DIALOG7212_MIXOUT_R_SELECT,
		DIALOG7212_DAC_L_CTRL,
		DIALOG7212_DAC_R_CTRL,
		DIALOG7212_HP_L_CTRL,
		DIALOG7212_HP_R_CTRL,
		DIALOG7212_MIXOUT_L_CTRL,
		DIALOG7212_MIXOUT_R_CTRL,
		DIALOG7212_CP_VOL_THRESHOLD1,
        DIALOG7212_SYSTEM_STATUS,
        DIALOG7212_DAC_L_GAIN,
        DIALOG7212_DAC_R_GAIN,
		DIALOG7212_MIXIN_L_SELECT,
		DIALOG7212_MIXIN_R_SELECT,
		DIALOG7212_MIXIN_L_GAIN,
		DIALOG7212_MIXIN_R_GAIN,
		DIALOG7212_ADC_L_GAIN,
		DIALOG7212_ADC_R_GAIN,
		DIALOG7212_AUX_L_CTRL,
		DIALOG7212_AUX_R_CTRL,
		DIALOG7212_MIXIN_L_CTRL,
		DIALOG7212_MIXIN_R_CTRL,
		DIALOG7212_ADC_L_CTRL,
		DIALOG7212_ADC_R_CTRL,

};

static const uint8_t DA7212_gInitRegisterValuesSequence[DA7212_INIT_SIZE]=
{
		CLEAR_REGISTER,
		(DIALOG7212_PLL_STATUS_MCLK_STATUS_MASK | DIALOG7212_PLL_STATUS_LOCK_MASK),
		CLEAR_REGISTER,
		CLEAR_REGISTER,
		CLEAR_REGISTER,
		CLEAR_REGISTER,
		CLEAR_REGISTER,
		DIALOG7212_SR_48KHZ,
		DIALOG7212_REFERENCES_BIAS_EN_MASK,
		DIALOG7212_FB_DIV_FRAC_TOP_48K,
		DIALOG7212_FB_DIV_FRAC_BOT_48K,
		DIALOG7212_FB_DIV_INTEGER_48K,
		(DIALOG7212_PLL_EN_MASK | DIALOG7212_PLL_INDIV),
		(DIALOG7212_DAI_CLK_EN_MASK | DIALOG7212_DAI_BCLKS_PER_WCLK_BCLK32),
		(DIALOG7212_DAI_EN_MASK | DIALOG7212_DAI_OE__MASK | DIALOG7212_DAI_WORD_LENGTH_16B),
		(DIALOG7212_DIG_ROUTING_DAC_R_RSC_DAC_R | DIALOG7212_DIG_ROUTING_DAC_L_RSC_DAC_L),
		(DIALOG7212_CP_CTRL_EN_MASK | DIALOG7212_CP_CTRL_SMALL_SWIT_CH_FREQ_EN_MASK | DIALOG7212_CP_CTRL_MCHANGE_OUTPUT | DIALOG7212_CP_CTRL_MOD_CPVDD_1 | DIALOG7212_CP_CTRL_ANALOG_VLL_LV_BOOSTS_CP),
		(DIALOG7212_MIXOUT_L_SELECT_DAC_L_MASK),
		(DIALOG7212_MIXOUT_R_SELECT_DAC_R_MASK),
		(DIALOG7212_DAC_L_CTRL_ADC_EN_MASK | DIALOG7212_DAC_L_CTRL_ADC_RAMP_EN_MASK),
		(DIALOG7212_DAC_R_CTRL_ADC_EN_MASK | DIALOG7212_DAC_R_CTRL_ADC_RAMP_EN_MASK),
		(DIALOG7212_HP_L_CTRL_AMP_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_RAMP_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_ZC_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_OE_MASK),
		(DIALOG7212_HP_R_CTRL_AMP_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_RAMP_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_ZC_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_OE_MASK),
		(DIALOG7212_MIXOUT_L_CTRL_AMP_EN_MASK | DIALOG7212_MIXOUT_L_CTRL_AMP_SOFT_MIX_EN_MASK | DIALOG7212_MIXOUT_L_CTRL_AMP_MIX_EN_MASK),
		(DIALOG7212_MIXOUT_R_CTRL_AMP_EN_MASK | DIALOG7212_MIXOUT_R_CTRL_AMP_SOFT_MIX_EN_MASK | DIALOG7212_MIXOUT_R_CTRL_AMP_MIX_EN_MASK),
		(DIALOG7212_CP_VOL_THRESHOLD1_VDD2(0x32)),
		CLEAR_REGISTER,
		DA7212_DAC_GAIN_0DB,
		DA7212_DAC_GAIN_0DB,
		DIALOG7212_MIXIN_L_SELECT_AUX_L_SEL_MASK,
		DIALOG7212_MIXIN_R_SELECT_AUX_R_SEL_MASK,
		DIALOG7212_MIXIN_L_AMP_GAIN(0x03),
		DIALOG7212_MIXIN_R_AMP_GAIN(0x03),
		DIALOG7212_ADC_L_DIGITAL_GAIN(0x6F),
		DIALOG7212_ADC_R_DIGITAL_GAIN(0x6F),
		DIALOG7212_AUX_L_CTRL_AMP_EN_MASK|DIALOG7212_AUX_L_CTRL_AMP_RAMP_EN_MASK|DIALOG7212_AUX_L_CTRL_AMP_ZC_EN_MASK,
		DIALOG7212_AUX_R_CTRL_AMP_EN_MASK|DIALOG7212_AUX_R_CTRL_AMP_RAMP_EN_MASK|DIALOG7212_AUX_R_CTRL_AMP_ZC_EN_MASK,
		DIALOG7212_MIXIN_L_CTRL_AMP_EN_MASK|DIALOG7212_MIXIN_L_CTRL_AMP_MIX_EN_MASK,
		DIALOG7212_MIXIN_R_CTRL_AMP_EN_MASK|DIALOG7212_MIXIN_R_CTRL_AMP_MIX_EN_MASK,
		DIALOG7212_ADC_L_CTRL_ADC_EN_MASK|DIALOG7212_ADC_L_CTRL_ADC_RAMP_EN_MASK,
		DIALOG7212_ADC_R_CTRL_ADC_EN_MASK|DIALOG7212_ADC_R_CTRL_ADC_RAMP_EN_MASK,
};

static const uint8_t DA7212_gVolumeControlValues[DA7212_MAX_VOLUME_STEPS]=
{
		DA7212_DAC_GAIN_MUTE,
		DA7212_DAC_GAIN_M72DB,
		DA7212_DAC_GAIN_M60DB,
		DA7212_DAC_GAIN_M54DB,
		DA7212_DAC_GAIN_M48DB,
		DA7212_DAC_GAIN_M42DB,
		DA7212_DAC_GAIN_M36DB,
		DA7212_DAC_GAIN_M30DB,
		DA7212_DAC_GAIN_M24DB,
		DA7212_DAC_GAIN_M18DB,
		DA7212_DAC_GAIN_M12DB,
		DA7212_DAC_GAIN_M6DB,
		DA7212_DAC_GAIN_0DB,
		DA7213_DAC_GAIN_6DB,
		DA7213_DAC_GAIN_12DB};

static const  uint8_t DA7212_gChangeFreqRegisters[DA7212_CHANGE_FREQ_SIZE] = /**< Registers definitions to change frequency */
{
		DIALOG7212_PLL_CTRL,
		DIALOG7212_SR,
		DIALOG7212_PLL_FRAC_TOP,
		DIALOG7212_PLL_FRAC_BOT,
		DIALOG7212_PLL_INTEGER,
};

static const uint8_t DA7212_gchangeFreqValues[DA7212_SYS_FS_MAX][DA7212_CHANGE_FREQ_SIZE] =
{
		{
#if DA7212_8KHZ_16KHZ_32KHZ_48KHZ_96KHZ_NEED_PLL
				DIALOG7212_PLL_EN_MASK|DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_8KHZ,
				DIALOG7212_FB_DIV_FRAC_TOP_8K,
				DIALOG7212_FB_DIV_FRAC_BOT_8K,
				DIALOG7212_FB_DIV_INTEGER_8K
#else
				DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_8KHZ,
				CLEAR_REGISTER,
				CLEAR_REGISTER,
				CLEAR_REGISTER
#endif
		},
		{
#if DA7212_8KHZ_16KHZ_32KHZ_48KHZ_96KHZ_NEED_PLL
				DIALOG7212_PLL_EN_MASK|DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_16KHZ,
				DIALOG7212_FB_DIV_FRAC_TOP_16K,
				DIALOG7212_FB_DIV_FRAC_BOT_16K,
				DIALOG7212_FB_DIV_INTEGER_16K
#else
				DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_16KHZ,
				CLEAR_REGISTER,
				CLEAR_REGISTER,
				CLEAR_REGISTER
#endif
		},
		{
#if DA7212_8KHZ_16KHZ_32KHZ_48KHZ_96KHZ_NEED_PLL
				DIALOG7212_PLL_EN_MASK|DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_32KHZ,
				DIALOG7212_FB_DIV_FRAC_TOP_32K,
				DIALOG7212_FB_DIV_FRAC_BOT_32K,
				DIALOG7212_FB_DIV_INTEGER_32K
#else
				DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_32KHZ,
				CLEAR_REGISTER,
				CLEAR_REGISTER,
				CLEAR_REGISTER
#endif


		},
		{
#if DA7212_44_1KHZ_NEED_PLL
				DIALOG7212_PLL_EN_MASK|DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_44_1KHZ,
				DIALOG7212_FB_DIV_FRAC_TOP_44_1K,
				DIALOG7212_FB_DIV_FRAC_BOT_44_1K,
				DIALOG7212_FB_DIV_INTEGER_44_1K
#else
				DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_44_1KHZ,,
				CLEAR_REGISTER,
				CLEAR_REGISTER,
				CLEAR_REGISTER
#endif
		},
		{
#if DA7212_8KHZ_16KHZ_32KHZ_48KHZ_96KHZ_NEED_PLL
				DIALOG7212_PLL_EN_MASK|DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_48KHZ,
				DIALOG7212_FB_DIV_FRAC_TOP_48K,
				DIALOG7212_FB_DIV_FRAC_BOT_48K,
				DIALOG7212_FB_DIV_INTEGER_48K
#else
				DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_48KHZ,
				CLEAR_REGISTER,
				CLEAR_REGISTER,
				CLEAR_REGISTER
#endif

		},
		{
#if DA7212_8KHZ_16KHZ_32KHZ_48KHZ_96KHZ_NEED_PLL
				DIALOG7212_PLL_EN_MASK|DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_96KHZ,
				DIALOG7212_FB_DIV_FRAC_TOP_96K,
				DIALOG7212_FB_DIV_FRAC_BOT_96K,
				DIALOG7212_FB_DIV_INTEGER_96K
#else
				DIALOG7212_PLL_INDIV,
				DIALOG7212_SR_96KHZ,
				CLEAR_REGISTER,
				CLEAR_REGISTER,
				CLEAR_REGISTER
#endif
		}

};

typedef struct {
    uint8_t actualState;
    uint8_t prevState;
    uint8_t nextState;
    uint8_t errorState;
} state_machine_t;

#define OK (0)
#define ERROR (1)

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Variables Section
///////////////////////////////////////////////////////////////////////////////////////////////////

// static volatile uint8_t DA7212_TimeOutFlag;

static volatile uint8_t DA7212_gDriverStatus;

static state_machine_t DA7212_gStates;

static uint8_t  DA7212_gSysFs= DA7212_SYS_FS_48K;

static uint8_t DA7212_gDataOutBuffer[DA7212_CONTROL_BUFFER_SIZE];

static uint8_t *DA7212_gReadData;

static uint8_t DA7212_gRegisterOffset = 0;

static uint8_t  DA7212_gRegistersToWrite = 0;

static uint8_t *DA7212_gWriteBlockRegister;

static uint8_t *DA7212_gWriteBlockRegisterValue;

static uint8_t gbDA7212_gLDOTimer;

static uint8_t DA7212_gRegistersToChange[10];

static uint8_t DA7212_gValuesToChange[10];

static uint32_t bytesRemaining;

static uint32_t s_startTime = 0;

static I2C_Type * s_i2cBase = NULL;
static i2c_master_handle_t s_i2cHandle;
static i2c_master_transfer_t s_i2cTransfer;
static volatile bool s_i2cTransferCompleted = false;
static status_t s_i2cTransferStatus;

// static const i2c_device_t DA7212_I2C =
// {
// 		.address	=	DA7212_ADDRESS,
// 		.baudRate_kbps = baudRate
// };

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Functions Section
///////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t DA7212_GetStatus(void)
{
	return DA7212_gDriverStatus;
}

void DA7212_WriteRegister(uint8_t u8Register, uint8_t u8RegisterData)
{
	/* set the driver as busy and the next state to wait physical layer driver */
	DA7212_SET_STATUS(DA7212_STATUS_BUSY_MASK);

	/* Set the command buffer to send the data thru physical layer */
// 	DA7212_gDataOutBuffer[0] = u8Register;
	DA7212_gDataOutBuffer[0] = u8RegisterData;

// 	I2C_MasterSendData(I2C1_instance,&DA7212_I2C,NULL,0,&DA7212_gDataOutBuffer[0],DA7212_CONTROL_BUFFER_SIZE);

	s_i2cTransfer.flags = 0;
	s_i2cTransfer.slaveAddress = DA7212_ADDRESS;
	s_i2cTransfer.direction = kI2C_Write;
	s_i2cTransfer.subaddress = u8Register;
	s_i2cTransfer.subaddressSize = 1;
	s_i2cTransfer.data = DA7212_gDataOutBuffer;
	s_i2cTransfer.dataSize = 1;

    s_i2cTransferCompleted = false;

    printf("DA7212: write reg 0x%02x = 0x%02x\r\n", u8Register, u8RegisterData);
	I2C_MasterTransferNonBlocking(s_i2cBase, &s_i2cHandle, &s_i2cTransfer);

	DA7212_gStates.actualState = DA7212_CONTROL_STATE_WAIT_COMM;
}

void DA7212_ReadRegister(uint8_t u8Register, uint8_t *pu8RegisterData)
{
	/* Set the driver as busy and the next state logic */
	DA7212_SET_STATUS(DA7212_STATUS_BUSY_MASK);	/* Set next state logic and write the register */

	/* Setup control buffer */
	DA7212_gDataOutBuffer[0] = u8Register;

	/* Setup pointer to the RX buffer */
	DA7212_gReadData = pu8RegisterData;

	/* Use physical layer */
// 	I2C_DRV_MasterReceiveData(I2C1_instance,&DA7212_I2C,NULL,0,(uint8_t*)DA7212_gReadData,1);

	s_i2cTransfer.flags = 0;
	s_i2cTransfer.slaveAddress = DA7212_ADDRESS;
	s_i2cTransfer.direction = kI2C_Read;
	s_i2cTransfer.subaddress = u8Register;
	s_i2cTransfer.subaddressSize = 1;
	s_i2cTransfer.data = DA7212_gReadData;
	s_i2cTransfer.dataSize = 1;

	s_i2cTransferCompleted = false;

	I2C_MasterTransferNonBlocking(s_i2cBase, &s_i2cHandle, &s_i2cTransfer);

	DA7212_gStates.actualState = DA7212_CONTROL_STATE_WAIT_COMM;
}

void DA7212_Driver(void)
{
	DA7212_StateMachine[DA7212_gStates.actualState]();
}

void DA7212_I2CCompletionCallback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    printf("DA7212: reg write complete; status = %d\r\n", status);
    s_i2cTransferCompleted = true;
    s_i2cTransferStatus = status;
}

void DA7212_InitCodec(I2C_Type * base)
{
    s_i2cBase = base;

    I2C_MasterTransferCreateHandle(s_i2cBase, &s_i2cHandle, DA7212_I2CCompletionCallback, NULL);

	/* Set the driver as busy */
	DA7212_SET_STATUS(DA7212_STATUS_BUSY_MASK);	/* Set next state logic and write the register */

	/*Prepare to write reset configuration*/
	DA7212_gRegistersToWrite = DA7212_INIT_SIZE;
	DA7212_gWriteBlockRegister = (uint8_t *)&DA7212_gInitRegisterSequence[0];
	DA7212_gWriteBlockRegisterValue = (uint8_t *)&DA7212_gInitRegisterValuesSequence[0];

	DA7212_gRegisterOffset = 0;

	DA7212_gStates.actualState = DA7212_WRITE_BLOCK_STATE;
	DA7212_gStates.nextState = DA7212_CONTROL_STATE_WAIT_COMM;
	DA7212_gStates.prevState = DA7212_CONTROL_STATE_IDLE;
}

void DA7212_ChangeInput(eDA7212InputReg DA7212_Input)
{
	DA7212_SET_STATUS(DA7212_STATUS_BUSY_MASK);	/* Set next state logic and write the register */

	/*Prepare to write reset configuration*/
	DA7212_gRegistersToWrite = sizeof(DA7212_gInputRegisterSequence[DA7212_Input]);
	DA7212_gWriteBlockRegister = (uint8_t *)&DA7212_gInputRegisterSequence[DA7212_Input][0];
	DA7212_gWriteBlockRegisterValue = (uint8_t *)&DA7212_gInputRegisterValuesSequence[DA7212_Input][0];

	DA7212_gRegisterOffset = 0;
	DA7212_gStates.actualState = DA7212_WRITE_BLOCK_STATE;
	DA7212_gStates.nextState = DA7212_CONTROL_STATE_WAIT_COMM;
	DA7212_gStates.prevState = DA7212_CONTROL_STATE_IDLE;
}

void DA7212_ChangeOutput(eDA7212OutputReg DA7212_Output)
{
	DA7212_SET_STATUS(DA7212_STATUS_BUSY_MASK);	/* Set next state logic and write the register */

	/*Prepare to write reset configuration*/

	DA7212_gRegistersToWrite = sizeof(DA7212_gOutputRegisterSequence[DA7212_Output]);
	DA7212_gWriteBlockRegister = (uint8_t *)&DA7212_gOutputRegisterSequence[DA7212_Output][0];
	DA7212_gWriteBlockRegisterValue = (uint8_t *)&DA7212_gOutputRegisterValuesSequence[DA7212_Output][0];

	DA7212_gRegisterOffset = 0;
	DA7212_gStates.actualState = DA7212_WRITE_BLOCK_STATE;
	DA7212_gStates.nextState = DA7212_CONTROL_STATE_WAIT_COMM;
	DA7212_gStates.prevState = DA7212_CONTROL_STATE_IDLE;

}
static void DA7212_IdleState(void)
{
	DA7212_CLEAR_STATUS(DA7212_STATUS_BUSY_MASK);
}

uint8_t DA7212_ChangeHPVolume(uint8_t bDigitalGaindBIndex)
{
	uint8_t bStatus = ERROR;

	if(!DA7212_CONTROL_BUSY)
	{
		/* Set the driver as busy */
		DA7212_SET_STATUS(DA7212_STATUS_BUSY_MASK);
		/* Clear configured flag since will be configured again */
		DA7212_CLEAR_STATUS(DA7212_STATUS_CONFIGURED_MASK);

		DA7212_gRegistersToChange[0] = DIALOG7212_DAC_L_GAIN;
		DA7212_gRegistersToChange[1] = DIALOG7212_DAC_R_GAIN;
		DA7212_gValuesToChange[0] = DA7212_gVolumeControlValues[bDigitalGaindBIndex];
		DA7212_gValuesToChange[1] = DA7212_gVolumeControlValues[bDigitalGaindBIndex];

		DA7212_gRegistersToWrite = 2;
		DA7212_gWriteBlockRegister = (uint8_t *)&DA7212_gRegistersToChange[0];
		DA7212_gWriteBlockRegisterValue = (uint8_t *)&DA7212_gValuesToChange[0];
		DA7212_gRegisterOffset = 0;

		DA7212_gStates.actualState = DA7212_CONTROL_STATE_WAIT_COMM;
		DA7212_gStates.nextState = DA7212_WRITE_BLOCK_STATE;

		bStatus = OK;

	}else
	{
		DA7212_SET_STATUS(DA7212_STATUS_CONFIGURED_MASK);
		DA7212_gStates.actualState = DA7212_CONTROL_STATE_IDLE;
		DA7212_gStates.nextState = DA7212_CONTROL_STATE_WAIT_TIMEOUT;
	}

	return(bStatus);
}

void DA7212_Mute(uint8_t bMuteStatus)
{
	DA7212_SET_STATUS(DA7212_STATUS_BUSY_MASK);

	if(DA7212_MUTE_ON == bMuteStatus)
	{
		DA7212_gValuesToChange[0] = DA7212_DAC_MUTE_ENABLED;
		DA7212_gValuesToChange[1] = DA7212_DAC_MUTE_ENABLED;
	}
	else
	{
		DA7212_gValuesToChange[0] = DA7212_DAC_MUTE_DISABLED;
		DA7212_gValuesToChange[1] = DA7212_DAC_MUTE_DISABLED;
	}

	DA7212_gRegistersToChange[0] = DIALOG7212_DAC_L_CTRL;
	DA7212_gRegistersToChange[1] = DIALOG7212_DAC_R_CTRL;

	DA7212_gRegistersToWrite = 2;
	DA7212_gWriteBlockRegister = (uint8_t *)&DA7212_gRegistersToChange[0];
	DA7212_gWriteBlockRegisterValue = (uint8_t *)&DA7212_gValuesToChange[0];

	DA7212_gStates.actualState = DA7212_WRITE_BLOCK_STATE;

}


uint8_t DA7212_ChangeFrequency(eDa7212SysFs dwNewFreqValue)
{
	uint8_t bStatus = ERROR;

#if (DIALOG7212_MASTER == TRUE)

	/* confirm the new frequency is valid */
	if(DA7212_SYS_FS_8K == dwNewFreqValue || DA7212_SYS_FS_16K == dwNewFreqValue || DA7212_SYS_FS_32K == dwNewFreqValue ||  DA7212_SYS_FS_44_1K == dwNewFreqValue \
			|| DA7212_SYS_FS_48K == dwNewFreqValue ||  DA7212_SYS_FS_96K == dwNewFreqValue )
	{
		/* Set the driver as busy */
		DA7212_SET_STATUS(DA7212_STATUS_BUSY_MASK);
		/* Clear configured flag since will be configured again */
		DA7212_CLEAR_STATUS(DA7212_STATUS_CONFIGURED_MASK);

		/* set up the register and values table to be used */
		if(DA7212_SYS_FS_44_1K == dwNewFreqValue || DA7212_SYS_FS_44_1K == DA7212_gSysFs)
		{
			/* the amount of registers to write changes when 44.1hz is selected */
			DA7212_gRegistersToWrite = DA7212_441_PLL_REGISTERS_TO_WRITE;
		}
		else
		{
			DA7212_gRegistersToWrite = DA7212_16KHZ_32KHZ_48KHZ_96KHZ_PLL_REGISTERS_TO_WRITE;
		}

		/* set the new frequency */
		DA7212_gSysFs = dwNewFreqValue;

		/* set up the write block registers to the proper tables */
		DA7212_gWriteBlockRegister = (uint8_t*)&DA7212_gChangeFreqRegisters[0];
		DA7212_gWriteBlockRegisterValue = (uint8_t*)&DA7212_gchangeFreqValues[dwNewFreqValue][0];
		DA7212_gRegisterOffset = 0;


		DA7212_gStates.actualState = DA7212_WRITE_BLOCK_STATE;
		DA7212_gStates.nextState = DA7212_CONTROL_STATE_WAIT_COMM;


	}
	else
	{
		DA7212_SET_STATUS(DA7212_STATUS_CONFIGURED_MASK);
		DA7212_gStates.actualState = DA7212_CONTROL_STATE_IDLE;
		DA7212_gStates.nextState = DA7212_CONTROL_STATE_IDLE;
	}
	bStatus = OK;

#else
	iAPSSIMaster_ChangeFreq(dwNewFreqValue);
#endif

	return(bStatus);
}

static void DA7212_WaitCommState(void)
{
    if (s_i2cTransferCompleted)
    {
        if (s_i2cTransferStatus == kStatus_Success)
        {
			DA7212_gStates.actualState = DA7212_gStates.nextState;
        }
        else
        {
			/* a NACK or time out was received*/
			DA7212_SET_STATUS(DA7212_STATUS_I2C_ERROR_MASK);
			DA7212_gStates.actualState = DA7212_gStates.nextState;
        }
    }

	/*Check for the physical layer driver to be free*/
// 	if(!(I2C_DRV_MasterGetSendStatus(I2C1_instance, &bytesRemaining)==kStatus_I2C_Busy))
// 	{
// 		if((I2C_DRV_MasterGetSendStatus(I2C1_instance, &bytesRemaining)!=kStatus_I2C_ReceivedNak)&&(I2C_DRV_MasterGetSendStatus(I2C1_instance, &bytesRemaining)!=kStatus_I2C_Timeout))
// 		{
// 			DA7212_gStates.actualState = DA7212_gStates.nextState;
// 		}
// 		else
// 		{
// 			/* a NACK or time out was received*/
// 			DA7212_SET_STATUS(DA7212_STATUS_I2C_ERROR_MASK);
// 			DA7212_gStates.actualState = DA7212_gStates.nextState;
// 		}
// 	}
}


static void DA7212_WriteBlockState(void)
{
	/*retries if the I2C driver reported an error*/
	if(DA7212_CHECK_STATUS(DA7212_STATUS_I2C_ERROR_MASK))
	{
		DA7212_gRegisterOffset--;
		DA7212_CLEAR_STATUS(DA7212_STATUS_I2C_ERROR_MASK);
	}

	/* Check if there is any other write registers */
	if(DA7212_gRegisterOffset < DA7212_gRegistersToWrite)
	{
		/* Take the register and the data to be written */
        DA7212_WriteRegister(DA7212_gWriteBlockRegister[DA7212_gRegisterOffset], DA7212_gWriteBlockRegisterValue[DA7212_gRegisterOffset]);

		DA7212_gStates.actualState = DA7212_CONTROL_STATE_WAIT_COMM;
		DA7212_gStates.nextState = DA7212_WRITE_BLOCK_STATE;
		DA7212_gRegisterOffset++;
	}
	else
	{
         /* if this isn't the startup block write */
		 if(DA7212_CHECK_STATUS(DA7212_STATUS_STARTUP_DONE_MASK))
         {
			DA7212_SET_STATUS(DA7212_STATUS_CONFIGURED_MASK);
			DA7212_gStates.actualState = DA7212_CONTROL_STATE_IDLE;
         }
         else
         {
        	/* a >40ms delay is required after initialization to enable the LDO 						*/
			/* From Dialog Apps Engineer: This allows the bandgap internal voltage reference to settle	*/

            s_startTime = ar_get_millisecond_count();

        	/* reserve a timer and start it */
//  			gbDA7212_gLDOTimer = SWTimer_AllocateChannel(DA7212_50MS_TIMEOUT,DA7212_SWTimerCallback);
//  			SWTimer_EnableTimer(gbDA7212_gLDOTimer);

			/* set the startup done mask */
			DA7212_SET_STATUS(DA7212_STATUS_STARTUP_DONE_MASK);

			/* wait for the delay */
			DA7212_gStates.actualState = DA7212_CONTROL_STATE_WAIT_TIMEOUT;
         }

		 /* When finished, set IDLE state to Actual and Next states */
		 DA7212_gRegisterOffset = 0;
	}

}

static void DA7212_TimeoutState(void)
{
    uint32_t now = ar_get_millisecond_count();

	/* poll the SW timer */
// 	if(DA7212_TimeOutFlag)
    if (now - s_startTime >= 40)
	{
		/* once the SW timer has expired, enable the LDO*/
		DA7212_WriteRegister(DIALOG7212_LDO_CTRL,DIALOG7212_LDO_CTRL_EN_MASK);

		/* the timer won't be needed, release it */
// 		DA7212_TimeOutFlag = 0;
// 		SWTimer_DisableTimer(gbDA7212_gLDOTimer);
		/* go to idle */
		DA7212_gStates.nextState = DA7212_CONTROL_STATE_IDLE;
	}
}

void DA7212_SWTimerCallback (void)
{
// 	DA7212_TimeOutFlag = 1;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
