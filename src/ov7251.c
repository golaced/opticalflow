/****************************************************************************
 *
 *   Copyright (C) 2015 IAC Development Team. All rights reserved.
 *   Author: Chris Hsu <hsu.chris@iac.com.tw>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name IAC nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "ov7251.h"

extern uint8_t ov7251_debug[8];
uint8_t i2cread_ov7251[120];
//#define READ_OV7251 1

/**
  * @brief  Configures the ov7251 camera .
  */
void ov7251_context_configuration(void)
{
	ov7251_debug[4]=1;
	
	/* General Settings */
	//ov7251_WriteReg16(SC_MODE_SELECT, 0x00);
	ov7251_WriteReg16(SC_SW_RESET_REG, 0x01);
	ov7251_WriteReg16(SC_MODE_SELECT, 0x00);
	ov7251_WriteReg16(SC_REG5, 0x00);
	ov7251_WriteReg16(SC_MIPI_PHY, 0xC0);
	ov7251_WriteReg16(SC_MIPI_PHY_2, 0xD2);
	ov7251_WriteReg16(SC_MIPI_SC_CTRL0, 0x04);
	ov7251_WriteReg16(SC_CLKRST0, 0x10);
	ov7251_WriteReg16(SC_CLKRST1, 0x00);
	ov7251_WriteReg16(SC_CLKRST2, 0x00);
	ov7251_WriteReg16(SC_CLKRST4, 0x00);
	ov7251_WriteReg16(SC_CLKRST5, 0x00);
	ov7251_WriteReg16(SC_CLKRST6, 0x00);
	ov7251_WriteReg16(SC_LOW_PWR_CTRL, 0x05);
	ov7251_WriteReg16(SC_R37, 0xF0);
	
	ov7251_WriteReg16(PLL_PLL18, 0x04); //default
	ov7251_WriteReg16(PLL_PLL19, 0x28); //default
	ov7251_WriteReg16(PLL_PLL1A, 0x05); //default
	ov7251_WriteReg16(PLL_PLL1B, 0x04); //default
	ov7251_WriteReg16(PLL_VT_PIX_CLK_DIV, 0x0A); //default
	ov7251_WriteReg16(PLL_VT_SYS_CLK_DIV, 0x02); //default
	ov7251_WriteReg16(PLL_MULTIPLIER, 0x64);
	ov7251_WriteReg16(PLL_PLL1_PRE_PLL_DIV, 0x03);
	ov7251_WriteReg16(PLL_OP_PIX_CLK_DIV, 0x05);
	ov7251_WriteReg16(SB_SRB_CTRL, 0xDA);
	
	ov7251_WriteReg16(AEC_EXPO, 0x00);
	ov7251_WriteReg16(AEC_EXPO2, 0x0A);
	ov7251_WriteReg16(AEC_EXPO3, 0x00);
	ov7251_WriteReg16(AEC_MANUAL, 0x07);
	ov7251_WriteReg16(AEC_GAIN_CONVERT, 0x10);
	ov7251_WriteReg16(AEC_AGC_ADJ, 0x10);

	//analog control registers
	ov7251_WriteReg16(ANALOG_REG, 0x1C);
	ov7251_WriteReg16(ANALOG_REG2, 0x62);
	ov7251_WriteReg16(ANALOG_REG3, 0xB7);
	ov7251_WriteReg16(ANALOG_REG4, 0x04);
	ov7251_WriteReg16(ANALOG_REG5, 0x21);
	ov7251_WriteReg16(ANALOG_REG6, 0x30);
	ov7251_WriteReg16(ANALOG_REG7, 0x44);
	ov7251_WriteReg16(ANALOG_REG8, 0x35);
	ov7251_WriteReg16(ANALOG_REG9, 0x60);
	ov7251_WriteReg16(ANALOG_REG10, 0x00);
	ov7251_WriteReg16(ANALOG_REG11, 0x01);
	ov7251_WriteReg16(ANALOG_REG12, 0x70);
	ov7251_WriteReg16(ANALOG_REG13, 0xF0);
	ov7251_WriteReg16(ANA_CORE6, 0x0A);
	ov7251_WriteReg16(ANALOG_REG14, 0x1A);
	ov7251_WriteReg16(ANALOG_REG15, 0x00);
	ov7251_WriteReg16(ANALOG_REG16, 0x50);
	ov7251_WriteReg16(ANALOG_REG17, 0x01);
	ov7251_WriteReg16(ANALOG_REG18, 0xFF);
	ov7251_WriteReg16(ANALOG_REG19, 0x03);

	//sensor control registers
	ov7251_WriteReg16(SENSOR_CONTROL_REG, 0x41);
	ov7251_WriteReg16(SENSOR_CONTROL_REG2, 0x40);
	ov7251_WriteReg16(SENSOR_CONTROL_REG3, 0x08);
	ov7251_WriteReg16(SENSOR_CONTROL_REG4, 0xE0);
	ov7251_WriteReg16(SENSOR_CONTROL_REG5, 0xB3);
	ov7251_WriteReg16(SENSOR_CONTROL_REG6, 0x00);
	ov7251_WriteReg16(FIFO_CTRL0_H, 0x02);
	ov7251_WriteReg16(FIFO_CTRL0_L, 0x8C);

	//timing control registers
	ov7251_WriteReg16(TIMING_X_ADDR_START, 0x00); //default  00
	ov7251_WriteReg16(TIMING_X_ADDR_START2, 0x04); //default  04
	ov7251_WriteReg16(TIMING_Y_ADDR_START, 0x00); //default  00
	ov7251_WriteReg16(TIMING_Y_ADDR_START2, 0x00); //???????????  00
	ov7251_WriteReg16(TIMING_X_ADDR_END, 0x02); //default  02
	ov7251_WriteReg16(TIMING_X_ADDR_END2, 0x8B); //default  8B
	ov7251_WriteReg16(TIMING_Y_ADDR_END, 0x01); //default  01
	ov7251_WriteReg16(TIMING_Y_ADDR_END2, 0xEF); //??????????  EF
	
	ov7251_WriteReg16(TIMING_X_OUTPUT_SIZE, 0x00); //0x00A0 => 160  //0x0050 => 80
	ov7251_WriteReg16(TIMING_X_OUTPUT_SIZE2, 0xA0);
	ov7251_WriteReg16(TIMING_Y_OUTPUT_SIZE, 0x00); //0x0078 => 120  //0x003C => 60
	ov7251_WriteReg16(TIMING_Y_OUTPUT_SIZE2, 0x78);
	
	ov7251_WriteReg16(TIMING_HTS, 0x03); //default
	ov7251_WriteReg16(TIMING_HTS2, 0x04); //default
	ov7251_WriteReg16(TIMING_VTS, 0x00);
	ov7251_WriteReg16(TIMING_VTS2, 0xAD);	
	ov7251_WriteReg16(TIMING_ISP_X_WIN, 0x00);
	ov7251_WriteReg16(TIMING_ISP_X_WIN2, 0x04);
	ov7251_WriteReg16(TIMING_ISP_Y_WIN, 0x00);
	ov7251_WriteReg16(TIMING_ISP_Y_WIN2, 0x03);
	ov7251_WriteReg16(TIMING_X_INC, 0x44);
	ov7251_WriteReg16(TIMING_Y_INC, 0x44);
	ov7251_WriteReg16(TIMING_FORMAT1, 0x40);
	ov7251_WriteReg16(TIMING_FORMAT2, 0x00);
	ov7251_WriteReg16(TIMING_REG2F, 0x0E);
	ov7251_WriteReg16(TIMING_REG32, 0x00);
	ov7251_WriteReg16(TIMING_REG33, 0x05);
	ov7251_WriteReg16(TIMING_REG34, 0x00);
	ov7251_WriteReg16(TIMING_REG35, 0x0C);
	ov7251_WriteReg16(DIGITAL_BINNING_CTRL, 0x00); //default

	ov7251_WriteReg16(LED_PWM_REG00, 0x00);//0x00,
	ov7251_WriteReg16(LED_PWM_REG01, 0xa5);//0xa5
	ov7251_WriteReg16(LED_PWM_REG02, 0x10);//0x10,
	ov7251_WriteReg16(LED_PWM_REG03, 0x00);//0x00,
	ov7251_WriteReg16(LED_PWM_REG04, 0x08);//0x08,
	ov7251_WriteReg16(LED_PWM_REG05, 0x00);//0x00,
	ov7251_WriteReg16(LED_PWM_REG06, 0x01);//0x01,
	ov7251_WriteReg16(LED_PWM_REG07, 0x00);//0x00,
	ov7251_WriteReg16(LED_PWM_REG08, 0x00);//0x00,
	ov7251_WriteReg16(LED_PWM_REG09, 0x00);//0x00,
	ov7251_WriteReg16(LED_PWM_REG0A, 0x00);//0x00
	ov7251_WriteReg16(LED_PWM_REG0B, 0x05);//0x05
	ov7251_WriteReg16(LED_PWM_REG0C, 0x00);//0x00
	ov7251_WriteReg16(LED_PWM_REG0D, 0x00);//0x00
	ov7251_WriteReg16(LED_PWM_REG0E, 0x00);//0x00,
	ov7251_WriteReg16(LED_PWM_REG0F, 0x1a);//0x1a,
	ov7251_WriteReg16(LED_PWM_REG14, 0x05);//0x05,
	ov7251_WriteReg16(LED_PWM_REG15, 0xf2);//0xf2,
	ov7251_WriteReg16(LED_PWM_REG16, 0x40);//0x40,

	//lowpower mode control
	ov7251_WriteReg16(LOWPWR00, 0x89); //default
	ov7251_WriteReg16(LOWPWR01, 0x63); //0x63:low power mode, 0xAB:normal mode
	ov7251_WriteReg16(LOWPWR02, 0x01); //0x01:enable low power streaming mode
	ov7251_WriteReg16(LOWPWR03, 0x00); //0x00:low frame rate streaming mode
	ov7251_WriteReg16(LOWPWR04, 0x00);
	ov7251_WriteReg16(LOWPWR05, 0x03); //number of active frames
	ov7251_WriteReg16(LOWPWR06, 0x00);
	ov7251_WriteReg16(LOWPWR07, 0x06);
	ov7251_WriteReg16(LOWPWR0C, 0x01); //row period in units of input clock period : 0x0182=>386
	ov7251_WriteReg16(LOWPWR0D, 0x82);
	ov7251_WriteReg16(LOWPWR0E, 0x00); //number of rows per base frame : 0xAD=>173
	ov7251_WriteReg16(LOWPWR0F, 0xAD);
	
	ov7251_WriteReg16(BLC_CTRL01, 0x40);
	ov7251_WriteReg16(BLC_NUM, 0x02);
	ov7251_WriteReg16(BLC_MAN_CTRL, 0x00);
	ov7251_WriteReg16(BLC_AVG, 0x01);
	ov7251_WriteReg16(DATA_MAX_H, 0xFF); //default
	ov7251_WriteReg16(DATA_MIN_H, 0x00); //default
	ov7251_WriteReg16(SC_REG1501, 0x48);
	ov7251_WriteReg16(READ_START_H, 0x00);
	ov7251_WriteReg16(READ_START_L, 0x4E);	
	ov7251_WriteReg16(MIPI_CTRL01, 0x0F);
	ov7251_WriteReg16(MIPI_CTRL06, 0x0F);
	ov7251_WriteReg16(HS_ZERO_MIN, 0xAA);
	ov7251_WriteReg16(CLK_TRAIL_MIN, 0x3E);
	ov7251_WriteReg16(PCLK_PERIOD, 0x19);
	ov7251_WriteReg16(DEBUG_CTRL, 0x00);

	ov7251_WriteReg16(LOWPWR_CTRL_REG, 0x7F);//0x7f,
	ov7251_WriteReg16(LOWPWR_CTRL_REG2, 0xF0);//0xf0,

	ov7251_WriteReg16(TRULYCONFIG1, 0x30);//0x30,

	ov7251_WriteReg16(ISP_CTRL00, 0x85);
	ov7251_WriteReg16(ISP_CTRL01, 0x80);
/*
	ov7251_WriteReg16(0x5E00, 0x8C);
	ov7251_WriteReg16(0x4320, 0x82);
	ov7251_WriteReg16(0x4323, 0x14);
	ov7251_WriteReg16(0x4325, 0x25);
	ov7251_WriteReg16(0x4327, 0x36);
	ov7251_WriteReg16(0x4329, 0x47);
*/
	ov7251_WriteReg16(SC_MODE_SELECT, 0x01);


	delay(1);

#ifdef READ_OV7251	
	i2cread_ov7251[0] = ov7251_ReadReg16(SC_SW_RESET_REG);
	i2cread_ov7251[1] = ov7251_ReadReg16(SC_MODE_SELECT);
	i2cread_ov7251[2] = ov7251_ReadReg16(SC_REG5);
	i2cread_ov7251[3] = ov7251_ReadReg16(SC_MIPI_PHY);
	i2cread_ov7251[4] = ov7251_ReadReg16(SC_MIPI_PHY_2);
	i2cread_ov7251[5] = ov7251_ReadReg16(SC_MIPI_SC_CTRL0);
	i2cread_ov7251[6] = ov7251_ReadReg16(SC_CLKRST0);
	i2cread_ov7251[7] = ov7251_ReadReg16(SC_CLKRST1);
	i2cread_ov7251[8] = ov7251_ReadReg16(SC_CLKRST2);
	i2cread_ov7251[9] = ov7251_ReadReg16(SC_CLKRST4);
	i2cread_ov7251[10] = ov7251_ReadReg16(SC_CLKRST5);
	i2cread_ov7251[11] = ov7251_ReadReg16(SC_CLKRST6);
	i2cread_ov7251[12] = ov7251_ReadReg16(SC_LOW_PWR_CTRL);
	i2cread_ov7251[13] = ov7251_ReadReg16(SC_R37);
	i2cread_ov7251[14] = ov7251_ReadReg16(PLL_PLL18);
	i2cread_ov7251[15] = ov7251_ReadReg16(PLL_PLL19);

	i2cread_ov7251[16] = ov7251_ReadReg16(PLL_PLL1A);
	i2cread_ov7251[17] = ov7251_ReadReg16(PLL_PLL1B);
	i2cread_ov7251[18] = ov7251_ReadReg16(PLL_VT_PIX_CLK_DIV);
	i2cread_ov7251[19] = ov7251_ReadReg16(PLL_VT_SYS_CLK_DIV);
	i2cread_ov7251[20] = ov7251_ReadReg16(PLL_MULTIPLIER);
	i2cread_ov7251[21] = ov7251_ReadReg16(PLL_PLL1_PRE_PLL_DIV);
	i2cread_ov7251[22] = ov7251_ReadReg16(PLL_OP_PIX_CLK_DIV);
	i2cread_ov7251[23] = ov7251_ReadReg16(SB_SRB_CTRL);
	i2cread_ov7251[24] = ov7251_ReadReg16(AEC_EXPO);
	i2cread_ov7251[25] = ov7251_ReadReg16(AEC_EXPO2);
	i2cread_ov7251[26] = ov7251_ReadReg16(AEC_EXPO3);
	i2cread_ov7251[27] = ov7251_ReadReg16(AEC_MANUAL);
	i2cread_ov7251[28] = ov7251_ReadReg16(AEC_GAIN_CONVERT);
	i2cread_ov7251[29] = ov7251_ReadReg16(AEC_AGC_ADJ);
	i2cread_ov7251[30] = ov7251_ReadReg16(ANALOG_REG);
	i2cread_ov7251[31] = ov7251_ReadReg16(ANALOG_REG2);

	i2cread_ov7251[32] = ov7251_ReadReg16(ANALOG_REG3);
	i2cread_ov7251[33] = ov7251_ReadReg16(ANALOG_REG4);
	i2cread_ov7251[34] = ov7251_ReadReg16(ANALOG_REG5);
	i2cread_ov7251[35] = ov7251_ReadReg16(ANALOG_REG6);
	i2cread_ov7251[36] = ov7251_ReadReg16(ANALOG_REG7);
	i2cread_ov7251[37] = ov7251_ReadReg16(ANALOG_REG8);
	i2cread_ov7251[38] = ov7251_ReadReg16(ANALOG_REG9);
	i2cread_ov7251[39] = ov7251_ReadReg16(ANALOG_REG10);
	i2cread_ov7251[40] = ov7251_ReadReg16(ANALOG_REG11);
	i2cread_ov7251[41] = ov7251_ReadReg16(ANALOG_REG12);
	i2cread_ov7251[42] = ov7251_ReadReg16(ANALOG_REG13);
	i2cread_ov7251[43] = ov7251_ReadReg16(ANA_CORE6);
	i2cread_ov7251[44] = ov7251_ReadReg16(ANALOG_REG14);
	i2cread_ov7251[45] = ov7251_ReadReg16(ANALOG_REG15);
	i2cread_ov7251[46] = ov7251_ReadReg16(ANALOG_REG16);
	i2cread_ov7251[47] = ov7251_ReadReg16(ANALOG_REG17);
	
	i2cread_ov7251[48] = ov7251_ReadReg16(ANALOG_REG18);
	i2cread_ov7251[49] = ov7251_ReadReg16(ANALOG_REG19);
	i2cread_ov7251[50] = ov7251_ReadReg16(SENSOR_CONTROL_REG);
	i2cread_ov7251[51] = ov7251_ReadReg16(SENSOR_CONTROL_REG2);
	i2cread_ov7251[52] = ov7251_ReadReg16(SENSOR_CONTROL_REG3);
	i2cread_ov7251[53] = ov7251_ReadReg16(SENSOR_CONTROL_REG4);
	i2cread_ov7251[54] = ov7251_ReadReg16(SENSOR_CONTROL_REG5);
	i2cread_ov7251[55] = ov7251_ReadReg16(SENSOR_CONTROL_REG6);
	i2cread_ov7251[56] = ov7251_ReadReg16(FIFO_CTRL0_H);
	i2cread_ov7251[57] = ov7251_ReadReg16(FIFO_CTRL0_L);
	i2cread_ov7251[58] = ov7251_ReadReg16(TIMING_X_ADDR_START);
	i2cread_ov7251[59] = ov7251_ReadReg16(TIMING_X_ADDR_START2);
	i2cread_ov7251[60] = ov7251_ReadReg16(TIMING_Y_ADDR_START);
	i2cread_ov7251[61] = ov7251_ReadReg16(TIMING_Y_ADDR_START2);
	i2cread_ov7251[62] = ov7251_ReadReg16(TIMING_X_ADDR_END);
	i2cread_ov7251[63] = ov7251_ReadReg16(TIMING_X_ADDR_END2);

	i2cread_ov7251[64] = ov7251_ReadReg16(TIMING_Y_ADDR_END);
	i2cread_ov7251[65] = ov7251_ReadReg16(TIMING_Y_ADDR_END2);
	i2cread_ov7251[66] = ov7251_ReadReg16(TIMING_X_OUTPUT_SIZE);
	i2cread_ov7251[67] = ov7251_ReadReg16(TIMING_X_OUTPUT_SIZE2);
	i2cread_ov7251[68] = ov7251_ReadReg16(TIMING_Y_OUTPUT_SIZE);
	i2cread_ov7251[69] = ov7251_ReadReg16(TIMING_Y_OUTPUT_SIZE2);
	i2cread_ov7251[70] = ov7251_ReadReg16(TIMING_HTS);
	i2cread_ov7251[71] = ov7251_ReadReg16(TIMING_HTS2);
	i2cread_ov7251[72] = ov7251_ReadReg16(TIMING_VTS);
	i2cread_ov7251[73] = ov7251_ReadReg16(TIMING_VTS2);
	i2cread_ov7251[74] = ov7251_ReadReg16(TIMING_ISP_X_WIN);
	i2cread_ov7251[75] = ov7251_ReadReg16(TIMING_ISP_X_WIN2);
	i2cread_ov7251[76] = ov7251_ReadReg16(TIMING_ISP_Y_WIN);
	i2cread_ov7251[77] = ov7251_ReadReg16(TIMING_ISP_Y_WIN2);
	i2cread_ov7251[78] = ov7251_ReadReg16(TIMING_X_INC);
	i2cread_ov7251[79] = ov7251_ReadReg16(TIMING_Y_INC);

	i2cread_ov7251[80] = ov7251_ReadReg16(TIMING_FORMAT1);
	i2cread_ov7251[81] = ov7251_ReadReg16(TIMING_FORMAT2);
	i2cread_ov7251[82] = ov7251_ReadReg16(TIMING_REG2F);
	i2cread_ov7251[83] = ov7251_ReadReg16(TIMING_REG32);
	i2cread_ov7251[84] = ov7251_ReadReg16(TIMING_REG33);
	i2cread_ov7251[85] = ov7251_ReadReg16(TIMING_REG34);
	i2cread_ov7251[86] = ov7251_ReadReg16(TIMING_REG35);
	i2cread_ov7251[87] = ov7251_ReadReg16(DIGITAL_BINNING_CTRL);

	i2cread_ov7251[88] = ov7251_ReadReg16(LOWPWR00);
	i2cread_ov7251[89] = ov7251_ReadReg16(LOWPWR01);
	i2cread_ov7251[90] = ov7251_ReadReg16(LOWPWR02);
	i2cread_ov7251[91] = ov7251_ReadReg16(LOWPWR03);
	i2cread_ov7251[92] = ov7251_ReadReg16(LOWPWR04);
	i2cread_ov7251[93] = ov7251_ReadReg16(LOWPWR05);
	i2cread_ov7251[94] = ov7251_ReadReg16(LOWPWR06);
	i2cread_ov7251[95] = ov7251_ReadReg16(LOWPWR07);
	i2cread_ov7251[96] = ov7251_ReadReg16(LOWPWR0C);
	i2cread_ov7251[97] = ov7251_ReadReg16(LOWPWR0D);
	i2cread_ov7251[98] = ov7251_ReadReg16(LOWPWR0E);
	i2cread_ov7251[99] = ov7251_ReadReg16(LOWPWR0F);
	i2cread_ov7251[100] = ov7251_ReadReg16(BLC_CTRL01);
	i2cread_ov7251[101] = ov7251_ReadReg16(BLC_NUM);
	i2cread_ov7251[102] = ov7251_ReadReg16(BLC_MAN_CTRL);
	i2cread_ov7251[103] = ov7251_ReadReg16(BLC_AVG);

	i2cread_ov7251[104] = ov7251_ReadReg16(DATA_MAX_H);
	i2cread_ov7251[105] = ov7251_ReadReg16(DATA_MIN_H);
	i2cread_ov7251[106] = ov7251_ReadReg16(SC_REG1501);
	i2cread_ov7251[107] = ov7251_ReadReg16(READ_START_H);
	i2cread_ov7251[108] = ov7251_ReadReg16(READ_START_L);
	i2cread_ov7251[109] = ov7251_ReadReg16(MIPI_CTRL01);
	i2cread_ov7251[110] = ov7251_ReadReg16(MIPI_CTRL06);
	i2cread_ov7251[111] = ov7251_ReadReg16(HS_ZERO_MIN);
	i2cread_ov7251[112] = ov7251_ReadReg16(CLK_TRAIL_MIN);
	i2cread_ov7251[113] = ov7251_ReadReg16(PCLK_PERIOD);
	i2cread_ov7251[114] = ov7251_ReadReg16(DEBUG_CTRL);
	i2cread_ov7251[115] = ov7251_ReadReg16(ISP_CTRL00);
	i2cread_ov7251[116] = ov7251_ReadReg16(ISP_CTRL01);
#endif
}

/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Addr: ov7251 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t ov7251_WriteReg(uint16_t Addr, uint8_t Data)
{
  uint32_t timeout = TIMEOUT_MAX;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, ov7251_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address MSB */
  I2C_SendData( I2C2, (uint8_t)(0xFF&(Addr>>8)) );

   /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, (uint8_t)(0xFF&Addr));

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send Data */
  I2C_SendData(I2C2, Data);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 STOP Condition */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* If operation is OK, return 0 */
  return 0;
}

/**
  * @brief  Writes to a specific Camera register
  */
uint8_t ov7251_WriteReg16(uint16_t address, uint8_t Data)
{
	uint8_t result = ov7251_WriteReg(address, Data); // write upper byte
	//result |= ov7251_WriteReg(0xF0, (uint8_t) Data); // write lower byte
	return result;
}

/**
  * @brief  Reads a byte from a specific Camera register
  * @param  Addr: ov7251 register address.
  * @retval data read from the specific register or 0xFF if timeout condition
  *         occured.
  */
uint8_t ov7251_ReadReg(uint16_t Addr)
{
  uint32_t timeout = TIMEOUT_MAX;
  uint8_t Data = 0;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, ov7251_DEVICE_READ_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address MSB */
  I2C_SendData(I2C2, (uint8_t)(0xFF & (Addr>>8)));
  
   /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, (uint8_t)(0xFF & Addr));

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Clear AF flag if arised */
  I2C2->SR1 |= (uint16_t)0x0400;

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, ov7251_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Prepare an NACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, DISABLE);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Receive the Data */
  Data = I2C_ReceiveData(I2C2);

  /* return the read data */
  return Data;
}

/**
  * @brief  Reads from a specific Camera register
  */
uint8_t ov7251_ReadReg16(uint16_t address)
{
	uint8_t result = ov7251_ReadReg(address); // read upper byte
	//result |= ov7251_ReadReg(0xF0); // read lower byte
	return result;
}

