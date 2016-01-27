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
#include "stmipid02.h"

extern uint8_t ov7251_debug[8];
uint8_t i2cread_stmipid02[20];
//#define READ_STMIPID02 1

/**
  * @brief  Configures the stmipid02 mipi to parallel bridge .
  */
void stmipid02_context_configuration(void)
{
	ov7251_debug[5]=1;
	// MAIN CAMERA CLOCK LANE 1 (CLKP1, CLKN1)
	stmipid02_WriteReg8(CLK_LANE_REG1, 0x15); // Enable Clock Lane 1 (CLKP1, CLKN1) and UI programmation:0x15 between 400 and 344Mhz, 0x25 between 222 and 200Mhz	
	stmipid02_WriteReg8(CLK_LANE_REG3, 0x02); // 0x1c CCP mode , 0x02 CSI mode on main camera
	
	// MAIN CAMERA DATA LANE 1.1 (DATA1P1, DATA1N1)
	stmipid02_WriteReg8(DATA_LANE0_REG1, 0x03); // 0x03 Enable Data Lane 1.1 (DATA1P1, DATA1N1), 0x00 disable Data lane 1.1
	stmipid02_WriteReg8(DATA_LANE0_REG2, 0x01); // 0x01 for CSI mode set on Data Lane 1.1 (DATA1P1, DATA1N1)
	
	// MAIN CAMERA DATA LANE 1.2 (DATA2P1, DATA2N1)
	stmipid02_WriteReg8(DATA_LANE1_REG1, 0x00); // 0x00 disable Data Lane 1.2 (DATA2P1, DATA2N1). CSI dual lane or second lane, Enable Data Lane 1.2: 0x01
	stmipid02_WriteReg8(DATA_LANE1_REG2, 0x01); // CSI dual lane or second lane, set CSI mode 0x01, CCP second lane, set CCP 0x0e

	// SECOND CAMERA CLOCK LANE 2 (CLKP2, CLKN2)
	stmipid02_WriteReg8(CLK_LANE_REG1_C2, 0x00); // Disable second camera: 0x00
	// SECOND CAMERA DATA LANE 2 (DATA1P2, DATA1N2)
	stmipid02_WriteReg8(DATA_LANE3_REG1, 0x00); // 0x00 Disable Data Lane2 , Enable Data lane 2: 0x01 (for CCP or CSI)
	
	// MODE CONTROL 
	// chris : MODE_REG1 compression?
	stmipid02_WriteReg8(MODE_REG1, 0x40); // No decompression: 0x40 for CSi single lane, 0x42 for CSi dual lane. Decompression mode please refer to datasheet/register map.
	stmipid02_WriteReg8(MODE_REG2, 0x00); // Normal (non Tristated output), continious clock, clock polarity and synchronization signals not inverted
	stmipid02_WriteReg8(MODE_REG3, 0x20); // Enable compensation macro, 0.90Rev of DPHY. 0x20 for main camera, 0x21 for second camera

	stmipid02_WriteReg8(DATA_ID_RREG, 0x2B);
	// Data type: 0x1E YUV422 8-bit, 0x1F YUV422 10-bit, 0x22 RGB565, 0x2A RAW8, 0x2B RAW10,
	// 0x2C RAW12, for other mode please refer to CSI specifications
	stmipid02_WriteReg8(DATA_ID_RREG_EMB, 0x2B);
	// Data type of embedded data: 0x1E YUV422 8-bit, 0x1F YUV422 10-bit, 0x22 RGB565, 0x2A RAW8,
	//0x2B RAW10, 0x2C RAW12, for other mode please refer to CSI specifications
	stmipid02_WriteReg8(DATA_SELECTION_CTRL, 0x0C);
	// 0X00 Data type and pixel width extracted from data stream, 0x04 Data type programmed, pixel width
	// extracted from data type, 0x0c Data type and pixel width programmed
	
	// PIXEL WIDTH and decompression ON/OFF
	stmipid02_WriteReg8(PIX_WIDTH_CTRL, 0x0A);
	// Image data not compressed: 0x06 for Raw6, 0x07 for Raw7, 0x08 for 8 bits, 0x0A for 10bits, 0x0c for
	// Raw12. Image data compressed:0x1a for 12-10, 0x18 for 12-8 and 10-8, 0x17 for 12-7 and 10-7, 0x16 for
	// 12-6 and 10-6
	stmipid02_WriteReg8(PIX_WIDTH_CTRL_EMB, 0x0A);
	// Embedded data not compressed: 0x06 for Raw6, 0x07 for Raw7, 0x08 for 8 bits, 0x0A for 10bits, 0x0c
	// for Raw12. Embedded data compressed:0x1a for 12-10, 0x18 for 12-8 and 10-8, 0x17 for 12-7 and 10-7,
	// 0x16 for 12-6 and 10-6

#ifdef READ_STMIPID02
	i2cread_stmipid02[0] = stmipid02_ReadReg8(CLK_LANE_REG1);
	i2cread_stmipid02[1] = stmipid02_ReadReg8(CLK_LANE_REG3);
	i2cread_stmipid02[2] = stmipid02_ReadReg8(DATA_LANE0_REG1);
	i2cread_stmipid02[3] = stmipid02_ReadReg8(DATA_LANE0_REG2);
	i2cread_stmipid02[4] = stmipid02_ReadReg8(DATA_LANE1_REG1);
	i2cread_stmipid02[5] = stmipid02_ReadReg8(CLK_LANE_REG1_C2);
	i2cread_stmipid02[6] = stmipid02_ReadReg8(DATA_LANE3_REG1);
	i2cread_stmipid02[7] = stmipid02_ReadReg8(MODE_REG1);
	i2cread_stmipid02[8] = stmipid02_ReadReg8(MODE_REG2);
	i2cread_stmipid02[9] = stmipid02_ReadReg8(MODE_REG3);
	i2cread_stmipid02[10] = stmipid02_ReadReg8(DATA_ID_RREG);
	i2cread_stmipid02[11] = stmipid02_ReadReg8(DATA_ID_RREG_EMB);
	i2cread_stmipid02[12] = stmipid02_ReadReg8(DATA_SELECTION_CTRL);
	i2cread_stmipid02[13] = stmipid02_ReadReg8(PIX_WIDTH_CTRL);
	i2cread_stmipid02[14] = stmipid02_ReadReg8(PIX_WIDTH_CTRL_EMB);
#endif

}

/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Addr: stmipid02 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t stmipid02_WriteReg(uint8_t Addr, uint8_t Data)
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
  I2C_Send7bitAddress(I2C2, stmipid02_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address MSB */
  I2C_SendData( I2C2, (uint8_t)(0x00) );

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Addr);

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
uint8_t stmipid02_WriteReg8(uint8_t address, uint8_t Data)
{
	uint8_t result = stmipid02_WriteReg(address, Data); // write upper byte
	//result |= stmipid02_WriteReg(0xF0, (uint8_t) Data); // write lower byte
	return result;
}

/**
  * @brief  Reads a byte from a specific Camera register
  * @param  Addr: stmipid02 register address.
  * @retval data read from the specific register or 0xFF if timeout condition
  *         occured.
  */
uint8_t stmipid02_ReadReg(uint8_t Addr)
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
  I2C_Send7bitAddress(I2C2, stmipid02_DEVICE_READ_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }
  
  /* Send I2C2 location address MSB */
  I2C_SendData( I2C2, (uint8_t)(0x00) );

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Addr);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Clear AF flag if arised */
  I2C2->SR1 |= (uint16_t)0x0400;

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
  I2C_Send7bitAddress(I2C2, stmipid02_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);

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
uint8_t stmipid02_ReadReg8(uint8_t address)
{
	uint8_t result = stmipid02_ReadReg(address); // read upper byte
	//result |= stmipid02_ReadReg(0xF0); // read lower byte
	return result;
}




