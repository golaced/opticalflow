#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "ov7251.h"

/**
  * @brief  Configures the ov7251 camera .
  */
void ov7251_context_configuration(void)
{
	/* image dimentions */
	uint16_t new_width_context_a  = global_data.param[PARAM_IMAGE_WIDTH] * 4; // windowing off, row + col bin reduce size
	uint16_t new_height_context_a = global_data.param[PARAM_IMAGE_HEIGHT] * 4;
	uint16_t new_width_context_b  = OV7251_FULL_IMAGE_ROW_SIZE * 4; // windowing off, row + col bin reduce size
	uint16_t new_height_context_b = OV7251_FULL_IMAGE_COLUMN_SIZE * 4;
	
	/* General Settings */
	ov7251_WriteReg16(SC_SW_RESET_REG, 0x01);
	ov7251_WriteReg16(SC_MODE_SELECT, 0x00);
	ov7251_WriteReg16(SC_REG5, 0x00);
	ov7251_WriteReg16(SC_MIPI_PHY, 0xc0);
	ov7251_WriteReg16(SC_MIPI_PHY_2, 0xd2);
	ov7251_WriteReg16(SC_MIPI_SC_CTRL0, 0x04);
	ov7251_WriteReg16(SC_CLKRST0, 0x00);
	ov7251_WriteReg16(SC_CLKRST1, 0x00);
	ov7251_WriteReg16(SC_CLKRST2, 0x00);
	ov7251_WriteReg16(SC_CLKRST4, 0x00);
	ov7251_WriteReg16(SC_CLKRST5, 0x00);
	ov7251_WriteReg16(SC_CLKRST6, 0x00);
	ov7251_WriteReg16(SC_LOW_PWR_CTRL, 0x05);
	ov7251_WriteReg16(SC_R37, 0xf0);
	
	ov7251_WriteReg16(PLL_PLL18, 0x04);
	ov7251_WriteReg16(PLL_PLL19, 0x28);
	ov7251_WriteReg16(PLL_PLL1A, 0x05);
	ov7251_WriteReg16(PLL_PLL1B, 0x04);
	ov7251_WriteReg16(PLL_VT_PIX_CLK_DIV, 0x0a);
	ov7251_WriteReg16(PLL_VT_SYS_CLK_DIV, 0x01);
	ov7251_WriteReg16(PLL_MULTIPLIER, 0x64);
	ov7251_WriteReg16(PLL_PLL1_PRE_PLL_DIV, 0x03);
	ov7251_WriteReg16(PLL_OP_PIX_CLK_DIV, 0x05);
	ov7251_WriteReg16(SB_SRB_CTRL, 0xda);
	
	ov7251_WriteReg16(AEC_EXPO, 0x00);
	ov7251_WriteReg16(AEC_EXPO2, 0x0a);
	ov7251_WriteReg16(AEC_EXPO3, 0x00);
	ov7251_WriteReg16(AEC_MANUAL, 0x07);
	ov7251_WriteReg16(AEC_GAIN_CONVERT, 0x10);
	ov7251_WriteReg16(AEC_AGC_ADJ, 0x10);
	
	ov7251_WriteReg16(ANALOG_REG, 0x1c);
	ov7251_WriteReg16(ANALOG_REG2, 0x62);
	ov7251_WriteReg16(ANALOG_REG3, 0xb7);
	ov7251_WriteReg16(ANALOG_REG4, 0x04);
	ov7251_WriteReg16(ANALOG_REG5, 0x21);
	ov7251_WriteReg16(ANALOG_REG6, 0x30);
	ov7251_WriteReg16(ANALOG_REG7, 0x44);
	ov7251_WriteReg16(ANALOG_REG8, 0x35);
	ov7251_WriteReg16(ANALOG_REG9, 0x60);
	ov7251_WriteReg16(ANALOG_REG10, 0x00);
	ov7251_WriteReg16(ANALOG_REG11, 0x01);
	ov7251_WriteReg16(ANALOG_REG12, 0x70);
	ov7251_WriteReg16(ANALOG_REG13, 0xf0);
	ov7251_WriteReg16(ANA_CORE6, 0x0a);
	ov7251_WriteReg16(ANALOG_REG14, 0x1a);
	ov7251_WriteReg16(ANALOG_REG15, 0x00);
	ov7251_WriteReg16(ANALOG_REG16, 0x50);
	ov7251_WriteReg16(ANALOG_REG17, 0x01);
	ov7251_WriteReg16(ANALOG_REG18, 0xff);
	ov7251_WriteReg16(ANALOG_REG19, 0x03);
	
	ov7251_WriteReg16(SENSOR_CONTROL_REG, 0x41);
	ov7251_WriteReg16(SENSOR_CONTROL_REG2, 0x40);
	ov7251_WriteReg16(SENSOR_CONTROL_REG3, 0x08);
	ov7251_WriteReg16(SENSOR_CONTROL_REG4, 0xe0);
	ov7251_WriteReg16(SENSOR_CONTROL_REG5, 0xb3);
	ov7251_WriteReg16(SENSOR_CONTROL_REG6, 0x00);
	ov7251_WriteReg16(FIFO_CTRL0_H, 0x02);
	ov7251_WriteReg16(FIFO_CTRL0_L, 0x8c);
	
	ov7251_WriteReg16(TIMING_X_ADDR_START, 0x00);
	ov7251_WriteReg16(TIMING_X_ADDR_START2, 0x04);
	ov7251_WriteReg16(TIMING_Y_ADDR_START, 0x00);
	ov7251_WriteReg16(TIMING_Y_ADDR_START2, 0x00);
	ov7251_WriteReg16(TIMING_X_ADDR_END, 0x02);
	ov7251_WriteReg16(TIMING_X_ADDR_END2, 0x8b);
	ov7251_WriteReg16(TIMING_Y_ADDR_END, 0x01);
	ov7251_WriteReg16(TIMING_Y_ADDR_END2, 0xef);
	ov7251_WriteReg16(TIMING_X_OUTPUT_SIZE, 0x00);
	ov7251_WriteReg16(TIMING_X_OUTPUT_SIZE2, 0xa0);
	ov7251_WriteReg16(TIMING_Y_OUTPUT_SIZE, 0x00);
	ov7251_WriteReg16(TIMING_Y_OUTPUT_SIZE2, 0x78);
	ov7251_WriteReg16(TIMING_HTS, 0x03);
	ov7251_WriteReg16(TIMING_HTS2, 0x04);
	ov7251_WriteReg16(TIMING_VTS, 0x00);
	ov7251_WriteReg16(TIMING_VTS2, 0xad);	
	ov7251_WriteReg16(TIMING_ISP_X_WIN, 0x00);
	ov7251_WriteReg16(TIMING_ISP_X_WIN2, 0x04);
	ov7251_WriteReg16(TIMING_ISP_Y_WIN, 0x00);
	ov7251_WriteReg16(TIMING_ISP_Y_WIN2, 0x03);
	ov7251_WriteReg16(TIMING_X_INC, 0x44);
	ov7251_WriteReg16(TIMING_Y_INC, 0x44);
	ov7251_WriteReg16(TIMING_FORMAT1, 0x40);
	ov7251_WriteReg16(TIMING_FORMAT2, 0x00);
	ov7251_WriteReg16(TIMING_REG2F, 0x0e);
	ov7251_WriteReg16(TIMING_REG32, 0x00);
	ov7251_WriteReg16(TIMING_REG33, 0x05);
	ov7251_WriteReg16(TIMING_REG34, 0x00);
	ov7251_WriteReg16(TIMING_REG35, 0x0c);
	ov7251_WriteReg16(DIGITAL_BINNING_CTRL, 0x00);
	
	ov7251_WriteReg16(LOWPWR00, 0x89);
	ov7251_WriteReg16(LOWPWR01, 0x63);
	ov7251_WriteReg16(LOWPWR02, 0x01);
	ov7251_WriteReg16(LOWPWR03, 0x00);
	ov7251_WriteReg16(LOWPWR04, 0x00);
	ov7251_WriteReg16(LOWPWR05, 0x03);
	ov7251_WriteReg16(LOWPWR06, 0x00);
	ov7251_WriteReg16(LOWPWR07, 0x06);
	ov7251_WriteReg16(LOWPWR0C, 0x01);
	ov7251_WriteReg16(LOWPWR0D, 0x82);
	ov7251_WriteReg16(LOWPWR0E, 0x00);
	ov7251_WriteReg16(LOWPWR0F, 0xad);
	
	ov7251_WriteReg16(BLC_CTRL01, 0x40);
	ov7251_WriteReg16(BLC_NUM, 0x02);
	ov7251_WriteReg16(BLC_MAN_CTRL, 0x00);
	ov7251_WriteReg16(BLC_AVG, 0x01);
	ov7251_WriteReg16(DATA_MAX_H, 0xff);
	ov7251_WriteReg16(DATA_MIN_H, 0x00);
	ov7251_WriteReg16(SC_REG1501, 0x48);
	ov7251_WriteReg16(READ_START_H, 0x00);
	ov7251_WriteReg16(READ_START_L, 0x4e);	
	ov7251_WriteReg16(MIPI_CTRL01, 0x0f);
	ov7251_WriteReg16(MIPI_CTRL06, 0x0f);
	ov7251_WriteReg16(HS_ZERO_MIN, 0xaa);
	ov7251_WriteReg16(CLK_TRAIL_MIN, 0x3e);
	ov7251_WriteReg16(PCLK_PERIOD, 0x19);
	ov7251_WriteReg16(DEBUG_CTRL, 0x00);
	ov7251_WriteReg16(ISP_CTRL00, 0x85);
	ov7251_WriteReg16(ISP_CTRL01, 0x80);

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

