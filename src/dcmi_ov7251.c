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

#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "utils.h"
#include "dcmi_ov7251.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "stm32f4xx.h"

#include "communication.h"


/* counters */
volatile uint8_t image_counter = 0;
volatile uint32_t frame_counter;
volatile uint32_t time_last_frame = 0;
volatile uint32_t cycle_time = 0;
volatile uint32_t time_between_next_images;
volatile uint8_t dcmi_calibration_counter = 0;

/* state variables */
volatile uint8_t dcmi_image_buffer_memory0 = 1;
volatile uint8_t dcmi_image_buffer_memory1 = 2;
volatile uint8_t dcmi_image_buffer_unused = 3;
volatile uint8_t calibration_used;
volatile uint8_t calibration_unused;
volatile uint8_t calibration_mem0;
volatile uint8_t calibration_mem1;

/*ov7251 image buffers */

uint8_t dcmi_image_buffer_10bit_1[OV7251_FULL_IMAGE_SIZE];
uint8_t dcmi_image_buffer_10bit_2[OV7251_FULL_IMAGE_SIZE];
uint8_t dcmi_image_buffer_10bit_3[OV7251_FULL_IMAGE_SIZE];


uint32_t time_between_images;

/* extern functions */
extern uint32_t get_boot_time_ms(void);
extern void delay(unsigned msec);

uint8_t ov7251_debug[9]={0};


/**
 * @brief Initialize DCMI DMA and enable image capturing
 */
void enable_ov7251_image_capture(void)
{
	ov7251_debug[0]=1;
	stmipid02_ov7251_clock_init();
	stmipid02_ov7251_hw_init();
	ov7251_dcmi_dma_init(global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]);
	stmipid02_context_configuration();
	ov7251_context_configuration();
	ov7251_dcmi_dma_enable();
}

/**
 * @brief DMA reconfiguration after changing image window
 */
void ov7251_dma_reconfigure(void)
{
	ov7251_dcmi_dma_disable();

	if(global_data.param[PARAM_VIDEO_ONLY])
		ov7251_dcmi_dma_init(OV7251_FULL_IMAGE_SIZE);
	else
		ov7251_dcmi_dma_init(global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]);

	ov7251_dcmi_dma_enable();
}

/**
 * @brief Calibration image collection routine restart
 */
void dcmi_restart_calibration_routine(void)
{
	/* wait until we have all 4 parts of image */
	while(frame_counter < 4){}
	frame_counter = 0;
	ov7251_dcmi_dma_enable();
}

/**
 * @brief Interrupt handler of DCMI
 */
void DCMI_IRQHandler(void)
{
	if (DCMI_GetITStatus(DCMI_IT_FRAME) != RESET)
	{
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);
	}

	return;
}

/**
 * @brief Interrupt handler of DCMI DMA stream
 */
void DMA2_Stream1_IRQHandler(void)
{
	/* transfer completed */
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET)
	{
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
		frame_counter++;

		if(global_data.param[PARAM_VIDEO_ONLY])
		{
			if (frame_counter >= 4)
			{
				ov7251_dcmi_dma_disable();
				calibration_used = DMA_GetCurrentMemoryTarget(DMA2_Stream1);
				calibration_unused = dcmi_image_buffer_unused;
				calibration_mem0 = dcmi_image_buffer_memory0;
				calibration_mem1 = dcmi_image_buffer_memory1;
			}
		}

		return;
	}

	/* transfer half completed
	 *
	 * We use three buffers and switch the buffers if dma transfer
	 * is in half state.
	 */
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_HTIF1) != RESET)
	{
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_HTIF1);
	}

	dma_swap_buffers();
}

/**
 * @brief Swap DMA image buffer addresses
 */
void dma_swap_buffers(void)
{
	ov7251_debug[7]=1;
	/* check which buffer is in use */
	if (DMA_GetCurrentMemoryTarget(DMA2_Stream1))
	{
		ov7251_debug[7]=dcmi_image_buffer_unused;
		/* swap dcmi image buffer */
		if (dcmi_image_buffer_unused == 1)
			DMA_MemoryTargetConfig(DMA2_Stream1, (uint32_t) dcmi_image_buffer_10bit_1, DMA_Memory_0);
		else if (dcmi_image_buffer_unused == 2)
			DMA_MemoryTargetConfig(DMA2_Stream1, (uint32_t) dcmi_image_buffer_10bit_2, DMA_Memory_0);
		else
			DMA_MemoryTargetConfig(DMA2_Stream1, (uint32_t) dcmi_image_buffer_10bit_3, DMA_Memory_0);

		int tmp_buffer = dcmi_image_buffer_memory0;
		dcmi_image_buffer_memory0 = dcmi_image_buffer_unused;
		dcmi_image_buffer_unused = tmp_buffer;
	}
	else
	{
		ov7251_debug[7]=dcmi_image_buffer_unused;
		/* swap dcmi image buffer */
		if (dcmi_image_buffer_unused == 1)
			DMA_MemoryTargetConfig(DMA2_Stream1, (uint32_t) dcmi_image_buffer_10bit_1, DMA_Memory_1);
		else if (dcmi_image_buffer_unused == 2)
			DMA_MemoryTargetConfig(DMA2_Stream1, (uint32_t) dcmi_image_buffer_10bit_2, DMA_Memory_1);
		else
			DMA_MemoryTargetConfig(DMA2_Stream1, (uint32_t) dcmi_image_buffer_10bit_3, DMA_Memory_1);

		int tmp_buffer = dcmi_image_buffer_memory1;
		dcmi_image_buffer_memory1 = dcmi_image_buffer_unused;
		dcmi_image_buffer_unused = tmp_buffer;
	}

	/* set next time_between_images */
	cycle_time = get_boot_time_ms() - time_last_frame;
	time_last_frame = get_boot_time_ms();

	if(image_counter) // image was not fetched jet
	{
		time_between_next_images = time_between_next_images + cycle_time;
	}
	else
	{
		time_between_next_images = cycle_time;
	}

	/* set new image true and increment frame counter*/
	image_counter += 1;

	return;
}

uint32_t ov7251_get_time_between_images(void){
	return time_between_images;
}

uint32_t ov7251_get_frame_counter(void){
	return frame_counter;
}

/**
 * @brief Copy image to fast RAM address
 *
 * @param current_image Current image buffer
 * @param previous_image Previous image buffer
 * @param image_size Image size of the image to copy
 * @param image_step Image to wait for (if 1 no waiting)
 */
void ov7251_dma_copy_image_buffers(uint8_t ** current_image, uint8_t ** previous_image, uint16_t image_size, uint8_t image_step){

	/* swap image buffers */
	uint8_t * tmp_image = *current_image;
	*current_image = *previous_image;
	*previous_image = tmp_image;

	/* wait for new image if needed */
	while(image_counter < image_step) {}
	image_counter = 0;

	/* time between images */
	time_between_images = time_between_next_images;

	ov7251_debug[8]=dcmi_image_buffer_unused;

	/* copy image */
	if (dcmi_image_buffer_unused == 1)
	{
		for (uint16_t pixel = 0; pixel < image_size; pixel++){
			(*current_image)[pixel] = (uint8_t) (dcmi_image_buffer_10bit_1[pixel]) ;
			//(*current_image)[pixel] = (uint8_t)( (dcmi_image_buffer_10bit_1[pixel]>>2) & 0xFF);  //pixel source is 8 bit?		
		}
	}
	else if (dcmi_image_buffer_unused == 2)
	{
		for (uint16_t pixel = 0; pixel < image_size; pixel++){
			(*current_image)[pixel] = (uint8_t) (dcmi_image_buffer_10bit_2[pixel]) ;  
			//(*current_image)[pixel] = (uint8_t)( (dcmi_image_buffer_10bit_2[pixel]>>2) & 0xFF);  //pixel source is 8 bit?
		}
	}
	else
	{
		for (uint16_t pixel = 0; pixel < image_size; pixel++){
			(*current_image)[pixel] = (uint8_t) (dcmi_image_buffer_10bit_3[pixel]) ; 
			//(*current_image)[pixel] = (uint8_t)( (dcmi_image_buffer_10bit_3[pixel]>>2) & 0xFF);  //pixel source is 8 bit?
		}
	}
	
}

/**
 * @brief Send calibration image with MAVLINK over USB
 *
 * @param image_buffer_fast_1 Image buffer in fast RAM
 * @param image_buffer_fast_2 Image buffer in fast RAM
 */
void send_calibration_image(uint8_t ** image_buffer_fast_1, uint8_t ** image_buffer_fast_2) {

	/*  transmit raw 8-bit image */
	/* TODO image is too large for this transmission protocol (too much packets), but it works */
	mavlink_msg_data_transmission_handshake_send(
			MAVLINK_COMM_2,
			MAVLINK_DATA_STREAM_IMG_RAW8U,
			OV7251_FULL_IMAGE_SIZE * 4,
			OV7251_FULL_IMAGE_ROW_SIZE * 2,
			OV7251_FULL_IMAGE_COLUMN_SIZE * 2,
			OV7251_FULL_IMAGE_SIZE * 4 / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1,
			MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN,
			100);

	uint16_t frame = 0;
	uint8_t image = 0;
	uint8_t frame_buffer[MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN];

	for (int i = 0; i < OV7251_FULL_IMAGE_SIZE * 4; i++)
	{

		if (i % OV7251_FULL_IMAGE_SIZE == 0 && i != 0)
		{
			image++;
		}

		if (i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN == 0 && i != 0)
		{
			mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, frame_buffer);
			frame++;
			delay(2);
		}

		if (image == 0 )
		{
			frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t)(*image_buffer_fast_1)[i % OV7251_FULL_IMAGE_SIZE];
		}
		else if (image == 1 )
		{
			frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t)(*image_buffer_fast_2)[i % OV7251_FULL_IMAGE_SIZE];
		}
		else if (image == 2)
		{
			if (calibration_unused == 1)
				frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t) (dcmi_image_buffer_10bit_1[i % OV7251_FULL_IMAGE_SIZE]);
			else if (calibration_unused == 2)
				frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t) (dcmi_image_buffer_10bit_2[i % OV7251_FULL_IMAGE_SIZE]);
			else
				frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t) (dcmi_image_buffer_10bit_3[i % OV7251_FULL_IMAGE_SIZE]);
		}
		else
		{
			if (calibration_used)
			{
				if (calibration_mem0 == 1)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t) (dcmi_image_buffer_10bit_1[i % OV7251_FULL_IMAGE_SIZE]);
				else if (calibration_mem0 == 2)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t) (dcmi_image_buffer_10bit_2[i % OV7251_FULL_IMAGE_SIZE]);
				else
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t) (dcmi_image_buffer_10bit_3[i % OV7251_FULL_IMAGE_SIZE]);
			}
			else
			{
				if (calibration_mem1 == 1)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t) (dcmi_image_buffer_10bit_1[i % OV7251_FULL_IMAGE_SIZE])&0xFF;
				else if (calibration_mem1 == 2)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t) (dcmi_image_buffer_10bit_2[i % OV7251_FULL_IMAGE_SIZE])&0xFF;
				else
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t) (dcmi_image_buffer_10bit_3[i % OV7251_FULL_IMAGE_SIZE])&0xFF;
			}
		}
	}

	mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, frame_buffer);

}

/**
 * @brief Initialize/Enable DCMI Interrupt
 */
void ov7251_dcmi_it_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DCMI global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DCMI_ITConfig(DCMI_IT_FRAME,ENABLE);
}

/**
 * @brief Initialize/Enable DMA Interrupt
 */
void ov7251_dma_it_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DMA global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_HT, ENABLE); // half transfer interrupt
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE); // transfer complete interrupt
}

/**
 * @brief Enable DCMI DMA stream
 */
void ov7251_dcmi_dma_enable()
{
	ov7251_debug[6]=1;
	/* Enable DMA2 stream 1 and DCMI interface then start image capture */
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
	ov7251_dma_it_init();
}

/**
 * @brief Disable DCMI DMA stream
 */
void ov7251_dcmi_dma_disable()
{
	/* Disable DMA2 stream 1 and DCMI interface then stop image capture */
	DMA_Cmd(DMA2_Stream1, DISABLE);
	DCMI_Cmd(DISABLE);
	DCMI_CaptureCmd(DISABLE);
}

void reset_frame_counter()
{
	frame_counter = 0;
}

/**
 * @brief HW initialization of STMIPID02 & OV7251 clock
 */
void stmipid02_ov7251_clock_init()
{
	/* The main clock for stmipid02 & ov7251 */
	//EXTCLK(stmipid02) & MCLK(ov7251) used the same pin in the STM32F427 : PB3 (TIM2_EXTCLK)
	ov7251_debug[1]=1;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	// TIM2 clock enable 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// GPIOB clock enable 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// GPIOB Configuration:  TIM2 CH2 (PB3)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Connect TIM2 pins to AF1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

	// Time base configuration 
	TIM_TimeBaseStructure.TIM_Period = 3;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// PWM1 Mode configuration: Channel2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 2;// TIM_TimeBaseStructure.TIM_Period/2;

	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);

	// TIM2 enable counter 
	TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief HW initialization STMIPID02 & OV7251
 */
void stmipid02_ov7251_hw_init(void)
{
	ov7251_debug[2]=1;
	uint16_t image_size = global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT];

	/* Reset image buffers */
	for (int i = 0; i < image_size; i++) {
		dcmi_image_buffer_10bit_1 [i] = 0;
		dcmi_image_buffer_10bit_2 [i] = 0;
		dcmi_image_buffer_10bit_3 [i] = 0;
	}
	
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStruct;

	/* ----------------------------------The power on sequence pull pin -------------------------*/
	//XSDN & XSHUTDOWN used the same pin in the STM32F427 : PA2 (TIM5_PowerdownN)
	//XSDN pin should pull up after the VDDIN_LDO is stable (STMIPID02)
	//XSHUTDOWN pin is pulled up after AVDD and DOVDD are stable (OV7251)
	//POR_SGN pin is the power on reset signal : PA3

	// Enable GPIOs clocks 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//GPIO configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	delay(1);
	GPIO_SetBits(GPIOA, GPIO_Pin_3);
	delay(1);

	/*** Configures the DCMI GPIOs to interface with the OV2640 camera module ***/
	/* Enable DCMI GPIOs clocks */
	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
					| RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

	/* Connect DCMI pins to AF13 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); //DCMI_HSYNC
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_PIXCL

	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_DCMI); //DCMI_D10  ==>??
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D5
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); //DCMI_VSYNC

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D0
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI); //DCMI_D1
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_DCMI); //DCMI_D8
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_DCMI); //DCMI_D9
	
	//GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_DCMI); //DCMI_D11  ==>??

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI); //DCMI_D2
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI); //DCMI_D3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI); //DCMI_D4
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI); //DCMI_D6
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D7

	/* DCMI GPIO configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
			| GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* -------------Initial I2C2 GPIO : PB10, PB11 (TRANSLATOR_SCL, TRANSLATOR_SDA) ------------*/
	// I2C2 clock enable 
	// GPIOB clock enable 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// Connect I2C2 pins to AF4 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	// Configure I2C2 GPIOs 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// I2C DeInit 
	I2C_DeInit(I2C2);
	// Enable the I2C peripheral 
	I2C_Cmd(I2C2, ENABLE);

	// Set the I2C structure parameters 
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 100000;

	// Initialize the I2C peripheral w/ selected parameters 
	I2C_Init(I2C2, &I2C_InitStruct);
}

/**
  * @brief  Configures DCMI/DMA to capture image from the ov7251 camera.
  *
  * @param  buffer_size Buffer size in bytes
  */
void ov7251_dcmi_dma_init(uint16_t buffer_size)
{
	ov7251_debug[3]=1;
	reset_frame_counter();

	DCMI_InitTypeDef DCMI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/*** Configures the DCMI to interface with the ov7251 camera module ***/
	/* Enable DCMI clock */
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

	/* DCMI configuration */
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;

	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Falling;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_Low;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;

	/* Configures the DMA2 to transfer Data from DCMI */
	/* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	/* DMA2 Stream1 Configuration */
	DMA_DeInit(DMA2_Stream1);

	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = OV7251_DCMI_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) dcmi_image_buffer_10bit_1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = buffer_size / 4; // buffer size in date unit (word)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_DoubleBufferModeConfig(DMA2_Stream1,(uint32_t) dcmi_image_buffer_10bit_2, DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream1,ENABLE);

	/* DCMI configuration */
	DCMI_Init(&DCMI_InitStructure);

	/* DMA2 IRQ channel Configuration */
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
}

void print_ov7251_initlog(void)
{
	for(int i=0 ; i<9 ; i++){
		print("ov7251_initlog[%d] = %d \r\n",i,ov7251_debug[i]);
	}
	print("image_counter =%d \r\n",image_counter);
}
