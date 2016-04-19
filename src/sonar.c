/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "utils.h"
#include "usart.h"
#include "settings.h"
#include "sonar.h"
#include "sonar_mode_filter.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

#define SONAR_SCALE 1000.0f

extern int atoi (__const char *__nptr);
extern uint32_t get_boot_time_us(void);

static char data_buffer[5]; // array for collecting decoded data

static volatile uint32_t last_measure_time = 0;
static volatile uint32_t measure_time = 0;
static volatile float dt = 0.0f;
static volatile int valid_data;
static volatile int data_counter = 0;
static volatile int data_valid = 0;
static volatile int new_value = 0;

static volatile uint32_t sonar_measure_time_interrupt = 0;
static volatile uint32_t sonar_measure_time = 0;

static volatile float record_sonar_value_filtered = 0.0f;
static volatile float record_sonar_value_raw = 0.0f;

/* kalman filter states */
float x_pred = 0.0f; // m
float v_pred = 0.0f;
float x_post = 0.0f; // m
float v_post = 0.0f; // m/s

float sonar_raw = 0.0f;  // m

float sonar_mode = 0.0f;				/**< the mode of all sonar measurements */

void EXTI13_IRQHandler(void) {
	u16 count;
	
	if(EXTI_GetITStatus(EXTI_Line13) != RESET){
		TIM_Cmd(TIM4, DISABLE);
		count = TIM_GetCounter(TIM4);

		record_sonar_value_filtered = (count*0.000344)/2.0;
		record_sonar_value_raw = (count*0.000344)/2.0;	
		//EXTI_ClearITPendingBit(EXTI_Line13); 
	}
}

/**
  * @brief  Triggers the sonar to measure the next value
  *
  * see datasheet for more info
  */
void sonar_trigger(){
	last_measure_time = measure_time;
	measure_time = get_boot_time_ms();
    sonar_measure_time_interrupt = measure_time;

	GPIO_SetBits(GPIOE, GPIO_Pin_8);
	
	TIM_SetCounter(TIM4, 0);
	TIM_Cmd(TIM4, ENABLE);
}

void sonar_trigger_low(){	
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);
}

/**
  * @brief  Basic Kalman filter
  */
void sonar_filter()
{
	/* no data for long time */
	if (dt > 0.25) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt * v_pred;
	v_pred = v_post;

	float x_new = sonar_mode;
	sonar_raw = x_new;
	x_post = x_pred + global_data.param[PARAM_SONAR_KALMAN_L1] * (x_new - x_pred);
	v_post = v_pred + global_data.param[PARAM_SONAR_KALMAN_L2] * (x_new - x_pred);

}


/**
  * @brief  Read out newest sonar data
  *
  * @param  sonar_value_filtered Filtered return value
  * @param  sonar_value_raw Raw return value
  */
void sonar_read(float* sonar_value_filtered, float* sonar_value_raw)
{
	// 344 m/s = 344 m/1*1000*1000us = 0.000344 m/us
	*sonar_value_filtered = record_sonar_value_filtered;
	*sonar_value_raw = record_sonar_value_raw;
}

/**
 * @brief  Configures the sonar sensor Peripheral.
 */
void sonar_config(void)
{
	//valid_data = 0;
	uint16_t PrescalerValue = 0;
    PrescalerValue = 84-1;

	//PE8 , TRIGER
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);

	//PD13 , ECHO
	GPIO_InitTypeDef GPIO_InitStructure2;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure2);

	//PF8 , DELAY
	GPIO_InitTypeDef GPIO_InitStructure3;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure3);

	//============================================
	
	//PD13 using TIM4
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 49999; //
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue; //
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//PF8 using TIM13
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure2;  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE); 
    TIM_TimeBaseStructure2.TIM_Period = 9999; //1us x 10000 = 10ms
    TIM_TimeBaseStructure2.TIM_Prescaler = PrescalerValue; // APB1 = 42MHz, TIMxCLK = 84MHz, (84MHz/83+1)=1MHz => 1us 
    TIM_TimeBaseStructure2.TIM_ClockDivision = 0x0;  
    TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure2); 
		
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource13); 

	EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI13_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

uint32_t get_sonar_measure_time()
{
    return sonar_measure_time;
}

uint32_t get_sonar_measure_time_interrupt()
{
    return sonar_measure_time_interrupt;
}

