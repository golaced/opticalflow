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

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "settings.h"
#include "utils.h"
#include "led.h"
#include "flow.h"
#include "gyro.h"
#include "i2c.h"
#include "usart.h"
#include "sonar.h"
#include "communication.h"
#include "debug.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "main.h"

#include "dcmi_ov7251.h"
#include "stmipid02.h"
#include "ov7251.h"

/* coprocessor control register (fpu) */
#ifndef SCB_CPACR
#define SCB_CPACR (*((uint32_t*) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

/* fast image buffers for calculations */
uint8_t* image_buffer_8bit_1 = ((uint8_t*) 0x10000000);
uint8_t* image_buffer_8bit_2 = ((uint8_t*) ( 0x10000000 | OV7251_FULL_IMAGE_SIZE ));
uint8_t buffer_reset_needed;

/* boot time in milli seconds */
volatile uint32_t boot_time_ms = 0;

//chris
//#define DEBUG_MT9V034   1
//#define DEBUG_STMIPID02 1
//#define DEBUG_OV7251    1

/* timer constants */
#define NTIMERS         	8
#define TIMER_CIN       	0
#define TIMER_LED       	1
#define TIMER_DELAY     	2
#define TIMER_SONAR			3
#define TIMER_SYSTEM_STATE	4
#define TIMER_RECEIVE		5
#define TIMER_PARAMS		6
#define TIMER_IMAGE			7
#define LED_TIMER_COUNT		500
#define SONAR_TIMER_COUNT 	100
#define SYSTEM_STATE_COUNT	1000
#define PARAMS_COUNT		100
static volatile unsigned timer[NTIMERS];

/* timer/system booleans */
bool send_system_state_now = true;
bool receive_now = true;
bool send_params_now = true;
bool send_image_now = true;

extern uint8_t i2cread_stmipid02[20] = {0};
extern uint8_t i2cread_ov7251[120] = {0};

/**
  * @brief  Increment boot_time variable and decrement timer array.
  * @param  None
  * @retval None
  */
void timer_update(void)
{
	boot_time_ms++;

	/* each timer decrements every millisecond if > 0 */
	for (unsigned i = 0; i < NTIMERS; i++)
		if (timer[i] > 0)
			timer[i]--;

	if (timer[TIMER_LED] == 0)
	{
		/* blink activitiy */
		LEDToggle(LED_ACT);
		timer[TIMER_LED] = LED_TIMER_COUNT;
	}

	if (timer[TIMER_SONAR] == 0)
	{
		sonar_trigger();
		timer[TIMER_SONAR] = SONAR_TIMER_COUNT;
	}

	if (timer[TIMER_SYSTEM_STATE] == 0)
	{
		send_system_state_now = true;
		timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	}

	if (timer[TIMER_RECEIVE] == 0)
	{
		receive_now = true;
		timer[TIMER_RECEIVE] = SYSTEM_STATE_COUNT;
	}

	if (timer[TIMER_PARAMS] == 0)
	{
		send_params_now = true;
		timer[TIMER_PARAMS] = PARAMS_COUNT;
	}

	if (timer[TIMER_IMAGE] == 0)
	{
		send_image_now = true;
		timer[TIMER_IMAGE] = global_data.param[PARAM_VIDEO_RATE];
	}
}

uint32_t get_boot_time_ms(void)
{
	return boot_time_ms;
}

void delay(unsigned msec)
{
	timer[TIMER_DELAY] = msec;
	while (timer[TIMER_DELAY] > 0) {};
}

void buffer_reset(void) {
	buffer_reset_needed = 1;
}

/**
  * @brief  Main function.
  */
int main(void)
{
	/* load settings and parameters */
	global_data_reset_param_defaults();
	global_data_reset();

	/* init led */
	LEDInit(LED_ACT);
	LEDInit(LED_COM);
	LEDInit(LED_ERR);
	LEDOff(LED_ACT);
	LEDOff(LED_COM);
	LEDOff(LED_ERR);

	/* enable FPU on Cortex-M4F core */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

	/* init clock */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* capture clock error */
		LEDOn(LED_ERR);
		while (1);
	}

	/* init usb */
	USBD_Init(	&USB_OTG_dev,
				USB_OTG_FS_CORE_ID,
				&USR_desc,
				&USBD_CDC_cb,
				&USR_cb);

	/* init mavlink */
	communication_init();

//chris
	/* enable image capturing */
#ifdef DEBUG_MT9V034
	//not compile dcmi.c 
	enable_image_capture();
#endif

	enable_ov7251_image_capture();

	/* gyro config */
	gyro_config();

	/* init and clear fast image buffers */
	for (int i = 0; i < global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]; i++)
	{
		image_buffer_8bit_1[i] = 0;
		image_buffer_8bit_2[i] = 0;
	}

	uint8_t * current_image = image_buffer_8bit_1;
	uint8_t * previous_image = image_buffer_8bit_2;

	/* usart config*/
	usart_init();

    /* i2c config*/
    i2c_init();

	/* sonar config*/
	float sonar_distance_filtered = 0.0f; // distance in meter
	float sonar_distance_raw = 0.0f; // distance in meter
	bool distance_valid = false;
	sonar_config();

	/* reset/start timers */
	timer[TIMER_SONAR] = SONAR_TIMER_COUNT;
	timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	timer[TIMER_RECEIVE] = SYSTEM_STATE_COUNT / 2;
	timer[TIMER_PARAMS] = PARAMS_COUNT;
	timer[TIMER_IMAGE] = global_data.param[PARAM_VIDEO_RATE];

	/* variables */
	uint32_t counter = 0;
	uint8_t qual = 0;

	/* bottom flow variables */
	float pixel_flow_x = 0.0f;
	float pixel_flow_y = 0.0f;
	float pixel_flow_x_sum = 0.0f;
	float pixel_flow_y_sum = 0.0f;
	float velocity_x_sum = 0.0f;
	float velocity_y_sum = 0.0f;
	float velocity_x_lp = 0.0f;
	float velocity_y_lp = 0.0f;
	int valid_frame_count = 0;
	int pixel_flow_count = 0;

	u16 loop_count = 0;

	/* main loop */
	while (1)
	{
		/* reset flow buffers if needed */
		if(buffer_reset_needed)
		{
			buffer_reset_needed = 0;
			for (int i = 0; i < global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]; i++)
			{
				image_buffer_8bit_1[i] = 0;
				image_buffer_8bit_2[i] = 0;
			}
			delay(500);
			continue;
		}

		/* calibration routine */
		/**/
		if(global_data.param[PARAM_VIDEO_ONLY])
		{
			while(global_data.param[PARAM_VIDEO_ONLY])
			{
				dcmi_restart_calibration_routine();

				// waiting for first quarter of image 
				while(ov7251_get_frame_counter() < 2){}
				ov7251_dma_copy_image_buffers(&current_image, &previous_image, OV7251_FULL_IMAGE_SIZE, 1);

				// waiting for second quarter of image 
				while(ov7251_get_frame_counter() < 3){}
				ov7251_dma_copy_image_buffers(&current_image, &previous_image, OV7251_FULL_IMAGE_SIZE, 1);

				// waiting for all image parts 
				while(ov7251_get_frame_counter() < 4){}

				send_calibration_image(&previous_image, &current_image);

				if (global_data.param[PARAM_SYSTEM_SEND_STATE])
					communication_system_state_send();

				communication_receive_usb();
				debug_message_send_one();
				communication_parameter_send();

				LEDToggle(LED_COM);
			}

			dcmi_restart_calibration_routine();
			LEDOff(LED_COM);
		}
		

		uint16_t image_size = global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT];

		/* new gyroscope data */
		float x_rate_sensor, y_rate_sensor, z_rate_sensor;
		gyro_read(&x_rate_sensor, &y_rate_sensor, &z_rate_sensor);

		/* gyroscope coordinate transformation */
		float x_rate = y_rate_sensor; // change x and y rates
		float y_rate = - x_rate_sensor;
		float z_rate = z_rate_sensor; // z is correct

		/* calculate focal_length in pixel */
		const float focal_length_px = (global_data.param[PARAM_FOCAL_LENGTH_MM]) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled


		/* get sonar data */
		sonar_read(&sonar_distance_filtered, &sonar_distance_raw);

		//IAC chris added for the not ready sonar
		sonar_distance_raw = 1.5f;
		sonar_distance_filtered = 1.5f;

		/* compute optical flow */
		if(global_data.param[PARAM_SENSOR_POSITION] == BOTTOM)
		{
			/* copy recent image to faster ram */
			ov7251_dma_copy_image_buffers(&current_image, &previous_image, image_size, 1);

			/* compute optical flow */
			qual = compute_klt(previous_image, current_image, x_rate, y_rate, z_rate, &pixel_flow_x, &pixel_flow_y);
			
			if (sonar_distance_filtered > 5.0f || sonar_distance_filtered == 0.0f)
			{
				/* distances above 5m are considered invalid */
				sonar_distance_filtered = 0.0f;
				distance_valid = false;
			}
			else
			{
				distance_valid = true;
			}

			/*
			 * real point P (X,Y,Z), image plane projection p (x,y,z), focal-length f, distance-to-scene Z
			 * x / f = X / Z
			 * y / f = Y / Z
			 */
			float flow_compx = pixel_flow_x / focal_length_px / (ov7251_get_time_between_images() / 1000.0f);
			float flow_compy = pixel_flow_y / focal_length_px / (ov7251_get_time_between_images() / 1000.0f);


      /*
       * gyro compensation
       *
       * TODO: do not compensate more than the valid flow value (+/- 4.5 pixels)
       *
       * -y_rate gives x flow
       * x_rates gives y_flow
       */
      if (global_data.param[PARAM_BOTTOM_FLOW_GYRO_COMPENSATION])
      {
        if(fabsf(y_rate) > global_data.param[PARAM_GYRO_COMPENSATION_THRESHOLD])
        {
          flow_compx = flow_compx - y_rate;
        }

        if(fabsf(x_rate) > global_data.param[PARAM_GYRO_COMPENSATION_THRESHOLD])
        {
          flow_compy = flow_compy + x_rate;
        }
      }


			/* integrate velocity and output values only if distance is valid */
			if (distance_valid)
			{
				/* calc velocity (negative of flow values scaled with distance) */
				float new_velocity_x = - flow_compx * sonar_distance_filtered;
				float new_velocity_y = - flow_compy * sonar_distance_filtered;

				if (qual > 0)
				{
					velocity_x_sum += new_velocity_x;
					velocity_y_sum += new_velocity_y;
					valid_frame_count++;

					/* lowpass velocity output */
					velocity_x_lp = global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW] * new_velocity_x +
							(1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_x_lp;
					velocity_y_lp = global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW] * new_velocity_y +
							(1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_y_lp;
				}
				else
				{
					/* taking flow as zero */
					velocity_x_lp = (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_x_lp;
					velocity_y_lp = (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_y_lp;
				}
			}
			else
			{
				/* taking flow as zero */
				velocity_x_lp = (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_x_lp;
				velocity_y_lp = (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_y_lp;
			}

			pixel_flow_x_sum += pixel_flow_x;
			pixel_flow_y_sum += pixel_flow_y;
			pixel_flow_count++;

		}

		counter++;

//IAC chris added for debug
		loop_count++;
		while( loop_count == 2000){
			print("\r\n============================= Debug Log =============================\r\n");
			print("SONAR   distance = %f \r\n", sonar_distance_raw);
			print("GYRO    x = %f, y = %f, z = %f \r\n",x_rate,y_rate,z_rate);		
			
#ifdef DEBUG_MT9V034
			//not compile mt9v034.c 
			uint16_t version = mt9v034_ReadReg16(0x1324);
			//test the i2c of MT9V034
			uint16_t i2ctest = 0, i2ctestvalue = 3, i2ctestaddress = 0x1B;
			print("\r\n--------- i2c test for MT9V034 ---------\r\n");
			print("MT9V034 chip_version = %x \r\n", version);
			print("Write value (0) to address [0x1B]\r\n");
			mt9v034_WriteReg16(i2ctestaddress, 0);
			delay(3);
			i2ctest = mt9v034_ReadReg16(i2ctestaddress);
			print("MT9V034 i2c test, the value of [0x1B] = %x \r\n", i2ctest);
			print("Write value (3) to address [0x1B]\r\n");
			mt9v034_WriteReg16(i2ctestaddress, i2ctestvalue);
			delay(3);
			i2ctest = mt9v034_ReadReg16(i2ctestaddress);
			print("MT9V034 i2c test, the value of [0x1B] = %x \r\n", i2ctest);
			
			if(i2ctest==i2ctestvalue){
				print("!!! i2c test for MT9V034 PASS !!! \r\n");
			}else{
				print("@@@ i2c test for MT9V034 FAIL @@@ \r\n");
			}
#endif

#ifdef DEBUG_STMIPID02
			//test the i2c of STMIPID02
			uint8_t i2cread = 0;
			uint8_t stmipid02value = 1, stmipid02address = 0x14; //0x14

			print("\r\n--------- I2C test for STMIPID02 ---------\r\n");
			print("Write value (0) to address [0x14]\r\n");
			stmipid02_WriteReg8(stmipid02address, 0);
			//delay(3);
			i2cread = stmipid02_ReadReg8(stmipid02address);
			print("STMIPID02 i2c test, the value of [0x14] = %x \r\n", i2cread);
			print("Write value (1) to address [0x14]\r\n");
			stmipid02_WriteReg8(stmipid02address, stmipid02value);
			//delay(3);
			i2cread = stmipid02_ReadReg8(stmipid02address);
			print("STMIPID02 i2c test, the value of [0x14] = %x \r\n", i2cread);
			
			if(i2cread==stmipid02value){
				print("!!! I2C test for STMIPID02 PASS !!! \r\n");
			}else{
				print("@@@ I2C test for STMIPID02 FAIL @@@ \r\n");
			}
			/*
			for(int i =0 ; i<15 ; i++){
				print("stmipi registers[%d] = 0x%x\r\n",i , i2cread_stmipid02[i]);
			}
			*/
#endif

#ifdef DEBUG_OV7251
			//test the i2c of OV7251
			uint8_t imageread = 0, ov7251value = 2;
			uint16_t ov7251address = 0x3820;
			uint8_t read_id;
			
			print_ov7251_initlog();
			print("\r\n--------- I2C test for OV7251 ---------\r\n");				
			read_id = ov7251_ReadReg16(0x300A);
			print("OV7251 ID high = %x\r\n",read_id);
			read_id = ov7251_ReadReg16(0x300B);
			print("OV7251 ID low = %x\r\n",read_id);
			
			print("Write value (0) to address [0x3820]\r\n");
			ov7251_WriteReg16(ov7251address, 0);
			delay(3);
			imageread = ov7251_ReadReg16(ov7251address);
			print("OV7251 i2c test, the value of [0x3820] = %x \r\n", imageread);
			print("Write value (2) to address [0x3820]\r\n");
			ov7251_WriteReg16(ov7251address, ov7251value);
			delay(3);
			imageread = ov7251_ReadReg16(ov7251address);
			print("OV7251 i2c test, the value of [0x3820] = %x \r\n", imageread);
			
			if(imageread==ov7251value){
				print("!!! I2C test for OV7251 PASS !!! \r\n");
			}else{
				print("@@@ I2C test for OV7251 FAIL @@@ \r\n");
			}
			/*
			for(int j =0 ; j<120 ; j++){
				print("ov7251 registers[%d] = 0x%x\r\n",j , i2cread_ov7251[j]);
			}
			*/
#endif
			print("\r\n\n\n");
			loop_count = 0;
		}

		/* TODO for debugging */
		//mavlink_msg_named_value_float_send(MAVLINK_COMM_2, boot_time_ms, "blabla", blabla);

		if(global_data.param[PARAM_SENSOR_POSITION] == BOTTOM)
		{
			/* send bottom flow if activated */
			if (counter % 2 == 0)
			{
				float flow_comp_m_x = 0.0f;
				float flow_comp_m_y = 0.0f;
				float ground_distance = 0.0f;
				if(global_data.param[PARAM_BOTTOM_FLOW_LP_FILTERED])
				{
					flow_comp_m_x = velocity_x_lp;
					flow_comp_m_y = velocity_y_lp;
				}
				else
				{
					flow_comp_m_x = velocity_x_sum/valid_frame_count;
					flow_comp_m_y = velocity_y_sum/valid_frame_count;
				}

				if(global_data.param[PARAM_SONAR_FILTERED])
					ground_distance = sonar_distance_filtered;
				else
					ground_distance = sonar_distance_raw;

				if (valid_frame_count > 0)
				{
					// send flow
					mavlink_msg_optical_flow_send(MAVLINK_COMM_0, get_boot_time_ms() * 1000, global_data.param[PARAM_SENSOR_ID],
							pixel_flow_x_sum * 10.0f, pixel_flow_y_sum * 10.0f,
							flow_comp_m_x, flow_comp_m_y, qual, ground_distance);

					if (global_data.param[PARAM_USB_SEND_FLOW])
						mavlink_msg_optical_flow_send(MAVLINK_COMM_2, get_boot_time_ms() * 1000, global_data.param[PARAM_SENSOR_ID],
							pixel_flow_x_sum * 10.0f, pixel_flow_y_sum * 10.0f,
							flow_comp_m_x, flow_comp_m_y, qual, ground_distance);

                    update_TX_buffer(pixel_flow_x_sum * 10.0f, pixel_flow_y_sum * 10.0f, flow_comp_m_x, flow_comp_m_y, qual,
                            ground_distance, x_rate, y_rate, z_rate);

				}
				else
				{
					// send distance
					mavlink_msg_optical_flow_send(MAVLINK_COMM_0, get_boot_time_ms() * 1000, global_data.param[PARAM_SENSOR_ID],
						pixel_flow_x_sum * 10.0f, pixel_flow_y_sum * 10.0f,
						0.0f, 0.0f, 0, ground_distance);

					if (global_data.param[PARAM_USB_SEND_FLOW])
						mavlink_msg_optical_flow_send(MAVLINK_COMM_2, get_boot_time_ms() * 1000, global_data.param[PARAM_SENSOR_ID],
							pixel_flow_x_sum * 10.0f, pixel_flow_y_sum * 10.0f,
							0.0f, 0.0f, 0, ground_distance);
	
                    update_TX_buffer(pixel_flow_x_sum * 10.0f, pixel_flow_y_sum * 10.0f, 0.0f, 0.0f, 0, ground_distance, x_rate, y_rate,
                            z_rate);
                }

				if(global_data.param[PARAM_USB_SEND_GYRO])
				{
					mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "GYRO", get_boot_time_ms() * 1000, x_rate, y_rate, z_rate);
				}

				velocity_x_sum = 0.0f;
				velocity_y_sum = 0.0f;
				pixel_flow_x_sum = 0.0f;
				pixel_flow_y_sum = 0.0f;
				valid_frame_count = 0;
				pixel_flow_count = 0;
			}
		}

		/* forward flow from other sensors */
		if (counter % 2)
		{
			communication_receive_forward();
		}

		/* send system state, receive commands */
		if (send_system_state_now)
		{
			/* every second */
			if (global_data.param[PARAM_SYSTEM_SEND_STATE])
			{
				communication_system_state_send();
			}
			send_system_state_now = false;
		}

		/* receive commands */
		if (receive_now)
		{
			/* test every second */
			communication_receive();
			communication_receive_usb();
			receive_now = false;
		}

		/* sending debug msgs and requested parameters */
		if (send_params_now)
		{
			debug_message_send_one();
			communication_parameter_send();
			send_params_now = false;
		}

		/*  transmit raw 8-bit image */
		if (global_data.param[PARAM_USB_SEND_VIDEO] && send_image_now)
		{
			/* get size of image to send */
			uint16_t image_size_send;
			uint16_t image_width_send;
			uint16_t image_height_send;

			image_width_send = global_data.param[PARAM_IMAGE_WIDTH];
			image_height_send = global_data.param[PARAM_IMAGE_HEIGHT];//+global_data.param[PARAM_IMAGE_HEIGHT]/2;
			image_size_send = image_width_send*image_height_send;

			mavlink_msg_data_transmission_handshake_send(
					MAVLINK_COMM_2,
					MAVLINK_DATA_STREAM_IMG_RAW8U,
					image_size_send,
					image_width_send,
					image_height_send,
					image_size_send / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1,
					MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN,
					100);
			LEDToggle(LED_COM);
			uint16_t frame = 0;

			for (frame = 0; frame < image_size_send / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1; frame++)
			{
				mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, &((uint8_t *) current_image)[frame * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN]);
			}

			send_image_now = false;
		}
		else if (!global_data.param[PARAM_USB_SEND_VIDEO])
		{
			LEDOff(LED_COM);
		}
	}
}
