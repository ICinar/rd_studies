/**
 * \file
 *
 * \brief FreeRTOS demo task implementations.
 *
 * Copyright (C) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <asf.h>
#include <conf_demo.h>
#include <stdio_serial.h>
#include <string.h>
#include <time.h>
#include "demotasks.h"

/**
 * \addtogroup freertos_sam0_demo_tasks_group
 *
 * @{
 */

//! \name Task configuration
//@{



#define TICK_TASK_PRIORITY		  (tskIDLE_PRIORITY + 1) 
#define TICK_TASK_DELAY			  (2000 / portTICK_RATE_MS)
//@}



/**
 * \brief Instance for \ref oled1_xpro_io_group
 *
 * The extension header to use is configured with \ref OLED1_EXT_HEADER.
 */
static OLED1_CREATE_INSTANCE(oled1, OLED1_EXT_HEADER);

//! \name Tasks for demo
//@{

static void tick_task(void *params);


/**
 * \brief Initialize tasks and resources for demo
 *
 * This function initializes the \ref oled1_xpro_io_group instance and the
 * \ref edbg_cdc_rx_group instance for reception, then creates all
 * the objects for FreeRTOS to run the demo.
 */


struct usart_module usart_instance;





void uart_init(uint32_t baud)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	
	config_usart.baudrate 	= baud;	 
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;	
	 stdio_serial_init(&usart_instance, EDBG_CDC_MODULE, &config_usart);
	 usart_enable(&usart_instance);	
	
}

void serialClear(void){
	usart_write_buffer_wait(&usart_instance, (uint8_t *)"\033[2J", 4);
	usart_write_buffer_wait(&usart_instance, (uint8_t *)"\033[0;0H", 6);
}

void serialRead(char* buffer, uint16_t length){
	for(uint8_t i = 0; i < length; i++) {
		
		char tempStr[2] ={0,0};
		uint16_t temp;
		bool loop = true;
		uint8_t char_;
		
		while(loop){
			if (usart_read_wait(&usart_instance, &temp) == STATUS_OK) {
				char_ = temp;
				if ((char_ >= 'A' && char_ <= 'Z') ||
				    (char_ >= '0' && char_ <= '9') ||
				    (char_ >= 'a' && char_ <= 'z'))
				{
					usart_write_wait(&usart_instance, char_);
					tempStr[0] = char_;
					strcat(buffer, tempStr);
				} else if(char_ == '\n'|| char_ == '\r') {
					loop = false;
				}
			}
		}
		usart_write_wait(&usart_instance, '\n');
		usart_write_wait(&usart_instance, '\r');

 }
}

void serialWrite(char* buffer, uint16_t length) {
	 if (usart_write_buffer_wait(&usart_instance, (uint8_t *)buffer, length) == STATUS_OK) { 
	 }
}

void demotasks_init(void)
{
	// Initialize hardware for the OLED1 Xplained Pro driver instance
	oled1_init(&oled1);
	xTaskCreate(tick_task,
			(const char *) "TICK",
			configMINIMAL_STACK_SIZE,
			NULL,
			TICK_TASK_PRIORITY,
			NULL);

}


/* defined TASKS for FreeRTOS */

static void tick_task(void *params)
{
	TickType_t starttick = xTaskGetTickCount();
	delay_ms(1000);
	TickType_t endtick = xTaskGetTickCount();
	uart_init(115200);	
	uint32_t diff = endtick - starttick;
	char buf[2];
	sprintf(buf,"%lu\n", diff);
	for(;;) {

		//sprintf(bufferR, "Tick = %un", diff);
		oled1_set_led_state(&oled1, OLED1_LED2_ID, true);
		serialClear();
		serialWrite(buf,sizeof(buf)+1);
		vTaskDelay(TICK_TASK_DELAY);
	}
}