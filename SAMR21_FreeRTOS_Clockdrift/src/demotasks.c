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
#include "demotasks.h"

/**
 * \addtogroup freertos_sam0_demo_tasks_group
 *
 * @{
 */

//! \name Task configuration
//@{

#define LED_TASK_ON_PRIORITY      (tskIDLE_PRIORITY + 1)
#define LED_TASK_ON_DELAY         (1000 / portTICK_RATE_MS)
#define LED_TASK_OFF_PRIORITY     (tskIDLE_PRIORITY + 2)
#define TICK_TASK_PRIORITY		  (tskIDLE_PRIORITY + 3) 
#define LED_TASK_OFF_DELAY        (500 / portTICK_RATE_MS)
#define TICK_TASK_DELAY			  (500 / portTICK_RATE_MS)
//@}



/**
 * \brief Instance for \ref oled1_xpro_io_group
 *
 * The extension header to use is configured with \ref OLED1_EXT_HEADER.
 */
static OLED1_CREATE_INSTANCE(oled1, OLED1_EXT_HEADER);




//! \name Tasks for demo
//@{

static void led_task_on(void *params);
static void led_task_off(void *params);
static void tick_task(void *params);


/**
 * \brief Initialize tasks and resources for demo
 *
 * This function initializes the \ref oled1_xpro_io_group instance and the
 * \ref edbg_cdc_rx_group instance for reception, then creates all
 * the objects for FreeRTOS to run the demo.
 */
void demotasks_init(void)
{
	// Initialize hardware for the OLED1 Xplained Pro driver instance
	oled1_init(&oled1);
	xTaskCreate(led_task_on,
			(const char *) "LED-ON",
			configMINIMAL_STACK_SIZE,
			NULL,
			LED_TASK_ON_PRIORITY,
			NULL);

	xTaskCreate(led_task_off,
			(const char *) "LED-OFF",
			configMINIMAL_STACK_SIZE,
			NULL,
			LED_TASK_OFF_PRIORITY,
			NULL);
	
	xTaskCreate(tick_task,
			(const char *) "TICK",
			configMINIMAL_STACK_SIZE,
			NULL,
			TICK_TASK_PRIORITY,
			NULL);

}
/**
 * \brief Main demo task
 *
 * This task keeps track of which screen the user has selected, which tasks
 * to resume/suspend to draw the selected screen, and also draws the menu bar.
 *
 * The menu bar shows which screens the user can select by clicking the
 * corresponding buttons on the OLED1 Xplained Pro:
 * - \ref graph_task() "graph" (selected at start-up)
 * - \ref terminal_task() "term."
 * - \ref about_task() "about"
 *
 * \param params Parameters for the task. (Not used.)
 */
static void led_task_on(void *params)
{
	portTickType xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		oled1_set_led_state(&oled1, OLED1_LED2_ID, true);

		vTaskDelay(LED_TASK_ON_DELAY);
	}
}


static void led_task_off(void *params)
{
	portTickType xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		oled1_set_led_state(&oled1, OLED1_LED2_ID, false);
		


		vTaskDelay(LED_TASK_OFF_DELAY);
	}
}

static void tick_task(void *params)
{
	TickType_t starttick = xTaskGetTickCount();
	vTaskDelay(5000);
	TickType_t endtick = xTaskGetTickCount();
	
	int i = 0;
	//printf("Test");
	//printf("msec2ticks:%u ticksn", (uint16_t)(endtick-starttick));
	//vPrintString("Test");
	for(;;) {
		oled1_set_led_state(&oled1, OLED1_LED2_ID, false);
		str[6] = i + '0';
		while (USART_Send(USART6, str,strlen(str), NON_BLOCKING) < 0);
		i = (i +1) % 10;
		vTaskDelay(TICK_TASK_DELAY);
	}
}
