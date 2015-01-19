/**
 *
 * \file
 *
 * \brief Wifi Door opener.
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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

/** \mainpage
 * \section intro Introduction
 * This program demonstrates the use of the Wifi NMI with the SAMD21 Xplained Pro
 * board to implement a temperature sensor for the Internet Of Things.
 * It uses the following hardware:
 * - the Wifi NMI on EXT1.
 * - the IO1 sensors on EXT2.
 *
 * \section files Main Files
 * - main.c : instanciate the sensor task
 * - demo.c : handle the wifi and sensor logic
 * - demo.h : sensor demo header file
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC compiler using Atmel Studio 6.2
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com">Atmel</A>.\n
 */

#include "asf.h"
#include "application_specific/uart_PC.h"
#include "application_specific/config.h"
#include "stdio_serial.h"
#include "conf_uart_serial.h"
#include "door.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- Wifi Door opener --"STRING_EOL \
		"-- "BOARD_NAME" --"STRING_EOL \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** UART module for debug. */
static struct usart_module cdc_uart_module;

/** SysTick counter to avoid busy wait delay. */
volatile uint32_t ms_ticks = 0;

/**
 * \brief SysTick handler used to measure precise delay. 
 */
void SysTick_Handler(void)
{
	ms_ticks++;
}

/**
 * \brief Main application function. 
 *
 * Start the sensor task then start the scheduler.
 *
 * \return program return value.
 */
int main(void)
{
	/*Initialize everything*/
	config_application();

	/*Send welcome message over debug interface*/
	uart_report("Turret command center: \n\r", UART_COLOR_WHITE);
	
	/* Enable SysTick interrupt for non busy wait delay. */
	if (SysTick_Config(system_cpu_clock_get_hz() / 1000)) {
		uart_report("main: SysTick configuration error!", UART_COLOR_WHITE);
		while (1);
	}

	/* Output example information */
	uart_report(STRING_HEADER, UART_COLOR_WHITE);

	/* Start the demo task. */
	door_start();
	
	while(1);
	
	return 0;
}
