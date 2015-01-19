/*
 * heartbeat.c
 *
 * Created: 15.07.2014 07:41:57
 *  Author: magne.normann
 */ 

#include <asf.h>
#include "heartbeat.h"
#include "uart_PC.h"
#include "turret.h"
#include "motors.h"

void heartbeat_init(void){
	heartbeat_configure();
	heartbeat_configure_callbacks();
}

void heartbeat_timeout(struct tc_module *const module_inst)
{
	/* heartbeat is not OK if a heartbeat signal has not been recvd in the last 2 sec */
	if(!heartbeat_recvd){
		turret_stop();
		motors_stop();
		
		if (heartbeat_OK) {
			uart_report("Connection lost!\r\n", UART_COLOR_WHITE);
			uart_report("Cannon standing still\r\n", UART_COLOR_YELLOW);	
			heartbeat_OK = false;
		}
	}
	
	heartbeat_recvd = false;
}

void heartbeat_configure(void)
{
	struct tc_config config_timer;
	tc_get_config_defaults(&config_timer);
	config_timer.counter_size			= TC_COUNTER_SIZE_8BIT;
	config_timer.clock_source			= GCLK_GENERATOR_4;
	config_timer.clock_prescaler		= TC_CLOCK_PRESCALER_DIV1024;
	config_timer.counter_8_bit.period	= 244;	// 2 sec
	tc_init(&heartbeat_timer, TC3, &config_timer);
	tc_enable(&heartbeat_timer);
}
void heartbeat_configure_callbacks(void)
{
	tc_register_callback(&heartbeat_timer, heartbeat_timeout, TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&heartbeat_timer, TC_CALLBACK_OVERFLOW);
}