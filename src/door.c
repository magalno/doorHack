/*
 * door.c
 *
 * Created: 24.01.2015 10:53:45
 *  Author: Normann
 */ 

#include "door.h"
#include <asf.h>

void door_init(void){
	/*Set driving pins as outputs*/
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	
	/* Set turret motor controller pins as output */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(DOOR_PIN, &pin_conf);
	
	port_pin_set_output_level(DOOR_PIN, false);

	door_timer_configure();
	door_timer_configure_callbacks();
}

/* Callback function for door_open_timer */
void door_timer_timeout(struct tc_module *const module_inst)
{
	/* Stop pushing button */
	port_pin_set_output_level(DOOR_PIN, false);
	port_pin_set_output_level(LED_0_PIN, true);
}

/* wrapper for tc_start_counter */
void door_timer_start(void){
	tc_start_counter(&door_open_timer);
}


void door_timer_configure(void)
{
	struct tc_config config_timer;
	tc_get_config_defaults(&config_timer);
	
	/* Timer will start once for each fire command, and complete 2 sec later */
	config_timer.oneshot				= true;
	config_timer.counter_size			= TC_COUNTER_SIZE_8BIT;
	config_timer.clock_source			= GCLK_GENERATOR_4;
	config_timer.clock_prescaler		= TC_CLOCK_PRESCALER_DIV1024;
	config_timer.counter_8_bit.period	= 122;	// 1 sec
	
	tc_init(&door_open_timer, TC4, &config_timer);
	
	/* Start the timer */
	tc_enable(&door_open_timer);
	tc_stop_counter(&door_open_timer);
}

void door_timer_configure_callbacks(void)
{
	tc_register_callback(&door_open_timer, door_timer_timeout, TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&door_open_timer, TC_CALLBACK_OVERFLOW);
}
