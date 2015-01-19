/*
 * turret.c
 *
 * Created: 11.07.2014 15:26:01
 *  Author: alexander_eide.thors and magne.normann
 */ 
#include "application_specific//turret.h"
#include "application_specific/uart_PC.h"

#include <asf.h>

/*Configure needed timers, pins and state variables*/
void turret_init(void){
	/*Enable the turret state by setting turretState to TURRET_STATE_IDLE */
	turretState = TURRET_STATE_IDLE;
	
	/*Set all turretMotor driving pins as outputs*/
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	
	/* Set turret motor controller pins as output */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(TURRET_MOTOR_FIRE, &pin_conf);
	port_pin_set_config(TURRET_MOTOR_DOWN, &pin_conf);
	port_pin_set_config(TURRET_MOTOR_UP, &pin_conf);
	port_pin_set_config(TURRET_MOTOR_LEFT, &pin_conf);
	port_pin_set_config(TURRET_MOTOR_RIGHT, &pin_conf);
	port_pin_set_config(TURRET_LED_AIM, &pin_conf);
	
	/* When firing, turret end-switch needs to be disabled for 1 second. 
	** Turret_timer callback function re-enables it when time is up */
	turret_timer_configure();
	turret_configure_callbacks();
}

int turret_fire_start(void){
	if (turretState==TURRET_STATE_IDLE){
		
		/* Disable end-switch reading in turret for 2 sec to allow firing */
		turret_timer_start();
		
		/* Fire! */
		port_pin_set_output_level(TURRET_MOTOR_FIRE,1);
		return 1;
	} 
	return 0;
}

void turret_fire_stop(void){
	port_pin_set_output_level(TURRET_MOTOR_FIRE,0);
	turretState=TURRET_STATE_IDLE;
}

int turret_up_start(void){
	if (port_pin_get_input_level(TURRET_SW_UP)!=0){
		port_pin_set_output_level(TURRET_MOTOR_DOWN,0);
		port_pin_set_output_level(TURRET_MOTOR_UP,1);
		return 1; 
	}
	return 0;
}

void turret_up_stop(void){
	if (port_pin_get_output_level(TURRET_MOTOR_UP)==1){
		port_pin_set_output_level(TURRET_MOTOR_UP,0);
	}
}

int turret_down_start(void){
	if (port_pin_get_input_level(TURRET_SW_DOWN)!=0){
		port_pin_set_output_level(TURRET_MOTOR_UP,0);
		port_pin_set_output_level(TURRET_MOTOR_DOWN,1);
		return 1;
	}
	return 0;
}

void turret_down_stop(void){
	if (port_pin_get_output_level(TURRET_MOTOR_DOWN)==1){
		port_pin_set_output_level(TURRET_MOTOR_DOWN,0);
	}
}

int turret_left_start(void){
	if (port_pin_get_input_level(TURRET_SW_LEFT)!=0){
		port_pin_set_output_level(TURRET_MOTOR_RIGHT,0);
		port_pin_set_output_level(TURRET_MOTOR_LEFT,1);
		return 1;
	}
	return 0;
}

void turret_left_stop(void){
	if (port_pin_get_output_level(TURRET_MOTOR_LEFT)==1){
		port_pin_set_output_level(TURRET_MOTOR_LEFT,0);
	}
}

int turret_right_start(void){
	if (port_pin_get_input_level(TURRET_SW_RIGHT)!=0){
		port_pin_set_output_level(TURRET_MOTOR_LEFT,0);
		port_pin_set_output_level(TURRET_MOTOR_RIGHT,1);
		return 1;
	}
	return 0;
}

void turret_right_stop(void){
	if (port_pin_get_output_level(TURRET_MOTOR_RIGHT)==1){
		port_pin_set_output_level(TURRET_MOTOR_RIGHT,0);
	}
}

void turret_stop(void){
	port_pin_set_output_level(TURRET_MOTOR_RIGHT,0);
	port_pin_set_output_level(TURRET_MOTOR_LEFT,0);
	port_pin_set_output_level(TURRET_MOTOR_UP,0);
	port_pin_set_output_level(TURRET_MOTOR_DOWN,0);
}

/************************ Turret firing timer ******************************/

/* Callback function for turret_firing_timer */
void turret_timer_timeout(struct tc_module *const module_inst)
{
	/* Re-enable end-switch in turret */
	turretState=TURRET_STATE_READ_SW;
}

/* wrapper for tc_start_counter */
void turret_timer_start(void){
	tc_start_counter(&turret_firing_timer);
}


void turret_timer_configure(void)
{	
	struct tc_config config_timer;
	tc_get_config_defaults(&config_timer);
	
	/* Timer will start once for each fire command, and complete 2 sec later */
	config_timer.oneshot				= true;
	config_timer.counter_size			= TC_COUNTER_SIZE_8BIT;
	config_timer.clock_source			= GCLK_GENERATOR_4;
	config_timer.clock_prescaler		= TC_CLOCK_PRESCALER_DIV1024;
	config_timer.counter_8_bit.period	= 244;	// 2 sec
	
	tc_init(&turret_firing_timer, TC4, &config_timer);
	
	/* Start the timer */
	tc_enable(&turret_firing_timer);
	tc_stop_counter(&turret_firing_timer);
}

void turret_configure_callbacks(void)
{
	tc_register_callback(&turret_firing_timer, turret_timer_timeout, TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&turret_firing_timer, TC_CALLBACK_OVERFLOW);
}


