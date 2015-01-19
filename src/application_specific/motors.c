/*
 * motors.c
 *
 * Created: 14.07.2014 10:59:45
 *  Author: magne.normann
 */ 

#include "conf_motors.h"
#include "motors.h"

/* Initialization of the two servos used to drive the Abot */
void motors_init(void)
{
	struct tcc_config config_motors;
	tcc_get_config_defaults(&config_motors, CONF_MOTORS_MODULE);
	
	/* Generate Single Slope PWM signals with period set to 20 ms */
	config_motors.counter.period							= PWM_COUNTER_PERIOD;
	config_motors.counter.clock_source						= GCLK_GENERATOR_3;		// 8 Mhz
	config_motors.counter.clock_prescaler					= TCC_CLOCK_PRESCALER_DIV8;
	config_motors.compare.wave_generation					= TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	
	/* Set default speed to zero. (Motors are operational in the range [1200:1900] */
	config_motors.compare.match[CONF_MOTORS_LEFT_CHANNEL]	= NEUTRAL;
	config_motors.compare.match[CONF_MOTORS_RIGHT_CHANNEL]	= NEUTRAL;
	
	/* Enable PWM Output on pins defined in the conf_motor header */
	config_motors.pins.enable_wave_out_pin[CONF_MOTORS_LEFT_OUTPUT]		= true;
	config_motors.pins.enable_wave_out_pin[CONF_MOTORS_RIGHT_OUTPUT]	= true;
	config_motors.pins.wave_out_pin[CONF_MOTORS_LEFT_OUTPUT]			= CONF_MOTORS_LEFT_OUT_PIN;
	config_motors.pins.wave_out_pin[CONF_MOTORS_RIGHT_OUTPUT]			= CONF_MOTORS_RIGHT_OUT_PIN;
	config_motors.pins.wave_out_pin_mux[CONF_MOTORS_LEFT_OUTPUT]		= CONF_MOTORS_LEFT_OUT_MUX;
	config_motors.pins.wave_out_pin_mux[CONF_MOTORS_RIGHT_OUTPUT]		= CONF_MOTORS_RIGHT_OUT_MUX;
	
	/* Configuration is done! initialize the motors */
	tcc_init(&motors, CONF_MOTORS_MODULE, &config_motors);
	tcc_enable(&motors);
}

/************************************** Abot Commands **********************************************/

void motors_goForward(void){
	TCC0->CC[2].reg = COUNTERCLOCKWISE;
	TCC0->CC[3].reg = CLOCKWISE;
}

void motors_goLeft(void){
	TCC0->CC[2].reg = CLOCKWISE;
	TCC0->CC[3].reg = CLOCKWISE;
}

void motors_goRight(void){
	TCC0->CC[2].reg = COUNTERCLOCKWISE;
	TCC0->CC[3].reg = COUNTERCLOCKWISE;
}

void motors_stop(void){
	TCC0->CC[2].reg = NEUTRAL;
	TCC0->CC[3].reg = NEUTRAL;
}

void motors_reverse(void){
	TCC0->CC[2].reg = CLOCKWISE;
	TCC0->CC[3].reg = COUNTERCLOCKWISE;
}