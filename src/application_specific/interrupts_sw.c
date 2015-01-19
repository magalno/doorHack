/*
 * interrupts_sw.c
 *
 * Created: 15.07.2014 09:08:04
 *  Author: alexander_eide.thors
 */ 
#include <asf.h>
#include "application_specific/turret.h"
#include "application_specific/interrupts_sw.h"
#include "application_specific/interface.h"

/*Function configures external interrupts for all end switches*/
void swInterrupt_init(void){
	configure_swInterrupt_channel();
	configure_swInterrupt_callbacks();	
}

/*Configure interrupt channels*/
void configure_swInterrupt_channel(void)
{
/*Crate and initialize configuration container */
struct extint_chan_conf config_swInterrupt_chan;
extint_chan_get_config_defaults(&config_swInterrupt_chan);

/*Register the relevant pins as inputs*/
config_swInterrupt_chan.gpio_pin_mux = SYSTEM_PINMUX_PIN_DIR_INPUT;

/*Enable internal pull ups */
config_swInterrupt_chan.gpio_pin_pull = EXTINT_PULL_UP;

/*Trigger interrupt only on falling edges*/
config_swInterrupt_chan.detection_criteria = EXTINT_DETECT_FALLING;

/*Connect the different switches to their external interrupt controller channels*/
config_swInterrupt_chan.gpio_pin = TURRET_SW_RIGHT;
extint_chan_set_config(3,&config_swInterrupt_chan);

config_swInterrupt_chan.gpio_pin = TURRET_SW_LEFT;
extint_chan_set_config(15, &config_swInterrupt_chan);


config_swInterrupt_chan.gpio_pin = TURRET_SW_UP;
extint_chan_set_config(8, &config_swInterrupt_chan);

config_swInterrupt_chan.gpio_pin = TURRET_SW_DOWN;
extint_chan_set_config(12, &config_swInterrupt_chan);

config_swInterrupt_chan.gpio_pin = TURRET_SW_FIRE;
extint_chan_set_config(14, &config_swInterrupt_chan);
}

/*Enable and register callbacks for the used EIC channels*/
void configure_swInterrupt_callbacks(void)
{
extint_register_callback(swInterrupt_callback,15,EXTINT_CALLBACK_TYPE_DETECT);
extint_chan_enable_callback(15,EXTINT_CALLBACK_TYPE_DETECT);

extint_register_callback(swInterrupt_callback,14,EXTINT_CALLBACK_TYPE_DETECT);
extint_chan_enable_callback(14,EXTINT_CALLBACK_TYPE_DETECT);

extint_register_callback(swInterrupt_callback,12,EXTINT_CALLBACK_TYPE_DETECT);
extint_chan_enable_callback(12,EXTINT_CALLBACK_TYPE_DETECT);

extint_register_callback(swInterrupt_callback,8,EXTINT_CALLBACK_TYPE_DETECT);
extint_chan_enable_callback(8,EXTINT_CALLBACK_TYPE_DETECT);

extint_register_callback(swInterrupt_callback,3,EXTINT_CALLBACK_TYPE_DETECT);
extint_chan_enable_callback(3,EXTINT_CALLBACK_TYPE_DETECT);
}

/*The switch interrupt callback*/
void swInterrupt_callback(void)
{
	/*Check all switches*/
	process_sw();
}
