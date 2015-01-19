#include <asf.h>
#include "application_specific/uart_PC.h"
#include "application_specific/uart_BT.h"
#include "application_specific/config.h"
#include "application_specific/turret.h"
#include "application_specific/motors.h"
#include "application_specific/interrupts_sw.h"
#include "application_specific/heartbeat.h"

void config_application(void){
	system_init();
		
	/* Init Tank */
	motors_init();
	turret_init();
	
	/* Init PC USUART */
	uart_PC_init();
	
#if BT	
	uart_BT_init();
	
#else
	//TODO:: move wifi init code here
	
	/* Initialize the delay driver. */
	delay_init();
	
	/* Init temp sensor */
	//at30tse_init();
	
	/* Turn LED0 off initially */
	port_pin_set_output_level(LED_0_PIN, true);
		
#endif	
	/* Init interrupts for end-switches*/
	swInterrupt_init();
		
	/* Abot Tank should only be operational when fed with heartbeats from app */
	//heartbeat_init();
	
	/* Enable global interrupts */
	system_interrupt_enable_global();
	
}