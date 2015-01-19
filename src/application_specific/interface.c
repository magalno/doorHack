/*
 * interface.c
 *
 * Created: 10.07.2014 11:08:03
 *  Author: alexander_eide.thors and magne.normann
 */ 

#include "interface.h"
#include "uart_PC.h"
#include "uart_BT.h"
#include "application_specific/turret.h"
#include "application_specific/motors.h"
#include "application_specific/heartbeat.h"

/* Function accept one parameter, key. 
   Depending on the key definitions and the received key different motors will be started/stopped
*/

void process_key(const uint8_t *key){
	
	/* FOR DEBUGGING PURPOSES */
	heartbeat_OK = true;		//TODO: Implement heartbeat functionality!
	
	/*Check if the received key was a HEARTBEAT, and unlock tank control accordingly*/
	if (*key == HEARTBEAT_KEY){
		heartbeat_recvd = true;
		heartbeat_OK	= true;
	} 	
	/*Process keys only if HEARTBEAT is ok*/
	else if (heartbeat_OK){
		switch (*key) {
			/*Process abbot keys*/
			case ABBOT_KEY_REV:
				motors_reverse();
				uart_report("Tank reverse\r\n", UART_COLOR_CURRENT);
			break;
			case ABBOT_KEY_FWD:
				motors_goForward();
				uart_report("Tank forward\r\n", UART_COLOR_CURRENT);
			break;
			case ABBOT_KEY_LEFT:
				motors_goLeft();
				uart_report("Tank turning left\r\n", UART_COLOR_CURRENT);
			break;
			case ABBOT_KEY_RIGHT:
				motors_goRight();
				uart_report("Tank turning right\r\n", UART_COLOR_CURRENT);
			break;
			case ABBOT_KEY_QUIT:
				motors_stop();
				uart_report("Tank idle\r\n", UART_COLOR_CURRENT);
			break;
			
			/*Process turret keys*/
			case TURRET_KEY_FIRE:
				if (turret_fire_start()){
					uart_report("FIRE!\r\n", UART_COLOR_RED);
				} else {
					uart_report("Cannon is firing\r\n", UART_COLOR_RED);
				}
			break;
			case TURRET_KEY_UP:
				if (turret_up_start()){
					uart_report("Lifting cannon\r\n", UART_COLOR_CURRENT);
				} else {
					uart_report("Cannon is up!\r\n", UART_COLOR_YELLOW);
				}
			break;
			case TURRET_KEY_DOWN:
				if (turret_down_start()){
					uart_report("Lowering cannon\r\n", UART_COLOR_CURRENT);
				} else {
					uart_report("Cannon is down!\r\n", UART_COLOR_YELLOW);
				}
			break;
			case TURRET_KEY_LEFT:
				if (turret_left_start()){
					uart_report("Turning cannon left\r\n", UART_COLOR_CURRENT);
				} else {
					uart_report("Cannon is left!\r\n", UART_COLOR_YELLOW);
				}
			break;
			case TURRET_KEY_RIGHT:
				if (turret_right_start()){
					uart_report("Turning cannon right\r\n", UART_COLOR_CURRENT);
				} else {
					uart_report("Cannon is right!\r\n", UART_COLOR_YELLOW);
				}
			break;
			case TURRET_KEY_AIM:
				port_pin_toggle_output_level(TURRET_LED_AIM);
				uart_report("Aiming!\n\r", UART_COLOR_CURRENT);
			break;
			case TURRET_KEY_QUIT:
				turret_stop();
				uart_report("Cannon standing still\r\n", UART_COLOR_YELLOW);
			break;
			case CMD_LED_ON:
				if (turret_fire_start()){
					uart_report("FIRE!\r\n", UART_COLOR_RED);
				} else {
					uart_report("Cannon is firing\r\n", UART_COLOR_RED);
				}
			break;
			default:
				uart_report("Unrecognized command\r\n", UART_COLOR_YELLOW);
			break;
		}
	} 
	/*The connection has timed out*/
	else {
		uart_report("Connection lost\r\n", UART_COLOR_CURRENT);
	}
}

/*Function checks all of the turrets end switches,
  and stops motors corresponding to activated switches.
  a report is also sent to the debug terminal
*/
void process_sw(void){
	if (port_pin_get_input_level(TURRET_SW_RIGHT)==0){
		turret_right_stop();
		uart_report("Cannon is right!\n\r", UART_COLOR_YELLOW);
	}
	
	if (port_pin_get_input_level(TURRET_SW_LEFT)==0){
		turret_left_stop();
		uart_report("Cannon is left!\n\r", UART_COLOR_YELLOW);
	}
	
	if (port_pin_get_input_level(TURRET_SW_UP)==0){
		turret_up_stop();
		uart_report("Cannon is up!\n\r", UART_COLOR_YELLOW);
	}
	
	if (port_pin_get_input_level(TURRET_SW_DOWN)==0){
		turret_down_stop();
		uart_report("Cannon is down!\n\r", UART_COLOR_YELLOW);
	}
	
	/*The fire mechanism need some time to move the motor away from the end switch
	  Check turretState to determine if the fire routine is over, and the motor should stop
	*/
	if (port_pin_get_input_level(TURRET_SW_FIRE)==0 && turretState==TURRET_STATE_READ_SW){
		turret_fire_stop();
		uart_report("Missile launched!\n\r", UART_COLOR_RED);
	}
}