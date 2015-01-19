/*
 * interface.h
 *
 * Created: 10.07.2014 11:08:18
 *  Author: alexander_eide.thors
 */ 


#ifndef INTERFACE_H_
#define INTERFACE_H_

#include <asf.h>

/* For DEBUGGING */
#define CMD_LED_ON	'1'

/*Turret key command definitions */
#define TURRET_KEY_FIRE			' '
#define TURRET_KEY_UP			'i'
#define TURRET_KEY_DOWN			'k'
#define TURRET_KEY_LEFT			'j'
#define TURRET_KEY_RIGHT		'l'
#define TURRET_KEY_QUIT			'u'
#define TURRET_KEY_AIM			'n'

/*Abbot key command definitions */
#define ABBOT_KEY_LEFT			'a'
#define ABBOT_KEY_RIGHT			'd'
#define ABBOT_KEY_FWD			'w'
#define ABBOT_KEY_REV			's'
#define ABBOT_KEY_QUIT			'q'

/*Communication protocol definitions */
#define HEARTBEAT_KEY	'.'

void process_key(const uint8_t *key);
void process_sw(void);

#endif /* INTERFACE_H_ */