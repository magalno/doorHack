/*
 * turret.h
 *
 * Created: 11.07.2014 15:26:12
 *  Author: alexander_eide.thors and magne.normann
 */ 


#ifndef TURRET_H_
#define TURRET_H_
#define TURRET_STATE_IDLE		-1
#define TURRET_STATE_READ_SW	0

/*Turret motor pin definitions*/
#define TURRET_MOTOR_LEFT		PIN_PB11
#define TURRET_MOTOR_RIGHT		PIN_PB10
#define TURRET_MOTOR_UP			PIN_PB17
#define TURRET_MOTOR_DOWN		PIN_PB22
#define TURRET_MOTOR_FIRE		PIN_PB23

/*Turret end switch pin definitions*/
#define TURRET_SW_UP			PIN_PA28
#define TURRET_SW_DOWN			PIN_PA12
#define TURRET_SW_LEFT			PIN_PA27
#define TURRET_SW_RIGHT			PIN_PA03
#define TURRET_SW_FIRE			PIN_PB30

/*Turret LED/LASER pin definitions*/
#define TURRET_LED_AIM			PIN_PA02

/*turretState global declaration*/
volatile int turretState;

void turret_init(void);
int turret_fire_start(void);
int turret_up_start(void);
int turret_down_start(void);
int turret_left_start(void);
int turret_right_start(void);

void turret_fire_stop(void);
void turret_up_stop(void);
void turret_down_stop(void);
void turret_left_stop(void);
void turret_right_stop(void);

void turret_stop(void);

struct tc_module turret_firing_timer;

void turret_timer_timeout(struct tc_module *const module_inst);
void turret_timer_start(void);
void turret_timer_configure(void);
void turret_configure_callbacks(void);



#endif /* TURRET_H_ */