/*
 * motors.h
 *
 * Created: 14.07.2014 11:02:30
 *  Author: magne.normann
 */ 


#ifndef MOTORS_H_
#define MOTORS_H_

#define PWM_COUNTER_PERIOD	20000-1 // (GCC0/TCC_prescaler)*(1/F_PWM)-1

#define CLOCKWISE			1900	// 1.9/(PWM_PERIOD/PWM_COUNTER_PERIOD)
#define NEUTRAL				1500	// 1.5/(PWM_PERIOD/PWM_COUNTER_PERIOD)
#define COUNTERCLOCKWISE	1200	// 1.2/(PWM_PERIOD/PWM_COUNTER_PERIOD)

struct tcc_module motors;

void motors_init(void);
void motors_goForward(void);
void motors_goLeft(void);
void motors_goRight(void);
void motors_stop(void);
void motors_reverse(void);


#endif /* MOTORS_H_ */