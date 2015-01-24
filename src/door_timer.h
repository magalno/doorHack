/*
 * door_timer.h
 *
 * Created: 24.01.2015 10:21:15
 *  Author: Normann
 */ 


#ifndef DOOR_TIMER_H_
#define DOOR_TIMER_H_

#define DOOR_PIN PIN_PB23

struct tc_module door_open_timer;

void door_timer_timeout(struct tc_module *const module_inst);
void door_timer_start(void);
void door_timer_configure(void);
void door_timer_configure_callbacks(void);




#endif /* DOOR_TIMER_H_ */