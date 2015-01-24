/*
 * door.h
 *
 * Created: 24.01.2015 10:53:58
 *  Author: Normann
 */ 


#ifndef DOOR_H_
#define DOOR_H_

#define DOOR_PIN PIN_PB23

struct tc_module door_open_timer;

void door_init(void);
void door_timer_timeout(struct tc_module *const module_inst);
void door_timer_start(void);
void door_timer_configure(void);
void door_timer_configure_callbacks(void);



#endif /* DOOR_H_ */