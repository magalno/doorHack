/*
 * heartbeat.h
 *
 * Created: 15.07.2014 07:42:16
 *  Author: magne.normann
 */ 


#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_

volatile int heartbeat_recvd;
volatile int heartbeat_OK;

struct tc_module heartbeat_timer;

void heartbeat_init(void);
void heartbeat_timeout(struct tc_module *const module_inst);
void heartbeat_configure(void);
void heartbeat_configure_callbacks(void);


#endif /* HEARTBEAT_H_ */