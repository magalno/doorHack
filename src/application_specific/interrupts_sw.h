/*
 * interrupts_sw.h
 *
 * Created: 15.07.2014 09:08:14
 *  Author: alexander_eide.thors
 */ 


#ifndef INTERRUPTS_SW_H_
#define INTERRUPTS_SW_H_

void swInterrupt_init(void);
void swInterrupt_callback(void);
void configure_swInterrupt_callbacks(void);
void configure_swInterrupt_channel(void);




#endif /* INTERRUPTS_SW_H_ */