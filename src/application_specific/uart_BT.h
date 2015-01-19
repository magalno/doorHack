/*
 * uart_BT.h
 *
 * Created: 10.07.2014 10:57:41
 *  Author: alexander_eide.thors
 */ 


#ifndef UART_BT_H_
#define UART_BT_H_

/*Global definition of the bluetooth receiver buffers.*/
#define MAX_rx_buffer_BT_LENGTH 1
volatile uint8_t rx_buffer_BT1[MAX_rx_buffer_BT_LENGTH];
volatile uint8_t rx_buffer_BT2[MAX_rx_buffer_BT_LENGTH];

/*Global declaration of BT usart modules*/
struct usart_module usart_BT1;
struct usart_module usart_BT2;

void uart_BT_init(void);
void usart_read_callback_BT1(const struct usart_module *const usart_module);
void usart_read_callback_BT2(const struct usart_module *const usart_module);
void usart_write_callback_BT1(const struct usart_module *const usart_module);
void usart_write_callback_BT2(const struct usart_module *const usart_module);
void configure_usart_BT1(void);
void configure_usart_BT2(void);
void configure_usart_callbacks_BT1(void);
void configure_usart_callbacks_BT2(void);




#endif /* UART_BT_H_ */