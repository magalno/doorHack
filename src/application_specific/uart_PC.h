/*
 * uart_PC.h
 *
 * Created: 10.07.2014 11:00:36
 *  Author: alexander_eide.thors
 */ 


#ifndef UART_PC_H_
#define UART_PC_H_
#include <asf.h>

#define CONF_STDIO_USART_MODULE  EDBG_CDC_MODULE
#define CONF_STDIO_MUX_SETTING   EDBG_CDC_SERCOM_MUX_SETTING
#define CONF_STDIO_PINMUX_PAD0   EDBG_CDC_SERCOM_PINMUX_PAD0
#define CONF_STDIO_PINMUX_PAD1   EDBG_CDC_SERCOM_PINMUX_PAD1
#define CONF_STDIO_PINMUX_PAD2   EDBG_CDC_SERCOM_PINMUX_PAD2
#define CONF_STDIO_PINMUX_PAD3   EDBG_CDC_SERCOM_PINMUX_PAD3
#define CONF_STDIO_BAUDRATE      9600

/*Uart ANSI terminal collor commands*/
#define UART_COLOR_BLUE        "\033[22;34m"
#define UART_COLOR_RED         "\033[22;31m"
#define UART_COLOR_YELLOW      "\033[01;33m"
#define UART_COLOR_WHITE       "\033[01;37m"
#define UART_COLOR_CURRENT     0

/*Global receiver buffer defined for debug interface.*/
#define MAX_rx_buffer_PC_LENGTH 1
volatile uint8_t rx_buffer_PC[MAX_rx_buffer_PC_LENGTH];

/*Global definition of the PC uart*/
struct usart_module usart_PC;

void uart_PC_init(void);

/*Sends a text string over the uart using the specified color*/
#define uart_report(text, color)	\
do { \
	if (color != UART_COLOR_CURRENT){ \
		usart_write_buffer_wait(&usart_PC, (const uint8_t *)color, sizeof(color)); \
	} \
	usart_write_buffer_wait(&usart_PC, (const uint8_t *)text, sizeof(text)); \
} while (0)

/*UART ANSI terminal commands*/
#define uart_setColor(color)   usart_write_buffer_wait(&usart_PC, (const uint8_t *)color, sizeof(color))
#define uart_hideCursor()      usart_write_buffer_wait(&usart_PC, (const uint8_t *)"\x1B[?25l", sizeof("\x1B[?25l"))            
#define uart_clearScreen()     usart_write_buffer_wait(&usart_PC, (const uint8_t *)"\x1B[2J\x1B[;H", sizeof("\x1B[2J\x1B[;H"))
#define uart_clearLine()       usart_write_buffer_wait(&usart_PC, (const uint8_t *)"\x1B[2K\r", sizeof("\x1B[2K\r"))

void usart_read_callback_PC(const struct usart_module *const usart_module);
void usart_write_callback_PC(const struct usart_module *const usart_module);
void configure_usart_PC(void);
void configure_usart_callbacks_PC(void);


#endif /* UART_PC_H_ */