/*
 * uart_PC.c
 *
 * Created: 10.07.2014 11:00:48
 *  Author: alexander_eide.thors
 */ 
#include <asf.h>
#include "uart_PC.h"
#include "uart_BT.h"
#include "interface.h"

/*Initializes the debug uart interface*/
void uart_PC_init(void){
	configure_usart_PC();
	configure_usart_callbacks_PC();
	
	usart_read_buffer_job(&usart_PC, (uint8_t *)rx_buffer_PC, MAX_rx_buffer_PC_LENGTH);
	
	uart_hideCursor();
	uart_setColor(UART_COLOR_WHITE);
	uart_clearScreen();
}

/*Callback called every time a key is received over the debug interface*/
void usart_read_callback_PC (const struct usart_module *const usart_module)
{
	uart_setColor(UART_COLOR_WHITE);
	process_key((const uint8_t*)&rx_buffer_PC[0]);
	usart_read_buffer_job(&usart_PC, (uint8_t *)rx_buffer_PC, MAX_rx_buffer_PC_LENGTH);
}

/*Callback used for interrupt driven uart send operations*/ 
void usart_write_callback_PC(const struct usart_module *const usart_module)
{
	usart_read_buffer_job(&usart_PC, (uint8_t *)rx_buffer_PC, MAX_rx_buffer_PC_LENGTH);
}

/*Configure the debug interface uart*/
void configure_usart_PC(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.mux_setting = CONF_STDIO_MUX_SETTING;
	config_usart.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	config_usart.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	config_usart.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	config_usart.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	config_usart.baudrate    = CONF_STDIO_BAUDRATE;
	
	while (usart_init(&usart_PC,
	EDBG_CDC_MODULE, &config_usart) != STATUS_OK) {
	}
	usart_enable(&usart_PC);
}


/*Register and enable callbacks*/
void configure_usart_callbacks_PC(void)
{
	usart_register_callback(&usart_PC, usart_write_callback_PC, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_PC, usart_read_callback_PC, USART_CALLBACK_BUFFER_RECEIVED);
	
	usart_enable_callback(&usart_PC, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_PC, USART_CALLBACK_BUFFER_RECEIVED);
}