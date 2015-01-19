/*
 * uart_BT.c
 *
 * Created: 10.07.2014 10:56:38
 *  Author: alexander_eide.thors and magne.normann
 */ 
#include <asf.h>
#include "uart_BT.h"
#include "uart_PC.h"
#include "interface.h"

/*Initialize the usart modules used for bluetooth communication*/
void uart_BT_init(void){
	configure_usart_BT1();
	configure_usart_callbacks_BT1();
	
	configure_usart_BT2();
	configure_usart_callbacks_BT2();
	
	usart_read_buffer_job(&usart_BT1, (uint8_t *)rx_buffer_BT1, MAX_rx_buffer_BT_LENGTH);
	usart_read_buffer_job(&usart_BT2, (uint8_t *)rx_buffer_BT2, MAX_rx_buffer_BT_LENGTH);
}

/*Read callback for BT1, called every time a command is received over BT1*/
void usart_read_callback_BT1(const struct usart_module *const usart_module)
{
	/*Set terminal font color to blue to make it clear the request was received over
	  bluetooth	*/
	uart_setColor(UART_COLOR_BLUE);
	process_key((uint8_t *)rx_buffer_BT1);
	usart_read_buffer_job(&usart_BT1, (uint8_t *)rx_buffer_BT1, MAX_rx_buffer_BT_LENGTH);
}

/*Uart write callback - Not used*/
void usart_write_callback_BT1(const struct usart_module *const usart_module)
{
	usart_read_buffer_job(&usart_BT1, (uint8_t *)rx_buffer_BT1, MAX_rx_buffer_BT_LENGTH);
}

/*Read callback for BT2, called for every command received over BT2*/
void usart_read_callback_BT2(const struct usart_module *const usart_module)
{
	uart_setColor(UART_COLOR_BLUE);
	process_key((uint8_t *)rx_buffer_BT2);
	usart_read_buffer_job(&usart_BT2, (uint8_t *)rx_buffer_BT2, MAX_rx_buffer_BT_LENGTH);
}

/*Uart write callback - Not used*/
void usart_write_callback_BT2(const struct usart_module *const usart_module)
{
	usart_read_buffer_job(&usart_BT2, (uint8_t *)rx_buffer_BT2, MAX_rx_buffer_BT_LENGTH);
}

void configure_usart_BT1(void)
{
	struct usart_config config_usart1;
	usart_get_config_defaults(&config_usart1);

	config_usart1.baudrate    = 9600;
	config_usart1.mux_setting = EXT1_UART_SERCOM_MUX_SETTING;
	config_usart1.pinmux_pad0 = EXT1_UART_SERCOM_PINMUX_PAD0;
	config_usart1.pinmux_pad1 = EXT1_UART_SERCOM_PINMUX_PAD1;
	config_usart1.pinmux_pad2 = EXT1_UART_SERCOM_PINMUX_PAD2;
	config_usart1.pinmux_pad3 = EXT1_UART_SERCOM_PINMUX_PAD3;
	
	while (usart_init(&usart_BT1,EXT3_UART_MODULE, &config_usart1) != STATUS_OK) {
	}
	usart_enable(&usart_BT1);
}


void configure_usart_BT2(void)
{
	struct usart_config config_usart2;
	usart_get_config_defaults(&config_usart2);

	config_usart2.baudrate    = 9600;
	config_usart2.mux_setting = USART_RX_1_TX_0_XCK_1;
	config_usart2.pinmux_pad0 = PINMUX_PA04D_SERCOM0_PAD0;
	config_usart2.pinmux_pad1 = PINMUX_PA05D_SERCOM0_PAD1;
	config_usart2.pinmux_pad2 = PINMUX_UNUSED;
	config_usart2.pinmux_pad3 = PINMUX_UNUSED;

	while (usart_init(&usart_BT2,SERCOM0, &config_usart2) != STATUS_OK) {
	} 
	usart_enable(&usart_BT2);
}


void configure_usart_callbacks_BT1(void)
{
	usart_register_callback(&usart_BT1, usart_write_callback_BT1, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_BT1, usart_read_callback_BT1, USART_CALLBACK_BUFFER_RECEIVED);
	
	usart_enable_callback(&usart_BT1, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_BT1, USART_CALLBACK_BUFFER_RECEIVED);
}

void configure_usart_callbacks_BT2(void)
{
	usart_register_callback(&usart_BT2, usart_write_callback_BT2, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_BT2, usart_read_callback_BT2, USART_CALLBACK_BUFFER_RECEIVED);
	
	usart_enable_callback(&usart_BT2, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_BT2, USART_CALLBACK_BUFFER_RECEIVED);
}