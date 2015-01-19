/**
 *
 * \file
 *
 * \brief Wifi M2M configuration
 *
 * Copyright (C) 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_WIFI_M2M_H_INCLUDED
#define CONF_WIFI_M2M_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"

/*
   ---------------------------------
   -------- Module settings --------
   ---------------------------------
*/

#define CONF_WIFI_M2M_RESET_PIN				PIN_PB06
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN		PIN_PB05
#define CONF_WIFI_M2M_WAKE_PIN				PIN_PB07

#define CONF_WIFI_M2M_INT_PIN				EXT1_IRQ_PIN;
#define CONF_WIFI_M2M_INT_MUX				EXT1_IRQ_MUX;
#define CONF_WIFI_M2M_INT_EIC				(4)

#define USE_SPI								(1)
//#define USE_I2C							(1)

/*
   ---------------------------------
   ---------- SPI settings ---------
   ---------------------------------
*/

#define CONF_WIFI_M2M_SPI_MODULE			EXT1_SPI_MODULE
#define CONF_WIFI_M2M_SPI_SERCOM_MUX		EXT1_SPI_SERCOM_MUX_SETTING
#define CONF_WIFI_M2M_SPI_PINMUX_PAD0		EXT1_SPI_SERCOM_PINMUX_PAD0//in
#define CONF_WIFI_M2M_SPI_PINMUX_PAD1		PINMUX_UNUSED//cs drive from software
#define CONF_WIFI_M2M_SPI_PINMUX_PAD2		EXT1_SPI_SERCOM_PINMUX_PAD2//out
#define CONF_WIFI_M2M_SPI_PINMUX_PAD3		EXT1_SPI_SERCOM_PINMUX_PAD3//sck
#define CONF_WIFI_M2M_SPI_CS_PIN			EXT1_PIN_SPI_SS_0
#define CONF_WIFI_M2M_SPI_BAUDRATE			(4000000UL)

/*
   ---------------------------------
   --------- Debug options ---------
   ---------------------------------
*/

#define CONF_WIFI_M2M_DEBUG					(0)
#define CONF_WIFI_M2M_PRINTF				printf

/*
   ---------------------------------
   --------- WiFi Config  ----------
   ---------------------------------
*/
#define CONF_WIFI_MAC_ADDRESS_ENABLE        (0)
#define CONF_WIFI_MAC_ADDRESS               { 0xf8, 0xf0, 0x05, 0x20, 0x0b, 0x00 }
#define CONF_WIFI_M2M_DEVICE_NAME_ENABLE    (0)
#define CONF_WIFI_M2M_DEVICE_NAME           "NMC1500"
/*
   ---------------------------------
   ------ Power save option  -------
   ---------------------------------
*/
#define CONF_WIFI_PS_LISTEN_INTERVAL 		(1)

#define WIFI_PS_MODE_NONE                   (0)
#define WIFI_PS_MODE_STATIC_PS              (1)
#define WIFI_PS_MODE_DYNAMIC_PS             (2)
#define CONF_WIFI_PS_MODE                   WIFI_PS_MODE_NONE


#ifdef __cplusplus
}
#endif

#endif /* CONF_WIFI_M2M_H_INCLUDED */
