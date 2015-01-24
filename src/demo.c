/**
 *
 * \file
 *
 * \brief Wifi NMI temperature sensor demo.
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "asf.h"
#include "demo.h"
#include "door.h"
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "conf_wifi_m2m.h"

/** Message format definitions. */
typedef struct s_msg_temp_keepalive {
	uint8_t id0;
	uint8_t id1;
	uint8_t name[9];
	uint8_t type;
} t_msg_temp_keepalive;

typedef struct s_msg_temp_report {
	uint8_t id0;
	uint8_t id1;
	uint8_t name[9];
	uint8_t led;
	uint32_t temp;
} t_msg_temp_report;

/** Message format declarations. */
static t_msg_temp_keepalive msg_temp_keepalive =
{
	.id0 = 0,
	.id1 = 1,
	.name = DEMO_PRODUCT_NAME,
	.type = 2,
};

static t_msg_temp_report msg_temp_report =
{
	.id0 = 0,
	.id1 = 2,
	.name = DEMO_PRODUCT_NAME,
	.led = 0,
	.temp = 0,
};

/** Receive buffer definition. */
#define TEST_BUFFER_SIZE 1460
static uint8 gau8SocketTestBuffer[TEST_BUFFER_SIZE];

/** RX and TX socket handlers. */
static SOCKET rx_socket = -1;
static SOCKET tx_socket = -1;

/** Wifi status variable. */
static volatile uint8 wifi_connected = 0;

/** Global counter delay for timer. */
static uint32_t delay = 0;

/** SysTick counter for non busy wait delay. */
extern uint32_t ms_ticks;

/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void m2m_wifi_socket_handler(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	/* Check for socket event on RX socket. */
	if (sock == rx_socket) {
		if (u8Msg == SOCKET_MSG_BIND) {
			tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
			if (pstrBind && pstrBind->status == 0) {
				/* Prepare next buffer reception. */
				recvfrom(sock, gau8SocketTestBuffer, TEST_BUFFER_SIZE, 0);
			}
			else {
				puts("m2m_wifi_socket_handler: bind error!");
			}
		}
		else if (u8Msg == SOCKET_MSG_RECVFROM) {
			tstrSocketRecvMsg *pstrRx = (tstrSocketRecvMsg *)pvMsg;
			if (pstrRx->pu8Buffer && pstrRx->s16BufferSize) {

				/* Check for server report and update led status if necessary. */
				t_msg_temp_report report;
				memcpy(&report, pstrRx->pu8Buffer, sizeof(t_msg_temp_report));
				if (report.id0 == 0 && report.id1 == 2 && (strcmp((char *)report.name, DEMO_PRODUCT_NAME) == 0)) {
					puts("wifi_nc_data_callback: received app message");
					//port_pin_set_output_level(LED_0_PIN, report.led ? true : false);
					port_pin_set_output_level(LED_0_PIN, false);
					port_pin_set_output_level(DOOR_PIN, true);
					door_timer_start();
					delay = 0;
				}

				/* Prepare next buffer reception. */
				recvfrom(sock, gau8SocketTestBuffer, TEST_BUFFER_SIZE, 0);
			}
			else {
				if (pstrRx->s16BufferSize == SOCK_ERR_TIMEOUT) {
					/* Prepare next buffer reception. */
					recvfrom(sock, gau8SocketTestBuffer, TEST_BUFFER_SIZE, 0);
				}
			}
		}
	}
}

/**
 * \brief Callback to get the Wifi status update.
 *
 * \param[in] u8MsgType type of Wifi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void m2m_wifi_state(uint8 u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
		case M2M_WIFI_RESP_CON_STATE_CHANGED: {
			tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged*) pvMsg;
			if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
				puts("m2m_wifi_state: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED");
				m2m_wifi_request_dhcp_client();
			}
			else if(pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
				puts("m2m_wifi_state: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED");
				wifi_connected = 0;
				m2m_wifi_connect((char *)DEMO_WLAN_SSID, sizeof(DEMO_WLAN_SSID),
						DEMO_WLAN_AUTH, (char *)DEMO_WLAN_PSK, M2M_WIFI_CH_ALL);
			}
			break;
		}
		case M2M_WIFI_REQ_DHCP_CONF: {
			uint8 *pu8IPAddress = (uint8*) pvMsg;
			wifi_connected = 1;
			/* Turn LED0 on to declare that IP address received. */
			port_pin_set_output_level(LED_0_PIN, false);
			printf("m2m_wifi_state: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\n",
					pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
			break;
		}
	default: {
			break;
		}
	}
}

/**
 * \brief Sensor thread entry.
 *
 * \param[in] params unused parameter.
 */
void demo_start(void)
{
	tstrWifiInitParam param;
	struct sockaddr_in addr;
	sint8 ret;
	
	/* Initialize Wi-Fi parameters structure. */
	param.pfAppWifiCb = m2m_wifi_state;

	/* Initialize socket address structure. */
	addr.sin_family	= AF_INET;
	addr.sin_port = _htons(DEMO_SERVER_PORT);
	addr.sin_addr.s_addr = 0xFFFFFFFF;
	
	/* Turn LED0 off initially. */
	port_pin_set_output_level(LED_0_PIN, true);
	
	door_init();

	/* Initialize temperature sensor. */
	at30tse_init();
	
	/* Reset network controller */
	nm_bsp_init();
	
	/* Initialize Wifi driver with data and Wifi status callbacks. */
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		puts("demo_start: nm_drv_init call error!");
		while (1)
			;
	}
	
	/* Initialize Socket module */
	socketInit();
	registerSocketCallback(m2m_wifi_socket_handler, NULL);

	/* Connect to router. */
	m2m_wifi_connect((char *)DEMO_WLAN_SSID, sizeof(DEMO_WLAN_SSID),
			DEMO_WLAN_AUTH, (char *)DEMO_WLAN_PSK, M2M_WIFI_CH_ALL);

	while (1) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);

		if ((wifi_connected == 1) && (ms_ticks - delay > DEMO_REPORT_INTERVAL)) {
			delay = ms_ticks;

			/* Open server socket. */
			if (rx_socket < 0) {
				if ((rx_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
					puts("demo_start: failed to create RX UDP client socket error!");
					continue;
				}
				bind(rx_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
			}
			
			/* Open client socket. */
			if (tx_socket < 0) {
				if ((tx_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
					puts("demo_start: failed to create TX UDP client socket error!");
					continue;
				}
			}
			
			/* Send client discovery frame. */
			sendto(tx_socket, &msg_temp_keepalive, sizeof(t_msg_temp_keepalive), 0,
					(struct sockaddr *)&addr, sizeof(addr));
			
			/* Send client report. */
			msg_temp_report.temp = (uint32_t)(at30tse_read_temperature() * 100);
			msg_temp_report.led = !port_pin_get_output_level(LED_0_PIN);
			ret = sendto(tx_socket, &msg_temp_report, sizeof(t_msg_temp_report), 0,
					(struct sockaddr *)&addr, sizeof(addr));

			if (ret == M2M_SUCCESS) {
				puts("demo_start: sensor report sent");
			} else {
				puts("demo_start: failed to send status report error!");
			}

		}
	}
}
