/**
 *
 * \file
 *
 * \brief NMC1500 IoT Application Interface Internal Types.
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

#ifndef __M2M_HOST_IFACE_H__
#define __M2M_HOST_IFACE_H__


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifdef _FIRMWARE_
#include "m2m_common.h"
#else
#include "common/include/nm_common.h"
#endif



/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*======*======*======*======*
		FIRMWARE VERSION NO INFO
 *======*======*======*======*/

#define M2M_FIRMWARE_VERSION_MAJOR_NO 					((uint16)16)
/*!< Firmware Major release version number.
*/


#define M2M_FIRMWARE_VERSION_MINOR_NO					((uint16)0)
/*!< Firmware Minor release version number.
*/



#if !defined(M2M_FIRMWARE_VERSION_MAJOR_NO) || !defined(M2M_FIRMWARE_VERSION_MINOR_NO)
#error Undefined version number
#endif

#define M2M_BUFFER_MAX_SIZE								(1600UL - 4)
/*!< Maximum size for the shared packet buffer.
 */


#define M2M_MAC_ADDRES_LEN                                 6
/*!< The size fo 802 MAC address.
 */

#define M2M_ETHERNET_HDR_OFFSET							34
/*!< The offset of the Ethernet header within the WLAN Tx Buffer.
 */


#define M2M_ETHERNET_HDR_LEN								14
/*!< Length of the Etherenet header in bytes.
*/


#define M2M_MAX_SSID_LEN 									33
/*!< Maximum size for the Wifi SSID including the NULL termination.
 */


#define M2M_MAX_PSK_LEN           								65
/*!< Maximum size for the WPA PSK including the NULL termination.
 */


#define M2M_DEVICE_NAME_MAX								16
/*!< Maximum Size for the device name including the NULL termination.
 */
 

#define M2M_1X_USR_NAME_MAX								21
/*!< The maximum size of the user name including the NULL termination. 
	It is used for RADIUS authentication in case of connecting the device to
	an AP secured with WPA-Enterprise. 
*/


#define M2M_1X_PWD_MAX									41
/*!< The maximum size of the password including the NULL termination. 
	It is used for RADIUS authentication in case of connecting the device to
	an AP secured with WPA-Enterprise. 
*/


#define M2M_CONFIG_CMD_BASE								1
#define M2M_SERVER_CMD_BASE								20
#define M2M_STA_CMD_BASE									40
#define M2M_AP_CMD_BASE										70
#define M2M_P2P_CMD_BASE									90


#define WEP_40_KEY_SIZE 								((uint8)5)
#define WEP_104_KEY_SIZE 								((uint8)13)
#define WEP_40_KEY_STRING_SIZE 						((uint8)10)
#define WEP_104_KEY_STRING_SIZE 						((uint8)26)
#define WEP_KEY_MAX_INDEX								((uint8)4)


#define M2M_SCAN_FAIL 									((uint8)1)
#define M2M_JOIN_FAIL	 								((uint8)2)
#define M2M_AUTH_FAIL 									((uint8)3)
#define M2M_ASSOC_FAIL 								((uint8)4)

#define M2M_SCAN_ERR_WIFI   	 						((sint8)-2)
#define M2M_SCAN_ERR_IP      							((sint8)-3)
#define M2M_SCAN_ERR_AP      							((sint8)-4)		
#define M2M_SCAN_ERR_P2P      							((sint8)-5)		
#define M2M_SCAN_ERR_WPS      							((sint8)-6)		


/*======*======*======*======*
	MONTIORING MODE DEFINITIONS
 *======*======*======*======*/

#define M2M_WIFI_FRAME_TYPE_ANY							0xFF

#define M2M_WIFI_FRAME_SUB_TYPE_ANY						0xFF

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*!
@enum	\
	tenuM2mConfigCmd
	
@brief
	
*/
typedef enum {
	M2M_WIFI_REQ_RESTART = M2M_CONFIG_CMD_BASE,
	M2M_WIFI_REQ_SET_MAC_ADDRESS,
	M2M_WIFI_REQ_CURRENT_RSSI,
	M2M_WIFI_RESP_CURRENT_RSSI,
	M2M_WIFI_REQ_SET_DEVICE_NAME,
	M2M_WIFI_REQ_MEMORY_DUMP,
	M2M_WIFI_RESP_MEMORY_RECOVER
}tenuM2mConfigCmd;


/*!
@enum	\
	tenuM2mServerCmd
	
@brief

*/
typedef enum {
	M2M_WIFI_REQ_CLIENT_CTRL = M2M_SERVER_CMD_BASE,
	M2M_WIFI_RESP_CLIENT_INFO,
	M2M_WIFI_REQ_SERVER_INIT
}tenuM2mServerCmd;


/*!
@enum	\
	tenuM2mStaCmd
	
@brief

*/
typedef enum {
	M2M_WIFI_REQ_CONNECT = M2M_STA_CMD_BASE,
	M2M_WIFI_REQ_DISCONNECT,
	M2M_WIFI_RESP_CON_STATE_CHANGED,
	M2M_WIFI_RESP_CONNTION_STATE,
	M2M_WIFI_REQ_SLEEP,
	M2M_WIFI_REQ_SCAN,
	M2M_WIFI_REQ_WPS_SCAN,
	M2M_WIFI_RESP_SCAN_DONE,
	M2M_WIFI_REQ_SCAN_RESULT,
	M2M_WIFI_RESP_SCAN_RESULT,
	M2M_WIFI_REQ_WPS,
	M2M_WIFI_REQ_START_WPS,
	M2M_WIFI_REQ_DISABLE_WPS,
	M2M_WIFI_REQ_DHCP_CONF,
	M2M_WIFI_RESP_IP_CONFIGURED,
	M2M_WIFI_RESP_IP_CONFLICT,
	M2M_WIFI_REQ_ENABLE_MONITORING,
	M2M_WIFI_REQ_DISABLE_MONITORING,
	M2M_WIFI_RESP_WIFI_RX_PACKET,
	M2M_WIFI_REQ_SEND_WIFI_PACKET,
	M2M_WIFI_REQ_LSN_INT
} tenuM2mStaCmd;


/*!
@enum	\
	tenuM2mP2pCmd
	
@brief

*/
typedef enum {
	M2M_WIFI_REQ_P2P_INT_CONNECT = M2M_AP_CMD_BASE,
	M2M_WIFI_REQ_ENABLE_P2P,
	M2M_WIFI_RESP_P2P,
	M2M_WIFI_REQ_DISABLE_P2P
}tenuM2mP2pCmd;


/*!
@enum	\
	tenuM2mApCmd
	
@brief

*/
typedef enum {
	M2M_WIFI_REQ_ENABLE_AP = M2M_P2P_CMD_BASE,
	M2M_WIFI_REQ_DISABLE_AP,
	M2M_WIFI_RESP_AP
}tenuM2mApCmd;


/*!
@enum	\
	tenuM2mIpCmd
	
@brief

*/
typedef enum {
	/* Request IDs corresponding to the IP GROUP. */
	M2M_IP_REQ_STATIC_IP_CONF = ((uint8) 3),
	M2M_IP_REQ_DHCP_CLIENT_CONF,
	M2M_IP_REQ_DHCP_SERVER_CONF
} tenuM2mIpCmd;


/*!
@enum	\
	tenuM2mConnState
	
@brief
	Wi-Fi Connection State.
*/
typedef enum {
	M2M_WIFI_DISCONNECTED = 0,
	/*!< Wi-Fi state is disconnected.
	*/
	M2M_WIFI_CONNECTED,
	/*!< Wi-Fi state is connected.
	*/
	M2M_WIFI_UNDEF = 0xff
	/*!< Undefined Wi-Fi State.
	*/
}tenuM2mConnState;


/*!
@enum	\
	tenuM2mSecType
	
@brief
	Wi-Fi Supported Security types.
*/
typedef enum {
	M2M_WIFI_SEC_INVALID = 0,
	/*!< Invalid security type.
	*/
	M2M_WIFI_SEC_OPEN,
	/*!< Wi-Fi network is not secured.
	*/
	M2M_WIFI_SEC_WPA_PSK,
	/*!< Wi-Fi network is secured with WPA/WPA2 personal(PSK).
	*/
	M2M_WIFI_SEC_WEP,
	/*!< Security type WEP (40 or 104) OPEN OR SHARED.
	*/
	M2M_WIFI_SEC_802_1X
	/*!< Wifi network is secured with WPA/WPA2 Enterprise.IEEE802.1x user-name/password authentication.
	 */
}tenuM2mSecType;


/*!
@enum	\
	tenuM2mSecType
	
@brief
	Wi-Fi Supported Security types.
*/
typedef enum {
	SSID_MODE_VISIBLE = 0,
	/*!< SSID is visible to others.
	*/
	SSID_MODE_HIDDEN
	/*!< SSID is hidden.
	*/
}tenuM2mSsidMode;

/*!
@enum	\
	tenuM2mScanCh
	
@brief
	Wi-Fi RF Channels.
*/
typedef enum {
	M2M_WIFI_CH_1 = ((uint8) 0),
	M2M_WIFI_CH_2,
	M2M_WIFI_CH_3,
	M2M_WIFI_CH_4,
	M2M_WIFI_CH_5,
	M2M_WIFI_CH_6,
	M2M_WIFI_CH_7,
	M2M_WIFI_CH_8,
	M2M_WIFI_CH_9,
	M2M_WIFI_CH_10,
	M2M_WIFI_CH_11,
	M2M_WIFI_CH_12,
	M2M_WIFI_CH_13,
	M2M_WIFI_CH_14,
	M2M_WIFI_CH_ALL = ((uint8) 255)
}tenuM2mScanCh;


/*!
@enum	\
	tenuPowerSaveModes
	
@brief
	Power Save Modes.
*/
typedef enum {
	M2M_NO_PS,
	M2M_PS_AUTOMATIC,
	M2M_PS_H_AUTOMATIC,
	M2M_PS_DEEP_AUTOMATIC,
	M2M_PS_MANUAL
}tenuPowerSaveModes;


/*!
@enum	\
	tenuWPSTrigger
	
@brief
	WPS Triggering Methods.
*/
typedef enum{
	WPS_PIN_TRIGGER = 0,
	/*!< WPS is triggered in PIN method.
	*/
	WPS_PBC_TRIGGER = 4
	/*!< WPS is triggered via push button.
	*/
}tenuWPSTrigger;


/*!
@struct	\
	tstrM2mWifiWepParams

@brief
	WEP security key parameters.

@author
	Ahmed Ezzat
*/
typedef struct{
	uint8	u8KeyIndx;
	/*!< Wep key Index.
	*/
	uint8	u8KeySz;
	/*!< Wep key Size.
	*/
	uint8	au8WepKey[WEP_104_KEY_STRING_SIZE + 1];
	/*!< WEP Key represented as a NULL terminated ASCII string.
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes to keep the structure word alligned.
	*/
}tstrM2mWifiWepParams;


/*!
@struct	\
	tstr1xAuthCredentials

@brief
	Credentials for the user to authenticate with the AAA server (WPA-Enterprise Mode IEEE802.1x).

@author
	Ahmed Ezzat
*/
typedef struct{
	uint8	au8UserName[M2M_1X_USR_NAME_MAX];
	/*!< User Name. It must be Null terminated string.
	*/
	uint8	au8Passwd[M2M_1X_PWD_MAX];
	/*!< Password corresponding to the user name. It must be Null terminated string.
	*/
}tstr1xAuthCredentials;


/*!
@union	\
	tuniM2MWifiAuth

@brief
	Wi-Fi Security Parameters for all supported security modes.

@author
	Ahmed Ezzat
*/
typedef union{
	uint8				au8PSK[M2M_MAX_PSK_LEN];
	/*!< Pre-Shared Key in case of WPA-Personal security.
	*/
	tstr1xAuthCredentials	strCred1x;
	/*!< Credentials for RADIUS server authentication in case of WPA-Enterprise security.
	*/
	tstrM2mWifiWepParams	strWepInfo;
	/*!< WEP key parameters in case of WEP security.
	*/
}tuniM2MWifiAuth;


/*!
@struct	\
	tstrM2MWifiSecInfo

@brief
	Authentication credentials to connect to a Wi-Fi network.

@author
	Ahmed Ezzat
*/
typedef struct{
	tuniM2MWifiAuth		uniAuth;
	/*!< Union holding all possible authentication parameters corresponding the current security types.
	*/
	uint8				u8SecType;
	/*!< Wi-Fi network security type. See tenuM2mSecType for supported security types.
	*/
#define __PADDING__		(4 - ((sizeof(tuniM2MWifiAuth) + 1) % 4))
	uint8				__PAD__[__PADDING__];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2MWifiSecInfo;


/*!
@struct	\
	tstrM2mWifiConnect

@brief	
	Wi-Fi Connect Request

@author
	Ahmed Ezzat
*/
typedef struct{
	tstrM2MWifiSecInfo		strSec;
	/*!< Security parameters for authenticating with the AP.
	*/
	uint16				u16Ch;
	/*!< RF Channel for the target SSID.
	*/
	uint8				au8SSID[M2M_MAX_SSID_LEN];
	/*!< SSID of the desired AP. It must be NULL terminated string.
	*/
#define __CONN_PAD_SIZE__		(4 - ((sizeof(tstrM2MWifiSecInfo) + M2M_MAX_SSID_LEN + 2) % 4))
	uint8				__PAD__[__CONN_PAD_SIZE__];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mWifiConnect;


/*!
@struct	\
	tstrM2MWPSConnect

@brief	
	WPS Configuration parameters

@sa 
	tenuWPSTrigger
*/
typedef struct {
	uint8 	u8TriggerType;
	/*!< WPS triggering method (Push button or PIN)
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2MWPSConnect;


/*!
@struct	\
	tstrM2MWPSInfo

@brief	WPS Result

	This structure is passed to the application in response to a WPS request. If the WPS session is completed successfully, the
	structure will have Non-ZERO authentication type. If the WPS Session fails (due to error or timeout) the authentication type
	is set to ZERO.

@sa
	tenuM2mSecType
	
@author
	Ahmed Ezzat

@todo
	Use different error codes to differentaite error types.
*/
typedef struct{
	uint8	u8AuthType;
	/*!< Network authentication type.
	*/
	uint8   	u8Ch;
	/*!< RF Channel for the AP.
	*/
	uint8	au8SSID[M2M_MAX_SSID_LEN];
	/*!< SSID obtained from WPS.
	*/
	uint8	au8PSK[M2M_MAX_PSK_LEN];
	/*!< PSK for the network obtained from WPS.
	*/
}tstrM2MWPSInfo;


/*!
@struct	\
	tstrM2MScan
	
@brief	
	Wi-Fi Scan Request

@sa 
	tenuM2mScanCh
*/
typedef struct {
	uint8 	u8ChNum;
	/*!< The Wi-Fi RF Channel number
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2MScan;


/*!
@struct	\	
	tstrM2mScanDone

@brief	
	Wi-Fi Scan Result
	
@author		
	M.S.M
*/
typedef struct{
	uint8 	u8NumofCh;
	/*!< Number of found APs
	*/
	sint8 	s8ScanState;
	/*!< Scan status
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mScanDone;


/*!
@struct	\
	tstrM2mReqScanResult

@brief	Scan Result Request

	The Wi-Fi Scan results list is stored in Firmware. The application can request a certain scan result by its index.

@author
	M.S.M
*/
typedef struct {
	uint8 	u8Index;
	/*!< Index of the desired scan result
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mReqScanResult;


/*!
@struct	\
	tstrM2mWifiscanResult

@brief	Wi-Fi Scan Result

	Information corresponding to an AP in the Scan Result list identified by its order (index) in the list.

@author
	M.S.M
*/
typedef struct {
	uint8 	u8index; 
	/*!< AP index in the scan result list.
	*/
	sint8 	s8rssi; 
	/*!< AP signal strength.
	*/
	uint8 	u8AuthType; 
	/*!< AP authentication type.
	*/
	uint8 	u8ch; 
	/*!< AP RF channel.
	*/
	uint8	au8BSSID[6];
	/*!< BSSID of the AP.
	*/
	uint8 	au8SSID[M2M_MAX_SSID_LEN]; 
	/*!< AP ssid.
	*/
	uint8 	_PAD8_;
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mWifiscanResult;


/*!
@struct	\		
	tstrM2mWifiStateChanged

@brief		
	Wi-Fi Connection State

@sa			
	M2M_WIFI_DISCONNECTED, M2M_WIFI_CONNECTED, M2M_WIFI_REQ_CON_STATE_CHANGED

@author		
	M. Abdelmawla
*/
typedef struct {
	uint8	u8CurrState;
	/*!< Current Wi-Fi connection state
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mWifiStateChanged;


/*!
@struct	\
	tstrM2mPsType

@brief		
	Power Save Configuration

@sa
	tenuPowerSaveModes

@author		
	M.S.M
*/
typedef struct{
	uint8 	u8PsType;
	/*!< Power save operating mode
	*/
	uint8 	u8BcastEn;
	/*!<
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mPsType;


/*!
@struct	\
	tstrM2mLsnInt

@brief	Listen interval 

	It is the value of the Wi-Fi STA listen interval for power saving. It is given in 
	units of Beacon period. Periodically after the listen interval fires, the STA is
	wakeup requesting any buffered frames for it from the AP (Ps-POLL).
	
@author
	Mohammed Nour
*/
typedef struct {
	uint8 	u8LsnInt;
	/*!< Listen interval in Beacon period count.
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mLsnInt;


/*!
@struct	\
	tstrM2MWifiMonitorModeCtrl

@brief	Wi-Fi Monitor Mode Filter

	This structure sets the filtering criteria for Wlan packets when monitoring mode is enable. The received packets
	matching the filtering parameters, are passed directly to the application.

@author
	Ahmed Ezzat
*/
typedef struct{
	uint8	u8ChannelID;
	/* RF Channel ID.
	*/
	uint8	u8FrameType;
	/*!< It must use values from tenuWifiFrameType. 
	*/
	uint8	u8FrameSubtype;
	/*!< It must use values from tenuSubTypes.
	*/
	uint8	au8SrcMacAddress[6];
	/* ZERO means DO NOT FILTER Source address.
	*/
	uint8	au8DstMacAddress[6];
	/* ZERO means DO NOT FILTER Destination address.
	*/
	uint8	au8BSSID[6];
	/* ZERO means DO NOT FILTER BSSID.
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2MWifiMonitorModeCtrl;


/*!
@struct	\
	tstrM2MWifiRxPacketInfo

@brief	Wi-Fi RX Frame Header

	The M2M application has the ability to allow Wi-Fi monitoring mode for receiving all Wi-Fi Raw frames matching
	a well defined filtering criteria. When a target Wi-Fi packet is received, the header information are extracted and
	assigned in this structure.

@author
	Ahmed Ezzat
*/
typedef struct{
	uint8	u8FrameType;
	/*!< It must use values from tenuWifiFrameType. 
	*/
	uint8	u8FrameSubtype;
	/*!< It must use values from tenuSubTypes.
	*/
	uint8	u8ServiceClass;
	/*!< Service class from wifi header.
	*/
	uint8	u8Priority;
	/*!< Priority from wifi header.
	*/
	uint8	u8HeaderLength;
	/*!< Frame Header length.
	*/
	uint8	u8CipherType;
	/*!< Encryption type for the rx packet.
	*/
	uint8	au8SrcMacAddress[6];
	/* ZERO means DO NOT FILTER Source address.
	*/
	uint8	au8DstMacAddress[6];
	/* ZERO means DO NOT FILTER Destination address.
	*/
	uint8	au8BSSID[6];
	/* ZERO means DO NOT FILTER BSSID.
	*/
	uint16	u16DataLength;
	/*!< Data payload length (Header excluded).
	*/
	uint16	u16FrameLength;
	/*!< Total frame length (Header + Data).
	*/
	uint32	u32DataRateKbps;
	/*!< Data Rate in Kbps.
	*/
	sint8		s8RSSI;
	/*!< RSSI.
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2MWifiRxPacketInfo;


/*!
@struct	\
	tstrM2MWifiTxPacketInfo

@brief	Wi-Fi Tx Packet Info

	The M2M Application has the ability to compose a RAW Wi-Fi frames (under the application responsibility). When
	transmitting a Wi-Fi packet, the application must supply the firmware with this structure for sending the target frame.

@author
	Ahmed Ezzat
*/
typedef struct{
	uint16	u16PacketSize;
	/*!< Wlan frame length.
	*/
	uint16	u16HeaderLength;
	/*!< Wlan frame header length.
	*/
}tstrM2MWifiTxPacketInfo;


/*!
 @struct	\	
 	tstrM2MP2PConnect

 @brief		
 	Set the device to operate in the Wi-Fi Direct (P2P) mode.

 @author
	Mohammed Nour
*/
typedef struct {
	uint8 	u8ListenChannel; 
	/*!< P2P Listen Channel (1, 6 or 11)
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2MP2PConnect;


/*!
@struct	\	
	tstrM2MP2pResp

@brief		
	Wi-Fi Direct (P2P) session result.

@author
	Mohammed Nour
*/
typedef struct {
	uint8 	u8ErrorCode; 
	/*!< error code
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2MP2pResp;


/*!
@struct	\
	tstrM2MAPConfig

@brief	AP Configuration

	This structure holds the configuration parameters for the M2M AP mode. It should be set by the application when
	it requests to enable the M2M AP operation mode. The M2M AP mode currently supports only WEP security (with
	the NO Security option availabe of course).

@author
	Mohammed Nour
*/
typedef struct {
	uint8 	au8SSID[M2M_MAX_SSID_LEN]; 
	/*!< AP SSID
	*/
	uint8 	u8ListenChannel; 
	/*!< Wi-Fi RF Channel which the AP will operate on
	*/
	uint8	u8KeyIndx; 
	/*!< Wep key Index
	*/
	uint8	u8KeySz; 
	/*!< Wep key Size 
	*/
	uint8	au8WepKey[WEP_104_KEY_STRING_SIZE + 1]; 
	/*!< Wep key 
	*/
	uint8 	u8SecType; 
	/*!< Security type: Open or WEP only in the current implementation
	*/
	uint8 	u8SsidHide;
	/*!< SSID Status "Hidden(1)/Visible(0)"
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing allignment
	*/
}tstrM2MAPConfig;


/*!
@struct	\	
	tstrM2MAPResp

@brief		
	AP Mode Error response code.

@author
	Mohammed Nour
*/
typedef struct {
	uint8 	u8ErrorCode;
	/*!< error code
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2MAPResp;


/*!
@struct	\	
	tstrM2mServerInit

@brief	
	PS Server initialization.
	
@author		
	M.S.M
*/
typedef struct {
	uint8 	u8Channel;
	/*!< Server Listen channel
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mServerInit;


/*!
@struct	\	
	tstrM2mClientState

@brief	
	PS Client State.
	
@author		
	M.S.M
*/
typedef struct {
	uint8 	u8State;
	/*!< PS Client State
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mClientState;


/*!
@struct	\	
	tstrM2Mservercmd

@brief
	PS Server Cmd

@author		
	M.S.M
*/
typedef struct {
	uint8	u8cmd; 
	/*!< PS Server Cmd
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2Mservercmd;


/*!
@struct	\
	tstrM2mSetMacAddress
	
@brief		
	Sets the MAC address from application. It is only intended for testing purpose.
	This method is not used for production SW. Production SW reads MAC Address from EFUSE.

@note		
	It's recommended to call this only before calling connect
*/
typedef struct {
	uint8 	au8Mac[6]; 
	/*!< MAC address array 
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte allignment
	*/
}tstrM2mSetMacAddress;


/*!
@struct	\
 	tstrM2MDeviceNameConfig
 	
@brief	Device name

	It is assigned by the application. It is used mainly for Wi-Fi Direct device 
	discovery and WPS device information.

@author
	Ahmed Ezzat
*/
typedef struct {
	uint8 	au8DeviceName[M2M_DEVICE_NAME_MAX];
	/*!< NULL terminated device name
	*/
}tstrM2MDeviceNameConfig;


/*!
@struct	\	
 	tstrM2MIPConfig
 	
@brief		
 	Static IP configuration.

@author		
	Ahmed Ezzat

@note
 	All member IP addresses are expressed in Network Byte Order (eg. "192.168.10.1" will be expressed as 0x010AA8C0).
 */
typedef struct {
	uint32 	u32StaticIP;
	/*!< The static IP assigned to the device.
	*/
	uint32 	u32Gateway;
	/*!< IP of the Default internet gateway.
	*/
	uint32 	u32DNS;
	/*!< IP for the DNS server.
	*/
	uint32 	u32SubnetMask;
	/*!< Subnet mask for the local area network.
	*/
} tstrM2MIPConfig;

#endif
