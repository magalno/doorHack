/**
 *
 * \file
 *
 * \brief NMC1500 IoT Application Interface.
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

#ifndef __M2M_WIFI_H__
#define __M2M_WIFI_H__

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#include "common\include\nm_common.h"
#include "driver\include\m2m_wifi_types.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifdef M2M_MGMT

/*!
@enum	\
	tenuWifiFrameType

@brief
	Basic Wlan Frame Type Codes (2-bit) 
*/
typedef enum {
	MANAGEMENT            = 0x00,
	/*!< Wi-Fi Management frame (Probe Req/Resp, Beacon, Association Req/Resp ...etc).
	*/
	CONTROL               = 0x04,
	/*!< Wi-Fi Control frame (eg. ACK frame).
	*/
	DATA_BASICTYPE        = 0x08,
	/*!< Wi-Fi Data frame.
	*/
	RESERVED              = 0x0C
}tenuWifiFrameType;


/*!
@enum	\
	tenuSubTypes

@brief
	Wi-Fi MAC Frame Sub-Types (6-bit) 
*/
typedef enum {
	ASSOC_REQ             = 0x00,
	ASSOC_RSP             = 0x10,
	REASSOC_REQ           = 0x20,
	REASSOC_RSP           = 0x30,
	PROBE_REQ             = 0x40,
	PROBE_RSP             = 0x50,
	BEACON                = 0x80,
	ATIM                  = 0x90,
	DISASOC               = 0xA0,
	AUTH                  = 0xB0,
	DEAUTH                = 0xC0,
	ACTION                = 0xD0,
	PS_POLL               = 0xA4,
	RTS                   = 0xB4,
	CTS                   = 0xC4,
	ACK                   = 0xD4,
	CFEND                 = 0xE4,
	CFEND_ACK             = 0xF4,
	DATA                  = 0x08,
	DATA_ACK              = 0x18,
	DATA_POLL             = 0x28,
	DATA_POLL_ACK         = 0x38,
	NULL_FRAME            = 0x48,
	CFACK                 = 0x58,
	CFPOLL                = 0x68,
	CFPOLL_ACK            = 0x78,
	QOS_DATA              = 0x88,
	QOS_DATA_ACK          = 0x98,
	QOS_DATA_POLL         = 0xA8,
	QOS_DATA_POLL_ACK     = 0xB8,
	QOS_NULL_FRAME        = 0xC8,
	QOS_CFPOLL            = 0xE8,
	QOS_CFPOLL_ACK        = 0xF8,
	BLOCKACK_REQ          = 0x84,
	BLOCKACK              = 0x94
}tenuSubTypes;


/*!
@enum	\
	tenuInfoElementId

@brief
	Wi-Fi Information Element IDs
*/
typedef enum {
	ISSID               = 0,   
	/*!< Service Set Identifier (SSID)
	*/
	ISUPRATES           = 1,   
	/*!< Supported Rates
	*/
	IFHPARMS            = 2,   
	/*!< FH parameter set
	*/
	IDSPARMS            = 3,   
	/*!< DS parameter set
	*/
	ICFPARMS            = 4,   
	/*!< CF parameter set
	*/
	ITIM                = 5,   
	/*!< Traffic Information Map
	*/
	IIBPARMS            = 6,   
	/*!< IBSS parameter set
	*/
	ICOUNTRY            = 7,   
	/*!< Country element.
	*/
	IEDCAPARAMS         = 12,  
	/*!< EDCA parameter set
	*/
	ITSPEC              = 13,  
	/*!< Traffic Specification
	*/
	ITCLAS              = 14,  
	/*!< Traffic Classification
	*/
	ISCHED              = 15,  
	/*!< Schedule.
	*/
	ICTEXT              = 16,  
	/*!< Challenge Text
	*/
	IPOWERCONSTRAINT    = 32,  
	/*!< Power Constraint.
	*/
	IPOWERCAPABILITY    = 33,  
	/*!< Power Capability
	*/
	ITPCREQUEST         = 34,  
	/*!< TPC Request                    
	*/
	ITPCREPORT          = 35,  
	/*!< TPC Report                     
	*/
	ISUPCHANNEL         = 36, 
	/* Supported channel list  
	*/
	ICHSWANNOUNC        = 37,  
	/*!< Channel Switch Announcement    
	*/
	IMEASUREMENTREQUEST = 38,  
	/*!< Measurement request            
	*/
	IMEASUREMENTREPORT  = 39,  
	/*!< Measurement report             
	*/
	IQUIET              = 40,  
	/*!< Quiet element Info             
	*/
	IIBSSDFS            = 41,  
	/*!< IBSS DFS                       
	*/
	IERPINFO            = 42,  
	/*!< ERP Information                
	*/
	ITSDELAY            = 43,  
	/*!< TS Delay                       
	*/
	ITCLASPROCESS       = 44,  
	/*!< TCLAS Processing               
	*/
	IHTCAP              = 45,  
	/*!< HT Capabilities                
	*/
	IQOSCAP             = 46,  
	/*!< QoS Capability                 
	*/
	IRSNELEMENT         = 48,  
	/*!< RSN Information Element        
	*/
	IEXSUPRATES         = 50,  
	/*!< Extended Supported Rates       
	*/
	IEXCHSWANNOUNC      = 60,  
	/*!< Extended Ch Switch Announcement
	*/
	IHTOPERATION        = 61,  
	/*!< HT Information                 
	*/
	ISECCHOFF           = 62,  
	/*!< Secondary Channel Offeset      
	*/
	I2040COEX           = 72,  
	/*!< 20/40 Coexistence IE           
	*/
	I2040INTOLCHREPORT  = 73,  
	/*!< 20/40 Intolerant channel report
	*/
	IOBSSSCAN           = 74,  
	/*!< OBSS Scan parameters           
	*/
	IEXTCAP             = 127, 
	/*!< Extended capability            
	*/
	IWMM                = 221, 
	/*!< WMM parameters                 
	*/
	IWPAELEMENT         = 221 
	/*!< WPA Information Element        
	*/
}tenuInfoElementId;


/*!
@struct	\
	tenuWifiCapability

@brief
	Capability Information field bit assignments.  
*/
typedef enum{
	ESS            = 0x01,   
	/*!< ESS capability               
	*/
	IBSS           = 0x02,   
	/*!< IBSS mode                    
	*/
	POLLABLE       = 0x04,   
	/*!< CF Pollable                  
	*/
	POLLREQ        = 0x08,   
	/*!< Request to be polled         
	*/
	PRIVACY        = 0x10,   
	/*!< WEP encryption supported     
	*/
	SHORTPREAMBLE  = 0x20,   
	/*!< Short Preamble is supported  
	*/
	SHORTSLOT      = 0x400,  
	/*!< Short Slot is supported      
	*/
	PBCC           = 0x40,   
	/*!< PBCC                         
	*/
	CHANNELAGILITY = 0x80,   
	/*!< Channel Agility              
	*/
	SPECTRUM_MGMT  = 0x100,  
	/*!< Spectrum Management          
	*/
	DSSS_OFDM      = 0x2000  
	/*!< DSSS-OFDM                    
	*/
}tenuWifiCapability;


#endif


#define M2M_CLIENT_CMD_WAKE_FIRMWARE    ((uint8)10)
#define M2M_CLIENT_CMD_LED_ON 			((uint8)12)
#define M2M_CLIENT_CMD_LED_OFF 			((uint8)11)

/**/
#define M2M_CLIENT_RESP_MOVEMENT 		((uint8)20)
#define M2M_CLIENT_RESP_BTN1_PRESS 		((uint8)21)
#define M2M_CLIENT_RESP_BTN2_PRESS 		((uint8)22)
#define M2M_CLIENT_CHECK_STATE			((uint8)0)

typedef void (*tpfAppWifiCb) (uint8 u8MsgType, void * pvMsg);

/**
*	@struct		tstrWifiInitParam
*	@brief		Structure to hold m2m_wifi_init() parameters.
*	@author		M.S.M
*	@version	1.0
*/
typedef struct {
	tpfAppWifiCb pfAppWifiCb;
} tstrWifiInitParam;

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifdef __cplusplus
     extern "C" {
#endif

/*!
@fn	\
	NMI_API void  m2m_wifi_init(void);

@brief
	Initialize the M2M_WIFI layer.

@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8  m2m_wifi_init(tstrWifiInitParam * pWifiInitParam);

/*!
@fn	\
	NMI_API void  m2m_wifi_init(void);

@brief
	Free resources used by the M2M_WIFI layer.

@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8  m2m_wifi_deinit(void * arg);

/*!
@fn	\
	NMI_API void  m2m_wifi_hanle_events(void);

@brief
	M2M event handler loop

@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_handle_events(void * arg);

/*!
@fn	\
	NMI_API sint8 m2m_wifi_connect(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch);

@brief
	Request a Wi-Fi connection to a specific AP.

@param [in]	pcSsid
				A buffer holding the SSID corresponding to the requested AP.
				
@param [in]	u8SsidLen
				Length of the given SSID (not including the NULL termination).

@param [in]	u8SecType
				Wi-Fi security type security for the network. It can be one of the following types:
				- [M2M_WIFI_SEC_OPEN](@ref M2M_WIFI_SEC_OPEN)
				- [M2M_WIFI_SEC_WEP](@ref M2M_WIFI_SEC_WEP)
				- [M2M_WIFI_SEC_WPA_PSK](@ref M2M_WIFI_SEC_WPA_PSK)
				- [M2M_WIFI_SEC_802_1X](@ref M2M_WIFI_SEC_802_1X)

@param [in]	pvAuthInfo
				Authentication parameters reqired for completeing the connection. It is type is based on the Security type.

@param [in]	u16Ch
				Wi-Fi Channel number as in [tenuM2mScanCh](@ref tenuM2mScanCh).

@return		
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tuniM2MWifiAuth
	tstr1xAuthCredentials
	tstrM2mWifiWepParams
*/
NMI_API sint8 m2m_wifi_connect(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_disconnect(void);

@brief
	Request a Wi-Fi disconnect from the currently connected AP.

@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_disconnect(void);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_set_mac_address(uint8 au8MacAddress[6]);

@brief
	Assign MAC address to the NMC1500. It is used for non-production SW.

@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_set_mac_address(uint8 au8MacAddress[6]);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_wps(uint8 u8TriggerType);

@brief
	Initialize the M2M_WIFI host driver.

@param [in]	u8TriggerType
				WPS Trigger method. Could be:
				- [WPS_PIN_TRIGGER](@ref WPS_PIN_TRIGGER)
				- [WPS_PBC_TRIGGER](@ref WPS_PBC_TRIGGER)

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuWPSTrigger
*/
NMI_API sint8 m2m_wifi_wps(uint8 u8TriggerType);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_wps_disable(void);

@brief
	Disable the NMC1500 WPS operation.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_wps_disable(void);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_p2p(uint8 u8Channel);

@brief
	Enable the NMC1500 device to work in Wi-Fi direct mode (P2P).

@param [in]	u8Channel
				P2P Listen channel. It could be 1, 6 or 11 

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_p2p(uint8 u8Channel);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_p2p_disconnect(void);

@brief
	Disable the NMC1500 device Wi-Fi direct mode (P2P).

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_p2p_disconnect(void);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_enable_ap(CONST tstrM2MAPConfig* pstrM2MAPConfig);

@brief
	Enable the NMC1500 device Wi-Fi Hotspot mode (Soft AP).

@param [in]	pstrM2MAPConfig
				A structure holding the AP configurations.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tstrM2MAPConfig
*/
NMI_API sint8 m2m_wifi_enable_ap(CONST tstrM2MAPConfig* pstrM2MAPConfig);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_disable_ap(void);

@brief
	Disable the NMC1500 device Wi-Fi Hotspot mode.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_disable_ap(void);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_set_static_ip(tstrM2MIPConfig * pstrStaticIPConf);

@brief
	Force the use of a certain static IP Address configurations.

@param [in]	pstrStaticIPConf
				Pointer to a structure holding the static IP Configurations (IP, 
				Gateway, subnet mask and DNS address).
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_set_static_ip(tstrM2MIPConfig * pstrStaticIPConf);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_request_dhcp_client(void);

@brief
	Request DHCP IP from the connected AP.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_request_dhcp_client(void);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_request_dhcp_server(uint8* addr);

@brief
	Start a local DHCP Server on the NMC1500 in case of AP mode.

@param [in]	addr
				IP Address assigned to the AP.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_request_dhcp_server(uint8* addr);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_request_scan(uint8 ch);

@brief
	Request Wi-Fi scan on the given channel.

@param [in]	ch
				RF Channel ID for SCAN operation. It should be set according to tenuM2mScanCh. 

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuM2mScanCh
*/
NMI_API sint8 m2m_wifi_request_scan(uint8 ch);


/*!
@fn	\
	NMI_API uint8 m2m_wifi_get_num_ap_found(void);

@brief
	Reads the number of APs found in the last SCAN process.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API uint8 m2m_wifi_get_num_ap_found(void);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_req_scan_result(uint8 index);

@brief
	Reads the AP information from the SCAN Result list with the given index

@param [in]	index
				Index for the requested result

@return
	The function SHALL return 0 for success and a negative value otherwise
*/
NMI_API sint8 m2m_wifi_req_scan_result(uint8 index);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_req_curr_rssi(void);

@brief
	Request the current RSSI for the current connected AP.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_req_curr_rssi(void);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn);

@brief
	Set the power saving mode for the NMC1500. 

@param [in]	PsTyp
				Desired power saving mode. Supported types are defined in tenuPowerSaveModes.

@param [in]	BcastEn
				Broadcast reception enable flag. 
				If it is 1, the NMC1500 must be awake each DTIM Beacon for receiving Broadcast traffic.
				If it is 0, the NMC1500 will not wakeup at the DTIM Beacon, but its wakeup depends only 
				on the the configured Listen Interval. 

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuPowerSaveModes
*/
NMI_API sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_request_sleep(void);

@brief
	Set the NMC1500 device to work in the current configured Power save mode.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuPowerSaveModes
*/
NMI_API sint8 m2m_wifi_request_sleep(void);


/*!
@fn	\
	NMI_API uint8 m2m_wifi_get_sleep_mode(void);

@brief
	Get the current Power save mode.

@return
	The current operating power saving mode.

@sa
	tenuPowerSaveModes
*/
NMI_API uint8 m2m_wifi_get_sleep_mode(void);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_req_client_ctrl(uint8 cmd);

@brief
	Send a command to the PS Client (An NMC1500 board running the ps_firmware)

@param [in]	cmd
				Cotrol command sent from PS Server to PS Client.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_req_client_ctrl(uint8 cmd);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_req_server_init(uint8 ch);

@brief
	Initialize the PS Server

@param [in]	ch
				Server listening channel

@return
	The function SHALL return 0 for success and a negative value otherwise
*/
NMI_API sint8 m2m_wifi_req_server_init(uint8 ch);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength);

@brief
	Set the NMC1500 device name which is used as P2P device name.

@param [in]	pu8DeviceName
				Buffer holding the device name.

@param [in]	u8DeviceNameLength
				Length of the device name.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt * pstrM2mLsnInt);

@brief
	Set the Wi-Fi listen interval for power save operation. It is represented in units
	of AP Beacon periods.

@param [in]	pstrM2mLsnInt
				Structure holding the listen interval configurations.
	
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt *pstrM2mLsnInt);


#ifdef M2M_MGMT

/*!
@fn	\
	NMI_API sint8 m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl *pstrMtrCtrl, uint8 *pu8PayloadBuffer, \ 
										   uint16 u16BufferSize, uint16 u16DataOffset);

@brief
	Enable Wi-Fi monitoring mode (Promiscuous mode) with the given filtering information. All packets that meet the
	filtering criteria are passed to the application through calling the function wifi_monitoring_cb.

@param [in]	pstrMtrCtrl
				Pointer to a structure holding the Filtering parameters. The Channel ID must be set to a value
				between 1 and 11.

@param [in]	pu8PayloadBuffer
				Pointer to a Buffer allocated by the application. The buffer SHALL hold the Data field of 
				the Wi-Fi Rx Packet (Or a part from it). If it is set to NULL, the Wi-Fi data payload will 
				be discarded by the monitoring driver.

@param [in]	u16BufferSize
				The total size of the pu8PayloadBuffer in bytes.

@param [in]	u16DataOffset
				Starting offset in the DATA FIELD of the received Wi-Fi packet. The application may be interested
				in reading specific information from the received packet. It must assign the offset to the starting
				position of it relative to the DATA payload start.
				Example, if the SSID is needed to be read from a PROBE REQ packet, the u16Offset MUST be set to 0.

@return		The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl *pstrMtrCtrl, uint8 *pu8PayloadBuffer, 
										   uint16 u16BufferSize, uint16 u16DataOffset);

/*!
@fn	\
	NMI_API sint8 m2m_wifi_disable_monitoring_mode(void)

@brief
	Disable Wi-Fi monitoring mode (Promiscuous mode).
*/
NMI_API sint8 m2m_wifi_disable_monitoring_mode(void);


/*!
@fn	\
	NMI_API void  wifi_monitoring_cb(tstrM2MWifiRxPacketInfo *pstrWifiRxPacket, uint8 *pu8Payload, uint16 u16PayloadSize);

@brief	
	Deliver a Wi-Fi packet from the monitoring driver. 
	The function MUST be implemented by the application developer. 

@param [in]	pstrWifiRxPacket
				Pointer to a structure holding the Wi-Fi packet header parameters.

@param [in]	pu8Payload
				Buffer holding the Wi-Fi packet payload information required by the application starting from the
				defined OFFSET by the application (when calling m2m_wifi_enable_monitoring_mode). It shall be NULL
				if the application does not need any data from the payload.

@param [in]	u16PayloadSize
				The size of the payload in bytes. It cannot exceed the buffer size given 
				through m2m_wifi_enable_monitoring_mode.
*/
NMI_API void  wifi_monitoring_cb(tstrM2MWifiRxPacketInfo *pstrWifiRxPacket, uint8 *pu8Payload, uint16 u16PayloadSize);


/*!
@fn	\
	NMI_API sint8 m2m_wifi_send_wlan_pkt(uint8 *pu8WlanPacket, uint16 u16WlanHeaderLength, uint16 u16WlanPktSize);

@brief	Transmit a Wi-Fi RAW packet.

	The Wi-Fi packet composition is left to the application developer. 

@param [in]	pu8WlanPacket
				Pointer to a buffer holding the whole Wi-Fi frame.

@param [in]	u16WlanHeaderLength
				The size of the Wi-Fi packet header.

@param [in]	u16WlanPktSize
				The size of the whole packet in bytes. 

@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_send_wlan_pkt(uint8 *pu8WlanPacket, uint16 u16WlanHeaderLength, uint16 u16WlanPktSize);

#endif

#ifdef __cplusplus
}
#endif
#endif /* __M2M_WIFI_H__ */
