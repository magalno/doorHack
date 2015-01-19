/**
 *
 * \file
 *
 * \brief This module contains M2M Wi-Fi APIs implementation.
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

#include "driver\include\m2m_wifi.h"
#include "driver\source\nmdrv.h"
#include "m2m_hif.h"

static volatile uint8 gu8ChNum;
static volatile uint8 gu8scanInProgress = 0;
static tpfAppWifiCb gpfAppWifiCb = NULL;

#ifdef M2M_MGMT
static struct _tstrMgmtCtrl
{
	uint8* pu8Buf;
	uint16 u16Offset;
	uint16 u16Sz;
}
gstrMgmtCtrl =
{ NULL, 0 ,0};
#endif
/**
*	@fn			m2m_wifi_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr, uint8 grp)
*	@brief		WiFi call back function
*	@param [in]	u8OpCode
*					HIF Opcode type.
*	@param [in]	u16DataSize
*					HIF data length.
*	@param [in]	u32Addr
*					HIF address.
*	@param [in]	grp
*					HIF group type.
*	@author		
*	@date		
*	@version	1.0
*/
static void m2m_wifi_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr, uint8 grp)
{
	uint8 rx_buf[8];
	if (u8OpCode == M2M_WIFI_RESP_CON_STATE_CHANGED)
	{
		tstrM2mWifiStateChanged strState;
		if (hif_receive(u32Addr, (uint8*) &strState,sizeof(tstrM2mWifiStateChanged)) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_CON_STATE_CHANGED, &strState);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_MEMORY_RECOVER)
	{
#if 0
		if (hif_receive(u32Addr, rx_buf, 4) == M2M_SUCCESS)
		{
			tstrM2mWifiStateChanged strState;
			m2m_memcpy((uint8*) &strState, rx_buf,sizeof(tstrM2mWifiStateChanged));
			if (app_wifi_recover_cb)
				app_wifi_recover_cb(strState.u8CurrState);
		}
#endif
	}
	else if (u8OpCode == M2M_WIFI_REQ_DHCP_CONF)
	{
		if (hif_receive(u32Addr, rx_buf, 4) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_REQ_DHCP_CONF, rx_buf);
		}
	}
	else if (u8OpCode == M2M_WIFI_REQ_WPS)
	{
		tstrM2MWPSInfo strWps;
		m2m_memset((uint8*)&strWps,0,sizeof(tstrM2MWPSInfo));
		if(hif_receive(u32Addr, (uint8*)&strWps, sizeof(tstrM2MWPSInfo)) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_REQ_WPS, &strWps);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_P2P)
	{
		tstrM2MP2pResp strP2p;
		m2m_memset((uint8*)&strP2p,0,sizeof(tstrM2MP2pResp));
		if(hif_receive(u32Addr, (uint8*)&strP2p,sizeof(tstrM2MP2pResp)) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_P2P, &strP2p);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_AP)
	{
		tstrM2MAPResp strAp;
		m2m_memset((uint8*)&strAp,0,sizeof(tstrM2MAPResp));
		if(hif_receive(u32Addr, (uint8*)&strAp,sizeof(tstrM2MAPResp)) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_AP, &strAp);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_IP_CONFLICT)
	{
		if(hif_receive(u32Addr, rx_buf, 4) == M2M_SUCCESS)
		{
			M2M_DBG("Conflicted IP\"%u.%u.%u.%u\"\n", rx_buf[0], rx_buf[1],rx_buf[2], rx_buf[3]);
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_IP_CONFLICT, NULL);
	
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_SCAN_DONE)
	{
		tstrM2mScanDone strState;
		gu8scanInProgress = 0;
		if(hif_receive(u32Addr, (uint8*)&strState, sizeof(tstrM2mScanDone)) == M2M_SUCCESS)
		{
			gu8ChNum = strState.u8NumofCh;
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_SCAN_DONE, &strState);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_SCAN_RESULT)
	{
		tstrM2mWifiscanResult strScanResult;
		if(hif_receive(u32Addr, (uint8*)&strScanResult, sizeof(tstrM2mWifiscanResult)) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_SCAN_RESULT, &strScanResult);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_CURRENT_RSSI)
	{
		if (hif_receive(u32Addr, rx_buf, 4) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_CURRENT_RSSI, rx_buf);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_CLIENT_INFO)
	{
		if (hif_receive(u32Addr, rx_buf, 4) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_CLIENT_INFO, rx_buf);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_CONNTION_STATE)
	{
		if(hif_receive(u32Addr, rx_buf, 4) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_CONNTION_STATE, rx_buf);
		}
	}
#ifdef M2M_MGMT
	else if(u8OpCode == M2M_WIFI_RESP_WIFI_RX_PACKET)
	{
		tstrM2MWifiRxPacketInfo		strRxPacketInfo;
		if(hif_receive(u32Addr, (uint8*)&strRxPacketInfo, sizeof(tstrM2MWifiRxPacketInfo)) == M2M_SUCCESS)
		{
			u16DataSize -= sizeof(tstrM2MWifiRxPacketInfo);
			if(gstrMgmtCtrl.pu8Buf != NULL)
			{
				if(u16DataSize > (gstrMgmtCtrl.u16Sz + gstrMgmtCtrl.u16Offset))
				{
					u16DataSize = gstrMgmtCtrl.u16Sz;
				}
				u32Addr += sizeof(tstrM2MWifiRxPacketInfo) + gstrMgmtCtrl.u16Offset;
				if(hif_receive(u32Addr , gstrMgmtCtrl.pu8Buf, u16DataSize) != M2M_SUCCESS)
				{
					u16DataSize = 0;
				}
			}
			hif_set_rx_done();
			wifi_monitoring_cb(&strRxPacketInfo, gstrMgmtCtrl.pu8Buf,u16DataSize);
		}
	}
#endif
	else
	{
		M2M_ERR("REQ Not defined %d\n",u8OpCode);
	}
}


sint8 m2m_wifi_init(tstrWifiInitParam * param)
{
	sint8 ret = M2M_SUCCESS;
	
	gpfAppWifiCb = param->pfAppWifiCb;
	gu8scanInProgress = 0;
	
	/* Apply device specific initialization. */
	ret = nm_drv_init(NULL);
	if(ret != M2M_SUCCESS) 	goto _EXIT0;
	/* Initialize host interface module */
	ret = hif_init(NULL);
	if(ret != M2M_SUCCESS) 	goto _EXIT1;
	
	hif_register_wifi_cb(m2m_wifi_cb);

#if (CONF_WIFI_MAC_ADDRESS_ENABLE == 1)
	{
		uint8 mac_addr[M2M_MAC_ADDRES_LEN] = CONF_WIFI_MAC_ADDRESS;
		/* Apply parameter MAC address if provided. */
		ret = m2m_wifi_set_mac_address(mac_addr);
		if(ret != M2M_SUCCESS) 	goto _EXIT2;
	}
#endif 
#if (CONF_WIFI_M2M_DEVICE_NAME_ENABLE == 1)
	/* Set device name for WiFi direct mode. */
	ret = m2m_wifi_set_device_name((uint8 *)CONF_WIFI_M2M_DEVICE_NAME,
			(uint8)m2m_strlen((uint8 *)CONF_WIFI_M2M_DEVICE_NAME));
	if(ret != M2M_SUCCESS) 	goto _EXIT2;	
#endif
	
#if CONF_WIFI_PS_MODE == WIFI_PS_MODE_DYNAMIC_PS
	{
		tstrM2mLsnInt strM2mLsnInt;
		/*power save disabled in that release please wait for the next release*/
		ret = m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);
		if(ret != M2M_SUCCESS) 	goto _EXIT2;	
		strM2mLsnInt.u8LsnInt = CONF_WIFI_PS_LISTEN_INTERVAL;
		ret = m2m_wifi_set_lsn_int(&strM2mLsnInt);
		if(ret != M2M_SUCCESS) 	goto _EXIT2;	
	}
#elif CONF_WIFI_PS_MODE == WIFI_PS_MODE_STATIC_PS
	m2m_wifi_set_sleep_mode(M2M_PS_MANUAL, 1);
	if(ret != M2M_SUCCESS) 	goto _EXIT2;	
#elif CONF_WIFI_PS_MODE == WIFI_PS_MODE_NONE
	ret = m2m_wifi_set_sleep_mode(M2M_NO_PS, 1);
	if(ret != M2M_SUCCESS) 	goto _EXIT2;	
#else
#error "Please select proper value for CONF_WIFI_PS_MODE"
#endif

	return ret;

_EXIT2:
	hif_deinit(NULL);
_EXIT1:
	nm_drv_deinit(NULL);
_EXIT0:
	return ret;
}

sint8  m2m_wifi_deinit(void * arg)
{
#if CONF_WIFI_PS_MODE == WIFI_PS_MODE_DYNAMIC_PS
	m2m_wifi_set_sleep_mode(M2M_NO_PS, 1);	
#endif
	
	hif_deinit(NULL);
	nm_drv_deinit(NULL);
	
	return M2M_SUCCESS;
}


sint8 m2m_wifi_handle_events(void * arg)
{
	return hif_handle_isr();
}

sint8 m2m_wifi_connect(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch)
{
	sint8				ret = M2M_SUCCESS;
	tstrM2mWifiConnect	strConnect;
	tstrM2MWifiSecInfo	*pstrAuthInfo;

	if(u8SecType != M2M_WIFI_SEC_OPEN)
	{
		if((pvAuthInfo == NULL)||(m2m_strlen(pvAuthInfo)<=0)||(m2m_strlen(pvAuthInfo)>=M2M_MAX_PSK_LEN))
		{
			M2M_ERR("PSK LEN INVALID\n");
			ret = M2M_ERR_FAIL;
			goto ERR1;
		}
	}
	if((u8SsidLen<=0)||(u8SsidLen>=M2M_MAX_SSID_LEN))
	{
		M2M_ERR("SSID LEN INVALID\n");
		ret = M2M_ERR_FAIL;
		goto ERR1;
	}

	if(u16Ch>M2M_WIFI_CH_14)
	{
		if(u16Ch!=M2M_WIFI_CH_ALL)
		{
			M2M_ERR("CH INVALID\n");
			ret = M2M_ERR_FAIL;
			goto ERR1;
		}
	}


	m2m_memcpy(strConnect.au8SSID, (uint8*)pcSsid, u8SsidLen);
	strConnect.au8SSID[u8SsidLen]	= 0;
	strConnect.u16Ch				= NM_BSP_B_L_16(u16Ch);

	pstrAuthInfo = &strConnect.strSec;
	pstrAuthInfo->u8SecType		= u8SecType;

	if(u8SecType == M2M_WIFI_SEC_WEP)
	{
		tstrM2mWifiWepParams	* pstrWepParams = (tstrM2mWifiWepParams*)pvAuthInfo;
		tstrM2mWifiWepParams	*pstrWep = &pstrAuthInfo->uniAuth.strWepInfo;
		pstrWep->u8KeyIndx =pstrWepParams->u8KeyIndx-1;
		
		if(pstrWep->u8KeyIndx >= WEP_KEY_MAX_INDEX)
		{
			M2M_ERR("Invalid Wep key index %d\n", pstrWep->u8KeyIndx);
			ret = M2M_ERR_FAIL;
			goto ERR1;
		}
		pstrWep->u8KeySz = pstrWepParams->u8KeySz-1;
		if ((pstrWep->u8KeySz != WEP_40_KEY_STRING_SIZE)&& (pstrWep->u8KeySz != WEP_104_KEY_STRING_SIZE))
		{
			M2M_ERR("Invalid Wep key length %d\n", pstrWep->u8KeySz);
			ret = M2M_ERR_FAIL;
			goto ERR1;
		}
		m2m_memcpy((uint8*)pstrWep->au8WepKey,(uint8*)pstrWepParams->au8WepKey, pstrWepParams->u8KeySz);
		pstrWep->au8WepKey[pstrWepParams->u8KeySz] = 0;		
	
	}


	else if(u8SecType == M2M_WIFI_SEC_WPA_PSK)
	{
		uint8	u8KeyLen = m2m_strlen((uint8*)pvAuthInfo) + 1;
		m2m_memcpy(pstrAuthInfo->uniAuth.au8PSK, (uint8*)pvAuthInfo, u8KeyLen);
	}
	else if(u8SecType == M2M_WIFI_SEC_802_1X)
	{
		m2m_memcpy((uint8*)&pstrAuthInfo->uniAuth.strCred1x, (uint8*)pvAuthInfo, sizeof(tstr1xAuthCredentials));
	}
	else if(u8SecType == M2M_WIFI_SEC_OPEN)
	{

	}
	else
	{
		M2M_ERR("undefined sec type\n");
		ret = M2M_ERR_FAIL;
		goto ERR1;
	}

	ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_CONNECT, (uint8*)&strConnect, sizeof(tstrM2mWifiConnect),NULL, 0,0);

ERR1:
	return ret;
}

sint8 m2m_wifi_disconnect(void)
{
	return hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_DISCONNECT, NULL, 0, NULL, 0,0);
}
sint8 m2m_wifi_set_mac_address(uint8 au8MacAddress[6])
{
	tstrM2mSetMacAddress strTmp;
	m2m_memcpy((uint8*) strTmp.au8Mac, (uint8*) au8MacAddress, 6);
	return hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_SET_MAC_ADDRESS,
		(uint8*) &strTmp, sizeof(tstrM2mSetMacAddress), NULL, 0,0);
}

sint8 m2m_wifi_set_static_ip(tstrM2MIPConfig * pstrStaticIPConf)
{
	pstrStaticIPConf->u32DNS = NM_BSP_B_L_32(pstrStaticIPConf->u32DNS);
	pstrStaticIPConf->u32Gateway = NM_BSP_B_L_32(pstrStaticIPConf->u32Gateway);
	pstrStaticIPConf->u32StaticIP = NM_BSP_B_L_32(
		pstrStaticIPConf->u32StaticIP);
	pstrStaticIPConf->u32SubnetMask = NM_BSP_B_L_32(
		pstrStaticIPConf->u32SubnetMask);
	return hif_send(M2M_REQ_GRP_IP, M2M_IP_REQ_STATIC_IP_CONF,
		(uint8*) pstrStaticIPConf, sizeof(tstrM2MIPConfig), NULL, 0,0);
}

sint8 m2m_wifi_request_dhcp_client(void)
{
	return hif_send(M2M_REQ_GRP_IP, M2M_IP_REQ_DHCP_CLIENT_CONF, NULL, 0, NULL, 0, 0);
}
sint8 m2m_wifi_request_dhcp_server(uint8* addr)
{
	return hif_send(M2M_REQ_GRP_IP, M2M_IP_REQ_DHCP_SERVER_CONF, addr, 4, NULL, 0, 0);
}

sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt* pstrM2mLsnInt)
{
	return hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_LSN_INT, (uint8*)pstrM2mLsnInt, sizeof(tstrM2mLsnInt), NULL, 0, 0);
}

sint8 m2m_wifi_request_scan(uint8 ch)
{
	sint8	s8Ret = M2M_ERR_SCAN_IN_PROGRESS;

	if(!gu8scanInProgress)
	{
		tstrM2MScan strtmp;
		strtmp.u8ChNum = ch;
		s8Ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_SCAN, (uint8*)&strtmp, sizeof(tstrM2MScan),NULL, 0,0);
		if(s8Ret == M2M_SUCCESS)
		{
			gu8scanInProgress = 1;
		}
		else
		{
			M2M_ERR("SCAN Failed Ret = %d\n",s8Ret);
		}
	}
	else
	{
		M2M_ERR("SCAN In Progress\n");
	}
	return s8Ret;
}
sint8 m2m_wifi_wps(uint8 u8TriggerType)
{
	tstrM2MWPSConnect strtmp;
	
	/* Stop Scan if it is ongoing.
	*/
	gu8scanInProgress = 0;
	strtmp.u8TriggerType = u8TriggerType;
	return hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_WPS, (uint8*)&strtmp,sizeof(tstrM2MWPSConnect), NULL, 0,0);
}
sint8 m2m_wifi_wps_disable(void)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_DISABLE_WPS, NULL,0, NULL, 0, 0);
	return ret;
}
sint8 m2m_wifi_req_client_ctrl(uint8 u8Cmd)
{

	sint8 ret = M2M_SUCCESS;
#ifdef _PS_SERVER_
	tstrM2Mservercmd	strCmd;
	strCmd.u8cmd = u8Cmd; 
	ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_CLIENT_CTRL, (uint8*)&strCmd, sizeof(tstrM2Mservercmd), NULL, 0, 0);
#else
	M2M_ERR("_PS_SERVER_ is not defined\n");
#endif
	return ret;
}
sint8 m2m_wifi_req_server_init(uint8 ch)
{
	sint8 ret = M2M_SUCCESS;
#ifdef _PS_SERVER_
	tstrM2mServerInit strServer;
	strServer.u8Channel = ch;
	ret = hif_send(M2M_REQ_GRP_WIFI,M2M_WIFI_REQ_SERVER_INIT, (uint8*)&strServer, sizeof(tstrM2mServerInit), NULL, 0, 0);
#else
	M2M_ERR("_PS_SERVER_ is not defined\n");
#endif
	return ret;
}
sint8 m2m_wifi_p2p(uint8 u8Channel)
{
	sint8 ret = M2M_SUCCESS; 
	if((u8Channel == 1) || (u8Channel == 6) || (u8Channel == 11))
	{
		tstrM2MP2PConnect strtmp;
		strtmp.u8ListenChannel = u8Channel;
		ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_ENABLE_P2P, (uint8*)&strtmp, sizeof(tstrM2MP2PConnect), NULL, 0,0);
	}
	else
	{
		M2M_ERR("Listen channel should only be 1, 6 or 11\n");
		ret = M2M_ERR_FAIL;
	}
	return ret;
}
sint8 m2m_wifi_p2p_disconnect(void)
{
	sint8 ret = M2M_SUCCESS; 
	ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_DISABLE_P2P, NULL, 0, NULL, 0, 0);
	return ret;
}
sint8 m2m_wifi_enable_ap(CONST tstrM2MAPConfig* pstrM2MAPConfig)
{
	sint8 ret = M2M_SUCCESS; 
	ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_ENABLE_AP, (uint8 *)pstrM2MAPConfig, sizeof(tstrM2MAPConfig), NULL, 0, 0);
	return ret;
}
sint8 m2m_wifi_disable_ap(void)
{
	sint8 ret = M2M_SUCCESS; 
	ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_DISABLE_AP, NULL, 0, NULL, 0, 0);
	return ret;
}
sint8 m2m_wifi_req_curr_rssi(void)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_CURRENT_RSSI, NULL, 0, NULL,0, 0);
	return ret;
}

sint8 m2m_wifi_req_scan_result(uint8 index)
{
	sint8 ret = M2M_SUCCESS;
	tstrM2mReqScanResult strReqScanRlt;
	strReqScanRlt.u8Index = index;
	ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_SCAN_RESULT, (uint8*) &strReqScanRlt, sizeof(tstrM2mReqScanResult), NULL, 0, 0);
	return ret;
}
uint8 m2m_wifi_get_num_ap_found(void)
{
	return gu8ChNum;
}
uint8 m2m_wifi_get_sleep_mode(void)
{
	return hif_get_sleep_mode();
}
sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn)
{
	sint8 ret = M2M_SUCCESS;
	M2M_INFO("POWER SAVE %d\n",PsTyp);
	if(PsTyp != M2M_PS_MANUAL)
	{
		tstrM2mPsType strPs;
		strPs.u8PsType = PsTyp;
		strPs.u8BcastEn = BcastEn;
		ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_SLEEP, (uint8*) &strPs,sizeof(tstrM2mPsType), NULL, 0, 0);
	}
	hif_set_sleep_mode(PsTyp);
	return ret;
}
sint8 m2m_wifi_request_sleep(void)
{
	sint8 ret = M2M_SUCCESS;
	uint8 psType = 0;
	psType = hif_get_sleep_mode();
	if(psType == M2M_PS_MANUAL)
	{
		tstrM2mPsType strPs;

		m2m_memcpy((uint8*) &strPs, (uint8*) &psType, sizeof(tstrM2mPsType));
		ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_SLEEP, (uint8*) &strPs,sizeof(tstrM2mPsType), NULL, 0, 0);
	}
	if((psType == M2M_PS_DEEP_AUTOMATIC)||(psType == M2M_PS_MANUAL))
	{
		hif_chip_sleep();
	}
	return ret;
}
sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength)
{
	tstrM2MDeviceNameConfig strDeviceName;
	if(u8DeviceNameLength >= M2M_DEVICE_NAME_MAX)
	{
		u8DeviceNameLength = M2M_DEVICE_NAME_MAX;
	}
	//pu8DeviceName[u8DeviceNameLength] = '\0';
	u8DeviceNameLength ++;
	m2m_memcpy(strDeviceName.au8DeviceName, pu8DeviceName, u8DeviceNameLength);
	return hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_SET_DEVICE_NAME,
		(uint8*)&strDeviceName, sizeof(tstrM2MDeviceNameConfig), NULL, 0,0);
}

/*void m2m_MemoryDump(void)
{
	hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_MEMORY_DUMP, NULL,0,NULL,0, 0);
}*/
#ifdef M2M_MGMT

sint8 m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl *pstrMtrCtrl, uint8 *pu8PayloadBuffer,
								   uint16 u16BufferSize, uint16 u16DataOffset)
{
	sint8	s8Ret = -1;
	if((pstrMtrCtrl->u8ChannelID >= 1) && (pstrMtrCtrl->u8ChannelID <= 11))
	{
		gstrMgmtCtrl.pu8Buf		= pu8PayloadBuffer;
		gstrMgmtCtrl.u16Sz		= u16BufferSize;
		gstrMgmtCtrl.u16Offset	= u16DataOffset;
		s8Ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_ENABLE_MONITORING | M2M_REQ_DATA_PKT,
			(uint8*)pstrMtrCtrl, sizeof(tstrM2MWifiMonitorModeCtrl), NULL, 0,0);
	}
	return s8Ret;
}

sint8 m2m_wifi_disable_monitoring_mode(void)
{
	return hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_DISABLE_MONITORING, NULL, 0, NULL, 0,0);
}

sint8 m2m_wifi_send_wlan_pkt(uint8 *pu8WlanPacket, uint16 u16WlanHeaderLength, uint16 u16WlanPktSize)
{
	sint8	s8Ret = -1;
	if(pu8WlanPacket != NULL)
	{
		tstrM2MWifiTxPacketInfo		strTxPkt;

		strTxPkt.u16PacketSize		= u16WlanPktSize;
		strTxPkt.u16HeaderLength	= u16WlanHeaderLength;
		s8Ret = hif_send(M2M_REQ_GRP_WIFI, M2M_WIFI_REQ_SEND_WIFI_PACKET | M2M_REQ_DATA_PKT,
		(uint8*)&strTxPkt, sizeof(tstrM2MWifiTxPacketInfo), pu8WlanPacket, u16WlanPktSize, sizeof(tstrM2MWifiTxPacketInfo));
	}
	return s8Ret;
}
#endif
