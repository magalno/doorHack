/**
 *
 * \file
 *
 * \brief This module contains M2M host interface APIs implementation.
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

#ifndef _M2M_HIF_
#define _M2M_HIF_

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#include "common\include\nm_common.h"
/*!< Include depends on UNO Board is used or not*/
#ifdef ENABLE_UNO_BOARD
#include "m2m_uno_hif.h"
#endif

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#define M2M_HIF_MAX_PACKET_SIZE      (1600 - 4)
/*!< Maximum size of the buffer could be transferred between Host and Firmware.
*/

#define M2M_HIF_HDR_OFFSET (sizeof(tstrHifHdr))

/**
*	@struct		tstrHifHdr
*	@brief		Structure to hold HIF header
*	@author		Mahfouz Sheref
*	@version	1.0
*/ 
typedef struct 
{
    uint8   u8Gid;		/*!< Group ID */
    uint8   u8Opcode;	/*!< OP code */
    uint16  u16Length;	/*!< Payload length */
}tstrHifHdr;

#ifdef __cplusplus
     extern "C" {
#endif

/*!
@typedef
@brief	used to point to wifi call back function depend on Arduino project or other projects.
@param [in]	u8OpCode
				HIF Opcode type.
@param [in]	u16DataSize
				HIF data length.
@param [in]	u32Addr
				HIF address.
@param [in]	grp
				HIF group type.
@author		Awad A. Bekhet
@date		23 June 2014
@version	1.0
*/
typedef void (*tpfSetWiFiCallBack)(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr, uint8 grp);
extern tpfSetWiFiCallBack pfWifiCb; /*!< pointer to wifi call back function */
/*!
@typedef
@brief	used to point to Socket call back function depend on Arduino project or other projects.
@param [in]	u8OpCode
				HIF Opcode type.
@param [in]	u16DataSize
				HIF data length.
@param [in]	u32Addr
				HIF address.
@param [in]	grp
				HIF group type.
@author		Awad A. Bekhet
@date		23 June 2014
@version	1.0
*/
typedef void (*tpfSetIPCallBack)(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr, uint8 grp);
extern tpfSetIPCallBack pfIpCb;	/*!< pointer to Socket call back function */


NMI_API sint8 hif_init(void * arg);
NMI_API sint8 hif_deinit(void * arg);
NMI_API sint8 hif_send(uint8 u8Gid,uint8 u8Opcode,uint8 *pu8CtrlBuf,uint16 u16CtrlBufSize,
					   uint8 *pu8DataBuf,uint16 u16DataSize, uint16 u16DataOffset);
NMI_API sint8 hif_receive(uint32 u32Addr, uint8 *pu8Buf, uint16 u16Sz);
NMI_API void hif_set_rx_done(void);

/**
*	@fn		m2m_set_WifiCb(tpfSetWiFiCallBack fn)
*	@brief	To set Call back function for WiFi Component
*	@param [in]	fn
*				function which you want to set
*   @author		Awad A. Bekhet
*   @date		23 June 2014
*	@version	1.0
*/
NMI_API void hif_register_wifi_cb(tpfSetWiFiCallBack fn);
/**
*	@fn		m2m_set_IpCb(tpfSetIPCallBack fn)
*	@brief	To set Call back function for Socket Component
*	@param [in]	fn
*				function which you want to set
*   @author		Awad A. Bekhet
*   @date		23 June 2014
*	@version	1.0
*/
NMI_API void hif_register_ip_cb(tpfSetIPCallBack fn);

NMI_API void hif_chip_sleep(void);
NMI_API void hif_chip_wake(void);
NMI_API void hif_set_sleep_mode(uint8 u8Pstype);
NMI_API uint8 hif_get_sleep_mode(void);

#ifdef CORTUS_APP
/**
*	@fn		hif_Resp_handler(uint8 *pu8Buffer, uint16 u16BufferSize)
*	@brief
*	@param [in]	pu8Buffer
*
*	@param [in]	u16BufferSize
*
*   @author		Ahmad.Mohammad.Yahya
*   @date		27 MARCH 2013
*	@version	1.0
*/
NMI_API sint8 hif_Resp_handler(uint8 *pu8Buffer, uint16 u16BufferSize);
#endif

/**
*	@fn		hif_handle_isr(void)
*	@brief	Handle interrupt received from NMC1500 firmware.
*   @author		M.S.M
*   @date		27 MARCH 2013
*   @return     The function SHALL return 0 for success and a negative value otherwise.
*	@version	1.0
*/
NMI_API sint8 hif_handle_isr(void);

#ifdef __cplusplus
}
#endif
#endif
