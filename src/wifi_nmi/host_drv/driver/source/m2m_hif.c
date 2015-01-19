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

#include "common\include\nm_common.h"
#include "driver\source\nmbus.h"
#include "bsp\include\nm_bsp.h"
#include "m2m_hif.h"
#include "driver\include\m2m_wifi_types.h"
#include "driver\source\nmasic.h"

#if (defined EDGE_INTERRUPT)&&(defined LEVEL_INTERRUPT)
#error "only one type of interrupt EDGE_INTERRUPT,LEVEL_INTERRUPT"
#endif

#if !((defined EDGE_INTERRUPT)||(defined LEVEL_INTERRUPT))
#error "define interrupt type EDGE_INTERRUPT,LEVEL_INTERRUPT"
#endif

#ifndef CORTUS_APP
#define NMI_AHB_DATA_MEM_BASE  0x30000
#define NMI_AHB_SHARE_MEM_BASE 0xd0000

#define WIFI_HOST_RCV_CTRL_0	(0x1070)
#define WIFI_HOST_RCV_CTRL_1	(0x1084)
#define WIFI_HOST_RCV_CTRL_2    (0x1078)
#define WIFI_HOST_RCV_CTRL_3    (0x106c)
#define WAKE_VALUE				(0x5678)
#define SLEEP_VALUE				(0x4321)
#define WAKE_REG				(0x1074)

#define HIF_WAKE		((uint8)1)
#define HIF_SLEEP		((uint8)0)


static volatile uint8 gu8ChipMode = 0;
static volatile uint8 gu8ChipSleep = 0;
static volatile uint8 gu8HifSizeDone = 0;
static volatile uint8 gu8Interrupt = 0;

tpfSetWiFiCallBack pfWifiCb = NULL;	/*!< pointer to wifi call back function */
tpfSetIPCallBack pfIpCb = NULL;		/*!< pointer to Socket call back function */

static void isr(void)
{
	gu8Interrupt++;
}

void hif_chip_wake(void)
{
	//if(gu8ChipSleep == HIF_SLEEP)
	{
		if(gu8ChipMode == M2M_PS_MANUAL)
		{
#if CONF_WIFI_PS_MODE == WIFI_PS_MODE_STATIC_PS
			enable_rf_blocks();
#endif
		}
		else if(gu8ChipMode == M2M_PS_DEEP_AUTOMATIC)
		{
#if CONF_WIFI_PS_MODE == WIFI_PS_MODE_DYNAMIC_PS
			//nm_bsp_sleep(100);
			nm_clkless_wake();
			nm_write_reg(WAKE_REG, WAKE_VALUE);

#endif
		}
		else
		{
		}
		gu8ChipSleep = HIF_WAKE;
	}
}

void hif_set_sleep_mode(uint8 u8Pstype)
{
	gu8ChipMode = u8Pstype;
}
uint8 hif_get_sleep_mode(void)
{
	return gu8ChipMode;
}
void hif_chip_sleep(void)
{
	//if(gu8ChipSleep == HIF_WAKE)
	{
		if(gu8ChipMode == M2M_PS_DEEP_AUTOMATIC)
		{
#if CONF_WIFI_PS_MODE == WIFI_PS_MODE_DYNAMIC_PS
			uint32 reg = 0;
			nm_write_reg(WAKE_REG, SLEEP_VALUE);
			/* Clear bit 1 */
			nm_read_reg_with_ret(0x1, &reg);
			if(reg&0x2)
			{
				reg &=~(1 << 1);
				nm_write_reg(0x1, reg);
			}
#endif
		}
		else
		{
		}
		gu8ChipSleep = HIF_SLEEP;
	}
}

void hif_set_rx_done(void)
{
	uint32 reg;
#ifdef EDGE_INTERRUPT
	nm_bsp_interrupt_ctrl(1);
#endif
	/*Clearing RX interrupt*/
	reg = nm_read_reg(WIFI_HOST_RCV_CTRL_0);		
	reg &= ~0x1;

	/* Set RX Done */
	reg |= (1<<1);		
	nm_write_reg(WIFI_HOST_RCV_CTRL_0,reg);
#ifdef LEVEL_INTERRUPT
	nm_bsp_interrupt_ctrl(1);
#endif
	
}


sint8 hif_init(void * arg)
{
	pfWifiCb = NULL;
	pfIpCb = NULL;
	
	gu8ChipSleep = HIF_WAKE;
	gu8ChipMode = M2M_NO_PS;

	gu8Interrupt = 0;
	nm_bsp_register_isr(isr);
	
	return M2M_SUCCESS;
}
sint8 hif_deinit(void * arg)
{
#if 0
	uint32 reg = 0, cnt=0;
	while (reg != M2M_DISABLE_PS)
	{
		nm_bsp_sleep(1);
		reg = nm_read_reg(STATE_REG);
		if(++cnt > 1000)
		{
			M2M_DBG("failed to stop power save\n");
			break;
		}
	}
#endif
	hif_chip_wake();
	
	return M2M_SUCCESS;
}


/**
*	@fn		sint8 hif_send(uint8 u8Gid,uint8 u8Opcode,uint8 *pu8CtrlBuf,uint16 u16CtrlBufSize,
			   uint8 *pu8DataBuf,uint16 u16DataSize, uint16 u16DataOffset)
*	@brief	Send packet using host interface
*	@param [in]	u8Gid
*				Group ID
*	@param [in]	u8Opcode
*				Operation ID
*	@param [in]	pu8Buff
*				Packet buffer. Allocated by the caller
*	@param [in]	Packet buffer size (including the HIF header)
*	@return	M2M_SUCCESS in case of success and -ve error code in case of failure
*	@version	1.0
*/ 
sint8 hif_send(uint8 u8Gid,uint8 u8Opcode,uint8 *pu8CtrlBuf,uint16 u16CtrlBufSize,
			   uint8 *pu8DataBuf,uint16 u16DataSize, uint16 u16DataOffset)
{
	sint8		ret = M2M_ERR_SEND;
	tstrHifHdr	strHif;

	strHif.u8Opcode		= u8Opcode&(~NBIT7);
	strHif.u8Gid		= u8Gid;
	strHif.u16Length	= M2M_HIF_HDR_OFFSET;
	if(pu8DataBuf != NULL)
	{
		strHif.u16Length += u16DataOffset + u16DataSize;
	}
	else
	{
		strHif.u16Length += u16CtrlBufSize;
	}
	hif_chip_wake();
	{
		volatile uint32 reg, dma_addr;
		uint16 cnt = 0;
		uint8	u8PadSize;

		nm_write_reg(0x150014, 0x1);	
		u8PadSize = 0;	

		if (strHif.u16Length & 0x3) 
		{
			u8PadSize = 4 - (strHif.u16Length % 4);
		}
		reg = 0UL;
		reg |= (uint32)u8Gid;
		reg |= ((uint32)u8Opcode<<8);
		reg |= ((uint32)strHif.u16Length<<16);
		nm_write_reg(0x108c,reg);
		reg = 0;
		reg |= (1<<1);
		nm_write_reg(WIFI_HOST_RCV_CTRL_2, reg);
		dma_addr = 0;

		nm_bsp_interrupt_ctrl(0);

		for(cnt = 0; cnt < 1000; cnt ++)
		{			
			reg = nm_read_reg(WIFI_HOST_RCV_CTRL_2);		
			if (!(reg & 0x2))
			{
				dma_addr = nm_read_reg(0x150400);
				ret = M2M_SUCCESS;
				break;
			} 
		}
		nm_bsp_interrupt_ctrl(1);

		if (dma_addr != 0) 
		{
			uint32	u32CurrAddr;
			u32CurrAddr = dma_addr;
			strHif.u16Length=NM_BSP_B_L_16(strHif.u16Length);
			nm_write_block(u32CurrAddr, (uint8*)&strHif, M2M_HIF_HDR_OFFSET);
			u32CurrAddr += M2M_HIF_HDR_OFFSET;
			if(pu8CtrlBuf != NULL)
			{
				nm_write_block(u32CurrAddr, pu8CtrlBuf, u16CtrlBufSize);
				u32CurrAddr += u16CtrlBufSize;
			}
			if(pu8DataBuf != NULL)
			{
				u32CurrAddr += (u16DataOffset - u16CtrlBufSize);
				nm_write_block(u32CurrAddr, pu8DataBuf, u16DataSize);
				u32CurrAddr += u16DataSize;
			}
			if(u8PadSize)
			{
				uint8	au8PadBuf[4];
				m2m_memset(au8PadBuf, 0, sizeof(au8PadBuf));
				nm_write_block(u32CurrAddr, au8PadBuf, u8PadSize);
			}
			reg = dma_addr << 2;
			reg |= (1 << 1);
			nm_write_reg(WIFI_HOST_RCV_CTRL_3, reg);
			ret =  M2M_SUCCESS;
		}
		else
		{
			ret =  M2M_ERR_MEM_ALLOC;
		}

	}
#ifndef WIN32
	if(gu8ChipMode == M2M_PS_DEEP_AUTOMATIC)
	{
		hif_chip_sleep();
	}
#endif
	return ret;
}
/**
*	@fn		hif_isr
*	@brief	Host interface interrupt serviece routine
*	@author	M. Abdelmawla
*	@date	15 July 2012
*	@return	1 in case of interrupt received else 0 will be returned
*	@version	1.0
*/ 
static sint8 hif_isr(void)
{
	sint8 ret = M2M_SUCCESS;
	uint32 reg;

	tstrHifHdr strHif;

	/* validate if the bus is up or not by checking the chip id  */
	if((nm_read_reg(0x1000) & 0xfffff000) !=  0x100000) goto ERR1;


	ret = nm_read_reg_with_ret(0x1070, &reg);

	if(M2M_SUCCESS == ret)
	{
		if(reg & 0x1)	/* New interrupt has been received */
		{
			uint16 size;

			nm_bsp_interrupt_ctrl(0);

			/* read the rx size */	
			ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0, &reg);
			if(M2M_SUCCESS != ret)
			{
				M2M_ERR("(hif) WIFI_HOST_RCV_CTRL_0 bus fail\n");
				nm_bsp_interrupt_ctrl(1);
				goto ERR1;
			}
			gu8HifSizeDone = 0;
			size = (uint16)((reg >> 2) & 0xfff);	
			if (size > 0) {
				uint32 address = 0;
				/**
				start bus transfer
				**/
				ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_1, &address);
				if(M2M_SUCCESS != ret)
				{
					M2M_ERR("(hif) WIFI_HOST_RCV_CTRL_1 bus fail\n");
					nm_bsp_interrupt_ctrl(1);
					goto ERR1;
				}	
				ret = nm_read_block(address, (uint8*)&strHif, sizeof(tstrHifHdr));
				strHif.u16Length = NM_BSP_B_L_16(strHif.u16Length);
				if(M2M_SUCCESS != ret)
				{
					M2M_ERR("(hif) address bus fail\n");
					nm_bsp_interrupt_ctrl(1);
					goto ERR1;
				}
				if(strHif.u16Length != size)
				{
					if((size - strHif.u16Length) > 4)
					{
						M2M_ERR("(hif) Corrupted packet Size = %u <L = %u, G = %u, OP = %02X>\n",
							size, strHif.u16Length, strHif.u8Gid, strHif.u8Opcode);
						nm_bsp_interrupt_ctrl(1);
						ret = M2M_ERR_BUS_FAIL;
						goto ERR1;
					}
				}

				if(M2M_REQ_GRP_WIFI == strHif.u8Gid)
				{
					if(pfWifiCb)
						pfWifiCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET, strHif.u8Gid);
					
				} 
				else if(M2M_REQ_GRP_IP == strHif.u8Gid)
				{
					if(pfIpCb) 
						pfIpCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET, strHif.u8Gid);
				}
				else
				{
					M2M_ERR("(hif) invalid group ID\n");
				}
				#ifndef ENABLE_UNO_BOARD
				if(!gu8HifSizeDone)
				{
					hif_set_rx_done();
				}
				#endif
			}
			else
			{
				ret = M2M_ERR_BUS_FAIL;
				hif_set_rx_done();
			}
		}
	}
	else
	{
		M2M_ERR("(hif) bus error\n");
	}

ERR1:

	return ret;
}

/**
*	@fn		hif_handle_isr(void)
*	@brief	Handle interrupt received from NMC1500 firmware.
*   @author		M.S.M
*   @date		27 MARCH 2013
*	@version	1.0
*/
sint8 hif_handle_isr(void)
{
	sint8 ret = M2M_SUCCESS;
	
	while (gu8Interrupt) {
		ret = hif_isr(); 
		if(ret != M2M_SUCCESS) {
			break;
		}		
		gu8Interrupt--;
	}
	return ret;
}

/*
*	@fn		hif_receive
*	@brief	Host interface interrupt serviece routine
*	@param [in]	u32Addr
*				Receive start address
*	@param [out]	pu8Buf
*				Pointer to receive buffer. Allocated by the caller
*	@param [in]	u16Sz
*				Receive buffer size
*	@return	M2M_SUCCESS in case of success and -ve error code in case of failure
*	@author	M. Abdelmawla
*	@date	15 July 2012
*	@return	1 in case of interrupt received else 0 will be returned
*	@version	1.0
*/ 
sint8 hif_receive(uint32 u32Addr, uint8 *pu8Buf, uint16 u16Sz)
{
	uint32 address, reg;
	uint16 size;

	reg = nm_read_reg(WIFI_HOST_RCV_CTRL_0);	

	if(reg & 0x1)
	{
		size = (uint16)((reg >> 2) & 0xfff);	
		address = nm_read_reg(WIFI_HOST_RCV_CTRL_1);

		/* Receive the payload */
		nm_read_block(u32Addr, pu8Buf, u16Sz);

		/* check if this is the last packet */
		if(((address+size) - (u32Addr+u16Sz)) < 4)
		{
			gu8HifSizeDone = 1;
			/* set RX done */
			hif_set_rx_done();
		}
	}
	else
	{
		M2M_ERR("receive called without RX flag\n");
		return M2M_ERR_RCV;
	}

	return M2M_SUCCESS;
}

/**
*	@fn		m2m_set_WifiCb(tpfSetWiFiCallBack fn)
*	@brief	To set Call back function for WiFi Component
*	@param [in]	fn
*				function which you want to set
*   @author		Awad A. Bekhet
*   @date		23 June 2014
*	@version	1.0
*/
void hif_register_wifi_cb(tpfSetWiFiCallBack fn)
{
	pfWifiCb = fn;
}

/**
*	@fn		m2m_set_IpCb(tpfSetIPCallBack fn)
*	@brief	To set Call back function for Socket Component
*	@param [in]	fn
*				function which you want to set
*   @author		Awad A. Bekhet
*   @date		23 June 2014
*	@version	1.0
*/
void hif_register_ip_cb(tpfSetIPCallBack fn)
{
	pfIpCb = fn;
}

#endif
