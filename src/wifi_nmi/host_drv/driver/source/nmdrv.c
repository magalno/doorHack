/**
 *
 * \file
 *
 * \brief This module contains NMC1000 M2M driver APIs implementation.
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
#include "driver\source\nmdrv.h"
#include "driver\source\nmasic.h"

#ifdef USE_SPI
#include "driver\source\nmspi.h"
#endif

#define rNMI_GP_REG_1          (0x14a0)

#define rHAVE_SDIO_IRQ_GPIO_BIT     (NBIT0)
#define rHAVE_USE_PMU_BIT           (NBIT1)
#define rHAVE_SLEEP_CLK_SRC_RTC_BIT (NBIT2)
#define rHAVE_SLEEP_CLK_SRC_XO_BIT  (NBIT3)
#define rHAVE_EXT_PA_INV_TX_RX      (NBIT4)
#define rHAVE_LEGACY_RF_SETTINGS    (NBIT5)


static sint8 nm_get_firmware_info(tstrM2mRev* M2mRev);


static sint8 chip_wake(void)
{
	sint8 ret = M2M_SUCCESS;

	ret  = nm_clkless_wake();
	if(ret != M2M_SUCCESS) return ret;
	
	enable_rf_blocks();

	return ret;
}

static void chip_apply_conf(void)
{
	sint8 ret = M2M_SUCCESS;
	
	uint32 val32;
	val32 = 0;
#ifdef __ENABLE_PMU__
	val32 |= rHAVE_USE_PMU_BIT;
#endif
#ifdef __ENABLE_SLEEP_CLK_SRC_RTC__
	val32 |= rHAVE_SLEEP_CLK_SRC_RTC;
#elif defined __ENABLE_SLEEP_CLK_SRC_XO__
	val32 |= rHAVE_SLEEP_CLK_SRC_XO;
#endif
#ifdef __ENABLE_EXT_PA_INV_TX_RX__
	val32 |= rHAVE_EXT_PA_INV_TX_RX;
#endif
#ifdef __ENABLE_LEGACY_RF_SETTINGS__
	val32 |= rHAVE_LEGACY_RF_SETTINGS;
#endif
	do  {
		nm_write_reg(rNMI_GP_REG_1, val32);
		if(val32 != 0) {		
			uint32 reg = 0;
			ret = nm_read_reg_with_ret(rNMI_GP_REG_1, &reg);
			if(ret == M2M_SUCCESS) {
				if(reg == val32)
					break;
			}
		} else {
			break;
		}
	} while(1);
}

/*
*	@fn		nm_drv_init
*	@brief	Initialize NMC1000 driver
*	@return	M2M_SUCCESS in case of success and Negative error code in case of failure
*   @param [in]	arg
*				Generic argument
*	@author	M. Abdelmawla
*	@date	15 July 2012
*	@version	1.0
*/
sint8 nm_drv_init(void * arg)
{
	tstrM2mRev strtmp;
	sint8 ret = M2M_SUCCESS;
	uint32 reg = 0, cnt = 0;
	
	ret = nm_bus_iface_init();
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi start]: fail init bus\n");
		goto ERR1;
	}
	
	ret = chip_wake();
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi start]: fail chip_wakeup\n");
		goto ERR2;
	}
	/**
	Go...
	**/
	ret = chip_reset();
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}

#ifdef USE_SPI
	/* Must do this after global reset to set SPI data packet size. */
	nm_spi_init();
#endif
	/*return power save to default value*/
	chip_idle();
	M2M_INFO("Chip ID %x\n", nmi_get_chipid());

	ret = cpu_start();
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}
	ret = wait_for_bootrom();
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}
	ret = wait_for_firmware_start();
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}
	ret = enable_interrupts();
	if (M2M_SUCCESS != ret) {
		M2M_ERR("failed to enable interrupts..\n");
		goto ERR2;
	}
	
	chip_apply_conf();
	
	if(M2M_ERR_FW_VER_MISMATCH == nm_get_firmware_info(&strtmp))
	{
		ret = M2M_ERR_FW_VER_MISMATCH;
		M2M_ERR("Mismatch Firmawre Version\n");
		M2M_INFO("Firmware_info: REV_MAJOR = %d, REV_MINOR= %d\n",strtmp.u16Major, strtmp.u16Minor);
		M2M_INFO("Host_info:     REV_MAJOR = %d, REV_MINOR= %d\n",M2M_FIRMWARE_VERSION_MAJOR_NO,M2M_FIRMWARE_VERSION_MINOR_NO);
	}
	return ret;
ERR2:
	nm_bus_iface_deinit();
ERR1:	
	return ret;
}

/*
*	@fn		nm_drv_deinit
*	@brief	Deinitialize NMC1000 driver
*	@author	M. Abdelmawla
*	@date	17 July 2012
*	@version	1.0
*/
sint8 nm_drv_deinit(void * arg) 
{
	sint8 ret;
	
	ret = chip_deinit();
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi stop]: chip_deinit fail\n");
		goto ERR1;
	}

	ret = nm_bus_iface_deinit();
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi stop]: fail init bus\n");
		goto ERR1;
	}

ERR1:
	return ret;
}

/**
*	@fn		nm_get_firmware_info(tstrM2mRev* M2mRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*			    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
*   @author		Ahmad.Mohammad.Yahya
*   @date		27 MARCH 2013
*	@version	1.0
*/
static sint8 nm_get_firmware_info(tstrM2mRev* M2mRev)
{
	uint32	reg = 0;
	sint8	ret = M2M_SUCCESS;

	ret = nm_read_reg_with_ret(NMI_REV_REG, &reg);

	M2mRev->u16Major	= (uint16)(reg >> 16);
	M2mRev->u16Minor	= (uint16)(reg);
	M2mRev->u32Chipid	= nmi_get_chipid();
	if((M2mRev->u16Major != M2M_FIRMWARE_VERSION_MAJOR_NO) && (M2mRev->u16Minor != M2M_FIRMWARE_VERSION_MINOR_NO))
	{
		ret = M2M_ERR_FW_VER_MISMATCH;
	}
	return ret;
}
