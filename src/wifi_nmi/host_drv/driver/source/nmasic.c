/**
 *
 * \file
 *
 * \brief This module contains NMC1500 ASIC specific internal APIs.
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
#include "driver\source\nmasic.h"

#define NMI_GLB_RESET_0 (NMI_PERIPH_REG_BASE + 0x400)
#define NMI_INTR_REG_BASE (NMI_PERIPH_REG_BASE+0xa00)
#define NMI_PIN_MUX_0 (NMI_PERIPH_REG_BASE + 0x408)
#define NMI_INTR_ENABLE (NMI_INTR_REG_BASE)
#define GET_UINT32(X,Y) (X[0+Y] + ((uint32)X[1+Y]<<8) + ((uint32)X[2+Y]<<16) +((uint32)X[3+Y]<<24))


#define M2M_FINISH_INIT_STATE 0x02532636UL
#define M2M_FINISH_BOOT_ROM   0x10add09eUL
#define M2M_START_FIRMWARE    0xef522f61UL
#define M2M_DISABLE_PS        0xD0UL

extern uint32 probe_spi_flash(void);

void chip_idle(void)
{
	uint32 reg =0;
	nm_read_reg_with_ret(0x1, &reg);
	if(reg&0x2)
	{
		reg &=~(1 << 1);
		nm_write_reg(0x1, reg);
	}
}

void enable_rf_blocks(void)
{
	nm_write_reg(0x6, 0xdb);
	nm_write_reg(0x7, 0x6);
	nm_bsp_sleep(10);
	nm_write_reg(0x1480, 0);
	nm_write_reg(0x1484, 0);
	nm_bsp_sleep(10);

	nm_write_reg(0x6, 0x0);
	nm_write_reg(0x7, 0x0);
}

sint8 enable_interrupts(void) 
{
	uint32 reg;
	sint8 ret;
	/**
	interrupt pin mux select
	**/
	ret = nm_read_reg_with_ret(NMI_PIN_MUX_0, &reg);
	if (M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
	}
	reg |= ((uint32) 1 << 8);
	ret = nm_write_reg(NMI_PIN_MUX_0, reg);
	if (M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
	}
	/**
	interrupt enable
	**/
	ret = nm_read_reg_with_ret(NMI_INTR_ENABLE, &reg);
	if (M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
	}
	reg |= ((uint32) 1 << 16);
	ret = nm_write_reg(NMI_INTR_ENABLE, reg);
	if (M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
	}
	return M2M_SUCCESS;
}

sint8 cpu_start(void) {
	uint32 reg;
	sint8 ret;

	/**
	reset regs 
	*/
	nm_write_reg(BOOTROM_REG,0);
	nm_write_reg(STATE_REG,0);
	nm_write_reg(NMI_REV_REG,0);
	
	/**
	Go...
	**/
	ret = nm_read_reg_with_ret(0x1118, &reg);
	if (M2M_SUCCESS != ret) {
		ret = M2M_ERR_BUS_FAIL;
		M2M_ERR("[nmi start]: fail read reg 0x1118 ...\n");
	}
	reg |= (1 << 0);
	ret = nm_write_reg(0x1118, reg);
	ret = nm_write_reg(0x150014, 0x1);
	ret += nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
	if ((reg & (1ul << 10)) == (1ul << 10)) {
		reg &= ~(1ul << 10);
		ret += nm_write_reg(NMI_GLB_RESET_0, reg);
	}

	reg |= (1ul << 10);
	ret += nm_write_reg(NMI_GLB_RESET_0, reg);
	nm_bsp_sleep(1); /* TODO: Why bus error if this delay is not here. */
	return ret;
}

uint32 nmi_get_chipid(void)
{
	static uint32 chipid = 0;

	if (chipid == 0) {
		//uint32 revid;
		uint32 rfrevid;

		if((nm_read_reg_with_ret(0x1000, &chipid)) != M2M_SUCCESS) {
			chipid = 0;
			return 0;
		}
		//if((ret = nm_read_reg_with_ret(0x11fc, &revid)) != M2M_SUCCESS) {
		//	return 0;
		//}
		if((nm_read_reg_with_ret(0x13f4, &rfrevid)) != M2M_SUCCESS) {
			chipid = 0;
			return 0;
		}

		if (chipid == 0x1002a0)  {
			if (rfrevid == 0x1) { /* 1002A0 */
			} else /* if (rfrevid == 0x2) */ { /* 1002A1 */
				chipid = 0x1002a1;
			}
		} else if(chipid == 0x1002b0) {
			if(rfrevid == 3) { /* 1002B0 */
			} else if(rfrevid == 4) { /* 1002B1 */
				chipid = 0x1002b1;
			} else /* if(rfrevid == 5) */ { /* 1002B2 */
				chipid = 0x1002b2;
			}
		} else {
		}
#define PROBE_FLASH
#ifdef PROBE_FLASH
		if(chipid) {
			uint32 flashid;

			flashid = probe_spi_flash();
			if(flashid == 0x1230ef) {
				chipid &= ~(0x0f0000);
				chipid |= 0x050000;
			}
		}
#else
		chipid &= ~(0x0f0000);
		chipid |= 0x050000;
#endif /* PROBE_FLASH */
	}
	return chipid;
}

void restore_pmu_settings_after_global_reset(void)
{
	/* 
	* Must restore PMU register value after 
	* global reset if PMU toggle is done at 
	* least once since the last hard reset.
	*/
	if(REV(nmi_get_chipid()) >= REV_B0) {
		nm_write_reg(0x1e48, 0xb78469ce);
	}
}

void nmi_update_pll(void)
{
	uint32 pll;

	pll = nm_read_reg(0x1428);
	pll &= ~0x1ul;
	nm_write_reg(0x1428, pll);
	pll |= 0x1ul;
	nm_write_reg(0x1428, pll);

}
void nmi_set_sys_clk_src_to_xo(void) 
{
	uint32 val32;
	
	/* Switch system clock source to XO. This will take effect after nmi_update_pll(). */
	val32 = nm_read_reg(0x141c);
	val32 |= (1 << 2);
	nm_write_reg(0x141c, val32);
	
	/* Do PLL update */
	nmi_update_pll();
}

sint8 chip_reset(void)
{
	sint8 ret = M2M_SUCCESS;

	nmi_set_sys_clk_src_to_xo();
	ret += nm_write_reg(NMI_GLB_RESET_0, 0);
	nm_bsp_sleep(50);

	restore_pmu_settings_after_global_reset();
	
	return ret;
}

sint8 wait_for_bootrom(void)
{
	sint8 ret = M2M_SUCCESS;
	uint32 reg = 0, cnt = 0;
	
	reg = 0;
	while(1) {
		reg = nm_read_reg(0x1014);	/* wait for efuse loading done */
		if (reg & 0x80000000) {
			break;
		}
		nm_bsp_sleep(1); /* TODO: Why bus error if this delay is not here. */
	}
	reg = nm_read_reg(M2M_WAIT_FOR_HOST_REG);
	reg &= 0x1;

	/* check if waiting for the host will be skipped or not */
	if(reg == 0)
	{
		reg = 0;
		while(reg != M2M_FINISH_BOOT_ROM)
		{
			nm_bsp_sleep(1);
			reg = nm_read_reg(BOOTROM_REG);

			if(++cnt > 1000)
			{
				M2M_DBG("failed to load firmware from flash.\n");
				ret = M2M_ERR_INIT;
				goto ERR2;
			}
		}
	}

	nm_write_reg(BOOTROM_REG,M2M_START_FIRMWARE);

ERR2:
	return ret;
}

sint8 wait_for_firmware_start(void)
{
	sint8 ret = M2M_SUCCESS;
	uint32 reg = 0, cnt = 0;;
	
	while (reg != M2M_FINISH_INIT_STATE)
	{
		nm_bsp_sleep(1); /* TODO: Why bus error if this delay is not here. */
		reg = nm_read_reg(STATE_REG);
		if(++cnt > 1000)
		{
			M2M_DBG("Time out for wait firmware Run\n");
			ret = M2M_ERR_INIT;
			goto ERR;
		}
	}
	nm_write_reg(STATE_REG,0);
ERR:
	return ret;
}

sint8 chip_deinit(void)
{
	uint32 reg = 0;
	sint8 ret;
	uint8 timeout = 10;

	/**
	stop the firmware, need a re-download
	**/
	ret = nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
	if (ret != M2M_SUCCESS) {
		M2M_ERR("failed to de-initialize\n");
	}
	reg &= ~(1 << 10);
	ret = nm_write_reg(NMI_GLB_RESET_0, reg);

	if (ret != M2M_SUCCESS) {
		M2M_ERR("Error while writing reg\n");
		return ret;
	}

	do {
		ret = nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
		if (ret != M2M_SUCCESS) {
			M2M_ERR("Error while reading reg\n");
			return ret;
		}
		/*Workaround to ensure that the chip is actually reset*/
		if ((reg & (1 << 10))) {
			M2M_DBG("Bit 10 not reset retry %d\n", timeout);
			reg &= ~(1 << 10);
			ret = nm_write_reg(NMI_GLB_RESET_0, reg);
			timeout--;
		} else {
			break;
		}

	} while (timeout);
		
}