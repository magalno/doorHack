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
#ifndef _NMASIC_H_
#define _NMASIC_H_

#include "common\include\nm_common.h"

#define NMI_PERIPH_REG_BASE 0x1000
#define NMI_CHIPID	(NMI_PERIPH_REG_BASE)

#define STATE_REG		(0x108c)
#define BOOTROM_REG		(0xc000c)
#define NMI_REV_REG  (0x207ac)
#define M2M_WAIT_FOR_HOST_REG (0x207bc)

#define REV_B0        (0x2B0)

#define ISNMC1000(id)   (((id & 0xfffff000) == 0x100000) ? 1 : 0)
#define ISNMC1500(id)   (((id & 0xfffff000) == 0x150000) ? 1 : 0)
#define REV(id)         ( ((id) & 0x00000fff ) )

/**
*  @struct		tstrM2mWifiGetRevision
*  @brief		Structure holding firmware version parameters
*  @author		Ahmad.Mohammad.Yahya
*  @date		26 MARCH 2013
*  @sa			M2M_WIFI_AUTH_WEB, M2M_WIFI_AUTH_WPA, M2M_WIFI_AUTH_WPA2
*  @version		1.0
*/
typedef struct {
	uint16 u16Major; /* Version Major Number which represents the official release base */
	uint16 u16Minor; /* Version Minor Number which represents the engineering release base */
	uint32 u32Chipid; /* HW revision which will be basically the chip ID */
} tstrM2mRev;

#ifdef __cplusplus
     extern "C" {
 #endif

void chip_idle(void);

void enable_rf_blocks(void);

sint8 enable_interrupts(void);

sint8 cpu_start(void);

uint32 nmi_get_chipid(void);

void restore_pmu_settings_after_global_reset(void);

void nmi_update_pll(void);

void nmi_set_sys_clk_src_to_xo(void);

sint8 chip_reset(void);

sint8 wait_for_bootrom(void);

sint8 wait_for_firmware_start(void);

sint8 chip_deinit(void);

#ifdef __cplusplus
	 }
 #endif

#endif	/*_NMASIC_H_*/
