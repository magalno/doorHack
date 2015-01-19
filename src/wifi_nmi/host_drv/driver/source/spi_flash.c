/**
 *
 * \file
 *
 * \brief NMC1500 stack SPI Flash internal APIs implementation.
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

#define TIMEOUT 10000 /*MS*/
#define SPI_FLASH_BASE		(0x10200)
#define SPI_FLASH_MODE		(SPI_FLASH_BASE + 0x00)
#define SPI_FLASH_CMD_CNT	(SPI_FLASH_BASE + 0x04)
#define SPI_FLASH_DATA_CNT	(SPI_FLASH_BASE + 0x08)
#define SPI_FLASH_BUF1		(SPI_FLASH_BASE + 0x0c)
#define SPI_FLASH_BUF2		(SPI_FLASH_BASE + 0x10)
#define SPI_FLASH_BUF_DIR	(SPI_FLASH_BASE + 0x14)
#define SPI_FLASH_TR_DONE	(SPI_FLASH_BASE + 0x18)
#define SPI_FLASH_DMA_ADDR	(SPI_FLASH_BASE + 0x1c)
#define SPI_FLASH_MSB_CTL	(SPI_FLASH_BASE + 0x20)
#define SPI_FLASH_TX_CTL	(SPI_FLASH_BASE + 0x24)

#define DUMMY_REGISTER	(0x1084)

static uint32 spi_flash_rdid(void)
{
	unsigned char cmd[1];
	uint32 reg;

	cmd[0] = 0x9f;

	nm_write_reg(SPI_FLASH_DATA_CNT, 4);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x1);
	nm_write_reg(SPI_FLASH_DMA_ADDR, DUMMY_REGISTER);
	nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	while(nm_read_reg(SPI_FLASH_TR_DONE) != 1);
	reg = nm_read_reg(DUMMY_REGISTER);
	M2M_PRINT("Flash id %x \n",reg);
	return reg;
}

uint32 probe_spi_flash(void);

uint32 probe_spi_flash(void) 
{
	uint32 pin_mux_0;
	uint32 orig_pin_mux_0;
	uint32 flashid;

	pin_mux_0 = nm_read_reg(0x1408);
	orig_pin_mux_0 = pin_mux_0;

	if( (orig_pin_mux_0 & 0xffff000) != 0x1111000) {
		/* Select PINMUX to use SPI MASTER */
		pin_mux_0 &= ~0xffff000;
		pin_mux_0 |= 0x1111000;
		nm_write_reg(0x1408, pin_mux_0);
	}

	flashid = spi_flash_rdid();

	if( (orig_pin_mux_0 & 0xffff000) != 0x1111000) {
		nm_write_reg(0x1408, orig_pin_mux_0);
	}

	return flashid;
}