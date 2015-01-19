/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus APIs implementation.
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

#include "nmbus.h"
#include "nmi2c.h"
#include "nmspi.h"
#include "nmuart.h"

#define MAX_TRX_CFG_SZ		8

/**
*	@fn		nm_bus_iface_init
*	@brief	Initialize bus interface
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/ 
sint8 nm_bus_iface_init(void)
{
	sint8 ret = M2M_SUCCESS;
	ret = nm_bus_init();


	
	return ret;
}

/**
*	@fn		nm_bus_iface_deinit
*	@brief	Deinitialize bus interface
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	Samer Sarhan
*	@date	07 April 2014
*	@version	1.0
*/ 
sint8 nm_bus_iface_deinit(void)
{
	sint8 ret = M2M_SUCCESS;

	nm_bus_deinit();
	
	return ret;
}


/*
*	@fn		nm_read_reg
*	@brief	Read register
*	@param [in]	u32Addr
*				Register address
*	@return	Register value
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/ 
uint32 nm_read_reg(uint32 u32Addr)
{
#ifdef USE_UART
	return nm_uart_read_reg(u32Addr);
#elif defined (USE_SPI)
	return nm_spi_read_reg(u32Addr);
#elif defined (USE_I2C)
	return nm_i2c_read_reg(u32Addr);
#else
#error "Plesae define bus usage"
#endif

}

/*
*	@fn		nm_read_reg_with_ret
*	@brief	Read register with error code return
*	@param [in]	u32Addr
*				Register address
*	@param [out]	pu32RetVal
*				Pointer to u32 variable used to return the read value
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/ 
sint8 nm_read_reg_with_ret(uint32 u32Addr, uint32* pu32RetVal)
{
#ifdef USE_UART
	return nm_uart_read_reg_with_ret(u32Addr,pu32RetVal);
#elif defined (USE_SPI)
	return nm_spi_read_reg_with_ret(u32Addr,pu32RetVal);
#elif defined (USE_I2C)
	return nm_i2c_read_reg_with_ret(u32Addr,pu32RetVal);
#else
#error "Plesae define bus usage"
#endif
}

/*
*	@fn		nm_write_reg
*	@brief	write register
*	@param [in]	u32Addr
*				Register address
*	@param [in]	u32Val
*				Value to be written to the register
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/ 
sint8 nm_write_reg(uint32 u32Addr, uint32 u32Val)
{
#ifdef USE_UART
	return nm_uart_write_reg(u32Addr,u32Val);
#elif defined (USE_SPI)
	return nm_spi_write_reg(u32Addr,u32Val);
#elif defined (USE_I2C)
	return nm_i2c_write_reg(u32Addr,u32Val);
#else
#error "Plesae define bus usage"
#endif
}

static sint8 p_nm_read_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz)
{
	#ifdef USE_UART
	return nm_uart_read_block(u32Addr,puBuf,u16Sz);
#elif defined (USE_SPI)
	return nm_spi_read_block(u32Addr,puBuf,u16Sz);
#elif defined (USE_I2C)
	return nm_i2c_read_block(u32Addr,puBuf,u16Sz);
#else
#error "Plesae define bus usage"
#endif

}
/*
*	@fn		nm_read_block
*	@brief	Read block of data
*	@param [in]	u32Addr
*				Start address
*	@param [out]	puBuf
*				Pointer to a buffer used to return the read data
*	@param [in]	u16Sz
*				Number of bytes to read. The buffer size must be >= u16Sz
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/ 
sint8 nm_read_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz)
{
	uint16 u16MaxTrxSz = egstrNmBusCapabilities.u16MaxTrxSz - MAX_TRX_CFG_SZ;
	uint16 off = 0;
	sint8 s8Ret = M2M_SUCCESS;

	for(;;)
	{
		if(u16Sz <= u16MaxTrxSz)
		{
			s8Ret += p_nm_read_block(u32Addr, &puBuf[off], u16Sz);	
			break;
		}
		else
		{
			s8Ret += p_nm_read_block(u32Addr, &puBuf[off], u16MaxTrxSz);
			u16Sz -= u16MaxTrxSz;
			off += u16MaxTrxSz;
			u32Addr += u16MaxTrxSz;
		}
	}

	return s8Ret;
}

static sint8 p_nm_write_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz)
{
	#ifdef USE_UART
	return nm_uart_write_block(u32Addr,puBuf,u16Sz);
#elif defined (USE_SPI)
	return nm_spi_write_block(u32Addr,puBuf,u16Sz);
#elif defined (USE_I2C)
	return nm_i2c_write_block(u32Addr,puBuf,u16Sz);
#else
#error "Plesae define bus usage"
#endif

}
/**
*	@fn		nm_write_block
*	@brief	Write block of data
*	@param [in]	u32Addr
*				Start address
*	@param [in]	puBuf
*				Pointer to the buffer holding the data to be written
*	@param [in]	u16Sz
*				Number of bytes to write. The buffer size must be >= u16Sz
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/ 
sint8 nm_write_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz)
{
	uint16 u16MaxTrxSz = egstrNmBusCapabilities.u16MaxTrxSz - MAX_TRX_CFG_SZ;
	uint16 off = 0;
	sint8 s8Ret = M2M_SUCCESS;

	for(;;)
	{
		if(u16Sz <= u16MaxTrxSz)
		{
			s8Ret += p_nm_write_block(u32Addr, &puBuf[off], u16Sz);	
			break;
		}
		else
		{
			s8Ret += p_nm_write_block(u32Addr, &puBuf[off], u16MaxTrxSz);
			u16Sz -= u16MaxTrxSz;
			off += u16MaxTrxSz;
			u32Addr += u16MaxTrxSz;
		}
	}

	return s8Ret;
}


static uint32 clk_status_reg_adr = 0xf; /* Assume initially it is B0 chip */

/**
*	@fn		nm_clkless_wake
*	@brief	Wakeup the chip using clockless registers
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	Samer Sarhan
*	@date	06 June 2014
*	@version	1.0
*/
sint8 nm_clkless_wake(void)
{
	sint8 ret = M2M_SUCCESS;
	uint32 reg, clk_status_reg,trials = 0;
	ret = nm_read_reg_with_ret(0x1, &reg);
	if(ret != M2M_SUCCESS) {
		M2M_ERR("Bus error (1). Wake up failed\n");
		goto _WAKE_EXIT;
	}
	/*
	 * At this point, I am not sure whether it is B0 or A0
	 * If B0, then clks_enabled bit exists in register 0xf
	 * If A0, then clks_enabled bit exists in register 0xe
	 */
	do
	{
		/* Set bit 1 */
		nm_write_reg(0x1, reg | (1 << 1));

		// Check the clock status
		ret = nm_read_reg_with_ret(clk_status_reg_adr, &clk_status_reg);
		if( (ret != M2M_SUCCESS) || ((ret == M2M_SUCCESS) && (clk_status_reg == 0)) ) {
			/* Register 0xf did not exist in A0.
			 * If register 0xf fails to read or if it reads 0,
			 * then the chip is A0.
			 */
			clk_status_reg_adr = 0xe;
			ret = nm_read_reg_with_ret(clk_status_reg_adr, &clk_status_reg);
			if(ret != M2M_SUCCESS) {
				M2M_ERR("Bus error (2). Wake up failed\n");
				goto _WAKE_EXIT;
			}
		}

		// in case of clocks off, wait 2ms, and check it again.
		// if still off, wait for another 2ms, for a total wait of 6ms.
		// If still off, redo the wake up sequence
		while( ((clk_status_reg & 0x4) == 0) && (((++trials) %3) == 0))
		{
			/* Wait for the chip to stabilize*/
			nm_bsp_sleep(1);

			// Make sure chip is awake. This is an extra step that can be removed
			// later to avoid the bus access overhead
			nm_read_reg_with_ret(clk_status_reg_adr, &clk_status_reg);

			if((clk_status_reg & 0x4) == 0)
			{
				M2M_ERR("clocks still OFF. Wake up failed\n");
			}
		}
		// in case of failure, Reset the wakeup bit to introduce a new edge on the next loop
		if((clk_status_reg & 0x4) == 0)
		{
			// Reset bit 0
			nm_write_reg(0x1, reg | (1 << 1));
		}
	} while((clk_status_reg & 0x4) == 0);

_WAKE_EXIT:
	return ret;
}
