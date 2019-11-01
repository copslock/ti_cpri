/*
 * RTWP_evm.c
 *
 * This file is used to read RTWP measurements in the SmallCell WCDMA stack.
 *
 *   Support for TCI6614 EVM + SCBP/CPRI-relay.
 *   Support for TCI6636K2H EVM + SCBP/CPRI-relay.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without      
 *  modification, are permitted provided that the following conditions      
 *  are met:                                                                
 *                                                                          
 *    Redistributions of source code must retain the above copyright        
 *    notice, this list of conditions and the following disclaimer.         
 *                                                                          
 *    Redistributions in binary form must reproduce the above copyright     
 *    notice, this list of conditions and the following disclaimer in the   
 *    documentation and/or other materials provided with the                
 *    distribution.                                                         
 *                                                                          
 *    Neither the name of Texas Instruments Incorporated nor the names of   
 *    its contributors may be used to endorse or promote products derived   
 *    from this software without specific prior written permission.         
 *                                                                          
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    
 *
 */
#include <RTWP.h>
#include <c6x.h>

/* Uncomment the following when the SCBP/CPRI-relay is using a FPGA firmware
   that supports Dual-Carrier.  */
// #define CONFIG_SANITY_CHECKS
// #define CONFIG_ENABLE_RTWP_IN_FPGA
#define CONFIG_SUPPORT_WCDMA_DUAL_CARRIER 1
#define RTWP_DEBUG_INFO 1

/* These functions, found in scbpfpga_spi.c, are used here to access the SCBP
   FPGA registers.  */
extern int spi_scbp_fpga_write(uint32_t reg, uint32_t data);
extern int spi_scbp_fpga_read(uint32_t reg, uint32_t *data);
extern int spi_scbp_fpga_lock(void);
extern int spi_scbp_fpga_unlock(void);

/* The RTWP measurement support in the SCBP FPGA is configured to produce
   floating point results.  */
#define RTWP_FPGA_FLOATING_POINT (1)
#define RTWP_FPGA_IQ_COUNT       (13)
#define RTWP_FPGA_CFG_MASK       ((RTWP_FPGA_FLOATING_POINT<<8)|(RTWP_FPGA_IQ_COUNT))

typedef struct {
	int opened;
	float rf_rx_gain;
	int closed;
	uint32_t fpga_revision;
	uint32_t calls;
	uint32_t calls_in[4];
	uint32_t calls_ok[4];
	int      index[4];
	float    measurement[4][32];
} RTWP_info;

/* This rtwp_info structure is used to keep information about the RTWP API state.
   It is used in various places in the API implementation in particular to ensure
   that the sequence of calls is done in the proper order.  */
RTWP_info rtwp_info = {  /* For debugging purpose this structure is not made static.  */
	0,      /* opened  */
	0.0,    /* rf_rx_gain  */
	0,      /* closed  */
	0xffff, /* fpga_revision  */
	0,      /* calls */
	{0, 0, 0, 0},     /* calls_in */
	{0, 0, 0, 0},     /* calls_ok */
	{0, 0, 0, 0}      /* index */
};

uint32_t RTWP_open(uint32_t cpri_link_nbr, float rf_rx_gain1, float rf_rx_gain2)
{
	int status;
	int i;
	/* for EVM + SCBP, only one CPRI link is used, and gain1 is assumed to be equal to gain2 */
	float rf_rx_gain = rf_rx_gain1;
	
	/* IMPORTANT NOTE: The assumption made in this code is that the
	   SPI module has been properly initialized before calling this function.  */

	/* The first time this API is invoked we do a full FPGA initialization.  */
	if (rtwp_info.opened && !rtwp_info.closed) {
		/* The module has been opened before, but has not been closed yet.  */
		return 1;
	}
	if (rtwp_info.opened && rtwp_info.closed) {
		/* No need for another initialization as it has already been done.  */
		rtwp_info.rf_rx_gain = rf_rx_gain;
		rtwp_info.closed = 0;
	} else {
		status = 0;
		/* Read the SCBP FPGA firmware revision from register 0x0000.  */
		if (spi_scbp_fpga_lock() < 0) {
			/* Failed to lock.  */
			return 2;
		}
		status |= spi_scbp_fpga_read(0, &(rtwp_info.fpga_revision));
		if (spi_scbp_fpga_unlock() < 0 || status) {
			/* Failed to unlock, or read the fpga_revision register.  */
			return 3;
		}
#ifdef CONFIG_SANITY_CHECKS
		if (!rtwp_info.fpga_revision || rtwp_info.fpga_revision & 0x0000ff00) {
			/* No FPGA firmware revision could be read, or it is not a WCDMA firmware.  */
			return 4;
		}
#endif

#ifdef CONFIG_ENABLE_RTWP_IN_FPGA
		status = 0;
		/* Setup the SCBP FPGA for RTWP measurements on all supported antennas.  */
		if (spi_scbp_fpga_lock() < 0) {
			/* Failed to lock.  */
			return 5;
		}
		/* Configure RTWP for AxC #0 and AxC #1.  */
		status |= spi_scbp_fpga_write(0x002f, RTWP_FPGA_CFG_MASK);
		status |= spi_scbp_fpga_write(0x102f, RTWP_FPGA_CFG_MASK);
		/* Additional configuration required when the Dual-Carrier
           support is available.  */
#ifdef CONFIG_SUPPORT_WCDMA_DUAL_CARRIER
		/* Configure RTWP for AxC #2 and AxC #3.  */
		status |= spi_scbp_fpga_write(0x202f, RTWP_FPGA_CFG_MASK);
		status |= spi_scbp_fpga_write(0x302f, RTWP_FPGA_CFG_MASK);
#endif
		if (spi_scbp_fpga_unlock() < 0 || status) {
			/* Failed to unlock or write the configuration registers properly.  */
			return 6;
		}
#endif
		rtwp_info.opened = 1;
		rtwp_info.rf_rx_gain = rf_rx_gain;
	}
#ifdef RTWP_DEBUG_INFO
	/* Initialize debug info.  */
	rtwp_info.calls = 0;
	rtwp_info.index[0] = 0;
	rtwp_info.index[1] = 0;
	rtwp_info.index[2] = 0;
	rtwp_info.index[3] = 0;
	rtwp_info.calls_in[0] = 0;
	rtwp_info.calls_in[1] = 0;
	rtwp_info.calls_in[2] = 0;
	rtwp_info.calls_in[3] = 0;
	rtwp_info.calls_ok[0] = 0;
	rtwp_info.calls_ok[1] = 0;
	rtwp_info.calls_ok[2] = 0;
	rtwp_info.calls_ok[3] = 0;
	for (i=0; i<32; i++) {
		rtwp_info.measurement[0][i] = 0.0;
		rtwp_info.measurement[1][i] = 0.0;
		rtwp_info.measurement[2][i] = 0.0;
		rtwp_info.measurement[3][i] = 0.0;
	}
#endif
	return 0;
}

uint32_t RTWP_read( uint32_t link_nbr_rtwp_address, float *result_mW, float *result_raw)
{
	uint32_t high_address, low_address;
	uint32_t high_value, low_value;
	float value_raw;
	float value_mW;
	int status = 0;
	uint32_t rtwp_address;
#ifdef RTWP_DEBUG_INFO
	int antenna;
	
	rtwp_info.calls++;
#endif
	/* This is a trick to rebuild the expected FPGA address because the 4 MSB are now used for the CPRI lik number*/
	rtwp_address = (link_nbr_rtwp_address & 0x00ffffff);
	rtwp_address = (rtwp_address | (((link_nbr_rtwp_address >> 8) & 0x000000ff) << 24));

#ifdef CONFIG_SANITY_CHECKS
	/* Sanity check of the address to ensure that invalid address is used to read
	   from the FPGA through this API.  */
	if (rtwp_address != 0x00310030 && rtwp_address != 0x10311030
	 && rtwp_address != 0x20312030 && rtwp_address != 0x30313030) {
		/* This is not one of the valid set of SCBP FPGA addresses to read from.  */
		return 1;
	}
#endif
	if (!rtwp_info.opened || !(rtwp_info.opened && !rtwp_info.closed)) {
		/* Can't read as the the RTWP module is either not opened, or
		   already closed.  */
		return 2;
	}
#ifdef RTWP_DEBUG_INFO
	switch (rtwp_address) {
	case 0x00310030:
		antenna = 0;
		break;
	case 0x10311030:
		antenna = 1;
		break;
	case 0x20312030:
		antenna = 2;
		break;
	case 0x30313030:
		antenna = 3;
		break;
	default:
		antenna = 0;
	}
	rtwp_info.calls_in[antenna]++;
#endif
	/* Read the raw floating point value of the RTWP from the low and high 16-bit
	   registers in the FPGA of the SCBP.  */
	high_address = (rtwp_address & 0xffff0000) >> 16;
	low_address  = (rtwp_address & 0x0000ffff);
	if (spi_scbp_fpga_lock() < 0) {
		/* Failed to lock.  */
		return 3;
	}
	/* Be careful with the following two lines as the behavior of the
	   FPGA depends on the order in which these two registers are read.  */
	status |= spi_scbp_fpga_read(low_address, &low_value);
	status |= spi_scbp_fpga_read(high_address, &high_value);
	if (spi_scbp_fpga_unlock() < 0 || status) {
		/* Failed to unlock, or failed read values from the FPGA.  */
		return 4;
	}

	/* Combine the two values in one, and convert to float.  */
	value_raw = _itof(_pack2(high_value, low_value));
	value_mW = value_raw * rtwp_info.rf_rx_gain;
	
	/* Write the results to the addresses provided as parameters to this API.  */
	*result_mW = value_mW;
	*result_raw = value_raw;

#ifdef RTWP_DEBUG_INFO
	/* Record information for debug and observation.  */
	rtwp_info.calls_ok[antenna]++;
	rtwp_info.measurement[antenna][rtwp_info.index[antenna]++] = value_raw;
	if (rtwp_info.index[antenna] >= 32) { rtwp_info.index[antenna] = 0; }
#endif
	return 0;
}

uint32_t RTWP_close() {
	/* This is just code to insure that the sequence of API calls is executed in
	   the intended order.  */
	if (!rtwp_info.opened || !(rtwp_info.opened && !rtwp_info.closed)) {
		/* Can't close as it is either not opened yet, or already closed.  */
		return 1;
	}
	rtwp_info.closed = 1;
	return 0;
}
