#if EVM_TYPE == 5

/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/
#include <stdio.h>
#include "scbpfpga_spi.h"

#define CSNR_BITS_POS 16
//#define CSNR_FLASH  0xE
#define CSNR_6614EVM_FPGA 0xD
#define CSNR_DAC 0xB
#define CSNR_SCBP_FPGA 0x7

#define DFSEL_BITS_POS 24
//#define DFSEL_FLASH 0
#define DFSEL_6614EVM_FPGA 1
#define DFSEL_DAC 2
#define DFSEL_SCBP_FPGA 3

//For all data formats, WSEL = CSHOLD = 0
#define SPIDAT1_SCBP_FPGA_CTRL    ( (DFSEL_SCBP_FPGA     << DFSEL_BITS_POS) | (CSNR_SCBP_FPGA    << CSNR_BITS_POS) )
#define SPIDAT1_DAC_CTRL          ( (DFSEL_DAC           << DFSEL_BITS_POS) | (CSNR_DAC          << CSNR_BITS_POS) )
#define SPIDAT1_6614EVM_FPGA_CTRL ( (DFSEL_6614EVM_FPGA  << DFSEL_BITS_POS) | (CSNR_6614EVM_FPGA << CSNR_BITS_POS) )

#define SPIDAT1_TXDATA_MASK 0x0000FFFF

#define SPIDAT1_WRITE_SCBP_FPGA(x)     WR_SPI_REG(SPIDAT1, (SPIDAT1_SCBP_FPGA_CTRL    | ((x) & SPIDAT1_TXDATA_MASK)))
#define SPIDAT1_WRITE_6614EVM_FPGA(x)  WR_SPI_REG(SPIDAT1, (SPIDAT1_6614EVM_FPGA_CTRL | ((x) & SPIDAT1_TXDATA_MASK)))
#define SPIDAT1_WRITE_DAC(x)           WR_SPI_REG(SPIDAT1, (SPIDAT1_DAC_CTRL          | ((x) & SPIDAT1_TXDATA_MASK)))

void spi_init(void)
{
	unsigned int	value;

	/* 1) Reset the SPI CPIGCR0.RESET = 0
	 * 2) Take the SPI out of reset: CPIGCR0.RESET = 1
	 * 3) Configure SPI for master mode by configuraing the CLKMOD & MASTER bits in SPIGCR1
	 * 4) Configure SPI for 4-pin with chip select mode (SPIPC0)
	 * 5) Choose SPI data format register n (SPIFMTn) to be used (SPIDAT1.DFSEL)
	 * 6) Configure SPI data rate, char len, shift dir, phase, polarity, & other format (SPIFMTn)
	 * 7) In master mode, configure the master delay options usinf SPIDELAY
	 * 8) Select the error interrupt notifications by configuring the SPI interrupt reg (SPIINT0) & SPI interrupt level (SPILVL)
	 * 9) Enable SPI communication: SPIGCR1.ENABLE = 1
	 * 10) Setup & enable DMA for SPI data handling and enable DMA servicing 
	 * 11) Handle SPI data transfer requeset using DMA & service any SPI error conditions
	 */
	 
	WR_SPI_REG(SPIGCR0, 0);		// SPI is in reset state
	WR_SPI_REG(SPIGCR0, 1);		// SPI is out of reset state
	 
	WR_SPI_REG(SPIGCR1, (3 << 0));		// Master mode
	 
	// No interrupt (SPIINT0 & SPILVL)
	 
	// SOMIFUN, SIMOFUN, CLKFUN, SCSFUN[0-3] = 1111b = 15
	WR_SPI_REG(SPIPC0, (1 << 11) | (1 << 10) | (1 << 9) | (15 << 0));
	
	//SPI peripheral will run at 1228.8 MHz/6 max speed = 204.8 MHz,
	//prescalars for format registers based on this.

	//Format0 reserved for flash, not programmed currently, will have to change
	//SPIPC0, SPIDEF and if needed SPIDELAY when enabling this
	//WR_SPI_REG(SPIFMT0,...);

	//Format1 for the local FPGA on Appleton EVM, max freq is 40 MHz, prescale is 5,
	//actual division is 6 i.e 204.8/6 = 34 MHz
	//No delay, MSB first, POL=0, PHA=0, PRESCALE=5, data word = 16bit
	WR_SPI_REG(SPIFMT1, (0 << 17) | (0 << 16) | (5 << 8) | (16 << 0));

	//Format2 for the external DAC, DAC max clock is 1/(tch+tcl) = 1/60 ns = 16.66 MHz.
	//Run it at 10 MHz, So prescaler is set to 20, actual division 21 i.e 204.8/21 = 9.75 MHz
	// No delay, MSB first, POL=1, PHA=0, PRESCALE=20, data word = 12bit
	WR_SPI_REG(SPIFMT2, (1 << 17) | (0 << 16) | (20 << 8) | (12 << 0));

	//Format1 for the FPGA on SCBP, FPGA frequency is 16 Mhz, so ideal prescale will be 12,
	//actual division is 13, to give 204.8/13 = 15.75 MHz
	//but because SCBP connection is through cable, it is better to keep lower,
	//set prescalar to 15, actual division is 16.
	// No delay, MSB first, POL=0, PHA=1, PRESCALE=13, data word = 16bit
	WR_SPI_REG(SPIFMT3, (0 << 17) | (1 << 16) | (15 << 8) | (16 << 0));
	
	// All CS are active low, CSDEF[0-3] = 1111b = 15
	WR_SPI_REG(SPIDEF,(15 << 0));

	WR_SPI_REG(SPIDELAY, (7 << 24) | (14 << 16));
	
#if 1
	value = RD_SPI_REG(SPIGCR1);
	WR_SPI_REG(SPIGCR1, value | (1 << 24));		// SPI enable
#endif
}

int spi_scbp_fpga_lock(void)
{
    FPGADATA        arbiter;
    int				lock_cnt = 0;

    do
    {
        if (spi_scbp_fpga_read(0x7000, &arbiter) != 0)
        {
        	printf(">>> spi_scbp_fpga_read (lock) error\n");
        	return -1;
        }
        switch (arbiter)
        {
        case 0:
        	lock_cnt = 0;
            if (spi_scbp_fpga_write(0x7000, 0x2) != 0)
            {
            	printf(">>> spi_scbp_fpga_write (lock) error\n");
            	return -1;
            }
            break;

        case 1:
        	lock_cnt = 0;
        	break;

        case 2: // now NYQUIST is arbiter of fpga
            ++lock_cnt;
            if (lock_cnt < 2)
                break;
            return 0;

        default:
        	lock_cnt = 0;
        	printf(">>> fpga arbiter (lock) error = %x\n", arbiter);
        	break;
        }
    } while (1);
}

int spi_scbp_fpga_unlock(void)
{
    FPGADATA        arbiter;
    int				unlock_cnt = 0;

    do
    {
        if (spi_scbp_fpga_read(0x7000, &arbiter) != 0)
        {
        	printf(">>> spi_fpga_read (unlock) error\n");
            return -1;
        }
        switch (arbiter)
        {
        case 0:
        	if (++unlock_cnt < 2)
        		break;
            return 0;

        case 1:
        	unlock_cnt = 0;
        	break;

        case 2:
        	unlock_cnt = 0;
            if (spi_scbp_fpga_write(0x7000, 0x0) != 0)
            {
            	printf(">>> spi_scbp_fpga_write (unlock) error\n");
            	return -1;
            }
        	break;

        default:
        	unlock_cnt = 0;
        	printf(">>> fpga arbiter (unlock) error = %x\n", arbiter);
        	break;
        }
    } while (1);
}

static inline void clear_rx_conditions(void)
{
	volatile unsigned int value;

	value = RD_SPI_REG(SPIBUF);
	value = RD_SPI_REG(SPIFLG);
}

#define TIMED_OUT 1
#define NOT_TIMED_OUT 0
static inline int wait_until_tx_empty(void)
{
	int timeout;
	unsigned int value;

	timeout = FPGATIMEOUT;
	do
	{
		value = RD_SPI_REG(SPIBUF);
		if (!(value & SPIBUF_TXFULL))				// = 0: tx buffer empty
			break;
	} while (--timeout > 0);

	if (timeout == 0) {
		return(TIMED_OUT);
	}
	else {
		return(NOT_TIMED_OUT);
	}
}

static inline int wait_until_rx_full(unsigned int *value)
{
	int timeout;

	timeout = FPGATIMEOUT;
	do
	{
		*value = RD_SPI_REG(SPIBUF);
		if (!(*value & SPIBUF_RXEMPTY))				// = 0: rx buffer full
			break;
	} while (--timeout > 0);

	if (timeout == 0) {
		return(TIMED_OUT);
	}
	else {
		return(NOT_TIMED_OUT);
	}
}

int spi_scbp_fpga_write(FPGAREG reg, FPGADATA data)
{
	unsigned int	value;
	
#if 0
	value = RD_SPI_REG(SPIGCR1);
	WR_SPI_REG(SPIGCR1, value | SPIGCR1_ENABLE);		// SPI enable
#endif
	/* clear out any rx conditions */
	clear_rx_conditions();
	
	/* Send to FPGA the Write address */
    if (wait_until_tx_empty() == TIMED_OUT)
		return -1;		
	SPIDAT1_WRITE_SCBP_FPGA(reg | 0x8000);
	
	/* Read dummy value */
	if (wait_until_rx_full(&value) == TIMED_OUT)
		return -2;		

	/* Program the FPGA register with specified data */
	if (wait_until_tx_empty() == TIMED_OUT)
		return -3;		
	SPIDAT1_WRITE_SCBP_FPGA(data);

	/* Read dummy value */
	if (wait_until_rx_full(&value) == TIMED_OUT)
		return -4;

#if 0
	value = RD_SPI_REG(SPIGCR1);
	WR_SPI_REG(SPIGCR1, value & ~SPIGCR1_ENABLE);		// SPI disable
#endif

	return 0;
}

int spi_scbp_fpga_read(FPGAREG reg, FPGADATA *data)
{
	unsigned int	value;
	
#if 0
	value = RD_SPI_REG(SPIGCR1);
	WR_SPI_REG(SPIGCR1, value | SPIGCR1_ENABLE);		// SPI enable
#endif
//	value = RD_SPI(SPIGCR1);

	/* clear out any rx conditions */
	clear_rx_conditions();

	/*
	 * 1st phase: send to FPGA the register to read
	 */
    if (wait_until_tx_empty() == TIMED_OUT)
		return -1;
	SPIDAT1_WRITE_SCBP_FPGA(reg);

	/* Read dummy value */
	if (wait_until_rx_full(&value) == TIMED_OUT)
		return -2;		

	/*
	 * 2nd phase: generate a new CS to read the value from FPGA
	 */
    if (wait_until_tx_empty() == TIMED_OUT)
		return -3;
	SPIDAT1_WRITE_SCBP_FPGA(0);

	/* Read the value of the FPGA register */
	if (wait_until_rx_full(&value) == TIMED_OUT)
		return -4;
	*data = (value & 0x0000FFFF);

#if 0
	value = RD_SPI_REG(SPIGCR1);
	WR_SPI_REG(SPIGCR1, value & ~SPIGCR1_ENABLE);		// SPI disable
#endif
	return 0;
}

int spi_scbp_fpga_resync(FPGAREG reg)
{
#if 0
	unsigned int    value;
#endif

printf(">>> FPGA resync <<<\n");

#if 0
	value = RD_SPI_REG(SPIGCR1);
	WR_SPI_REG(SPIGCR1, value | SPIGCR1_ENABLE);		// SPI enable
#endif
	/* clear out any rx conditions */
	clear_rx_conditions();

    /*
     * Write the address register of fpga
     */
    if (wait_until_tx_empty() == TIMED_OUT)
		return -1;
	SPIDAT1_WRITE_SCBP_FPGA(reg);

    /*
     * check if the register value has been transmitted
     */
    if (wait_until_tx_empty() == TIMED_OUT)
		return -2;

#if 0
	value = RD_SPI_REG(SPIGCR1);
	WR_SPI_REG(SPIGCR1, value & ~SPIGCR1_ENABLE);		// SPI disable
#endif

    return 0;
}

static inline int spi_dac_write(int dacword)
{
	unsigned int value;

	clear_rx_conditions();

    if (wait_until_tx_empty() == TIMED_OUT)
		return -1;
	SPIDAT1_WRITE_DAC(dacword);

	/* Read dummy value */
	if (wait_until_rx_full(&value) == TIMED_OUT)
		return -2;

#if 1
	if (RD_SPI_REG(SPIBUF) & 0x10000000) {
		printf("error\n");
		while(1);
	}
#endif

	return 0;
}

int spi_6614evm_fpga_write(int reg, int data)
{
	unsigned int value;

	/* clear out any rx conditions */
	clear_rx_conditions();

	/* Send to FPGA the Write address */
    if (wait_until_tx_empty() == TIMED_OUT)
		return -1;
	SPIDAT1_WRITE_6614EVM_FPGA(((reg & 0x7F) << 8) | (data & 0x0FF));

	/* Read dummy value */
	if (wait_until_rx_full(&value) == TIMED_OUT)
		return -2;

	return 0;
}

int spi_6614evm_fpga_read(int reg, int *data)
{
	unsigned int value;

	/* clear out any rx conditions */
	clear_rx_conditions();

	/* Send to FPGA the Write address */
    if (wait_until_tx_empty() == TIMED_OUT)
		return -1;
	SPIDAT1_WRITE_6614EVM_FPGA(((reg | 0x80) << 8));

	/* Read the value of the FPGA register */
	if (wait_until_rx_full(&value) == TIMED_OUT)
		return -2;
	*data = (value & 0xFF);

	return 0;
}

static inline delay(void)
{
	/*volatile int i;

	//a litle delay (dac will be running at > 10 MHz, DSP CPU is 1 GHz)
	//to be safe since we don't have feedback
	for(i=0; i < 30; i++);*/
}

int spi_6614evm_dac_write_transaction(int data)
{
	int retval;
	volatile int i;

#if 0
	//spi_6614evm_fpga_write(0x51, 0x3);
	spi_6614evm_fpga_read(0x51, &data);
	printf("data = %x\n",data);
	while(1);
#endif

	//set LD high
	spi_6614evm_fpga_write(0x51, 0x3);

	delay();

	if ((retval = spi_dac_write(data)) != 0) {
		return (retval);
	}

	//need to wait until tx transmission done, before signaling Load
    if (wait_until_tx_empty() == TIMED_OUT)
		return (retval -1);

    delay();

	//set LD low
	spi_6614evm_fpga_write(0x51,0x1);

	delay();

	return(0);

}
#elif EVM_TYPE == 7
/*
 * scbpfpga_spi.c
 *
 * This file is used for SPI access on KeyStone-II EVM for small Cell 
 * platforms.
 * 
 * Copyright (C) 2012 Azcom Technology - http://www.azcom.it/
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *    Neither the name of Azcom Technology nor the names of
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
#include <stdio.h>
#include "scbpfpga_spi.h"

#define DAC_SPI_PORT  1
#define SCBP_FPGA_SPI_PORT 2

#define CSNR_BITS_POS 16
#define CSNR_DAC 0xE //DAC is hanging off of CS0 of SPI#1
#define CSNR_SCBP_FPGA 0x7  //SCBP FPGA is hanging off of CS3 of SPI#2

#define DFSEL_BITS_POS 24
#define DFSEL_MS_16bits_DAC 0  //format reg 0 is used for most significant 16-bits of 24-bit DAC write
#define DFSEL_LS_8bits_DAC 1   //format reg 1 is used for least significant 8-bits of 24-bit DAC write
#define DFSEL_SCBP_FPGA 0

#define CSHOLD_BITS_POS 28
#define CSHOLD_ENABLE 1
#define CSHOLD_DISABLE 0

//For all data formats, WSEL = CSHOLD = 0
#define SPIDAT1_SCBP_FPGA_CTRL     ( (DFSEL_SCBP_FPGA     << DFSEL_BITS_POS) | (CSNR_SCBP_FPGA << CSNR_BITS_POS) | (CSHOLD_DISABLE << CSHOLD_BITS_POS) )
#define SPIDAT1_MS_16bits_DAC_CTRL ( (DFSEL_MS_16bits_DAC << DFSEL_BITS_POS) | (CSNR_DAC       << CSNR_BITS_POS) | (CSHOLD_ENABLE  << CSHOLD_BITS_POS) )
#define SPIDAT1_LS_8bits_DAC_CTRL  ( (DFSEL_LS_8bits_DAC  << DFSEL_BITS_POS) | (CSNR_DAC       << CSNR_BITS_POS) | (CSHOLD_DISABLE << CSHOLD_BITS_POS) )

#define SPIDAT1_TXDATA_MASK 0x0000FFFF

#define SPIDAT1_WRITE_SCBP_FPGA(x)     WR_SPI_REG(SPI2_DAT1, (SPIDAT1_SCBP_FPGA_CTRL     | ((x) & SPIDAT1_TXDATA_MASK)))
#define SPIDAT1_WRITE_MS_16bits_DAC(x) WR_SPI_REG(SPI1_DAT1, (SPIDAT1_MS_16bits_DAC_CTRL | ((x) & SPIDAT1_TXDATA_MASK)))
#define SPIDAT1_WRITE_LS_8bits_DAC(x)  WR_SPI_REG(SPI1_DAT1, (SPIDAT1_MS_16bits_DAC_CTRL | ((x) & SPIDAT1_TXDATA_MASK)))

void spi_init_dac(void)
{
	unsigned int	value;

	/* 1) Reset the SPI CPIGCR0.RESET = 0
	 * 2) Take the SPI out of reset: CPIGCR0.RESET = 1
	 * 3) Configure SPI for master mode by configuraing the CLKMOD & MASTER bits in SPIGCR1
	 * 4) Configure SPI for 4-pin with chip select mode (SPIPC0)
	 * 5) Choose SPI data format register n (SPIFMTn) to be used (SPIDAT1.DFSEL)
	 * 6) Configure SPI data rate, char len, shift dir, phase, polarity, & other format (SPIFMTn)
	 * 7) In master mode, configure the master delay options usinf SPIDELAY
	 * 8) Select the error interrupt notifications by configuring the SPI interrupt reg (SPIINT0) & SPI interrupt level (SPILVL)
	 * 9) Enable SPI communication: SPIGCR1.ENABLE = 1
	 * 10) Setup & enable DMA for SPI data handling and enable DMA servicing 
	 * 11) Handle SPI data transfer requeset using DMA & service any SPI error conditions
	 */
	 
	WR_SPI_REG(SPI1_GCR0, 0);		// SPI is in reset state
	WR_SPI_REG(SPI1_GCR0, 1);		// SPI is out of reset state
	 
	WR_SPI_REG(SPI1_GCR1, (3 << 0));		// Master mode
	 
	// No interrupt (SPIINT0 & SPILVL)
	 
	// SOMIFUN, SIMOFUN, CLKFUN, SCSFUN[0-3] = 1111b = 15
	WR_SPI_REG(SPI1_PC0, (1 << 11) | (1 << 10) | (1 << 9) | (15 << 0));
	
	//SPI peripheral will run at 1228.8 MHz/6 max speed = 204.8 MHz,
	//prescalar based on this.
    //From DAC8550 spec:
	//  Data clocked in on rising edge => POL = 0, PHA = 0
    //  MSB first. 
	//  30 MHz max => 204.8/30 = 6.82 ~ 7, for safety keep 8 or 204.8/8 = 25.6 MHz -> prescalar = 8-1 = 7
    //24-bit input will require CSHold feature usage:
    //  API input will be 16-bit value [valid = [0,-26214]]
    //  2 write operations 1st 16-bit write has D23..D8, with all MSBs beyond D15 set to 0.
    //  First write will have CSHold = 1, 2nd write will have CSHold = 0.
    //  Since DAC is not shared with anything else on SPI#1, we are free to use all format registers
	//  We use two format registers for different lengths or write different values
	//No delay, MSB first, POL=0, PHA=0, PRESCALE=7, data word = 16bit for fmt0 and 24-16 bits for fmt1
	WR_SPI_REG(SPI1_FMT0, (0 << 17) | (0 << 16) | (7 << 8) | (16 << 0));
	WR_SPI_REG(SPI1_FMT1, (0 << 17) | (0 << 16) | (7 << 8) | ((24-16) << 0));
	
	// All CS are active low, CSDEF[0-3] = 1111b = 15
	WR_SPI_REG(SPI1_DEF,(15 << 0));
	
	WR_SPI_REG(SPI1_DELAY, (7 << 24) | (14 << 16));
	
	value = RD_SPI_REG(SPI1_GCR1);
	WR_SPI_REG(SPI1_GCR1, value | (1 << 24));		// SPI enable
}

void spi_init_scbp_fpga(void)
{
	unsigned int	value;

	/* 1) Reset the SPI CPIGCR0.RESET = 0
	 * 2) Take the SPI out of reset: CPIGCR0.RESET = 1
	 * 3) Configure SPI for master mode by configuraing the CLKMOD & MASTER bits in SPIGCR1
	 * 4) Configure SPI for 4-pin with chip select mode (SPIPC0)
	 * 5) Choose SPI data format register n (SPIFMTn) to be used (SPIDAT1.DFSEL)
	 * 6) Configure SPI data rate, char len, shift dir, phase, polarity, & other format (SPIFMTn)
	 * 7) In master mode, configure the master delay options usinf SPIDELAY
	 * 8) Select the error interrupt notifications by configuring the SPI interrupt reg (SPIINT0) & SPI interrupt level (SPILVL)
	 * 9) Enable SPI communication: SPIGCR1.ENABLE = 1
	 * 10) Setup & enable DMA for SPI data handling and enable DMA servicing 
	 * 11) Handle SPI data transfer requeset using DMA & service any SPI error conditions
	 */	 
	WR_SPI_REG(SPI2_GCR0, 0);		// SPI is in reset state
	WR_SPI_REG(SPI2_GCR0, 1);		// SPI is out of reset state
	 
	WR_SPI_REG(SPI2_GCR1, (3 << 0));		// Master mode
	 
	// No interrupt (SPIINT0 & SPILVL)
	 
	// SOMIFUN, SIMOFUN, CLKFUN, SCSFUN[0-3] = 1111b = 15
	WR_SPI_REG(SPI2_PC0, (1 << 11) | (1 << 10) | (1 << 9) | (15 << 0));
	
	//Format1 for the FPGA on SCBP, FPGA frequency is 16 Mhz, so ideal prescale will be 12,
	//actual division is 13, to give 204.8/13 = 15.75 MHz
	//but because SCBP connection is through cable, it is better to keep lower,
	//set prescalar to 15, actual division is 16.
	// No delay, MSB first, POL=0, PHA=1, PRESCALE=13, data word = 16bit
	WR_SPI_REG(SPI2_FMT0, (0 << 17) | (1 << 16) | (15 << 8) | (16 << 0));
	
	// All CS are active low, CSDEF[0-3] = 1111b = 15
	WR_SPI_REG(SPI2_DEF,(15 << 0));
	
	WR_SPI_REG(SPI2_DELAY, (7 << 24) | (14 << 16));
	
	value = RD_SPI_REG(SPI2_GCR1);
	WR_SPI_REG(SPI2_GCR1, value | (1 << 24));		// SPI enable
}

void spi_init(void)
{
    //spi_init_dac();
    spi_init_scbp_fpga();
}

int spi_scbp_fpga_lock(void)
{
    FPGADATA        arbiter;
    int				lock_cnt = 0;

    do
    {
        if (spi_scbp_fpga_read(0x7000, &arbiter) != 0)
        {
        	printf(">>> spi_scbp_fpga_read (lock) error\n");
        	return -1;
        }
        switch (arbiter)
        {
        case 0:
        	lock_cnt = 0;
            if (spi_scbp_fpga_write(0x7000, 0x2) != 0)
            {
            	printf(">>> spi_scbp_fpga_write (lock) error\n");
            	return -1;
            }
            break;

        case 1:
        	lock_cnt = 0;
        	break;

        case 2: // now NYQUIST is arbiter of fpga
            ++lock_cnt;
            if (lock_cnt < 2)
                break;
            return 0;

        default:
        	lock_cnt = 0;
        	printf(">>> fpga arbiter (lock) error = %x\n", arbiter);
        	break;
        }
    } while (1);
}

int spi_scbp_fpga_unlock(void)
{
    FPGADATA        arbiter;
    int				unlock_cnt = 0;

    do
    {
        if (spi_scbp_fpga_read(0x7000, &arbiter) != 0)
        {
        	printf(">>> spi_fpga_read (unlock) error\n");
            return -1;
        }
        switch (arbiter)
        {
        case 0:
        	if (++unlock_cnt < 2)
        		break;
            return 0;

        case 1:
        	unlock_cnt = 0;
        	break;

        case 2:
        	unlock_cnt = 0;
            if (spi_scbp_fpga_write(0x7000, 0x0) != 0)
            {
            	printf(">>> spi_scbp_fpga_write (unlock) error\n");
            	return -1;
            }
        	break;

        default:
        	unlock_cnt = 0;
        	printf(">>> fpga arbiter (unlock) error = %x\n", arbiter);
        	break;
        }
    } while (1);
}

static inline void clear_rx_conditions(int spiPort)
{
	volatile unsigned int value;

	if (spiPort == 0) {
		value = RD_SPI_REG(SPI0_BUF);
		value = RD_SPI_REG(SPI0_FLG);
	}
	else if (spiPort == 1) {
		value = RD_SPI_REG(SPI1_BUF);
		value = RD_SPI_REG(SPI1_FLG);	
	
	}
	else if (spiPort == 2) {
		value = RD_SPI_REG(SPI2_BUF);
		value = RD_SPI_REG(SPI2_FLG);
	}
}

#define TIMED_OUT 1
#define NOT_TIMED_OUT 0
static inline int wait_until_tx_empty(int spiPort)
{
	int timeout, spibuf;
	unsigned int value;

	if (spiPort == 0) {
		spibuf = SPI0_BUF;
	}
	else if (spiPort == 1) {
		spibuf = SPI1_BUF;
	}
	else if (spiPort == 2) {
		spibuf = SPI2_BUF;
	}
	
	timeout = FPGATIMEOUT;
	do
	{
		value = RD_SPI_REG(spibuf);
		if (!(value & SPIBUF_TXFULL))				// = 0: tx buffer empty
			break;
	} while (--timeout > 0);

	if (timeout == 0) {
		return(TIMED_OUT);
	}
	else {
		return(NOT_TIMED_OUT);
	}
}

static inline int wait_until_rx_full(int spiPort, unsigned int *value)
{
	int timeout, spibuf;
	
	if (spiPort == 0) {
		spibuf = SPI0_BUF;
	}
	else if (spiPort == 1) {
		spibuf = SPI1_BUF;
	}
	else if (spiPort == 2) {
		spibuf = SPI2_BUF;
	}

	timeout = FPGATIMEOUT;
	do
	{
		*value = RD_SPI_REG(spibuf);
		if (!(*value & SPIBUF_RXEMPTY))				// = 0: rx buffer full
			break;
	} while (--timeout > 0);

	if (timeout == 0) {
		return(TIMED_OUT);
	}
	else {
		return(NOT_TIMED_OUT);
	}
}

int spi_scbp_fpga_write(FPGAREG reg, FPGADATA data)
{
	unsigned int	value;
	
#if 0
	value = RD_SPI_REG(SPIGCR1);
	WR_SPI_REG(SPIGCR1, value | SPIGCR1_ENABLE);		// SPI enable
#endif
	/* clear out any rx conditions */
	clear_rx_conditions(SCBP_FPGA_SPI_PORT);
	
	/* Send to FPGA the Write address */
    if (wait_until_tx_empty(SCBP_FPGA_SPI_PORT) == TIMED_OUT)
		return -1;		
	SPIDAT1_WRITE_SCBP_FPGA(reg | 0x8000);
	
	/* Read dummy value */
	if (wait_until_rx_full(SCBP_FPGA_SPI_PORT, &value) == TIMED_OUT)
		return -2;		

	/* Program the FPGA register with specified data */
	if (wait_until_tx_empty(SCBP_FPGA_SPI_PORT) == TIMED_OUT)
		return -3;		
	SPIDAT1_WRITE_SCBP_FPGA(data);

	/* Read dummy value */
	if (wait_until_rx_full(SCBP_FPGA_SPI_PORT, &value) == TIMED_OUT)
		return -4;

#if 0
	value = RD_SPI_REG(SPI2_GCR1);
	WR_SPI_REG(SPI2_GCR1, value & ~SPIGCR1_ENABLE);		// SPI disable
#endif

	return 0;
}

int spi_scbp_fpga_read(FPGAREG reg, FPGADATA *data)
{
	unsigned int	value;
	
#if 0
	value = RD_SPI_REG(SPIGCR1);
	WR_SPI_REG(SPIGCR1, value | SPIGCR1_ENABLE);		// SPI enable
#endif
//	value = RD_SPI(SPIGCR1);

	/* clear out any rx conditions */
	clear_rx_conditions(SCBP_FPGA_SPI_PORT);

	/*
	 * 1st phase: send to FPGA the register to read
	 */
    if (wait_until_tx_empty(SCBP_FPGA_SPI_PORT) == TIMED_OUT)
		return -1;
	SPIDAT1_WRITE_SCBP_FPGA(reg);

	/* Read dummy value */
	if (wait_until_rx_full(SCBP_FPGA_SPI_PORT, &value) == TIMED_OUT)
		return -2;		

	/*
	 * 2nd phase: generate a new CS to read the value from FPGA
	 */
    if (wait_until_tx_empty(SCBP_FPGA_SPI_PORT) == TIMED_OUT)
		return -3;
	SPIDAT1_WRITE_SCBP_FPGA(0);

	/* Read the value of the FPGA register */
	if (wait_until_rx_full(SCBP_FPGA_SPI_PORT, &value) == TIMED_OUT)
		return -4;
	*data = (value & 0x0000FFFF);

#if 0
	value = RD_SPI_REG(SPI2_GCR1);
	WR_SPI_REG(SPI2_GCR1, value & ~SPIGCR1_ENABLE);		// SPI disable
#endif
	return 0;
}

int spi_scbp_fpga_resync(FPGAREG reg)
{
#if 0
	unsigned int    value;
#endif

printf(">>> FPGA resync <<<\n");

#if 0
	value = RD_SPI_REG(SPI2_GCR1);
	WR_SPI_REG(SPI2_GCR1, value | SPIGCR1_ENABLE);		// SPI enable
#endif
	/* clear out any rx conditions */
	clear_rx_conditions(SCBP_FPGA_SPI_PORT);

    /*
     * Write the address register of fpga
     */
    if (wait_until_tx_empty(SCBP_FPGA_SPI_PORT) == TIMED_OUT)
		return -1;
	SPIDAT1_WRITE_SCBP_FPGA(reg);

    /*
     * check if the register value has been transmitted
     */
    if (wait_until_tx_empty(SCBP_FPGA_SPI_PORT) == TIMED_OUT)
		return -2;

#if 0
	value = RD_SPI_REG(SPI2_GCR1);
	WR_SPI_REG(SPI2_GCR1, value & ~SPIGCR1_ENABLE);		// SPI disable
#endif

    return 0;
}

/* The DAC 8550 is a 16-bit DAC, the input here is the 16-bit input
   this routine will construct the 24 bits to be clocked to the DAC chip */
int spi_dac_write(int dacword)
{
	unsigned int value;

	clear_rx_conditions(DAC_SPI_PORT);

    if (wait_until_tx_empty(DAC_SPI_PORT) == TIMED_OUT)
		return -1;
    
	//MS 8-bits of 24-bit are for power control = all zero => MS 8-bits
	//of the 16-bit first SPI write are zero.
    //Note right shift below will be sign extended because dacword is signed
    // which is what is required.	
	SPIDAT1_WRITE_MS_16bits_DAC((dacword >> 8) & 0xFF);

	/* Read dummy value */
	if (wait_until_rx_full(DAC_SPI_PORT, &value) == TIMED_OUT)
		return -2;
		
    if (wait_until_tx_empty(DAC_SPI_PORT) == TIMED_OUT)
		return -1;
	SPIDAT1_WRITE_LS_8bits_DAC(dacword & 0xFF);

	/* Read dummy value */
	if (wait_until_rx_full(DAC_SPI_PORT, &value) == TIMED_OUT)
		return -2;
		
#if 1
	if (RD_SPI_REG(SPI1_BUF) & 0x10000000) {
		printf("error\n");
		while(1);
	}
#endif

	return 0;
}

#endif

