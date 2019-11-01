
#if EVM_TYPE == 5 || EVM_TYPE == 7
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

#ifndef SCBPFPGA_SPI_H_
#define SCBPFPGA_SPI_H_

#if EVM_TYPE == 5
#define	SPI_BASE_ADDRESS		0x20BF0000

#define SPIGCR0					(SPI_BASE_ADDRESS + 0x00)
#define SPIGCR1					(SPI_BASE_ADDRESS + 0x04)
#define SPIINT0					(SPI_BASE_ADDRESS + 0x08)
#define SPIILVL					(SPI_BASE_ADDRESS + 0x0C)
#define SPIFLG					(SPI_BASE_ADDRESS + 0x10)
#define SPIPC0					(SPI_BASE_ADDRESS + 0x14)
#define SPIDAT0					(SPI_BASE_ADDRESS + 0x38)
#define SPIDAT1					(SPI_BASE_ADDRESS + 0x3C)
#define SPIBUF					(SPI_BASE_ADDRESS + 0x40)
#define SPIEMU					(SPI_BASE_ADDRESS + 0x44)
#define SPIDELAY				(SPI_BASE_ADDRESS + 0x48)
#define SPIDEF					(SPI_BASE_ADDRESS + 0x4C)
#define SPIFMT0					(SPI_BASE_ADDRESS + 0x50)
#define SPIFMT1					(SPI_BASE_ADDRESS + 0x54)
#define SPIFMT2					(SPI_BASE_ADDRESS + 0x58)
#define SPIFMT3					(SPI_BASE_ADDRESS + 0x5C)
#define SPIVEC0					(SPI_BASE_ADDRESS + 0x60)
#define SPIVEC1					(SPI_BASE_ADDRESS + 0x64)
#else
#define	SPI0__BASE_ADDRESS		0x21000400
#define	SPI1__BASE_ADDRESS		0x21000600
#define	SPI2__BASE_ADDRESS		0x21000800

#define SPI0_GCR0					(SPI0__BASE_ADDRESS + 0x00)
#define SPI0_GCR1					(SPI0__BASE_ADDRESS + 0x04)
#define SPI0_INT0					(SPI0__BASE_ADDRESS + 0x08)
#define SPI0_ILVL					(SPI0__BASE_ADDRESS + 0x0C)
#define SPI0_FLG					(SPI0__BASE_ADDRESS + 0x10)
#define SPI0_PC0					(SPI0__BASE_ADDRESS + 0x14)
#define SPI0_DAT0					(SPI0__BASE_ADDRESS + 0x38)
#define SPI0_DAT1					(SPI0__BASE_ADDRESS + 0x3C)
#define SPI0_BUF					(SPI0__BASE_ADDRESS + 0x40)
#define SPI0_EMU					(SPI0__BASE_ADDRESS + 0x44)
#define SPI0_DELAY					(SPI0__BASE_ADDRESS + 0x48)
#define SPI0_DEF					(SPI0__BASE_ADDRESS + 0x4C)
#define SPI0_FMT0					(SPI0__BASE_ADDRESS + 0x50)
#define SPI0_FMT1					(SPI0__BASE_ADDRESS + 0x54)
#define SPI0_FMT2					(SPI0__BASE_ADDRESS + 0x58)
#define SPI0_FMT3					(SPI0__BASE_ADDRESS + 0x5C)
#define SPI0_VEC0					(SPI0__BASE_ADDRESS + 0x60)
#define SPI0_VEC1					(SPI0__BASE_ADDRESS + 0x64)

#define SPI1_GCR0					(SPI1__BASE_ADDRESS + 0x00)
#define SPI1_GCR1					(SPI1__BASE_ADDRESS + 0x04)
#define SPI1_INT0					(SPI1__BASE_ADDRESS + 0x08)
#define SPI1_ILVL					(SPI1__BASE_ADDRESS + 0x0C)
#define SPI1_FLG					(SPI1__BASE_ADDRESS + 0x10)
#define SPI1_PC0					(SPI1__BASE_ADDRESS + 0x14)
#define SPI1_DAT0					(SPI1__BASE_ADDRESS + 0x38)
#define SPI1_DAT1					(SPI1__BASE_ADDRESS + 0x3C)
#define SPI1_BUF					(SPI1__BASE_ADDRESS + 0x40)
#define SPI1_EMU					(SPI1__BASE_ADDRESS + 0x44)
#define SPI1_DELAY					(SPI1__BASE_ADDRESS + 0x48)
#define SPI1_DEF					(SPI1__BASE_ADDRESS + 0x4C)
#define SPI1_FMT0					(SPI1__BASE_ADDRESS + 0x50)
#define SPI1_FMT1					(SPI1__BASE_ADDRESS + 0x54)
#define SPI1_FMT2					(SPI1__BASE_ADDRESS + 0x58)
#define SPI1_FMT3					(SPI1__BASE_ADDRESS + 0x5C)
#define SPI1_VEC0					(SPI1__BASE_ADDRESS + 0x60)
#define SPI1_VEC1					(SPI1__BASE_ADDRESS + 0x64)

#define SPI2_GCR0					(SPI2__BASE_ADDRESS + 0x00)
#define SPI2_GCR1					(SPI2__BASE_ADDRESS + 0x04)
#define SPI2_INT0					(SPI2__BASE_ADDRESS + 0x08)
#define SPI2_ILVL					(SPI2__BASE_ADDRESS + 0x0C)
#define SPI2_FLG					(SPI2__BASE_ADDRESS + 0x10)
#define SPI2_PC0					(SPI2__BASE_ADDRESS + 0x14)
#define SPI2_DAT0					(SPI2__BASE_ADDRESS + 0x38)
#define SPI2_DAT1					(SPI2__BASE_ADDRESS + 0x3C)
#define SPI2_BUF					(SPI2__BASE_ADDRESS + 0x40)
#define SPI2_EMU					(SPI2__BASE_ADDRESS + 0x44)
#define SPI2_DELAY					(SPI2__BASE_ADDRESS + 0x48)
#define SPI2_DEF					(SPI2__BASE_ADDRESS + 0x4C)
#define SPI2_FMT0					(SPI2__BASE_ADDRESS + 0x50)
#define SPI2_FMT1					(SPI2__BASE_ADDRESS + 0x54)
#define SPI2_FMT2					(SPI2__BASE_ADDRESS + 0x58)
#define SPI2_FMT3					(SPI2__BASE_ADDRESS + 0x5C)
#define SPI2_VEC0					(SPI2__BASE_ADDRESS + 0x60)
#define SPI2_VEC1					(SPI2__BASE_ADDRESS + 0x64)
#endif
#define	SPIGCR1_ENABLE			(1 << 24)
#define	SPIBUF_RXEMPTY			0x80000000 //(1 << 31)
#define	SPIBUF_TXFULL			(1 << 29)

#define WR_SPI_REG(addr, data)   *(volatile unsigned int*)(addr) =(unsigned int)(data)
#define RD_SPI_REG(addr)         *(volatile unsigned int*)(addr)

typedef unsigned int			FPGAREG;	
typedef unsigned int			FPGADATA;	

#define	FPGATIMEOUT				1000000

extern void 					spi_init(void);
extern int 						spi_scbp_fpga_write(FPGAREG reg, FPGADATA data);
extern int 						spi_scbp_fpga_read(FPGAREG reg, FPGADATA *data);
extern int 						spi_scbp_fpga_resync(FPGAREG reg);
extern int  					spi_scbp_fpga_lock(void);
extern int  					spi_scbp_fpga_unlock(void);
#if EVM_TYPE == 5
extern int                      spi_6614evm_fpga_read(int reg, int *data);
extern int                      spi_6614evm_fpga_write(int reg, int data);
extern int                      spi_6614evm_dac_write_transaction(int data);
#else
extern int                      spi_dac_write(int);
#endif

#endif /*SCBPFPGA_SPI_H_*/

#endif
