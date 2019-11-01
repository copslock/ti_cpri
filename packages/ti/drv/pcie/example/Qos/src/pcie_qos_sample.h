/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010-2019
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


/**  
 * @file pcie_sample.h
 *
 * @brief 
 *  Holds all the constants and API definitions required by the example
 *  application to run.  
 */

#ifndef _PCIE_QOS_SAMPLE_H_
#define _PCIE_QOS_SAMPLE_H_

/* C Standard library include */
#include <string.h>

/* XDC include */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS include */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* CSL include */
#include <ti/csl/cslr_device.h>

/* PCIE LLD include */
#include <ti/drv/pcie/pcie.h>

#if defined (__aarch64__)
/* Barrier function */
#define BARRIER     asm volatile("    DMB SY");
#endif

#if defined (__TI_ARM_V7R4__)
/* Barrier function */
#define BARRIER     __asm volatile("   DMB   ");
#endif

/* Use UDMA to generate background traffic */
#define UDMA
#define PCIE_EXAMPLE_LINE_SIZE     (1048576*8)          /* 8MB */

/* AM6 is GEN3 */
#define GEN3   

/* Set up printf */
#include <ti/drv/uart/UART_stdio.h>
#define PCIE_logPrintf  UART_printf
#define PCIE_logScanf   UART_scanFmt

/* Size of application buffers */
#define PCIE_BUFSIZE_APP 40 

/* In this example all addresses are 32bit */
/* Outbound Base Address for PCIe RC */
#define PCIE_OB_LO_ADDR_RC_VC0   0x70000000
#define PCIE_OB_HI_ADDR_RC_VC0   0

#define PCIE_OB_LO_ADDR_RC_VC1   0x71000000
#define PCIE_OB_HI_ADDR_RC_VC1   0

#define PCIE_OB_LO_ADDR_RC_VC2   0x72000000
#define PCIE_OB_HI_ADDR_RC_VC2   0

#define PCIE_OB_LO_ADDR_RC_VC3   0x73000000
#define PCIE_OB_HI_ADDR_RC_VC3   0

/* Inbound  Base Address for PCIe RC */
#define PCIE_IB_LO_ADDR_RC_VC0   0x90000000
#define PCIE_IB_HI_ADDR_RC_VC0   0

#define PCIE_IB_LO_ADDR_RC_VC1   0x91000000
#define PCIE_IB_HI_ADDR_RC_VC1   0

#define PCIE_IB_LO_ADDR_RC_VC2   0x92000000
#define PCIE_IB_HI_ADDR_RC_VC2   0

#define PCIE_IB_LO_ADDR_RC_VC3   0x93000000
#define PCIE_IB_HI_ADDR_RC_VC3   0

/* Outbound Base Address for PCIe EP */
#define PCIE_OB_LO_ADDR_EP_VC0   PCIE_IB_LO_ADDR_RC_VC0
#define PCIE_OB_HI_ADDR_EP_VC0   0

#define PCIE_OB_LO_ADDR_EP_VC1   PCIE_IB_LO_ADDR_RC_VC1
#define PCIE_OB_HI_ADDR_EP_VC1   0

#define PCIE_OB_LO_ADDR_EP_VC2   PCIE_IB_LO_ADDR_RC_VC2
#define PCIE_OB_HI_ADDR_EP_VC2   0

#define PCIE_OB_LO_ADDR_EP_VC3   PCIE_IB_LO_ADDR_RC_VC3
#define PCIE_OB_HI_ADDR_EP_VC3   0

/* Inbound  Base Address for PCIe EP */
#define PCIE_IB_LO_ADDR_EP_VC0   PCIE_OB_LO_ADDR_RC_VC0
#define PCIE_IB_HI_ADDR_EP_VC0   0

#define PCIE_IB_LO_ADDR_EP_VC1   PCIE_OB_LO_ADDR_RC_VC1
#define PCIE_IB_HI_ADDR_EP_VC1   0

#define PCIE_IB_LO_ADDR_EP_VC2   PCIE_OB_LO_ADDR_RC_VC2
#define PCIE_IB_HI_ADDR_EP_VC2   0

#define PCIE_IB_LO_ADDR_EP_VC3   PCIE_OB_LO_ADDR_RC_VC3
#define PCIE_IB_HI_ADDR_EP_VC3   0

#ifdef am65xx_idk
#define PCIE_WINDOW_START    0x10000000U
#define PCIE_REG_BASE        0x05500000U
#endif
  
#ifdef am65xx_evm
#define PCIE_WINDOW_START    0x18000000U
#define PCIE_REG_BASE        0x05600000U
#endif

/* Outbound PCIE data address for VCs, applies to RC and EP */
#define PCIE_WINDOW_MEM_BASE_VC0 PCIE_WINDOW_START
#define PCIE_WINDOW_MEM_MASK_VC0 0x00FFFFFFU

#define PCIE_WINDOW_MEM_BASE_VC1 (PCIE_WINDOW_START + 0x01000000U)
#define PCIE_WINDOW_MEM_MASK_VC1 0x00FFFFFFU

#define PCIE_WINDOW_MEM_BASE_VC2 (PCIE_WINDOW_START + 0x02000000U)
#define PCIE_WINDOW_MEM_MASK_VC2 0x00FFFFFFU

#define PCIE_WINDOW_MEM_BASE_VC3 (PCIE_WINDOW_START + 0x03000000U)
#define PCIE_WINDOW_MEM_MASK_VC3 0x00FFFFFFU

/* Outbound PCIE data address for CFG access, applies to RC */
#define PCIE_WINDOW_CFG_BASE (PCIE_WINDOW_START + 0x04000000U)
#define PCIE_WINDOW_CFG_MASK 0x0000FFFFU

/* MSI address in PCIE data window */
#define PCIE_WINDOW_MSI_ADDR (PCIE_WINDOW_START + 0x04010000U)
#define PCIE_WINDOW_MSI_MASK 0x0000FFFFU

/* PCIE address space for MSI */
#define PCIE_PCIE_MSI_BASE   (0x00010000U)
#define PCIE_PCIE_MSI_OFF    (0x00000040U)

/* SPI number (a block of reserved ARM GIC SPIs) to use for MSI) */
#define PCIE_SPI_BASE        (0x00000300U)
#define PCIE_WINDOW_MSI_DATA (PCIE_SPI_BASE)

#endif


