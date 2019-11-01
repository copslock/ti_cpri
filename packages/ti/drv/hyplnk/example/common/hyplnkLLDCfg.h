/*
 *
 * Copyright (C) 2010-2016 Texas Instruments Incorporated - http://www.ti.com/ 
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

/*  
 * This file contains the definitions that are used to configure the
 * Hyperlink peripheral, such as selecting the serial speed and 
 * loopbacks.
 */
/*****************************************************************************
 * Select what if any register contents to print
 *****************************************************************************/
/* Allowed values for hyplnk_EXAMPLE_PRINT_*_REG */
#define hyplnk_EXAMPLE_PRINT_NONE    0
#define hyplnk_EXAMPLE_PRINT_RAW     1
#define hyplnk_EXAMPLE_PRINT_DETAILS 2
#define hyplnk_EXAMPLE_PORT          0

/* Change definitions as desired */
#define hyplnk_EXAMPLE_PRINT_REV_REG         hyplnk_EXAMPLE_PRINT_RAW
#define hyplnk_EXAMPLE_PRINT_STATUS_REG      hyplnk_EXAMPLE_PRINT_RAW
#define hyplnk_EXAMPLE_PRINT_LINK_STATUS_REG hyplnk_EXAMPLE_PRINT_RAW
#define hyplnk_EXAMPLE_PRINT_CONTROL_REG     hyplnk_EXAMPLE_PRINT_RAW
#define hyplnk_EXAMPLE_PRINT_ECC_ERRORS_REG  hyplnk_EXAMPLE_PRINT_RAW
#define hyplnk_EXAMPLE_PRINT_SERDES_STS_REGS hyplnk_EXAMPLE_PRINT_RAW

/*****************************************************************************
 * Match the reference clock on your board
 *
 * The value hyplnk_EXAMPLE_REFCLK_USE_PLATCFG uses the reference clock 
 * defined through hyplnk_EXAMPLE_HYPLNK_PLATCFG_REF_CLK_MHZ 
 * in hyplnkPlatCfg.h.
 * 
 * hyplnk_EXAMPLE_REFCLK_USE_PLATCFG can be commented out and the specific 
 * value specified below.
 *****************************************************************************/
#define hyplnk_EXAMPLE_REFCLK_USE_PLATCFG
//#define hyplnk_EXAMPLE_REFCLK_156p25
//#define hyplnk_EXAMPLE_REFCLK_250p00
//#define hyplnk_EXAMPLE_REFCLK_312p50

/*****************************************************************************
 * Select internal loopback or use the SERDES connection
 *****************************************************************************/
#define hyplnk_EXAMPLE_LOOPBACK

/*****************************************************************************
 * Select number of lanes allowed
 *****************************************************************************/
//#define hyplnk_EXAMPLE_ALLOW_0_LANES
//#define hyplnk_EXAMPLE_ALLOW_1_LANE
#define hyplnk_EXAMPLE_ALLOW_4_LANES

/*****************************************************************************
 * Select a serial rate
 *****************************************************************************/
//#define hyplnk_EXAMPLE_SERRATE_01p250
#define hyplnk_EXAMPLE_SERRATE_03p125
//#define hyplnk_EXAMPLE_SERRATE_06p250
//#define hyplnk_EXAMPLE_SERRATE_07p500
//#define hyplnk_EXAMPLE_SERRATE_10p000
//#define hyplnk_EXAMPLE_SERRATE_12p500

/*****************************************************************************
 * Set if each endpoint has its own hyperlink clock
 *****************************************************************************/
#define hyplnk_EXAMPLE_ASYNC_CLOCKS

/*****************************************************************************
 * Set to perform equalization analysis
 *****************************************************************************/
// EQ analysis not ready for ARMv7
#ifndef __ARMv7
#define hyplnk_EXAMPLE_EQ_ANALYSIS
#endif

/*****************************************************************************
 * Select one or more test cases
 *
 * The synchronized token exchange requires the same SW on each endpoint
 *
 * The asynchronous block transfers do not require any SW running on the other
 * endpoint, however both sides can optionally run the test at the same time.
 *****************************************************************************/

/* Synchronized token passing between two endpoints using CPU IO */
#define hyplnk_EXAMPLE_TEST_CPU_TOKEN_EXCHANGE

#ifndef __LINUX_USER_SPACE
/* for enabling and disabling edma examples. comment this define for disabling edma examples */
#define enableEDMA
/* for enabling and disabling infraDma examples. comment this define for disabling infradma examples */
#define infraDMA
#endif

#ifdef enableEDMA
/* Synchronized token passing between two endpoints using EDMA IO */
#define hyplnk_EXAMPLE_TEST_DMA_TOKEN_EXCHANGE
#endif

/* Asynchronous block transfers using CPU (memset/memcmp) */
#define hyplnk_EXAMPLE_TEST_CPU_BLOCK_XFER

#ifdef enableEDMA
/* Asynchronous block transfers using EDMA */
#define hyplnk_EXAMPLE_TEST_DMA_BLOCK_XFER
#endif

#ifdef infraDMA  
/* Asynchronous block transfers using QMSS */
#define hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER
#endif

/*****************************************************************************
 * Cache line size (128 works on any location)
 *****************************************************************************/
#define hyplnk_EXAMPLE_LINE_SIZE      128

/*****************************************************************************
 * Size of block transfer
 * If using InfraDMA:
 *          - size must be a multiple of 128 for cache alignment
 *
 *****************************************************************************/
#define hyplnk_EXAMPLE_BLOCK_BUF_SIZE 4096

 /*****************************************************************************
 * Number of iterations (* NTOKENS); 0 is forever
 *****************************************************************************/
#define hyplnk_EXAMPLE_NITERS     3
/*****************************************************************************
 * Set to enable an error interrupt on uncorrectable serial errors
 *****************************************************************************/
// Interrupts not ready for ARMv7
#ifndef __ARMv7
 #define hyplnk_EXAMPLE_ERROR_INTERRUPT
#endif

#ifdef enableEDMA
/*****************************************************************************
 * Choose the type of EDMA transfer (Current options are "DMA" and "QDMA")
 *****************************************************************************/
#define EDMA_TYPE 0 //DMA
//#define EDMA_TYPE 1 //QDMA
#endif

#ifdef infraDMA  
/*****************************************************************************
 * Number of monolithic descriptors in the memory region
 *     -Must be at least 32
 *     -Must be no more than 2^20
 *****************************************************************************/
#define NUM_MONOLITHIC_DESC         32

/*****************************************************************************
 * Monolithic descriptor data offset
 *     -Offset used to insert information about the packets
 *****************************************************************************/
#define MONOLITHIC_DESC_DATA_OFFSET 128

/*****************************************************************************
 * Number of packets
 *****************************************************************************/
#define NUM_PACKETS                 16

/*****************************************************************************
 * Size of data buffer
 *****************************************************************************/
#define SIZE_DATA_BUFFER            hyplnk_EXAMPLE_BLOCK_BUF_SIZE

/*****************************************************************************
 * Number of bytes per entry in data buffer
 *     -Data buffer will be separated into packets
 *****************************************************************************/
#define DATA_SIZE                   4    //Number of bytes per entry in packet

/*****************************************************************************
 * Size of packet within monolithic descriptor
 * 	   -For this example the hyplnk_EXAMPLE_BLOCK_BUF_SIZE is split up into packets
 * 	   -Each packet entry can only accept 1 byte
 * 	   -Since hyplnk_EXAMPLE_BLOCK_BUF_SIZE has 4 byte per entry then
 * 	    each packet must has 4 times as many entries to accommodate
 *****************************************************************************/
#define SIZE_DATA_BUFFER_PACKET     (SIZE_DATA_BUFFER/NUM_PACKETS)*DATA_SIZE

/*****************************************************************************
 * Size of monolithic descriptor
 * 	   -Should be size of data packets plus the size of the packet information
 * 	   -Size is in bytes
 * 	   -Must be a multiple of 16
 *****************************************************************************/
#define SIZE_MONOLITHIC_DESC        ((SIZE_DATA_BUFFER/DATA_SIZE) + MONOLITHIC_DESC_DATA_OFFSET)

/*****************************************************************************
 * Check if definitions are setup correctly
 *****************************************************************************/
#if (((SIZE_DATA_BUFFER/DATA_SIZE) + MONOLITHIC_DESC_DATA_OFFSET) > SIZE_MONOLITHIC_DESC )
#error Monolithic descriptor size is too small
#endif

#if (SIZE_MONOLITHIC_DESC%16 != 0 )
#error Monolithic descriptor size must be divisable by 16
#endif
#endif

/*
 * Set to enable Cache invalidates and writebacks
 *****************************************************************************/
// Cache operations not ready for ARMv7
#ifndef __ARMv7
#define hyplnk_CACHE_ENABLE
#endif

/* Nothing past this point */

