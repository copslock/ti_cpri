/**  
 * @file local_variables.c
 *
 * @brief 
 *  This file holds all the variables local to that core/process required
 *  for the multicore example.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2013, Texas Instruments, Inc.
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
#include <multicore_example.h>

/* QMSS queue handles */

/* Queue with free descriptors */
Qmss_QueueHnd                           gGlobalFreeQHnd;

/* TX queues used to send data to PA PDSP/CPSW.*/
#pragma DATA_ALIGN   (gPaTxQHnd, 128)
Qmss_QueueHnd                           gPaTxQHnd [NUM_PA_TX_QUEUES];

#pragma DATA_ALIGN   (gPaL4Handles, CACHE_LINESZ)
paHandleL4_t        gPaL4Handles;

/* RX queue used by the application to receive packets from PASS/CPSW.
   Each core has an independent RX queue. */
Qmss_QueueHnd                           gRxQHnd;

Cppi_ChHnd                              gCpdmaTxChanHnd [NUM_PA_TX_CHANNELS];

Cppi_ChHnd                              gCpdmaRxChanHnd [NUM_PA_RX_CHANNELS];

Cppi_FlowHnd                            gRxFlowHnd;

/* PA command response queue handle */
Qmss_QueueHnd                           gPaCfgCmdRespQHnd;

/* TX queue with free decriptors attached to data buffers for transmission.*/
Qmss_QueueHnd                           gTxFreeQHnd;

/* RX queue with free decriptors attached to data buffers to be used
   by the PASS CPDMA to hold the received data.*/
Qmss_QueueHnd                           gRxFreeQHnd;

/* CPPI Handles used by the application */
Cppi_Handle                             gCpdmaHnd;

/* PA Driver Handle */
Pa_Handle                               gPAInstHnd;
/* PA L2 Handle */
paHandleL2L3_t                          gPaL2Handles;

/* PA L3 Handle */
paHandleL2L3_t                          gPaL3Handles;

/* PA L4 Handle */
paHandleL4_t                            gPaL4Handles;

/* Pointer to the shared memory to get to all the handles needed for
 * multicore/multiprocess */
pa_Example_MC_ShObj_t*                  shObj;


/* Constructed data packet to send.
   Each core will have a slightly modified version
   of this packet which is stored in the core's local memory. */
#pragma DATA_ALIGN(pktMatchBuf, 16)
uint8_t pktMatchBuf[PACKET_SIZE] = {
							0x10, 0x11, 0x12, 0x13, 0x14, 0x15,                      /* Dest MAC */
                            0x00, 0x01, 0x02, 0x03, 0x04, 0x05,                      /* Src MAC  */
                            0x08, 0x00,                                              /* Ethertype = IPv4 */
                            0x45, 0x00, 0x00, 0x6c,                                  /* IP version, services, total length */
                            0x00, 0x00, 0x00, 0x00,                                  /* IP ID, flags, fragment offset */
                            0x05, 0x11, 0x32, 0x26,                                  /* IP ttl, protocol (UDP), header checksum */
                            0xc0, 0xa8, 0x01, 0x01,                                  /* Source IP address */
                            0xc0, 0xa8, 0x01, 0x0a,                                  /* Destination IP address */
                            0x12, 0x34, 0x56, 0x78,                                  /* UDP source port, dest port */
                            0x00, 0x58, 0x1d, 0x18,                                  /* UDP len, UDP checksum */
                            0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,          /* 80 bytes of payload data */
                            0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41,
                            0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
                            0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
                            0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
                            0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61,
                            0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
                            0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71,
                            0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
                            0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81  };

uint8_t*   pktMatch;							

/* Tx/Rx packet counters */
volatile uint32_t						gTxCounter = 0, gRxCounter = 0;

/* RM Client Vars */
Rm_Handle           rmClientHandle = NULL;
Rm_ServiceHandle   *rmClientServiceHandle = NULL;

/*
 * Default test configuration for the silicon
 *
 * To run test at the CCS simulator
 *    cpswSimTest = 1
 *    cpswLpbkMode = CPSW_LOOPBACK_INTERNAL
 */
#ifdef  SIMULATOR_SUPPORT
Int cpswSimTest = 1;
Int cpswLpbkMode = CPSW_LOOPBACK_INTERNAL;
#else
Int cpswSimTest = 0;
#ifndef __LINUX_USER_SPACE
Int cpswLpbkMode = CPSW_LOOPBACK_INTERNAL;
#endif
#endif

/* Default test configuration for the silicon
 * 
 * To run the test at the CCS (with no boot mode, using GEL files) - Default
 *  no_bootMode = TRUE
 * To run the test at the CCS (with other boot modes when linux is up)
 *  no_bootMode = FALSE 
 */
#ifdef __LINUX_USER_SPACE
int no_bootMode = FALSE;
#else
int no_bootMode = TRUE;
#endif

void mdebugHaltPdsp (Int pdspNum);
volatile Int mdebugWait = 1;

/* Error counter */
uint32_t                errorCount = 0;

/* Indicates the core or logical task ID test is running on */
uint32_t                coreNum;

/* Indicate the test status */
char	test_stat[50];

#ifndef __LINUX_USER_SPACE
/* Handle to CPPI heap */
IHeap_Handle            cppiHeap;
#endif


