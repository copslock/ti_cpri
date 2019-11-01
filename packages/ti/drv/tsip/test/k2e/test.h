/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010-2012
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
 * @file test.h
 *
 * @brief 
 *  Holds all the constants and API definitions required by the test
 *  application to run.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010-2012, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par 
 */

#ifndef _TEST_H_
#define _TEST_H_

/* C Standard library include */
#include <string.h>

/* XDC include */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS include */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/knl/Event.h>

/* CSL include */
#include <ti/csl/csl_tsip.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/cslr_device.h>

/* TSIP LLD include */
#include <ti/drv/tsip/tsip.h>
#include <ti/drv/tsip/tsip_cfg.h>


#if defined(DEVICE_K2E) || defined(SOC_K2E)
#include <ti/csl/soc/k2e/src/csl_device_interrupt.h>
#endif


#define TSIP_HALBITMASK(x,y)      (   (   (  (1U << ((x)-(y)+1U) ) - 1U )   )   <<  (y)   )
#define TSIP_HAL_READ_BITFIELD(z,x,y)   (((z) & TSIP_HALBITMASK(x,y)) >> (y))
#define TSIP_HAL_SET_BITFIELD(z,f,x,y)  ((z) & ~TSIP_HALBITMASK(x,y)) | ( ((f) << (y)) & TSIP_HALBITMASK(x,y) )



/*  Application data buffer descriptor. Data can be placed into or
    removed from the buffer only in frameLength amounts. The element
    base repesents the base of data buffer (appToTsipBuffer and 
    tsipToAppBuffer). The element size represents the 
    true length of the data buffer. The element indexForTsip has
    direction dependent meanings 
    appToTsip - Application gives data to TSIP from this pointer
    tsipToApp - Application gives this pointer to TSIP to put data here
*/
typedef struct dataBuf_s {
  tsipData_t *base;      /* Base of buffer                 */
  UInt16  size;          /* Length of buffer in bytes      */
  UInt16  indexForTsip;  /* Location for TSIP to use       */
  UInt16  timeSlot;      /* time slot number associated to the buffer */
  UInt16  TsipPort;      /* tsip port number associated to the buffer.*/
} appBuf_t;

/* Size of application buffers */
#define BUFSIZE_APP 40*22 /*must be a multiple of the tx.frameSize (40). 
                            Due to L2 memory size, max value for this define is 880*/

/* Number of TSIP ports used in the test */
#define NUM_USED_TSIP_PORTS 1

/* This test uses the same number of time slots per TSIP port being tested*/
/* Number of used time slots per TSIP port (same for all ports)*/
#define NUM_USED_TIME_SLOTS 32 

#if ((NUM_USED_TIME_SLOTS * NUM_USED_TSIP_PORTS) > TSIP_MAX_TIMESLOTS )
#error Application is using more timeslots than allowed by the driver
#endif
 



#endif
