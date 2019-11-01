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
 * @file sample.h
 *
 * @brief 
 *  Holds all the constants and API definitions required by the example
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

#ifndef _SAMPLE_H_
#define _SAMPLE_H_

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

#if defined(DEVICE_K2E) || defined(SOC_K2E)
#include <ti/csl/soc/k2e/src/csl_device_interrupt.h>
#endif

/* TSIP LLD include */
#include <ti/drv/tsip/tsip.h>

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
} appBuf_t;

/* Size of application buffers */
#define BUFSIZE_APP 40*50 /*must be a multiple of the tx.frameSize (40)*/

/* Number of TSIP ports used in the test */
#define NUM_USED_TSIP_PORTS 1

#endif
