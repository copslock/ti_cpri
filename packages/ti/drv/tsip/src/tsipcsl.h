/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010
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

/* ================================================================= */
/*  file  tsipcsl.h
 *
 *  Data structures and definitions for TSIP-CSL interface
 *
 */
#ifndef _TSIPCSL_H
#define _TSIPCSL_H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include <stdint.h>
#include <stdlib.h>
#include "tsip.h"

/*********************************************************************************
 * Definition: Macro to create the timeslot/link value. The link value is
 *             placed in the lower three bits, the timeslot value in the
 *             upper 13.
 *********************************************************************************/
#define CSL_TSIP_CREATE_OFFSET_MAPVAL(x,y)   (((x)<<3) | (y))

/* ------------------------------------------------------------------------------------ */
/*    DMA Context writes relative to a base address                                     */
/* ------------------------------------------------------------------------------------ */
#define CSL_TSIP_CONTEXT_ADDR_BASE(x)         ((uint32_t)(x) + 0)  /* Base is at the base */
#define CSL_TSIP_CONTEXT_ADDR_ALLOC(x)        ((uint32_t)(x) + 4)  /* Offset from base to frame alloc */
#define CSL_TSIP_CONTEXT_ADDR_FRAMESIZE(x)    ((uint32_t)(x) + 8)  /* Offset from base to frame size */

/* ------------------------------------------------------------------------------------ */
/*    Set bit operation                                                                 */
/* ------------------------------------------------------------------------------------ */
#define TSIPBITMASK(x,y)      (   (   (  (1 << ((x)-(y)+1) ) - 1 )   )   <<  (y)   )
#define TSIP_SETBITFIELD(z,f,x,y)  ((z) & ~TSIPBITMASK(x,y)) | ( ((f) << (y)) & TSIPBITMASK(x,y) )

/*************************************************************************************
 * Definition: The tsip channel context. Used to store context info between
 *             csl invokations
 *************************************************************************************/
typedef struct cslTsipContext_s {

  int16_t   port;       /* The port number                  */
  uint16_t  cxnum;      /* Context number, 0 or 1           */
  uint16_t  cnid;       /* The channel ID value             */
  uint16_t  nLinks;     /* The number of links              */
  uint16_t  nTs;        /* The number of timeslots per link */
  
  void  *channelMaps;   /* Pointer to the channel maps    */
  uint32_t  context;    /* The dma memory context         */
  
} cslTsipContext_t;

/************************************************************************************
 * Definition: Structure used to enable a channel
 ************************************************************************************/
typedef struct cslTsipChEnable_s {

  int16_t port;    /* Phsyical port number */
  int16_t channel; /* Tsip channel   */
  uint16_t cnid;   /* Initial CnID   */
  
  uint32_t baseAddressGlobal;
  uint32_t frameAlloc;
  uint32_t frameSize;
  uint32_t frameCount;
  
} cslTsipChEnable_t;

/************************************************************************************
 * Prototypes                                                                      
 ************************************************************************************/
int16_t cslTsipConfigPort (tsipConfig_t *cfg);
int16_t cslTsipEnablePort (int16_t port);
int16_t cslTsipEnableTxChannel (cslTsipChEnable_t *en);
int16_t cslTsipEnableRxChannel (cslTsipChEnable_t *en);
int16_t cslTsipClearChannel (int16_t port, int16_t txChan, int16_t rxChan);

void cslTsipEnableTimeslot (cslTsipContext_t *cxt, uint16_t link, uint16_t ts, int16_t compand);
void cslTsipGetFreeTxContext (int16_t port, int16_t txChannel, cslTsipContext_t *cxt);
void cslTsipGetFreeRxContext (int16_t port, int16_t txChannel, cslTsipContext_t *cxt);
void cslTsipGetActiveRxContext (int16_t port, int16_t rxChannel, cslTsipContext_t *cxt);

uint32_t cslTsipNewTxContext (cslTsipContext_t *cxt);
uint32_t cslTsipNewRxContext (cslTsipContext_t *cxt);
uint32_t cslTsipReadTxFrfc (int16_t port);
uint32_t cslTsipReadRxFrfc (int16_t port);

uint32_t cslTsipGetAndSetFrameSize (cslTsipContext_t *cxt);
void cslTsipSetBaseAndAlloc (cslTsipContext_t *cxt, uint32_t base, uint32_t alloc);
uint16_t cslTsipGetDataOffset (cslTsipContext_t *cxt, uint16_t link, uint16_t ts);
int16_t cslTsipCreateOffsetMaps (cslTsipContext_t *cxt, uint16_t *tsmap, uint16_t *offsets, int16_t arraySize);

int16_t cslTsipGetGlobalStatus (int16_t port);
void cslTsipDisableTxRxDma (int16_t port, int16_t chnum);
void cslTsipEnableTxRxDma (int16_t port, int16_t chnum);

void cslTsipSendNewTxContext (int16_t port, uint32_t cnId, int16_t txChannel);
void cslTsipSendNewRxContext (int16_t port, uint32_t cnId, int16_t rxChannel);

void cslTsipStopRxDma (int16_t port, int16_t chan);
void cslTsipStartRxDma (int16_t port, int16_t chan);
void cslTsipStopTxDma (int16_t port, int16_t chan);
void cslTsipStartTxDma (int16_t port, int16_t chan);

#ifdef __cplusplus
}
#endif

#endif  /* _TSIPCSL_H */
