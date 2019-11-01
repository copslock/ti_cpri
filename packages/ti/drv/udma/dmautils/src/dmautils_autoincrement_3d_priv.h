/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 */

/**
 *  \file dmautils_autoincrement_3d_priv.h
 *
 *  \brief This header is an internal header for decleration autoincrement 3D internal
*             context
 */

#ifndef DMAUTILS_AUTOINCREMENT_3D_PRIV_H_
#define DMAUTILS_AUTOINCREMENT_3D_PRIV_H_

#include "stdint.h"

#include "ti/drv/udma/dmautils/include/dmautils_autoincrement_3d.h"
#include "ti/drv/udma/udma.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
  uint64_t                                 ringMem;
  uint64_t                                 reserved[15];
  uint64_t                                 responseRingMem;
  uint64_t                                 tdRingMem;
  volatile uint64_t                    *swTriggerPointer;
  uint64_t                                 waitWord;
  struct Udma_ChObj               chHandle;
  struct Udma_EventObj           eventHandle;
} DmaUtilsAutoInc3d_ChannelContext;


typedef struct
{
  uint16_t blkIdx[UDMA_RM_MAX_BLK_COPY_CH];
  DmaUtilsAutoInc3d_ChannelContext *channelContext[UDMA_RM_MAX_BLK_COPY_CH];
  DmaUtilsAutoInc3d_InitParam initParams;
} DmaUtilsAutoInc3d_Context;


#ifdef __cplusplus
}
#endif

#endif /*DMAUTILS_AUTOINCREMENT_3D_PRIV_H_*/
/*!
*! Revision History
*! ================
*! Jan-2018   Anshu: Initial Draft
*/

